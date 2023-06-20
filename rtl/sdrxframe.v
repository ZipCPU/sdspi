////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdrxframe.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Process incoming data from the front end.  Data will come in
//		on (potentially) every edge of DS.  We need to turn it here
//	into one data stream for each IO pin, check the CRC on each data
//	stream, and then compose the results into (up to) 32-bits per clock
//	cycle to be written to the SRAM on an outgoing stream.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2018-2023, Gisselquist Technology, LLC
// {{{
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of  the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
`default_nettype	none
// }}}
module	sdrxframe #(
		// {{{
		parameter	LGLEN = 15, NUMIO=4,
		parameter	MW = 32,
		localparam	LGLENW = LGLEN-$clog2(MW/8),
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		parameter	LGTIMEOUT = 22
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		//
		input	wire			i_cfg_ds,
		input	wire	[1:0]		i_cfg_width,
		//
		input	wire			i_rx_en,
		input	wire			i_crc_en,
		input	wire	[LGLEN:0]	i_length,	// In bytes
		//
		input	wire	[1:0]		i_rx_strb,
		input	wire	[15:0]		i_rx_data,

		input	wire			S_ASYNC_VALID,
		input	wire	[31:0]		S_ASYNC_DATA,

		output	wire			o_mem_valid,
		output	wire	[MW/8-1:0]	o_mem_strb,
		output	wire	[LGLENW-1:0]	o_mem_addr,	// Word address
		output	wire	[MW-1:0]	o_mem_data,	// Outgoing data

		output	reg			o_done,
		output	reg			o_err
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[1:0]	WIDTH_1W = 2'b00,
				WIDTH_4W = 2'b01;
				// WIDTH_8W = 2'b10;

	localparam	NCRC = 16;
	localparam	[NCRC-1:0]	CRC_POLYNOMIAL = 16'h1021;
	genvar	gk;

	reg		i_sync;
	reg		syncd;
	reg	[4:0]	sync_fill;
	reg	[19:0]	sync_sreg;

	reg		s2_valid;
	reg	[1:0]	s2_fill;
	reg	[15:0]	s2_data;

	reg				mem_valid, mem_full;
	reg	[MW/8-1:0]		mem_strb;
	reg	[MW-1:0]		mem_data;
	reg	[LGLENW-1:0]		mem_addr;
	reg	[$clog2(MW/8)-1:0]	subaddr;

	reg			busy, data_phase, load_crc;
	reg	[LGLEN+4-1:0]	rail_count;

	reg	[15:0]		crc	[0:NUMIO-1];
	reg	[15:0]		check	[0:NUMIO-1];
	reg	[NUMIO-1:0]	good, err;

	reg	[LGTIMEOUT-1:0]	r_timeout;
	reg			r_watchdog;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Notes:
	// {{{
	// Two channels of processing:
	//	 Sync processing: (No data strobe)
	//	0. The PHY will sample data a fixed distance from each clk edge
	//		That means it will always be sampling, whether or not
	//		we're expecting something.
	//	1. Wait for the enable
	//	2. Wait for the start bit: (S_VALID && S_DATA != 8'hff)
	//	3. Capture every S_VALID, and realign incoming bits
	//	4. Move to stage #2
	//
	//	Async processing: (w/ data strobe)
	//	0. The PHY will detect DS edges for us, and provide data
	//		VALID, (READY *MUST* be true), DATA, STRB
	//	1. Pack data into M_VALID, 32bits of M_DATA, M_LAST
	//	2. Move to stage #2
	//
	// Stage #2
	//	1. Calculate CRC on each of the 8 channels
	//	2. Generate error on failed CRC, failed start, or failed stop
	//	3. Drop unused data paths
	//	4. 
	//
	// Potential errors:
	//	Start bits not aligned
	//	Stop bits not aligned, or not at the right place
	//	Failed CRC on a data bit we are using
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Synchronous (no DS) data path
	// {{{

	// Step #1: Bit sync
	// {{{
	always @(*)
	begin
		case(i_cfg_width)
		WIDTH_1W:
			i_sync = (i_rx_strb[1] && i_rx_data[8] == 1'b0)
				|| (i_rx_strb[0] && i_rx_data[0] == 1'b0);
		WIDTH_4W: i_sync = ((i_rx_strb[1] && i_rx_data[11:8] == 4'h0)
				||(i_rx_strb[0] && i_rx_data[ 3:0] == 4'h0));
		default: i_sync = ((i_rx_strb[1] && i_rx_data[15:8] == 8'h0)
				||(i_rx_strb[0] && i_rx_data[ 7:0] == 8'h0));
		endcase
	end

	always @(posedge i_clk)
	if (!i_rx_en)
		syncd <= 0;
	else if (i_sync)
		syncd <= 1;

	always @(posedge i_clk)
	if (!i_rx_en)
		sync_fill <= 0;
	else if (i_rx_strb == 0)
	begin
	end else if (syncd)
	begin
		// Verilator lint_off WIDTH
		case(i_cfg_width)
		WIDTH_1W: sync_fill <= sync_fill[2:0] + (i_rx_strb[1] ? 1:0)
					+ (i_rx_strb[0] ? 1:0);
		WIDTH_4W: sync_fill <= sync_fill[2:0] + (i_rx_strb[1] ? 4:0)
					+ (i_rx_strb[0] ? 4:0);
		default:  sync_fill <= sync_fill[2:0] + (i_rx_strb[1] ? 8:0)
					+ (i_rx_strb[0] ? 8:0);
		endcase
	end else case(i_cfg_width)
	WIDTH_1W: sync_fill <= (i_rx_strb[1] && i_rx_data[8] == 1'b0)
				? (i_rx_strb[0] ? 1:0) : 0;
	WIDTH_4W: sync_fill <= (i_rx_strb[1] && i_rx_data[11:8] == 4'h0) ? 4:0;
	default: sync_fill  <= (i_rx_strb[1] && i_rx_data[15:8] == 8'h0) ? 8:0;
	endcase
	// Verilator lint_on  WIDTH

	always @(posedge i_clk)
	if (!i_rx_en)
		sync_sreg <= 0;
	else if (i_rx_strb != 0)
	begin
		case(i_cfg_width)
		WIDTH_1W: if (i_rx_strb == 2'b11)
			sync_sreg <= { sync_sreg[17:0], i_rx_data[8], i_rx_data[0] };	
			else if (i_rx_strb[1])
				sync_sreg <= { sync_sreg[18:0], i_rx_data[8] };	
			else
				sync_sreg <= { sync_sreg[18:0], i_rx_data[0] };	
		WIDTH_4W: if (i_rx_strb == 2'b11)
			sync_sreg <= { sync_sreg[11:0], i_rx_data[11:8], i_rx_data[3:0] };	
			else if (i_rx_strb[1])
				sync_sreg <= { sync_sreg[15:0], i_rx_data[11:8] };	
			else
				sync_sreg <= { sync_sreg[15:0], i_rx_data[3:0] };	
		default: if (i_rx_strb == 2'b11)
			sync_sreg <= { sync_sreg[3:0], i_rx_data[15:8], i_rx_data[7:0] };	
			else if (i_rx_strb[1])
				sync_sreg <= { sync_sreg[11:0], i_rx_data[15:8] };	
			else
				sync_sreg <= { sync_sreg[11:0], i_rx_data[7:0] };	
		endcase
	end
	// }}}

	// Step #2: Byte sync: s2_valid, s2_fill (byte count), and s2_data
	// {{{
	// We assume this register gets fully and completely emptied on every
	// cycle, so we only have to capture what spills out from the sync_*
	// registers.
	always @(posedge i_clk)
	if (i_reset || !i_rx_en || !syncd || i_cfg_ds)
		// If RX is not enabled, or
		//   or we haven't seen the sync bit(s)
		//   or we are using the async path,
		// Then keep s2 disabled
		s2_valid <= 0;
	else
		s2_valid <= sync_fill >= 8;

	always @(posedge i_clk)
	if (i_reset || !i_rx_en || !syncd)
		s2_fill <= 0;
	else
		s2_fill <= sync_fill[4:3];

	// Verilator lint_off WIDTH
	always @(posedge i_clk)
	if (sync_fill[4])
		s2_data <= sync_sreg >> sync_fill[2:0];
	else if (sync_fill[3])
		s2_data <= {sync_sreg, 8'h0} >> sync_fill[2:0];
	// Verilator lint_on  WIDTH
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Asynchronous (w/ DS) data path
	// {{{

	// Nothing really more needs to be done.  The async path comes in at
	// 4bytes at a times, all four valid (presently), and needs to be
	// synchronized to a 4-byte stream w/ all four valid.  Of course, this
	// will only work as long as the DS strobe signals only ever arrive
	// in pairs of pulses.

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Align, and write to (external) memory
	// {{{
	// (Memory is just external to this module, not necessarily to the chip)
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	if (i_reset || !i_rx_en || mem_full
			|| (mem_valid && &mem_addr && mem_strb[0]))
		mem_valid <= 0;
	else if (!i_cfg_ds)
	begin
		mem_valid <= s2_valid;
	end else begin
		mem_valid <= S_ASYNC_VALID && data_phase;
	end

	always @(posedge i_clk)
	if (i_reset || !i_rx_en)
		mem_full <= 0;
	else if (mem_valid && &mem_addr && mem_strb[0])
		mem_full <= 1;

	always @(posedge i_clk)
	if (i_reset || !i_rx_en)
		{ mem_addr, subaddr } <= 0;
	else if (!i_cfg_ds)
	begin
		// Verilator lint_off WIDTH
		{ mem_addr, subaddr } <= { mem_addr, subaddr } + s2_fill;
		// Verilator lint_on  WIDTH
	end else begin
		{ mem_addr, subaddr } <= { mem_addr, subaddr } + 4;
	end

	always @(posedge i_clk)
	if (i_reset || !i_rx_en || mem_full)
		mem_data <= 0;
	else if (!i_cfg_ds)
	begin
		mem_strb <= 0;

		if (s2_fill[1])
		begin
			mem_data <= { s2_data[15:0], {(MW-16){1'b0}} } >> (subaddr*8);
			mem_strb <= { 2'b11, {(MW/8-2){1'b0}} } >> (subaddr);
		end else begin
			mem_data <= { s2_data[7:0], {(MW-8){1'b0}} } >> (subaddr*8);
			mem_strb <= { 1'b1, {(MW/8-1){1'b0}} } >> (subaddr);
		end
	end else if (S_ASYNC_VALID)
	begin
		mem_data <= { S_ASYNC_DATA, {(MW-32){1'b0}} } >> (subaddr*8);
		mem_strb <= { 4'hf, {(MW/8-4){1'b0}} } >> (subaddr);
	end else
		mem_strb <= 0;

	assign	o_mem_valid = mem_valid;
	assign	o_mem_addr  = mem_addr;

	generate if (OPT_LITTLE_ENDIAN)
	begin
		reg	[MW/8-1:0]	swap_strb;
		reg	[MW-1:0]	swap_data;
		integer			ik;

		always @(*)
		for(ik=0; ik<MW/8; ik=ik+1)
		begin
			swap_strb[ik] = mem_strb[MW/8-1-ik];
			swap_data[ik*8 +: 8] = mem_strb[MW-ik*8 +: 8];
		end

		assign	o_mem_strb  = swap_strb;
		assign	o_mem_data  = swap_data;
	end else begin
		assign	o_mem_strb  = mem_strb;
		assign	o_mem_data  = mem_data;
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Are we done?  Have we received a full packet?
	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	// {{{

	always @(posedge i_clk)
	if (i_reset)
		rail_count <= 0;
	else if (!busy && i_rx_en)
	begin
		// Verilator lint_off WIDTH
		if (i_cfg_ds)
			rail_count <= i_length + 16;
		else case(i_cfg_width)
		WIDTH_1W: rail_count <= (i_length << 3) + 16;
		WIDTH_4W: rail_count <= (i_length << 1) + 16;
		default:  rail_count <= i_length + 16;
		endcase
		// Verilator lint_on  WIDTH

		data_phase <= 1;
		load_crc   <= 0;
	end else if (!i_cfg_ds)
	begin
		if (syncd && i_rx_strb == 2'b11)
		begin
			rail_count <= rail_count - 2;
			data_phase <= (rail_count > 18);
			load_crc   <= (rail_count <= 18)&&(rail_count > 2);

			if (rail_count < 2)
				rail_count <= 0;
		end else if ((syncd && |i_rx_strb)
					|| (i_rx_strb == 2'b11 && i_sync))
		begin
			rail_count <= rail_count - 1;
			data_phase <= (rail_count > 17);
			load_crc   <= (rail_count <= 17)&&(rail_count > 1);

			if (rail_count < 1)
				rail_count <= 0;
		end
	end else if (S_ASYNC_VALID)
	begin
		rail_count <= rail_count - 4;
		data_phase <= (rail_count > 20);
		load_crc   <= (rail_count <= 20)&&(rail_count > 4);

		if (rail_count < 4)
			rail_count <= 0;
	end

	always @(posedge i_clk)
	if (i_reset)
		busy <= 0;
	else if (!busy)
		busy <= i_rx_en && i_length > 0;
	else
		busy <= (data_phase || load_crc);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CRC checking
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	generate for(gk=0; gk<NUMIO; gk=gk+1)
	begin

		// crc[gk]: Calculate the CRC for each rail
		// {{{
		initial	crc[gk] = 0;
		always @(posedge i_clk)
		if (i_reset || !i_rx_en || !i_crc_en || !data_phase)
			crc[gk] <= 0;
		else if (!i_cfg_ds)
		begin // CRC based upon synchronous inputs
			// {{{
			if (gk >= 1 && i_cfg_width == WIDTH_1W)
				// Zero out any unused rails
				crc[gk] <= 0;
			else if (gk >= 4 && i_cfg_width == WIDTH_4W)
				crc[gk] <= 0;
			else if (syncd && i_rx_strb == 2'b11)
				crc[gk] <= STEPCRC(STEPCRC(crc[gk],
					i_rx_data[8+gk]),
					i_rx_data[gk]);
			else if (syncd && i_rx_strb[1])
				crc[gk] <= STEPCRC(crc[gk],
						i_rx_data[8+gk]);
			else if ((syncd || (i_sync && i_rx_strb[1]))
						&& i_rx_strb[0])
				crc[gk] <= STEPCRC(crc[gk], i_rx_data[0+gk]);
			// }}}
		end else if (S_ASYNC_VALID)
		begin // Asynchronous CRC generation
			// {{{
			// Note that in ASYNC mode, all rails are used (IIRC)
			crc[gk] <= STEPCRC(
					STEPCRC(
						STEPCRC(
							STEPCRC(crc[gk],
								S_ASYNC_DATA[gk+24]),
							S_ASYNC_DATA[gk+16]),
						S_ASYNC_DATA[gk+8]),
					S_ASYNC_DATA[gk]);
			// }}}
		end
		// }}}

		always @(posedge i_clk)
		if (i_reset || !i_rx_en || !i_crc_en || !load_crc)
		begin
			check[gk] <= 0;
		end else if (!i_cfg_ds)
		begin
			// {{{
			if (!syncd)
				check[gk] <= 0;
			else if (gk >= 1 && i_cfg_width == WIDTH_1W)
				// Zero out any unused rails
				check[gk] <= 0;
			else if (gk >= 4 && i_cfg_width == WIDTH_4W)
				// Zero out any unused rails
				check[gk] <= 0;
			else if (i_rx_strb == 2'b11)
				check[gk] <= { check[gk][13:0],
					i_rx_data[8+gk], i_rx_data[gk] };
			else if (i_rx_strb[1])
				check[gk] <= { check[gk][14:0],
						i_rx_data[8+gk] };
			else if (i_rx_strb[0])
				check[gk] <= { check[gk][14:0], i_rx_data[0+gk] };
			// }}}
		end else if (S_ASYNC_VALID)
		begin
			check[gk] <= { check[gk][11:0],
				S_ASYNC_DATA[24+gk], S_ASYNC_DATA[16+gk],
				S_ASYNC_DATA[ 8+gk], S_ASYNC_DATA[ 0+gk] };
		end

		always @(posedge i_clk)
		if (i_reset || !i_rx_en || !i_crc_en || data_phase || load_crc)
		begin
			good[gk] <= 0;
			err[gk]  <= 0;
		end else begin
			good[gk] <= (crc[gk] == check[gk]);
			err[gk]  <= (crc[gk] != check[gk]);
		end
	end endgenerate

	initial	o_done = 0;
	always @(posedge i_clk)
	if (i_reset || !i_rx_en)
		{ o_done, o_err } <= 0;
	else if (!o_done)
	begin
		if (i_rx_en && (data_phase || load_crc))
			{ o_done, o_err } <= {(2){r_watchdog}};
		else if (mem_valid || s2_valid)
			{ o_done, o_err } <= 0;
		else if (good != 0 || err != 0)
		begin
			o_done <= 1'b1;
			o_err  <= |err;
		end
	end

	function automatic [NCRC-1:0]	STEPCRC(reg[NCRC-1:0] prior,
						input i_data);
	begin
		if (prior[NCRC-1] ^ i_data)
			STEPCRC = { prior[NCRC-2:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			STEPCRC = { prior[NCRC-2:0], 1'b0 };
	end endfunction

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Watchdog timeout
	// {{{

	always @(posedge i_clk)
	if (i_reset || !i_rx_en
			|| (!i_cfg_ds && i_rx_strb != 0)
			|| (i_cfg_ds && S_ASYNC_VALID))
	begin
		r_watchdog <= 0;
		r_timeout  <= -1;
	end else if (r_timeout != 0)
	begin
		r_watchdog <= (r_timeout <= 1);
		r_timeout  <= r_timeout - 1;
	end
	// }}}

	//
	// Make verilator happy
	// verilator lint_off UNUSED
	// wire	unused;
	// assign	unused = i_wb_cyc;
	// verilator lint_on  UNUSED

`ifdef	FORMAL
	reg	f_past_valid;
	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
		assume(i_stb = (!i_reset));

	reg	[14:0]	f_count;
	initial	f_count = 0;
	always @(posedge i_clk)
	if (i_reset)
		f_count <= 0;
	else if (i_stb)
		f_count <= f_count + 1;

	always @(posedge i_clk)
		assert(f_count == count);

	always @(posedge i_clk)
	if (i_reset)
		assume(i_data == 0);

	always @(*)
		assume(i_length == 4096);
	always @(*)
	if (busy)
		assert(length == 4096);
	always @(*)
		assume(i_stb);

	else if (i_stb)
	begin
		if (f_step < length)
			assume(i_data == 4'hf);
		else
			assert(f_step != length);
	end

	always @(posedge i_clk)
		assert(count < length+16);
	always @(posedge i_clk)
		busy = (count >0)&&(count < length+16);

`endif	// FORMAL
endmodule

