////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	spitxdata.v
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	To handle all of the processing associated with sending data
//		from a memory to our lower-level SPI processor.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2021, Gisselquist Technology, LLC
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype none
// }}}
module spitxdata #(
		// {{{
		parameter	DW = 32, AW = 8, RDDELAY = 2,
		localparam	CRC_POLYNOMIAL = 16'h1021
		// }}}
	) (
		// {{{
		input	wire	i_clk, i_reset,
		//
		input	wire		i_start,
		input	wire	[3:0]	i_lgblksz,
		input	wire		i_fifo,
		output	reg		o_busy,
		//
		output	reg 		o_read,
		output	reg [AW-1:0]	o_addr,
		input	wire [DW-1:0]	i_data,
		//
		input	wire		i_ll_busy,
		output	reg		o_ll_stb,
		output	wire [7:0]	o_ll_byte,
		//
		input	wire		i_ll_stb,
		input	wire [7:0]	i_ll_byte,
		//
		output	reg 		o_rxvalid,
		output	reg [7:0]	o_response
		// }}}
	);

	// Signal declarations
	// {{{
	reg	[RDDELAY-1:0]		rdvalid;
	reg	[8+DW-1:0]		gearbox;
	reg	[1+(DW/8)-1:0]		fill;
	reg				crc_flag, crc_stb, data_read,
					all_mem_read;

	reg				lastaddr, data_sent, received_token,
					all_idle;
	reg				crc_active;
	reg [$clog2(1+DW/2)-1:0]	crc_fill;
	(* keep *) reg	[DW-1:0]	crc_gearbox;
	reg	[15:0]			crc_data;

	reg				token;
	reg	[2:0]			r_lgblksz_m3;
	reg	[15:0]			next_crc_data;
	// }}}

	// token
	// {{{
	always @(*)
		token = (data_sent && i_ll_stb && i_ll_byte[0] &&!i_ll_byte[4]);
	// }}}

	// o_busy
	// {{{
	initial	o_busy = 0;
	always @(posedge i_clk)
	if (i_reset)
		o_busy <= 0;
	else if (!o_busy)
		o_busy <= i_start;
	else if (all_idle && i_ll_stb && (&i_ll_byte))
		o_busy <= 0;
	// }}}

	// o_rxvalid, received_token
	// {{{
	initial	o_rxvalid = 0;
	initial	received_token = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		{ received_token, o_rxvalid } <= 0;
	else if (token && !received_token)
		{ received_token, o_rxvalid } <= 2'b11;
	else
		o_rxvalid <= 0;
	// }}}

	// all_idle
	// {{{
	initial	all_idle = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		all_idle <= 0;
	else if (received_token && i_ll_stb && (&i_ll_byte))
		all_idle <= 1'b1;
	// }}}

	// o_response
	// {{{
	always @(posedge i_clk)
	if (token)
		o_response <= i_ll_byte;
	// }}}

	// o_read & o_addr
	// 0: arbited read request
	// 1: read data
	// 2: muxed read data

	// rdvalid
	// {{{
	initial	rdvalid = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		rdvalid <= 0;
	else
		rdvalid <= { rdvalid[RDDELAY-2:0], o_read };
	// }}}

	// o_ll_stb
	// {{{
	initial	o_ll_stb = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		o_ll_stb <= 0;
	else if (rdvalid[RDDELAY-1])
		o_ll_stb <= 1;
	else if (data_read && !fill[4])
		o_ll_stb <= 0;
	// }}}

	// fill
	// {{{
	initial	fill = 0;
	always @(posedge i_clk)
	begin
		if (!o_ll_stb && rdvalid[RDDELAY-1])
		begin
			gearbox <= { 8'hfe, i_data };
			fill <= 5'h1f;
		end else if (rdvalid[RDDELAY-1])
		begin
			gearbox <= { gearbox[DW+8-1:DW], i_data };
			fill <= 5'h1f;
		end else if (crc_stb)
		begin
			gearbox <= { gearbox[DW+8-1:DW], crc_data, 16'hff };
			fill[3:0] <= 4'hc;
		end else if (o_ll_stb && !i_ll_busy)
		begin
			gearbox <= { gearbox[DW-1:0], 8'hff };
			fill <= fill << 1;
		end

		if (!o_busy)
			gearbox[39:32] <= 8'hfe;	// Start token

		if (i_reset)
			fill <= 0;
		else if (!o_busy)
			fill <= (i_start) ? 5'h10 : 0;
	end
	// }}}

	assign	o_ll_byte = gearbox[39:32];

	// crc_stb, o_read
	// {{{
	initial	{ crc_stb, o_read } = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		{ crc_stb, o_read } <= 0;
	else if (!fill[3] || (!fill[2] && (!o_ll_stb || !i_ll_busy)))
	begin
		{ crc_stb, o_read } <= 0;
		if (!o_read && rdvalid == 0 && !data_read)
		begin
			if (!all_mem_read)
				o_read <= 1;
			else
				crc_stb <= (!crc_flag)&&(!crc_stb);
		end
	end else
		{ crc_stb, o_read } <= 0;
	// }}}

	// o_addr
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		o_addr <= { i_fifo, {(AW-1){1'b0}} };
	else if (o_read && !lastaddr)
		o_addr[AW-2:0] <= o_addr[AW-2:0] + 1;
	// }}}

	// all_mem_read
	// {{{
	initial	all_mem_read = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		all_mem_read <= 0;
	else if (o_read && lastaddr)
		all_mem_read <= 1;
	// }}}

	// crc_flag
	// {{{
	initial	crc_flag = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		crc_flag <= 0;
	else if (crc_stb)
		crc_flag <= 1;
	// }}}

	// data_read
	// {{{
	always @(*)
		data_read = crc_flag;
	// }}}

	// data_sent
	// {{{
	initial	data_sent = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		data_sent <= 1'b0;
	else if (data_read && !fill[3] && o_ll_stb && !i_ll_busy)
		data_sent <= 1'b1;
	// }}}

	// r_lgblksz_m3, lastaddr
	// {{{
	initial	r_lgblksz_m3 = 0;
	initial	lastaddr = 0;
	always @(posedge i_clk)
	if (!o_busy)
	begin
		lastaddr <= (i_lgblksz < 4);
		// Verilator lint_off WIDTH
		r_lgblksz_m3 <= i_lgblksz-3;
		// Verilator lint_on WIDTH
	end else if (o_read && !lastaddr)
	begin
		case(r_lgblksz_m3)
		0: begin end // assert(lastaddr);		//   8 bytes
		1: lastaddr <= (&o_addr[1:1]);	//  16 bytes
		2: lastaddr <= (&o_addr[2:1]);	//  32 bytes
		3: lastaddr <= (&o_addr[3:1]);	//  64 bytes
		4: lastaddr <= (&o_addr[4:1]);	// 128 bytes
		5: lastaddr <= (&o_addr[5:1]);	// 256 bytes
		default: lastaddr <= (&o_addr[6:1]);	// 512 bytes
		endcase
	end
	// }}}

	////////////////////////////////////////////////////////////////////////
	//
	// CRC calculation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// crc_fill
	// {{{
	initial	crc_fill = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
	begin
		crc_fill <= 0;
		crc_active <= 0;
	end else if (crc_active || rdvalid[RDDELAY-1])
	begin
		// Verilator lint_off WIDTH
		crc_fill <= crc_fill - (crc_active ? 1:0)
					+ (rdvalid[RDDELAY-1] ? (DW/2):0);
		// Verilator lint_on WIDTH
		if (rdvalid[RDDELAY-1])
			crc_active <= 1;
		else
			crc_active <= (crc_fill > 1);
	end
	// }}}

	// crc_gearbox
	// {{{
	always @(posedge i_clk)
	if (!crc_active)
		crc_gearbox <= i_data;
	else
		crc_gearbox <= { crc_gearbox[DW-3:0], 2'b00 };
	// }}}

	// next_crc_data
	// {{{
	always @(*)
	begin
		next_crc_data = crc_data << 1;;

		if (crc_data[15] ^ crc_gearbox[31])
			next_crc_data = next_crc_data ^ CRC_POLYNOMIAL;

		if (next_crc_data[15] ^ crc_gearbox[30])
			next_crc_data = (next_crc_data << 1) ^ CRC_POLYNOMIAL;
		else
			next_crc_data = (next_crc_data << 1);
	end
	// }}}

	// crc_data
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		crc_data <= 0;
	else if (crc_active)
		crc_data <= next_crc_data;
	// }}}
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`ifdef	SPITXDATA
`define	ASSUME	assume
`else
`define	ASSUME	assert
`endif

	reg		f_past_valid;
	reg	[3:0]	f_lgblksz;
	wire	[3:0]	f_lgblksz_m3;


	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	// Reset check(s)
	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(!o_ll_stb);
		assert(!o_busy);
		assert(rdvalid  == 0);
		assert(!o_rxvalid);
	end

	always @(posedge i_clk)
	if (!f_past_valid || !$past(o_busy))
	begin
		assert(rdvalid  == 0);
		assert(!o_rxvalid);
		if (!f_past_valid || $past(i_reset))
			assert(fill == 0);
		else if ($past(i_start))
			assert(fill == 5'h10);
		else
			assert(fill == 0);
		assert(crc_fill == 0);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Data assumptions
	//
	reg	[6:0]	f_last_read;

	initial	f_last_read = 0;
	always @(posedge i_clk)
	begin
		f_last_read <= f_last_read << 1;
		if (o_ll_stb && !i_ll_busy)
			f_last_read[0] <= 1;
	end

`ifdef	VERIFIC
	always @(*)
		`ASSUME($onehot0({f_last_read, (o_ll_stb && !i_ll_busy)}));
`else
	always @(*)
	case({f_last_read, (o_ll_stb && !i_ll_busy)})
	8'h00: begin end
	8'h01: begin end
	8'h02: begin end
	8'h04: begin end
	8'h08: begin end
	8'h10: begin end
	8'h20: begin end
	8'h40: begin end
	8'h80: begin end
	default: `ASSUME(0);
	endcase
`endif


	always @(*)
	if (i_start)
	begin
		`ASSUME(i_lgblksz <= 9);
		`ASSUME(i_lgblksz  > 2);
	end

	always @(*)
	if (crc_fill > 1)
	begin
		assert(o_ll_stb);
		assert(!crc_stb);
	end

	always @(*)
	if (o_read || rdvalid != 0)
		assert(!o_ll_stb || i_ll_busy);

	////////////////////////////////////////////////////////////////////////
	//
	// Induction assertions
	//

	always @(*)
	if (rdvalid != 0)
		assert(crc_fill <= 1);

	always @(*)
	if (o_busy)
		assert(crc_active == (crc_fill > 0));

	always @(*)
	if (o_read)
		assert(rdvalid == 0);
	else
`ifdef	VERIFIC
		assert($onehot0(rdvalid));
`else
	begin
		assert((rdvalid == 0)
			||(rdvalid == 1)
			||(rdvalid == 2)
			||(rdvalid == 4));
	end
`endif

	always @(*)
	if (o_busy && all_idle)
		assert(received_token);

	always @(*)
	if (o_busy && received_token)
		assert(data_sent);

	always @(*)
	if (o_busy && data_sent)
	begin
		assert(data_read);
		assert(!crc_stb);
		assert(!crc_active);
	end

	always @(*)
	if (crc_fill > 5'h0c)
		assert(&fill[4:1]);
	else if (crc_fill > 4)
		assert(&fill[4:2]);
	else if (crc_fill > 0)
		assert(&fill[4:3]);

	always @(*)
		assert(crc_fill <= 5'h10);
	always @(*)
	if (o_busy && data_read)
	begin
		assert(crc_flag);
		assert(!crc_stb);
		assert(!crc_active);
	end

	always @(*)
	if (o_busy && crc_flag)
	begin
		assert(!crc_stb);
		assert(all_mem_read);
		assert(!crc_active);
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset) && $past(o_busy))
		assert($rose(crc_flag) == $past(crc_stb));

	always @(*)
	if (o_busy && all_mem_read)
		assert(lastaddr);

	always @(*)
	if (o_busy && !lastaddr)
		assert(!all_mem_read);

	always @(posedge i_clk)
	if (!o_busy)
		f_lgblksz <= i_lgblksz;

	assign	f_lgblksz_m3 = f_lgblksz - 3;

	always @(*)
	if(o_busy)
		assert(f_lgblksz >= 3);

	always @(*)
	if (o_busy)
		assert(f_lgblksz_m3[2:0] == r_lgblksz_m3);

	always @(*)
	if (o_busy)
	case(r_lgblksz_m3)
	3'h0: assert(lastaddr);			//   8 bytes
	3'h1: assert(lastaddr == (&o_addr[1:0]));	//  16 bytes
	3'h2: assert(lastaddr == (&o_addr[2:0]));	//  32 bytes
	3'h3: assert(lastaddr == (&o_addr[3:0]));	//  64 bytes
	3'h4: assert(lastaddr == (&o_addr[4:0]));	// 128 bytes
	3'h5: assert(lastaddr == (&o_addr[5:0]));	// 256 bytes
	default: assert(lastaddr == (&o_addr[AW-2:0]));
	endcase

	always @(*)
	if (o_ll_stb && !data_sent)
		assert(fill[4]);

	always @(*)
	if (fill != 0)
		assert(fill[DW/8]);

	genvar	k;

	generate for(k=DW/8; k>0; k=k-1)
	begin
		always @(*)
		if ((fill != 0) && !fill[k])
			assert(fill[k-1:0]==0);
	end endgenerate

	always @(posedge i_clk)
	if (f_past_valid && $past(crc_stb))
		assert(!crc_stb);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal property section
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	SPITXDATA
	reg	[5:0]		f_read_seq;
	(* anyseq *) reg	f_read_check;
	reg	[DW-1:0]	f_read_data;

	always @(posedge i_clk)
	if (rdvalid[RDDELAY-1])
		f_read_data <= i_data;

	always @(*)
	if (f_read_seq != 0)
		assume(!f_read_check);

	initial	f_read_seq = 0;
	always @(posedge i_clk)
	if (!o_busy)
		f_read_seq <= 0;
	else if (f_read_check && o_read)
		f_read_seq <= 1;
	else if (rdvalid != 0)
	begin
		if (rdvalid[RDDELAY-1])
			f_read_seq <= 2;
	end else if (!i_ll_busy)
		f_read_seq <= (f_read_seq << 1);

	always @(*)
	if (i_reset || !o_busy || (f_read_seq == 0))
	begin end
	else case(f_read_seq)
	6'h01: begin
		assert(rdvalid != 0);
		assert(fill == 5'h10);
		end
	6'h02: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h1f);
		assert(gearbox[DW-1:0] == f_read_data);
		end
	6'h04: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h1e);
		assert(gearbox[8+DW-1:8] == f_read_data);
		end
	6'h08: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h1c);
		assert(gearbox[8+DW-1:16] == f_read_data[23:0]);
		end
	6'h10: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h18);
		assert(gearbox[8+DW-1:24] == f_read_data[15:0]);
		end
	6'h20: begin
		assert(o_ll_stb);
		assert(fill[4]);
		assert(gearbox[8+DW-1:DW] == f_read_data[7:0]);
		end
	default:
`ifdef	VERIFIC
		assert($onehot0(f_read_seq));
`else
		if (f_read_seq)
			assert(0);
`endif
	endcase
`endif
	////////////////////////////////////////////////////////////////////////
	//
	// Cover properties
	//
	always @(posedge i_clk)
	begin
		cover(f_lgblksz == 4 && o_rxvalid);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h00);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h01);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h02);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h03);
		cover(o_busy && f_lgblksz == 4 && lastaddr);
		cover(o_busy && f_lgblksz == 4 && crc_stb);
		cover(o_busy && f_lgblksz == 4 && all_mem_read);
		cover(o_busy && f_lgblksz == 4 && crc_flag);
		cover(o_busy && f_lgblksz == 4 && data_read);
		cover(o_busy && f_lgblksz == 4 && data_sent);
		cover(o_busy && f_lgblksz == 4 && received_token);
		cover(o_busy && f_lgblksz == 4 && all_idle);
		cover(!o_busy && f_lgblksz == 4 && all_idle);
	end
`endif
// }}}
endmodule
