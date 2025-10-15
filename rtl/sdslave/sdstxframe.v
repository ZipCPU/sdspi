////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdstxframe.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	SDIO Slave receive module.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2025, Gisselquist Technology, LLC
// {{{
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
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
`default_nettype none
// }}}
module	sdstxframe #(
		parameter [0:0]	OPT_DDR = 1'b1,
		parameter [0:0]	OPT_DS  = 1'b0,
		parameter	NUMIO = 4
	) (
		// {{{
		input	wire		i_clk,	// MUST be the SD card clock
					i_reset,
		//
		input	wire		i_cfg_en,
		// input wire	[3:0]	i_cfg_sz, // 4=16B, 9=512B, 15=32768B
		input	wire		i_cfg_pp,
		input	wire		i_cfg_ds,
		input	wire		i_cfg_ddr,
		input	wire	[1:0]	i_cfg_width,	// 0=1b, 1=4b, 2=8b
		//
		input	wire		i_busy, i_rxgood, i_rxfail,
		//
		input	wire		i_valid,
		output	wire		o_ready,
		input	wire	[31:0]	i_data,
		input	wire		i_last,
		//
		output	wire	[15:0]	o_data,
		output	wire	[7:0]	o_tristate,
		output	wire	[1:0]	o_ds,
		// No DS tristate, DS is always an output
		//
		output	wire		o_done
		// }}}
	);

	// Local declarations
	// {{{
	localparam	NCRC = 16;
	localparam	[NCRC-1:0]	CRC_POLY = 16'h1021;

	localparam	WIDTH_1W = 2'b00,
			WIDTH_4W = 2'b01,
			WIDTH_8W = 2'b10;
	integer	ik, jk;
	reg		started, crc_active, last, s_last, ready;
	reg	[4:0]	count;
	reg	[31:0]	sreg;
	reg	[NCRC-1:0]	crc_fill	[2*NUMIO-1:0];

	reg		token_active;
	reg	[2:0]	token_sreg, token_count;

	reg	[15:0]	iovec;
	reg	[7:0]	iotri;
	wire		cfg_ddr;
	wire	[1:0]	cfg_width;
	// }}}

	assign	cfg_ddr = OPT_DDR && i_cfg_ddr;
	assign	cfg_width = (NUMIO >= 8) ? i_cfg_width
			: (NUMIO < 4) ? WIDTH_1W
			: { 1'b0, i_cfg_width[0] };

	// started, crc_active, count, last, s_last
	// {{{
	always @(posedge i_clk)
	if (i_reset || !i_cfg_en)
	begin
		started <= 0;
		crc_active <= 0;
		count <= 0;
		last <= 0;
		s_last <= 0;
		ready  <= 1'b0;
	end else if (i_valid && (!started || o_ready))
	begin	// Start bit and subsequent data beats
		// {{{
		last <= 0;
		crc_active <= 0;
		if (!started)
		begin
			s_last <= 0;
			count <= 0;
			started <= !i_busy && !i_rxgood && !i_rxfail
						&& !token_active;
			ready <= !i_busy && !i_rxgood && !i_rxfail
						&& !token_active;
		end else // if (o_ready)
		begin
			ready <= 0;
			s_last <= i_last;
			case(cfg_width)
			WIDTH_4W: if (cfg_ddr)
					count <= count + 3; // (32-8)/8;
				else
					count <= count + 7; // (32-4)/4;
			WIDTH_8W: if (cfg_ddr)
					count <= count + 1; // (32-16)/16;
				else
					count <= count + 3; // (32-8)/8
			default: if (cfg_ddr)
					count <= count + 15; // (32-2) / 2;
				else
					count <= count + 31; // (32-1) / 1
			endcase
		end
		// }}}
	end else if (!last && !crc_active)
	begin // Empty shift register
		// {{{
		if (count > 0)
			count <= count - 1;
		ready <= (count <= 1) && !s_last;
		crc_active <= (count <= 1) && s_last;
		if (!i_valid && o_ready)
		begin
			count <= 15;
			ready <= 0;
			last <= 0;
			crc_active <= 1;
		end
		// }}}
	end else if (!last) // && crc_active
	begin
		count <= count - 1;
		last <= (count <= 1);
	end else begin
		started <= 0;
		last <= 0;
		count <= 0;
		crc_active <= 0;
		s_last <= 0;
	end

	assign	o_ready = ready;
	assign	o_done  = started && last && crc_active;
	// }}}

	always @(posedge i_clk)
	if (i_reset || !i_cfg_en || !started)
	begin
		sreg  <= 32'hffff_ffff;
	end else if (i_valid && o_ready)
	begin
		if (cfg_ddr)
		begin
			// {{{
			case(cfg_width)
			WIDTH_4W: sreg <= { i_data[27:24], i_data[19:16],
						i_data[15:12], i_data[7:4],
						i_data[11: 8], i_data[3:0],
						8'hff };
			WIDTH_8W: sreg <= { i_data[15:0], 16'hff_ff };
			// WIDTH_1W:
			default: sreg <= { i_data[30], i_data[22],
					i_data[29], i_data[21],
					i_data[28], i_data[20],
					i_data[27], i_data[19],
					i_data[26], i_data[18],
					i_data[25], i_data[17],
					i_data[24], i_data[16],
					//
					i_data[15], i_data[ 7],
					i_data[14], i_data[ 6],
					i_data[13], i_data[ 5],
					i_data[12], i_data[ 4],
					i_data[11], i_data[ 3],
					i_data[10], i_data[ 2],
					i_data[ 9], i_data[ 1],
					i_data[ 8], i_data[ 0], 2'b11 };
			endcase
			// }}}
		end else begin
			// {{{
			case(cfg_width)
			WIDTH_4W: begin
				sreg <= { i_data[27:0], 4'hf };
				end
			WIDTH_8W: begin
				sreg <= { i_data[23:0], 8'hff };
				end
			// WIDTH_1W:
			default: begin
				sreg <= { i_data[30:0], 1'b1 };
				end
			endcase
			// }}}
		end
	end else // if (i_ready)
	begin
		if (cfg_ddr)
		begin
			// {{{
			case(cfg_width)
			WIDTH_4W: sreg <= { sreg[23:0], 8'hff };
			WIDTH_8W: sreg <= 32'hffff_ffff;
			// WIDTH_1W:
			default: sreg <= { sreg[29:0], 2'b11 };
			endcase
			// }}}
		end else begin
			// {{{
			case(cfg_width)
			WIDTH_4W: sreg <= { sreg[27:0], 4'hf };
			WIDTH_8W: sreg <= { sreg[23:0], 8'hff };
			// WIDTH_1W:
			default: sreg <= { sreg[30:0], 1'b1 };
			endcase
			// }}}
		end
	end

	// token_sreg, token_active, token_count
	// {{{
	always @(posedge i_clk)
	if (i_reset || i_cfg_en)
	begin
		token_sreg <= 3'b111;
		token_active <= 0;
		token_count <= 4;
	end else if (i_rxgood || i_rxfail)
	begin
		if (i_rxgood)	// 0-010-1
			token_sreg <= 3'b010;
		else
			token_sreg <= 3'b101;
		token_active <= 1;
		token_count <= 4;
	end else // if (token_active)
	begin
		token_sreg <= { token_sreg[1:0], 1'b1 };
		token_count <= (token_active) ? (token_count - 1) : 0;
		token_active <= (token_count > 1);
	end
	// }}}

	// iovec, iotri
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		iovec <= 16'hff_ff;
		iotri <= 8'hff;	// All pins in tristate
	end else if (token_active)
	begin
		iovec <= {(2){ 7'hf, token_sreg[2] }};
	end else if (i_rxgood || i_rxfail)
	begin
		iovec <= {(2){8'hfe}};
		iotri <= 8'hfe;
	end else if (!i_cfg_en)
	begin
		// {{{
		iovec <= {(16){1'b1}};
		iotri <= {( 8){1'b1}};
		if (i_busy)
		begin
			iovec[8] <= 1'b0;
			iovec[0] <= 1'b0;
			iotri[0] <= 1'b0;
		end
		// }}}
	end else if (i_valid && !started)
	begin
		// {{{
		iovec <= 16'h00_00;
		case(cfg_width)
		WIDTH_4W: iotri <= 8'hf0;	// Bottom four pins drive
		WIDTH_8W: iotri <= 8'h00;	// All pins drive
		// WIDTH_1W:
		default:  iotri <= 8'hfe;	// Only bottom pin drives
		endcase
		// }}}
	end else if (i_valid && o_ready)
	begin
		// {{{
		case(cfg_width)
		WIDTH_4W: begin
			if (cfg_ddr)
			begin
				iovec <= { 4'hf, i_data[31:28],
							4'hf, i_data[23:20] };
				iotri[3:0] <= i_data[31:28] & i_data[23:20];
				iotri[7:4] <= 4'hf;
			end else begin
				iovec <= {(2){ 4'hf, i_data[31:28] } };
				iotri <= { 4'hf, i_data[31:28] };
			end
			if (i_cfg_pp)
				iotri <= 8'hf0;
			end
		WIDTH_8W: begin
			if (cfg_ddr)
			begin
				iovec <= i_data[31:16];
				iotri <= i_data[31:24] & i_data[23:16];
			end else begin
				iovec <= {(2){i_data[31:24]}};
				iotri <= i_data[31:24];
			end
			if (i_cfg_pp)
				iotri <= 8'h00;
			end
		// WIDTH_1W:
		default: begin
			if (cfg_ddr)
			begin
				iovec <= { 7'h7f, i_data[31],
						7'h7f, i_data[23] };
				iotri <= { 7'h7f, i_data[31] && i_data[23] };
			end else begin
				iovec <= {(2){ 7'h7f, i_data[31] }};
				iotri <= { 7'h7f, i_data[31] };
			end
			if (i_cfg_pp)
				iotri <= 8'hfe;
			end
		endcase
		// }}}
	end else if (crc_active)
	begin
		// {{{
		iovec <= 16'hff_ff;
		iotri <= 8'hff;
		for(ik=0; ik<NUMIO; ik=ik+1)
		begin
			iovec[      ik] <= crc_fill[      ik][15];
			iovec[NUMIO+ik] <= crc_fill[NUMIO+ik][15];
			iotri[ik] <= crc_fill[ik][15] & crc_fill[NUMIO+ik][15];

			if (!cfg_ddr)
			begin
				iovec[ik] <= crc_fill[NUMIO+ik][15];
				iotri[ik] <= crc_fill[NUMIO+ik][15];
			end

			if (i_cfg_pp)
				iotri[ik] <= 1'b0;
		end

		case(cfg_width)
		WIDTH_4W: begin
			iovec[15:12] <= 4'hf;
			iovec[ 7: 4] <= 4'hf;
			iotri[ 7: 4] <= 4'hf;
			end
		WIDTH_8W: begin end	// Already set properly by default
		default: begin
			iovec[15: 9] <= 7'h7f;
			iovec[ 7: 1] <= 7'h7f;
			iotri[ 7: 1] <= 7'hf;
			end
		endcase
		// }}}
	end else // if (!crc_active) ... but the shift register isn't empty
	begin
		// {{{
		case(cfg_width)
		WIDTH_4W: if (cfg_ddr)
				iovec <= { 4'hf, sreg[31:28],
							4'hf, sreg[27:24] };
			else
				iovec <= {(2){ 4'hf, sreg[31:28] }};
		WIDTH_8W: if (cfg_ddr)
				iovec <= sreg[31:16];
			else
				iovec <= {(2){ sreg[31:24] }};
		// WIDTH_1W:
		default: if (cfg_ddr)
				iovec <= { 7'h7f, sreg[31], 7'h7f, sreg[30] };
			else
				iovec <= {(2){ 7'h7f, sreg[31] }};
		endcase
		// }}}
	end

	assign	o_data = iovec;
	assign	o_tristate = iotri;
	// }}}

	// CRC fill and stepping
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		for(jk=0; jk<2*NUMIO; jk=jk+1)
			crc_fill[jk] <= 0;
	end else if (!started || last || !i_cfg_en || token_active)
	begin
		for(jk=0; jk<2*NUMIO; jk=jk+1)
			crc_fill[jk] <= 0;
	end else if (!crc_active)
	begin
		// {{{
		for(jk=0; jk<NUMIO; jk=jk+1)
		begin
			// Positive edge
			crc_fill[NUMIO+jk] <= ADVANCE_CRC(crc_fill[NUMIO+jk],
							iovec[NUMIO+jk]);
			// Negative edge
			crc_fill[jk] <= ADVANCE_CRC(crc_fill[jk], iovec[jk]);

			if (!cfg_ddr)
				crc_fill[jk] <= 0;

			case(cfg_width)
			WIDTH_4W: if (jk >= 4)
				begin
				crc_fill[jk] <= 0;
				crc_fill[NUMIO+jk] <= 0;
				end
			WIDTH_8W: begin end
			// WIDTH_1W:
			default: if (jk >= 1)
				begin
				crc_fill[jk] <= 0;
				crc_fill[NUMIO+jk] <= 0;
				end
			endcase
		end // for(ik ...)
		// }}}
	end else // if (crc_active)
	begin
		for(jk=0; jk<NUMIO; jk=jk+1)
		begin
			// Positive edge
			crc_fill[NUMIO+jk]<= { crc_fill[NUMIO+jk][14:0], 1'b1 };

			// Negative edge
			crc_fill[jk] <= { crc_fill[jk][14:0], 1'b1 };
		end
	end
	// }}}

	// DS handling
	// {{{
	generate if (OPT_DS)
	begin : GEN_DS
		reg	[1:0]	dsvec;

		// This is great, but ... what about enhanced commands?
		//   Simple: we don't support those (yet)
		always @(posedge i_clk)
		if (i_reset || !i_cfg_ds)
			dsvec <= 2'b00;
		else if (i_valid || crc_active || i_rxgood || i_rxfail
				|| token_active)
		begin
			dsvec <= 2'b10;
		end else
			dsvec <= 2'b00;

		assign	o_ds = dsvec;
	end else begin
		assign	o_ds = 2'b00;

		// Verilator lint_off UNUSED
		wire	unused_ds;
		assign	unused_ds = &{ 1'b0, i_cfg_ds };
	end endgenerate
	// }}}

	function automatic [NCRC-1:0]	ADVANCE_CRC(input[NCRC-1:0] prior,
		// {{{
						input i_crc_data);
	begin
		if (prior[NCRC-1] ^ i_crc_data)
			ADVANCE_CRC = { prior[NCRC-2:0], 1'b0 } ^ CRC_POLY;
		else
			ADVANCE_CRC = { prior[NCRC-2:0], 1'b0 };
	end endfunction
	// }}}
endmodule
