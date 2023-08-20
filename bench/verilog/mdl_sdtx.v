////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	mdl_sdtx.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Model the IO associated with transmitting data from an SD card
//		via the SDIO interface.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2023, Gisselquist Technology, LLC
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
`timescale 1ns/1ps
// }}}
module mdl_sdtx #(
		parameter realtime FF_HOLD  = 1.25
	) (
		// {{{
		inout	wire		sd_clk,
		inout	wire	[7:0]	sd_dat,
		output	wire		sd_ds,
		input	wire		i_en,
		input	wire	[1:0]	i_width,
		input	wire		i_ddr,
		//
		input	wire		i_valid,
		output	wire		o_ready,
		input	wire	[31:0]	i_data,
		input	wire		i_last
		// }}}
	);

	// Local declarations
	// {{{
	localparam	NCRC = 16;
	localparam [NCRC-1:0]	CRC_POLYNOMIAL = 16'h1021;

	genvar		gk;

	reg	[15:0]	crc	[15:0];
	reg	[47:0]	tx_sreg;
	reg	[5:0]	r_count;
	reg		r_crc, r_active, ds;
	reg	r_ready;
	// }}}

	// tx_sreg, r_count, r_crc r_active: positive edge of the clock
	// {{{
	// Setup for the positive clock edge
	always @(negedge sd_clk)
		r_ready <= o_ready;

	always @(negedge sd_clk)
	if (!i_en && !r_active)
	begin
		tx_sreg <= 0;
		r_count <= 0;
		r_crc   <= 0;
		ds      <= 0;
	end else if (i_valid && o_ready)
	begin // New data
		// {{{
		ds <= #FF_HOLD 1'b1;

		if (!r_active)
		begin // New data, plus a start bit
			// {{{
			if (i_width[0])
			begin // 4b width
				if (i_ddr)
					tx_sreg  <= #FF_HOLD { 4'b0, 4'bx, i_data, 8'hff };
				else
					tx_sreg  <= #FF_HOLD { 4'b0, i_data, 12'hfff };
				r_count  <= 9 + (i_ddr ? 1:0);
			end else if (i_width[1])
			begin // 8b width
				if (i_ddr)
					tx_sreg  <= #FF_HOLD { 8'b0, 8'bx, i_data };
				else
					tx_sreg  <= #FF_HOLD { 8'b0, i_data, 8'hff };
				r_count  <= 17 + (i_ddr ? 1:0);
			end else begin // 1b width
				if (i_ddr)
					tx_sreg  <= #FF_HOLD { 1'b0, 1'bx, i_data, 6'h3f, 8'hff };
				else
					tx_sreg  <= #FF_HOLD { 1'b0, i_data, 7'h7f, 8'hff };
				r_count  <= 33 + (i_ddr ? 1:0);
			end
			// }}}
		end else begin
			tx_sreg  <= #FF_HOLD { i_data, 16'hffff };
			r_count  <= (i_width[0]) ? 8 : (i_width[1]) ? 4 : 32;
		end
		r_active <= 1'b1;
		r_crc    <= 1'b0;
		// }}}
	end else if (r_active)
	begin
		ds <= #FF_HOLD 1'b1;

		r_count <= r_count - 1;
		if (i_width[0])
			tx_sreg <= #FF_HOLD { tx_sreg[43:0], 4'hf };
		else if (i_width[1])
			tx_sreg <= #FF_HOLD { tx_sreg[39:0], 8'hff };
		else
			tx_sreg <= #FF_HOLD { tx_sreg[46:0], 1'b1 };

		if (r_crc || (!r_crc && r_count <= 1))
		begin
			if (i_width[0])
				tx_sreg <= #FF_HOLD { crc[3][15],
					crc[2][15], crc[1][15], crc[0][15],
					44'hfff_ffff_ffff };
			else if (i_width[1])
				tx_sreg <= #FF_HOLD {
				crc[7][15], crc[6][15], crc[5][15], crc[4][15],
				crc[3][15], crc[2][15], crc[1][15], crc[0][15],
					40'hff_ffff_ffff };
			else
				tx_sreg <= #FF_HOLD { crc[0][15], 7'h7f,
					40'hff_ffff_ffff };
		end

		if (r_count <= 1)
		begin
			if (!r_crc)
			begin
				r_crc <= 1'b1;
				r_count <= 16;
			end else
				r_active <= 0;
		end
	end
	// }}}

	// Negative clock edge
	// {{{
	initial	r_active = 1'b0;
	always @(posedge sd_clk)
		ds <= #FF_HOLD 1'b0;

	always @(posedge sd_clk)
	if (i_ddr && r_active)
	begin
		r_count <= #FF_HOLD r_count - 1;
		if (i_width[0])
			tx_sreg <= #FF_HOLD { tx_sreg[43:0], 4'hf };
		else if (i_width[1])
			tx_sreg <= #FF_HOLD { tx_sreg[39:0], 8'hff };
		else
			tx_sreg <= #FF_HOLD { tx_sreg[46:0], 1'b1 };

		if (r_crc)
		begin
			if (i_width[0])
				tx_sreg <= #FF_HOLD { crc[11][15],
					crc[10][15], crc[9][15], crc[8][15],
					44'hfff_ffff_ffff };
			else if (i_width[1])
				tx_sreg <= #FF_HOLD {
				crc[15][15],crc[14][15],crc[13][15],crc[12][15],
				crc[11][15],crc[10][15],crc[ 9][15],crc[ 8][15],
					40'hff_ffff_ffff };
			else
				tx_sreg <= #FF_HOLD { crc[8][15], 7'h7f,
					40'hff_ffff_ffff };
		end

		if (r_count <= 1)
		begin
			if (!r_crc)
			begin
				r_crc <= #FF_HOLD 1'b1;
				r_count <= #FF_HOLD 16;
			end else
				r_active <= #FF_HOLD 0;
		end
	end
	// }}}

	assign	sd_dat[0] = !r_active ? 1'bz
				: (i_width[0]) ? tx_sreg[44]
				: (i_width[1]) ? tx_sreg[40] : tx_sreg[47];

	assign	sd_dat[3:1] = (!r_active || i_width==0) ? 3'bz
				: i_width[0] ? tx_sreg[47:45]
						: tx_sreg[43:41];
	assign	sd_dat[7:4] = (!r_active ||!i_width[1]) ? 4'bz
					: tx_sreg[47:44];
	assign	sd_ds = ds;

	assign	o_ready = !r_active || (sd_clk && r_count == 1)
						|| (!sd_clk && r_ready);

	// CRC generation
	// {{{
	generate for(gk=0; gk<8; gk=gk+1)
	begin : GEN_CRC
		reg	[15:0]	pedge_crc, nedge_crc;

		always @(posedge sd_clk)
		if (!i_en && !r_active)
			crc[gk] <= 0;
		else if (!r_crc)
			crc[gk] <= STEPCRC(crc[gk], sd_dat[gk]);
		else
			crc[gk] <= crc[gk] << 1;

		always @(negedge sd_clk)
		if (!i_ddr || (!i_en && !r_active) || !i_ddr)
			crc[8+gk] <= 0;
		else if (!r_crc)
			crc[8+gk] <= STEPCRC(crc[8+gk], sd_dat[gk]);
		else
			crc[8+gk] <= crc[8+gk] << 1;

		always @(*) pedge_crc = crc[  gk];
		always @(*) nedge_crc = crc[4+gk];

	end endgenerate
	// }}}

	function automatic [NCRC-1:0] STEPCRC(input [NCRC-1:0] prior,
		// {{{
				input i_bit);
	begin
		if (prior[NCRC-1] ^ i_bit)
			STEPCRC = { prior[NCRC-2:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			STEPCRC = { prior[NCRC-2:0], 1'b0 };
	end endfunction
	// }}}

	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ i_last };
	// Verilator lint_on  UNUSED
endmodule
