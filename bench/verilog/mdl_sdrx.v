////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	mdl_sdrx.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Model the receive half of the SDIO card.  This half will receive
//		data from the (FPGA) SD Controller, check it's CRCs, and write
//	the data back out to the main SDIO model.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2023, Gisselquist Technology, LLC
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
module	mdl_sdrx(
		// {{{
		input	wire		sd_clk,
		inout	wire	[7:0]	sd_dat,
		//
		input	wire		i_rx_en,	// Expect data
		input	wire	[1:0]	i_width,	// 0=1b,  1=4b, 2=8b
		input	wire		i_ddr,		// 0=SDR, 1=DDR
		input	wire	[15:0]	i_len,		// PktLen in 32b words
		//
		output	reg		o_valid,
		output	reg	[31:0]	o_data,
		output	reg		o_last,
		output	reg		o_good, o_err
		// }}}
	);

	// Local declarations
	// {{{
	genvar	gk;
	parameter		NCRC = 16;
	parameter [NCRC-1:0]	CRC_POLYNOMIAL  = 16'h1021;
	reg	[7:0]		rail_fail, eval_rail, half_fail, eval_half;
	reg			rx_started, rx_complete;
	reg	[15:0]		rail_count, max_rail_count, full_count;
	reg	[31:0]		rx_sreg;

	// }}}

	always @(*)
	begin
		if (i_width[0])
			max_rail_count = i_len * 2 + 16 - 1;
		else if (i_width[1])
			max_rail_count = i_len     + 16 - 1;
		else
			max_rail_count = i_len * 8 + 16 - 1;

		if (i_ddr)
			max_rail_count = max_rail_count + 16;
	end

	initial	rx_started = 0;
	always @(posedge sd_clk)
	if (!i_rx_en)
	begin
		rx_started  <= 1'b0;
		rx_complete <= 1'b0;
		rail_count  <= 0;
	end else if (rx_complete)
	begin
		// Do nothing
	end else if (rx_started)
	begin
		rail_count <= rail_count + 1;
		if (!i_ddr && rail_count >= max_rail_count)
			rx_complete <= 1'b1;
	end else if ((i_width[0] && sd_dat[3:0] == 0)
			|| (i_width[1] && sd_dat[7:0] == 1'b0)
			|| (i_width == 2'b0 && sd_dat[0] == 1'b0))
		rx_started <= 1'b1;

	always @(negedge sd_clk)
	if (!rx_started || rx_complete || !i_ddr)
	begin
		// Do nothing
	end else if (rx_started && rail_count[0])
	begin
		rail_count <= rail_count + 1;
		if (rail_count == max_rail_count)
			rx_complete <= 1'b1;
	end

	generate for(gk=0; gk<8; gk=gk+1)
	begin : GEN_CRC_CHECK
		reg		validpin;
		reg	[15:0]	crcfill, halffill;

		initial	rail_fail[gk] = 1'b0;
		initial	eval_rail[gk] = 1'b0;

		initial	validpin = 0;
		always @(posedge sd_clk)
		if (gk == 0 || (i_width[0] && gk<4) || i_width[1])
			validpin <= 1;
		else
			validpin <= 0;

		always @(posedge sd_clk)
		if (!i_rx_en || !rx_started || !validpin)
		begin
			crcfill <= 0;
			eval_rail[gk] <= 1'b0;
			rail_fail[gk] <= 1'b0;
		end else if (rx_started && !rx_complete)
		begin
			crcfill <= STEPCRC(crcfill, sd_dat[gk] !== 1'b0);
		end else if (rx_started && !eval_rail[gk]) // && rx_complete
		begin
			assert(crcfill == 0);
			eval_rail[gk] <= 1'b1;
			rail_fail[gk] <= (crcfill != 0) ||(sd_dat[gk] === 1'b0);
		end

		always @(negedge sd_clk)
		if (!i_rx_en || !rx_started || !validpin || !i_ddr)
		begin
			halffill <= 0;
			eval_half[gk] <= 1'b0;
			half_fail[gk] <= 1'b0;
		end else if (rx_started && !rx_complete)
		begin
			halffill <= STEPCRC(crcfill, sd_dat[gk] !== 1'b0);
			eval_half[gk] <= 1'b0;
			half_fail[gk] <= 1'b0;
		end else if (rx_started && !eval_half[gk]) // && rx_complete
		begin
			assert(halffill == 0);
			eval_half[gk] <= 1'b1;
			half_fail[gk] <= (halffill != 0) || sd_dat[gk] === 1'b0;
		end

	end endgenerate

	always @(posedge sd_clk)
	if (!i_rx_en)
		rx_sreg <= 0;
	else if (rx_started && !rx_complete)
	begin
		if (i_width[0])
		begin
			rx_sreg <= { rx_sreg[27:0],
				sd_dat[3] !== 1'b0,
				sd_dat[2] !== 1'b0,
				sd_dat[1] !== 1'b0,
				sd_dat[0] !== 1'b0 };
		end else if (i_width[1])
		begin
			rx_sreg <= { rx_sreg[23:0],
				sd_dat[7] !== 1'b0,
				sd_dat[6] !== 1'b0,
				sd_dat[5] !== 1'b0,
				sd_dat[4] !== 1'b0,
				sd_dat[3] !== 1'b0,
				sd_dat[2] !== 1'b0,
				sd_dat[1] !== 1'b0,
				sd_dat[0] !== 1'b0 };
		end else
			rx_sreg <= { rx_sreg[30:0], sd_dat[0] !== 1'b0 };
	end

	always @(negedge sd_clk)
	if (i_ddr && rx_started && !rx_complete)
	begin
		if (i_width[0])
		begin
			rx_sreg <= { rx_sreg[27:0],
				sd_dat[3] !== 1'b0,
				sd_dat[2] !== 1'b0,
				sd_dat[1] !== 1'b0,
				sd_dat[0] !== 1'b0 };
		end else if (i_width[1])
		begin
			rx_sreg <= { rx_sreg[23:0],
				sd_dat[7] !== 1'b0,
				sd_dat[6] !== 1'b0,
				sd_dat[5] !== 1'b0,
				sd_dat[4] !== 1'b0,
				sd_dat[3] !== 1'b0,
				sd_dat[2] !== 1'b0,
				sd_dat[1] !== 1'b0,
				sd_dat[0] !== 1'b0 };
		end else
			rx_sreg <= { rx_sreg[30:0], sd_dat[0] !== 1'b0};
	end

	always @(*)
	if (i_width[0])
		full_count = rail_count * 4;
	else if (i_width[1])
		full_count = rail_count * 8;
	else
		full_count = rail_count;

	initial	o_valid = 0;
	always @(posedge sd_clk)
	if (!i_rx_en)
		o_valid <= 1'b0;
	else if (full_count != 0 && full_count[4:0] == 0 && { 5'h0, full_count[15:3] } <= i_len)
		o_valid <= 1'b1;
	else
		o_valid <= 1'b0;

	always @(posedge sd_clk)
	if (!i_rx_en)
		o_data <= 32'b0;
	else if (full_count != 0 && full_count[4:0] == 0)
		o_data <= rx_sreg;

	initial	o_last = 0;
	always @(posedge sd_clk)
	if (!i_rx_en)
		o_last <= 1'b0;
	else if (full_count != 0 && full_count[4:0] == 0)
		o_last <= ({ 5'h0, full_count[15:3] } >= i_len);

	initial	{ o_good, o_err } = 0;
	always @(posedge sd_clk)
	if (!i_rx_en)
		{ o_good, o_err } <= 0;
	else if ( ((i_width[0] && (&eval_rail[3:0]))
				|| (i_width[1] && (&eval_rail))
				|| (i_width == 0 && eval_rail[0]))
		&& (!i_ddr || (i_ddr &&
				((i_width[0] && (&eval_half[3:0]))
				|| (i_width[1] && (&eval_half))
				|| (i_width == 0 && eval_half[0])))))
	begin
		o_good <= (rail_fail == 0 && half_fail == 0);
		o_err  <= (rail_fail != 0 || half_fail != 0);
	end

	always @(*)
	if (i_rx_en && rx_complete)
		assert(!o_err);

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
endmodule
