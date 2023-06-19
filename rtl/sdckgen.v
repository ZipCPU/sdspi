////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdckgen.v
// {{{
// Project:	SDIO SD-Card controller, using a shared SPI interface
//
// Purpose:	Generate a digitally divided pre-serdes clock.  The serdes this
//		will feed is an 8:1 serdes.  Hence, we generate 8 outputs per
//	clock period.
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
module	sdckgen #(
		// {{{
		// To hit 100kHz from a 100MHz clock, we'll need to divide by
		// 4, and then by another 250.  Hence, we'll need Lg(256)-2
		// bits.  (The first three are special)
		localparam	LGMAXDIV = 8
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		//
		input	wire			i_cfg_clk90,
		input	wire	[LGMAXDIV-1:0]	i_cfg_ckspd,
		input	wire			i_cfg_shutdown,
		//
		output	reg			o_ckstb,
		output	reg	[7:0]		o_ckwide
		// }}}
	);

	// Local declarations
	// {{{
	localparam		NCTR = LGMAXDIV+2;
	reg			nxt_stb, nxt_clk;
	reg	[NCTR-1:0]	nxt_counter, counter;
	reg			clk90;
	reg	[LGMAXDIV-1:0]	ckspd;
	wire			w_clk90;
	wire	[LGMAXDIV-1:0]	w_ckspd;
	// }}}

	// nxt_stb, nxt_clk, nxt_counter
	// {{{
	always @(*)
	begin
		nxt_stb = 1'b0;
		nxt_clk = 1'b0;
		nxt_counter = counter;

		{ nxt_stb, nxt_counter[NCTR-3:0] } = counter[NCTR-3:0] - 1;

		if (nxt_stb)
		begin
			// Advance the top two bits
			{ nxt_clk, nxt_counter[NCTR-1:NCTR-2] }
						= nxt_counter[NCTR-1:NCTR-2] +1;

			if (ckspd <= 1)
			begin
				nxt_clk = 1;
				nxt_counter[NCTR-3:0] = 0;
			end else if (ckspd == 2)
			begin
				nxt_clk = counter[NCTR-1];
				nxt_counter[NCTR-3:0] = 0;
			end else
				nxt_counter[NCTR-3:0] = ckspd-3;
		end

		if (nxt_clk)
		begin
			if (i_cfg_ckspd <= 1)
				nxt_counter = {2'b11, {(NCTR-2){1'b0}} };
			else if (i_cfg_ckspd == 2)
				nxt_counter = { 2'b01, {(NCTR-2){1'b0}} };
			else
				nxt_counter[NCTR-3:0] = i_cfg_ckspd-2;
		end
	end

	always @(posedge i_clk)
	if (i_reset)
		counter <= 0;
	else
		counter <= nxt_counter;
	// }}}

	// w_clk90, w_ckspd: Register the requested clock speed
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		clk90 <= 0;
	else
		clk90 <= w_clk90;

	always @(posedge i_clk)
	if (i_reset)
		ckspd <= 0;
	else
		ckspd <= w_ckspd;


	assign	w_clk90 = (nxt_clk) ? i_cfg_clk90 : clk90;
	assign	w_ckspd = (nxt_clk) ? i_cfg_ckspd : ckspd;
	// }}}

	// o_ckstb, o_ckwide
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		o_ckstb  <= 0;
		o_ckwide <= 0;
	end else if ((nxt_clk && i_cfg_shutdown) && (w_ckspd == 0))
	begin
		o_ckstb  <= 1'b1;	// Or should this be !i_cfg_shutdown?
		o_ckwide <= (i_cfg_shutdown) ? 8'h00 : 8'h66;
	end else if (w_ckspd == 1)
	begin
		o_ckstb  <= 1'b1;
		o_ckwide <= 8'h3c;
	end else if (w_ckspd == 2)
	begin
		o_ckstb  <= !nxt_counter[NCTR-1];
		if (w_clk90)
			o_ckwide <= (!nxt_counter[NCTR-1]) ? 8'h0f : 8'hf0;
		else
			o_ckwide <= (!nxt_counter[NCTR-1]) ? 8'h00 : 8'hff;
	end else begin
		o_ckstb <= nxt_clk;
		if (w_clk90)
			o_ckwide <= {(8){nxt_counter[NCTR-1]
						^ nxt_counter[NCTR-2]}};
		else
			o_ckwide <= {(8){nxt_counter[NCTR-1]}};
	end
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
`endif	// FORMAL
// }}}
endmodule

