////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	xsdddr.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	An 2:1 OSERDES followed by an (optional) 1:2 ISERDES implemented
//		via the Xilinx ODDR and IDDR elements.  That simple, nothing
//	more.  This implementation is specific to Xilinx FPGAs.  It's designed,
//	however, so that there may be a minimum number of components that
//	need replacing when switching hardware platforms.
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
`default_nettype	none
`ifdef	VERILATOR
`define	OPENSIM
`endif
`ifdef	IVERILOG
`define	OPENSIM
`endif
// }}}
module	xsdddr #(
		parameter [0:0]		OPT_BIDIR = 1'b1
	) (
		// {{{
		input	wire		i_clk,
		//
		input	wire		i_en,
		input	wire	[1:0]	i_data,
		output	wire		io_pin_tristate,
		input	wire		i_pin,
		output	wire		o_pin,
		output	wire	[1:0]	o_wide
		// }}}
	);

	wire	w_in, w_out;
	reg	high_z;

	initial	high_z = OPT_BIDIR;
	always @(posedge i_clk)
		high_z <= !i_en && OPT_BIDIR;

`ifdef	OPENSIM
	// {{{
	reg	[1:0]	r_out;

	always @(posedge i_clk)
		r_out <= i_data;

	assign	w_out = (i_clk) ? r_out[1] : r_out[0];
	assign	io_pin_tristate = high_z;
	assign	o_pin = w_out;

	assign	w_in  = (high_z) ? i_pin : w_out;
	// }}}
`else
	ODDR #(
		// {{{
		.DDR_CLK_EDGE("SAME_EDGE"),
		.INIT(1'b1),
		.SRTYPE("SYNC")
		// }}}
	) u_oddr (
		// {{{
		.CE(1'b1), .R(1'b0), .S(1'b0),
		//
		.C(i_clk),
		.Q(w_out), .D1(i_data[1]), .D2(i_data[0])
		// }}}
	);

	assign	io_pin_tristate = high_z;
	assign	o_pin = w_out;
	assign	w_in  = i_pin;
`endif

	generate if (OPT_BIDIR)
	begin : GEN_BIDIRECTIONAL
		// {{{
`ifdef	OPENSIM
		reg		r_p, r_n;
		reg	[1:0]	r_in;

		always @(posedge i_clk)
			r_p <= w_in;
		always @(negedge i_clk)
			r_n <= w_in;
		always @(posedge i_clk)
			r_in <= { r_p, r_n };

		assign	o_wide = r_in;
`else
		IDDR #(
			.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
			.INIT_Q1(1'b1),
			.INIT_Q2(1'b1),
			.SRTYPE("SYNC")
		) u_iddr (
			.Q1(o_wide[1]), .Q2(o_wide[0]),
			.C(i_clk), .CE(1'b1), .D(i_pin),
			.R(1'b0), .S(1'b0)
		);
`endif
		// }}}
	end else begin : GEN_OUTPUT

		assign	o_wide = 2'b11;

		// Keep Verilator happy
		// {{{
		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused;
		assign	unused = &{ 1'b0,
`ifdef	VERILATOR
				i_pin,
`endif
				w_in };
		// Verilator lint_on  UNUSED
		// Verilator coverage_on
		// }}}
	end endgenerate
endmodule
