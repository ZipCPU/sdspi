////////////////////////////////////////////////////////////////////////////////
//
// Filename:	xsddelay.txt
// {{{
// Project:	SD-Card controller
//
// Purpose:	Delays an incoming IO by a programmable amount.
//
//	As of this writing, the only implementation uses an AMD/Xilinx 7-series
//	IDELAYE2 element.  Other elements may be implemented in the future.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2026, Gisselquist Technology, LLC
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
`timescale 1ns / 1ps
`default_nettype none
//
// OPENSIM, if defined, will generate a simulatable look-alike to this path,
// to be used when using simulators that don't have simulatable models of the
// AMD/Xilinx IDELAYE2 element.  As of this writing, this offers only an
// approximate implementation that may (or may not) reflect reality.
`ifdef	VERILATOR
`define	OPENSIM
`endif
`ifdef	IVERILOG
`define	OPENSIM
`endif
// }}}
module xsddelay #(
		parameter	[0:0]	OPT_CLK = 0,
		// Verilator lint_off UNUSED
		parameter	[4:0]	DEF_DELAY = 0
		// Verilator lint_on  UNUSED
	) (
		// {{{
		input	wire		i_clk,		// 100MHz, system clk
		input	wire	[4:0]	i_delay,
		input	wire		i_pin,
		output	wire		o_delayed
		// }}}
	);


`ifdef	OPENSIM
	// {{{
	realtime	delay;
	reg		r_delayed;

	initial	delay = 0.0;
	always @(posedge i_clk)
		delay <= i_delay * 5.0 / 32;

	initial	r_delayed = !OPT_CLK;
	always @(i_pin)
		r_delayed <= #delay i_pin;

	assign	o_delayed = r_delayed;
	// }}}
`else
	// (* IODELAY_GROUP="some-200MHz-group" *)
	IDELAYE2 #(
		// {{{
		.REFCLK_FREQUENCY(200.0),
		.DELAY_SRC("IDATAIN"),
		.HIGH_PERFORMANCE_MODE("TRUE"),
		.IDELAY_TYPE("VAR_LOAD"),
		.SIGNAL_PATTERN(OPT_CLK ? "CLOCK" : "DATA"),
		.IDELAY_VALUE(DEF_DELAY),
		.PIPE_SEL("FALSE")
		// }}}
	) u_delay_emmc (
		// {{{
		.C(i_clk),
		.LD(1'b1),
		.CNTVALUEIN(i_delay),
		//
		.IDATAIN(i_pin),
		.DATAOUT(o_delayed),
		// Irrelevant / unused
		// {{{
		.CNTVALUEOUT(),		// Current delay feedback
		.CE(1'b0),		// Increment/decrement control
		.INC(1'b0),
		.CINVCTRL(1'b0),	// Clock inversion control
		.DATAIN(),		// Alternate input pin
		.LDPIPEEN(1'b0),
		.REGRST(1'b0)
		// }}}
		// }}}
	);
`endif

endmodule
