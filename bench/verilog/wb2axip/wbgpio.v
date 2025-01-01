////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/wb2axip/wbgpio.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	This extremely simple GPIO controller, although minimally
//		featured, is designed to control up to sixteen general purpose
//	input and sixteen general purpose output lines of a module from a
//	single address on a 32-bit wishbone bus.
//
//	Input GPIO values are contained in the top 16-bits.  Any change in
//	input values will generate an interrupt.
//
//	Output GPIO values are contained in the bottom 16-bits.  To change an
//	output GPIO value, writes to this port must also set a bit in the
//	upper sixteen bits.  Hence, to set GPIO output zero, one would write
//	a 0x010001 to the port, whereas a 0x010000 would clear the bit.  This
//	interface makes it possible to change only the bit of interest, without
//	needing to capture and maintain the prior bit values--something that
//	might be difficult from a interrupt context within a CPU.
//
//	Unlike other controllers, this controller offers no capability to
//	change input/output direction, or to implement pull-up or pull-down
//	resistors.  It simply changes and adjusts the values going out the
//	output pins, while allowing a user to read the values on the input
//	pins.
//
//	Any change of an input pin value will result in the generation of an
//	interrupt signal.
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
`timescale		1ns/1ps
`default_nettype	none
// }}}
module wbgpio #(
		// {{{
		parameter		NIN=16, NOUT=16,
		parameter [(NOUT-1):0]	DEFAULT=0
		// }}}
	) (
		// {{{
		input	wire		i_clk,
		//
		input	wire			i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[31:0]		i_wb_data,
		input	wire	[32/8-1:0]	i_wb_sel,
		//
		output	wire		o_wb_stall,
		output	wire		o_wb_ack,
		output	wire	[31:0]	o_wb_data,
		//
		input	wire	[(NIN-1):0]	i_gpio,
		output	reg	[(NOUT-1):0]	o_gpio,
		//
		output	reg		o_int
		// }}}
	);

	// Local declarations
	// {{{
	reg	[(NIN-1):0]	r_gpio;
	(* ASYNC_REG *)	reg	[(NIN-1):0]	x_gpio, q_gpio;
	wire	[15:0]	hi_bits, low_bits;
	// }}}

	assign	o_wb_ack   = i_wb_stb;
	assign	o_wb_stall = 1'b0;

	// o_gpio
	// {{{
	// 9LUT's, 16 FF's
	initial	o_gpio = DEFAULT;
	always @(posedge i_clk)
	if (i_wb_stb && i_wb_we && (&i_wb_sel))
		o_gpio <= ((o_gpio)&(~i_wb_data[(NOUT+16-1):16]))
			|((i_wb_data[(NOUT-1):0])&(i_wb_data[(NOUT+16-1):16]));
	// }}}

	// 3 LUTs, 33 FF's
	// {{{
	always @(posedge i_clk)
	begin
		{ r_gpio, q_gpio, x_gpio } <= { q_gpio, x_gpio, i_gpio };
		o_int  <= (q_gpio != r_gpio);
	end
	// }}}

	// o_wb_data
	// {{{
	assign	hi_bits[ (NIN -1):0] = r_gpio;
	assign	low_bits[(NOUT-1):0] = o_gpio;

	generate
	if (NIN < 16)
	begin : GEN_HIBITS
		assign hi_bits[ 15: NIN] = 0;
	end
	if (NOUT < 16)
	begin : GEN_LOBITS
		assign low_bits[15:NOUT] = 0;
	end endgenerate

	assign	o_wb_data = { hi_bits, low_bits };
	// }}}

	// Make Verilator happy
	// {{{
	// verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_wb_cyc, i_wb_data[31:0], i_wb_sel };
	// verilator lint_on  UNUSED
	// }}}
endmodule
