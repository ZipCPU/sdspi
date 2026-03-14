////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdtfrvalue.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Illustrates a slow method of moving data across clock domains,
//		together with a formal proof of the same.  This is the "faster"
//	version of two methods, the second one given in tfrslow.v.  This one
//	is faster because it requires only a single round trip of the request
//	and the acknowledgement.
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
`default_nettype	none
`timescale 1ns/1ps
// }}}
module sdtfrvalue #(
		parameter		W = 32,
		parameter [W-1:0]	DEFAULT = 0
	) (
		// {{{
		input	wire		i_a_clk,
		input	wire		i_a_reset_n,
		input	wire		i_a_valid,
		output	wire		o_a_ready,
		input	wire	[W-1:0]	i_a_data,
		//
		input	wire		i_b_clk,
		input	wire		i_b_reset_n,
		output	reg		o_b_valid,
		input	wire		i_b_ready,
		output	reg	[W-1:0]	o_b_data
		// }}}
	);

	// Register declarations
	// {{{
	localparam	NFF = 2;
	reg			a_ack;
	reg	[W-1:0]		a_data;
	(* ASYNC_REG *) reg	a_req;
	(* ASYNC_REG *) reg	[NFF-2:0]	a_pipe;

	reg			b_last, b_stb;
	(* ASYNC_REG *) reg	b_req;
	(* ASYNC_REG *) reg	[NFF-2:0]	b_pipe;
	// }}}

	// Launch
	// {{{
	initial	a_req = 0;
	always @(posedge i_a_clk, negedge i_a_reset_n)
	if (!i_a_reset_n)
		a_req <= 1'b0;
	else if (i_a_valid && o_a_ready)
		a_req  <= !a_req;

	initial	a_data = DEFAULT;
	always @(posedge i_a_clk)
	if (i_a_valid && o_a_ready)
		a_data <= i_a_data;
	// }}}

	// Request to B
	// {{{
	initial	{ b_last, b_req, b_pipe } = 0;
	always @(posedge i_b_clk, negedge i_b_reset_n)
	if (!i_b_reset_n)
		{ b_last, b_req, b_pipe } <= 0;
	else begin
		{ b_last, b_req, b_pipe } <= { b_req, b_pipe, a_req };
		if (o_b_valid && !i_b_ready)
			b_last <= b_last;
	end
	// }}}

	// Return ACK
	// {{{
	always @(posedge i_a_clk, negedge i_a_reset_n)
	if (!i_a_reset_n)
		{ a_ack, a_pipe } <= 0;
	else
		{ a_ack, a_pipe } <= { a_pipe, b_last };
	// }}}

	// Return ready
	assign	o_a_ready = (a_ack == a_req);

	// Outgoing strobe and data
	// {{{
	always @(*)
		b_stb = (b_last != b_req);

	initial	o_b_data = DEFAULT;
	always @(posedge i_b_clk)
	if (b_stb && (!o_b_valid || i_b_ready))
		o_b_data <= a_data;

	initial	o_b_valid = 0;
	always @(posedge i_b_clk, negedge i_b_reset_n)
	if (!i_b_reset_n)
		o_b_valid <= 1'b0;
	else if (!o_b_valid || i_b_ready)
		o_b_valid <= b_stb;
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
	// Define our registers, and f_past_valid_*
	// {{{
	// Verilator lint_off UNDRIVEN
	(* gclk *) reg gbl_clk;
	// Verilator lint_on  UNDRIVEN
			reg		f_past_valid_gbl, f_past_valid_a,
					f_past_valid_b;
	localparam	LGCLK = 5;
	// Verilator lint_off UNDRIVEN
	(* anyconst *)	reg	[LGCLK-2:0]	f_step_a,  f_step_b;
	// Verilator lint_on  UNDRIVEN
			reg	[LGCLK-1:0]	f_count_a, f_count_b;

	initial	f_past_valid_gbl = 0;
	always @(posedge gbl_clk)
		f_past_valid_gbl <= 1;

	initial	f_past_valid_a = 0;
	always @(posedge i_a_clk)
		f_past_valid_a <= 1;

	initial	f_past_valid_b = 0;
	always @(posedge i_b_clk)
		f_past_valid_b <= 1;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Assume two clocks
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge gbl_clk)
	begin
		f_count_a <= f_count_a + { 1'b0, f_step_a } + 1;
		f_count_b <= f_count_b + { 1'b0, f_step_b } + 1;
	end

	always @(*)
		assume(i_a_clk == f_count_a[LGCLK-1]);

	always @(*)
		assume(i_b_clk == f_count_b[LGCLK-1]);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Reset assumptions
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Start in reset
	always @(*)
	if (!f_past_valid_gbl)
		assume(!i_a_reset_n && !i_b_reset_n);

	// Both resets will always fall together
	always @(posedge gbl_clk)
		assume($fell(i_b_reset_n) == $fell(i_a_reset_n));

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Stability properties
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	//
	// On clock A
	// {{{
	always @(posedge gbl_clk)
	if (!$rose(i_a_clk))
	begin
		assume(!$rose(i_a_reset_n));
		assume($stable(i_a_valid));
		assume($stable(i_a_data));
		if (i_a_reset_n)
			assert($stable(o_a_ready));
	end
	// }}}

	//
	// On clock B
	// {{{
	always @(posedge gbl_clk)
	if (!$rose(i_b_clk))
	begin
		assume(!$rose(i_b_reset_n));
		if (i_b_reset_n)
			assume($stable(i_b_ready));
		if (f_past_valid_b && i_b_reset_n && $stable(i_b_reset_n))
		begin
			assert($stable(o_b_valid));
			assert($stable(o_b_data));
		end
	end
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// AXI Stream properties
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_a_clk)
	if (!f_past_valid_a || !i_a_reset_n)
	begin
		assume(!i_a_valid);
	end else if ($past(i_a_valid && !o_a_ready))
	begin
		assume(i_a_valid);
		assume($stable(i_a_data));
	end

	always @(posedge i_b_clk)
	if (!f_past_valid_b || !i_b_reset_n)
	begin
		assert(!o_b_valid);
	end else if ($past(o_b_valid && !i_b_ready))
	begin
		assert(o_b_valid);
		assert($stable(o_b_data));
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Induction
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge gbl_clk)
	case({ a_ack, a_pipe, b_last, b_req, b_pipe, a_req })
	6'b000_000: begin end
	6'b000_001: begin end
	6'b000_011: begin end
	6'b000_111: begin end
	6'b001_111: begin end
	6'b011_111: begin end
	6'b111_111: begin end
	6'b111_110: begin end
	6'b111_100: begin end
	6'b111_000: begin end
	6'b110_000: begin end
	6'b100_000: begin end
	default: assert(0);
	endcase

	always @(posedge i_b_clk)
	if ( (!(&{b_req, b_pipe})) && ({b_req, b_pipe} != 0) )
		assert($stable(a_data));
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Cover checks
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	reg	[3:0]	cvr_stbcount;

	initial	cvr_stbcount = 0;
	always @(posedge i_b_clk, negedge i_b_reset_n)
	if (!i_b_reset_n)
		cvr_stbcount <= 0;
	else if (o_b_valid && i_b_ready && (o_b_data[3:0] == cvr_stbcount))
		cvr_stbcount <= cvr_stbcount + 1;

	always @(*)
	begin
		cover(cvr_stbcount == 1);
		cover(cvr_stbcount == 2);
		cover(cvr_stbcount == 3);
		cover(cvr_stbcount == 4);
		cover(cvr_stbcount == 5);
		//
		// These next two take too much CPU time, so we don't insist
		// upon them here.
		// cover(cvr_stbcount == 6);
		// cover(cvr_stbcount[3]);
	end
	// }}}
`endif
// }}}
endmodule
