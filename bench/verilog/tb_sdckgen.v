////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	tb_sdckgen.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	An ad-hoc test of the clock generator.  Pass-fail is judged
//		by looking at the trace and not automatically.
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
`timescale 1ns/1ns
// }}}
module tb_sdckgen;
	// Local declarations
	// {{{
	reg	clk, reset;

	reg		cfg_clk90, cfg_shutdown;
	reg	[7:0]	cfg_ckspd;
	wire		w_ckstb, w_halfck;
	wire	[7:0]	w_ckwide;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Clock and reset generation
	// {{{
	initial begin
		$dumpfile("tb_sdckgen.vcd");
		$dumpvars(0,tb_sdckgen);
		reset = 1'b1;
		clk = 0;
		forever
			#5 clk = !clk;
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Test script
	// {{{
	task	capture_beats;
	begin
		repeat(5)
		begin
			wait(w_ckstb);
			@(posedge clk);
		end
	end endtask

	initial begin
		{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h0fc;
		repeat (5)
			@(posedge clk)
		@(posedge clk)
			reset <= 0;

		// 100kHz (10us)
		capture_beats;

		// 200 kHz (5us)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h07f;
		capture_beats;

		// 400 kHz (2.52us)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h041;
		capture_beats;

		//   1MHz (1us)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h01b;
		capture_beats;

		//   5MHz (200ns)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h007;
		capture_beats;

		//  12MHz (80ns)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h004;
		capture_beats;

		//  25MHz (40ns)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h003;
		capture_beats;

		//  50MHz (20ns)
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h002;
		capture_beats;

		// 100MHz
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h001;
		capture_beats;

		// 200MHz
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h000;
		capture_beats;


		//  25MHz, CLK90
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h103;
		capture_beats;

		//  25MHz, CLK90
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h102;
		capture_beats;

		// 100MHz, CLK90
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h101;
		capture_beats;

		// 200MHz, CLK90
		@(posedge clk)
			{ cfg_shutdown, cfg_clk90, cfg_ckspd } = 10'h100;
		capture_beats;

		$finish;
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Module under test
	// {{{

	sdckgen
	u_ckgen (
		// {{{
		.i_clk(clk), .i_reset(reset),
		//
		.i_cfg_clk90(cfg_clk90),
		.i_cfg_ckspd(cfg_ckspd),
		.i_cfg_shutdown(cfg_shutdown),
		//
		.o_ckstb(w_ckstb),
		.o_hlfck(w_halfck),
		.o_ckwide(w_ckwide)
		// }}}
	);
	// }}}
endmodule
