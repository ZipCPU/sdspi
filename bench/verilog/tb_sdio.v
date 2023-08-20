////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	tb_sdio.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	
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
`timescale 1ns / 1ps
// }}}
module	tb_sdio;
	// Local declarations
	// {{{
	parameter	[1:0]	OPT_SERDES = 1'b1;
	parameter	[1:0]	OPT_DDR = 1'b1;
	parameter	[0:0]	OPT_VCD = 1'b0;
	localparam		AW = 3, DW=32;
	localparam		VCD_FILE = "trace.vcd";

	reg	[2:0]		ckcounter;
	wire			clk, hsclk;
	reg			reset;

	wire			bfm_cyc, bfm_stb, bfm_we,
				bfm_stall, bfm_ack, bfm_err;
	wire	[AW-1:0]	bfm_addr;
	wire	[DW-1:0]	bfm_data, bfm_idata;
	wire	[DW/8-1:0]	bfm_sel;

	wire			sd_cmd, sd_ck;
	wire	[3:0]		sd_dat;
	wire			interrupt;
	wire	[31:0]		scope_debug;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Clock/reset generation
	// {{{
	localparam	realtime CLK_PERIOD = 10.0;	// 100MHz

	initial	begin
		ckcounter = 0;
		forever
			#(CLK_PERIOD/8) ckcounter = ckcounter + 1;
	end

	assign	hsclk = ckcounter[0];
	assign	clk   = ckcounter[2];

	initial	begin
		reset <= 0;
		@(posedge hsclk)
			reset <= 1;
		@(posedge clk);
		@(posedge clk);
		@(posedge clk)
			reset <= 0;
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Unit under test
	// {{{
	sdio_top #(
		// {{{
		.LGFIFO(12), .NUMIO(4), .MW(32),
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR),
		.OPT_CARD_DETECT(0), .LGTIMEOUT(10),
		.OPT_EMMC(1'b0)
		// }}}
	) u_sdio (
		// {{{
		.i_clk(clk), .i_reset(reset), .i_hsclk(hsclk),
		//
		.i_wb_cyc(bfm_cyc), .i_wb_stb(bfm_stb), .i_wb_we(bfm_we),
		.i_wb_addr(bfm_addr), .i_wb_data(bfm_data),.i_wb_sel(bfm_sel),
		.o_wb_stall(bfm_stall),.o_wb_ack(bfm_ack),.o_wb_data(bfm_idata),
		//
		.o_ck(sd_ck), .i_ds(1'b0), .io_cmd(sd_cmd), .io_dat(sd_dat),
		.i_card_detect(1'b1), .o_int(interrupt), .o_debug(scope_debug)
		// }}}
	);

	assign	bfm_err = 1'b0;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone bus functional model
	// {{{

	wb_bfm #(
		.AW(AW), .DW(DW), .LGFIFO(4)
	) u_bfm (
		.i_clk(clk), .i_reset(reset),
		//
		.o_wb_cyc(bfm_cyc), .o_wb_stb(bfm_stb), .o_wb_we(bfm_we),
		.o_wb_addr(bfm_addr), .o_wb_data(bfm_data), .o_wb_sel(bfm_sel),
		.i_wb_stall(bfm_stall), .i_wb_ack(bfm_ack),
			.i_wb_data(bfm_idata), .i_wb_err(bfm_err)
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDIO Device model
	// {{{

	mdl_sdio
	u_sdcard (
		.sd_clk(sd_ck), .sd_cmd(sd_cmd), .sd_dat(sd_dat)
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// VCD generation
	// {{{

	initial if (OPT_VCD && VCD_FILE != 0)
	begin
		$dumpfile(VCD_FILE);
		$dumpvars(0, tb_sdio);
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Test script
	// {{{
	reg	error_flag;
`include	`SCRIPT

	initial begin
		error_flag = 1'b0;
		@(posedge clk);
		wait(!reset);
		@(posedge clk);
		testscript;

		if (error_flag)
		begin
			$display("TEST FAIL!");
		end else begin
			$display("Test pass");
		end

		$finish;
	end
	// }}}
endmodule
