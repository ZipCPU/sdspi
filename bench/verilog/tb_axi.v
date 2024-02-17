////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/tb_axi.v
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
// Copyright (C) 2016-2024, Gisselquist Technology, LLC
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
`timescale 1ns / 1ps
// }}}
module	tb_axi;
	// Local declarations
	// {{{
// `define	SDIO_AXI
	parameter	[1:0]	OPT_SERDES = 1'b1;
	parameter	[1:0]	OPT_DDR = 1'b1;
	parameter	[0:0]	OPT_DMA = 1'b0;
	parameter	[0:0]	OPT_VCD = 1'b0;
	parameter		DW = 64;
	parameter		AXI_IW = 2;
	localparam		BFM_DW=32;
	localparam		VCD_FILE = "axtrace.vcd";
	parameter		LGMEMSZ = 16;	// 64kB
	localparam		ADDRESS_WIDTH = LGMEMSZ + 1;

	localparam	AXILSB = $clog2(DW/8);
	localparam	AW = ADDRESS_WIDTH,
			BFM_AW = AW;

	parameter	MEM_ADDR = { 1'b1, {(AW-1){1'b0}} },
			SDIO_ADDR = { 3'b001,{(AW-3){1'b0}} },
			EMMC_ADDR = { 3'b010,{(AW-3){1'b0}} };
	//
	parameter	MEM_MASK = { 1'b1, {(AW-1){1'b0}} },
			SDIO_MASK = { 3'b111,{(AW-8){1'b1}}, {(5){1'b0}} },
			EMMC_MASK = { 3'b111,{(AW-8){1'b1}}, {(5){1'b0}} };

	reg	[2:0]		ckcounter;
	wire			clk, hsclk;
	reg			reset;

	// BFM_AXIL_**
	// {{{
	wire			BFM_AXIL_AWVALID, BFM_AXIL_AWREADY;
	wire	[AW-1:0]	BFM_AXIL_AWADDR;
	wire	[2:0]		BFM_AXIL_AWPROT;

	wire			BFM_AXIL_WVALID, BFM_AXIL_WREADY;
	wire	[32-1:0]	BFM_AXIL_WDATA;
	wire	[32/8-1:0]	BFM_AXIL_WSTRB;

	wire			BFM_AXIL_BVALID, BFM_AXIL_BREADY;
	wire	[1:0]		BFM_AXIL_BRESP;

	wire			BFM_AXIL_ARVALID, BFM_AXIL_ARREADY;
	wire	[AW-1:0]	BFM_AXIL_ARADDR;
	wire	[2:0]		BFM_AXIL_ARPROT;

	wire			BFM_AXIL_RVALID, BFM_AXIL_RREADY;
	wire	[32-1:0]	BFM_AXIL_RDATA;
	wire	[1:0]		BFM_AXIL_RRESP;
	// }}}

	// BFMW_AXIL_*
	// {{{
	wire			BFMW_AXIL_AWVALID, BFMW_AXIL_AWREADY;
	wire	[AW-1:0]	BFMW_AXIL_AWADDR;
	wire	[2:0]		BFMW_AXIL_AWPROT;

	wire			BFMW_AXIL_WVALID, BFMW_AXIL_WREADY;
	wire	[DW-1:0]	BFMW_AXIL_WDATA;
	wire	[DW/8-1:0]	BFMW_AXIL_WSTRB;

	wire			BFMW_AXIL_BVALID, BFMW_AXIL_BREADY;
	wire	[1:0]		BFMW_AXIL_BRESP;

	wire			BFMW_AXIL_ARVALID, BFMW_AXIL_ARREADY;
	wire	[AW-1:0]	BFMW_AXIL_ARADDR;
	wire	[2:0]		BFMW_AXIL_ARPROT;

	wire			BFMW_AXIL_RVALID, BFMW_AXIL_RREADY;
	wire	[DW-1:0]	BFMW_AXIL_RDATA;
	wire	[1:0]		BFMW_AXIL_RRESP;
	// }}}

	// BFM_AXI_*
	// {{{
	wire			BFM_AXI_AWVALID, BFM_AXI_AWREADY;
	wire	[AXI_IW-1:0]	BFM_AXI_AWID;
	wire	[AW-1:0]	BFM_AXI_AWADDR;
	wire	[7:0]		BFM_AXI_AWLEN;
	wire	[2:0]		BFM_AXI_AWSIZE, wrong_BFM_AXI_AWSIZE;
	wire	[1:0]		BFM_AXI_AWBURST;
	wire			BFM_AXI_AWLOCK;
	wire	[3:0]		BFM_AXI_AWCACHE;
	wire	[2:0]		BFM_AXI_AWPROT;
	wire	[3:0]		BFM_AXI_AWQOS;

	wire			BFM_AXI_WVALID, BFM_AXI_WREADY;
	wire	[DW-1:0]	BFM_AXI_WDATA;
	wire	[DW/8-1:0]	BFM_AXI_WSTRB;
	wire			BFM_AXI_WLAST;

	wire			BFM_AXI_BVALID, BFM_AXI_BREADY;
	wire	[AXI_IW-1:0]	BFM_AXI_BID;
	wire	[1:0]		BFM_AXI_BRESP;

	wire			BFM_AXI_ARVALID, BFM_AXI_ARREADY;
	wire	[AXI_IW-1:0]	BFM_AXI_ARID;
	wire	[AW-1:0]	BFM_AXI_ARADDR;
	wire	[7:0]		BFM_AXI_ARLEN;
	wire	[2:0]		BFM_AXI_ARSIZE, wrong_BFM_AXI_ARSIZE;
	wire	[1:0]		BFM_AXI_ARBURST;
	wire			BFM_AXI_ARLOCK;
	wire	[3:0]		BFM_AXI_ARCACHE;
	wire	[2:0]		BFM_AXI_ARPROT;
	wire	[3:0]		BFM_AXI_ARQOS;

	wire			BFM_AXI_RVALID, BFM_AXI_RREADY;
	wire	[AXI_IW-1:0]	BFM_AXI_RID;
	wire	[DW-1:0]	BFM_AXI_RDATA;
	wire			BFM_AXI_RLAST;
	wire	[1:0]		BFM_AXI_RRESP;
	// }}}

	// SDIO_AXI_*
	// {{{
	wire			SDIO_AXI_AWVALID, SDIO_AXI_AWREADY;
	wire	[AXI_IW-1:0]	SDIO_AXI_AWID;
	wire	[AW-1:0]	SDIO_AXI_AWADDR;
	wire	[7:0]		SDIO_AXI_AWLEN;
	wire	[2:0]		SDIO_AXI_AWSIZE;
	wire	[1:0]		SDIO_AXI_AWBURST;
	wire			SDIO_AXI_AWLOCK;
	wire	[3:0]		SDIO_AXI_AWCACHE;
	wire	[2:0]		SDIO_AXI_AWPROT;
	wire	[3:0]		SDIO_AXI_AWQOS;

	wire			SDIO_AXI_WVALID, SDIO_AXI_WREADY;
	wire	[DW-1:0]	SDIO_AXI_WDATA;
	wire	[DW/8-1:0]	SDIO_AXI_WSTRB;
	wire			SDIO_AXI_WLAST;

	wire			SDIO_AXI_BVALID, SDIO_AXI_BREADY;
	wire	[AXI_IW-1:0]	SDIO_AXI_BID;
	wire	[1:0]		SDIO_AXI_BRESP;

	wire			SDIO_AXI_ARVALID, SDIO_AXI_ARREADY;
	wire	[AXI_IW-1:0]	SDIO_AXI_ARID;
	wire	[AW-1:0]	SDIO_AXI_ARADDR;
	wire	[7:0]		SDIO_AXI_ARLEN;
	wire	[2:0]		SDIO_AXI_ARSIZE;
	wire	[1:0]		SDIO_AXI_ARBURST;
	wire			SDIO_AXI_ARLOCK;
	wire	[3:0]		SDIO_AXI_ARCACHE;
	wire	[2:0]		SDIO_AXI_ARPROT;
	wire	[3:0]		SDIO_AXI_ARQOS;

	wire			SDIO_AXI_RVALID, SDIO_AXI_RREADY;
	wire	[AXI_IW-1:0]	SDIO_AXI_RID;
	wire	[DW-1:0]	SDIO_AXI_RDATA;
	wire			SDIO_AXI_RLAST;
	wire	[1:0]		SDIO_AXI_RRESP;
	// }}}

	// SIDO_DMA_*
	// {{{
	wire			SDIO_DMA_AWVALID, SDIO_DMA_AWREADY;
	wire	[AXI_IW-1:0]	SDIO_DMA_AWID;
	wire	[AW-1:0]	SDIO_DMA_AWADDR;
	wire	[7:0]		SDIO_DMA_AWLEN;
	wire	[2:0]		SDIO_DMA_AWSIZE;
	wire	[1:0]		SDIO_DMA_AWBURST;
	wire			SDIO_DMA_AWLOCK;
	wire	[3:0]		SDIO_DMA_AWCACHE;
	wire	[2:0]		SDIO_DMA_AWPROT;
	wire	[3:0]		SDIO_DMA_AWQOS;

	wire			SDIO_DMA_WVALID, SDIO_DMA_WREADY;
	wire	[DW-1:0]	SDIO_DMA_WDATA;
	wire	[DW/8-1:0]	SDIO_DMA_WSTRB;
	wire			SDIO_DMA_WLAST;

	wire			SDIO_DMA_BVALID, SDIO_DMA_BREADY;
	wire	[AXI_IW-1:0]	SDIO_DMA_BID;
	wire	[1:0]		SDIO_DMA_BRESP;

	wire			SDIO_DMA_ARVALID, SDIO_DMA_ARREADY;
	wire	[AXI_IW-1:0]	SDIO_DMA_ARID;
	wire	[AW-1:0]	SDIO_DMA_ARADDR;
	wire	[7:0]		SDIO_DMA_ARLEN;
	wire	[2:0]		SDIO_DMA_ARSIZE;
	wire	[1:0]		SDIO_DMA_ARBURST;
	wire			SDIO_DMA_ARLOCK;
	wire	[3:0]		SDIO_DMA_ARCACHE;
	wire	[2:0]		SDIO_DMA_ARPROT;
	wire	[3:0]		SDIO_DMA_ARQOS;

	wire			SDIO_DMA_RVALID, SDIO_DMA_RREADY;
	wire	[AXI_IW-1:0]	SDIO_DMA_RID;
	wire	[DW-1:0]	SDIO_DMA_RDATA;
	wire			SDIO_DMA_RLAST;
	wire	[1:0]		SDIO_DMA_RRESP;
	// }}}

	// SDIO_*
	// {{{
	wire			SDIO_AWVALID, SDIO_AWREADY;
	wire	[AW-1:0]	SDIO_AWADDR;
	wire	[2:0]		SDIO_AWPROT;

	wire			SDIO_WVALID, SDIO_WREADY;
	wire	[32-1:0]	SDIO_WDATA;
	wire	[32/8-1:0]	SDIO_WSTRB;

	wire			SDIO_BVALID, SDIO_BREADY;
	wire	[1:0]		SDIO_BRESP;

	wire			SDIO_ARVALID, SDIO_ARREADY;
	wire	[AW-1:0]	SDIO_ARADDR;
	wire	[2:0]		SDIO_ARPROT;

	wire			SDIO_RVALID, SDIO_RREADY;
	wire	[32-1:0]	SDIO_RDATA;
	wire	[1:0]		SDIO_RRESP;
	// }}}

	// EMMC_AXI_*
	// {{{
	wire			EMMC_AXI_AWVALID, EMMC_AXI_AWREADY;
	wire	[AXI_IW-1:0]	EMMC_AXI_AWID;
	wire	[AW-1:0]	EMMC_AXI_AWADDR;
	wire	[7:0]		EMMC_AXI_AWLEN;
	wire	[2:0]		EMMC_AXI_AWSIZE;
	wire	[1:0]		EMMC_AXI_AWBURST;
	wire			EMMC_AXI_AWLOCK;
	wire	[3:0]		EMMC_AXI_AWCACHE;
	wire	[2:0]		EMMC_AXI_AWPROT;
	wire	[3:0]		EMMC_AXI_AWQOS;

	wire			EMMC_AXI_WVALID, EMMC_AXI_WREADY;
	wire	[DW-1:0]	EMMC_AXI_WDATA;
	wire	[DW/8-1:0]	EMMC_AXI_WSTRB;
	wire			EMMC_AXI_WLAST;

	wire			EMMC_AXI_BVALID, EMMC_AXI_BREADY;
	wire	[AXI_IW-1:0]	EMMC_AXI_BID;
	wire	[1:0]		EMMC_AXI_BRESP;

	wire			EMMC_AXI_ARVALID, EMMC_AXI_ARREADY;
	wire	[AXI_IW-1:0]	EMMC_AXI_ARID;
	wire	[AW-1:0]	EMMC_AXI_ARADDR;
	wire	[7:0]		EMMC_AXI_ARLEN;
	wire	[2:0]		EMMC_AXI_ARSIZE;
	wire	[1:0]		EMMC_AXI_ARBURST;
	wire			EMMC_AXI_ARLOCK;
	wire	[3:0]		EMMC_AXI_ARCACHE;
	wire	[2:0]		EMMC_AXI_ARPROT;
	wire	[3:0]		EMMC_AXI_ARQOS;

	wire			EMMC_AXI_RVALID, EMMC_AXI_RREADY;
	wire	[AXI_IW-1:0]	EMMC_AXI_RID;
	wire	[DW-1:0]	EMMC_AXI_RDATA;
	wire			EMMC_AXI_RLAST;
	wire	[1:0]		EMMC_AXI_RRESP;
	// }}}

	// EMMC_DMA_*
	// {{{
	wire			EMMC_DMA_AWVALID, EMMC_DMA_AWREADY;
	wire	[AXI_IW-1:0]	EMMC_DMA_AWID;
	wire	[AW-1:0]	EMMC_DMA_AWADDR;
	wire	[7:0]		EMMC_DMA_AWLEN;
	wire	[2:0]		EMMC_DMA_AWSIZE;
	wire	[1:0]		EMMC_DMA_AWBURST;
	wire			EMMC_DMA_AWLOCK;
	wire	[3:0]		EMMC_DMA_AWCACHE;
	wire	[2:0]		EMMC_DMA_AWPROT;
	wire	[3:0]		EMMC_DMA_AWQOS;

	wire			EMMC_DMA_WVALID, EMMC_DMA_WREADY;
	wire	[DW-1:0]	EMMC_DMA_WDATA;
	wire	[DW/8-1:0]	EMMC_DMA_WSTRB;
	wire			EMMC_DMA_WLAST;

	wire			EMMC_DMA_BVALID, EMMC_DMA_BREADY;
	wire	[AXI_IW-1:0]	EMMC_DMA_BID;
	wire	[1:0]		EMMC_DMA_BRESP;

	wire			EMMC_DMA_ARVALID, EMMC_DMA_ARREADY;
	wire	[AXI_IW-1:0]	EMMC_DMA_ARID;
	wire	[AW-1:0]	EMMC_DMA_ARADDR;
	wire	[7:0]		EMMC_DMA_ARLEN;
	wire	[2:0]		EMMC_DMA_ARSIZE;
	wire	[1:0]		EMMC_DMA_ARBURST;
	wire			EMMC_DMA_ARLOCK;
	wire	[3:0]		EMMC_DMA_ARCACHE;
	wire	[2:0]		EMMC_DMA_ARPROT;
	wire	[3:0]		EMMC_DMA_ARQOS;

	wire			EMMC_DMA_RVALID, EMMC_DMA_RREADY;
	wire	[AXI_IW-1:0]	EMMC_DMA_RID;
	wire	[DW-1:0]	EMMC_DMA_RDATA;
	wire			EMMC_DMA_RLAST;
	wire	[1:0]		EMMC_DMA_RRESP;
	// }}}

	// EMMC_*
	// {{{
	wire			EMMC_AWVALID, EMMC_AWREADY;
	wire	[AW-1:0]	EMMC_AWADDR;
	wire	[2:0]		EMMC_AWPROT;

	wire			EMMC_WVALID, EMMC_WREADY;
	wire	[32-1:0]	EMMC_WDATA;
	wire	[32/8-1:0]	EMMC_WSTRB;

	wire			EMMC_BVALID, EMMC_BREADY;
	wire	[1:0]		EMMC_BRESP;

	wire			EMMC_ARVALID, EMMC_ARREADY;
	wire	[AW-1:0]	EMMC_ARADDR;
	wire	[2:0]		EMMC_ARPROT;

	wire			EMMC_RVALID, EMMC_RREADY;
	wire	[32-1:0]	EMMC_RDATA;
	wire	[1:0]		EMMC_RRESP;
	// }}}

	// RAM model declarations
	// {{{
	wire			MEM_AWVALID, MEM_AWREADY;
	wire	[AXI_IW-1:0]	MEM_AWID;
	wire	[AW-1:0]	MEM_AWADDR;
	wire	[7:0]		MEM_AWLEN;
	wire	[2:0]		MEM_AWSIZE;
	wire	[1:0]		MEM_AWBURST;
	wire			MEM_AWLOCK;
	wire	[3:0]		MEM_AWCACHE;
	wire	[2:0]		MEM_AWPROT;
	wire	[3:0]		MEM_AWQOS;

	wire			MEM_WVALID, MEM_WREADY;
	wire	[DW-1:0]	MEM_WDATA;
	wire	[DW/8-1:0]	MEM_WSTRB;
	wire			MEM_WLAST;

	wire			MEM_BVALID, MEM_BREADY;
	wire	[AXI_IW-1:0]	MEM_BID;
	wire	[1:0]		MEM_BRESP;

	wire			MEM_ARVALID, MEM_ARREADY;
	wire	[AXI_IW-1:0]	MEM_ARID;
	wire	[AW-1:0]	MEM_ARADDR;
	wire	[7:0]		MEM_ARLEN;
	wire	[2:0]		MEM_ARSIZE;
	wire	[1:0]		MEM_ARBURST;
	wire			MEM_ARLOCK;
	wire	[3:0]		MEM_ARCACHE;
	wire	[2:0]		MEM_ARPROT;
	wire	[3:0]		MEM_ARQOS;

	wire			MEM_RVALID, MEM_RREADY;
	wire	[AXI_IW-1:0]	MEM_RID;
	wire	[DW-1:0]	MEM_RDATA;
	wire			MEM_RLAST;
	wire	[1:0]		MEM_RRESP;

	localparam	RAM_AW = LGMEMSZ-$clog2(DW/8);
	wire			ram_we, ram_rd;
	wire	[DW-1:0]	ram_wdata;
	reg	[DW-1:0]	ram_rdata;
	wire	[DW/8-1:0]	ram_wstrb;
	wire	[RAM_AW-1:0]	ram_waddr, ram_raddr;

	reg	[DW-1:0]	mem	[0:(1<<(RAM_AW))-1];
	integer			ram_wk;
	// }}}

	wire			sd_cmd, sd_ck;
	wire	[3:0]		sd_dat;
	wire			emmc_cmd, emmc_ck, emmc_ds;
	wire	[7:0]		emmc_dat;
	wire			sdio_interrupt, emmc_interrupt;
	wire	[31:0]		sdio_debug, emmc_debug;
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

	initial	reset <= 1;
	initial	begin
		@(posedge clk);
		@(posedge clk)
			reset <= 0;
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// AXI-Lite / Wishbone bus functional model
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	axil_bfm #(
		.AW(AW), .DW(BFM_DW), .LGFIFO(4)
	) u_bfm (
		// {{{
		.i_aclk(clk), .i_aresetn(!reset),
		//
		.AXIL_AWVALID( BFM_AXIL_AWVALID),
		.AXIL_AWREADY( BFM_AXIL_AWREADY),
		.AXIL_AWADDR( BFM_AXIL_AWADDR),
		.AXIL_AWPROT( BFM_AXIL_AWPROT),
		//
		.AXIL_WVALID( BFM_AXIL_WVALID),
		.AXIL_WREADY( BFM_AXIL_WREADY),
		.AXIL_WDATA( BFM_AXIL_WDATA),
		.AXIL_WSTRB( BFM_AXIL_WSTRB),
		//
		.AXIL_BVALID( BFM_AXIL_BVALID),
		.AXIL_BREADY( BFM_AXIL_BREADY),
		.AXIL_BRESP( BFM_AXIL_BRESP),
		//
		.AXIL_ARVALID( BFM_AXIL_ARVALID),
		.AXIL_ARREADY( BFM_AXIL_ARREADY),
		.AXIL_ARADDR( BFM_AXIL_ARADDR),
		.AXIL_ARPROT( BFM_AXIL_ARPROT),
		//
		.AXIL_RVALID( BFM_AXIL_RVALID),
		.AXIL_RREADY( BFM_AXIL_RREADY),
		.AXIL_RDATA( BFM_AXIL_RDATA),
		.AXIL_RRESP( BFM_AXIL_RRESP)
		// }}}
	);

	axilupsz #(
		.C_S_AXIL_DATA_WIDTH(BFM_DW),
		.C_M_AXIL_DATA_WIDTH(DW),
		.C_AXIL_ADDR_WIDTH(AW)
	) u_bfm_upsz (
		// {{{
		.S_AXI_ACLK(clk), .S_AXI_ARESETN(!reset),
		//
		.S_AXIL_AWVALID(BFM_AXIL_AWVALID),
		.S_AXIL_AWREADY(BFM_AXIL_AWREADY),
		.S_AXIL_AWADDR( BFM_AXIL_AWADDR),
		.S_AXIL_AWPROT( BFM_AXIL_AWPROT),
		//
		.S_AXIL_WVALID( BFM_AXIL_WVALID),
		.S_AXIL_WREADY( BFM_AXIL_WREADY),
		.S_AXIL_WDATA(  BFM_AXIL_WDATA),
		.S_AXIL_WSTRB(  BFM_AXIL_WSTRB),
		//
		.S_AXIL_BVALID(BFM_AXIL_BVALID),
		.S_AXIL_BREADY(BFM_AXIL_BREADY),
		.S_AXIL_BRESP( BFM_AXIL_BRESP),
		//
		.S_AXIL_ARVALID(BFM_AXIL_ARVALID),
		.S_AXIL_ARREADY(BFM_AXIL_ARREADY),
		.S_AXIL_ARADDR( BFM_AXIL_ARADDR),
		.S_AXIL_ARPROT( BFM_AXIL_ARPROT),
		//
		.S_AXIL_RVALID( BFM_AXIL_RVALID),
		.S_AXIL_RREADY( BFM_AXIL_RREADY),
		.S_AXIL_RDATA(  BFM_AXIL_RDATA),
		.S_AXIL_RRESP(  BFM_AXIL_RRESP),
		//
		.M_AXIL_AWVALID(BFMW_AXIL_AWVALID),
		.M_AXIL_AWREADY(BFMW_AXIL_AWREADY),
		.M_AXIL_AWADDR( BFMW_AXIL_AWADDR),
		.M_AXIL_AWPROT( BFMW_AXIL_AWPROT),
		//
		.M_AXIL_WVALID( BFMW_AXIL_WVALID),
		.M_AXIL_WREADY( BFMW_AXIL_WREADY),
		.M_AXIL_WDATA(  BFMW_AXIL_WDATA),
		.M_AXIL_WSTRB(  BFMW_AXIL_WSTRB),
		//
		.M_AXIL_BVALID( BFMW_AXIL_BVALID),
		.M_AXIL_BREADY( BFMW_AXIL_BREADY),
		.M_AXIL_BRESP(  BFMW_AXIL_BRESP),
		//
		.M_AXIL_ARVALID(BFMW_AXIL_ARVALID),
		.M_AXIL_ARREADY(BFMW_AXIL_ARREADY),
		.M_AXIL_ARADDR( BFMW_AXIL_ARADDR),
		.M_AXIL_ARPROT( BFMW_AXIL_ARPROT),
		//
		.M_AXIL_RVALID( BFMW_AXIL_RVALID),
		.M_AXIL_RREADY( BFMW_AXIL_RREADY),
		.M_AXIL_RDATA(  BFMW_AXIL_RDATA),
		.M_AXIL_RRESP(  BFMW_AXIL_RRESP)
		// }}}
	);

	axilite2axi #(
		.C_AXI_ID_WIDTH(AXI_IW),
		.C_AXI_ADDR_WIDTH(ADDRESS_WIDTH),
		.C_AXI_DATA_WIDTH(DW)
	) u_bfm_toaxi (
		// {{{
		.ACLK(clk), .ARESETN(reset),
		//
		.S_AXI_AWVALID(BFMW_AXIL_AWVALID),
		.S_AXI_AWREADY(BFMW_AXIL_AWREADY),
		.S_AXI_AWADDR( BFMW_AXIL_AWADDR),
		.S_AXI_AWPROT( BFMW_AXIL_AWPROT),
		//
		.S_AXI_WVALID( BFMW_AXIL_WVALID),
		.S_AXI_WREADY( BFMW_AXIL_WREADY),
		.S_AXI_WDATA(  BFMW_AXIL_WDATA),
		.S_AXI_WSTRB(  BFMW_AXIL_WSTRB),
		//
		.S_AXI_BVALID(BFMW_AXIL_BVALID),
		.S_AXI_BREADY(BFMW_AXIL_BREADY),
		.S_AXI_BRESP( BFMW_AXIL_BRESP),
		//
		.S_AXI_ARVALID(BFMW_AXIL_ARVALID),
		.S_AXI_ARREADY(BFMW_AXIL_ARREADY),
		.S_AXI_ARADDR( BFMW_AXIL_ARADDR),
		.S_AXI_ARPROT( BFMW_AXIL_ARPROT),
		//
		.S_AXI_RVALID( BFMW_AXIL_RVALID),
		.S_AXI_RREADY( BFMW_AXIL_RREADY),
		.S_AXI_RDATA(  BFMW_AXIL_RDATA),
		.S_AXI_RRESP(  BFMW_AXIL_RRESP),
		//
		//
		.M_AXI_AWVALID(BFM_AXI_AWVALID),
		.M_AXI_AWREADY(BFM_AXI_AWREADY),
		.M_AXI_AWID(   BFM_AXI_AWID),
		.M_AXI_AWADDR( BFM_AXI_AWADDR),
		.M_AXI_AWLEN(  BFM_AXI_AWLEN),
		.M_AXI_AWSIZE( wrong_BFM_AXI_AWSIZE),
		.M_AXI_AWBURST(BFM_AXI_AWBURST),
		.M_AXI_AWLOCK( BFM_AXI_AWLOCK),
		.M_AXI_AWCACHE(BFM_AXI_AWCACHE),
		.M_AXI_AWPROT( BFM_AXI_AWPROT),
		.M_AXI_AWQOS(  BFM_AXI_AWQOS),
		//
		.M_AXI_WVALID( BFM_AXI_WVALID),
		.M_AXI_WREADY( BFM_AXI_WREADY),
		.M_AXI_WDATA(  BFM_AXI_WDATA),
		.M_AXI_WSTRB(  BFM_AXI_WSTRB),
		.M_AXI_WLAST(  BFM_AXI_WLAST),
		//
		.M_AXI_BVALID(BFM_AXI_BVALID),
		.M_AXI_BREADY(BFM_AXI_BREADY),
		.M_AXI_BID(   BFM_AXI_BID),
		.M_AXI_BRESP( BFM_AXI_BRESP),
		//
		.M_AXI_ARVALID(BFM_AXI_ARVALID),
		.M_AXI_ARREADY(BFM_AXI_ARREADY),
		.M_AXI_ARID(   BFM_AXI_ARID),
		.M_AXI_ARADDR( BFM_AXI_ARADDR),
		.M_AXI_ARLEN(  BFM_AXI_ARLEN),
		.M_AXI_ARSIZE( wrong_BFM_AXI_ARSIZE),
		.M_AXI_ARBURST(BFM_AXI_ARBURST),
		.M_AXI_ARLOCK( BFM_AXI_ARLOCK),
		.M_AXI_ARCACHE(BFM_AXI_ARCACHE),
		.M_AXI_ARPROT( BFM_AXI_ARPROT),
		.M_AXI_ARQOS(  BFM_AXI_ARQOS),
		//
		.M_AXI_RVALID( BFM_AXI_RVALID),
		.M_AXI_RREADY( BFM_AXI_RREADY),
		.M_AXI_RDATA(  BFM_AXI_RDATA),
		.M_AXI_RRESP(  BFM_AXI_RRESP),
		.M_AXI_RLAST(  BFM_AXI_RLAST)
		// }}}
	);

	assign	BFM_AXI_AWSIZE = 2;
	assign	BFM_AXI_ARSIZE = 2;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Crossbar
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	axixbar #(
		.NM(3), .NS(3),
		.C_AXI_ADDR_WIDTH(AW), .C_AXI_DATA_WIDTH(DW),
		.C_AXI_ID_WIDTH(AXI_IW),
		.SLAVE_ADDR({
			MEM_ADDR,
			EMMC_ADDR,
			SDIO_ADDR}),
		.SLAVE_MASK({
			MEM_MASK,
			EMMC_MASK,
			SDIO_MASK})
	) u_xbar (
		// {{{
		.S_AXI_ACLK(clk), .S_AXI_ARESETN(!reset),
		// Slave (incoming)
		// {{{
		.S_AXI_AWVALID({ EMMC_DMA_AWVALID, SDIO_DMA_AWVALID, BFM_AXI_AWVALID }),
		.S_AXI_AWREADY({ EMMC_DMA_AWREADY, SDIO_DMA_AWREADY, BFM_AXI_AWREADY }),
		.S_AXI_AWID({    EMMC_DMA_AWID,    SDIO_DMA_AWID,    BFM_AXI_AWID }),
		.S_AXI_AWADDR({  EMMC_DMA_AWADDR,  SDIO_DMA_AWADDR,  BFM_AXI_AWADDR }),
		.S_AXI_AWLEN({   EMMC_DMA_AWLEN,   SDIO_DMA_AWLEN,   BFM_AXI_AWLEN }),
		.S_AXI_AWSIZE({  EMMC_DMA_AWSIZE,  SDIO_DMA_AWSIZE,  BFM_AXI_AWSIZE }),
		.S_AXI_AWBURST({ EMMC_DMA_AWBURST, SDIO_DMA_AWBURST, BFM_AXI_AWBURST }),
		.S_AXI_AWLOCK({  EMMC_DMA_AWLOCK,  SDIO_DMA_AWLOCK,  BFM_AXI_AWLOCK }),
		.S_AXI_AWCACHE({ EMMC_DMA_AWCACHE, SDIO_DMA_AWCACHE, BFM_AXI_AWCACHE }),
		.S_AXI_AWPROT({  EMMC_DMA_AWPROT,  SDIO_DMA_AWPROT,  BFM_AXI_AWPROT }),
		.S_AXI_AWQOS({   EMMC_DMA_AWQOS,   SDIO_DMA_AWQOS,   BFM_AXI_AWQOS }),
		//
		.S_AXI_WVALID({ EMMC_DMA_WVALID, SDIO_DMA_WVALID, BFM_AXI_WVALID }),
		.S_AXI_WREADY({ EMMC_DMA_WREADY, SDIO_DMA_WREADY, BFM_AXI_WREADY }),
		.S_AXI_WDATA({  EMMC_DMA_WDATA,  SDIO_DMA_WDATA,  BFM_AXI_WDATA }),
		.S_AXI_WSTRB({  EMMC_DMA_WSTRB,  SDIO_DMA_WSTRB,  BFM_AXI_WSTRB }),
		.S_AXI_WLAST({  EMMC_DMA_WLAST,  SDIO_DMA_WLAST,  BFM_AXI_WLAST }),
		//
		.S_AXI_BVALID({ EMMC_DMA_BVALID, SDIO_DMA_BVALID, BFM_AXI_BVALID }),
		.S_AXI_BREADY({ EMMC_DMA_BREADY, SDIO_DMA_BREADY, BFM_AXI_BREADY }),
		.S_AXI_BID({    EMMC_DMA_BID,    SDIO_DMA_BID,    BFM_AXI_BID }),
		.S_AXI_BRESP({  EMMC_DMA_BRESP,  SDIO_DMA_BRESP,  BFM_AXI_BRESP }),
		//
		.S_AXI_ARVALID({ EMMC_DMA_ARVALID, SDIO_DMA_ARVALID, BFM_AXI_ARVALID }),
		.S_AXI_ARREADY({ EMMC_DMA_ARREADY, SDIO_DMA_ARREADY, BFM_AXI_ARREADY }),
		.S_AXI_ARID({    EMMC_DMA_ARID,    SDIO_DMA_ARID,    BFM_AXI_ARID }),
		.S_AXI_ARADDR({  EMMC_DMA_ARADDR,  SDIO_DMA_ARADDR,  BFM_AXI_ARADDR }),
		.S_AXI_ARLEN({   EMMC_DMA_ARLEN,   SDIO_DMA_ARLEN,   BFM_AXI_ARLEN }),
		.S_AXI_ARSIZE({  EMMC_DMA_ARSIZE,  SDIO_DMA_ARSIZE,  BFM_AXI_ARSIZE }),
		.S_AXI_ARBURST({ EMMC_DMA_ARBURST, SDIO_DMA_ARBURST, BFM_AXI_ARBURST }),
		.S_AXI_ARLOCK({  EMMC_DMA_ARLOCK,  SDIO_DMA_ARLOCK,  BFM_AXI_ARLOCK }),
		.S_AXI_ARCACHE({ EMMC_DMA_ARCACHE, SDIO_DMA_ARCACHE, BFM_AXI_ARCACHE }),
		.S_AXI_ARPROT({  EMMC_DMA_ARPROT,  SDIO_DMA_ARPROT,  BFM_AXI_ARPROT }),
		.S_AXI_ARQOS({   EMMC_DMA_ARQOS,   SDIO_DMA_ARQOS,   BFM_AXI_ARQOS }),
		//
		.S_AXI_RVALID({ EMMC_DMA_RVALID, SDIO_DMA_RVALID, BFM_AXI_RVALID }),
		.S_AXI_RREADY({ EMMC_DMA_RREADY, SDIO_DMA_RREADY, BFM_AXI_RREADY }),
		.S_AXI_RID({    EMMC_DMA_RID,    SDIO_DMA_RID,    BFM_AXI_RID }),
		.S_AXI_RDATA({  EMMC_DMA_RDATA,  SDIO_DMA_RDATA,  BFM_AXI_RDATA }),
		.S_AXI_RRESP({  EMMC_DMA_RRESP,  SDIO_DMA_RRESP,  BFM_AXI_RRESP }),
		.S_AXI_RLAST({  EMMC_DMA_RLAST,  SDIO_DMA_RLAST,  BFM_AXI_RLAST }),
		// }}}
		// Master
		// {{{
		.M_AXI_AWVALID({ MEM_AWVALID, EMMC_AXI_AWVALID, SDIO_AXI_AWVALID }),
		.M_AXI_AWREADY({ MEM_AWREADY, EMMC_AXI_AWREADY, SDIO_AXI_AWREADY }),
		.M_AXI_AWID({    MEM_AWID,    EMMC_AXI_AWID,    SDIO_AXI_AWID }),
		.M_AXI_AWADDR({  MEM_AWADDR,  EMMC_AXI_AWADDR,  SDIO_AXI_AWADDR }),
		.M_AXI_AWLEN({   MEM_AWLEN,   EMMC_AXI_AWLEN,   SDIO_AXI_AWLEN }),
		.M_AXI_AWSIZE({  MEM_AWSIZE,  EMMC_AXI_AWSIZE,  SDIO_AXI_AWSIZE }),
		.M_AXI_AWBURST({ MEM_AWBURST, EMMC_AXI_AWBURST, SDIO_AXI_AWBURST }),
		.M_AXI_AWLOCK({  MEM_AWLOCK,  EMMC_AXI_AWLOCK,  SDIO_AXI_AWLOCK }),
		.M_AXI_AWCACHE({ MEM_AWCACHE, EMMC_AXI_AWCACHE, SDIO_AXI_AWCACHE }),
		.M_AXI_AWPROT({  MEM_AWPROT,  EMMC_AXI_AWPROT,  SDIO_AXI_AWPROT }),
		.M_AXI_AWQOS({   MEM_AWQOS,   EMMC_AXI_AWQOS,   SDIO_AXI_AWQOS }),
		//
		.M_AXI_WVALID({ MEM_WVALID, EMMC_AXI_WVALID, SDIO_AXI_WVALID }),
		.M_AXI_WREADY({ MEM_WREADY, EMMC_AXI_WREADY, SDIO_AXI_WREADY }),
		.M_AXI_WDATA({  MEM_WDATA,  EMMC_AXI_WDATA,  SDIO_AXI_WDATA }),
		.M_AXI_WSTRB({  MEM_WSTRB,  EMMC_AXI_WSTRB,  SDIO_AXI_WSTRB }),
		.M_AXI_WLAST({  MEM_WLAST,  EMMC_AXI_WLAST,  SDIO_AXI_WLAST }),
		//
		.M_AXI_BVALID({ MEM_BVALID, EMMC_AXI_BVALID, SDIO_AXI_BVALID }),
		.M_AXI_BREADY({ MEM_BREADY, EMMC_AXI_BREADY, SDIO_AXI_BREADY }),
		.M_AXI_BID({    MEM_BID,    EMMC_AXI_BID,    SDIO_AXI_BID }),
		.M_AXI_BRESP({  MEM_BRESP,  EMMC_AXI_BRESP,  SDIO_AXI_BRESP }),
		//
		.M_AXI_ARVALID({ MEM_ARVALID, EMMC_AXI_ARVALID, SDIO_AXI_ARVALID }),
		.M_AXI_ARREADY({ MEM_ARREADY, EMMC_AXI_ARREADY, SDIO_AXI_ARREADY }),
		.M_AXI_ARID({    MEM_ARID,    EMMC_AXI_ARID, SDIO_AXI_ARID }),
		.M_AXI_ARADDR({  MEM_ARADDR,  EMMC_AXI_ARADDR, SDIO_AXI_ARADDR }),
		.M_AXI_ARLEN({   MEM_ARLEN,   EMMC_AXI_ARLEN, SDIO_AXI_ARLEN }),
		.M_AXI_ARSIZE({  MEM_ARSIZE,  EMMC_AXI_ARSIZE, SDIO_AXI_ARSIZE }),
		.M_AXI_ARBURST({ MEM_ARBURST, EMMC_AXI_ARBURST, SDIO_AXI_ARBURST }),
		.M_AXI_ARLOCK({  MEM_ARLOCK,  EMMC_AXI_ARLOCK, SDIO_AXI_ARLOCK }),
		.M_AXI_ARCACHE({ MEM_ARCACHE, EMMC_AXI_ARCACHE, SDIO_AXI_ARCACHE }),
		.M_AXI_ARPROT({  MEM_ARPROT,  EMMC_AXI_ARPROT, SDIO_AXI_ARPROT }),
		.M_AXI_ARQOS({   MEM_ARQOS,   EMMC_AXI_ARQOS, SDIO_AXI_ARQOS }),
		//
		.M_AXI_RVALID({ MEM_RVALID, EMMC_AXI_RVALID, SDIO_AXI_RVALID }),
		.M_AXI_RREADY({ MEM_RREADY, EMMC_AXI_RREADY, SDIO_AXI_RREADY }),
		.M_AXI_RID({    MEM_RID,    EMMC_AXI_RID,    SDIO_AXI_RID }),
		.M_AXI_RDATA({  MEM_RDATA,  EMMC_AXI_RDATA,  SDIO_AXI_RDATA }),
		.M_AXI_RRESP({  MEM_RRESP,  EMMC_AXI_RRESP,  SDIO_AXI_RRESP }),
		.M_AXI_RLAST({  MEM_RLAST,  EMMC_AXI_RLAST,  SDIO_AXI_RLAST })
		// }}}
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// RAM model
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	axiram #(
		.C_S_AXI_ID_WIDTH(AXI_IW), .C_S_AXI_DATA_WIDTH(DW),
		.C_S_AXI_ADDR_WIDTH(LGMEMSZ), .OPT_LOCK(1'b0)
	) u_mem (
		// {{{
		.S_AXI_ACLK(clk), .S_AXI_ARESETN(!reset),
		//
		.S_AXI_AWVALID(MEM_AWVALID), .S_AXI_AWREADY(MEM_AWREADY),
		.S_AXI_AWID(MEM_AWID), .S_AXI_AWADDR(MEM_AWADDR[LGMEMSZ-1:0]),
		.S_AXI_AWLEN(MEM_AWLEN), .S_AXI_AWSIZE(MEM_AWSIZE),
		.S_AXI_AWBURST(MEM_AWBURST), .S_AXI_AWLOCK(MEM_AWLOCK),
		.S_AXI_AWCACHE(MEM_AWCACHE), .S_AXI_AWPROT(MEM_AWPROT),
		.S_AXI_AWQOS(MEM_AWQOS),
		//
		.S_AXI_WVALID(MEM_WVALID), .S_AXI_WREADY(MEM_WREADY),
		.S_AXI_WDATA(MEM_WDATA), .S_AXI_WSTRB(MEM_WSTRB),
		.S_AXI_WLAST(MEM_WLAST),
		//
		.S_AXI_BVALID(MEM_BVALID), .S_AXI_BREADY(MEM_BREADY),
		.S_AXI_BID(MEM_BID), .S_AXI_BRESP(MEM_BRESP),
		//
		.S_AXI_ARVALID(MEM_ARVALID), .S_AXI_ARREADY(MEM_ARREADY),
		.S_AXI_ARID(MEM_ARID), .S_AXI_ARADDR(MEM_ARADDR[LGMEMSZ-1:0]),
		.S_AXI_ARLEN(MEM_ARLEN), .S_AXI_ARSIZE(MEM_ARSIZE),
		.S_AXI_ARBURST(MEM_ARBURST), .S_AXI_ARLOCK(MEM_ARLOCK),
		.S_AXI_ARCACHE(MEM_ARCACHE), .S_AXI_ARPROT(MEM_ARPROT),
		.S_AXI_ARQOS(MEM_ARQOS),
		//
		.S_AXI_RVALID(MEM_RVALID), .S_AXI_RREADY(MEM_RREADY),
		.S_AXI_RID(MEM_RID), .S_AXI_RDATA(MEM_RDATA),
		.S_AXI_RLAST(MEM_RLAST), .S_AXI_RRESP(MEM_RRESP),
		//
		.o_we(ram_we), .o_waddr(ram_waddr), .o_wdata(ram_wdata),
		.o_wstrb(ram_wstrb), .o_rd(ram_rd), .o_raddr(ram_raddr),
		.i_rdata(ram_rdata)
		// }}}
	);

	
	always @(posedge clk)
	if (ram_we)
	for(ram_wk=0; ram_wk < DW/8; ram_wk=ram_wk+1)
	if (ram_wstrb[ram_wk])
		mem[ram_waddr][ram_wk*8 +: 8] <= ram_wdata[ram_wk*8 +: 8];

	always @(posedge clk)
	if (ram_rd)
		ram_rdata <= mem[ram_raddr];

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Unit under test
	// {{{

	// Downsize
	axi2axilsub #(
		// {{{
		.C_AXI_ID_WIDTH(AXI_IW),
		.C_S_AXI_DATA_WIDTH(DW),
		.C_M_AXI_DATA_WIDTH(32),
		.C_AXI_ADDR_WIDTH(ADDRESS_WIDTH)
		// }}}
	) u_sdio_downsz (
		.S_AXI_ACLK(clk), .S_AXI_ARESETN(!reset),
		// Slave
		// {{{
		.S_AXI_AWVALID( SDIO_AXI_AWVALID ),
		.S_AXI_AWREADY( SDIO_AXI_AWREADY ),
		.S_AXI_AWID(    SDIO_AXI_AWID ),
		.S_AXI_AWADDR(  SDIO_AXI_AWADDR ),
		.S_AXI_AWLEN(   SDIO_AXI_AWLEN ),
		.S_AXI_AWSIZE(  SDIO_AXI_AWSIZE ),
		.S_AXI_AWBURST( SDIO_AXI_AWBURST ),
		.S_AXI_AWLOCK(  SDIO_AXI_AWLOCK ),
		.S_AXI_AWCACHE( SDIO_AXI_AWCACHE ),
		.S_AXI_AWPROT(  SDIO_AXI_AWPROT ),
		.S_AXI_AWQOS(   SDIO_AXI_AWQOS ),
		//
		.S_AXI_WVALID( SDIO_AXI_WVALID ),
		.S_AXI_WREADY( SDIO_AXI_WREADY ),
		.S_AXI_WDATA(  SDIO_AXI_WDATA ),
		.S_AXI_WSTRB(  SDIO_AXI_WSTRB ),
		.S_AXI_WLAST(  SDIO_AXI_WLAST ),
		//
		.S_AXI_BVALID( SDIO_AXI_BVALID ),
		.S_AXI_BREADY( SDIO_AXI_BREADY ),
		.S_AXI_BID(    SDIO_AXI_BID ),
		.S_AXI_BRESP(  SDIO_AXI_BRESP ),
		//
		.S_AXI_ARVALID( SDIO_AXI_ARVALID ),
		.S_AXI_ARREADY( SDIO_AXI_ARREADY ),
		.S_AXI_ARID(    SDIO_AXI_ARID ),
		.S_AXI_ARADDR(  SDIO_AXI_ARADDR ),
		.S_AXI_ARLEN(   SDIO_AXI_ARLEN ),
		.S_AXI_ARSIZE(  SDIO_AXI_ARSIZE ),
		.S_AXI_ARBURST( SDIO_AXI_ARBURST ),
		.S_AXI_ARLOCK(  SDIO_AXI_ARLOCK ),
		.S_AXI_ARCACHE( SDIO_AXI_ARCACHE ),
		.S_AXI_ARPROT(  SDIO_AXI_ARPROT ),
		.S_AXI_ARQOS(   SDIO_AXI_ARQOS ),
		//
		.S_AXI_RVALID( SDIO_AXI_RVALID ),
		.S_AXI_RREADY( SDIO_AXI_RREADY ),
		.S_AXI_RID(    SDIO_AXI_RID ),
		.S_AXI_RDATA(  SDIO_AXI_RDATA ),
		.S_AXI_RRESP(  SDIO_AXI_RRESP ),
		.S_AXI_RLAST(  SDIO_AXI_RLAST ),
		// }}}
		// Master
		// {{{
		.M_AXI_AWVALID( SDIO_AWVALID ),
		.M_AXI_AWREADY( SDIO_AWREADY ),
		.M_AXI_AWADDR(  SDIO_AWADDR ),
		.M_AXI_AWPROT(  SDIO_AWPROT ),
		//
		.M_AXI_WVALID( SDIO_WVALID ),
		.M_AXI_WREADY( SDIO_WREADY ),
		.M_AXI_WDATA(  SDIO_WDATA ),
		.M_AXI_WSTRB(  SDIO_WSTRB ),
		//
		.M_AXI_BVALID( SDIO_BVALID ),
		.M_AXI_BREADY( SDIO_BREADY ),
		.M_AXI_BRESP(  SDIO_BRESP ),
		//
		.M_AXI_ARVALID( SDIO_ARVALID ),
		.M_AXI_ARREADY( SDIO_ARREADY ),
		.M_AXI_ARADDR(  SDIO_ARADDR ),
		.M_AXI_ARPROT(  SDIO_ARPROT ),
		//
		.M_AXI_RVALID( SDIO_RVALID ),
		.M_AXI_RREADY( SDIO_RREADY ),
		.M_AXI_RDATA(  SDIO_RDATA ),
		.M_AXI_RRESP(  SDIO_RRESP )
		// }}}
	);

	axi2axilsub #(
		// {{{
		.C_AXI_ID_WIDTH(AXI_IW),
		.C_S_AXI_DATA_WIDTH(DW),
		.C_M_AXI_DATA_WIDTH(32),
		.C_AXI_ADDR_WIDTH(ADDRESS_WIDTH)
		// }}}
	) u_emmc_downsz (
		.S_AXI_ACLK(clk), .S_AXI_ARESETN(!reset),
		// Slave
		// {{{
		.S_AXI_AWVALID( EMMC_AXI_AWVALID ),
		.S_AXI_AWREADY( EMMC_AXI_AWREADY ),
		.S_AXI_AWID(    EMMC_AXI_AWID ),
		.S_AXI_AWADDR(  EMMC_AXI_AWADDR ),
		.S_AXI_AWLEN(   EMMC_AXI_AWLEN ),
		.S_AXI_AWSIZE(  EMMC_AXI_AWSIZE ),
		.S_AXI_AWBURST( EMMC_AXI_AWBURST ),
		.S_AXI_AWLOCK(  EMMC_AXI_AWLOCK ),
		.S_AXI_AWCACHE( EMMC_AXI_AWCACHE ),
		.S_AXI_AWPROT(  EMMC_AXI_AWPROT ),
		.S_AXI_AWQOS(   EMMC_AXI_AWQOS ),
		//
		.S_AXI_WVALID( EMMC_AXI_WVALID ),
		.S_AXI_WREADY( EMMC_AXI_WREADY ),
		.S_AXI_WDATA(  EMMC_AXI_WDATA ),
		.S_AXI_WSTRB(  EMMC_AXI_WSTRB ),
		.S_AXI_WLAST(  EMMC_AXI_WLAST ),
		//
		.S_AXI_BVALID( EMMC_AXI_BVALID ),
		.S_AXI_BREADY( EMMC_AXI_BREADY ),
		.S_AXI_BID(    EMMC_AXI_BID ),
		.S_AXI_BRESP(  EMMC_AXI_BRESP ),
		//
		.S_AXI_ARVALID( EMMC_AXI_ARVALID ),
		.S_AXI_ARREADY( EMMC_AXI_ARREADY ),
		.S_AXI_ARID(    EMMC_AXI_ARID ),
		.S_AXI_ARADDR(  EMMC_AXI_ARADDR ),
		.S_AXI_ARLEN(   EMMC_AXI_ARLEN ),
		.S_AXI_ARSIZE(  EMMC_AXI_ARSIZE ),
		.S_AXI_ARBURST( EMMC_AXI_ARBURST ),
		.S_AXI_ARLOCK(  EMMC_AXI_ARLOCK ),
		.S_AXI_ARCACHE( EMMC_AXI_ARCACHE ),
		.S_AXI_ARPROT(  EMMC_AXI_ARPROT ),
		.S_AXI_ARQOS(   EMMC_AXI_ARQOS ),
		//
		.S_AXI_RVALID( EMMC_AXI_RVALID ),
		.S_AXI_RREADY( EMMC_AXI_RREADY ),
		.S_AXI_RID(    EMMC_AXI_RID ),
		.S_AXI_RDATA(  EMMC_AXI_RDATA ),
		.S_AXI_RRESP(  EMMC_AXI_RRESP ),
		.S_AXI_RLAST(  EMMC_AXI_RLAST ),
		// }}}
		// Master
		// {{{
		.M_AXI_AWVALID( EMMC_AWVALID ),
		.M_AXI_AWREADY( EMMC_AWREADY ),
		.M_AXI_AWADDR(  EMMC_AWADDR ),
		.M_AXI_AWPROT(  EMMC_AWPROT ),
		//
		.M_AXI_WVALID( EMMC_WVALID ),
		.M_AXI_WREADY( EMMC_WREADY ),
		.M_AXI_WDATA(  EMMC_WDATA ),
		.M_AXI_WSTRB(  EMMC_WSTRB ),
		//
		.M_AXI_BVALID( EMMC_BVALID ),
		.M_AXI_BREADY( EMMC_BREADY ),
		.M_AXI_BRESP(  EMMC_BRESP ),
		//
		.M_AXI_ARVALID( EMMC_ARVALID ),
		.M_AXI_ARREADY( EMMC_ARREADY ),
		.M_AXI_ARADDR(  EMMC_ARADDR ),
		.M_AXI_ARPROT(  EMMC_ARPROT ),
		//
		.M_AXI_RVALID( EMMC_RVALID ),
		.M_AXI_RREADY( EMMC_RREADY ),
		.M_AXI_RDATA(  EMMC_RDATA ),
		.M_AXI_RRESP(  EMMC_RRESP )
		// }}}
	);

	sdio_top #(
		// {{{
		.LGFIFO(12), .NUMIO(4),
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.DW(DW), .AXI_IW(AXI_IW),
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR),
		.OPT_CARD_DETECT(0), .LGTIMEOUT(10),
		.OPT_DMA(OPT_DMA), .OPT_EMMC(1'b0)
		// }}}
	) u_sdio (
		// {{{
		.i_clk(clk), .i_reset(reset), .i_hsclk(hsclk),
		//
		// AXI-Lite Control/data interface
		// {{{
		.S_AXIL_AWVALID( SDIO_AWVALID),
		.S_AXIL_AWREADY( SDIO_AWREADY),
		.S_AXIL_AWADDR( SDIO_AWADDR[4:0]),
		.S_AXIL_AWPROT( SDIO_AWPROT),
		//
		.S_AXIL_WVALID( SDIO_WVALID),
		.S_AXIL_WREADY( SDIO_WREADY),
		.S_AXIL_WDATA( SDIO_WDATA),
		.S_AXIL_WSTRB( SDIO_WSTRB),
		//
		.S_AXIL_BVALID( SDIO_BVALID),
		.S_AXIL_BREADY( SDIO_BREADY),
		.S_AXIL_BRESP( SDIO_BRESP),
		//
		.S_AXIL_ARVALID( SDIO_ARVALID),
		.S_AXIL_ARREADY( SDIO_ARREADY),
		.S_AXIL_ARADDR( SDIO_ARADDR[4:0]),
		.S_AXIL_ARPROT( SDIO_ARPROT),
		//
		.S_AXIL_RVALID( SDIO_RVALID),
		.S_AXIL_RREADY( SDIO_RREADY),
		.S_AXIL_RDATA( SDIO_RDATA),
		.S_AXIL_RRESP( SDIO_RRESP),
		// }}}
		// DMA interface
		// {{{
		.M_AXI_AWVALID( SDIO_DMA_AWVALID),
		.M_AXI_AWREADY( SDIO_DMA_AWREADY),
		.M_AXI_AWID(    SDIO_DMA_AWID   ),
		.M_AXI_AWADDR(  SDIO_DMA_AWADDR ),
		.M_AXI_AWLEN(   SDIO_DMA_AWLEN  ),
		.M_AXI_AWSIZE(  SDIO_DMA_AWSIZE ),
		.M_AXI_AWBURST( SDIO_DMA_AWBURST),
		.M_AXI_AWLOCK(  SDIO_DMA_AWLOCK ),
		.M_AXI_AWCACHE( SDIO_DMA_AWCACHE),
		.M_AXI_AWPROT(  SDIO_DMA_AWPROT ),
		.M_AXI_AWQOS(   SDIO_DMA_AWQOS  ),
		//
		.M_AXI_WVALID( SDIO_DMA_WVALID),
		.M_AXI_WREADY( SDIO_DMA_WREADY),
		.M_AXI_WDATA(  SDIO_DMA_WDATA ),
		.M_AXI_WSTRB(  SDIO_DMA_WSTRB ),
		.M_AXI_WLAST(  SDIO_DMA_WLAST ),
		//
		.M_AXI_BVALID( SDIO_DMA_BVALID),
		.M_AXI_BREADY( SDIO_DMA_BREADY),
		.M_AXI_BID(    SDIO_DMA_BID   ),
		.M_AXI_BRESP(  SDIO_DMA_BRESP ),
		//
		.M_AXI_ARVALID( SDIO_DMA_ARVALID),
		.M_AXI_ARREADY( SDIO_DMA_ARREADY),
		.M_AXI_ARID(    SDIO_DMA_ARID   ),
		.M_AXI_ARADDR(  SDIO_DMA_ARADDR ),
		.M_AXI_ARLEN(   SDIO_DMA_ARLEN  ),
		.M_AXI_ARSIZE(  SDIO_DMA_ARSIZE ),
		.M_AXI_ARBURST( SDIO_DMA_ARBURST),
		.M_AXI_ARLOCK(  SDIO_DMA_ARLOCK ),
		.M_AXI_ARCACHE( SDIO_DMA_ARCACHE),
		.M_AXI_ARPROT(  SDIO_DMA_ARPROT ),
		.M_AXI_ARQOS(   SDIO_DMA_ARQOS  ),
		//
		.M_AXI_RVALID( SDIO_DMA_RVALID),
		.M_AXI_RREADY( SDIO_DMA_RREADY),
		.M_AXI_RID(    SDIO_DMA_RID   ),
		.M_AXI_RDATA(  SDIO_DMA_RDATA ),
		.M_AXI_RRESP(  SDIO_DMA_RRESP ),
		.M_AXI_RLAST(  SDIO_DMA_RLAST ),
		// }}}
		//
		.o_ck(sd_ck), .i_ds(1'b0), .io_cmd(sd_cmd), .io_dat(sd_dat),
		.i_card_detect(1'b1), .o_int(sdio_interrupt),
		.o_debug(sdio_debug)
		// }}}
	);

	sdio_top #(
		// {{{
		.LGFIFO(12), .NUMIO(8),
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.DW(DW), .AXI_IW(AXI_IW),
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR),
		.OPT_CARD_DETECT(0), .LGTIMEOUT(10),
		.OPT_DMA(OPT_DMA), .OPT_EMMC(1'b1)
		// }}}
	) u_emmc (
		// {{{
		.i_clk(clk), .i_reset(reset), .i_hsclk(hsclk),
		//
		// AXI-Lite Control/data interface
		// {{{
		.S_AXIL_AWVALID( EMMC_AWVALID),
		.S_AXIL_AWREADY( EMMC_AWREADY),
		.S_AXIL_AWADDR( EMMC_AWADDR[4:0]),
		.S_AXIL_AWPROT( EMMC_AWPROT),
		//
		.S_AXIL_WVALID( EMMC_WVALID),
		.S_AXIL_WREADY( EMMC_WREADY),
		.S_AXIL_WDATA( EMMC_WDATA),
		.S_AXIL_WSTRB( EMMC_WSTRB),
		//
		.S_AXIL_BVALID( EMMC_BVALID),
		.S_AXIL_BREADY( EMMC_BREADY),
		.S_AXIL_BRESP( EMMC_BRESP),
		//
		.S_AXIL_ARVALID( EMMC_ARVALID),
		.S_AXIL_ARREADY( EMMC_ARREADY),
		.S_AXIL_ARADDR( EMMC_ARADDR[4:0]),
		.S_AXIL_ARPROT( EMMC_ARPROT),
		//
		.S_AXIL_RVALID( EMMC_RVALID),
		.S_AXIL_RREADY( EMMC_RREADY),
		.S_AXIL_RDATA( EMMC_RDATA),
		.S_AXIL_RRESP( EMMC_RRESP),
		// }}}
		// DMA interface
		// {{{
		.M_AXI_AWVALID( EMMC_DMA_AWVALID),
		.M_AXI_AWREADY( EMMC_DMA_AWREADY),
		.M_AXI_AWID(    EMMC_DMA_AWID   ),
		.M_AXI_AWADDR(  EMMC_DMA_AWADDR ),
		.M_AXI_AWLEN(   EMMC_DMA_AWLEN  ),
		.M_AXI_AWSIZE(  EMMC_DMA_AWSIZE ),
		.M_AXI_AWBURST( EMMC_DMA_AWBURST),
		.M_AXI_AWLOCK(  EMMC_DMA_AWLOCK ),
		.M_AXI_AWCACHE( EMMC_DMA_AWCACHE),
		.M_AXI_AWPROT(  EMMC_DMA_AWPROT ),
		.M_AXI_AWQOS(   EMMC_DMA_AWQOS  ),
		//
		.M_AXI_WVALID( EMMC_DMA_WVALID),
		.M_AXI_WREADY( EMMC_DMA_WREADY),
		.M_AXI_WDATA(  EMMC_DMA_WDATA ),
		.M_AXI_WSTRB(  EMMC_DMA_WSTRB ),
		.M_AXI_WLAST(  EMMC_DMA_WLAST ),
		//
		.M_AXI_BVALID( EMMC_DMA_BVALID),
		.M_AXI_BREADY( EMMC_DMA_BREADY),
		.M_AXI_BID(    EMMC_DMA_BID   ),
		.M_AXI_BRESP(  EMMC_DMA_BRESP ),
		//
		.M_AXI_ARVALID( EMMC_DMA_ARVALID),
		.M_AXI_ARREADY( EMMC_DMA_ARREADY),
		.M_AXI_ARID(    EMMC_DMA_ARID   ),
		.M_AXI_ARADDR(  EMMC_DMA_ARADDR ),
		.M_AXI_ARLEN(   EMMC_DMA_ARLEN  ),
		.M_AXI_ARSIZE(  EMMC_DMA_ARSIZE ),
		.M_AXI_ARBURST( EMMC_DMA_ARBURST),
		.M_AXI_ARLOCK(  EMMC_DMA_ARLOCK ),
		.M_AXI_ARCACHE( EMMC_DMA_ARCACHE),
		.M_AXI_ARPROT(  EMMC_DMA_ARPROT ),
		.M_AXI_ARQOS(   EMMC_DMA_ARQOS  ),
		//
		.M_AXI_RVALID( EMMC_DMA_RVALID),
		.M_AXI_RREADY( EMMC_DMA_RREADY),
		.M_AXI_RID(    EMMC_DMA_RID   ),
		.M_AXI_RDATA(  EMMC_DMA_RDATA ),
		.M_AXI_RRESP(  EMMC_DMA_RRESP ),
		.M_AXI_RLAST(  EMMC_DMA_RLAST ),
		// }}}
		//
		.o_ck(emmc_ck),
			.io_cmd(emmc_cmd), .io_dat(emmc_dat), .i_ds(emmc_ds),
		.i_card_detect(1'b1), .o_int(emmc_interrupt),
		.o_debug(emmc_debug)
		// }}}
	);


	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// eMMC Device model
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	mdl_emmc #(
		.LGMEMSZ(20),
		.OPT_HIGH_CAPACITY(1'b1)
	) u_mcchip (
		.rst_n(!reset),
		.sd_clk(emmc_ck), .sd_cmd(emmc_cmd), .sd_dat(emmc_dat),
			.sd_ds(emmc_ds)
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDIO Device model
	// {{{

	mdl_sdio #(
		.LGMEMSZ(16),
		.OPT_HIGH_CAPACITY(1'b1)
	) u_sdcard (
		// .rst_n(1'b1),
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
		$dumpvars(0, tb_axi);
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

	always @(posedge error_flag)
	if (!reset)
	begin
		$display("ERROR-FLAG DETECTED");
		repeat(200)
			@(posedge clk);
		$display("TEST-FAIL/ERROR FLAG");
		$finish;
	end
	// }}}
endmodule
