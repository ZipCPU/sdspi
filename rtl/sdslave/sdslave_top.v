////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdslave_top.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	A top level file for the SDIO slave controller.  This
//		file references both architecture specific modules in
//	sdsfrontend.v, and non-architecture specific logic via sdslave.v.
//	Otherwise, the top level (non-architecture specific) module would be
//	sdslave.v.
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
`timescale 1ns/1ps
`default_nettype none
// }}}
module sdslave_top #(
		// {{{
		// ADDRESS_WIDTH: Number of bits to the DMA's address lines,
		// {{{
		// as required to access octets of memory.  This is not the word
		// address width, but the octet/byte address width.
		parameter	ADDRESS_WIDTH=48,
		parameter	DW=64,
`ifdef	SDIO_AXI
		parameter		AXI_IW=4,
		parameter [AXI_IW-1:0]	AXI_READ_ID  = 0,
		parameter [AXI_IW-1:0]	AXI_WRITE_ID = 1,
		localparam		AW = ADDRESS_WIDTH,
`else
		localparam		AW = ADDRESS_WIDTH - $clog2(DW/8),
`endif
		// }}}
		// NUMIO can only be 1, 4, or 8.  It can *only* be 8 if OPT_EMMC
		// {{{
		//   is set, otherwise it must be either 1 or 4.  Most SDIO
		//   devices want to operate in 4bit mode, so 1bit is not
		//   recommended.  (This slave does *not* support the 1b SPI
		//   protocol.)
		parameter	NUMIO = 4,
		// }}}
		// If OPT_EMMC is set, our state machine will support eMMC
		// {{{
		//   devices.  This state machine hasn't (yet) been built, so
		//   we're stuck (for the time being) with SDIO devices only.
		localparam [0:0]	OPT_EMMC = 1'b0,
		// }}}
		// If OPT_DDR is set, we support DDR protocols, such as DDR50,
		// {{{
		//   and (perhaps later) HS400.  For now, this *must* be 0,
		//   since our FSM doesn't (yet) support getting into or out of
		//   any DDR modes.
		localparam [0:0]	OPT_DDR = 1'b0,
		// }}}
		// If OPT_DS is set, we (might) support HS400.  This requires
		// {{{
		//   OPT_EMMC && OPT_DDR to be set.  It's only a *might* at
		//   present, since the EMMC logic doesn't (yet) exist.
		localparam [0:0]	OPT_DS = OPT_EMMC && OPT_DDR,
		// }}}
		// If OPT_1P8V is set, we'll (somehow) support voltage switching
		// {{{
		//   from 3.3V mode down to 1.8V.  Since this isn't (normally)
		//   possible from an FPGA, this is set to zero.
		localparam [0:0]	OPT_1P8V = 1'b0
		// }}}
		// }}}
	) (
		// {{{
		input	wire			i_bus_clk, i_aresetn,
		// DMA Interface
		// {{{
`ifdef	SDIO_AXI
		// AXI DMA (Master) interface
		// {{{
		// AXI Write address
		output	wire			M_AXI_AWVALID,
		input	wire			M_AXI_AWREADY,
		output	wire	[AXI_IW-1:0]	M_AXI_AWID,
		output	wire	[AW-1:0]	M_AXI_AWADDR,
		output	wire	[7:0]		M_AXI_AWLEN,
		output	wire	[2:0]		M_AXI_AWSIZE,
		output	wire	[1:0]		M_AXI_AWBURST,
		output	wire			M_AXI_AWLOCK,
		output	wire	[3:0]		M_AXI_AWCACHE,
		output	wire	[2:0]		M_AXI_AWPROT,
		output	wire	[3:0]		M_AXI_AWQOS,
		// AXI Write data
		output	wire			M_AXI_WVALID,
		input	wire			M_AXI_WREADY,
		output	wire	[DW-1:0]	M_AXI_WDATA,
		output	wire	[DW/8-1:0]	M_AXI_WSTRB,
		output	wire			M_AXI_WLAST,
		// AXI Write response
		input	wire			M_AXI_BVALID,
		output	wire			M_AXI_BREADY,
		input	wire	[AXI_IW-1:0]	M_AXI_BID,
		input	wire	[1:0]		M_AXI_BRESP,
		// AXI Read address
		output	wire			M_AXI_ARVALID,
		input	wire			M_AXI_ARREADY,
		output	wire	[AXI_IW-1:0]	M_AXI_ARID,
		output	wire	[AW-1:0]	M_AXI_ARADDR,
		output	wire	[7:0]		M_AXI_ARLEN,
		output	wire	[2:0]		M_AXI_ARSIZE,
		output	wire	[1:0]		M_AXI_ARBURST,
		output	wire			M_AXI_ARLOCK,
		output	wire	[3:0]		M_AXI_ARCACHE,
		output	wire	[2:0]		M_AXI_ARPROT,
		output	wire	[3:0]		M_AXI_ARQOS,
		// AXI Read data
		input	wire			M_AXI_RVALID,
		output	wire			M_AXI_RREADY,
		input	wire	[AXI_IW-1:0]	M_AXI_RID,
		input	wire	[DW-1:0]	M_AXI_RDATA,
		input	wire			M_AXI_RLAST,
		input	wire	[1:0]		M_AXI_RRESP,
		// }}}
`else
		output	wire			o_cyc, o_stb, o_we,
		output	wire	[AW-1:0]	o_addr,
		output	wire	[DW-1:0]	o_data,
		output	wire	[DW/8-1:0]	o_sel,
		input	wire			i_stall,
		input	wire			i_ack,
		input	wire	[DW-1:0]	i_data,
		input	wire			i_err,
`endif
		// }}}
		// IO interface
		// {{{
		input	wire			i_ck,
		output	wire			o_ds,
		inout	wire			io_cmd,
		inout	wire	[NUMIO-1:0]	io_dat
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	reg		slv_resetn, slv_resetn_pipe;

	wire		tx_cmd, cmd_tristate, tx_ds;
	wire	[15:0]	tx_data;
	wire	[7:0]	tx_tristate;

	wire		rx_cmd, w_collision;
	wire	[15:0]	rx_data;
	wire	[1:0]	w_ds;
	wire		w_ds_tristate;
	// }}}

	// Bus clock and reset
	// {{{
	initial	{ slv_resetn, slv_resetn_pipe } <= 2'b00;
	always @(posedge i_bus_clk or negedge i_aresetn)
	if (!i_aresetn)
		{ slv_resetn, slv_resetn_pipe } <= 2'b00;
	else
		{ slv_resetn, slv_resetn_pipe } <= { slv_resetn_pipe, 1'b1 };
	// }}}


	sdslave #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH), .DW(DW),
`ifdef	SDIO_AXI
		.AXI_IW(AXI_IW), .AXI_READ_ID(AXI_READ_ID),
		.AXI_WRITE_ID(AXI_WRITE_ID),
`endif
		.OPT_DDR(OPT_DDR),
		.NUMIO(NUMIO) // ,
		// .OPT_DS(OPT_DS)		// OPT_DS isn't supported here
		// .OPT_EMMC(OPT_EMMC),
		// .OPT_1P8V(OPT_1P8V),
		// }}}
	) u_sdslave (
		// {{{
		.i_clk(i_bus_clk), .i_reset(!slv_resetn),
		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// AXI master (DMA) interface
		// {{{
		// AXI Write address
		.M_AXI_AWVALID(M_AXI_AWVALID),
		.M_AXI_AWREADY(M_AXI_AWREADY),
		.M_AXI_AWID(M_AXI_AWID),
		.M_AXI_AWADDR(M_AXI_AWADDR),
		.M_AXI_AWLEN(M_AXI_AWLEN),
		.M_AXI_AWSIZE(M_AXI_AWSIZE),
		.M_AXI_AWBURST(M_AXI_AWBURST),
		.M_AXI_AWLOCK(M_AXI_AWLOCK),
		.M_AXI_AWCACHE(M_AXI_AWCACHE),
		.M_AXI_AWPROT(M_AXI_AWPROT),
		.M_AXI_AWQOS(M_AXI_AWQOS),
		// AXI Write data
		.M_AXI_WVALID(M_AXI_WVALID),
		.M_AXI_WREADY(M_AXI_WREADY),
		.M_AXI_WDATA(M_AXI_WDATA),
		.M_AXI_WSTRB(M_AXI_WSTRB),
		.M_AXI_WLAST(M_AXI_WLAST),
		// AXI Write respons
		.M_AXI_BVALID(M_AXI_BVALID),
		.M_AXI_BREADY(M_AXI_BREADY),
		.M_AXI_BID(M_AXI_BID),
		.M_AXI_BRESP(M_AXI_BRESP),
		//
		// AXI Read address
		.M_AXI_ARVALID(M_AXI_ARVALID),
		.M_AXI_ARREADY(M_AXI_ARREADY),
		.M_AXI_ARID(M_AXI_ARID),
		.M_AXI_ARADDR(M_AXI_ARADDR),
		.M_AXI_ARLEN(M_AXI_ARLEN),
		.M_AXI_ARSIZE(M_AXI_ARSIZE),
		.M_AXI_ARBURST(M_AXI_ARBURST),
		.M_AXI_ARLOCK(M_AXI_ARLOCK),
		.M_AXI_ARCACHE(M_AXI_ARCACHE),
		.M_AXI_ARPROT(M_AXI_ARPROT),
		.M_AXI_ARQOS(M_AXI_ARQOS),
		// AXI Read data
		.M_AXI_RVALID(M_AXI_RVALID),
		.M_AXI_RREADY(M_AXI_RREADY),
		.M_AXI_RID(M_AXI_RID),
		.M_AXI_RDATA(M_AXI_RDATA),
		.M_AXI_RLAST(M_AXI_RLAST),
		.M_AXI_RRESP(M_AXI_RRESP),
		// }}}
`else
		// Wishbone master (DMA) interface
		// {{{
		.o_dma_cyc(o_cyc),
		.o_dma_stb(o_stb),
		.o_dma_we(o_we),
		.o_dma_addr(o_addr),
		.o_dma_data(o_data),
		.o_dma_sel(o_sel),
		.i_dma_stall(i_stall),
		.i_dma_ack(i_ack),
		.i_dma_data(i_data),
		.i_dma_err(i_err),
		// }}}
`endif
		// }}}
		// Interface to PHY
		// {{{
		.i_sd_clk(i_ck),
		.i_cmd(rx_cmd),
		.o_cmd(tx_cmd),
		.o_cmd_tristate(cmd_tristate),
		//
		.i_dat(rx_data),
		.o_dat(tx_data),
		.o_dat_tristate(tx_tristate),
		//
		.o_ds(w_ds),
		.o_ds_tristate(w_ds_tristate)
		// }}}
		// }}}
	);

	sdsfrontend #(
		// {{{
		.NUMIO(NUMIO), .OPT_DS(OPT_DS)
		// , .OPT_COLLISION(OPT_COLLISION),
		// }}}
	) u_frontend (
		// {{{
		// .i_reset(i_reset),
		// Tx path
		// {{{
		.i_tx_cmd(tx_cmd),
		.i_tx_cmd_tristate(cmd_tristate),
		//
		.i_tx_data(tx_data), .i_tx_data_tristate(tx_tristate),
			.i_tx_ds({ tx_ds, 1'b0 }),
		// }}}
		// Synchronous Rx path
		// {{{
		.o_rx_cmd(rx_cmd),
		.o_cmd_collision(w_collision),
		//
		.o_rx_dat(rx_data),
		// }}}
		// I/O ports
		// {{{
		.i_ck(i_ck), .o_ds(o_ds),
		.io_cmd(io_cmd),
		.io_dat(io_dat)
		// }}}
		// }}}
	);

endmodule
