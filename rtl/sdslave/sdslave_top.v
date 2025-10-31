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
`timescale 1ns/1ps
`default_nettype none
// }}}
module sdslave_top #(
		// {{{
		// NUMIO : Controls the number of data pins available on the
		// {{{
		// interface.  eMMC cards can have up to 8 data pins.  SDIO
		// is limited to 4 data pins.  Both have modes that can support
		// a single data pin alone.  Set appropriately based upon your
		// ultimate hardware.
		parameter	NUMIO=4,
		// }}}
		// ADDRESS_WIDTH: Number of bits to the DMA's address lines,
		// {{{
		// as required to access octets of memory.  This is not the word
		// address width, but the octet/byte address width.
		parameter	ADDRESS_WIDTH=48,
		localparam	AW = ADDRESS_WIDTH,
		// }}}
		// Bus data width, DW: Bit-width of the bus, number of bits that
		// {{{
		// can be transferred across the bus (by the DMA) in any one
		// clock cycle.
		parameter	DW=64
		// }}}
		// OPT_EMMC: Enables eMMC support.  There are subtle differences
		// {{{
		// between the eMMC protocol and the SDIO protocol.  This
		// enables the extra features required by eMMC.  These extra
		// features still need to be implemented in a to-be-written
		// emmcsfsm.v module.
		// parameter [0:0]	OPT_EMMC=1,
		// }}}
		// OPT_DS: Data strobe support
		// {{{
		// eMMC chips include a data strobe pin, which can be used to
		// clock return values.  Set this parameter true to support
		// sampling based upon this data strobe.  Beware that you may
		// need additional timing constraints to make this work.
		// parameter [0:0]	OPT_DS= OPT_EMMC,
		// }}}
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		// Control interface
		//	Slaves don't have a control interface
		// DMA interface
		// {{{
`ifdef	SDSLAVE_AXI
		// (Optional) AXI (full) interface
		// {{{
		output	wire			M_AXI_AWVALID,
		input	wire			M_AXI_AWREADY,
		output	wire [AXI_IW-1:0]	M_AXI_AWID,
		output	wire [AW-1:0]		M_AXI_AWADDR,
		output	wire [7:0]		M_AXI_AWLEN,
		output	wire [2:0]		M_AXI_AWSIZE,
		output	wire [1:0]		M_AXI_AWBURST,
		output	wire 			M_AXI_AWLOCK,
		output	wire [3:0]		M_AXI_AWCACHE,
		output	wire	[2:0]		M_AXI_AWPROT,
		output	wire [3:0]		M_AXI_AWQOS,
		//
		output	wire			M_AXI_WVALID,
		input	wire			M_AXI_WREADY,
		output	wire [DW-1:0]		M_AXI_WDATA,
		output	wire [DW/8-1:0]		M_AXI_WSTRB,
		output	wire			M_AXI_WLAST,
		//
		input	wire			M_AXI_BVALID,
		output	wire			M_AXI_BREADY,
		input	wire [AXI_IW-1:0]	M_AXI_BID,
		input	wire	[1:0]		M_AXI_BRESP,
		//
		output	wire			M_AXI_ARVALID,
		input	wire			M_AXI_ARREADY,
		output	wire [AXI_IW-1:0]	M_AXI_ARID,
		output	wire [AW-1:0]		M_AXI_ARADDR,
		output	wire [7:0]		M_AXI_ARLEN,
		output	wire [2:0]		M_AXI_ARSIZE,
		output	wire [1:0]		M_AXI_ARBURST,
		output	wire 			M_AXI_ARLOCK,
		output	wire [3:0]		M_AXI_ARCACHE,
		output	wire	[2:0]		M_AXI_ARPROT,
		output	wire [3:0]		M_AXI_ARQOS,
		//
		input	wire			M_AXI_RVALID,
		output	wire			M_AXI_RREADY,
		input	wire [AXI_IW-1:0]	M_AXI_RID,
		input	wire [DW-1:0]		M_AXI_RDATA,
		input	wire			M_AXI_RLAST,
		input	wire	[1:0]		M_AXI_RRESP,
		// }}}
`else
		output	wire			o_dma_cyc, o_dma_stb, o_dma_we,
		output	wire	[AW-1:0]	o_dma_addr,
		output	wire	[DW-1:0]	o_dma_data,
		output	wire	[DW/8-1:0]	o_dma_sel,
		input	wire			i_dma_stall,
		input	wire			i_dma_ack,
		input	wire	[DW-1:0]	i_dma_data,
		input	wire			i_dma_err,
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
	wire		tx_cmd, cmd_tristate, tx_ds;
	wire	[15:0]	tx_data;
	wire	[7:0]	tx_tristate;

	wire		rx_cmd, w_collision;
	wire	[15:0]	rx_data;
	wire	[1:0]	w_ds;
	wire		w_ds_tristate;
	// }}}

	sdslave #(
		// {{{
		.NUMIO(NUMIO), .AW(AW), .DW(DW)
		// , .OPT_DDR(OPT_DDR)
		// .OPT_DS(OPT_DS),
		// .OPT_EMMC(OPT_EMMC),
		// .OPT_1P8V(OPT_1P8V),
		// }}}
	) u_sdslave (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// AXI DMA interface
		// {{{
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
		//
		.M_AXI_WVALID(M_AXI_WVALID),
		.M_AXI_WREADY(M_AXI_WREADY),
		.M_AXI_WDATA(M_AXI_WDATA),
		.M_AXI_WSTRB(M_AXI_WSTRB),
		.M_AXI_WLAST(M_AXI_WLAST),
		//
		//
		.M_AXI_BVALID(M_AXI_BVALID),
		.M_AXI_BREADY(M_AXI_BREADY),
		.M_AXI_BID(M_AXI_BID),
		.M_AXI_BRESP(M_AXI_BRESP),
		//
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
		//
		.M_AXI_RVALID(M_AXI_RVALID),
		.M_AXI_RREADY(M_AXI_RREADY),
		.M_AXI_RID(M_AXI_RID),
		.M_AXI_RDATA(M_AXI_RDATA),
		.M_AXI_RLAST(M_AXI_RLAST),
		.M_AXI_RRESP(M_AXI_RRESP),
		// }}}
`else
		.o_dma_cyc(o_dma_cyc),
		.o_dma_stb(o_dma_stb),
		.o_dma_we(o_dma_we),
		.o_dma_addr(o_dma_addr),
		.o_dma_data(o_dma_data),
		.o_dma_sel(o_dma_sel),
		.i_dma_stall(i_dma_stall),
		.i_dma_ack(i_dma_ack),
		.i_dma_data(i_dma_data),
		.i_dma_err(i_dma_err),
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
		.NUMIO(NUMIO) // , .OPT_DS(OPT_DS)
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
			.i_tx_ds(tx_ds),
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
