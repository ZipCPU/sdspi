////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdio_top.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	A top level file for both eMMC and SDIO controllers.  This
//		file references both architecture specific modules in
//	sdfrontend.v, and non-architecture specific logic via sdio.v.
//	Otherwise, the top level (non-architecture specific) module would be
//	sdio.v.
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
`timescale 1ns/1ps
`default_nettype none
// }}}
module sdio_top #(
		// {{{
		parameter	LGFIFO = 12, NUMIO=4, MW=32,
		parameter	ADDRESS_WIDTH=48,
		parameter	DW=64, SW=32,
		parameter [0:0]	OPT_DMA = 1'b0,
`ifdef	SDIO_AXI
		parameter	AXI_IW= 1,
		parameter [AXI_IW-1:0]	AXI_WRITE_ID= 1'b0,
		parameter [AXI_IW-1:0]	AXI_READ_ID = 1'b0,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b1,
		localparam	AW = ADDRESS_WIDTH,
`else
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		localparam	AW = ADDRESS_WIDTH-$clog2(DW/8),
`endif
		parameter [0:0]	OPT_ISTREAM=1'b0,
		parameter [0:0]	OPT_OSTREAM=1'b0,
		parameter [0:0]	OPT_EMMC=1,
		parameter [0:0]	OPT_SERDES=0,
		parameter [0:0]	OPT_DDR=1,
		parameter [0:0]	OPT_DS=OPT_SERDES && OPT_EMMC,
		parameter [0:0]	OPT_CARD_DETECT=!OPT_EMMC,
		parameter [0:0]	OPT_1P8V=1,
		parameter	LGTIMEOUT = 23
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset, i_hsclk,
		// Control interface
		// {{{
`ifdef	SDIO_AXI
		// (Optional) AXI-Lite interface
		// {{{
		input	wire		S_AXIL_AWVALID,
		output	wire		S_AXIL_AWREADY,
		input	wire	[4:0]	S_AXIL_AWADDR,
		input	wire	[2:0]	S_AXIL_AWPROT,
		//
		input	wire		S_AXIL_WVALID,
		output	wire		S_AXIL_WREADY,
		input	wire	[31:0]	S_AXIL_WDATA,
		input	wire	[3:0]	S_AXIL_WSTRB,
		//
		output	wire		S_AXIL_BVALID,
		input	wire		S_AXIL_BREADY,
		output	wire	[1:0]	S_AXIL_BRESP,
		//
		input	wire		S_AXIL_ARVALID,
		output	wire		S_AXIL_ARREADY,
		input	wire	[4:0]	S_AXIL_ARADDR,
		input	wire	[2:0]	S_AXIL_ARPROT,
		//
		output	wire		S_AXIL_RVALID,
		input	wire		S_AXIL_RREADY,
		output	wire	[31:0]	S_AXIL_RDATA,
		output	wire	[1:0]	S_AXIL_RRESP,
		// }}}
`else
		// Control (Wishbone) interface
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[2:0]	i_wb_addr,
		input	wire [MW-1:0]	i_wb_data,
		input	wire [MW/8-1:0]	i_wb_sel,
		//
		output	wire		o_wb_stall, o_wb_ack,
		output	wire [MW-1:0]	o_wb_data,
`endif
		// }}}
		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// (Optional) AXI-Lite interface
		// {{{
		output	wire		M_AXI_AWVALID,
		input	wire		M_AXI_AWREADY,
		output	wire [AXI_IW-1:0]	M_AXI_AWID,
		output	wire [AW-1:0]	M_AXI_AWADDR,
		output	wire [7:0]	M_AXI_AWLEN,
		output	wire [2:0]	M_AXI_AWSIZE,
		output	wire [1:0]	M_AXI_AWBURST,
		output	wire 		M_AXI_AWLOCK,
		output	wire [3:0]	M_AXI_AWCACHE,
		output	wire	[2:0]	M_AXI_AWPROT,
		output	wire [3:0]	M_AXI_AWQOS,
		//
		output	wire		M_AXI_WVALID,
		input	wire		M_AXI_WREADY,
		output	wire [DW-1:0]	M_AXI_WDATA,
		output	wire [DW/8-1:0]	M_AXI_WSTRB,
		output	wire		M_AXI_WLAST,
		//
		input	wire		M_AXI_BVALID,
		output	wire		M_AXI_BREADY,
		input	wire [AXI_IW-1:0]	M_AXI_BID,
		input	wire	[1:0]	M_AXI_BRESP,
		//
		output	wire		M_AXI_ARVALID,
		input	wire		M_AXI_ARREADY,
		output	wire [AXI_IW-1:0]	M_AXI_ARID,
		output	wire [AW-1:0]	M_AXI_ARADDR,
		output	wire [7:0]	M_AXI_ARLEN,
		output	wire [2:0]	M_AXI_ARSIZE,
		output	wire [1:0]	M_AXI_ARBURST,
		output	wire 		M_AXI_ARLOCK,
		output	wire [3:0]	M_AXI_ARCACHE,
		output	wire	[2:0]	M_AXI_ARPROT,
		output	wire [3:0]	M_AXI_ARQOS,
		//
		input	wire		M_AXI_RVALID,
		output	wire		M_AXI_RREADY,
		input	wire [AXI_IW-1:0]	M_AXI_RID,
		input	wire [DW-1:0]	M_AXI_RDATA,
		input	wire		M_AXI_RLAST,
		input	wire	[1:0]	M_AXI_RRESP,
		// }}}
`else
		output	wire		o_dma_cyc, o_dma_stb, o_dma_we,
		output	wire	[AW-1:0]	o_dma_addr,
		output	wire	[DW-1:0]	o_dma_data,
		output	wire	[DW/8-1:0]	o_dma_sel,
		input	wire			i_dma_stall,
		input	wire			i_dma_ack,
		input	wire	[DW-1:0]	i_dma_data,
		input	wire			i_dma_err,
`endif
		// }}}
		// External DMA streaming interface
		// {{{
		input	wire			s_valid,
		output	wire			s_ready,
		input	wire	[SW-1:0]	s_data,
		//
		output	wire			m_valid,
		input	wire			m_ready,
		output	wire	[SW-1:0]	m_data,
		output	wire			m_last,
		// }}}
		// IO interface
		// {{{
		output	wire			o_ck,
		input	wire			i_ds,
`ifdef	VERILATOR
		output	wire			io_cmd_tristate,
		output	wire			o_cmd,
		input	wire			i_cmd,
		//
		output	wire	[NUMIO-1:0]	io_dat_tristate,
		output	wire	[NUMIO-1:0]	o_dat,
		input	wire	[NUMIO-1:0]	i_dat,
`else
		inout	wire			io_cmd,
		inout	wire	[NUMIO-1:0]	io_dat,
`endif
		// }}}
		input	wire		i_card_detect,
		output	wire		o_1p8v,
		output	wire		o_int,
		output	wire	[31:0]	o_debug
		// }}}
	);

	// Local declarations
	// {{{
	wire		cfg_ddr, cfg_ds, cfg_dscmd;
	wire	[4:0]	cfg_sample_shift;
	wire	[7:0]	sdclk;
		//
	wire		cmd_en, pp_cmd, cmd_collision;
	wire	[1:0]	cmd_data;
		//
	wire		data_en, pp_data, rx_en;
	wire	[31:0]	tx_data;
		//
	wire	[1:0]	rply_strb, rply_data;
	wire		card_busy;
	wire	[1:0]	rx_strb;
	wire	[15:0]	rx_data;
		//
	wire		AC_VALID;
	wire	[1:0]	AC_DATA;
	wire		AD_VALID;
	wire	[31:0]	AD_DATA;
	// }}}

	sdio #(
		// {{{
		.LGFIFO(LGFIFO), .NUMIO(NUMIO), .MW(MW),
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.DMA_DW(DW), .SW(SW),
		.OPT_DMA(OPT_DMA),
		.OPT_ISTREAM(OPT_ISTREAM), .OPT_OSTREAM(OPT_OSTREAM),
`ifdef	SDIO_AXI
		.AXI_IW(AXI_IW),
		.AXI_WRITE_ID(AXI_WRITE_ID),
		.AXI_READ_ID(AXI_READ_ID),
`endif
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.OPT_DDR(OPT_DDR), .OPT_SERDES(OPT_SERDES),
		.OPT_DS(OPT_DS),
		.OPT_CARD_DETECT(OPT_CARD_DETECT),
		.OPT_EMMC(OPT_EMMC),
		.OPT_1P8V(OPT_1P8V),
		.LGTIMEOUT(LGTIMEOUT)
		// }}}
	) u_sdio (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// Control interface
		// {{{
`ifdef	SDIO_AXI
		// AXI-Lite
		// {{{
		.S_AXIL_AWVALID(S_AXIL_AWVALID),
		.S_AXIL_AWREADY(S_AXIL_AWREADY),
		.S_AXIL_AWADDR(S_AXIL_AWADDR),
		.S_AXIL_AWPROT(S_AXIL_AWPROT),
		//
		.S_AXIL_WVALID(S_AXIL_WVALID),
		.S_AXIL_WREADY(S_AXIL_WREADY),
		.S_AXIL_WDATA(S_AXIL_WDATA),
		.S_AXIL_WSTRB(S_AXIL_WSTRB),
		//
		.S_AXIL_BVALID(S_AXIL_BVALID),
		.S_AXIL_BREADY(S_AXIL_BREADY),
		.S_AXIL_BRESP(S_AXIL_BRESP),
		//
		.S_AXIL_ARVALID(S_AXIL_ARVALID),
		.S_AXIL_ARREADY(S_AXIL_ARREADY),
		.S_AXIL_ARADDR(S_AXIL_ARADDR),
		.S_AXIL_ARPROT(S_AXIL_ARPROT),
		//
		.S_AXIL_RVALID(S_AXIL_RVALID),
		.S_AXIL_RREADY(S_AXIL_RREADY),
		.S_AXIL_RDATA(S_AXIL_RDATA),
		.S_AXIL_RRESP(S_AXIL_RRESP),
		// }}}
`else
		// Control (Wishbone) interface
		// {{{
		.i_wb_cyc(i_wb_cyc), .i_wb_stb(i_wb_stb), .i_wb_we(i_wb_we),
		.i_wb_addr(i_wb_addr),.i_wb_data(i_wb_data),.i_wb_sel(i_wb_sel),
		//
		.o_wb_stall(o_wb_stall), .o_wb_ack(o_wb_ack),
		.o_wb_data(o_wb_data),
		// }}}
`endif
		// }}}
		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// (Optional) AXI-Lite interface
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
		// External DMA streaming interface
		// {{{
		.s_valid(s_valid),
		.s_ready(s_ready),
		.s_data(s_data),
		//
		.m_valid(m_valid),
		.m_ready(m_ready),
		.m_data(m_data),
		.m_last(m_last),
		// }}}
		.i_card_detect(i_card_detect),
		.o_1p8v(o_1p8v),
		.o_int(o_int),
		// Interface to PHY
		// {{{
		.o_cfg_ddr(cfg_ddr), .o_cfg_ds(cfg_ds), .o_cfg_dscmd(cfg_dscmd),
		.o_cfg_sample_shift(cfg_sample_shift),
		.o_sdclk(sdclk),
		//
		.o_cmd_en(cmd_en), .o_pp_cmd(pp_cmd),
		.o_cmd_data(cmd_data),
		//
		.o_data_en(data_en), .o_rx_en(rx_en), .o_pp_data(pp_data),
		.o_tx_data(tx_data),
		//
		.i_cmd_strb(rply_strb), .i_cmd_data(rply_data),
			.i_cmd_collision(cmd_collision),
		.i_card_busy(card_busy),
		.i_rx_strb(rx_strb),
		.i_rx_data(rx_data),
		//
		.S_AC_VALID(AC_VALID), .S_AC_DATA(AC_DATA),
		.S_AD_VALID(AD_VALID), .S_AD_DATA(AD_DATA)
		// }}}
		// }}}
	);

	sdfrontend #(
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR), .NUMIO(NUMIO),
		.OPT_DS(OPT_DS)
	) u_sdfrontend (
		// {{{
		.i_clk(i_clk), .i_hsclk(i_hsclk), .i_reset(i_reset),
		.i_cfg_ddr(cfg_ddr), .i_cfg_ds(cfg_ds), .i_cfg_dscmd(cfg_dscmd),
		.i_sample_shift(cfg_sample_shift),
		// Tx path
		// {{{
		// MSB "first" incoming data.
		.i_sdclk(sdclk),
		//
		.i_cmd_en(cmd_en), .i_pp_cmd(pp_cmd), .i_cmd_data(cmd_data),				.o_data_busy(card_busy),
		//
		.i_data_en(data_en), .i_pp_data(pp_data), .i_tx_data(tx_data),
		// }}}
		// Synchronous Rx path
		// {{{
		.i_rx_en(rx_en),
		.o_cmd_strb(rply_strb),
		.o_cmd_data(rply_data),
		.o_cmd_collision(cmd_collision),
		//
		.o_rx_strb(rx_strb),
		.o_rx_data(rx_data),
		// }}}
		// Async Rx path
		// {{{
		.MAC_VALID(AC_VALID), .MAC_DATA(AC_DATA),
		.MAD_VALID(AD_VALID), .MAD_DATA(AD_DATA),
		// }}}
		// I/O ports
		// {{{
		.o_ck(o_ck), .i_ds(i_ds),
`ifdef	VERILATOR
		.io_cmd_tristate(io_cmd_tristate),
		.o_cmd(o_cmd), .i_cmd(i_cmd),
		//
		.io_dat_tristate(io_dat_tristate),
		.o_dat(o_dat), .i_dat(i_dat),
`else
		.io_cmd(io_cmd),
		.io_dat(io_dat),
`endif
		// }}}
		.o_debug(o_debug)
		// }}}
	);

endmodule
