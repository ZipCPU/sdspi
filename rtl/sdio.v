////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdio.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Top level module (minus the front end) for the SDIO controller.
//		Contains submodules, but very little logic.
//
// Submodules:
//	sdwb:		Wishbone bus handler.  Also includes FIFO memory.
//	sdckgen:	Digital clock divider
//	sdcmd:		Request commands, process responses
//	sdrxframe:	Process receive frames into memory writes
//	sdtxframe:	Process memory reads into frames to transmit
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
`default_nettype	none
// }}}
module	sdio #(
		// {{{
		parameter	LGFIFO = 15,//	= log_2(FIFO size in bytes)
				NUMIO=4,
		parameter	MW = 32,
		parameter [0:0]	OPT_DMA = 1'b0,
		parameter	ADDRESS_WIDTH = 30,
		parameter	DMA_DW = 32,	// DMA bus width
		// DMA_AW: The DMA address connection width
`ifdef	SDIO_AXI
		parameter	AXI_IW = 1,	// ID width
		parameter [AXI_IW-1:0]	AXI_READ_ID = 1'b0,
		parameter [AXI_IW-1:0]	AXI_WRITE_ID = 1'b0,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b1,
		parameter	DMA_AW = ADDRESS_WIDTH,
`else
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		parameter	DMA_AW = ADDRESS_WIDTH-$clog2(DMA_DW/8),
`endif
		// To support more than one bit of IO per clock, we need
		//  serdes support.  Setting OPT_SERDES to zero will disable
		//  that support, effectively limiting our operation to 50MHz
		//  from a 100MHz clock.
		parameter [0:0]	OPT_SERDES = 1'b0,
		parameter [0:0]	OPT_DDR = 1'b0,
		parameter [0:0]	OPT_DS  = OPT_SERDES,
		parameter [0:0]	OPT_EMMC = 1'b1,
		parameter [0:0]	OPT_HWRESET = OPT_EMMC,
		parameter [0:0]	OPT_1P8V= 1'b0,
		parameter [0:0]	OPT_CARD_DETECT = !OPT_EMMC,
		parameter [0:0]	OPT_CRCTOKEN = 1'b1,
		parameter	LGTIMEOUT = 23,
		parameter [0:0]	OPT_ISTREAM = 0, OPT_OSTREAM = 0,
		parameter	SW = 32
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Control interface
		// {{{
`ifdef	SDIO_AXI
		// AXI-Lite
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
		// {{{
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[2:0]	i_wb_addr,
		input	wire [MW-1:0]	i_wb_data,
		input	wire [MW/8-1:0]	i_wb_sel,
		//
		output	wire		o_wb_stall, o_wb_ack,
		output	wire [MW-1:0]	o_wb_data,
		// }}}
`endif
		// }}}
		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// DMA AXI interface
		// {{{
		output	wire			M_AXI_AWVALID,
		input	wire			M_AXI_AWREADY,
		output	wire	[AXI_IW-1:0]	M_AXI_AWID,
		output	wire	[DMA_AW-1:0]	M_AXI_AWADDR,
		output	wire	[7:0]		M_AXI_AWLEN,
		output	wire	[2:0]		M_AXI_AWSIZE,
		output	wire	[1:0]		M_AXI_AWBURST,
		output	wire			M_AXI_AWLOCK,
		output	wire	[3:0]		M_AXI_AWCACHE,
		output	wire	[2:0]		M_AXI_AWPROT,
		output	wire	[3:0]		M_AXI_AWQOS,
		//
		output	wire			M_AXI_WVALID,
		input	wire			M_AXI_WREADY,
		output	wire	[DMA_DW-1:0]	M_AXI_WDATA,
		output	wire	[DMA_DW/8-1:0]	M_AXI_WSTRB,
		output	wire			M_AXI_WLAST,
		//
		input	wire			M_AXI_BVALID,
		input	wire	[AXI_IW-1:0]	M_AXI_BID,
		output	wire			M_AXI_BREADY,
		input	wire	[1:0]		M_AXI_BRESP,
		//
		output	wire			M_AXI_ARVALID,
		input	wire			M_AXI_ARREADY,
		output	wire	[AXI_IW-1:0]	M_AXI_ARID,
		output	wire	[DMA_AW-1:0]	M_AXI_ARADDR,
		output	wire	[7:0]		M_AXI_ARLEN,
		output	wire	[2:0]		M_AXI_ARSIZE,
		output	wire	[1:0]		M_AXI_ARBURST,
		output	wire			M_AXI_ARLOCK,
		output	wire	[3:0]		M_AXI_ARCACHE,
		output	wire	[2:0]		M_AXI_ARPROT,
		output	wire	[3:0]		M_AXI_ARQOS,
		//
		//
		input	wire			M_AXI_RVALID,
		output	wire			M_AXI_RREADY,
		input	wire	[AXI_IW-1:0]	M_AXI_RID,
		input	wire	[DMA_DW-1:0]	M_AXI_RDATA,
		input	wire			M_AXI_RLAST,
		input	wire	[1:0]		M_AXI_RRESP,
		// }}}
`else
		// DMA (Wishbone) interface
		// {{{
		output	wire			o_dma_cyc, o_dma_stb, o_dma_we,
		output	wire [DMA_AW-1:0]	o_dma_addr,
		output	wire [DMA_DW-1:0]	o_dma_data,
		output	wire [DMA_DW/8-1:0]	o_dma_sel,
		//
		input	wire			i_dma_stall, i_dma_ack,
		input	wire [DMA_DW-1:0]	i_dma_data,
		input	wire			i_dma_err,
		// }}}
`endif
		// }}}
		// External stream interface
		// {{{
		input	wire			s_valid,
		output	wire			s_ready,
		input	wire	[SW-1:0]	s_data,

		output	wire			m_valid,
		input	wire			m_ready,
		output	wire	[SW-1:0]	m_data,
		output	wire			m_last,
		// }}}
		input	wire		i_card_detect,
		output	wire		o_hwreset_n,
		output	wire		o_1p8v,
		input	wire		i_1p8v,
		output	wire		o_int,
		// Interface to PHY
		// {{{
		// Not these wires--those will be the connections handled by
		// the PHY
		// inout	wire		io_cmd,
		// inout	wire		io_ds,
		// inout wire [NUMIO-1:0]	io_dat,
		// But these ones ...
		output	wire		o_cfg_ddr, o_cfg_ds, o_cfg_dscmd,
		output	wire	[4:0]	o_cfg_sample_shift,
		output	reg	[7:0]	o_sdclk,
		//
		output	wire		o_cmd_en, o_cmd_tristate,
		output	wire	[1:0]	o_cmd_data,
		//
		output	wire		o_data_en, o_data_tristate, o_rx_en,
		output	wire	[31:0]	o_tx_data,
		//
		input	wire	[1:0]	i_cmd_strb, i_cmd_data,
		input	wire		i_cmd_collision,
		input	wire		i_card_busy,
		input	wire	[1:0]	i_rx_strb,
		input	wire	[15:0]	i_rx_data,
		input	wire		i_crcack, i_crcnak,
		//
		input	wire		S_AC_VALID,
		input	wire	[1:0]	S_AC_DATA,
		input	wire		S_AD_VALID,
		input	wire	[31:0]	S_AD_DATA
		// }}}
		// output	reg	[31:0]	o_debug
		// }}}
	);

	// Local declarations
	// {{{
	wire			soft_reset;

	wire			cfg_clk90, cfg_clk_shutdown, cfg_expect_ack,
				cfg_cmd_pp, cfg_data_pp;
	wire	[7:0]		cfg_ckspeed;
	wire	[1:0]		cfg_width;

	wire			clk_stb, clk_half, clk_clk90;
	wire	[7:0]		clk_wide, clk_ckspd;

	wire			cmd_request, cmd_err, cmd_busy, cmd_done;
	wire			cmd_selfreply;
	wire	[1:0]		cmd_type, cmd_ercode;
	wire			rsp_stb;
	wire	[6:0]		cmd_id;
	wire	[5:0]		rsp_id;
	wire	[31:0]		cmd_arg, rsp_arg;
	wire			cmd_mem_valid;
	wire	[MW/8-1:0]	cmd_mem_strb;
	wire	[LGFIFO-$clog2(MW/8)-1:0]	cmd_mem_addr;
	wire	[MW-1:0]	cmd_mem_data;

	wire			tx_en, tx_mem_valid, tx_mem_ready, tx_mem_last;
	wire	[31:0]		tx_mem_data;

	wire			crc_en;
	wire	[LGFIFO:0]	rx_length;
	wire			rx_mem_valid;
	wire	[LGFIFO-$clog2(MW/8)-1:0]	rx_mem_addr;
	wire	[MW/8-1:0]	rx_mem_strb;
	wire	[MW-1:0]	rx_mem_data;
	wire			rx_done, rx_err, rx_ercode, rx_active, rx_en;
	wire			tx_done, tx_err, tx_ercode;

	// wire	[31:0]		w_debug;

	// DMA declarations
	// {{{
	wire		dma_sd2s, sd2s_valid, sd2s_ready, sd2s_last;
	wire	[31:0]	sd2s_data;
		//
	wire		dma_s2sd, s2sd_valid, s2sd_ready;
	wire	[31:0]	s2sd_data;
		//
	wire			dma_busy, dma_abort, dma_err;
	wire	[ADDRESS_WIDTH+((OPT_ISTREAM||OPT_OSTREAM) ? 1:0)-1:0] dma_addr;
	wire	[LGFIFO:0]	dma_len;
	// }}}
	// }}}

`ifdef	SDIO_AXI
	sdaxil #(
		// {{{
		.LGFIFO(LGFIFO), .NUMIO(NUMIO),
		.OPT_DMA(OPT_DMA),
		.DMA_AW(ADDRESS_WIDTH + ((OPT_ISTREAM||OPT_OSTREAM) ? 1:0)),
		.OPT_SERDES(OPT_SERDES),
		.OPT_DDR(OPT_DDR),
		.OPT_DS(OPT_DS),
		.OPT_CARD_DETECT(OPT_CARD_DETECT),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.OPT_EMMC(OPT_EMMC),
			.OPT_HWRESET(OPT_HWRESET), .OPT_1P8V(OPT_1P8V),
		.OPT_STREAM(OPT_ISTREAM || OPT_OSTREAM),
		.OPT_CRCTOKEN(OPT_CRCTOKEN)
		// }}}
	) u_control (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
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
		// Configuration options
		// {{{
		.o_cfg_clk90(cfg_clk90), .o_cfg_ckspeed(cfg_ckspeed),
		.o_cfg_shutdown(cfg_clk_shutdown),
		.o_cfg_width(cfg_width), .o_cfg_ds(o_cfg_ds),
			.o_cfg_dscmd(o_cfg_dscmd), .o_cfg_ddr(o_cfg_ddr),
		.o_pp_cmd(cfg_cmd_pp), .o_pp_data(cfg_data_pp), // Push-pull
		.o_cfg_sample_shift(o_cfg_sample_shift),
		.o_cfg_expect_ack(cfg_expect_ack),
		.i_ckspd(clk_ckspd),
		// }}}
		.o_soft_reset(soft_reset),
		// DMA interface
		// {{{
		.o_dma_sd2s(dma_sd2s),
		.o_sd2s_valid(sd2s_valid),
		.i_sd2s_ready(sd2s_ready),
		.o_sd2s_data(sd2s_data),
		.o_sd2s_last(sd2s_last),
		//
		.o_dma_s2sd(dma_s2sd),
		.i_s2sd_valid(OPT_DMA && s2sd_valid),
		.o_s2sd_ready(s2sd_ready),
		.i_s2sd_data(s2sd_data),
		//
		.o_dma_addr(dma_addr),
		.o_dma_len(dma_len),
		.i_dma_busy(OPT_DMA && dma_busy),
		.i_dma_err(OPT_DMA && dma_err),
		.o_dma_abort(dma_abort),
		// }}}
		// CMD control interface
		// {{{
		.o_cmd_request(cmd_request), .o_cmd_type(cmd_type),
		.o_cmd_selfreply(cmd_selfreply),
		.o_cmd_id(cmd_id), .o_arg(cmd_arg),
		//
		.i_cmd_busy(cmd_busy), .i_cmd_done(cmd_done),
			.i_cmd_err(cmd_err), .i_cmd_ercode(cmd_ercode),
		//
		.i_cmd_response(rsp_stb), .i_resp(rsp_id),
			.i_arg(rsp_arg),
		//
		.i_cmd_mem_valid(cmd_mem_valid), .i_cmd_mem_strb(cmd_mem_strb),
			.i_cmd_mem_addr(cmd_mem_addr),
			.i_cmd_mem_data(cmd_mem_data),
		// }}}
		// TX interface
		// {{{
		.o_tx_en(tx_en),
		//
		.o_tx_mem_valid(tx_mem_valid),
			.i_tx_mem_ready(tx_mem_ready && tx_en),
		.o_tx_mem_data(tx_mem_data), .o_tx_mem_last(tx_mem_last),
		.i_tx_done(tx_done), .i_tx_err(tx_err), .i_tx_ercode(tx_ercode),
		// }}}
		// RX interface
		// {{{
		.o_rx_en(rx_en), .o_crc_en(crc_en), .o_length(rx_length),
		//
		.i_rx_mem_valid(rx_mem_valid), .i_rx_mem_strb(rx_mem_strb),
			.i_rx_mem_addr(rx_mem_addr),.i_rx_mem_data(rx_mem_data),
		//
		.i_rx_done(rx_done), .i_rx_err(rx_err), .i_rx_ercode(rx_ercode),
		// }}}
		.i_card_detect(i_card_detect),
		.i_card_busy(i_card_busy),
		.o_hwreset_n(o_hwreset_n),
		.o_1p8v(o_1p8v), .i_1p8v(i_1p8v),
		.o_int(o_int)
		// }}}
	);
`else
	sdwb #(
		// {{{
		.LGFIFO(LGFIFO), .NUMIO(NUMIO),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.OPT_SERDES(OPT_SERDES),
		.OPT_DS(OPT_DS),
		.OPT_DDR(OPT_DDR),
		.OPT_CARD_DETECT(OPT_CARD_DETECT),
		.OPT_DMA(OPT_DMA),
		.DMA_AW(ADDRESS_WIDTH + ((OPT_ISTREAM||OPT_OSTREAM) ? 1:0)),
		.OPT_EMMC(OPT_EMMC),
			.OPT_HWRESET(OPT_HWRESET), .OPT_1P8V(OPT_1P8V),
		.OPT_STREAM(OPT_ISTREAM || OPT_OSTREAM),
		.OPT_CRCTOKEN(OPT_CRCTOKEN)
		// }}}
	) u_control (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// Wishbone slave
		// {{{
		.i_wb_cyc(i_wb_cyc), .i_wb_stb(i_wb_stb),
		.i_wb_we(i_wb_we), .i_wb_addr(i_wb_addr),
		.i_wb_data(i_wb_data), .i_wb_sel(i_wb_sel),
		//
		.o_wb_stall(o_wb_stall), .o_wb_ack(o_wb_ack),
			.o_wb_data(o_wb_data),
		// }}}
		// Configuration options
		// {{{
		.o_cfg_clk90(cfg_clk90), .o_cfg_ckspeed(cfg_ckspeed),
		.o_cfg_shutdown(cfg_clk_shutdown),
		.o_cfg_width(cfg_width), .o_cfg_ds(o_cfg_ds),
			.o_cfg_dscmd(o_cfg_dscmd), .o_cfg_ddr(o_cfg_ddr),
		.o_pp_cmd(cfg_cmd_pp), .o_pp_data(cfg_data_pp), // Push-pull
		.o_cfg_sample_shift(o_cfg_sample_shift),
		.o_cfg_expect_ack(cfg_expect_ack),
		.i_ckspd(clk_ckspd),
		// }}}
		.o_soft_reset(soft_reset),
		// DMA interface
		// {{{
		.o_dma_sd2s(dma_sd2s),
		.o_sd2s_valid(sd2s_valid),
		.i_sd2s_ready(sd2s_ready),
		.o_sd2s_data(sd2s_data),
		.o_sd2s_last(sd2s_last),
		//
		.o_dma_s2sd(dma_s2sd),
		.i_s2sd_valid(s2sd_valid),
		.o_s2sd_ready(s2sd_ready),
		.i_s2sd_data(s2sd_data),
		//
		.o_dma_addr(dma_addr),
		.o_dma_len(dma_len),
		.i_dma_busy(dma_busy),
		.i_dma_err(dma_err),
		.o_dma_abort(dma_abort),
		// }}}
		// CMD control interface
		// {{{
		.o_cmd_request(cmd_request), .o_cmd_type(cmd_type),
		.o_cmd_selfreply(cmd_selfreply),
		.o_cmd_id(cmd_id), .o_arg(cmd_arg),
		//
		.i_cmd_busy(cmd_busy), .i_cmd_done(cmd_done),
			.i_cmd_err(cmd_err), .i_cmd_ercode(cmd_ercode),
		//
		.i_cmd_response(rsp_stb), .i_resp(rsp_id),
			.i_arg(rsp_arg),
		//
		.i_cmd_mem_valid(cmd_mem_valid), .i_cmd_mem_strb(cmd_mem_strb),
			.i_cmd_mem_addr(cmd_mem_addr),
			.i_cmd_mem_data(cmd_mem_data),
		// }}}
		// TX interface
		// {{{
		.o_tx_en(tx_en),
		//
		.o_tx_mem_valid(tx_mem_valid),
			.i_tx_mem_ready(tx_mem_ready && tx_en),
		.o_tx_mem_data(tx_mem_data), .o_tx_mem_last(tx_mem_last),
		.i_tx_done(tx_done), .i_tx_err(tx_err), .i_tx_ercode(tx_ercode),
		// }}}
		// RX interface
		// {{{
		.o_rx_en(rx_en), .o_crc_en(crc_en), .o_length(rx_length),
		//
		.i_rx_mem_valid(rx_mem_valid), .i_rx_mem_strb(rx_mem_strb),
			.i_rx_mem_addr(rx_mem_addr),.i_rx_mem_data(rx_mem_data),
		//
		.i_rx_done(rx_done), .i_rx_err(rx_err), .i_rx_ercode(rx_ercode),
		// }}}
		.i_card_detect(i_card_detect),
		.i_card_busy(i_card_busy),
		.o_hwreset_n(o_hwreset_n),
		.o_1p8v(o_1p8v), .i_1p8v(i_1p8v),
		.o_int(o_int)
		// .o_debug(w_debug)
		// }}}
	);

	/*
	always @(posedge i_clk)
	begin
		o_debug <= w_debug;

		if (!w_debug[0]) // !o_rx_en
		begin
			o_debug[3] <= i_crcnak;	// i_rx_err
			o_debug[4] <= i_crcack;	// rx_done

			// if (o_data_en)
			//	o_debug[4:0] <= { 1'b1, o_tx_data[27:24] };
		end // if (o_rx_en && i_rx_strb[1])
		//	o_debug[19:15] <= { 1'b1, i_rx_data[11:8] };
	end
	*/
`endif

	assign	o_rx_en = rx_en && rx_active;

	sdckgen #(
		.OPT_SERDES(OPT_SERDES),
		.OPT_DDR(OPT_DDR)
	) u_clkgen (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		//
		.i_cfg_clk90(cfg_clk90), .i_cfg_ckspd(cfg_ckspeed),
		.i_cfg_shutdown(cfg_clk_shutdown && !rx_active),

		.o_ckstb(clk_stb), .o_hlfck(clk_half), .o_ckwide(clk_wide),
		.o_clk90(clk_clk90), .o_ckspd(clk_ckspd)
		// }}}
	);

	sdcmd #(
		// {{{
		.OPT_DS(OPT_DS),
		.OPT_EMMC(OPT_EMMC),
		.OPT_SERDES(OPT_SERDES),
		.MW(MW),
		.LGLEN(LGFIFO-$clog2(MW/8))
		// }}}
	) u_sdcmd (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || soft_reset),
		//
		.i_cfg_ds(o_cfg_dscmd), .i_cfg_dbl(cfg_ckspeed == 0),
		.i_cfg_pp(cfg_cmd_pp),
		.i_ckstb(clk_stb),
		//
		.i_cmd_request(cmd_request), .i_cmd_type(cmd_type),
		.i_cmd_selfreply(cmd_selfreply),
		.i_cmd(cmd_id), .i_arg(cmd_arg),
		//
		.o_busy(cmd_busy), .o_done(cmd_done), .o_err(cmd_err),
			.o_ercode(cmd_ercode),
		//
		.o_cmd_en(o_cmd_en), .o_cmd_data(o_cmd_data),
			.o_cmd_tristate(o_cmd_tristate),
		.i_cmd_strb(i_cmd_strb), .i_cmd_data(i_cmd_data),
			.i_cmd_collision(i_cmd_collision),
		.S_ASYNC_VALID(S_AC_VALID), .S_ASYNC_DATA(S_AC_DATA),
		//
		.o_cmd_response(rsp_stb), .o_resp(rsp_id),
			.o_arg(rsp_arg),
		//
		.o_mem_valid(cmd_mem_valid), .o_mem_strb(cmd_mem_strb),
			.o_mem_addr(cmd_mem_addr), .o_mem_data(cmd_mem_data)
		// }}}
	);

	sdtxframe #(
		// {{{
		.OPT_SERDES(OPT_SERDES || OPT_DDR),
		.OPT_CRCTOKEN(OPT_CRCTOKEN),
		.NUMIO(NUMIO)
		// .MW(MW)
		// }}}
	) u_txframe (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || soft_reset),
		//
		.i_cfg_spd(clk_ckspd),
		.i_cfg_width(cfg_width),
		.i_cfg_ddr(o_cfg_ddr),
		.i_cfg_pp(cfg_data_pp),
		.i_cfg_expect_ack(cfg_expect_ack),
		// Extra insights, for assertion checking
		.i_cfg_clk90(clk_clk90),
		.i_ckwide(clk_wide),
		//
		.i_en(tx_en), .i_ckstb(clk_stb), .i_hlfck(clk_half),
		//
		.S_VALID(tx_en && tx_mem_valid), .S_READY(tx_mem_ready),
		.S_DATA(tx_mem_data), .S_LAST(tx_mem_last),
		//
		.tx_valid(o_data_en), .tx_data(o_tx_data),
			.tx_tristate(o_data_tristate),
		.i_crcack(i_crcack && OPT_CRCTOKEN),
			.i_crcnak(i_crcnak && OPT_CRCTOKEN),
		.o_done(tx_done), .o_err(tx_err), .o_ercode(tx_ercode)
		// }}}
	);

	sdrxframe #(
		// {{{
		.OPT_DS(OPT_SERDES), .NUMIO(NUMIO),
		.LGLEN(LGFIFO),
		.MW(MW),
		.LGTIMEOUT(LGTIMEOUT)
		// }}}
	) u_rxframe (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || soft_reset),
		//
		.i_cfg_ddr(o_cfg_ddr),
		.i_cfg_ds(o_cfg_ds), .i_cfg_width(cfg_width),
		.i_rx_en(rx_en), .i_crc_en(crc_en), .i_length(rx_length),
		//
		.i_rx_strb(i_rx_strb), .i_rx_data(i_rx_data),
		.S_ASYNC_VALID(S_AD_VALID), .S_ASYNC_DATA(S_AD_DATA),
		//
		.o_mem_valid(rx_mem_valid), .o_mem_strb(rx_mem_strb),
			.o_mem_addr(rx_mem_addr), .o_mem_data(rx_mem_data),
		//
		.o_done(rx_done), .o_err(rx_err), .o_ercode(rx_ercode),
		.o_active(rx_active)
		// }}}
	);

	// Delay the clock by one cycle, to match the data
	// {{{
	// The following criteria are really redundant.  We could just set
	// o_sdclk to clk_wide in all conditions.  The extra criteria are here
	// to simply help ensure minimum logic where available, and to help
	// guarantee that minimum logic means the same to the tool as it does
	// to me.  It's also here as a bit of reminder-documentation of what
	// our various OPT* criteria can generate and handle.
	initial	o_sdclk = 0;
	always @(posedge i_clk)
	if (OPT_SERDES)
		// Could be one of: 0x66, 0x33, 0x3c, 0x0f, 0x00, or 0xff
		o_sdclk <= clk_wide;
	else if (OPT_DDR)
		// One of 0x0f, 0xf0, 0x00, or 0xff
		o_sdclk <= { {(4){clk_wide[7]}}, {(4){clk_wide[3]}} };
	else
		// Can only be one of 0x00 or 0x0ff
		o_sdclk <= {(8){ clk_wide[7] }};
	// }}}

	////////////////////////////////////////////////////////////////////////
	//
	// DMA
	// {{{
	generate if (OPT_DMA)
	begin : GEN_DMA

		sddma #(
			// {{{
`ifdef	SDIO_AXI
			.AXI_IW(AXI_IW),	// ID width
			.AXI_READ_ID(AXI_READ_ID),
			.AXI_WRITE_ID(AXI_WRITE_ID),
`endif
			.LGFIFO(LGFIFO),
			.ADDRESS_WIDTH(ADDRESS_WIDTH),
			.DW(DMA_DW),
			.OPT_ISTREAM(OPT_ISTREAM),
			.OPT_OSTREAM(OPT_OSTREAM),
			.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
			.SW(SW)
			// }}}
		) u_sddma (
			// {{{
			.i_clk(i_clk), .i_reset(i_reset),
			.i_soft_reset(soft_reset),
			.i_dma_sd2s(dma_sd2s), .i_dma_s2sd(dma_s2sd),
			//
			.i_dma_addr(dma_addr), .i_dma_len(dma_len),
			.o_dma_busy(dma_busy), .o_dma_err(dma_err),
			.i_dma_abort(dma_abort),
			//
			.i_sd2s_valid(sd2s_valid), .o_sd2s_ready(sd2s_ready),
			.i_sd2s_data(sd2s_data), .i_sd2s_last(sd2s_last),
			//
			.o_s2sd_valid(s2sd_valid), .i_s2sd_ready(s2sd_ready),
			.o_s2sd_data(s2sd_data),
			//
			.s_valid(s_valid), .s_ready(s_ready), .s_data(s_data),
			.m_valid(m_valid), .m_ready(m_ready), .m_data(m_data),
				.m_last(m_last),
`ifdef	SDIO_AXI
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
			.M_AXI_RRESP(M_AXI_RRESP)
`else
			.o_dma_cyc(o_dma_cyc), .o_dma_stb(o_dma_stb),
				.o_dma_we(o_dma_we),
			.o_dma_addr(o_dma_addr), .o_dma_data(o_dma_data),
				.o_dma_sel(o_dma_sel),
			//
			.i_dma_stall(i_dma_stall),
			.i_dma_ack(i_dma_ack), .i_dma_data(i_dma_data),
			.i_dma_err(i_dma_err)
`endif
			// }}}
		);

	end else begin : NO_DMA
		// {{{
		assign	sd2s_ready = 1'b1;

		assign	s2sd_valid = 1'b0;
		assign	s2sd_data  = 32'h0;

		assign	s_ready = 1'b1;
		assign	m_valid = 1'b0;
		assign	m_data  = 0;
		assign	m_last  = 1'b0;

		assign	dma_busy = 1'b0;
		assign	dma_err  = 1'b0;

		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// DMA AXI-Lite interface
		// {{{
		assign	M_AXI_AWVALID = 1'b0;
		assign	M_AXI_AWID    = {(AXI_IW){1'b0}};
		assign	M_AXI_AWADDR  = {(DMA_AW){1'b0}};
		assign	M_AXI_AWLEN   = 8'h0;
		assign	M_AXI_AWSIZE  = 3'h0;
		assign	M_AXI_AWBURST = 2'h0;
		assign	M_AXI_AWLOCK  = 1'h0;
		assign	M_AXI_AWCACHE = 4'h0;
		assign	M_AXI_AWPROT  = 3'h0;
		assign	M_AXI_AWQOS   = 4'h0;

		assign	M_AXI_WVALID  = 1'b0;
		assign	M_AXI_WDATA   = {(DMA_DW){1'b0}};
		assign	M_AXI_WSTRB   = {(DMA_DW/8){1'b0}};
		assign	M_AXI_WLAST   = 1'b0;
		//
		assign	M_AXI_BREADY  = 1'b1;
		//
		assign	M_AXI_ARVALID = 1'b0;
		assign	M_AXI_ARID    = {(AXI_IW){1'b0}};
		assign	M_AXI_ARADDR  = {(DMA_AW){1'b0}};
		assign	M_AXI_ARLEN   = 8'h0;
		assign	M_AXI_ARSIZE  = 3'h0;
		assign	M_AXI_ARBURST = 2'h0;
		assign	M_AXI_ARLOCK  = 1'h0;
		assign	M_AXI_ARCACHE = 4'h0;
		assign	M_AXI_ARPROT  = 3'h0;
		assign	M_AXI_ARQOS   = 4'h0;

		assign	M_AXI_RREADY  = 1'b1;
		// }}}
`else
		// DMA (Wishbone) interface
		// {{{
		assign	{ o_dma_cyc, o_dma_stb, o_dma_we } = 3'b0;
		assign	o_dma_addr = {(DMA_AW){1'b0}};
		assign	o_dma_data = {(DMA_DW){1'b0}};
		assign	o_dma_sel  = {(DMA_DW/8){1'b0}};
		// }}}
`endif
		// }}}

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_dma;
		assign	unused_dma = &{ 1'b0,
`ifdef	SDIO_AXI
				M_AXI_AWREADY, M_AXI_WREADY, M_AXI_ARREADY, 
				M_AXI_BVALID, M_AXI_BID, M_AXI_BRESP,
				M_AXI_RVALID, M_AXI_RID, M_AXI_RDATA,
					M_AXI_RLAST, M_AXI_RRESP,
`else
				i_dma_stall, i_dma_ack, i_dma_data, i_dma_err,
`endif
				dma_sd2s, dma_s2sd,
				sd2s_valid, sd2s_last, sd2s_data,
				dma_abort, dma_addr, dma_len, dma_err, dma_busy,
				s2sd_ready,
				s_valid, s_data, m_ready
				};
		// Verilator lint_on  UNUSED
		// }}}
		// }}}
	end endgenerate
	// }}}

	//
	// Make verilator happy
	// {{{
	// verilator coverage_off
	// verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0 };
	// verilator lint_on  UNUSED
	// verilator coverage_on
	// }}}
endmodule

