////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdio_top.v
// {{{
// Project:	SD-Card controller
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
module sdio_top #(
		// {{{
		// LGFIFO controls the size of the internal FIFOs (in bytes).
		// {{{
		// This also control the maximum amount of data that can be
		// sent or received in any one burst.
		parameter	LGFIFO = 12,
		// }}}
		// NUMIO : Controls the number of data pins available on the
		// {{{
		// interface.  eMMC cards can have up to 8 data pins.  SDIO
		// is limited to 4 data pins.  Both have modes that can support
		// a single data pin alone.  Set appropriately based upon your
		// ultimate hardware.
		parameter	NUMIO=4,
		// }}}
		localparam	MW=32,	// Bus width.  Do not change.
		// ADDRESS_WIDTH: Number of bits to the DMA's address lines,
		// {{{
		// as required to access octets of memory.  This is not the word
		// address width, but the octet/byte address width.
		parameter	ADDRESS_WIDTH=48,
		// }}}
		// Bus data width, DW: Bit-width of the bus, number of bits that
		// {{{
		// can be transferred across the bus (by the DMA) in any one
		// clock cycle.
		parameter	DW=64,
		// }}}
		// Stream data width, SW: Bit-width of the AXI stream inputs and
		// {{{
		// outputs of the IP (when used).  Only some bitwidths are
		// fully supported.  Set to 32 or DW for full support.
		parameter	SW=32,
		// }}}
		// OPT_DMA: Set to 1 to include the DMA in the design
		parameter [0:0]	OPT_DMA = 1'b0,
`ifdef	SDIO_AXI
		// AXI_IW: ID width of the AXI interface
		parameter	AXI_IW= 1,
		parameter [AXI_IW-1:0]	AXI_WRITE_ID= 1'b0,
		parameter [AXI_IW-1:0]	AXI_READ_ID = 1'b0,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b1,
		localparam	AW = ADDRESS_WIDTH,
`else
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		localparam	AW = ADDRESS_WIDTH-$clog2(DW/8),
`endif
		parameter	HWDELAY=0,
		// OPT_ISTREAM: Enable an incoming AXI stream to specify data
		// {{{
		// to the DMA, separate from any data that may be read from
		// memory.  This allows streaming applications to bypass the
		// memory interface and go directly to the SD card/EMMC chip
		// if desired.
		parameter [0:0]	OPT_ISTREAM=1'b0,
		// }}}
		// OPT_OSTREAM: Same as OPT_ISTREAM, but for outputs from the
		// {{{
		// SD card/eMMC chip.
		parameter [0:0]	OPT_OSTREAM=1'b0,
		// }}}
		// OPT_EMMC: Enables eMMC support.  There are subtle differences
		// {{{
		// between the eMMC protocol and the SDIO protocol.  This
		// enables the extra features required by eMMC.  These extra
		// features include self-replies to IRQ (interrupt) commands,
		// and data strobe support.
		parameter [0:0]	OPT_EMMC=1,
		// }}}
		// OPT_SERDES && OPT_DDR
		// {{{
		// Three front-end options are available:
		// OPT_DDR == OPT_SERDES == 0: Slowest front end, but requires
		//	no specialized hardware.  May only go up to the system
		//	clock frequency (nominally 100MHz) divided by four.
		// OPT_SERDES == 0, OPT_DDR = 1: A bit faster.  Transitions on
		//	both system clock edges.  May go up to the system
		//	clock frequency divided by two, or nominally 50 MHz.
		//	Will supports DDR data at this rate--assuming the device
		//	does.
		// OPT_SERDES == 1: Fastest clock frequency.  Depends upon
		//	I/OSERDES support.  Support may not be available in all
		//	FPGA vendors.  In this mode, the outgoing clock may
		//	get as high as twice the system clock rate, or nominally
		//	200MHz.  DDR data is supported at this rate.
		parameter [0:0]	OPT_SERDES=0,
		parameter [0:0]	OPT_DDR=1,
		// }}}
		// OPT_DS: Data strobe support
		// {{{
		// eMMC chips include a data strobe pin, which can be used to
		// clock return values.  Set this parameter true to support
		// sampling based upon this data strobe.  Beware that you may
		// need additional timing constraints to make this work.
		parameter [0:0]	OPT_DS=OPT_SERDES && OPT_EMMC,
		// }}}
		parameter [0:0]	OPT_CARD_DETECT=!OPT_EMMC,
		// OPT_CRCTOKEN : Look for a CRC token following every blk write
		// {{{
		// CRC tokens are returned by both eMMC and SD card devices
		// following block writes from the host to the card.  The token
		// tells the host whether or not the block was written validly
		// or not.
		//
		// At one time, I thought this these tokens were optional, then
		// that they were only on eMMC devices.  The parameter was built
		// so I could first have that optional support, then so that
		// support could be optionally configured in.  Now I understand
		// both eMMC and SD card devices use these toksn.  Therefore,
		// this parameter should be set and left.
		parameter [0:0]	OPT_CRCTOKEN=1'b1,
		// }}}
		// OPT_HWRESET
		// {{{
		// eMMC cards can have hardware resets.  SD Cards do not.  Set
		// OPT_HWRESET to include control logic to set the hardware
		// reset.
		parameter [0:0]	OPT_HWRESET = OPT_EMMC,
		// }}}
		// OPT_1P8V
		// {{{
		// Some protocols require switching voltages during the
		// handshaking process in order to switch to higher speeds.
		// OPT_1P8V enables a GPIO output which can be used to
		// request that external hardware (not part of this package)
		// switch the IO voltage to 1.8V from 3.3V.  No feedback is
		// provided, it simply controls a GPIO pin that is assumed to
		// command a 1.8V output.
		parameter [0:0]	OPT_1P8V=1,
		// }}}
		// OPT_COLLISION
		// {{{
		// eMMC is (supposed to) support IRQ (interrupts).  A specific
		// command tells the eMMC chip to wait for an internal interrupt
		// before responding to the command.  Hence, this interrupt
		// command is supposed to end with the device sending a command
		// reply.  However, according to protocol the host may pre-empt
		// this wait by replying to its own command.  If, however,
		// both attempt to send a command reply at the same time, there
		// is a possibility for a collision.  OPT_COLLISION adds logic
		// to detect such collisions and to get off the bus should any
		// such happen.  Detecting collisions requires a solid
		// knowledge internal to the front end about the delay through
		// the system, to avoid false alarms.
		//
		// NOTE: Collisions detection does not (currently) work with
		//   OPT_SERDES.
		parameter [0:0]	OPT_COLLISION=OPT_EMMC && !OPT_SERDES,
		// }}}
		// LGTIMEOUT
		// {{{
		// Controls how long to wait for a request from the device
		// before giving up with a timeout error.  LGTIMEOUT=23 will
		// wait for (approximately) 2^23 or 8-Million clock cycles.
		// If the device doesn't respond in this time, an internal
		// timeout will be generated, and the command will
		// fail--allowing software the opportunity to attempt to
		// resurrect the interface or give up on it at software's
		// choice.  Examples of what might cause such a timeout failure
		// include an unplugged card, a malfunctioning card, or a
		// bad board connection.
		parameter	LGTIMEOUT = 23
		// }}}
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
		output	wire		o_hwreset_n,
		output	wire		o_1p8v,
		input	wire		i_1p8v,
		output	wire		o_int,
		output	wire	[31:0]	o_debug
		// }}}
	);

	// Local declarations
	// {{{
	wire		cfg_ddr, cfg_ds, cfg_dscmd;
	wire	[4:0]	cfg_sample_shift;
	wire	[7:0]	sdclk;
	wire		w_crcack, w_crcnak;
		//
	wire		cmd_en, cmd_collision, cmd_tristate;
	wire	[1:0]	cmd_data;
		//
	wire		data_en, data_tristate, rx_en;
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
		.OPT_CRCTOKEN(OPT_CRCTOKEN),
		.OPT_HWRESET(OPT_HWRESET),
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
		.o_hwreset_n(o_hwreset_n),
		.o_1p8v(o_1p8v), .i_1p8v(i_1p8v),
		.o_int(o_int),
		// Interface to PHY
		// {{{
		.o_cfg_ddr(cfg_ddr), .o_cfg_ds(cfg_ds), .o_cfg_dscmd(cfg_dscmd),
		.o_cfg_sample_shift(cfg_sample_shift),
		.o_sdclk(sdclk),
		//
		.o_cmd_en(cmd_en), .o_cmd_tristate(cmd_tristate),
		.o_cmd_data(cmd_data),
		//
		.o_data_en(data_en), .o_data_tristate(data_tristate),
			.o_rx_en(rx_en),
		.o_tx_data(tx_data),
		//
		.i_cmd_strb(rply_strb), .i_cmd_data(rply_data),
			.i_cmd_collision(cmd_collision),
		.i_card_busy(card_busy),
		.i_rx_strb(rx_strb),
		.i_rx_data(rx_data),
		.i_crcack(w_crcack), .i_crcnak(w_crcnak),
		//
		.S_AC_VALID(AC_VALID), .S_AC_DATA(AC_DATA),
		.S_AD_VALID(AD_VALID), .S_AD_DATA(AD_DATA)
		// }}}
		// }}}
	);

	sdfrontend #(
		// {{{
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR), .NUMIO(NUMIO),
		.OPT_DS(OPT_DS), .OPT_COLLISION(OPT_COLLISION),
		.OPT_CRCTOKEN(OPT_CRCTOKEN), .HWBIAS(HWDELAY),
		.BUSY_CLOCKS(OPT_CRCTOKEN ? 16 : 4)
		// }}}
	) u_sdfrontend (
		// {{{
		.i_clk(i_clk),.i_hsclk(i_hsclk && OPT_SERDES),.i_reset(i_reset),
		.i_cfg_ddr(cfg_ddr), .i_cfg_ds(cfg_ds), .i_cfg_dscmd(cfg_dscmd),
		.i_sample_shift(cfg_sample_shift),
		// Tx path
		// {{{
		// MSB "first" incoming data.
		.i_sdclk(sdclk),
		//
		.i_cmd_en(cmd_en), .i_cmd_tristate(cmd_tristate),
			.i_cmd_data(cmd_data), .o_data_busy(card_busy),
		//
		.i_data_en(data_en), .i_data_tristate(data_tristate),
			.i_tx_data(tx_data),
		// }}}
		// Synchronous Rx path
		// {{{
		.i_rx_en(rx_en),
		.o_cmd_strb(rply_strb),
		.o_cmd_data(rply_data),
		.o_cmd_collision(cmd_collision),
		//
		.o_crcack(w_crcack), .o_crcnak(w_crcnak),
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
