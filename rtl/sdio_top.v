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
		parameter	LGFIFO = 12,
		// NUMIO : Controls the number of data pins available on the
		// {{{
		// interface.  eMMC cards can have up to 8 data pins.  SDIO
		// is limited to 4 data pins.  Both have modes that can support
		// a single data pin alone.  Set appropriately based upon your
		// ultimate hardware.
		parameter	NUMIO=4,
		// }}}
		localparam	MW=32,	// Bus width
		parameter [0:0]	OPT_EMMC=1,
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
		// OPT_1P8V
		// {{{
		// Some protocols require switching voltages during the
		// handshaking process in order to switch to higher speeds.
		// OPT_1P8V enables a GPIO output which can be used to
		// request that the voltage switch to 1.8V from 3.3V.  No
		// feedback is provided, it simply controls a GPIO pin that is
		// assumed to command a 1.8V output.
		parameter [0:0]	OPT_1P8V=1,
		// }}}
		// OPT_COLLISION
		// {{{
		// eMMC is (supposed to) support IRQ (interrupts).  An
		// interrupt command is supposed to end with the device sending
		// a command reply.  However, the host may pre-empt this.  If
		// both attempt to send a reply at the same time, there might be
		// a collision.  OPT_COLLISION adds logic to detect such
		// collisions and to get off the bus should any such happen.
		// Detecting collisions requires a solid knowledge internal to
		// the front end about the delay through the system, to avoid
		// false alarms.
		parameter [0:0]	OPT_COLLISION=OPT_EMMC,
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
`ifdef	SDIO_AXIL
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
`ifdef	SDIO_AXIL
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
		.OPT_DS(OPT_DS), .OPT_COLLISION(OPT_COLLISION)
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
