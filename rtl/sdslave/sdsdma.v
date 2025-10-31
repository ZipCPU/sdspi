////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdsdma.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	SD slave DMA.  Handles both WB -> TX and RX -> WB conversions.
//		Requires the FSM to set up our signaling.
//
// Definitions:
//	Host	The SDIO master, not us.
//	Dev	The SDIO slave--that's us.
//	Read/write are from the perspective of the *host* (SDIO master)
//		RX is a Write transaction, a HOST2DEV transfer.
//		TX is a Read transaction,  a DEV2HOST transfer.
//	Since these definitions are rather confusing, I shall use the
//	the localparams D_HOST2DEV and D_DEV2HOST to enumerate the two possible
//	directions.
//
// Assumptions:
//	1. The (external) FSM (sdsfsm.v) operates on the SD clock, configures
//		transfers, and so transfers require active SD clocks.  Further,
//		all of our configuration flags will come on the SD clock, and
//		may (will) need to be crossed from there to the bus (WB) clock.
//	2. The SDIO master may stop the SD clock at any time, pausing any
//		ongoing operations
//	3. The SDIO master may stop a transfer at any time--whether read or
//		write, although it is most likely to stop a WRITE (Host->DEV)
//		transfer between operations (because it can), and READ
//		(DEV->HOST) operations ... whenever.
//
// Components:
//	RxGears
//		A gearbox.  Takes incoming data, at 32b words, and builds full
//		bus sized words from it.
//	TxGears
//		Opposite direction gearbox.  Takes incoming data, at full
//		bus width size, and converts it to 32b word outputs.
//	AFIFO	Used to cross clock domains, either from SD to bus (WB), or
//		bus (WB) to SD clock domains.  In general, we'll try to keep
//		these asynchronous FIFO's small and low logic in this module.
//	SFIFO	A basic synchronous FIFO.  Must hold a minimum of one block of
//		the maximum transfer size.  Ideally, this should hold two blocks
//		of data at the maximum transfer size.
//
//		Our SFIFO will run on the bus (WB) clock.
//
//	S2MM
//	MM2S
//
//
// So ... how do we do this?
	// RX Data path: sdsrxframe -> rxgears -> AFIFO -> FIFO(sys) -> S2MM
	// TX Data path: MM2S -> FIFO(sys) -> AFIFO -> txgears -> sdstxframe
	//
	// Abort ... clears the TX path--mid packet if need be, and aborts any
	//   ongoing RX path operations.
	// RX CRC err ... should abort the incoming RX packet and all packets
	//   following.  This also means that the packet shouldn't leave the
	//   FIFO for the S2MM until the CRC comes in good.
//
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
`default_nettype none
`timescale 1ns/1ps
// }}}
module	sdsdma #(
		// {{{
		parameter	BUS_WIDTH = 64,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		parameter	ADDRESS_WIDTH=31,
		parameter	LGMAXBLKSZ = 12,	// 4kB
		// Abbreviations
		localparam	DW = BUS_WIDTH,
		localparam	AW = ADDRESS_WIDTH-$clog2(DW/8)
		// }}}
	) (
		// {{{
		input	wire	i_wb_clk, i_wb_reset,
		input	wire	i_sd_clk, i_sd_reset, i_sd_softreset,
		// CFG interface
		// {{{
		input	wire		i_cfg_valid,
		output	wire		o_cfg_ready,
		// Command interface
		input	wire		i_sd_request, i_sd_dir, i_sd_abort,
		input	wire	[3:0]	i_sd_lglen,
		input	wire	[ADDRESS_WIDTH-1:0]	i_sd_addr,
		//
		output	wire		o_s2mm_done, o_s2mm_err,
		output	wire		o_mm2s_done, o_mm2s_err,
		// }}}
		// SD interface
		// {{{
		// These arrive on the SD card clock
		input	wire		i_rx_valid,
		// output wire		o_rx_ready	// *MUST* be one
		input	wire	[31:0]	i_rx_data,
		input	wire		i_rx_last,
		// i_rx_good, i_rx_err -- not used here
		//
		// The TX interface is also on the SD card's clock
		output	wire		o_tx_valid,
		input	wire		i_tx_ready, // rx_gear_ready
		output	wire	[31:0]	o_tx_data,
		output	wire		o_tx_last,
		// }}}
		// DMA Wishbone interface
		// {{{
		output	wire			o_dma_cyc, o_dma_stb, o_dma_we,
		output	wire	[AW-1:0]	o_dma_addr,
		output	wire	[DW-1:0]	o_dma_data,
		output	wire	[DW/8-1:0]	o_dma_sel,
		input	wire			i_dma_stall, i_dma_ack,
		input	wire	[DW-1:0]	i_dma_data,
		input	wire			i_dma_err
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam [1:0]	SZ_BUS = 2'b00,
				SZ_32B = 2'b01;
	localparam [0:0]	D_DEV2HOST = 1'b0, D_HOST2DEV = 1'b1;
	// Contain two copies of the max block size
	localparam	WBLSB = $clog2(DW/8);
	localparam	LGFLEN = 1+LGMAXBLKSZ-WBLSB;

	reg				r_rtn_valid, r_dma_err, r_dma_busy,
					r_dma_done;
	wire				sd_rtn_ready, sd_rtn_valid,
					dma_done, dma_err,
					wb_softreset,
					wb_rtn_valid, wb_rtn_ready,
					wb_cfg_valid, wb_cfg_ready;

	wire				wb_request, wb_dir, wb_abort;
	wire	[3:0]			wb_lglen;
	wire	[ADDRESS_WIDTH-1:0]	wb_dma_addr;

	wire			mm2s_valid, mm2s_last,
				mm2s_request, mm2s_busy, mm2s_err;
	wire	[DW-1:0]	mm2s_data;
	wire	[WBLSB:0]	mm2s_bytes;

	wire			ign_rx_ready;
	wire			rx_gear_valid, rx_gear_last, ign_rx_afifo_full;
	wire	[DW-1:0]	rx_gear_data;
	wire	[WBLSB:0]	rx_gear_bytes;

	wire			rx_afifo_last, rx_afifo_empty;
	wire	[DW-1:0]	rx_afifo_data;
	wire	[WBLSB:0]	rx_afifo_bytes;

	wire			sfifo_rd;
	wire			ign_fifo_full;
	wire	[LGFLEN:0]	ign_fifo_fill;
	wire			sfifo_last, sfifo_empty;
	wire	[DW-1:0]	sfifo_data;
	wire	[WBLSB:0]	sfifo_bytes;

	wire			tx_afifo_full, tx_afifo_last,
				tx_afifo_empty;
	wire	[DW-1:0]	tx_afifo_data;
	wire	[WBLSB:0]	tx_afifo_bytes;

	wire			tx_gear_ready;
	wire	[DW-1:0]	wide_tx_data;
	wire	[WBLSB:0]	ign_tx_bytes;

	wire			rd_cyc, wr_cyc, rd_stb, wr_stb,
				rd_we, wr_we;
	wire	[AW-1:0]	rd_addr, wr_addr;
	wire	[DW/8-1:0]	rd_sel, wr_sel;

	wire	[WBLSB:0]	sz_32b;
	wire	[DW-1:0]	ign_rd_data;
	reg	[DW-1:0]	wide_rx_data;

	wire			s2mm_request, s2mm_busy, s2mm_err,
				s2mm_ready;

	reg	r_s2mm_busy, r_mm2s_busy, r_s2mm_done, r_mm2s_done;
	reg	r_s2mm_err, r_mm2s_err;
	wire	s2mm_done, mm2s_done;

	wire	sd_s2mm_done, sd_s2mm_err, sd_mm2s_done, sd_mm2s_err;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Cross parameters across clock domains
	// {{{

	// SD clock   -> WB clock
	////////////////////
	// i_sd_request -> wb_request
	// i_sd_lglen   -> wb_lglen
	// i_sd_addr    -> wb_dma_addr
	// i_sd_dir     -> wb_dir
	// i_sd_abort   -> wb_abort

	sdtfrvalue #(
		.W(4+4+ADDRESS_WIDTH)
		// .DEFAULT({(7+ADDRESS_WIDTH){1'b0}})
	) u_tfr2wb (
		// {{{
		.i_a_clk(i_sd_clk), .i_a_reset_n(!i_sd_reset),
		.i_a_valid(i_cfg_valid), .o_a_ready(o_cfg_ready),
		.i_a_data({ i_sd_request, i_sd_dir, i_sd_abort, i_sd_softreset,
					i_sd_lglen, i_sd_addr }),
		//
		.i_b_clk(i_wb_clk), .i_b_reset_n(!i_wb_reset),
		.o_b_valid(wb_cfg_valid), .i_b_ready(wb_cfg_ready),
		.o_b_data({ wb_request, wb_dir, wb_abort, wb_softreset,
						wb_lglen, wb_dma_addr })
		// }}}
	);

	assign	wb_cfg_ready = 1'b1;
	assign	mm2s_request = wb_request && wb_dir == D_DEV2HOST;
	assign	s2mm_request = wb_request && wb_dir == D_HOST2DEV;

	// WB clock -> SD clock
	////////////////////
	// (mm2s_err || s2mm_err) -> o_sd_err
	// (mm2s_busy || s2mm_busy) -> o_sd_busy

	initial	r_rtn_valid = 1'b0;
	always @(posedge i_wb_clk)
	if (i_wb_reset)
		r_rtn_valid <= 1'b0;
	else if (wb_rtn_valid && !wb_rtn_ready)
		r_rtn_valid <= 1'b1;
	else if (wb_rtn_ready)
		r_rtn_valid <= 1'b0;

	assign	wb_rtn_valid = r_rtn_valid
				|| (r_mm2s_err  != mm2s_err)
				|| (r_s2mm_err  != s2mm_err)
				|| (r_mm2s_busy != mm2s_busy)
				|| (r_s2mm_busy != s2mm_busy);

	initial	r_s2mm_err = 1'b0;
	always @(posedge i_wb_clk)
	if (i_wb_reset || wb_softreset || wb_abort)
		r_s2mm_err <= 1'b0;
	else if (s2mm_err && !wb_rtn_ready)
		r_s2mm_err <= 1'b1;
	else if (wb_rtn_ready)
		r_s2mm_err <= 1'b0;

	initial	r_mm2s_err = 1'b0;
	always @(posedge i_wb_clk)
	if (i_wb_reset || wb_softreset || wb_abort)
		r_mm2s_err <= 1'b0;
	else if (mm2s_err && !wb_rtn_ready)
		r_mm2s_err <= 1'b1;
	else if (wb_rtn_ready)
		r_mm2s_err <= 1'b0;

	always @(posedge i_wb_clk)
	if (i_wb_reset || wb_softreset || wb_abort)
		r_s2mm_busy <= 1'b0;
	else
		r_s2mm_busy <= s2mm_busy;

	always @(posedge i_wb_clk)
	if (i_wb_reset || wb_softreset || wb_abort)
		r_mm2s_busy <= 1'b0;
	else
		r_mm2s_busy <= mm2s_busy;

	initial	r_s2mm_done = 1'b0;
	always @(posedge i_wb_clk)
	if (i_wb_reset || wb_softreset || wb_abort)
		r_s2mm_done <= 1'b0;
	else if (wb_rtn_ready)
		r_s2mm_done <= 1'b0;
	else if (r_s2mm_busy && !s2mm_busy)
		r_s2mm_done <= 1'b1;

	initial	r_mm2s_done = 1'b0;
	always @(posedge i_wb_clk)
	if (i_wb_reset || wb_softreset || wb_abort)
		r_mm2s_done <= 1'b0;
	else if (wb_rtn_ready)
		r_mm2s_done <= 1'b0;
	else if (r_mm2s_busy && !mm2s_busy)
		r_mm2s_done <= 1'b1;

	assign	s2mm_done = r_s2mm_done || (r_s2mm_busy && !s2mm_busy);
	assign	mm2s_done = r_mm2s_done || (r_mm2s_busy && !mm2s_busy);

	sdtfrvalue #(
		.W(4)
		// .DEFAULT({(2){1'b0}})
	) u_tfr2sd (
		// {{{
		.i_a_clk(i_wb_clk), .i_a_reset_n(!i_wb_reset),
		.i_a_valid(wb_rtn_valid), .o_a_ready(wb_rtn_ready),
		.i_a_data({ s2mm_done, r_s2mm_err || s2mm_err, mm2s_done, r_mm2s_err || mm2s_err }),
		//
		.i_b_clk(i_sd_clk), .i_b_reset_n(!i_sd_reset),
		.o_b_valid(sd_rtn_valid), .i_b_ready(sd_rtn_ready),
		.o_b_data({ sd_s2mm_done, sd_s2mm_err, sd_mm2s_done, sd_mm2s_err })
		// }}}
	);

	assign	sd_rtn_ready = 1'b1;

	assign	o_s2mm_done = sd_s2mm_done && sd_rtn_valid;
	assign	o_s2mm_err  = sd_s2mm_err  && sd_rtn_valid;
	assign	o_mm2s_done = sd_mm2s_done && sd_rtn_valid;
	assign	o_mm2s_err  = sd_mm2s_err  && sd_rtn_valid;

	// Externally handled
	////////////////////
	// i_wb_reset -> i_sd_reset

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Ingest: sdsrxframe (external) or MM2S
	// {{{
	// RX ingest: RX Gears -> AFIFO
	// {{{

	assign	sz_32b = 4;
	always @(*)
	begin
		wide_rx_data = {(DW){1'b0}};
		wide_rx_data[DW-1:DW-32] = i_rx_data;
	end

	sddma_rxgears #(
		.BUS_WIDTH(DW),
		.OPT_LITTLE_ENDIAN(1'b0)
	) u_rxgears (
		// {{{
		.i_clk(	i_sd_clk),
		.i_reset(i_sd_reset),
		.i_soft_reset(i_sd_softreset),
		//
		.S_VALID(i_rx_valid),
		.S_READY(ign_rx_ready),
		.S_DATA(wide_rx_data),
		.S_BYTES(sz_32b),	// All RXFrame outputs are 32b
		.S_LAST(i_rx_last),
		// i_rx_good, i_rx_err -- not used here
		//
		.M_VALID(rx_gear_valid),
		.M_READY(1'b1), // rx_gear_ready
		.M_DATA(rx_gear_data),
		.M_BYTES(rx_gear_bytes),
		.M_LAST(rx_gear_last)
		// }}}
	);

	afifo #(
		.LGFIFO(3),
		.WIDTH(2+WBLSB+DW)
	) u_rxafifo (
		// {{{
		.i_wclk(	i_sd_clk),
		.i_wr_reset_n(	!i_sd_softreset ),
		.i_wr(	rx_gear_valid),
		.i_wr_data({ rx_gear_last, rx_gear_bytes, rx_gear_data }),
		.o_wr_full(ign_rx_afifo_full),
		//
		.i_rclk(i_wb_clk),
		.i_rd_reset_n(!wb_softreset),
		.i_rd(1'b1),
		.o_rd_data({ rx_afifo_last, rx_afifo_bytes, rx_afifo_data }),
		.o_rd_empty(rx_afifo_empty)
		// }}}
	);
	// }}}

	// WB ingest, MM2S
	// {{{
	reg	[LGMAXBLKSZ:0]	wide_wb_len;

	always @(*)
	begin
		wide_wb_len = 1;
		wide_wb_len = wide_wb_len << wb_lglen;
	end

	sddma_mm2s #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.BUS_WIDTH(DW),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.LGLENGTH(LGMAXBLKSZ)
		// }}}
	) u_mm2s (
		// {{{
		.i_clk(i_wb_clk), .i_reset(i_wb_reset),
		// Configuration -- *must* be in WB clock domain
		// {{{
		.i_request(mm2s_request),
		.o_busy(mm2s_busy),
		.o_err(mm2s_err),
		.i_inc(1'b1),
		.i_size(SZ_BUS),
		.i_transferlen(wide_wb_len),
		.i_addr(wb_dma_addr),
		// }}}
		// Wishbone connections
		// {{{
		.o_rd_cyc(  rd_cyc),
		.o_rd_stb(  rd_stb),
		.o_rd_we(   rd_we),
		.o_rd_addr( rd_addr),
		.o_rd_data( ign_rd_data),
		.o_rd_sel(  rd_sel),
		.i_rd_stall(i_dma_stall),
		.i_rd_ack(  i_dma_ack),
		.i_rd_data( i_dma_data),
		.i_rd_err(  i_dma_err),
		// }}}
		// Stream outputs
		// {{{
		.M_VALID(mm2s_valid),
		.M_READY(1'b1),	// For form, but *must* be one
		.M_DATA(mm2s_data),
		.M_BYTES(mm2s_bytes),	// May be < DW/8 if unaligned first beat
		.M_LAST(mm2s_last)
		// }}}
		// }}}
	);
	// }}}
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// FIFO
	// {{{

	sdfifo #(
		.BW(1+(WBLSB+1)+DW),
		// Always have enough room for 2x blocks, to allow for ping-pong
		.LGFLEN(LGFLEN),
		// If we want to support iCE40 chips, we can't use ASYNC reads
		.OPT_ASYNC_READ(1'b0)
	) u_sfifo (
		.i_clk(i_wb_clk),
		.i_reset(i_wb_reset || wb_softreset),

		.i_wr(mm2s_valid || !rx_afifo_empty),
		.i_data(wb_dir == D_DEV2HOST
			? { mm2s_last, mm2s_bytes, mm2s_data }
			: { rx_afifo_last, rx_afifo_bytes, rx_afifo_data }),
		.o_full(ign_fifo_full),	// -- Handled by FSM
		.o_fill(ign_fifo_fill),

		.i_rd(sfifo_rd),
		.o_data({ sfifo_last, sfifo_bytes, sfifo_data }),
		.o_empty(sfifo_empty)
	);

	// Normally, we'd set an mm2s_ready.  However, when using WB, there's
	//  no ability to offer any backpressure on the returns.  Therefore,
	//  we must make certain befoer starting any transaction that the
	//  FIFO will have sufficient room in it.
	// assign mm2s_ready = mm2s_busy && !fifo_full;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Exfil: S2MM or sdstxframe
	// {{{

	assign	sfifo_rd = (wb_dir == D_DEV2HOST) ? !tx_afifo_full : s2mm_ready;

	// TX exfil: AFIFO -> TX Gears -> [TxFrame external]
	// {{{

	afifo #(
		.LGFIFO(3),
		.WIDTH(2+WBLSB+DW)
	) u_txafifo (
		// {{{
		.i_wclk(i_wb_clk), .i_wr_reset_n( !wb_softreset ),
		//
		.i_wr(sfifo_rd && wb_dir == D_DEV2HOST && !sfifo_empty),
			.i_wr_data({ sfifo_last, sfifo_bytes, sfifo_data }),
		.o_wr_full(tx_afifo_full),
		//
		.i_rclk(i_sd_clk), .i_rd_reset_n(!i_sd_softreset),
			.i_rd(tx_gear_ready),
		.o_rd_data({ tx_afifo_last, tx_afifo_bytes, tx_afifo_data }),
		.o_rd_empty(tx_afifo_empty)
		// }}}
	);

	sddma_txgears #(
		.BUS_WIDTH(DW),
		.OPT_LITTLE_ENDIAN(1'b0)
	) u_txgears (
		// {{{
		.i_clk(	i_sd_clk),
		// In TxGears, there's no difference between reset and soft
		//  reset.  Both control the same things
		.i_reset( i_sd_reset || i_sd_softreset || i_sd_abort
						|| i_sd_dir==D_HOST2DEV),
		.i_soft_reset( i_sd_dir == D_HOST2DEV ),
		.i_size(SZ_32B),	// SZ_32B = 2'b01
		//
		.S_VALID(!tx_afifo_empty),
		.S_READY(tx_gear_ready),
		.S_DATA(tx_afifo_data),
		.S_BYTES(tx_afifo_bytes),	// All RXFrame outputs are 32b
		.S_LAST(tx_afifo_last),
		// i_rx_good, i_rx_err -- not used here
		//
		.M_VALID(o_tx_valid),
		.M_READY(i_tx_ready), // rx_gear_ready
		.M_DATA(wide_tx_data),
		.M_BYTES(ign_tx_bytes),	// Not used, must be 32b
		.M_LAST(o_tx_last)
		// }}}
	);

	assign	o_tx_data = wide_tx_data[DW-1:DW-32];

	// }}}

	// S2MM exfil
	// {{{
	sddma_s2mm #(
		// {{{
		.ADDRESS_WIDTH(AW + WBLSB),
		.BUS_WIDTH(DW),
		.OPT_LITTLE_ENDIAN(1'b0)
		// .LGPIPE
		// .DW, .AW
		//  }}}
	) u_s2mm (
		// {{{
		.i_clk(i_wb_clk), .i_reset(i_wb_reset || wb_softreset),
		// Config
		// {{{
		.i_request(s2mm_request),
		.o_busy(s2mm_busy),
		.o_err(s2mm_err),
		.i_inc(1'b1), .i_size(SZ_BUS), .i_addr(wb_dma_addr),
		// }}}
		// Stream
		// {{{
		.S_VALID(	!sfifo_empty && wb_dir == D_HOST2DEV ),
		.S_READY(	s2mm_ready ),
		.S_DATA(	sfifo_data ),
		.S_BYTES(	sfifo_bytes ),
		.S_LAST(	sfifo_last ),
		// }}}
		// Wishbone connections
		// {{{
		.o_wr_cyc(  wr_cyc),
		.o_wr_stb(  wr_stb),
		.o_wr_we(   wr_we),
		.o_wr_addr( wr_addr),
		.o_wr_data( o_dma_data),
		.o_wr_sel(  wr_sel),
		.i_wr_stall(i_dma_stall),
		.i_wr_ack(  i_dma_ack),
		.i_wr_data( {(DW){1'b0}}),
		.i_wr_err(  i_dma_err)
		// }}}
		// }}}
	);
	// }}}
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Arbiter
	// {{{
	// NOTE: This is *NOT* a proper arbiter.  This will only work if/because
	// only one of the S2MM|MM2S DMAs are operating at a time.
	assign	o_dma_cyc = rd_cyc || wr_cyc;
	assign	o_dma_stb = rd_stb || wr_stb;
	assign	o_dma_we  = wr_cyc;
	// assign	o_dma_data= wr_data;
	assign	o_dma_addr= rd_cyc ? rd_addr : wr_addr;
	assign	o_dma_sel = rd_cyc ? rd_sel : wr_sel;
	// }}}

	// Keep Verilator happy
	// {{{
	wire	unused;
	assign	unused = &{ 1'b0, wb_cfg_valid, ign_rx_ready, ign_rx_afifo_full,
				ign_fifo_full, ign_fifo_fill, ign_tx_bytes,
				ign_rd_data, rd_we, wr_we, wide_tx_data,
				sd_rtn_valid
			 };
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
	reg			f_past_valid;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);
`endif
// }}}
endmodule
