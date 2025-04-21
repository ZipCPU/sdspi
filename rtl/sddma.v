////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sddma.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Top level of the DMA component.  This is the component that
//		interacts with memory, to load a page to be transmitted to the
//	SD card, or to recover that page from the SD card and send it to
//	memory.  An option exists to use an external stream if desired.
//
//	This DMA is controlled entirely by the command controller, sdwb/sdaxil.
//
//	At present, this DMA controller supports Wishbone alone.
//
// Submodules:
//	sddma_mm2s:
//	sddma_rxgears:
//	sdfifo:
//	sddma_txgears:
//	sddma_s2mm:
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
module	sddma #(
		// {{{
		parameter	ADDRESS_WIDTH=32, // Byte address width
		parameter	DW = 64,	// DMA bus width
`ifdef	SDIO_AXI
		parameter	AXI_IW = 1,
		parameter [AXI_IW-1:0]	AXI_READ_ID  = 0,
		parameter [AXI_IW-1:0]	AXI_WRITE_ID = 0,
		parameter	OPT_LITTLE_ENDIAN = 1'b1,
		localparam	AW = ADDRESS_WIDTH,	// DMA address width
`else
		parameter	OPT_LITTLE_ENDIAN = 1'b0,
		localparam	AW = ADDRESS_WIDTH-$clog2(DW/8),
`endif
		parameter [0:0]	OPT_ISTREAM = 0,
		parameter [0:0]	OPT_OSTREAM = 0,
		parameter [0:0]	OPT_LOWPOWER = 0,
		parameter	LGFIFO = 15,//	= log_2(FIFO size in bytes)
		parameter	SW = 32		// Stream data width
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset, i_soft_reset,
		// Control interface
		// {{{
		input	wire			i_dma_sd2s,
		input	wire			i_dma_s2sd,
		//
		input wire [ADDRESS_WIDTH+((OPT_ISTREAM || OPT_OSTREAM) ? 1:0)-1:0]
							i_dma_addr,
		input	wire	[LGFIFO:0]	i_dma_len,
		output	wire			o_dma_busy,
		output	wire			o_dma_err,
		input	wire			i_dma_abort,
		// }}}
		// Streaming interface to the controller
		// {{{
		input	wire			i_sd2s_valid,
		output	wire			o_sd2s_ready,
		input	wire	[31:0]		i_sd2s_data,
		input	wire			i_sd2s_last,
		//
		output	wire			o_s2sd_valid,
		input	wire			i_s2sd_ready,
		output	wire	[31:0]		o_s2sd_data,
		// }}}
		// External Stream interface
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
		// DMA interface
		// {{{
`ifdef	SDIO_AXI
		// DMA AXI master interface
		// {{{
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
		//
		output	wire			M_AXI_WVALID,
		input	wire			M_AXI_WREADY,
		output	wire	[DW-1:0]	M_AXI_WDATA,
		output	wire	[DW/8-1:0]	M_AXI_WSTRB,
		output	wire			M_AXI_WLAST,
		//
		input	wire			M_AXI_BVALID,
		output	wire			M_AXI_BREADY,
		input	wire	[AXI_IW-1:0]	M_AXI_BID,
		input	wire	[1:0]		M_AXI_BRESP,
		//
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
		//
		input	wire			M_AXI_RVALID,
		output	wire			M_AXI_RREADY,
		input	wire	[AXI_IW-1:0]	M_AXI_RID,
		input	wire	[DW-1:0]	M_AXI_RDATA,
		input	wire			M_AXI_RLAST,
		input	wire	[1:0]		M_AXI_RRESP
		// }}}
`else
		// DMA (Wishbone) interface
		// {{{
		output	wire		o_dma_cyc, o_dma_stb, o_dma_we,
		output	wire [AW-1:0]	o_dma_addr,
		output	wire [DW-1:0]	o_dma_data,
		output	wire [DW/8-1:0]	o_dma_sel,
		//
		input	wire		i_dma_stall, i_dma_ack,
		input	wire [DW-1:0]	i_dma_data,
		input	wire		i_dma_err
		// }}}
`endif
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam	RXWIDTH = ((!OPT_ISTREAM && !OPT_OSTREAM) || DW > SW)
							? DW : SW;
	localparam	[1:0]	SZ_BUS = 2'b00,
				SZ_32B = 2'b01;
	localparam	ADDR_MSB = ADDRESS_WIDTH-1
				+ ((OPT_ISTREAM || OPT_OSTREAM) ? 1:0);
	//

	reg			wide_rx_valid, wide_rx_last;
	reg	[RXWIDTH-1:0]		wide_rx_data;
	reg	[$clog2(RXWIDTH/8):0]	wide_rx_bytes;

	reg	s2sd_busy, sd2s_busy;

`ifndef	SDIO_AXI
	// Pre-WB arbiter bus inputs
	wire			rd_cyc, rd_stb, rd_we;
	wire			wr_cyc, wr_stb, wr_we;
	wire	[AW-1:0]	rd_addr, wr_addr;
	wire	[DW-1:0]	ign_rd_data, wr_data;
	wire	[DW/8-1:0]	rd_sel,  wr_sel;
	wire			rd_stall, rd_ack, wr_stall, wr_ack;
`endif

	wire			mm2s_busy, mm2s_err;
	wire			s2mm_busy, s2mm_err, s2mm_ready;

	wire			mm2s_valid, mm2s_ready, mm2s_last;
	wire	[DW-1:0]	mm2s_data;
	wire [$clog2(DW/8):0]	mm2s_bytes;

	wire			rxgears_ready;


	wire			wide_valid, wide_ready, wide_last;
	wire	[DW-1:0]	wide_data;
	wire	[$clog2(DW/8):0]	wide_bytes;

	wire			fifo_valid, fifo_ready, fifo_last, fifo_empty,
				fifo_full;
	wire	[DW-1:0]	fifo_data;
	wire	[$clog2(DW/8):0]		fifo_bytes;
	wire	[LGFIFO-$clog2(DW/8):0]		ign_fifo_fill;

	wire			tx_valid, tx_ready, tx_last;
	wire	[DW-1:0]	tx_data;
	wire	[$clog2(DW/8):0]	tx_bytes;

	wire			m_active;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// MM2S -- Read from memory to generate an initial data stream
	// {{{

	// sddma_mm2s: Read memory into the RX stream
`ifdef	SDIO_AXI
	sdax_mm2s #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.BUS_WIDTH(DW),
		.AXI_IW(AXI_IW), .AXI_ID(AXI_READ_ID),
		.LGLENGTH(LGFIFO+1),
		// .OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.OPT_LOWPOWER(OPT_LOWPOWER)
		// }}}
	) u_s2sd_dma (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		.i_soft_reset(i_soft_reset),
		.i_request(i_dma_s2sd && (!OPT_ISTREAM || !i_dma_addr[ADDR_MSB])),
		.o_busy(mm2s_busy), .o_err(mm2s_err),
		.i_inc(1'b1), .i_size(SZ_BUS), .i_transferlen(i_dma_len),
		.i_addr(i_dma_addr[ADDRESS_WIDTH-1:0]),
		//
		// AXI Master interface
		// {{{
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
		// AXI Stream master (Memory to SD Card) interface
		// {{{
		.M_AXIS_VALID(mm2s_valid),
		.M_AXIS_READY(mm2s_ready),
		.M_AXIS_DATA(mm2s_data),
		.M_AXIS_BYTES(mm2s_bytes),
		.M_AXIS_LAST(mm2s_last)
		// }}}
		// }}}
	);

`else	// SDIO_AXI
	sddma_mm2s #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.BUS_WIDTH(DW),
		.LGLENGTH(LGFIFO),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.OPT_LOWPOWER(OPT_LOWPOWER)
		// }}}
	) u_s2sd_dma (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || i_soft_reset),
		.i_request(i_dma_s2sd && (!OPT_ISTREAM || !i_dma_addr[ADDR_MSB])),
		.o_busy(mm2s_busy), .o_err(mm2s_err),
		.i_inc(1'b1), .i_size(SZ_BUS), .i_transferlen(i_dma_len),
		.i_addr(i_dma_addr[ADDRESS_WIDTH-1:0]),
		//
		.o_rd_cyc(rd_cyc), .o_rd_stb(rd_stb), .o_rd_we(rd_we),
		.o_rd_addr(rd_addr), .o_rd_data(ign_rd_data), .o_rd_sel(rd_sel),
		.i_rd_stall(rd_stall), .i_rd_ack(rd_ack), .i_rd_data(i_dma_data),
		.i_rd_err(i_dma_err),
		//
		.M_VALID(mm2s_valid),
		.M_READY(mm2s_ready),
		.M_DATA(mm2s_data),
		.M_BYTES(mm2s_bytes),
		.M_LAST(mm2s_last)
		// }}}
	);
`endif
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Incoming data arbitration
	// {{{

	// Data arrives from one of 3 sources:
	//	The MM2S memory DMA
	//	The external AXI Stream
	//	An internal AXI stream from the external SD card/eMMC chip

	// s_last, s_count, s_active
	// {{{
	wire	s_last, s_active;

	generate if (OPT_ISTREAM)
	begin : GEN_ISTREAM
		reg			s_lost, r_last, r_active;
		reg	[LGFIFO:0]	s_count;

		always @(posedge i_clk)
		if (i_reset)
		begin
			r_last <= 0;
			s_lost <= 0;
			s_count <= 0;
			r_active <= 0;
		end else if (s_active)
		begin
			s_lost <= s_lost || i_soft_reset || i_dma_abort;

			if (s_valid && s_ready)
			begin
				s_lost <= (s_lost || i_soft_reset || i_dma_abort) && !s_last;
				s_count  <= s_count - 1;
				r_active <= !s_last;
				r_last   <= (s_count <= 1);
			end
		end else if (i_dma_s2sd)
		begin
			r_last   <= (i_dma_len <= SW/8);
			s_count  <= (i_dma_len >> $clog2(SW/8))-1;
			r_active <= i_dma_addr[ADDR_MSB];
		end

		assign	s_ready = s_active && (rxgears_ready
					|| i_dma_abort || s_lost);
		assign	s_last = r_last;
		assign	s_active = r_active;
	end else begin : NO_ISTREAM

		assign	s_active = 1'b0;
		assign	s_ready  = 1'b0;
		assign	s_last   = 1'b0;
	end endgenerate
	// }}}

	// wide_rx_*
	// {{{
	always @(*)
	begin
		if (sd2s_busy)
		begin
			wide_rx_valid = i_sd2s_valid;
			wide_rx_bytes = 4;
			wide_rx_last  = i_sd2s_last;
		end else if (!mm2s_busy && OPT_ISTREAM)
		begin
			wide_rx_valid = s_valid && s_active;
			// Verilator lint_off WIDTH
			wide_rx_bytes = SW/8;
			// Verilator lint_on  WIDTH
			wide_rx_last  = s_last;
		end else begin
			wide_rx_valid = mm2s_valid;
			wide_rx_bytes = mm2s_bytes;
			wide_rx_last  = mm2s_last;
		end

		wide_rx_data = 0;
		if (OPT_LITTLE_ENDIAN)
		begin
			if (sd2s_busy)
				wide_rx_data[31: 0]  = i_sd2s_data;
			else if (!mm2s_busy && OPT_ISTREAM)
				wide_rx_data[SW-1:0] = s_data;
			else
				wide_rx_data[DW-1:0] = mm2s_data;
		end else begin
			if (sd2s_busy)
				wide_rx_data[RXWIDTH-1:RXWIDTH-32]= i_sd2s_data;
			else if (!mm2s_busy && OPT_ISTREAM)
				wide_rx_data[RXWIDTH-1:RXWIDTH-SW]= s_data;
			else
				wide_rx_data[RXWIDTH-1:RXWIDTH-DW]= mm2s_data;
		end
	end
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// RX Gears -- Pack the data stream to the width of the bus
	// {{{

	sddma_rxgears #(	// Adjust from bus width (or less) to full busw
		.BUS_WIDTH(RXWIDTH),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN)
	) u_s2sd_rxgears (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		.i_soft_reset(i_soft_reset),
		//
		.S_VALID(wide_rx_valid),
		.S_READY(rxgears_ready),
		.S_DATA( wide_rx_data),
		.S_BYTES(wide_rx_bytes),
		.S_LAST( wide_rx_last),
		//
		.M_VALID(wide_valid), .M_READY(wide_ready),
		.M_DATA(wide_data), .M_BYTES(wide_bytes), .M_LAST(wide_last)
		// }}}
	);

	assign	o_sd2s_ready = rxgears_ready;	// && sd2s_busy
	assign	mm2s_ready   = rxgears_ready;	// && s2sd_busy && mm2s_busy
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// The FIFO
	// {{{

`ifdef	SDIO_AXI
	// Set USE_FIFO to zero once the AXI DMA has been integrated
	// for better resource usage.
	localparam	USE_FIFO = 1'b0;
`else
	localparam	USE_FIFO = 1'b1;
`endif

	generate if (USE_FIFO)
	begin : GEN_FIFO
		localparam	LGFLEN = LGFIFO-$clog2(DW/8);
		reg	flushing;

		sdfifo #(
			// {{{
			.LGFLEN(LGFLEN),
			.BW(2+$clog2(DW/8)+DW),
			.OPT_ASYNC_READ(1'b0),
			.OPT_WRITE_ON_FULL(1'b0),
			.OPT_READ_ON_EMPTY(1'b0)
			// }}}
		) u_sfifo (
			// {{{
			.i_clk(i_clk), .i_reset(i_reset),
			//
			.i_wr(wide_valid),
			.i_data({ wide_last, wide_bytes, wide_data }),
			.o_full(fifo_full),
			.o_fill(ign_fifo_fill),
			//
			.i_rd(fifo_ready && flushing),
			.o_data({ fifo_last, fifo_bytes, fifo_data }),
			.o_empty(fifo_empty)
			// }}}
		);

		always @(posedge i_clk)
		if (i_reset)
			flushing <= 0;
		else if (i_dma_s2sd || mm2s_busy
			|| (i_dma_sd2s && OPT_OSTREAM && i_dma_addr[ADDR_MSB]))
			flushing <= 1;
		else if (wide_valid && wide_last)
			flushing <= 1;
		else if (ign_fifo_fill[LGFLEN:LGFLEN-1] != 0)
			flushing <= 1;
		else if (fifo_empty)
			flushing <= 0;

		assign	fifo_valid = !fifo_empty && flushing;
		assign	wide_ready = !fifo_full;

	end else begin : NO_FIFO
		assign	fifo_valid = wide_valid;
		assign	fifo_empty = !wide_valid;
		assign	{ fifo_last, fifo_bytes, fifo_data }
				= { wide_last, wide_bytes, wide_data };
		assign	wide_ready = fifo_ready;
		assign	ign_fifo_fill = 0;
		assign	fifo_full = 0;
	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// TX Gears -- adjusting word packing, to either 32b, SW, or DW bits
	// {{{

	sddma_txgears #(
		// {{{
		.BUS_WIDTH(DW),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN)
		// }}}
	) u_s2sd_txgears (
		// {{{
		.i_clk(i_clk),
		.i_reset(i_reset),
		.i_soft_reset(i_soft_reset),
		//
		.i_size((s2sd_busy || (m_active && SW==32)) ? SZ_32B : SZ_BUS),
		//
		.S_VALID(fifo_valid), .S_READY(fifo_ready),
		.S_DATA(fifo_data), .S_BYTES(fifo_bytes), .S_LAST(fifo_last),
		//
		.M_VALID(tx_valid), .M_READY(tx_ready),
		.M_DATA(tx_data), .M_BYTES(tx_bytes), .M_LAST(tx_last)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// S2MM -- Write to memory
	// {{{

`ifdef	SDIO_AXI
	sdax_s2mm #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.BUS_WIDTH(DW),
		.IW(AXI_IW),
		.OPT_LOWPOWER(OPT_LOWPOWER)
		// .OPT_LITTLE_ENDIAN(1'b1),	AXI is always little endian
		// .LGPIPE(512 / BUS_WIDTH)
		// }}}
	) u_s2mm (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		.i_soft_reset(i_soft_reset || i_dma_abort),
		//
		.i_request(i_dma_sd2s && (!OPT_OSTREAM || !i_dma_addr[ADDR_MSB])),
		.o_busy(s2mm_busy), .o_err(s2mm_err),
		.i_inc(1'b1), .i_size(SZ_BUS),
		.i_addr(i_dma_addr[ADDRESS_WIDTH-1:0]),
		//
		.S_VALID(tx_valid && !m_active && (i_dma_sd2s || sd2s_busy)),
		.S_READY(s2mm_ready),
		.S_DATA(tx_data), .S_BYTES(tx_bytes), .S_LAST(tx_last),
		//
		.M_AWVALID(M_AXI_AWVALID),
		.M_AWREADY(M_AXI_AWREADY),
		.M_AWID(M_AXI_AWID),
		.M_AWADDR(M_AXI_AWADDR),
		.M_AWLEN(M_AXI_AWLEN),
		.M_AWSIZE(M_AXI_AWSIZE),
		.M_AWBURST(M_AXI_AWBURST),
		.M_AWLOCK(M_AXI_AWLOCK),
		.M_AWCACHE(M_AXI_AWCACHE),
		.M_AWPROT(M_AXI_AWPROT),
		.M_AWQOS(M_AXI_AWQOS),
		//
		.M_WVALID(M_AXI_WVALID),
		.M_WREADY(M_AXI_WREADY),
		.M_WDATA(M_AXI_WDATA),
		.M_WSTRB(M_AXI_WSTRB),
		.M_WLAST(M_AXI_WLAST),
		//
		.M_BVALID(M_AXI_BVALID),
		.M_BREADY(M_AXI_BREADY),
		.M_BID(M_AXI_BID),
		.M_BRESP(M_AXI_BRESP)
		// }}}
	);

`else	// SDIO_AXI
	sddma_s2mm #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.BUS_WIDTH(DW),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN),
		.LGPIPE(LGFIFO+1-$clog2(DW/8))
		// }}}
	) u_s2mm (
		// {{{
		.i_clk(i_clk),
		.i_reset(i_reset || i_soft_reset || i_dma_abort),
		.i_request(i_dma_sd2s && (!OPT_OSTREAM || !i_dma_addr[ADDR_MSB])),
		.o_busy(s2mm_busy), .o_err(s2mm_err),
		.i_inc(1'b1), .i_size(SZ_BUS),
		.i_addr(i_dma_addr[ADDRESS_WIDTH-1:0]),
		//
		.S_VALID(tx_valid && !m_active && (i_dma_sd2s || sd2s_busy)),
		.S_READY(s2mm_ready),
		.S_DATA(tx_data), .S_BYTES(tx_bytes), .S_LAST(tx_last),
		//
		.o_wr_cyc(wr_cyc), .o_wr_stb(wr_stb), .o_wr_we(wr_we),
		.o_wr_addr(wr_addr), .o_wr_data(wr_data), .o_wr_sel(wr_sel),
		.i_wr_stall(wr_stall), .i_wr_ack(wr_ack),
				.i_wr_data({(DW){1'b0}}),
		.i_wr_err(i_dma_err)
		// }}}
	);
`endif
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Outgoing stream generation
	// {{{

	// m_* stream signals
	generate if (OPT_OSTREAM)
	begin : GEN_OSTREAM
		reg	r_active;

		always @(posedge i_clk)
		if (i_reset)
			r_active <= 1'b0;
		else if (i_dma_abort || i_soft_reset)
			r_active <= 1'b0;
		else if (i_dma_sd2s && i_dma_addr[ADDR_MSB])
			r_active <= 1'b1;
		else if (m_valid && m_ready && m_last)
			r_active <= 1'b0;

		assign	m_valid = m_active && tx_valid;
		assign	tx_ready = s2sd_busy ? i_s2sd_ready :
					m_active ? m_ready : s2mm_ready;
		assign	m_data = (OPT_LITTLE_ENDIAN) ? tx_data[SW-1:0]
					: tx_data[DW-1:DW-SW];
		assign	m_active = r_active;
		assign	m_last = tx_last;

	end else begin : NO_OSTREAM

		assign	m_valid = 1'b0;
		assign	tx_ready = s2sd_busy ? i_s2sd_ready : s2mm_ready;
		assign	m_data = 0;
		assign	m_last = 1'b0;
		assign	m_active = 1'b0;

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_ostream = &{ 1'b0, m_ready };
		// Verilator lint_off UNUSED
		// }}}
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone Arbiter
	// {{{
`ifdef	SDIO_AXI
	// {{{
	// No arbiter is necessary when using AXI, since each of the two
	// AXI components is one way only, either read or write, so there's
	// no conflict.
	// }}}
`else
	assign	o_dma_cyc  = rd_cyc || wr_cyc;
	assign	o_dma_stb  = rd_stb || wr_stb;
	assign	o_dma_we   = s2mm_busy;
	assign	o_dma_addr = s2sd_busy ? rd_addr : wr_addr;
	assign	o_dma_data = wr_data;
	assign	o_dma_sel  = (s2sd_busy) ? rd_sel : wr_sel;
	assign	{ wr_stall, rd_stall } = {(2){i_dma_stall}};
	assign	wr_ack = i_dma_ack && wr_cyc;
	assign	rd_ack = i_dma_ack && rd_cyc;
`endif
	// }}}

	// s2sd_* signaling
	// {{{
	assign	o_s2sd_valid = s2sd_busy && tx_valid;
	assign	o_s2sd_data  = (OPT_LITTLE_ENDIAN) ? tx_data[31:0]
				: tx_data[DW-1:DW-32];

	// s2sd_busy
	always @(posedge i_clk)
	if (i_reset || s2mm_err)
		s2sd_busy <= 1'b0;
	else if (i_dma_s2sd)
		s2sd_busy <= 1'b1;
	else if (tx_valid && tx_ready && tx_last)
		s2sd_busy <= 1'b0;
	// }}}

	// sd2s_busy
	// {{{
	always @(posedge i_clk)
	if (i_reset || mm2s_err)
		sd2s_busy <= 1'b0;
	else if (i_dma_sd2s)
		sd2s_busy <= 1'b1;
	else if (tx_valid && tx_ready && tx_last)
		sd2s_busy <= 1'b0;
	// }}}

	assign	o_dma_busy = sd2s_busy || s2sd_busy;
	assign	o_dma_err  = mm2s_err || s2mm_err;

	// Make verilator happy
	// {{{
	// verilator coverage_off
	// verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, ign_fifo_fill, fifo_last,
`ifdef	SDIO_AXI
				s2mm_busy,
			tx_bytes, i_dma_len, i_dma_abort, i_dma_addr
`else
				wr_we, rd_we, ign_rd_data
`endif
				};
	// verilator lint_on  UNUSED
	// verilator coverage_on
	// }}}
endmodule

