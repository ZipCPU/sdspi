////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdwb.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Bus handler.  Accepts and responds to Wishbone bus requests.
//		Configures clock division, and IO speed and parameters.
//	Issues commands to the command handler, TX and RX handlers.
//
//
//	Basic command types:
//		0x00000040	Broadcast command, no response expected
//		0x00000140	Standard command, R1 expected response
//		0x00000240	Command expecting an R2 return
//		0x00000940	Read request, read data to follow
//		0x04000d40	Write request, data to follow
//		0x00000800	Continues a data read into a second sector
//		0x00000c00	Continues a data write into a second sector
//		0x00000168	(CMD40) GO_IRQ_STATE eMMC command
//					(open drain response)
//	   How to break an interrupt?
//		0x00000028 (Also requires open-drain mode)
//		0x00000040	(GO_IDLE, expects no response)
//	   How to reset an error without doing anything?
//		0x00008080
//	   How to reset the FIFO pointer without doing anything?
//		0x00000080
//	   How to keep the command controller from timing out while
//			waiting for an interrupt?  Send a GO_IRQ_STATE command
//			The command processor will need to know how to handle
//			this internally.
//		0x00000168
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
module	sdwb #(
		// {{{
		parameter	LGFIFO = 15,	// Log_2(FIFO size in bytes)
		parameter	NUMIO=4,
		localparam	MW = 32,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		parameter [0:0]	OPT_SERDES = 1'b0,
		parameter [0:0]	OPT_DS = OPT_SERDES,
		parameter [0:0]	OPT_DDR = 1'b0,
		parameter [0:0]	OPT_CARD_DETECT = 1'b1,
		parameter [0:0]	OPT_EMMC = 1'b1,
		parameter [0:0]	OPT_CRCTOKEN = 1'b1,
		localparam	LGFIFOW=LGFIFO-$clog2(MW/8),
		parameter [0:0]	OPT_DMA = 1'b0,
		parameter	DMA_AW = 30,
		parameter [0:0]	OPT_STREAM = 1'b0,
		// Set OPT_HWRESET if a reset pin exists for this H/W
		parameter [0:0]	OPT_HWRESET = OPT_EMMC,	// eMMC has resets
		parameter [0:0]	OPT_1P8V= 1'b0,	// 1.8V voltage switch capable?
		// OPT_R1B, if set, adds logic to the controller to only expect
		// a card busy signal following an R1B command.  This insures
		// the card busy status is set until the card has an opportunity
		// to set it.  Any potential card busy returns from the PHY,
		// for other reasons (data?) will be will be ignored otherwise
		// except following an R1B command.  If not set, then anytime
		// DAT[0] goes low following a command (and neither actively
		// transmitting or receiving), then the card busy will be set
		// until it returns high again.  This will catch all reasons
		// the card may be busy, and actually provide a direct wire
		// for reading the card busy bit from the card itself via the
		// PHY.
		parameter [0:0]	OPT_R1B = 1'b1,
		// If OPT_R1B is set, then we'll want to wait at least two
		// device clocks waiting for busy.  Each clock can be as long
		// as 1k system clock cycles.  Hence, we'll wait for up to 4095
		// clock cycles before timing out while waiting for busy.
		// Perhaps that's too long, but it's just a backup timeout.
		// If the device actually indicates a busy (like it's supposed
		// to), then we'll only be busy until the device releases.
		parameter	LGCARDBUSY = 12,
		parameter [0:0]	OPT_LOWPOWER = 1'b0
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		// Wishbone interface
		// {{{
		input	wire			i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[2:0]		i_wb_addr,
		input	wire	[32-1:0]	i_wb_data,
		input	wire	[32/8-1:0]	i_wb_sel,
		output	wire			o_wb_stall,
		output	reg			o_wb_ack,
		output	wire	[32-1:0]	o_wb_data,
		// }}}
		// Configuration options
		// {{{
		output	reg			o_cfg_clk90,
		output	wire	[7:0]		o_cfg_ckspeed,
		output	reg			o_cfg_shutdown,
		output	wire	[1:0]		o_cfg_width,
		output	wire			o_cfg_ds, o_cfg_dscmd,
		output	reg			o_cfg_ddr,
		output	reg			o_pp_cmd, o_pp_data,
		output	reg	[4:0]		o_cfg_sample_shift,
		output	reg			o_cfg_expect_ack,
		input	wire	[7:0]		i_ckspd,

		output	reg			o_soft_reset,
		// }}}
		// External DMA interface
		// {{{
		output	wire		o_dma_sd2s,
		output	wire		o_sd2s_valid,
		input	wire		i_sd2s_ready,
		output	wire	[31:0]	o_sd2s_data,
		output	wire		o_sd2s_last,
		//
		output	wire		o_dma_s2sd,
		input	wire		i_s2sd_valid,
		output	wire		o_s2sd_ready,
		input	wire	[31:0]	i_s2sd_data,

		output	wire [DMA_AW-1:0]	o_dma_addr,
		// o_dma_len: DMA transfer length (in bytes) = 1<<lgblk
		output	wire [LGFIFO:0]	o_dma_len,
		input	wire		i_dma_busy,
		input	wire		i_dma_err,
		output	wire		o_dma_abort,
		// }}}
		// CMD interface
		// {{{
		output	wire			o_cmd_selfreply,
		output	reg			o_cmd_request,
		output	reg	[1:0]		o_cmd_type,
		output	wire	[6:0]		o_cmd_id,
		output	wire	[31:0]		o_arg,
		//
		input	wire			i_cmd_busy, i_cmd_done,
						i_cmd_err,
		input	wire	[1:0]		i_cmd_ercode,
		//
		input	wire			i_cmd_response,
		input	wire	[5:0]		i_resp,
		input	wire	[31:0]		i_arg,
		//
		input	wire			i_cmd_mem_valid,
		input	wire	[MW/8-1:0]	i_cmd_mem_strb,
		input	wire	[LGFIFOW-1:0]	i_cmd_mem_addr,
		input	wire	[MW-1:0]	i_cmd_mem_data,
		// }}}
		// TX interface
		// {{{
		output	reg			o_tx_en,
		//
		output	reg			o_tx_mem_valid,
		input	wire			i_tx_mem_ready,
		output	reg	[31:0]		o_tx_mem_data,
		output	reg			o_tx_mem_last,
		//
		input	wire			i_tx_done, i_tx_err,i_tx_ercode,
		// }}}
		// RX interface
		// {{{
		output	reg			o_rx_en,
		output	wire			o_crc_en,
		output	wire	[LGFIFO:0]	o_length,
		//
		input	wire			i_rx_mem_valid,
		input	wire	[MW/8-1:0]	i_rx_mem_strb,
		input	wire	[LGFIFOW-1:0]	i_rx_mem_addr,
		input	wire	[MW-1:0]	i_rx_mem_data,
		//
		input	wire			i_rx_done, i_rx_err,i_rx_ercode,
		// }}}
		input	wire			i_card_detect,
		input	wire			i_card_busy,
		output	wire			o_hwreset_n,
		output	wire			o_1p8v,
		input	wire			i_1p8v,
		output	reg			o_int
		// }}}
	);

	// Local declarations
	// {{{
	localparam	LGFIFO32 = LGFIFO - $clog2(32/8);

	localparam	[2:0]	ADDR_CMD = 0,
				ADDR_ARG = 1,
				ADDR_FIFOA = 2,
				ADDR_FIFOB = 3,
				ADDR_PHY   = 4;

	localparam	[1:0]	CMD_PREFIX = 2'b01,
				NUL_PREFIX = 2'b00;
	localparam	[1:0]	RNO_REPLY = 2'b00,
				R2_REPLY = 2'b10,
				R1B_REPLY = 2'b11;
	// Command register bits
	localparam		EXPECT_ACK_BIT   = 26,
				HWRESET_BIT      = 25,
				CARD_REMOVED_BIT = 18,
				ERR_BIT          = 15,
				USE_DMA_BIT      = 13,
				FIFO_ID_BIT      = 12,
				USE_FIFO_BIT     = 11,
				FIFO_WRITE_BIT   = 10;	// Write to SD card
	// PHY register bits
	localparam		VOLTAGE_BIT = 22,
				DSCMD_BIT        = 21,
				CLK_SHUTDOWN_BIT = 15,
				CLK90_BIT        = 14,
				PP_CMD_BIT       = 13,
				PP_DATA_BIT      = 12,
				DS_BIT           =  9,
				DDR_BIT          =  8;

	localparam	[1:0]	WIDTH_1W = 2'b00,
				WIDTH_4W = 2'b01,
				WIDTH_8W = 2'b10;
	// localparam	[15:0]	CMD_SELFREPLY = 16'h0028;

	reg	cmd_busy, new_cmd_request, new_data_request, new_tx_request,
		new_r2_request, new_dma_request;
	reg	w_selfreply_request, r_clk_shutdown;
	reg	clear_err;

	// reg		bus_wrvalid, bus_rdvalid;
	reg	[31:0]	bus_rddata;
	wire		bus_write, bus_read;
	wire	[31:0]	bus_wdata;
	wire	[3:0]	bus_wstrb;
	wire	[2:0]	bus_wraddr, bus_rdaddr;

	wire		bus_cmd_stb, bus_phy_stb;
	reg	[6:0]	r_cmd;
	reg		r_tx_request, r_rx_request, r_tx_sent, r_ecode,
			r_fifo, r_cmd_err, r_transfer_err;
	reg	[1:0]	r_cmd_ecode;
	reg	[31:0]	r_arg;
	reg	[3:0]	lgblk;
	reg	[1:0]	r_width;
	reg	[7:0]	r_ckspeed;
	reg	[31:0]	w_cmd_word, w_phy_ctrl;
	reg	[15:0]	blk_words;

	integer	ika, ikb;
	localparam	NFIFOW = (1<<LGFIFO) / (MW/8);
	reg	[MW-1:0]	fifo_a	[0:NFIFOW-1];
	reg	[MW-1:0]	fifo_b	[0:NFIFOW-1];
	reg	[MW-1:0]	tx_fifo_a, tx_fifo_b;
	wire	[(($clog2(MW/32) > 0) ? ($clog2(MW/32)-1):0):0] tx_shift;
	reg	[LGFIFOW-1:0]	fif_wraddr, fif_rdaddr;
	reg	[LGFIFOW-1:0]	fif_a_rdaddr, fif_b_rdaddr;
	reg	[LGFIFO32-1:0]	tx_mem_addr;
	reg	[MW-1:0]	next_tx_mem;
	reg			tx_fifo_last, pre_tx_last,
				tx_pipe_valid;

	wire			card_present, card_removed;

	reg			pre_valid;
	reg	[1:0]		pre_sel;
	reg	[31:0]		pre_data;

	reg	[LGFIFOW-1:0]	mem_wr_addr_a, mem_wr_addr_b;
	reg	[MW/8-1:0]	mem_wr_strb_a, mem_wr_strb_b;
	reg	[MW-1:0]	mem_wr_data_a, mem_wr_data_b;

	reg			r_mem_busy, card_was_busy;
	wire			w_card_busy;

	// DMA signals
	wire		dma_busy, dma_fifo, dma_write, dma_read_fifo,
			dma_error, dma_last, dma_zero_len, dma_int, dma_stopped,
			dma_read_active, dma_tx;
	wire	[31:0]	dma_command;
	wire	[31:0]		dma_len_return;
	reg	[63:0]		dma_addr_return;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Registers
	// {{{

	// CMD/control register
	// {{{
	assign	bus_write  =(i_wb_stb && !o_wb_stall && i_wb_we) || dma_write;
	assign	bus_wraddr = (dma_write) ? 0 : i_wb_addr;
	assign	bus_wdata  = (dma_write) ? dma_command : i_wb_data;
	assign	bus_wstrb  = (dma_write) ? 4'hf : i_wb_sel;

	assign	bus_read   = i_wb_stb && !o_wb_stall && !i_wb_we;
	assign	bus_rdaddr = i_wb_addr;

	assign	bus_cmd_stb = bus_write && bus_wraddr == ADDR_CMD
		&& (dma_busy == dma_write)
		&&((!dma_error && !r_cmd_err && !r_transfer_err)
			|| (bus_wstrb[ERR_BIT/8] && bus_wdata[ERR_BIT]));


	// o_soft_reset
	// {{{
	initial	o_soft_reset = 1'b1;
	always @(posedge i_clk)
	if (i_reset || (OPT_CARD_DETECT && (!card_present || card_removed))
			|| (OPT_HWRESET && !o_hwreset_n))
	begin
		o_soft_reset <= 1'b1;
	end else if (bus_write && bus_wraddr == ADDR_CMD)
	begin
		o_soft_reset <= 1'b0;
		if (OPT_HWRESET && bus_wstrb[HWRESET_BIT/8])
			o_soft_reset <= bus_wdata[HWRESET_BIT];
		if (&bus_wstrb[3:0] && bus_wdata == 32'h5200_0000)
			o_soft_reset <= 1'b1;
	end else
		o_soft_reset <= 1'b0;
	// }}}

	// mem_busy
	// {{{
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_mem_busy <= 1'b0;
	else if (new_r2_request || (new_data_request
			&& (!bus_wdata[USE_DMA_BIT] || !dma_zero_len)
			&& (!bus_wdata[FIFO_WRITE_BIT] || new_tx_request))
			|| (!i_cmd_err && (r_tx_request || r_rx_request)))
		r_mem_busy <= 1'b1;
	else begin
		if (i_cmd_err && (r_tx_request || r_rx_request))
			r_mem_busy <= 1'b0;
		if (cmd_busy && i_cmd_done && !o_rx_en && !o_tx_en)
			r_mem_busy <= 1'b0;
		if (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last)
			r_mem_busy <= 1'b0;
		if (o_rx_en && i_rx_done)
			r_mem_busy <= 1'b0;
		if (w_selfreply_request && cmd_busy && o_cmd_type == R2_REPLY)
			r_mem_busy <= 1'b0;
		// if (!mem_busy) r_mem_busy <= 1'b0;
	end
`ifdef	FORMAL
	wire		f_mem_busy;
	wire [13:0]	f_blocksz;

	assign	f_blocksz = (14'h1 << (lgblk-2));

	assign	f_mem_busy = (o_tx_en && !r_tx_sent) || r_tx_request || o_rx_en
			|| r_rx_request ||(cmd_busy && o_cmd_type == R2_REPLY);

	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(!r_mem_busy && !f_mem_busy);

	always @(*)
	if (!i_reset && !o_soft_reset && o_hwreset_n)
		assert(r_mem_busy == f_mem_busy);

	always @(*)
	if (!i_reset && !o_tx_en)
		assert(!o_tx_mem_valid);
`endif
	// }}}

	// o_cmd_request
	// {{{
	always @(*)
	begin
		w_selfreply_request = !o_cmd_request && bus_cmd_stb
			&& (&bus_wstrb[1:0])
			&&(!bus_wdata[USE_DMA_BIT]
					&& !bus_wdata[USE_FIFO_BIT])
			&& (bus_wdata[9:8] == RNO_REPLY)
			&& ((bus_wdata[7:6] ==  NUL_PREFIX
					&& bus_wdata[5:0] != 6'h0) //IRQ Reply
			  ||(bus_wdata[7:6] ==  CMD_PREFIX
					&& bus_wdata[5:0] == 6'h0)); // GO_IDLE

		if (OPT_HWRESET && (!o_hwreset_n
					|| (bus_wstrb[HWRESET_BIT/8] && bus_wdata[HWRESET_BIT])))
			w_selfreply_request = 1'b0;
		if (i_reset || o_soft_reset || !OPT_EMMC)
			w_selfreply_request = 1'b0;
	end

	always @(*)
	begin
		// Default values (all == 0)
		// {{{
		new_cmd_request  = 1'b0;
		new_data_request = 1'b0;
		new_dma_request  = 1'b0;
		new_r2_request   = 1'b0;
		// }}}

		if (OPT_EMMC && w_selfreply_request)
		begin // Self-reply request -- EMMC only
			// {{{
			new_cmd_request  = 1'b1;
			new_data_request = 1'b0;
			new_dma_request  = 1'b0;
			new_r2_request   = 1'b0;
			// }}}
		end else if (OPT_HWRESET && bus_wstrb[HWRESET_BIT/8] && bus_wdata[HWRESET_BIT])
		begin // Hardware reset request -- overrides everything else
			// {{{
			new_cmd_request  = 1'b0;
			new_data_request = 1'b0;
			new_dma_request  = 1'b0;
			new_r2_request   = 1'b0;
			// }}}
		end else if (bus_wdata[9:6] == { R2_REPLY, CMD_PREFIX})
		begin // R2 request
			// {{{
			new_cmd_request  = 1'b1;
			new_data_request = 1'b1;
			new_dma_request  = 1'b0;
			new_r2_request   = 1'b1;

			if (cmd_busy || r_mem_busy || o_tx_en || dma_busy || i_cmd_err)
			begin
				new_cmd_request  = 1'b0;
				new_data_request = 1'b0;
				new_r2_request   = 1'b0;
				// invalid_request = 1'b1
			end
			// }}}
		end else if ((bus_wdata[7:6] == CMD_PREFIX
					|| bus_wdata[7:6] == NUL_PREFIX)
			&& bus_wdata[USE_DMA_BIT])
		begin // DMA request
			// {{{
			new_cmd_request  = (bus_wdata[7:6] == CMD_PREFIX)
						&& (!dma_busy || !dma_write);
			new_data_request = (!dma_busy || !dma_write);
			new_dma_request  = (!dma_busy || !dma_write);
			new_r2_request   = 1'b0;

			if (!OPT_DMA || dma_busy || r_mem_busy || o_tx_en
				|| dma_zero_len || i_cmd_err
				||(cmd_busy && bus_wdata[7:6] == CMD_PREFIX))
			begin
				new_cmd_request  = 1'b0;
				new_data_request = 1'b0;
				new_dma_request  = 1'b0;
				// invalid_request = (!OPT_DMA || dma_busy || r_mem_busy);
			end else if (bus_wdata[FIFO_WRITE_BIT])
				new_data_request = 1'b0;
			// }}}
		end else if ((bus_wdata[7:6] == CMD_PREFIX || bus_wdata[7:6] == NUL_PREFIX)
				&& !bus_wdata[USE_DMA_BIT]
				&& bus_wdata[USE_FIFO_BIT])
		begin // FIFO request
			// {{{
			new_cmd_request  = (bus_wdata[7:6] == CMD_PREFIX);
			new_data_request = 1'b1;
			new_dma_request  = 1'b0;

			if (r_mem_busy// || (OPT_DMA && dma_busy && !dma_write)
				|| o_tx_en || i_cmd_err
				||(cmd_busy && bus_wdata[7:6] == CMD_PREFIX))
			begin
				new_cmd_request  = 1'b0;
				new_data_request = 1'b0;
				// invalid_request = 1'b1
			end
			// }}}
		end else if (bus_wdata[7:6] == CMD_PREFIX)
		begin // Normal command request
			// {{{
			new_cmd_request  = 1'b1;
			new_data_request = 1'b0;
			new_dma_request  = 1'b0;
			new_r2_request   = 1'b0;
			if (cmd_busy || (OPT_DMA && dma_busy && !dma_write))
			begin
				new_cmd_request  = 1'b0;
			end
			// }}}
		end

		if (!bus_cmd_stb || (bus_wstrb[1:0] != 2'b11))
		begin // Only act following a bus write
			// {{{
			new_cmd_request  = 1'b0;
			new_data_request = 1'b0;
			new_dma_request  = 1'b0;
			new_r2_request   = 1'b0;
			// }}}
		end

		if (i_reset || o_soft_reset || (OPT_HWRESET && !o_hwreset_n))
			{ new_data_request, new_cmd_request, new_dma_request, new_r2_request } = 4'b0;
	end

	initial	o_cmd_request = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cmd_request <= 1'b0;
	else if (new_cmd_request)
	begin
		o_cmd_request <= 1'b1;
	end else if (!i_cmd_busy)
		o_cmd_request <= 1'b0;
	// }}}

	// o_cmd_selfreply -- send a command even if busy waiting on a reply
	// {{{
	generate if (OPT_EMMC)
	begin : GEN_SELFREPLY
		reg	r_cmd_selfreply;

		initial	r_cmd_selfreply = 1'b0;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_cmd_selfreply <= 1'b0;
		else if (w_selfreply_request)
			r_cmd_selfreply <= 1'b1;
		else if (!i_cmd_busy)
			r_cmd_selfreply <= 1'b0;

		assign	o_cmd_selfreply = r_cmd_selfreply;
`ifdef	FORMAL
		always @(*)
		if (!i_reset && !o_soft_reset && !o_hwreset_n)
			assert(!r_cmd_selfreply);
`endif
	end else begin : NO_SELFREPLY

		assign	o_cmd_selfreply = 1'b0;
	end endgenerate
	// }}}

	// cmd_busy: Are we waiting on a command to complete?
	// {{{
	initial	cmd_busy = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		cmd_busy <= 1'b0;
	else if (new_cmd_request)
		cmd_busy <= 1'b1;
	else if (i_cmd_done)
		cmd_busy <= 1'b0;
`ifdef	FORMAL
	always @(*)
	if (i_reset && o_cmd_request)
		assert(cmd_busy);
	always @(posedge i_clk)
	if (!i_reset && !$past(i_reset) && !o_soft_reset && !o_hwreset_n)
		assert(!cmd_busy);
`endif
	// }}}

	// o_cmd_id: What command are we issuing?
	// {{{
	initial	r_cmd = 7'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_cmd <= 7'b0;
	else if (new_cmd_request)
		r_cmd <= bus_wdata[6:0];
	else if (i_cmd_response)
		r_cmd <= { 1'b0, i_resp };

	assign	o_cmd_id = r_cmd[6:0];
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(r_cmd == 7'h0);
`endif
	// }}}

	// o_cmd_type: What response to expect?  None, R1, R2, or R1b
	// {{{
	initial	o_cmd_type = 2'b00;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cmd_type <= 2'b00;
	else if (new_cmd_request)
		o_cmd_type <= bus_wdata[9:8];
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(o_cmd_type == 2'b00);
`endif
	// }}}

	// r_expect_busy, r_card_busy
	// {{{
	generate if (OPT_R1B)
	begin : GEN_R1B
		// We want to wait at least two device clocks waiting for busy.
		// Each clock can be as long as 1k system clock cycles.  Hence,
		// we'll wait for up to 4095 clock cycles here.  Perhaps that's
		// too long, but it's just a timeout.  If the device actually
		// indicates a busy (like it's supposed to), then we'll be
		// busy until the device releases.
		reg	[LGCARDBUSY-1:0]	r_busy_counter;
		reg		r_expect_busy, r_card_busy;

		initial	r_expect_busy = 1'b0;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_expect_busy <= 1'b0;
		else if (o_tx_en)
			r_expect_busy <= 1'b1;
		else if (new_cmd_request)
			r_expect_busy <= (bus_wdata[9:8] == R1B_REPLY);
		else if (!cmd_busy && (i_card_busy || r_busy_counter == 0))
			r_expect_busy <= 1'b0;

		initial	r_card_busy = 1'b0;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_card_busy <= 1'b0;
		else if (o_tx_en || (r_card_busy && i_card_busy))
			r_card_busy <= 1'b1;
		else if (new_cmd_request)
			r_card_busy <= (bus_wdata[9:8] == R1B_REPLY);
		else if (!i_card_busy && !r_expect_busy && !cmd_busy)
			r_card_busy <= 1'b0;

		initial	r_busy_counter = 0;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_busy_counter <= 0;
		else if (o_rx_en || i_card_busy
				|| (cmd_busy && !r_expect_busy && !o_tx_en))
			r_busy_counter <= 0;
		else if ((cmd_busy && r_expect_busy) || o_tx_en)
		begin
			r_busy_counter <= -1;

			if (r_ckspeed < 4)
				// Max clock rate is 25/3 => 12.5MHz, or 8 cycls
				r_busy_counter <= 16;	// 2 clock periods
			else if (r_ckspeed < 8)
				// Max clock rate is 25/5 => 5MHz or 20cycles
				r_busy_counter <= 72;	// 3.5 clock periods
			else if (r_ckspeed < 16)
				// Max clock rate is 25/13 => 52 cycles
				r_busy_counter <= 192;	// 3.6 clock periods
			else if (r_ckspeed < 32)
				// Max clock rate is 25/29 => 116 cycles
				r_busy_counter <= 3*128;	// 3.3 clks
		end else if (r_busy_counter != 0)
			r_busy_counter <= r_busy_counter - 1;

		assign	w_card_busy = r_card_busy;
`ifdef	FORMAL
		// We need to stay officially busy as long as we are waiting
		// for a response from the card
		always @(*)
		if (!i_reset && r_expect_busy)
			assert(r_card_busy);

		// We only check timeouts while we expect a busy signal, and
		// before it takes place
		always @(*)
		if (!i_reset && !r_expect_busy && !cmd_busy)
			assert(r_busy_counter == 0);

		always @(*)
		if (!i_reset && !o_soft_reset && !o_hwreset_n)
		begin
			assert(r_expect_busy == 1'b0);
			assert(r_busy_counter == 0);
			assert(r_card_busy == 1'b0);
		end
`endif
	end else begin : DIRECT_CARD_BUSY
		assign	w_card_busy = i_card_busy;

		// Keep Verilator happy
		// {{{
		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused_r1b;
		assign	unused_r1b = &{ 1'b0, LGCARDBUSY };
		// Verilator lint_on  UNUSED
		// Verilator coverage_on
		// }}}
	end endgenerate

	initial	card_was_busy = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		card_was_busy <= 1'b0;
	else
		card_was_busy <= w_card_busy;
	// }}}

	// o_tx_en, r_tx_request, r_tx_sent, o_cfg_expect_ack
	// {{{
	always @(*)
	begin
		new_tx_request = new_data_request && bus_wdata[FIFO_WRITE_BIT];
		if (OPT_DMA && !dma_write && bus_wdata[USE_DMA_BIT])
			new_tx_request = 1'b0;
		if (bus_wdata[9:8] == R2_REPLY
				&& bus_wdata[7:6] == CMD_PREFIX)
			new_tx_request = 1'b0;
		if (i_reset || o_soft_reset || !o_hwreset_n)
			new_tx_request = 1'b0;
	end

	always @(posedge i_clk)
	if (i_reset || o_soft_reset || !OPT_CRCTOKEN)
		o_cfg_expect_ack <= 1'b0;
	else if (r_rx_request || o_rx_en || (dma_busy && !dma_tx))
	begin
		o_cfg_expect_ack <= 1'b0;
	end else if (dma_busy || r_mem_busy || o_tx_en
				|| !bus_write || bus_wraddr != ADDR_CMD)
	begin
	end else // if (&bus_wstrb[EXPECT_ACK_BIT/8-1:0])
	begin
		if (bus_wstrb[EXPECT_ACK_BIT/8])
			o_cfg_expect_ack <= bus_wdata[EXPECT_ACK_BIT];
		if (bus_wstrb[FIFO_WRITE_BIT/8] && !bus_wdata[FIFO_WRITE_BIT])
			o_cfg_expect_ack <= 1'b0;
		if ((bus_wstrb[USE_FIFO_BIT/8] && !bus_wdata[USE_FIFO_BIT])
				&&(!OPT_DMA || !bus_wdata[USE_DMA_BIT]))
			o_cfg_expect_ack <= 1'b0;
	end
`ifdef	FORMAL
	always @(posedge i_clk)
	if (i_reset || !OPT_CRCTOKEN)
	begin
	end else if ($past(i_reset || o_soft_reset))
	begin
		assert(!o_cfg_expect_ack);
	end else if (!dma_busy && (o_rx_en || r_rx_request))
	begin
		assert(!o_cfg_expect_ack);
	end else if (o_tx_en || (dma_busy && $past(dma_busy)) || $past(r_tx_request))
	begin
		assert($stable(o_cfg_expect_ack));
	end
`endif

	initial	r_tx_request = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset || i_cmd_err)
		r_tx_request <= 1'b0;
	else if (new_tx_request)
		r_tx_request <= 1'b1;
	else if (!cmd_busy && !o_cmd_request && !o_tx_en && !w_card_busy)
		r_tx_request <= 1'b0;

	initial	r_tx_sent = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset || !o_tx_en)
		r_tx_sent <= 1'b0;
	else if (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last)
		r_tx_sent <= 1'b1;

	initial	o_tx_en = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset || (i_cmd_err && !o_tx_en))
		o_tx_en <= 1'b0;
	else if (o_tx_en)
	begin
		if (r_tx_sent && i_tx_done)
			o_tx_en <= 1'b0;
	end else if (!o_tx_en)
	begin
		if (!cmd_busy && !o_cmd_request && !w_card_busy && r_tx_request)
			o_tx_en <= r_tx_request;
	end
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(!r_tx_request && !r_tx_sent && !o_tx_en);

	always @(*)
	if (!i_reset && !o_soft_reset)
		assert(!r_tx_request || !o_tx_en);

	always @(posedge i_clk)
	if (!i_reset && $past(!i_reset && !o_soft_reset
					&& r_tx_request && !i_cmd_err))
		assert(r_tx_request || o_tx_en);

	always @(posedge i_clk)
	if (!i_reset && $past(!i_reset && !o_soft_reset && new_data_request
			&& (!bus_wdata[USE_DMA_BIT] || !dma_zero_len)
			&& (!bus_wdata[FIFO_WRITE_BIT] || new_tx_request)))
		assert(r_tx_request || r_rx_request
			||(o_cmd_request && o_cmd_type == R2_REPLY));
`endif
	// }}}

	// o_rx_en, r_rx_request
	// {{{
	initial	r_rx_request = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset || i_cmd_err)
		r_rx_request <= 1'b0;
	else if (new_data_request && !bus_wdata[FIFO_WRITE_BIT]
			&& (!bus_wdata[USE_DMA_BIT] || !dma_zero_len)
			&& (bus_wdata[9:8] != R2_REPLY
					|| bus_wdata[7:6] == NUL_PREFIX))
		r_rx_request <= 1'b1;
	else if (!o_cmd_request)
		r_rx_request <= 1'b0;

	initial	o_rx_en = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset || (i_cmd_err && !o_rx_en))
		o_rx_en <= 1'b0;
	else if (o_rx_en && i_rx_done)
		o_rx_en <= 1'b0;
	else if (!o_cmd_request && r_rx_request)
		o_rx_en <= 1'b1;
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset)
		assert(!r_rx_request || !o_rx_en);
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(!r_rx_request && !o_rx_en);
	always @(posedge i_clk)
	if (!i_reset && $past(!i_reset && !o_soft_reset && (r_rx_request && !i_cmd_err)))
		assert(r_rx_request || o_rx_en);
`endif
	// }}}

	// r_fifo: Control which FIFO this command uses
	// {{{
	initial	r_fifo = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_fifo <= 1'b0;
	else if (!r_mem_busy && !o_tx_en
				&& bus_cmd_stb && bus_wstrb[FIFO_ID_BIT/8])
		r_fifo <= bus_wdata[FIFO_ID_BIT];
	// }}}

	always @(*)
	begin
		clear_err = bus_write && bus_wraddr == ADDR_CMD
			&& bus_wstrb[ERR_BIT/8] && bus_wdata[ERR_BIT];

		if (o_tx_en || r_tx_request || o_rx_en || r_rx_request)
			clear_err = 1'b0;
		if (dma_busy || cmd_busy)
			clear_err = 1'b0;

		if (i_reset || o_soft_reset)
			clear_err = 1'b1;
	end

	// r_cmd_err
	// {{{
	initial	r_cmd_err = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_cmd_err <= 1'b0;
	else if (i_cmd_err) //  || (o_rx_en && i_rx_err))
		r_cmd_err <= 1'b1;
	else if (clear_err && !dma_write)
		r_cmd_err <= 1'b0;

	initial	r_cmd_ecode = 2'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_cmd_ecode <= 2'b0;
	else if (clear_err)
		r_cmd_ecode <= 2'b0;
	else if (!r_cmd_err && i_cmd_done)
		r_cmd_ecode <= i_cmd_ercode;
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
	begin
		assert(r_cmd_err ==  1'b0);
		assert(r_cmd_ecode == 2'b00);
	end
`endif
	// }}}

	// r_transfer_err
	// {{{
	initial	r_transfer_err = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_transfer_err <= 1'b0;
	else begin
		if (clear_err)
			r_transfer_err <= 1'b0;
		if (!dma_busy)
		begin
			if (o_rx_en && i_rx_err)
				r_transfer_err <= 1'b1;
			if (o_tx_en && i_tx_err)
				r_transfer_err <= 1'b1;
		end
	end

	initial	r_ecode = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_ecode <= 1'b0;
	else if (clear_err)
		r_ecode <= 1'b0;
	else if (!r_transfer_err)
	begin
		if (i_rx_err && (!dma_busy || !dma_stopped))
			r_ecode <= i_rx_ercode;
		if (i_tx_err)
			r_ecode <= i_tx_ercode;
	end
`ifdef	FORMAL
	always @(posedge i_clk)
	if (f_past_valid && $past(o_soft_reset))
		assume(!i_rx_done && !i_rx_err);

	always @(*)
	if (!o_rx_en)
		assume(!i_rx_err);

	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
	begin
		assert(!r_transfer_err);
		assert(!r_ecode);
	end
`endif
	// }}}

	always @(*)
	begin
		w_cmd_word = 32'h0;
		w_cmd_word[26] = o_cfg_expect_ack;
		w_cmd_word[25] = !o_hwreset_n;
		w_cmd_word[24] = dma_error;
		w_cmd_word[23] = r_ecode;
		w_cmd_word[22] = r_transfer_err;
		w_cmd_word[21] = r_cmd_err;
		w_cmd_word[20] = w_card_busy;
		w_cmd_word[19] = !card_present;
		w_cmd_word[18] =  card_removed;
		w_cmd_word[17:16] = r_cmd_ecode;
		w_cmd_word[15] = r_cmd_err || r_transfer_err || dma_error;
		w_cmd_word[14] = cmd_busy;
		w_cmd_word[13] = dma_busy;
		w_cmd_word[12] = r_fifo;
		w_cmd_word[11] = r_mem_busy;
		w_cmd_word[10] = (o_tx_en || r_tx_request);
		w_cmd_word[9:8] = o_cmd_type;
		w_cmd_word[6:0] = r_cmd;
	end
	// }}}

	// Command argument register
	// {{{
	initial	r_arg = 32'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_arg <= 0;
	// else if (o_cmd_request && !i_cmd_busy)
	//	r_arg <= 0;
	else if (i_cmd_response)
		r_arg <= i_arg;
	else if (!cmd_busy && bus_write && bus_wraddr == ADDR_ARG)
	begin
		if (bus_wstrb[0])
			r_arg[ 7: 0] <= bus_wdata[ 7: 0];
		if (bus_wstrb[1])
			r_arg[15: 8] <= bus_wdata[15: 8];
		if (bus_wstrb[2])
			r_arg[23:16] <= bus_wdata[23:16];
		if (bus_wstrb[3])
			r_arg[31:24] <= bus_wdata[31:24];
	end

	assign	o_arg = r_arg;
	// }}}

	// PHY control register
	// {{{
	assign	bus_phy_stb = bus_write && bus_wraddr == ADDR_PHY;

	// o_length, lgblk
	// {{{
	initial	lgblk = 4'h9;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		lgblk <= 4'h9;
	else if (!o_tx_en && !o_rx_en && !r_tx_request && !r_rx_request
			&& bus_phy_stb && bus_wstrb[3] && !dma_busy)
	begin
		lgblk <= bus_wdata[27:24];
		if (bus_wdata[27:24] >= LGFIFO)
			lgblk <= LGFIFO;
		else if (bus_wdata[27:24] <= 2)
			lgblk <= 2;
	end

	assign	o_length = 1<<lgblk;
	// }}}

	// o_1p8v: Set to true to enable the hardware to operate at 1.8V
	// {{{
	generate if (OPT_1P8V)
	begin : GEN_1P8V
		reg	new_1p8v, r_1p8v;

		// Chips always start up at 3.3V, so we need to disable 1.8V
		// on startup.  Once they switch to 1.8V, they cannot switch
		// back.  Hence, any 1.8V hardware *MUST* also force the card
		// to reset (i.e. power down the card) any time the FPGA reloads
		// its configuration--else we'll be operating at 3.3V when the
		// card is configured for 1.8V.
		//
		// Note that there is *no* way to test this bit.  We're either
		// in 1.8V, or not, and we can't come back.  Hence, the PHY
		// supports bit[23] = OPT_1P8V, so the user can know that
		// the 1.8V option exists.  If the option exists, the user can
		// then query the card to know if the card supports 1.8V, and
		// then switch the card and other hardware confidently.
		//
		always @(*)
		begin
			// Once 1.8v is set, it should stay set.
			new_1p8v = r_1p8v || (bus_phy_stb
						&& bus_wstrb[VOLTAGE_BIT/8]
						&& bus_wdata[VOLTAGE_BIT]);

			// Our one exception is following a hardware reset
			// request.  In these cases--and only these cases, we
			// allow the 1.8v setting to be cleared.
			if (OPT_HWRESET && bus_write && bus_wraddr == ADDR_CMD
					&& bus_wstrb[HWRESET_BIT/8]
					&& bus_wdata[HWRESET_BIT])
				new_1p8v = 1'b0;

			// Of course, the 1.8v setting is *always* cleared on
			// either a reset, or if the card isn't present in the
			// first place.  These could be protocol errors, if
			// OPT_HWRESET (a forced power shutdown) isn't also
			// supported.
			if (i_reset || (OPT_HWRESET && !o_hwreset_n)
					|| !card_present)
				new_1p8v = 1'b0;
		end

		initial	r_1p8v = 1'b0;
		always @(posedge i_clk)
		if (i_reset || !card_present)
			r_1p8v <= 1'b0;
		else
			r_1p8v <= new_1p8v;

		assign	o_1p8v = r_1p8v;

	end else begin : NO_1P8V
		assign	o_1p8v = 1'b0;
	end endgenerate
	// }}}

	// o_hwreset_n
	// {{{
	generate if (OPT_HWRESET)
	begin : GEN_HWRESET
		localparam	CKRSTW = 100; // 1uS
		reg	r_hwreset, r_hwreset_req;
		reg	[$clog2(CKRSTW+1)-1:0]	r_rst_counter;
		wire	bus_write_reset;

		assign	bus_write_reset = bus_write && bus_wraddr == ADDR_CMD
				&& bus_wstrb[HWRESET_BIT/8];

		initial	r_hwreset_req = 1'b0;
		always @(posedge i_clk)
		if (i_reset)
			r_hwreset_req <= 1'b0;
		else if (bus_write_reset)
			r_hwreset_req <= bus_wdata[HWRESET_BIT];

		initial	r_rst_counter = CKRSTW;
		initial	r_hwreset = 1'b0;	// Start at 0, force a rst edge
		always @(posedge i_clk)
		if (i_reset)
		begin
			r_rst_counter <= CKRSTW;
			r_hwreset <= 1'b1;
		end else if (bus_write_reset && !r_hwreset && bus_wdata[HWRESET_BIT])
		begin
			r_rst_counter <= CKRSTW;
			r_hwreset <= 1'b1;
		end else begin
			if (r_rst_counter > 1)
			begin
				r_rst_counter <= r_rst_counter - 1;
				r_hwreset <= 1;
			end else if ((!bus_write_reset && r_hwreset_req)
				||(bus_write_reset && bus_wdata[HWRESET_BIT]))
			begin
				r_rst_counter <= 1;
				r_hwreset <= 1;
			end else begin
				r_rst_counter <= 0;
				r_hwreset <= 0;
			end

			// r_hwreset <= r_hwreset_req || (r_rst_counter > 1);
		end

		assign	o_hwreset_n = !r_hwreset;
`ifdef	FORMAL
		always @(*)
		if (r_hwreset_req)
			assert(r_hwreset);

		always @(posedge i_clk)
		if (!i_reset && !$past(i_reset) && $past(r_rst_counter) > 1)
			assert(r_rst_counter == $past(r_rst_counter)-1);

		always @(posedge i_clk)
		if (!i_reset && !$past(i_reset) && $past(r_rst_counter) == 1 && !r_hwreset_req)
			assert(r_rst_counter == 0);

		always @(*)
		if (!i_reset)
		begin
			assert(r_hwreset == (r_rst_counter != 0));
			assert(r_rst_counter <= CKRSTW);
			if (r_hwreset_req)
				assert(r_hwreset);
		end
`endif
	end else begin : NO_HWRESET
		assign	o_hwreset_n = 1'b1;
	end endgenerate
	// }}}

	// o_cfg_sample_shift: Control when we sample data returning from card
	// {{{
	initial	o_cfg_sample_shift = 5'h18;
	always @(posedge i_clk)
	begin
		if (i_reset || o_soft_reset)
			o_cfg_sample_shift <= 5'h18;
		else if (bus_phy_stb && bus_wstrb[2])
			o_cfg_sample_shift <= bus_wdata[20:16];

		if(!OPT_SERDES)
			o_cfg_sample_shift[1:0] <= 2'h0;
		if(!OPT_SERDES && !OPT_DDR)
			o_cfg_sample_shift[2] <= 1'h0;
	end
	// }}}

	// o_cfg_shutdown: Shutdown the SD Card clock when not in use
	// o_cfg_clk90: Use a clock that's 90 degrees offset from the data,
	// {{{
	//	vs allowing the data to align with the negative edge of the
	//	clock.  o_cfg_clk90 is required for all DDR modes, but may not
	//	be required for SDR modes.
	initial	{ r_clk_shutdown, o_cfg_clk90 } = 2'b00;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		{ r_clk_shutdown, o_cfg_clk90 } <= 2'b00;
	else if (bus_phy_stb && bus_wstrb[CLK90_BIT/8])
	begin
		r_clk_shutdown <= bus_wdata[CLK_SHUTDOWN_BIT];
		o_cfg_clk90 <= bus_wdata[CLK90_BIT] || bus_wdata[DDR_BIT];
	end

	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cfg_shutdown <= 1'b0;
	else begin
		o_cfg_shutdown <= r_clk_shutdown;
		if (bus_phy_stb && bus_wstrb[1])
			o_cfg_shutdown <= bus_wdata[CLK_SHUTDOWN_BIT];
		if (dma_busy)
			o_cfg_shutdown <= 1'b1;
		if (w_card_busy)
			o_cfg_shutdown <= 1'b0;
		if (r_tx_request || r_rx_request)
			o_cfg_shutdown <= 1'b0;
		if (o_tx_en && !i_tx_done)
			o_cfg_shutdown <= 1'b0;

		// No checks for RX shutdown--that needs to be done from the
		// receiver itself.

		if (o_cmd_request || (cmd_busy && !i_cmd_done))
			o_cfg_shutdown <= 1'b0;
	end
`ifdef	FORMAL
	always @(posedge i_clk)
	if (!i_reset)
	begin
		if (!r_clk_shutdown && !$past(dma_busy))
			assert(!o_cfg_shutdown);
		if ((cmd_busy && !o_cmd_request) || o_tx_en
						|| (w_card_busy && !cmd_busy))
			assert(!o_cfg_shutdown);
	end
`endif
	// }}}

	// o_pp_cmd, o_pp_data: If set, configure cmd/data to push-pull modes
	// {{{
	// If clear, the cmd/data lines will be left in their original
	// open-drain modes.  The SD spec requires that the card start in
	// open-drain, and then switch to push-pull modes when speeding up
	// the clock to higher speeds.  o_pp_cmd controls whether or not the
	// command wire is operated in push-pull mode, whereas o_pp_data
	// controls whether or not the data lines are operated in push-pull
	// mode.
	initial	{ o_pp_cmd, o_pp_data } = 2'b00;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		{ o_pp_cmd, o_pp_data } <= 2'b00;
	else if (bus_phy_stb && bus_wstrb[1])
	begin
		o_pp_cmd  <= bus_wdata[PP_CMD_BIT];
		o_pp_data <= bus_wdata[PP_DATA_BIT];
	end
	// }}}

	// o_cfg_ds: Enable return data strobe support
	// {{{
	generate if (OPT_DS && OPT_EMMC)
	begin : GEN_DS_CONTROL
		reg	r_cfg_ds, r_cfg_dscmd;

		initial	r_cfg_ds = 1'b0;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_cfg_ds <= 1'b0;
		else if (bus_phy_stb && bus_wstrb[DDR_BIT/8])
			r_cfg_ds <= bus_wdata[DS_BIT] && bus_wdata[DDR_BIT];

		initial	r_cfg_dscmd = 1'b0;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_cfg_dscmd <= 1'b0;
		else if (bus_phy_stb)
		begin
			case(bus_wstrb[2:1])
			2'b00: begin end
			2'b10: r_cfg_dscmd<= bus_wdata[DSCMD_BIT] && o_cfg_ds;
			2'b01: r_cfg_dscmd<= o_cfg_dscmd   && (&bus_wdata[9:8]);
			2'b11: r_cfg_dscmd<= bus_wdata[DSCMD_BIT] && (&bus_wdata[9:8]);
			endcase
		end

		assign	o_cfg_ds = r_cfg_ds;
		assign	o_cfg_dscmd = r_cfg_dscmd;
	end else begin : NO_DS_CONTROL
		assign	o_cfg_ds = 1'b0;
		assign	o_cfg_dscmd = 1'b0;
	end endgenerate
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_cfg_clk90)
		assert(!o_cfg_ddr);
	always @(*)
	if (!i_reset && (!o_cfg_ddr || !OPT_DS || !OPT_EMMC))
		assert(!o_cfg_ds);
	always @(*)
	if (!i_reset && !o_cfg_ds)
		assert(!o_cfg_dscmd);
`endif
	// }}}

	// o_cfg_ddr: Transmit data on both edges of the clock
	// {{{
	//	Note: this requires o_cfg_clk90 support
	initial	o_cfg_ddr = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cfg_ddr <= 1'b0;
	else if (bus_phy_stb && bus_wstrb[DDR_BIT/8])
		o_cfg_ddr <= bus_wdata[DDR_BIT];
	// }}}

	// o_cfg_width: Control the number of data bits, whether 1, 4, or 8
	// {{{
	//	SDIO uses either 1 or 4 data bits.
	//	eMMC can use 1, 4, or 8 data bits.
	initial	r_width = WIDTH_1W;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_width <= WIDTH_1W;
	else if (bus_phy_stb && bus_wstrb[1])
	begin
		case(bus_wdata[11:10])
		2'b00: r_width <= WIDTH_1W;
		2'b01: if (NUMIO < 4) r_width <= WIDTH_1W;
			else r_width <= WIDTH_4W;
		2'b10: if (NUMIO >= 8) r_width <= WIDTH_8W;
		2'b11: if (NUMIO >= 8) r_width <= WIDTH_8W;
			else if (NUMIO >= 4)	r_width <= WIDTH_4W;
			else			r_width <= WIDTH_1W;
		default: begin end
		endcase
	end

	assign	o_cfg_width = r_width;
	// }}}

	// o_cfg_ckspeed: Clock speed control
	// {{{
	wire	w_clk90;
	assign	w_clk90 = (bus_phy_stb && bus_wstrb[CLK90_BIT/8])
		? (bus_wdata[DDR_BIT]||bus_wdata[CLK90_BIT]) : o_cfg_clk90;

	initial	r_ckspeed = 252;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_ckspeed <= 252;
	else if (bus_phy_stb)
	begin
		if (bus_wstrb[0])
		begin
			r_ckspeed <= bus_wdata[7:0];
			if (!OPT_SERDES && !OPT_DDR && bus_wdata[7:0] <= 2)
			begin
				r_ckspeed <= 8'h2;
				if (w_clk90)
					r_ckspeed <= 8'h3;
			end else if (!OPT_SERDES && bus_wdata[7:0] <= 1)
			begin
				r_ckspeed <= 8'h1;
				if (w_clk90)
					r_ckspeed <= 8'h2;
			end
		end else if (!OPT_SERDES && bus_wstrb[CLK90_BIT/8]
				&& (bus_wdata[CLK90_BIT] || bus_wdata[DDR_BIT]))
		begin
			if (OPT_DDR && r_ckspeed <= 1)
				r_ckspeed <= 2;
			if (!OPT_DDR && r_ckspeed <= 2)
				r_ckspeed <= 3;
		end
	end

	assign	o_cfg_ckspeed = r_ckspeed;
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !OPT_SERDES && !OPT_DDR)
		assert(o_cfg_ckspeed >= 2);

	always @(*)
	if (!i_reset && !OPT_SERDES)
		assert(o_cfg_ckspeed >= 1);

	always @(*)
	if (!i_reset && o_cfg_clk90 && !OPT_SERDES)
	begin
		if (OPT_DDR)
		begin
			assert(o_cfg_ckspeed >= 2);
		end else
			assert(o_cfg_ckspeed >= 3);
	end
`endif
	// }}}

	always @(*)
	begin
		w_phy_ctrl = 0;
		w_phy_ctrl[31:28] = LGFIFO; // Can also set lgblk=15, & read MAX
		w_phy_ctrl[27:24] = lgblk;
		w_phy_ctrl[23]    = OPT_1P8V;
		w_phy_ctrl[22]    = i_1p8v;
		w_phy_ctrl[21]    = o_cfg_dscmd;
		w_phy_ctrl[20:16] = o_cfg_sample_shift;
		w_phy_ctrl[15]    = r_clk_shutdown;
		w_phy_ctrl[14]    = o_cfg_clk90;
		w_phy_ctrl[13]    = o_pp_cmd;	// Push-pull CMD line
		w_phy_ctrl[12]    = o_pp_data;	// Push-pull DAT line(s)
		w_phy_ctrl[11:10] = r_width;
		w_phy_ctrl[9:8]   = { o_cfg_ds, o_cfg_ddr };
		w_phy_ctrl[7:0]   = i_ckspd; // r_ckspeed;
	end
	// }}}

	assign	o_crc_en = 1'b1;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Card detection
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Depends upon the i_card_detect signal.  Set this signal to 1'b1 if
	// you your device doesn't have it.
	//
	//
	generate if (OPT_CARD_DETECT)
	begin : GEN_CARD_DETECT
		reg	[2:0]	raw_card_present;
`ifdef	FORMAL
		reg	[1:0]	card_detect_counter;
`else
		reg	[9:0]	card_detect_counter;
`endif
		reg		r_card_removed, r_card_present;

		// 3FF cross clock domains: i_card_detect->raw_card_present
		// {{{
		initial	raw_card_present = 0;
		always @(posedge i_clk)
		if (i_reset)
			raw_card_present <= 0;
		else
			raw_card_present <= { raw_card_present[1:0], i_card_detect };
		// }}}

		// card_removed: If the card is ever not present, then it has
		// {{{
		// been removed.  Set the removed flag so the user can see the
		// card has been removed, even if they come back to it later
		// and there's a card present.
		initial	r_card_removed = 1'b1;
		always @(posedge i_clk)
		if (i_reset)
			r_card_removed <= 1'b1;
		else if (!card_present)
			r_card_removed <= 1'b1;
		else if (bus_cmd_stb && bus_wdata[CARD_REMOVED_BIT]
					&& bus_wstrb[CARD_REMOVED_BIT/8])
			r_card_removed <= 1'b0;

		assign	card_removed = r_card_removed;
		// }}}

		// card_present: Require a card to be inserted for a period
		// {{{
		// of time before declaring it to be present.  This helps
		// to unbounce any card detection logic.
		initial	card_detect_counter = 0;
		always @(posedge i_clk)
		if (i_reset || !raw_card_present[2])
			card_detect_counter <= 0;
		else if (!(&card_detect_counter))
			card_detect_counter <= card_detect_counter + 1;

		initial r_card_present = 1'b0;
		always @(posedge i_clk)
		if (i_reset || !raw_card_present[2])
			r_card_present <= 1'b0;
		else if (&card_detect_counter)
			r_card_present <= 1'b1;

		assign	card_present = r_card_present;
		// }}}

	end else begin : NO_CARD_DETECT_SIGNAL

		assign	card_present = 1'b1;
		assign	card_removed = 1'b0;

		// Keep Verilator happy
		// {{{
		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused_card_detect;
		assign	unused_card_detect = &{ 1'b0, i_card_detect };
		// Verilator lint_on  UNUSED
		// Verilator coverage_on
		// }}}
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Interrupt generation
	// {{{

	initial	o_int = 0;
	always @(posedge i_clk)
	if (i_reset)
		o_int <= 1'b0;
	else begin
		o_int <= 1'b0;
		// 5 Types of interrupts:
		//
		case({ (o_tx_en || r_tx_request), (o_rx_en || r_rx_request)})
		// A) Command operation is complete, response is ready
		2'b00: if (i_cmd_done && cmd_busy) o_int <= 1'b1;
		//
		// B) Transmit to card operation is complete, and is now ready
		//	for another command (if desired)
		2'b10: if (o_tx_en && r_tx_sent && i_tx_done) o_int <= 1'b1;
		//
		// C) A block has been received.  We are now ready to receive
		//	another block (if desired)
		2'b01: if (o_rx_en && i_rx_done) o_int <= 1'b1;
		default: begin end
		endcase

		//
		// D) Any command error generates an interrupt
		if (i_cmd_done && i_cmd_err) o_int <= 1'b1;

		// The DMA will set the interrupt as well.
		if (dma_int)
			o_int <= 1'b1;

		// As will any time the card ceases to be busy
		if (card_was_busy && !w_card_busy)
			o_int <= 1'b1;

		// Now we need to suppress interrupts if operations remain
		//  ongoing
		if (cmd_busy && !i_cmd_done)
			o_int <= 1'b0;
		if (r_mem_busy && ((o_tx_en && !i_tx_done) || (o_rx_en && !i_rx_done)))
			o_int <= 1'b0;
		if (dma_busy && !dma_int)
			o_int <= 1'b0;
		if (w_card_busy)
			o_int <= 1'b0;
		if ((r_rx_request || r_tx_request)&&(!i_cmd_done || !i_cmd_err))
			o_int <= 1'b0;

		if (o_soft_reset)
			o_int <= 1'b0;

		//
		// E) A card has been removed or inserted, and yet not
		// akcnowledged.
		if (OPT_CARD_DETECT && !card_present && !card_removed)
			o_int <= 1'b1;
		if (OPT_CARD_DETECT && card_present && card_removed)
			o_int <= 1'b1;
		//
		// F) When the hardware reset clears?
		// if (HWRESETN && !last_hwreset_n && o_hwreset_n)
		//	o_int <= 1'b1;
	end
`ifdef	FORMAL
	wire	f_busy;

	assign	f_busy = w_cmd_word[20] || w_cmd_word[14]
				|| (dma_int||w_cmd_word[13]) || w_cmd_word[11];

	always @(posedge i_clk)
	if (!i_reset && !$past(i_reset)
		&& !$past(o_soft_reset) && !$past(o_soft_reset,2)
		&& $past(!OPT_CARD_DETECT || card_present != card_removed)
		&& (!OPT_CARD_DETECT || card_present != card_removed)
		&& !$past(bus_write) && !$past(bus_write,2))
	begin
		if ($past(w_card_busy) == $past(w_card_busy,2)
			&& w_card_busy == $past(w_card_busy))
		begin
			assert(o_int == $fell(f_busy));
		end else if (!f_busy && !$past(f_busy))
			assert(o_int == ($past(!w_card_busy) && $past(w_card_busy,2)));
	end
`endif

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// FIFO / Memory handling
	// {{{

	// User write pointer
	// {{{
	initial	fif_wraddr = 0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		fif_wraddr <= 0;
	else if (!dma_busy)
	begin
		if (bus_cmd_stb && ((bus_wstrb[1]
			&& (bus_wdata[USE_FIFO_BIT]
				|| bus_wdata[USE_DMA_BIT]
				|| bus_wdata[9:8] == R2_REPLY
				|| (r_fifo != bus_wdata[FIFO_ID_BIT])))
			|| (!r_mem_busy)
			|| (bus_wstrb[0] && bus_wdata[7])))
		begin
			// The DMA will clear this on a new command as well
			// as the user on a WB write
			fif_wraddr <= 0;
		end else if (bus_write && bus_wstrb[0]
			&&(bus_wraddr==ADDR_FIFOA || bus_wraddr==ADDR_FIFOB))
			fif_wraddr <= fif_wraddr + 1;
	end else if (dma_busy && i_s2sd_valid && o_s2sd_ready)
	begin
		if (dma_last)
			fif_wraddr <= 0;
		else
			fif_wraddr <= fif_wraddr + 1;
	end
	// }}}

	// User read pointer
	// {{{
	initial	fif_rdaddr = 0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		fif_rdaddr <= 0;
	else if (dma_busy)
	begin
		if (!dma_read_active)
			fif_rdaddr <= 0;
		else if (dma_read_fifo)
			fif_rdaddr <= fif_rdaddr + 1;
	end else if (bus_cmd_stb && new_data_request)
	begin
		fif_rdaddr <= 0;
	end else if (bus_cmd_stb && (
		(bus_wstrb[0] && bus_wdata[7])
		|| !r_mem_busy
		|| (bus_wstrb[1] && bus_wdata[FIFO_ID_BIT] != r_fifo)))
	begin
		fif_rdaddr <= 0;
	end else if (bus_read && i_wb_sel[0]
			&&(bus_rdaddr== ADDR_FIFOA || bus_rdaddr == ADDR_FIFOB))
		fif_rdaddr <= fif_rdaddr + 1;
`ifdef	FORMAL
	// always @(*)
	// if (!i_reset && !o_soft_reset && !o_hwreset_n)
	//	assert(fif_rdaddr == 0);
`endif
	// }}}

	// TX

	// o_tx_mem_valid
	// {{{
	initial	o_tx_mem_valid = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_tx_mem_valid <= 0;
	else if (!o_tx_en || r_tx_sent
			|| (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last))
		{ o_tx_mem_valid, tx_pipe_valid } <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid)
		{ o_tx_mem_valid, tx_pipe_valid } <= { tx_pipe_valid, 1'b1 };
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(!o_tx_mem_valid);
`endif
	// }}}

	// tx_mem_addr
	// {{{
	always @(posedge i_clk)
	if (i_reset || !o_tx_en || o_soft_reset || o_tx_mem_last || r_tx_sent)
		tx_mem_addr <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid)
		tx_mem_addr <= tx_mem_addr + 1;
`ifdef	FORMAL
	always @(*)
	if (!i_reset && !o_soft_reset && !o_hwreset_n)
		assert(tx_mem_addr == 0);
`endif
	// }}}

	always @(*)
		fif_a_rdaddr = (o_tx_en && !r_fifo) ? tx_mem_addr[LGFIFO32-1:$clog2(MW/32)] : fif_rdaddr;

	always @(*)
		fif_b_rdaddr = (o_tx_en &&  r_fifo) ? tx_mem_addr[LGFIFO32-1:$clog2(MW/32)] : fif_rdaddr;

	always @(*)
		blk_words = (1<<(lgblk-2))-1;

	// Verilator lint_off WIDTH
	always @(*)
		pre_tx_last = o_tx_en && (tx_mem_addr[LGFIFO32-1:$clog2(MW/32)] >= blk_words);
	// Verilator lint_on  WIDTH

	always @(posedge i_clk)
	if ((!r_fifo && (o_tx_en || !OPT_LOWPOWER)
				&& (!o_tx_mem_valid || i_tx_mem_ready))
			|| (r_fifo && (!dma_busy || dma_read_fifo)))
		tx_fifo_a <= fifo_a[fif_a_rdaddr];

	always @(posedge i_clk)
	if ((r_fifo && (o_tx_en || !OPT_LOWPOWER)
				&& (!o_tx_mem_valid || i_tx_mem_ready))
			|| (!r_fifo && (!dma_busy || dma_read_fifo)))
		tx_fifo_b <= fifo_b[fif_b_rdaddr];

	always @(posedge i_clk)
	if (OPT_LOWPOWER && (i_reset || o_soft_reset || !o_tx_en || r_tx_sent))
		tx_fifo_last <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready)
		tx_fifo_last <= pre_tx_last;

	// o_tx_mem_data
	// {{{
	generate if (MW <= 32)
	begin : NO_TX_SHIFT
		assign	tx_shift = 0;
	end else begin : GEN_TX_SHIFT
		reg	[$clog2(MW/32)-1:0]	r_tx_shift;

		always @(posedge i_clk)
		if (OPT_LOWPOWER && (i_reset || o_soft_reset || !o_tx_en || r_tx_sent))
			r_tx_shift <= 0;
		else if (!o_tx_mem_valid || i_tx_mem_ready)
			r_tx_shift <= tx_mem_addr[$clog2(MW/32)-1:0];

		assign	tx_shift = r_tx_shift;
	end endgenerate

	always @(*)
	begin
		next_tx_mem = (r_fifo) ? tx_fifo_b : tx_fifo_a;
		next_tx_mem = next_tx_mem >> (32*tx_shift);
	end

	always @(posedge i_clk)
	if ((!o_tx_mem_valid || i_tx_mem_ready) && !r_tx_sent)
		o_tx_mem_data <= next_tx_mem[31:0];
	// }}}

	// o_tx_mem_last
	// {{{
	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready)
		o_tx_mem_last <= tx_fifo_last;
	// }}}

	// Writing to memory

	// Take a clock to arbitrate writes
	// {{{
	// WARNING: This isn't a proper arbiter.  There is no ability to stall
	// at present if multiple sources attempt to write to the FIFO at the
	// same time.  Hence always make sure to write to the FIFO that isn't
	// currently in use.  It is the software's responsibility to ping-pong.
	// Failing to ping pong may result in corruption.
	//

	// mem_wr_[strb|addr|data]_a: Arbitrate writes to FIFO A
	// {{{
	initial	mem_wr_strb_a = 0;
	always @(posedge i_clk)
	begin
		mem_wr_strb_a <= 0;

		if (!dma_busy && bus_write && bus_wraddr == ADDR_FIFOA
				&& (|bus_wstrb) && (!r_mem_busy || r_fifo))
		begin
			mem_wr_addr_a <= fif_wraddr;
			mem_wr_strb_a <= bus_wstrb;
			mem_wr_data_a <= bus_wdata;
		end

		if (dma_busy && !dma_fifo && i_s2sd_valid && o_s2sd_ready)
		begin
			mem_wr_addr_a <= fif_wraddr;
			mem_wr_strb_a <= -1;
			mem_wr_data_a <= i_s2sd_data;
		end

		if (!r_fifo && i_cmd_mem_valid)
		begin
			mem_wr_addr_a <= i_cmd_mem_addr;
			mem_wr_strb_a <= i_cmd_mem_strb;
			mem_wr_data_a <= i_cmd_mem_data;
		end

		if (!r_fifo && i_rx_mem_valid)
		begin
			mem_wr_addr_a <= i_rx_mem_addr;
			mem_wr_strb_a <= i_rx_mem_strb;
			mem_wr_data_a <= i_rx_mem_data;
		end

		if (i_reset || o_soft_reset)
			mem_wr_strb_a <= 0;
	end
	// }}}

	// mem_wr_[strb|addr|data]_b: Arbitrate writes to FIFO B
	// {{{
	initial	mem_wr_strb_b = 0;
	always @(posedge i_clk)
	begin
		mem_wr_strb_b <= 0;

		if (!dma_busy && bus_write && bus_wraddr == ADDR_FIFOB
				&& (|bus_wstrb) && (!r_mem_busy || !r_fifo))
		begin
			mem_wr_addr_b <= fif_wraddr;
			mem_wr_strb_b <= bus_wstrb;
			mem_wr_data_b <= bus_wdata;
		end

		if (dma_busy && dma_fifo && i_s2sd_valid && o_s2sd_ready)
		begin
			mem_wr_addr_b <= fif_wraddr;
			mem_wr_strb_b <= -1;
			mem_wr_data_b <= i_s2sd_data;
		end

		if (r_fifo && i_cmd_mem_valid)
		begin
			mem_wr_addr_b <= i_cmd_mem_addr;
			mem_wr_strb_b <= i_cmd_mem_strb;
			mem_wr_data_b <= i_cmd_mem_data;
		end

		if (r_fifo && i_rx_mem_valid)
		begin
			mem_wr_addr_b <= i_rx_mem_addr;
			mem_wr_strb_b <= i_rx_mem_strb;
			mem_wr_data_b <= i_rx_mem_data;
		end

		if (i_reset || o_soft_reset)
			mem_wr_strb_b <= 0;
	end
	// }}}

`ifdef	FORMAL
	always @(*)
	if (!i_reset)
		assert(!i_cmd_mem_valid || !i_rx_mem_valid);
`endif
	// }}}

	// Actually write to the memories
	// {{{
	always @(posedge i_clk)
	for(ika=0; ika<MW/8; ika=ika+1)
	if (mem_wr_strb_a[ika])
		fifo_a[mem_wr_addr_a[LGFIFOW-1:0]][ika*8 +: 8] <= mem_wr_data_a[ika*8 +: 8];


	always @(posedge i_clk)
	for(ikb=0; ikb<MW/8; ikb=ikb+1)
	if (mem_wr_strb_b[ikb])
		fifo_b[mem_wr_addr_b[LGFIFOW-1:0]][ikb*8 +: 8] <= mem_wr_data_b[ikb*8 +: 8];
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// DMA processing
	// {{{
	generate if (OPT_DMA)
	begin : GEN_DMA_COMMANDS
		// {{{
		// Local declarations
		// {{{
		localparam	DMA_ADDR_LO = (OPT_LITTLE_ENDIAN) ? 3'h5 : 3'h6,
				DMA_ADDR_HI = (OPT_LITTLE_ENDIAN) ? 3'h6 : 3'h5,
				DMA_ADDR_LN = 3'h7;
		localparam [31:0]	DMA_STOP_TRANSMISSION = 32'h834c,
					DMA_NULL_READ = 32'h8800,
					DMA_NULL_WRITE= 32'h8c00;

		reg			r_dma, r_abort, r_tx, pre_dma_valid,
					r_dma_write, r_dma_stopped,r_last_block,
					r_dma_err, r_dma_fifo, r_dma_last,
					r_dma_int, dma_cmd_fifo,
					dma_s2sd, dma_sd2s, dma_last_beat,
					r_dma_zero_len, r_read_active,
					w_release_dma, w_dma_abort;
		reg	[31:0]		wide_block_count, r_block_count,
					r_dma_command;
		reg	[63:0]		wide_dma_addr;
		reg	[DMA_AW-1:0]	r_dma_addr;
		reg	[1:0]		r_dma_loaded;
		reg	[LGFIFO-2:0]	r_subblock;
		reg	[LGFIFO:0]	r_dma_len;
		reg			r_sd2s_valid;
		reg	[31:0]		r_sd2s_data;
		// }}}

		// r_dma, o_dma_s2sd, o_dma_sd2s, r_tx
		// {{{
		always @(posedge i_clk)
		if (!dma_busy && new_dma_request)
			r_tx <= bus_wdata[FIFO_WRITE_BIT];

		always @(*)
		begin
			w_release_dma= r_dma_zero_len || r_dma_err;
			if(r_mem_busy || i_dma_busy || o_dma_s2sd || o_dma_sd2s)
				w_release_dma = 1'b0;
			if (cmd_busy || !r_dma_stopped)
				w_release_dma = 1'b0;
			if (!r_dma)
				w_release_dma = 1'b0;
		end

		initial	{ r_dma, dma_s2sd, dma_sd2s } = 3'h0;
		always @(posedge i_clk)
		if (i_reset)
		begin
			// {{{
			r_dma        <= 1'b0;
			dma_s2sd   <= 1'b0;
			dma_sd2s   <= 1'b0;
			// }}}
		end else if (o_soft_reset)
		begin
			// {{{
			r_dma <= 1'b0;
			dma_s2sd   <= 1'b0;
			dma_sd2s   <= 1'b0;
			// o_dma_abort <= dma_busy;
			// }}}
		end else if (!dma_busy) // i.e. if !r_dma
		begin
			// {{{
			if (new_dma_request && !r_dma_zero_len
				&& !r_mem_busy && !o_tx_en && card_present
				&& (!dma_error || clear_err))
			begin // User command to activate the DMA
				r_dma <= 1'b1;
				if (bus_wdata[FIFO_WRITE_BIT])
					{ dma_s2sd, dma_sd2s } <= 2'b10;
				else
					// Reads have to wait for a full FIFO
					{ dma_s2sd, dma_sd2s } <= 2'b00;
			end
			// }}}
		end else if (!i_dma_busy && !o_dma_s2sd && !o_dma_sd2s)//&&r_dma
		begin
			{ dma_s2sd, dma_sd2s } <= 2'b00;

			if (r_dma_zero_len || r_dma_err)
			begin
				if (w_release_dma)
					// If we've finished, shut down
					r_dma <= 1'b0;
			end else if (r_tx)
			begin
				// Otherwise, if we are transmitting, wait
				// until our buffer is unloaded, then command
				// the DMA to load it.
				dma_s2sd <= !r_dma_loaded[r_dma_fifo] && !r_dma_err && !w_dma_abort;
			end else begin
				// If we are receiving, wait until the buffer
				// is fully loaded, then write it out.
				dma_sd2s <= r_dma_loaded[r_dma_fifo] && !r_dma_err && !w_dma_abort;
			end
		end else if (r_abort || i_dma_busy)	// &&r_dma
			{ dma_s2sd, dma_sd2s } <= 2'b00;

		always @(posedge i_clk)
		if (i_reset)
		begin
			r_dma_int <= 1'b0;
		end else if (o_soft_reset)
		begin
			r_dma_int <= 1'b0;
		end else if (!dma_busy && new_dma_request)
		begin // User command to activate the DMA
			// {{{
			r_dma_int <= 1'b0;
			if (r_dma_zero_len)
				r_dma_int <= 1'b1;
			// This one will generate an interrupt on its own,
			//   so no interrupt is necessary ... here.
			if (r_mem_busy || o_tx_en) // Also, clear potential Zero
				r_dma_int <= 1'b0; // length interrupts here
			if (!card_present)
				r_dma_int <= 1'b1;
			if (dma_error && !clear_err)
				r_dma_int <= 1'b1;
			// }}}
		end else if (w_release_dma)
			r_dma_int <= 1'b1;
		else
			r_dma_int <= 1'b0;

		assign	o_dma_s2sd = dma_s2sd;
		assign	o_dma_sd2s = dma_sd2s;
		assign	dma_busy = r_dma;
		assign	dma_int  = r_dma_int;
		assign	dma_tx   = r_tx;
`ifdef	FORMAL
		// {{{
		always @(*)
		if (!i_reset && !r_dma)
			assert(!dma_sd2s && !dma_s2sd);

		always @(*)
		if (!i_reset && !o_soft_reset && !o_hwreset_n)
			assert(!r_dma);

		always @(posedge i_clk)
		if (!i_reset && $past(r_abort))
		begin
			assert(!dma_sd2s);
			assert(!dma_s2sd);
		end

		always @(*)
		if (!i_reset && r_dma)
		begin
			if (r_tx)
			begin
				assert(!dma_sd2s); // && !f_dma_sd2s_busy);
				assert(!o_rx_en && !r_rx_request);
			end else begin
				assert(!dma_s2sd); // && !f_dma_s2sd_busy);
				assert(!o_tx_en && !r_tx_request);
			end

			if (dma_zero_len)
			begin
				if (!r_tx)
				begin
					assert(!o_rx_en && !r_rx_request);
					assert(r_dma_loaded == 0);
				end
			end else if (r_block_count == 1)
			begin
				assert(r_tx || !(&r_dma_loaded));
				if (r_dma_loaded != 0)
					assert(!o_rx_en && !r_rx_request);
			end

			if (dma_sd2s)
				assert(r_dma_loaded[dma_fifo]);
			if (dma_s2sd)
				assert(!r_dma_loaded[dma_fifo]);
			if (r_tx_request || o_tx_en)
			begin
				if (!r_tx_sent)
				begin
					assert(r_dma_fifo != r_fifo
					|| (!o_dma_s2sd && !i_dma_busy));

					assert(r_dma_loaded[r_fifo] || dma_error);
				end else begin
					// assert(!r_dma_loaded[r_fifo]);
					// assert(!o_dma_s2sd || dma_fifo != r_fifo);
					// assert(!i_s2sd_valid || dma_fifo != r_fifo);
				end
			end
			if (r_rx_request || o_rx_en)
				assert(!r_dma_loaded[r_fifo]);
		end

		always @(posedge i_clk)
		if (!f_past_valid || $past(i_reset || o_soft_reset))
		begin
			assert(!dma_int || !f_past_valid);
		end else if ($past(new_dma_request))
		begin
			assert(dma_int || dma_busy);
		end else if ($past(dma_busy) && !dma_busy)
		begin
			assert(dma_int);
		end else begin
			assert(!dma_int);
		end
		// }}}
`endif
		// }}}

		// r_dma_addr
		// {{{
		always @(*)
		begin
			wide_dma_addr[63:0] = 0;
			wide_dma_addr[DMA_AW-1:0] = r_dma_addr;

			if (bus_write && bus_wraddr == DMA_ADDR_LO)
			begin
				if (bus_wstrb[0])
					wide_dma_addr[ 7: 0] = bus_wdata[ 7: 0];
				if (bus_wstrb[1])
					wide_dma_addr[15: 8] = bus_wdata[15: 8];
				if (bus_wstrb[2])
					wide_dma_addr[23:16] = bus_wdata[23:16];
				if (bus_wstrb[3])
					wide_dma_addr[31:24] = bus_wdata[31:24];
				if (OPT_STREAM && DMA_AW <= 32)
					wide_dma_addr[DMA_AW-1] = (bus_wstrb[3]) ? wide_dma_addr[31] : r_dma_addr[DMA_AW-1];
			end

			if (bus_write && bus_wraddr == DMA_ADDR_HI)
			begin
				if (bus_wstrb[0])
					wide_dma_addr[39:32] = bus_wdata[ 7: 0];
				if (bus_wstrb[1])
					wide_dma_addr[47:40] = bus_wdata[15: 8];
				if (bus_wstrb[2])
					wide_dma_addr[55:48] = bus_wdata[23:16];
				if (bus_wstrb[3])
					wide_dma_addr[63:56] = bus_wdata[31:24];
				if (OPT_STREAM && DMA_AW > 32 && bus_wstrb[3])
					wide_dma_addr[DMA_AW-1] = (bus_wstrb[3]) ? wide_dma_addr[63] : r_dma_addr[DMA_AW-1];
			end
		end

		always @(posedge i_clk)
		if (i_reset)
			r_dma_addr <= 0;
		else if (!dma_busy && bus_write)
		begin
			r_dma_addr <= wide_dma_addr[DMA_AW-1:0];
			//
			// Can't zero the unused address here, lest we prevent
			// a two step address update--low address then upper,
			// where the prior stream address prevents the lower
			// addresses from updating.
			// if (OPT_STREAM && wide_dma_addr[DMA_AW-1])
			//	r_dma_addr[DMA_AW-2:0] <= 0;
		end else if (OPT_STREAM && r_dma_addr[DMA_AW-1])
		begin
			// Stream operations don't adjust the address
		end else if ((i_s2sd_valid && o_s2sd_ready)
				|| (o_sd2s_valid && i_sd2s_ready))
		begin
			r_dma_addr <= r_dma_addr + 4;

			// Prevent the stream bit from getting enabled
			// mid-transaction.  This means that we might still
			// wrap around memory should the DMA transfer size be
			// large enough.
			if (OPT_STREAM)
				r_dma_addr[DMA_AW-1] <= 1'b0;
		end

		assign	o_dma_addr = r_dma_addr;
		// }}}

		// r_abort
		// {{{
		always @(*)
		begin
			w_dma_abort = 1'b0;
			if (o_soft_reset || !card_present)
				w_dma_abort = 1'b1;
			if (i_dma_err || i_cmd_err || (o_rx_en && i_rx_err)
					|| i_tx_err)
				w_dma_abort = 1'b1;
			if (!dma_busy)
				w_dma_abort = 1'b0;
		end

		always @(posedge i_clk)
		if (i_reset || dma_error)
		begin
			r_abort <= 1'b0;
		end else if (w_dma_abort)
		begin
			r_abort <= 1'b0;
			if (o_dma_s2sd || o_dma_sd2s || i_dma_busy)
				r_abort <= 1'b1;
		end else
			r_abort <= 1'b0;

		assign	o_dma_abort = r_abort;
		// }}}

		// r_dma_err
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset || clear_err) // !card_present)
			r_dma_err <= 1'b0;
		else if (w_dma_abort)
			r_dma_err <= 1'b1;

		assign	dma_error = r_dma_err;
		// }}}

		// r_subblock, r_dma_last, o_dma_len: based on [io]_s[d]2s[d]*
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset || !dma_busy)
		begin
			r_subblock <= blk_words[LGFIFO-2:0];
			r_dma_last <= (lgblk <= 2);
			r_dma_len  <= 1<<lgblk;
		end else if ((i_s2sd_valid && o_s2sd_ready)
				|| (o_sd2s_valid && i_sd2s_ready))
		begin
			if (r_dma_last)
			begin
				r_subblock <= blk_words[LGFIFO-2:0];
				r_dma_last <= (lgblk <= 2);
			end else begin
				r_subblock <=  r_subblock - 1;
				r_dma_last <= (r_subblock <= 1);
			end
		end

		assign	o_dma_len = r_dma_len;
		assign	dma_last = r_dma_last;
`ifdef	FORMAL
		// {{{
		wire	[LGFIFOW-1:0]	f_dma_rdaddr;

		always @(*)
		if (!i_reset && dma_busy && r_dma_loaded[dma_fifo] == r_tx && !dma_error)
		begin
			assert(!r_dma_last);
			assert(r_subblock == blk_words);
		end

		always @(*)
		if (!i_reset && dma_busy && !dma_error)
		begin
			assert(r_dma_last == (r_subblock == 0));
			assert(r_subblock <= f_blocksz-1);
			if (!r_tx && !r_read_active)
				assert(r_subblock == f_blocksz-1);
		end

		assign	f_dma_rdaddr = f_blocksz-1 + (pre_dma_valid ? 1:0)
					+ (r_sd2s_valid ? 1:0) - r_subblock;
		always @(*)
		if (!i_reset && dma_busy && !dma_error)
		begin
			if (r_tx)
			begin
				assert(!o_sd2s_valid);
				assert(!dma_read_fifo);
				assert(fif_wraddr ==(f_blocksz- r_subblock-1));
			end else begin
				if (!r_sd2s_valid && !r_dma_err)
				begin
					assert(r_subblock == f_blocksz-1);
					assert(!dma_last || lgblk == 2);
				end

				if (r_read_active)
					assert(fif_rdaddr == f_dma_rdaddr
						|| r_dma_err);
				else begin
					assert(!pre_dma_valid);
					assert(!r_sd2s_valid);
					if (!dma_last)
						assert(r_subblock==f_blocksz-1);
				end
				if (r_sd2s_valid && !dma_last)
					assert(pre_dma_valid);
			end
		end
		// }}}
`endif
		// }}}

		// r_dma_fifo
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_dma_fifo <= 1'b0;
		else if (!dma_busy)
		begin
			r_dma_fifo <= bus_wdata[FIFO_ID_BIT];
		end else if ((i_s2sd_valid && o_s2sd_ready && r_dma_last)
				||(o_sd2s_valid && i_sd2s_ready && r_dma_last))
			r_dma_fifo <= !r_dma_fifo;

		assign	dma_fifo = r_dma_fifo;
		// }}}

		// r_dma_loaded
		// {{{
		// When writing, ... when is the FIFO loaded and ready to be
		// sent to the SD card?  When reading, ... when is the FIFO
		// unloaded and ready to be filled again?
		always @(posedge i_clk)
		if (i_reset || o_soft_reset || !dma_busy || dma_error)
			r_dma_loaded <= 2'b0;
		else if (r_tx)
		begin
			// Read into the FIFO, write FIFO to flash
			if (i_s2sd_valid && o_s2sd_ready && r_dma_last)
				r_dma_loaded[r_dma_fifo] <= 1'b1;
			if (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last)
				r_dma_loaded[r_fifo] <= 1'b0;
		end else begin
			if (o_sd2s_valid && i_sd2s_ready && o_sd2s_last)
				r_dma_loaded[r_dma_fifo] <= 1'b0;
			if (o_rx_en && i_rx_done)
				r_dma_loaded[r_fifo] <= 1'b1;
		end
		// }}}

		// dma_last_beat
		// {{{
		always @(*)
		begin
			if (r_tx)
				dma_last_beat = i_s2sd_valid && o_s2sd_ready;
			else
				dma_last_beat = o_sd2s_valid && i_sd2s_ready;

			if (!r_dma_last)
				dma_last_beat = 1'b0;
		end
		// }}}

		// r_block_count, r_last_block
		// {{{
		always @(*)
		begin
			wide_block_count = r_block_count;

			if (bus_wstrb[0])
				wide_block_count[ 7: 0] = bus_wdata[ 7: 0];
			if (bus_wstrb[1])
				wide_block_count[15: 8] = bus_wdata[15: 8];
			if (bus_wstrb[2])
				wide_block_count[23:16] = bus_wdata[23:16];
			if (bus_wstrb[3])
				wide_block_count[31:24] = bus_wdata[31:24];
		end

		always @(posedge i_clk)
		if (i_reset)
		begin
			r_block_count <= 0;
			r_last_block <= 1;
			r_dma_zero_len <= 1'b1;
		end else if (o_soft_reset)
		begin
			r_block_count <= 0;
			r_last_block <= 1;
			r_dma_zero_len <= 1'b1;
		end else if (dma_busy)
		begin
			if (dma_last_beat)
			begin
				if (!r_dma_zero_len)
					r_block_count <= r_block_count - 1;
				r_last_block <= (r_block_count <= 2);
				r_dma_zero_len <= (r_block_count <= 1);
			end
		end else if (bus_write && bus_wraddr == DMA_ADDR_LN)
		begin // && !dma_busy
			r_block_count <= wide_block_count;
			r_last_block  <= (wide_block_count <= 1);
			r_dma_zero_len <= (wide_block_count == 0);
		end

		assign	dma_zero_len = r_dma_zero_len;
		assign	dma_len_return = r_block_count;
`ifdef	FORMAL
		always @(*)
		if (!i_reset)
			assert(r_last_block == (r_block_count <= 1));
		always @(*)
		if (!i_reset)
			assert(dma_zero_len == (r_block_count == 0));
		always @(*)
		if (f_past_valid && dma_zero_len && r_tx)
		begin
			assume(!i_dma_busy);
			assert(!o_dma_s2sd);
		end
`endif
		// }}}

		// dma_cmd_fifo
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			dma_cmd_fifo <= 1'b0;
		else if (new_dma_request)
		begin
			if (bus_wdata[FIFO_WRITE_BIT])
				// We have to come back later to issue a
				// request to this ID
				dma_cmd_fifo <= bus_wdata[FIFO_ID_BIT];
			else
				// Request is being issued now, next one is
				// opposite.  dma_cmd_fifo is the *next* FIFO
				// ID, so ... swap.
				dma_cmd_fifo <= !bus_wdata[FIFO_ID_BIT];
		end else if (dma_write)
			dma_cmd_fifo <= !dma_cmd_fifo;
`ifdef	FORMAL
		always @(*)
		if (!i_reset && !o_soft_reset && !o_hwreset_n)
			assert(!dma_write);
`endif
		// }}}

		// dma_write, dma_command, dma_stopped
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset || !dma_busy)
		begin
			r_dma_write    <= 1'b0;
			r_dma_stopped  <= 1'b0;
		end else if (dma_write)
			r_dma_write <= 1'b0;
		else if (!o_dma_s2sd && !o_dma_sd2s && !r_dma_stopped)
		begin
			// FIXME: Reads can be stopped before the DMA transfer
			//  stops, writes cannot.  Below waits for DMA reads
			//  to complete entirely, a bit of an overkill.
			if (r_dma_err || (dma_zero_len
				&& (!r_tx || r_dma_loaded == 0)))
			begin // Send STOP_TRANSMISSION
				// {{{
				if (!cmd_busy && (!r_tx
					||(!o_tx_en
						&& (r_dma_err || r_dma_loaded == 2'b0))))
				begin
					r_dma_write <= 1'b1;
					// STOP_TRANSMISSION
					r_dma_stopped <= 1'b1;
				end
				// }}}
			end else if (!i_dma_busy && !o_tx_en && !o_rx_en
					&& !cmd_busy && !r_dma_err
					&& !r_tx_request && !r_rx_request)
			begin // Have the DMA read/write another block
				// {{{
				if ((r_tx ^ r_dma_loaded[dma_cmd_fifo]) == 1'b0)
					r_dma_write <= 1'b1;
				if (!r_tx && (|r_dma_loaded) && r_last_block)
					r_dma_write <= 1'b0;
				// }}}
			end else
				r_dma_write <= 1'b0;
		end

		// r_dma_command
		// {{{
		// Separate this from the logic above, in order to spare the
		// logic required for 32 bits--hence saving both logic and
		// (potentially) power.
		always @(posedge i_clk)
		if (r_dma && !dma_write)
		begin
			if (r_tx)
				r_dma_command <= DMA_NULL_WRITE;
			else
				r_dma_command <= DMA_NULL_READ;

			if (r_dma_err || (dma_zero_len
					&&(!r_tx || r_dma_loaded== 2'b0)))
			begin
				r_dma_command <= DMA_STOP_TRANSMISSION;
			end

			r_dma_command[FIFO_ID_BIT] <= dma_cmd_fifo;
		end
		// }}}

		assign	dma_write   = r_dma_write;
		assign	dma_command = r_dma_command;
		assign	dma_stopped = r_dma_stopped;
`ifdef	FORMAL
		// {{{
		always @(*)
		if (!i_reset && !o_soft_reset && !o_hwreset_n)
			assert(dma_stopped == 1'b0);

		always @(posedge i_clk)
		if (!i_reset && !$past(i_reset) && $rose(r_dma_stopped))
		begin
			assert(dma_write);
			assert(dma_command[FIFO_ID_BIT-1:0]
				== DMA_STOP_TRANSMISSION[FIFO_ID_BIT-1:0]);
			assert(dma_command[31:FIFO_ID_BIT+1]
				== DMA_STOP_TRANSMISSION[31:FIFO_ID_BIT+1]);
		end else if (!i_reset && dma_write)
		begin
			if (r_tx)
			begin
				assert(dma_command[FIFO_ID_BIT-1:0] == DMA_NULL_WRITE[FIFO_ID_BIT-1:0]);
			end else
				assert(dma_command[FIFO_ID_BIT-1:0] == DMA_NULL_READ[FIFO_ID_BIT-1:0]);
		end
		// }}}
`endif
		// }}}

		// o_sd2s_valid, pre_dma_valid
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset || !dma_busy || r_tx || dma_error)
			{ r_sd2s_valid, pre_dma_valid } <= 2'b00;
		else if (o_sd2s_valid && i_sd2s_ready && o_sd2s_last)
			{ r_sd2s_valid, pre_dma_valid } <= 2'b00;
		else if (dma_read_fifo)
			{ r_sd2s_valid, pre_dma_valid } <= { pre_dma_valid, 1'b1 };
		assign	o_sd2s_valid = r_sd2s_valid;
`ifdef	FORMAL
		always @(*)
		if (i_reset) begin
		end else if (!dma_busy || r_tx || r_dma_zero_len)
			// No receive outputs allowed while transmitting
			assert(!r_sd2s_valid && !pre_dma_valid);
		else if (!r_dma_loaded[dma_fifo])
			assert(!r_sd2s_valid && !pre_dma_valid);
`endif
		// }}}

		// o_sd2s_data
		// {{{
		always @(posedge i_clk)
		if (OPT_LOWPOWER && (i_reset || o_soft_reset || !i_dma_busy))
		begin
			r_sd2s_data <= 32'h0;
		end else if (dma_read_fifo && (!OPT_LOWPOWER || pre_valid))
		begin
			if (dma_fifo)
				r_sd2s_data <= tx_fifo_b;
			else
				r_sd2s_data <= tx_fifo_a;
		end

		assign	o_sd2s_data = r_sd2s_data;
`ifdef	FORMAL
		always @(*)
		if (OPT_LOWPOWER && !i_reset && !r_sd2s_valid)
			assert(r_sd2s_data == 0);
`endif
		// }}}

		// r_read_active
		// {{{
		always @(posedge i_clk)
		if (i_reset || o_soft_reset || r_tx || r_abort)
			r_read_active <= 1'b0;
		else if (o_dma_sd2s)
			r_read_active <= 1'b1;
		else if (o_sd2s_valid && i_sd2s_ready && o_sd2s_last)
			r_read_active <= 1'b0;

		assign	dma_read_active = r_read_active;
		// }}}

		assign	dma_read_fifo = (!r_tx && r_read_active
					&& !o_dma_sd2s
					&& r_dma_loaded[dma_fifo]
					&& (!o_sd2s_valid || i_sd2s_ready));
		assign	o_s2sd_ready = dma_busy && r_tx && !r_dma_zero_len && (!r_dma_loaded[dma_fifo]);
		assign	o_sd2s_last  = r_dma_last && !r_tx;

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_dma;
		assign	unused_dma = &{ 1'b0, wide_dma_addr[63:DMA_AW] };
		// Verilator lint_on  UNUSED
		// }}}
`ifdef	FORMAL
		// {{{
		reg	[31:0]	f_rx_blocks, f_tx_blocks;
		reg		f_cfg_fifo;
		reg[DMA_AW-1:0]	f_cfg_addr;
		reg	[31:0]	f_cfg_len;

		// assumptions about i_dma_busy
		// {{{
		// Busy only rises when requested
		always @(posedge i_clk)
		if (!$past(o_dma_s2sd) && !$past(o_dma_sd2s))
			assume(!$rose(i_dma_busy));

		always @(posedge i_clk)
		if (!i_reset && !dma_busy)
			// i_dma_busy can't start without a request
			assert(!i_dma_busy);

		always @(posedge i_clk)
		if (i_reset || $past(i_reset)
				|| $past(o_soft_reset) || $past(o_dma_abort))
		begin
			assume(!i_dma_busy);
		end else if ($past(i_s2sd_valid && o_s2sd_ready && dma_last))
		begin // Busy always falls after last on TX
			assume($fell(i_dma_busy));
		end else if ($past(o_sd2s_valid && i_sd2s_ready && dma_last))
		begin // Busy always falls after last on RX
			assume($fell(i_dma_busy));
		end else if ($past(o_dma_s2sd) || $past(o_dma_sd2s))
		begin // Busy always rises on request
			assume(i_dma_busy);
		end else if ($past(i_dma_err))
		begin // Busy always falls following an error
			assume(!i_dma_busy);
		end else
			assume($stable(i_dma_busy));

	//	always @(posedge i_clk)
	//	if (!r_dma_last && r_subblock != ((1<<(lgblk-2))-1))
	//		assume(i_dma_busy);

		always @(posedge i_clk)
		if (!i_dma_busy)
			// Valid only rises if the DMA is busy
			assume(!i_s2sd_valid);
		// }}}

		// f_cfg_* configuration copy
		// {{{
		always @(posedge i_clk)
		if (!dma_busy)
		begin
			if (bus_cmd_stb && bus_wstrb[FIFO_ID_BIT/8])
				f_cfg_fifo <= bus_wdata[FIFO_ID_BIT];
			f_cfg_addr <= o_dma_addr;
			f_cfg_len  <= r_block_count;
		end
		// }}}

		// f_rx_blocks
		// {{{
		always @(posedge i_clk)
		if (i_reset || !dma_busy || !o_hwreset_n)
			f_rx_blocks <= 0;
		else if (r_tx && i_s2sd_valid && o_s2sd_ready && dma_last)
			f_rx_blocks <= f_rx_blocks + 1;
		else if (!r_tx && o_rx_en && i_rx_done)
			f_rx_blocks <= f_rx_blocks + 1;
		// }}}

		// f_tx_blocks
		// {{{
		always @(posedge i_clk)
		if (i_reset || !dma_busy || !o_hwreset_n)
			f_tx_blocks <= 0;
		else if (r_tx && o_tx_mem_valid && i_tx_mem_ready
							&& o_tx_mem_last)
			f_tx_blocks <= f_tx_blocks + 1;
		else if (!r_tx && o_sd2s_valid && i_sd2s_ready && dma_last)
			f_tx_blocks <= f_tx_blocks + 1;
		// }}}

		// AXI Stream properties
		// {{{
		always @(posedge i_clk)
		if (!f_past_valid || $past(i_reset) || $past(o_soft_reset)
			|| $past(r_abort))
		begin
			assume(!i_s2sd_valid);
		end else if ($past(i_s2sd_valid && !o_s2sd_ready))
		begin
			assume(i_s2sd_valid);
			assume($stable(i_s2sd_data));
		end

		always @(posedge i_clk)
		if (!f_past_valid || $past(i_reset) || $past(o_soft_reset)
			|| $past(r_abort))
		begin
			assert(!o_sd2s_valid || !f_past_valid);
		end else if ($past(o_sd2s_valid && !i_sd2s_ready))
		begin
			assert(o_sd2s_valid);
			assert($stable(o_sd2s_data));
			assert($stable(o_sd2s_last));
		end
		// }}}

		// DMA Error/Abort properties
		// {{{
		always @(posedge i_clk)
		if (!i_reset && !$past(dma_busy))
			assume(!i_dma_err);

		always @(posedge i_clk)
		if (i_reset || $past(i_reset))
			assume(!i_dma_err);
		else if (!$past(i_dma_busy))
			assume(!i_dma_err);

		always @(posedge i_clk)
		if (!i_reset && $past(r_abort))
			assert(!r_abort);

		always @(posedge i_clk)
		if (!i_reset && r_abort && !$past(o_soft_reset))
			assert(r_dma_err);

		always @(posedge i_clk)
		if (!i_reset && !$rose(r_dma_err) && !$past(o_soft_reset))
			assert(!$rose(r_abort));

		always @(posedge i_clk)
		if (!i_reset && !r_dma_err)
		begin
			if (!$past(o_soft_reset))
				assert(!r_abort);
		end

		always @(posedge i_clk)
		if (!i_reset && $past(!i_reset && i_dma_busy && !card_present))
		begin
			// Abort on any card removal
			assert(r_dma_err || $past(o_soft_reset));
			assert(r_abort || $past(r_abort));
		end

		always @(posedge i_clk)
		if (!i_reset && r_dma_err)
		begin
			assert(!o_dma_sd2s || r_abort);
			assert(!o_dma_s2sd || r_abort);
			assert(!i_dma_busy || r_abort);
		end
		// }}}

		always @(posedge i_clk)
		if (!f_past_valid || $past(i_reset)
				|| $past(o_soft_reset) || $past(!o_hwreset_n))
		begin
			assert(!r_dma);
		end else if ($past(w_release_dma))
		begin
			assert(!r_dma);
		end else if (dma_error && $past(dma_error)
			&& !$past(cmd_busy || r_mem_busy || o_tx_en || r_abort) && !dma_write)
		begin
			if (!i_dma_busy && !r_mem_busy && !o_tx_en && !cmd_busy && !r_abort)
				assert(!r_dma);
		end

		always @(*)
		if (!i_reset && dma_busy && !dma_error)
		begin
			if (r_tx)
			begin
				assert(dma_fifo==(f_rx_blocks[0] ^ f_cfg_fifo));
				if (r_dma_stopped)
				begin
					assert(!r_tx_request);
					assert(!o_tx_en || r_tx_sent);
					assert(!r_rx_request);
					assert(!o_rx_en);
				end else if (r_dma_err)
				begin
				end else if ((r_tx_request || o_tx_en) && (!r_tx_sent))
				begin
					assert(dma_cmd_fifo==(f_tx_blocks[0]
						^ f_cfg_fifo ^ 1));
				end else begin
					assert(dma_cmd_fifo==(f_tx_blocks[0]
						^ f_cfg_fifo));
				end

				if (f_tx_blocks == 0)
					assert(!r_tx_sent);
				if (r_dma_err)
				begin
				end else if (f_tx_blocks == 0 && (!o_tx_en && !r_tx_request))
				begin
					assert(dma_cmd_fifo == r_fifo);
				end else begin
					assert(dma_cmd_fifo != r_fifo);
				end
				// r_fifo
				// dma_cmd_fifo
			end else if (!r_dma_err)
			begin
				assert(dma_fifo==(f_tx_blocks[0] ^ f_cfg_fifo));
				assert(dma_cmd_fifo != r_fifo);
			end
		end

		always @(*)
		if (!i_reset && dma_busy && !dma_error)
		begin
			assert(f_rx_blocks   <= f_cfg_len);
			assert(f_tx_blocks   <= f_cfg_len);
			assert(r_block_count <= f_cfg_len);
			assert(f_tx_blocks <= f_rx_blocks);
			if (r_tx)
			begin
				assert(f_rx_blocks + r_block_count== f_cfg_len); // !!!

				assert((f_tx_blocks + (r_dma_loaded[0] ? 1:0)
					+ (r_dma_loaded[1] ? 1:0)) == f_rx_blocks);
				if (dma_zero_len)
					assert(!o_s2sd_ready);
				if (r_dma_stopped && !r_dma_err && !r_abort)
				begin
					assert(r_dma_loaded == 0);
					assert(f_tx_blocks == f_rx_blocks);
					assert(f_tx_blocks == f_cfg_len);
					assert(!o_tx_en && !r_tx_request);
				end
			end else begin
				assert(f_tx_blocks + r_block_count == f_cfg_len);

				assert((f_tx_blocks + (r_dma_loaded[0] ? 1:0)
					+ (r_dma_loaded[1] ? 1:0)) == f_rx_blocks);
				if (r_dma_stopped && !r_dma_err && !r_abort)
				begin
					// assert(f_tx_blocks == f_rx_blocks);
					assert(f_tx_blocks == f_cfg_len);
					assert(!o_rx_en && !r_rx_request);
				end
			end
		end

		////////////////////////////////////////////////////////////////
		//
		// Cover checks
		// {{{
		always @(posedge i_clk)
		if (f_past_valid)
			cover($fell(o_soft_reset));

		always @(posedge i_clk)
		if (f_past_valid && !$past(i_reset) && !$past(o_soft_reset))
		begin
			cover(i_wb_stb);
			cover(i_wb_stb && i_wb_we);
			cover(i_wb_stb && i_wb_we && !o_wb_stall);
			cover(bus_write);
			cover(bus_write && bus_wstrb);
			cover(bus_write && bus_wstrb == 4'hf
					&& bus_wdata[15:0] == 16'ha850);
			cover(bus_write && bus_wstrb == 4'hf
					&& bus_wdata[15:0] == 16'haa50);
			cover(dma_busy);
			cover(dma_busy && r_dma &&  r_tx && dma_last_beat);
			cover(dma_busy && r_dma && !r_tx && dma_last_beat);
			if (!$past(r_dma_err) && !$past(r_abort) && $past(dma_busy))
			begin
				cover(!dma_busy &&  r_tx);	// Step 23
				cover(!dma_busy && !r_tx);	// Step 23
				cover(!dma_busy &&  r_tx && f_cfg_len > 1); //30
				cover(!dma_busy && !r_tx && f_cfg_len > 1); //29
				cover(!dma_busy &&  r_tx && f_cfg_len > 2); //37
				cover(!dma_busy && !r_tx && f_cfg_len > 2); //35
				cover(!dma_busy &&  r_tx && f_cfg_len > 1 && lgblk == 3);
				cover(!dma_busy && !r_tx && f_cfg_len > 1 && lgblk == 3);
			end
		end
		// }}}
		////////////////////////////////////////////////////////////////
		//
		// "Careless" assumptions
		// {{{
		// always @(*)
		// if (dma_busy) assume(r_tx);
		always @(*)
		if (dma_busy)
			assume(lgblk > 2);
		// }}}
		// }}}
`endif
		// }}}
	end else begin : NO_DMA
		// {{{
		// Internal control signals
		assign	dma_write     = 1'b0;
		assign	dma_command   = 32'h0;
		assign	dma_error     = 1'b0;	// True on bus error, i_dma_err
		assign	dma_busy      = 1'b0;	// || i_dma_busy
		assign	dma_fifo      = 1'b0;
		assign	dma_read_fifo = 1'b0;
		assign	dma_last = 1'b0;
		assign	dma_zero_len  = 1'b1;
		assign	dma_int       = 1'b0;
		assign	dma_stopped   = 1'b1;
		assign	dma_read_active = 1'b0;
		assign	dma_tx   = 1'b0;
		//
		// Common control signals
		assign	o_dma_addr   = 0;
		assign	o_dma_len    = 0;
		assign	o_dma_abort  = 1'b0;
		// SD2S signals
		assign	o_dma_sd2s   = 1'b0;	// Activate the S2MM DMA
		assign	o_sd2s_valid = 1'b0;
		assign	o_sd2s_data  = 32'h0;
		assign	o_sd2s_last  = 1'b0;
		// S2SD signals
		assign	o_dma_s2sd   = 1'b0;	// Activate the S2SD DMA
		assign	o_s2sd_ready = 1'b1;

		assign	dma_len_return = 0;

		// Keep Verilator happy with the DMA
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_dma;
		assign	unused_dma = &{ 1'b0, new_dma_request,
				i_s2sd_valid, i_s2sd_data,
				i_sd2s_ready, i_dma_err, i_dma_busy
				};
		// Verilator lint_on  UNUSED
		// }}}
`ifdef	FORMAL
		// {{{
		always @(*)
		begin
			assert(!dma_busy);
			assert(!dma_int);
			assert(dma_stopped);

			// With no DMA, we shouldn't be getting DMA signals
			assume(!i_s2sd_valid);
			assume(!i_dma_busy);
			assume(!i_dma_err);

			assert(!o_dma_sd2s);
			assert(!o_sd2s_valid);

			assert(!o_dma_s2sd);
			// assert(!o_sd2s_valid);
		end
		// }}}
`endif
		// }}}
	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone Return handling
	// {{{
	always @(*)
	begin
		dma_addr_return = 0;
		dma_addr_return[DMA_AW-1:0] = o_dma_addr;
		if (OPT_STREAM && o_dma_addr[DMA_AW-1])
		begin
			dma_addr_return = 0;
			if (DMA_AW <= 32)
			begin
				dma_addr_return[31] = 1'b1;
				// dma_addr_return[63] = 1'b1;
			end else
				dma_addr_return[63] = 1'b1;
		end
	end

	always @(posedge i_clk)
	begin
		pre_data <= 0;

		case(bus_rdaddr)
		ADDR_CMD: pre_data[31:0] <= w_cmd_word;
		ADDR_ARG: pre_data[31:0] <= r_arg;
		ADDR_PHY: pre_data[31:0] <= w_phy_ctrl;
		// 3'h3: pre_data <= w_ffta_word;
		// 3'h4: pre_data <= w_fftb_word;
		3'h5: pre_data[31:0] <= (OPT_LITTLE_ENDIAN)
			? dma_addr_return[31:0] : dma_addr_return[63:32];
		3'h6: pre_data[31:0] <= (OPT_LITTLE_ENDIAN)
			? dma_addr_return[63:32] : dma_addr_return[31:0];
		3'h7: pre_data[31:0] <= dma_len_return;
		default: begin end
		endcase

		if (!bus_read)
			pre_data <= 0;

		pre_sel <= 0;
		if (bus_read)
		begin
			if (bus_rdaddr == ADDR_FIFOA)
				pre_sel <= 1;
			else if (bus_rdaddr == ADDR_FIFOB)
				pre_sel <= 2;
		end
	end

	always @(posedge i_clk)
	begin
		bus_rddata <= 0;
		case(pre_sel)
		2'h0: bus_rddata[31:0] <= pre_data;
		2'h1: bus_rddata <= tx_fifo_a;
		2'h2: bus_rddata <= tx_fifo_b;
		default: begin end
		endcase
	end

	assign	o_wb_stall = (dma_write && i_wb_we);

	initial	{ o_wb_ack, pre_valid } = 2'b00;
	always @(posedge i_clk)
	if (i_reset || !i_wb_cyc)
		{ o_wb_ack, pre_valid } <= 2'b00;
	else
		{ o_wb_ack, pre_valid } <= { pre_valid, i_wb_stb && !o_wb_stall };

	assign	o_wb_data = bus_rddata;
	// }}}

	/*
	assign	o_debug = { w_cmd_word[ERR_BIT], w_card_busy,		// 1b
		// Command:
		o_cmd_request, cmd_busy || (i_cmd_busy && o_cmd_request),
					i_cmd_done,			// 7b
			i_cmd_err, i_cmd_ercode, i_cmd_response,
		// DMA:
		i_dma_busy, i_dma_err, o_dma_abort,			// 3b
		o_dma_sd2s, o_sd2s_valid, i_sd2s_ready, o_sd2s_last,	// 4b
		o_dma_s2sd, i_s2sd_valid, o_s2sd_ready,			// 3b
		// TX:
		o_tx_mem_valid, o_tx_mem_valid && i_tx_mem_ready, o_tx_mem_last,		// 7b
				o_tx_en, r_tx_request, i_tx_done, i_tx_err,
		// RX:
		i_rx_mem_valid, i_rx_done, i_rx_err,			// 6b
			o_rx_en ? i_rx_ercode : i_tx_ercode,
			r_rx_request, o_rx_en
	};
	*/

	// Keep Verilator happy
	// {{{
	wire	unused;
	assign	unused = &{ 1'b0, next_tx_mem };
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
	reg	f_past_valid;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);

	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone interface
	// {{{
	localparam	F_LGDEPTH=3;
	wire	[F_LGDEPTH-1:0]	fwb_nacks, fwb_nreqs, fwb_outstanding;

	fwb_slave #(
		.AW(3), .DW(32),
		.F_LGDEPTH(3)
	) fwb (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		.i_wb_cyc(i_wb_cyc), .i_wb_stb(i_wb_stb), .i_wb_we(i_wb_we),
		.i_wb_addr(i_wb_addr), .i_wb_data(i_wb_data),
				.i_wb_sel(i_wb_sel),
		.i_wb_ack(o_wb_ack), .i_wb_stall(o_wb_stall),
			.i_wb_idata(o_wb_data), .i_wb_err(1'b0),
		.f_nreqs(fwb_nreqs), .f_nacks(fwb_nacks),
			.f_outstanding(fwb_outstanding)
		// }}}
	);

	always @(posedge i_clk)
	if (f_past_valid && i_wb_cyc)
	begin
		assert(fwb_outstanding == (pre_valid ? 1:0) + (o_wb_ack ? 1:0));
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Command processing
	// {{{

	always @(*)
	if (!i_reset && o_cmd_request)
		assert(cmd_busy);

	always @(posedge i_clk)
	if (f_past_valid && $past(i_reset || (i_cmd_done && !w_selfreply_request)))
		assert(!cmd_busy);

	// always @(posedge i_clk)
	// if (f_past_valid && cmd_busy && !o_cmd_request)
	//	assume(i_cmd_busy);		// ???

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset) || $past(o_soft_reset))
	begin end
	else if ($past(o_cmd_request && !i_cmd_busy))
		assume(i_cmd_busy);

	always @(posedge i_clk)
	if (f_past_valid && (!cmd_busy || o_cmd_request))
		assume(!i_cmd_done && !i_cmd_err && !i_cmd_mem_valid
					&& !i_cmd_response);

	always @(*)
	if (!cmd_busy || o_cmd_request || o_cmd_type != R2_REPLY || !i_cmd_busy)
		assume(!i_cmd_mem_valid);

	always @(posedge i_clk)
	if (f_past_valid && $past(!i_reset && o_cmd_request && !i_cmd_busy))
	begin
		assume(i_cmd_ercode == 2'b00);

		assert(!o_cmd_request);
	end

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset || o_soft_reset))
		assert(!o_cmd_request);
	else if ($past(o_cmd_request && i_cmd_busy))
	begin
		assert(o_cmd_request);
		assert($stable(o_cmd_selfreply));
		assert($stable(o_cmd_type));
		assert($stable(o_cmd_id));
		assert($stable(o_arg));
	end

	// Self reply can only be set if o_cmd_request is also set
	always @(*)
	if (!i_reset)
	begin
		if (o_cmd_selfreply)
		begin
			assert(o_cmd_request && o_cmd_type == RNO_REPLY);
		end
	end

	// Only one interface should ever have access to the FIFO at a time
	always @(*)
	if (!i_reset)
	begin
		if (o_tx_en || r_tx_request)
		begin
			assert(!o_rx_en && !r_rx_request);
			assert(!cmd_busy || o_cmd_type != R2_REPLY);
		end else if (o_rx_en || r_rx_request)
		begin
			assert(!cmd_busy || o_cmd_type != R2_REPLY);
		end
	end

	always @(posedge i_clk)
	if (f_past_valid && cmd_busy && o_cmd_type == R2_REPLY)
		assert(!dma_busy);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Contract commands (listed at the top)
	// {{{
	reg	f_mem_request;

	always @(*)
	begin
		f_mem_request = 1'b0;
		if (bus_wdata[9:6] == { R2_REPLY, CMD_PREFIX })
			f_mem_request = 1'b1;
		if (OPT_DMA && bus_wdata[USE_DMA_BIT])
			f_mem_request = 1'b1;
		if (bus_wdata[USE_FIFO_BIT]) // && !bus_wdata[USE_DMA_BIT])
			f_mem_request = 1'b1;

		if (!bus_cmd_stb && bus_wstrb[1:0] != 2'b11)
			f_mem_request = 1'b0;
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset || o_soft_reset || !o_hwreset_n)
		&& o_hwreset_n
		&& !$past(o_cmd_request)
		&& !$past(cmd_busy)
		&& $past(bus_cmd_stb))
	begin
		if ($past(bus_wstrb[1:0] != 2'b11))
		begin
			assert(!o_cmd_request);
		end else if ($past((r_mem_busy || o_tx_en) && f_mem_request))
		begin
			// Can't start a command that would use memory, if the
			// memory is already in use
			assert(!o_cmd_request);
		end else begin
			if ($past(OPT_HWRESET && bus_wstrb[3] && bus_wdata[25])
				|| $past(bus_wdata[7]))
			begin
				assert(!o_cmd_request);
				assert(!o_cmd_selfreply);
			end else if ($past(bus_wdata[7:6] == 2'b01))
			begin
				assert(o_cmd_request
					|| ($past(bus_wdata[USE_DMA_BIT] && (!OPT_DMA || dma_busy || dma_zero_len))));
			end else if (OPT_EMMC && $past(bus_wdata[9:6] == 4'b00
					&&  bus_wdata[5:0] != 6'h0
					&& !bus_wdata[USE_DMA_BIT]
					&& !bus_wdata[USE_FIFO_BIT]))
			begin
				assert(o_cmd_request && o_cmd_selfreply);
			end else
				assert(!o_cmd_request);
		end

		if (o_cmd_request)
		begin
			assert(o_cmd_type == $past(bus_wdata[9:8]));
			assert(o_cmd_id   == $past(bus_wdata[6:0]));
			assert(o_arg      == $past(r_arg));
			assert(cmd_busy);
		end
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset || o_soft_reset || !o_hwreset_n)
		&& !$past(r_mem_busy || o_tx_en)
		&& !$past(r_cmd_err)
		&& !$past(cmd_busy)
		&& !$past(dma_busy)
		&& $past(bus_cmd_stb) && $past(&bus_wstrb[1:0]))
	begin
		if ($past(bus_wdata == 16'h0240))
		begin
			assert(o_cmd_request);
			assert(r_mem_busy);
		end

		if ($past(bus_wdata) == 16'h0940)
		begin
			assert(o_cmd_request);
			assert(r_rx_request);
		end

		if ($past(bus_wdata) == 16'h0d40)
		begin
			assert(o_cmd_request);
			assert(r_tx_request);
		end

		if ($past(bus_wdata) == 16'h0800)
		begin
			assert(!o_cmd_request);
			assert(r_rx_request);
		end

		if ($past(bus_wdata) == 16'h0c00)
		begin
			assert(!o_cmd_request);
			assert(r_tx_request);
		end
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(dma_busy) && $past(bus_cmd_stb)
		&&(($past(bus_wstrb[1:0])==2'b11 && $past(bus_wdata == 16'h080))
		  || $past(!r_mem_busy || (bus_wstrb[1] && bus_wdata[FIFO_ID_BIT] != r_fifo))
		  || $past(!r_mem_busy && bus_wstrb[1] && bus_wdata[USE_FIFO_BIT])
		  || $past(bus_wstrb[0] && bus_wdata[7])
		  ||($past(!o_cmd_request)&&(o_cmd_request
				&& o_cmd_type == R2_REPLY))))
	begin
		assert(fif_wraddr == 0);
		assert(fif_rdaddr == 0);
		if ($past(bus_wstrb[0] && bus_wdata[7]))
		begin
			assert(!$rose(o_cmd_request));
			assert(!$rose(r_mem_busy));
		end
	end

	always @(posedge i_clk)
	if (f_past_valid && !OPT_EMMC)
		assert(!o_cmd_selfreply);

	always @(posedge i_clk)
	if (f_past_valid && OPT_EMMC && !$past(i_reset || o_soft_reset || !o_hwreset_n)
		&& o_hwreset_n
		&& !$past(o_cmd_request)
		&& $past(bus_cmd_stb) && $past(&bus_wstrb[1:0]))
	begin
		if ($past(bus_wdata[15:0] == 16'h0028
			|| bus_wdata[15:0] == 16'h0040))
		begin
			assert(o_cmd_request);
			assert(o_cmd_selfreply);
			assert(o_cmd_type == RNO_REPLY);
			assert(o_cmd_id == $past(bus_wdata[6:0]));
		end
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// TX Handling
	// {{{
	always @(*)
	if (!o_tx_en || !r_tx_sent)
	begin
		assume(!i_tx_done);
		assume(!i_tx_err);
	end

	always @(*)
	if (!o_tx_en)
		assert(!o_tx_mem_valid);

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset || o_soft_reset))
		assert(!o_tx_en);
	else if ($past(o_tx_mem_valid && !i_tx_mem_ready))
	begin
		assert(o_tx_mem_valid);
		assert($stable(o_tx_mem_data));
		assert($stable(o_tx_mem_last));
	end

	always @(*)
	if (!i_reset && o_tx_en)
		assert(!r_tx_request);

	// always @(posedge i_clk)
	//	assert(!o_tx_en || !o_rx_en);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// RX Handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(*)
	if (!o_rx_en)
		assume(!i_rx_mem_valid);

	always @(*)
	if (!i_reset)
		assert(!r_rx_request || !o_rx_en);

	always @(*)
	if (!i_reset)
		assert(!(r_rx_request || o_rx_en)|| !(r_tx_request || o_tx_en));

	always @(*)
	if (!i_reset)
		assert(!(cmd_busy && o_cmd_type == R2_REPLY)
			|| !(r_rx_request || o_rx_en));

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset) || $past(o_soft_reset))
		assume(i_rx_mem_strb == 0);
	else if (!$past(o_rx_en) || $past(i_rx_done))
		assume(i_rx_mem_strb == 0);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// FIFO check
	// {{{
	integer	fk;
	(* anyconst *)	reg	[LGFIFO32:0]	fc_addr;
	reg	[31:0]		f_word, f_mem;
	reg	[LGFIFO32:0]	f_txaddr;
	wire	[LGFIFOW:0]	fcwaddr;

	assign	fcwaddr = fc_addr[LGFIFO32:$clog2(MW/32)];

	initial	begin
		for(fk=0; fk<LGFIFOW; fk=fk+1)
		begin
			fifo_a[fk] = 0;
			fifo_b[fk] = 0;
		end
	end

	always @(*)
	if (!fc_addr[LGFIFO32])
		f_mem = fifo_a[fcwaddr[LGFIFOW-1:0]];
	else
		f_mem = fifo_b[fcwaddr[LGFIFOW-1:0]];

	always @(posedge i_clk)
	if (i_reset)
	begin
		assume(f_mem == 0);
		assume(f_word == 0);
	end else begin
		assert(f_word == f_mem);
	end

	initial	f_word = 0;
	always @(posedge i_clk)
	if (!fc_addr[LGFIFO32] && mem_wr_addr_a[LGFIFOW-1:0]
					== fc_addr[LGFIFO32-1:$clog2(MW/32)])
	begin
		for(fk=0; fk<MW/8; fk=fk+1)
		if (mem_wr_strb_a[fk])
			f_word[fk*8 +: 8] <= mem_wr_data_a[fk*8 +: 8];
	end else if (fc_addr[LGFIFO32] && mem_wr_addr_b[LGFIFOW-1:0]
					== fc_addr[LGFIFO32-1:$clog2(MW/32)])
	begin
		for(fk=0; fk<MW/8; fk=fk+1)
		if (mem_wr_strb_b[fk])
			f_word[fk*8 +: 8] <= mem_wr_data_b[fk*8 +: 8];
	end

	always @(posedge i_clk)
	if (i_reset || (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last))
		f_txaddr <= 0;
	else if (r_tx_request)
		f_txaddr <= { r_fifo, {(LGFIFO32){1'b0}} };
	else if (o_tx_mem_valid && i_tx_mem_ready)
		f_txaddr <= f_txaddr + 1;

	always @(posedge i_clk)
	if (!i_reset && o_tx_mem_valid && f_txaddr == fc_addr)
		assert(o_tx_mem_data == f_word);

	always @(posedge i_clk)
	if (!i_reset && o_tx_mem_valid)
	begin
		assert(f_txaddr[LGFIFO32-1:0] < f_blocksz);
		assert(tx_mem_addr[LGFIFO32-1:0] < f_blocksz+2);
		assert(o_tx_mem_last == (f_txaddr[LGFIFO32-1:0] == f_blocksz-1));
	end

	// Need to relate f_txaddr to tx_mem_addr
	always @(*)
	if (!i_reset && o_tx_en && !r_tx_sent)
	begin
		assert(r_fifo == f_txaddr[LGFIFO32]);
		if (!o_tx_mem_valid || !o_tx_mem_last)
		begin
			assert(tx_mem_addr[LGFIFO32-1:0]==f_txaddr[LGFIFO32-1:0]
				+ (o_tx_mem_valid ? 1:0)
				+ (tx_pipe_valid ? 1:0));
		end
		if (o_tx_mem_valid)
		begin
			assert(tx_pipe_valid);
		end

		if (o_tx_mem_valid && tx_pipe_valid && r_fifo == fc_addr[LGFIFO32]
			&& f_txaddr[LGFIFO32-1:0]+1 == fc_addr[LGFIFO32-1:0])
		begin
			assert(next_tx_mem == f_word);
		end

		if (o_tx_mem_valid && tx_pipe_valid)
		begin
			assert(tx_fifo_last == (f_txaddr[LGFIFO32-1:0]+1 >= f_blocksz-1));
		end

		if (!tx_pipe_valid)
		begin
			assert(tx_mem_addr == 0);
		end else if (!o_tx_mem_valid)
			assert(tx_mem_addr == 1);
	end

	always @(*)
	if (!i_reset && o_tx_en && r_tx_sent)
		assert(!tx_pipe_valid && !o_tx_mem_valid);

	always @(*)
	if (f_past_valid && !dma_busy)
		assert(!dma_write);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Register checking
	// {{{

	reg	f_past_soft;
	always @(posedge i_clk)
		f_past_soft <= o_soft_reset || !o_hwreset_n;

	// PHY register
	// {{{
	fwb_register #(
		// {{{
		.AW(3), .DW(MW), .ADDR(ADDR_PHY),
		.MASK(32'h0018_b100
			| (OPT_1P8V   ? 32'h0040_0000 : 32'h00)
			| (OPT_SERDES ? 32'h001f_0000 : 32'h00)
			| (OPT_DDR    ? 32'h0018_0000 : 32'h00)),
		.FIXED_BIT_MASK(32'hf080_0000
			| (OPT_1P8V   ? 32'h00: 32'h0040_0000)
			| (OPT_SERDES ? 32'h00
			:  OPT_DDR    ? 32'h0023_0200
			:               32'h0024_0200))
		// }}}
	) fwb_phy (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || o_soft_reset || !o_hwreset_n),
		.i_wb_stb(i_wb_stb && !o_wb_stall), .i_wb_we(i_wb_we),
		.i_wb_addr(i_wb_addr), .i_wb_data(i_wb_data),
				.i_wb_sel(i_wb_sel),
		.i_wb_ack(pre_valid && !f_past_soft),
			.i_wb_return(pre_data),
		.i_register(w_phy_ctrl)
		// }}}
	);

	always @(*)
	if (!i_reset)
	begin
		assert(r_width != 2'b11);
		if (NUMIO < 4)
			assert(r_width == WIDTH_1W);
		else if (NUMIO < 8)
			assert(r_width == WIDTH_1W || r_width == WIDTH_4W);
	end

	always @(*)
	if (!i_reset && o_cfg_ds)
		assert(o_cfg_ddr);

	always @(*)
	if (!i_reset)
	begin
		assert(lgblk <= LGFIFO);
		assert(lgblk >= 2);
	end

	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Hardware reset checking
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge i_clk)
	if (!i_reset && !$past(i_reset) && $fell(o_hwreset_n))
		assert(o_soft_reset);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Careless assumptions
	// {{{

	always @(*)
	if (o_rx_en || o_tx_en)
		assume(lgblk < 15);	// Assume no overflow ... for now

	// Assume the user won't do something dumb, like write to the same FIFO
	// we're reading from, or either read or write from the same FIFO we
	// are writing to
	// {{{
	always @(*)
	if (o_tx_en || r_tx_request)
		assume(dma_busy || !i_wb_stb || !i_wb_we
				|| i_wb_addr != ADDR_FIFOA + r_fifo);

	always @(*)
	if (cmd_busy && o_cmd_type == R2_REPLY)
		assume(!i_wb_stb || i_wb_addr != ADDR_FIFOA + r_fifo);

	always @(*)
	if (o_rx_en || r_rx_request)
		assume(!i_wb_stb || i_wb_addr != ADDR_FIFOA + r_fifo);
	// }}}
	// }}}
`endif	// FORMAL
// }}}
endmodule

