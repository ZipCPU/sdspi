////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdwb.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Bus handler.  Accepts and responds to Wishbone bus requests.
//		Configures clock division, and IO speed and parameters.
//	Issues commands to the command handler, TX and RX handlers.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2023, Gisselquist Technology, LLC
// {{{
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of  the GNU General Public License as published
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
`default_nettype	none
// }}}
module	sdwb #(
		// {{{
		parameter	LGFIFO = 15,	// FIFO size in bytes
		parameter	NUMIO=4,
		parameter	MW = 32,
		// parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		parameter [0:0]	OPT_SERDES = 1'b0,
		parameter [0:0]	OPT_DDR = 1'b0,
		parameter [0:0]	OPT_CARD_DETECT = 1'b1,
		localparam	LGFIFOW=LGFIFO-$clog2(MW/8),
		parameter [0:0]	OPT_DMA = 1'b0
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		// Wishbone interface
		// {{{
		input	wire			i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[2:0]		i_wb_addr,
		input	wire	[MW-1:0]	i_wb_data,
		input	wire	[MW/8-1:0]	i_wb_sel,
		output	wire			o_wb_stall,
		output	reg			o_wb_ack,
		output	reg	[MW-1:0]	o_wb_data,
		// }}}
		// Configuration options
		// {{{
		output	reg			o_cfg_clk90,
		output	wire	[7:0]		o_cfg_ckspeed,
		output	reg			o_cfg_shutdown,
		output	wire	[1:0]		o_cfg_width,
		output	reg			o_cfg_ds, o_cfg_ddr,
		output	reg			o_pp_cmd, o_pp_data,
		output	reg	[4:0]		o_cfg_sample_shift,

		output	reg			o_soft_reset,
		// }}}
		// CMD interface
		// {{{
		output	reg			o_cmd_request,
		output	reg	[1:0]		o_cmd_type,
		output	wire	[5:0]		o_cmd_id,
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
		input	wire			i_rx_done, i_rx_err,
		// }}}
		input	wire			i_card_detect,
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

	localparam	[1:0]	CMD_PREFIX = 2'b01;
	localparam	[1:0]	R2_REPLY = 2'b10;
	localparam		CARD_REMOVED_BIT = 18,
				// ERR_BIT       = 15,
				FIFO_ID_BIT      = 12,
				USE_FIFO_BIT     = 11,
				FIFO_WRITE_BIT   = 10;

	localparam	[1:0]	WIDTH_1W = 2'b00,
				WIDTH_4W = 2'b01,
				WIDTH_8W = 2'b10;

	wire		wb_cmd_stb, wb_phy_stb;
	reg	[6:0]	r_cmd;
	reg		r_tx_request, r_rx_request;
	reg		r_fifo, r_cmd_err;
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

	reg			pre_ack;
	reg	[1:0]		pre_sel;
	reg	[31:0]		pre_data;

	reg	[LGFIFOW-1:0]	mem_wr_addr_a, mem_wr_addr_b;
	reg	[MW/8-1:0]	mem_wr_strb_a, mem_wr_strb_b;
	reg	[MW-1:0]	mem_wr_data_a, mem_wr_data_b;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Registers
	// {{{
	reg	cmd_busy;

	// CMD/control register
	// {{{
	assign	wb_cmd_stb = i_wb_stb && i_wb_addr == ADDR_CMD && i_wb_we;

	initial	o_soft_reset = 1'b1;
	always @(posedge i_clk)
	if (i_reset)
		o_soft_reset <= 1'b1;
	else
		o_soft_reset <= (wb_cmd_stb)&&(&i_wb_sel[3:0])
					&&(i_wb_data==32'h52_00_00_00);

	initial	o_cmd_request = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cmd_request <= 1'b0;
	else if (!cmd_busy && wb_cmd_stb && i_wb_sel[0])
	begin
		o_cmd_request <= (i_wb_data[7:6] == CMD_PREFIX);

		if ((o_tx_en || r_tx_request || o_rx_en || r_rx_request)
				&&((i_wb_sel[1] && i_wb_data[9:8] == R2_REPLY)
				||(!i_wb_sel[1]&&o_cmd_type == R2_REPLY)))
			o_cmd_request <= 1'b0;
	end else if (!i_cmd_busy)
		o_cmd_request <= 1'b0;

	initial	cmd_busy = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		cmd_busy <= 1'b0;
	else if (!cmd_busy && wb_cmd_stb && i_wb_sel[0])
	begin
		cmd_busy <= (i_wb_data[7:6] == CMD_PREFIX);
		if ((o_tx_en || r_tx_request || o_rx_en || r_rx_request)
				&&((i_wb_sel[1] && i_wb_data[9:8] == R2_REPLY)
				||(!i_wb_sel[1]&&o_cmd_type == R2_REPLY)))
			cmd_busy <= 1'b0;
	end else if (i_cmd_done)
		cmd_busy <= 1'b0;

	initial	r_cmd = 7'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_cmd <= 7'b0;
	else if (!cmd_busy && wb_cmd_stb && i_wb_sel[0])
	begin
		if (i_wb_data[7:6] == CMD_PREFIX)
			r_cmd <= i_wb_data[6:0];
	end else if (i_cmd_response)
		r_cmd <= { 1'b0, i_resp };

	assign	o_cmd_id = r_cmd[5:0];

	initial	o_cmd_type = 2'b00;
	always @(posedge i_clk)
	if (!cmd_busy && wb_cmd_stb && i_wb_sel[1])
		o_cmd_type <= i_wb_data[9:8];

	// o_tx_en, r_tx_request
	// {{{
	initial	r_tx_request = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_tx_request <= 1'b0;
	else if (!o_tx_en && !r_tx_request && !o_rx_en && !r_rx_request
		&& !cmd_busy && wb_cmd_stb && (&i_wb_sel[1:0]))
	begin
		r_tx_request <= i_wb_data[FIFO_WRITE_BIT]
			&&((i_wb_data[7:6]== CMD_PREFIX)||(i_wb_data[7:0] == 0))
			&& ((i_wb_data[13] && OPT_DMA) || i_wb_data[USE_FIFO_BIT]);
		if (i_wb_data[9:8] == R2_REPLY)
			r_tx_request <= 1'b0;
	end else if (!cmd_busy && !o_cmd_request && !o_tx_en)
		r_tx_request <= 1'b0;

	initial	o_tx_en = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_tx_en <= 1'b0;
	else if (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last)
		o_tx_en <= 1'b0;
	else if (!cmd_busy && !o_cmd_request && !o_tx_en && r_tx_request)
		o_tx_en <= r_tx_request;
	// }}}

	// o_rx_en, r_rx_request
	// {{{
	initial	r_rx_request = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_rx_request <= 1'b0;
	else if (!o_rx_en && !r_rx_request && !o_tx_en && !r_tx_request
		&& !cmd_busy && wb_cmd_stb && (&i_wb_sel[1:0]))
	begin
		r_rx_request <= !i_wb_data[FIFO_WRITE_BIT]
			&&((i_wb_data[7:6]== CMD_PREFIX)||(i_wb_data[7:0] == 0))
			&& ((i_wb_data[13] && OPT_DMA) || i_wb_data[USE_FIFO_BIT])
			&& i_wb_data[9:8] != R2_REPLY;
	end else if (!cmd_busy && !o_cmd_request)
		r_rx_request <= 1'b0;

	initial	o_rx_en = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_rx_en <= 1'b0;
	else if (i_rx_done)
		o_rx_en <= 1'b0;
	else if (!cmd_busy && !o_cmd_request && r_rx_request)
		o_rx_en <= 1'b1;
	// }}}

	initial	r_fifo = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_fifo <= 1'b0;
	else if (!cmd_busy && !o_tx_en && !o_rx_en
			&& !r_tx_request && !r_rx_request
			&& wb_cmd_stb && i_wb_sel[FIFO_ID_BIT/8])
		r_fifo <= i_wb_data[FIFO_ID_BIT];

	// always @(posedge i_clk)
	// if (i_reset || !OPT_DMA)
	//	r_dma <= 1'b0;
	// else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[1:0])
	//	r_dma <= i_wb_data[13];

	// r_cmd_err
	// {{{
	initial	r_cmd_err = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_cmd_err <= 1'b0;
	else if (!cmd_busy && wb_cmd_stb && i_wb_sel[0])
	begin
		if (i_wb_data[7:6] == CMD_PREFIX)
			r_cmd_err <= 1'b0;
	end else if (i_cmd_err)
		r_cmd_err <= 1'b0;

	initial	r_cmd_ecode = 2'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_cmd_ecode <= 2'b0;
	else if (!cmd_busy && wb_cmd_stb && i_wb_sel[0])
	begin
		if (i_wb_data[7:6] == CMD_PREFIX)
			r_cmd_ecode <= 2'b0;
	end else if (cmd_busy || i_cmd_err || i_cmd_done)
		r_cmd_ecode <= i_cmd_ercode;
	// }}}

	always @(*)
	begin
		w_cmd_word = 32'h0;
		w_cmd_word[19] = !card_present;
		w_cmd_word[18] =  card_removed;
		w_cmd_word[17:16] = r_cmd_ecode;
		w_cmd_word[15] = r_cmd_err;
		w_cmd_word[14] = cmd_busy;
		w_cmd_word[13] = 1'b0; // (== r_dma && OPT_DMA)
		w_cmd_word[12] = r_fifo;
		w_cmd_word[11] = (o_tx_en || r_tx_request
				|| o_rx_en || r_rx_request
			||(cmd_busy && o_cmd_type == 2'b10));
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
	else if (!cmd_busy && i_wb_stb && !o_wb_stall && i_wb_we && i_wb_addr == ADDR_ARG)
	begin
		if (i_wb_sel[0])
			r_arg[ 7: 0] <= i_wb_data[ 7: 0];
		if (i_wb_sel[1])
			r_arg[15: 8] <= i_wb_data[15: 8];
		if (i_wb_sel[2])
			r_arg[23:16] <= i_wb_data[23:16];
		if (i_wb_sel[3])
			r_arg[31:24] <= i_wb_data[31:24];
	end

	assign	o_arg = r_arg;
	// }}}

	// PHY control register
	// {{{
	assign	wb_phy_stb = i_wb_stb && !o_wb_stall && i_wb_addr == ADDR_PHY && i_wb_we;

	initial	lgblk = 4'h9;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		lgblk <= 4'h9;
	else if (!o_tx_en && !o_rx_en && !r_tx_request && !r_rx_request
			&& wb_phy_stb && i_wb_sel[3])
	begin
		lgblk <= i_wb_data[27:24];
		if (i_wb_data[27:24] >= LGFIFO)
			lgblk <= LGFIFO;
		else if (i_wb_data[27:24] <= 2)
			lgblk <= 2;
	end

	assign	o_length = 1<<lgblk;

	initial	o_cfg_sample_shift = 5'h18;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cfg_sample_shift <= 5'h18;
	else if (wb_phy_stb && i_wb_sel[2])
	begin
		o_cfg_sample_shift <= i_wb_data[20:16];

		if(!OPT_SERDES)
			o_cfg_sample_shift[1:0] <= 2'h0;
		if(!OPT_SERDES && !OPT_DDR)
			o_cfg_sample_shift[2] <= 1'h0;
	end

	initial	{ o_cfg_shutdown, o_cfg_clk90 } = 2'b00;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		{ o_cfg_shutdown, o_cfg_clk90 } <= 2'b00;
	else if (wb_phy_stb && i_wb_sel[1])
	begin
		{ o_cfg_shutdown, o_cfg_clk90 } <= i_wb_data[15:14];
		if (i_wb_data[8])
			o_cfg_clk90 <= 1'b1;
	end

	initial	{ o_pp_cmd, o_pp_data } = 2'b00;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		{ o_pp_cmd, o_pp_data } <= 2'b00;
	else if (wb_phy_stb && i_wb_sel[1])
		{ o_pp_cmd, o_pp_data } <= i_wb_data[13:12];

	initial	o_cfg_ds = 1'b0;
	always @(posedge i_clk)
	if (i_reset || !OPT_SERDES || o_soft_reset)
		o_cfg_ds <= 1'b0;
	else if (wb_phy_stb && i_wb_sel[1])
		o_cfg_ds <= (&i_wb_data[9:8]);

	initial	o_cfg_ddr = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_cfg_ddr <= 1'b0;
	else if (wb_phy_stb && i_wb_sel[1])
		o_cfg_ddr <= i_wb_data[8];

	initial	r_width = WIDTH_1W;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_width <= WIDTH_1W;
	else if (wb_phy_stb && i_wb_sel[1])
	begin
		case(i_wb_data[11:10])
		2'b00: r_width <= WIDTH_1W;
		2'b01: if (NUMIO < 4) r_width <= WIDTH_1W;
			else r_width <= WIDTH_4W;
		2'b10: if (NUMIO >= 8) r_width <= WIDTH_8W;
		default: begin end
		endcase
	end

	assign	o_cfg_width = r_width;

	initial	r_ckspeed = 252;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		r_ckspeed <= 252;
	else if (wb_phy_stb && i_wb_sel[0])
	begin
		r_ckspeed <= i_wb_data[7:0];
		if (!OPT_SERDES && i_wb_data[7:0] == 0)
			r_ckspeed <= (OPT_DDR) ? 8'h1 : 8'h2;
		if (!OPT_SERDES && !OPT_DDR && i_wb_data[7:0] == 1)
			r_ckspeed <= 8'h2;
	end

	assign	o_cfg_ckspeed = r_ckspeed;

	always @(*)
	begin
		w_phy_ctrl = 0;
		w_phy_ctrl[31:28] = LGFIFO;
		w_phy_ctrl[27:24] = lgblk;
		w_phy_ctrl[23:22] = (NUMIO < 4) ? WIDTH_1W
					: (NUMIO < 8) ? WIDTH_4W
					: WIDTH_8W;
		w_phy_ctrl[21]    = OPT_SERDES;
		w_phy_ctrl[20:16] = o_cfg_sample_shift;
		w_phy_ctrl[15] = o_cfg_shutdown;
		w_phy_ctrl[14] = o_cfg_clk90;
		w_phy_ctrl[13] = o_pp_cmd;	// Push-pull CMD line
		w_phy_ctrl[12] = o_pp_data;	// Push-pull DAT line(s)
		w_phy_ctrl[11:10] = r_width;
		w_phy_ctrl[9:8] = { o_cfg_ds, o_cfg_ddr };
		w_phy_ctrl[7:0] = r_ckspeed;
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
		reg	[9:0]	card_detect_counter;
		reg		r_card_removed, r_card_present;

		initial	r_card_removed = 1'b1;
		always @(posedge i_clk)
		if (i_reset || o_soft_reset)
			r_card_removed <= 1'b1;
		else if (!card_present)
			r_card_removed <= 1'b1;
		else if (wb_cmd_stb && i_wb_data[CARD_REMOVED_BIT]
					&& i_wb_sel[CARD_REMOVED_BIT/8])
			r_card_removed <= 1'b0;

		initial	raw_card_present = 0;
		always @(posedge i_clk)
		if (i_reset)
			raw_card_present <= 0;
		else
			raw_card_present <= { raw_card_present[1:0], i_card_detect };

		initial	card_detect_counter = 0;
		always @(posedge i_clk)
		if (i_reset || !raw_card_present[2] || o_soft_reset)
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
		assign	card_removed = r_card_removed;

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
	if (i_reset || o_soft_reset)
		o_int <= 1'b0;
	else begin
		o_int <= 1'b0;
		if (i_cmd_done && cmd_busy)
			o_int <= 1'b1;
		if (o_tx_en && o_tx_mem_valid && o_tx_mem_last)
			o_int <= 1'b1;
		if (o_rx_en && i_rx_done)
			o_int <= 1'b1;
	end

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
	else if (wb_cmd_stb && i_wb_sel[1] && i_wb_data[11])
		fif_wraddr <= 0;
	else if (i_wb_stb && !o_wb_stall && i_wb_we && i_wb_sel[0]
					&& (i_wb_addr == ADDR_FIFOA || i_wb_addr == ADDR_FIFOB))
		fif_wraddr <= fif_wraddr + 1;
	// }}}

	// User read pointer
	// {{{
	initial	fif_rdaddr = 0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		fif_rdaddr <= 0;
	else if (wb_cmd_stb && i_wb_sel[1] && i_wb_data[11])
		fif_rdaddr <= 0;
	else if (i_wb_stb && !i_wb_we && i_wb_sel[0]
					&& (i_wb_addr == ADDR_FIFOA || i_wb_addr == ADDR_FIFOB))
		fif_rdaddr <= fif_rdaddr + 1;
	// }}}

	// TX

	// o_tx_mem_valid
	// {{{
	initial	o_tx_mem_valid = 1'b0;
	always @(posedge i_clk)
	if (i_reset || o_soft_reset)
		o_tx_mem_valid <= 0;
	else if (!o_tx_en ||(o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last))
		{ o_tx_mem_valid, tx_pipe_valid } <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid)
		{ o_tx_mem_valid, tx_pipe_valid } <= { tx_pipe_valid, 1'b1 };
	// }}}

	// tx_mem_addr
	// {{{
	always @(posedge i_clk)
	if (i_reset || !o_tx_en || o_soft_reset)
		tx_mem_addr <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid)
		tx_mem_addr <= tx_mem_addr + 1;
	// }}}

	always @(*)
		fif_a_rdaddr = (o_tx_en && !r_fifo) ? tx_mem_addr[LGFIFO32-1:$clog2(MW/32)] : fif_rdaddr;

	always @(*)
		fif_b_rdaddr = (o_tx_en &&  r_fifo) ? tx_mem_addr[LGFIFO32-1:$clog2(MW/32)] : fif_rdaddr;

	always @(*)
		blk_words = (1<<(lgblk-2))-1;
	always @(*)
		pre_tx_last = o_tx_en && (tx_mem_addr[LGFIFO32-1:$clog2(MW/32)] >= blk_words);

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready || r_fifo)
		tx_fifo_a <= fifo_a[fif_a_rdaddr];

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready || !r_fifo)
		tx_fifo_b <= fifo_b[fif_b_rdaddr];

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready)
		tx_fifo_last <= pre_tx_last;

	// o_tx_mem_data
	// {{{
	generate if (MW <= 32)
	begin : NO_TX_SHIFT
		assign	tx_shift = 0;
	end else begin : GEN_TX_SHIFT
		reg	[$clog2(MW/32)-1:0]	r_tx_shift;

		always @(posedge i_clk)
		if (!o_tx_mem_valid || i_tx_mem_ready)
			r_tx_shift <= tx_mem_addr[$clog2(MW/32)-1:0];

		assign	tx_shift = r_tx_shift;
	end endgenerate

	always @(*)
	begin
		next_tx_mem = (r_fifo) ? tx_fifo_b : tx_fifo_a;
		next_tx_mem = next_tx_mem >> (32*tx_shift);
	end

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready)
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
	// same time.  Hence:
	//
	//	- When reading from SD/eMMC, do not write to the FIFO
	//
	initial	mem_wr_strb_a = 0;
	always @(posedge i_clk)
	begin
		mem_wr_strb_a <= 0;

		if (i_wb_stb && i_wb_we && i_wb_addr == ADDR_FIFOA
				&& (|i_wb_sel))
		begin
			mem_wr_addr_a <= fif_wraddr;
			mem_wr_strb_a <= i_wb_sel;
			mem_wr_data_a <= i_wb_data;
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

	initial	mem_wr_strb_b = 0;
	always @(posedge i_clk)
	begin
		mem_wr_strb_b <= 0;

		if (i_wb_stb && i_wb_we && i_wb_addr == ADDR_FIFOB
				&& (|i_wb_sel))
		begin
			mem_wr_addr_b <= fif_wraddr;
			mem_wr_strb_b <= i_wb_sel;
			mem_wr_data_b <= i_wb_data;
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
	// Wishbone Return handling
	// {{{

	always @(posedge i_clk)
	begin
		pre_data <= 0;

		case(i_wb_addr)
		ADDR_CMD: pre_data[31:0] <= w_cmd_word;
		ADDR_ARG: pre_data[31:0] <= r_arg;
		ADDR_PHY: pre_data[31:0] <= w_phy_ctrl;
		// 3'h3: pre_data <= w_ffta_word;
		// 3'h4: pre_data <= w_fftb_word;
		// 3'h5: DMA address (high, if !OPT_LITTLE_ENDIAN)
		// 3'h6: DMA address (low, if !OPT_LITTLE_ENDIAN)
		// 3'h7: DMA transfer length
		default: begin end
		endcase

		if (!i_wb_stb || i_wb_we)
			pre_data <= 0;

		pre_sel <= 0;
		if (i_wb_stb && !i_wb_we)
		begin
			if (i_wb_addr == ADDR_FIFOA)
				pre_sel <= 1;
			else if (i_wb_addr == ADDR_FIFOB)
				pre_sel <= 2;
		end
	end

	always @(posedge i_clk)
	begin
		o_wb_data <= 0;
		case(pre_sel)
		2'h0: o_wb_data[31:0] <= pre_data;
		2'h1: o_wb_data <= tx_fifo_a;
		2'h2: o_wb_data <= tx_fifo_b;
		default: begin end
		endcase
	end

	assign	o_wb_stall = 1'b0;

	initial	{ o_wb_ack, pre_ack } = 2'b00;
	always @(posedge i_clk)
	if (i_reset || !i_wb_cyc)
		{ o_wb_ack, pre_ack } <= 2'b00;
	else
		{ o_wb_ack, pre_ack } <= { pre_ack, i_wb_stb && !o_wb_stall };
	// }}}

	// Keep Verilator happy
	// {{{
	wire	unused;
	assign	unused = &{ 1'b0, i_rx_err, next_tx_mem };
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
		.AW(3), .DW(MW),
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
		assert(fwb_outstanding == (pre_ack ? 1:0) + (o_wb_ack ? 1:0));
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
	if (f_past_valid && $past(i_reset || i_cmd_done))
		assert(!cmd_busy);

	always @(posedge i_clk)
	if (f_past_valid && cmd_busy && !o_cmd_request)
		assume(i_cmd_busy);

	always @(posedge i_clk)
	if (f_past_valid && (!cmd_busy || o_cmd_request))
		assume(!i_cmd_done && !i_cmd_err && !i_cmd_mem_valid
					&& !i_cmd_response);

	always @(*)
	if (!cmd_busy || o_cmd_request || o_cmd_type != R2_REPLY)
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
		assert($stable(o_cmd_type));
		assert($stable(o_cmd_id));
		assert($stable(o_arg));
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
			assert(!cmd_busy || o_cmd_type != R2_REPLY);
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// TX Handling
	// {{{
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

	always @(*)
	if (!o_rx_en)
		assume(!i_rx_mem_valid);

	always @(*)
	if (!i_reset)
		assert(!r_rx_request || !o_rx_en);

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
		assert(f_txaddr[LGFIFO32-1:0] < (1<<(lgblk-2)));
		assert(tx_mem_addr[LGFIFO32-1:0] < (1<<(lgblk-2))+2);
		assert(o_tx_mem_last == (f_txaddr[LGFIFO32-1:0] == (1<<(lgblk-2))-1));
	end

	// Need to relate f_txaddr to tx_mem_addr
	always @(*)
	if (!i_reset && o_tx_en)
	begin
		assert(r_fifo == f_txaddr[LGFIFO32]);
		assert(tx_mem_addr[LGFIFO32-1:0] == f_txaddr[LGFIFO32-1:0]
			+ (o_tx_mem_valid ? 1:0) + (tx_pipe_valid ? 1:0));
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
			assert(tx_fifo_last == (f_txaddr[LGFIFO32-1:0]+1 >= (1<<(lgblk-2))-1));
		end

		if (!tx_pipe_valid)
		begin
			assert(tx_mem_addr == 0);
		end else if (!o_tx_mem_valid)
			assert(tx_mem_addr == 1);
	end


	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Register checking
	// {{{

	reg	f_past_soft;
	always @(posedge i_clk)
		f_past_soft <= o_soft_reset;

	// PHY register
	// {{{
	fwb_register #(
		.AW(3), .DW(MW), .ADDR(ADDR_PHY),
		.MASK(32'h0018_b1fc
			| (OPT_SERDES ? 32'h001f_02ff : 32'h00)
			| (OPT_DDR    ? 32'h001c_0000 : 32'h00)),
		.FIXED_BIT_MASK(32'hf0e0_0000
			| (OPT_SERDES ? 32'h00: 32'h0003_0200)
			| (OPT_DDR    ? 32'h00: 32'h0004_0000))
	) fwb_phy (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || o_soft_reset),
		.i_wb_stb(i_wb_stb), .i_wb_we(i_wb_we),
		.i_wb_addr(i_wb_addr), .i_wb_data(i_wb_data),
				.i_wb_sel(i_wb_sel),
		.i_wb_ack(pre_ack && !f_past_soft),
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
		assume(!i_wb_stb || !i_wb_we
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

