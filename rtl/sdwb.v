////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdwb.v
// {{{
// Project:	SDIO SD-Card controller, using a shared SPI interface
//
// Purpose:	
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
		output	reg	[7:0]		o_cfg_ckspeed,
		output	reg			o_cfg_shutdown,
		output	reg	[1:0]		o_cfg_width,
		output	reg			o_cfg_ds, o_cfg_ddr,
		output	reg			o_pp_cmd, o_pp_data,
		output	reg	[4:0]		o_cfg_sample_shift,
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
		output	wire			o_tx_mem_valid,
		input	wire			i_tx_mem_ready,
		output	reg	[31:0]		o_tx_mem_data,
		output	wire			o_tx_mem_last,
		// }}}
		// RX interface
		// {{{
		output	wire			o_rx_en,
		output	wire			o_crc_en,
		output	wire	[LGFIFO:0]	o_length,
		//
		input	wire			i_rx_mem_valid,
		input	wire	[MW/8-1:0]	i_rx_mem_strb,
		input	wire	[LGFIFOW-1:0]	i_rx_mem_addr,
		input	wire	[MW-1:0]	i_rx_mem_data,
		//
		input	wire			i_rx_done, i_rx_err
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[1:0]	WIDTH_1W = 2'b00,
				WIDTH_4W = 2'b01,
				WIDTH_8W = 2'b10;

	reg	[6:0]	r_cmd;
	reg		r_tx_request, r_rx_request;
	reg		r_fifo, r_cmd_err;
	reg	[1:0]	r_cmd_ecode;
	reg	[31:0]	r_arg;
	reg	[3:0]	lgblk;
	reg		r_clk_shutdown, r_clk90;
	reg	[1:0]	r_width;
	reg	[7:0]	r_ckspeed;
	reg	[31:0]	w_cmd_word, w_phy_ctrl;

	integer	ika, ikb;
	localparam	NFIFOW = (1<<LGFIFO) / (MW/8);
	reg	[MW-1:0]	fifo_a	[0:NFIFOW-1];
	reg	[MW-1:0]	fifo_b	[0:NFIFOW-1];
	reg	[MW-1:0]	tx_fifo_a, tx_fifo_b;
	wire	[(($clog2(MW/32) > 0) ? ($clog2(MW/32)-1):0):0] tx_shift;
	reg	[LGFIFO-$clog2(MW/8)-1:0]	fif_wraddr, fif_rdaddr;
	reg	[LGFIFO-$clog2(MW/8)-1:0]	fif_a_rdaddr, fif_b_rdaddr;
	reg	[LGFIFO-$clog2(MW/32)-1:0]	tx_mem_addr;
	reg	[MW-1:0]	next_tx_mem;
	reg			tx_fifo_last, pre_tx_last,
				tx_pipe_valid;

	reg			pre_ack;
	reg	[1:0]		pre_sel;
	reg	[31:0]		pre_data;

	reg	[LGFIFO-$clog2(MW/8):0]		mem_wr_addr;
	reg	[MW/8-1:0]			mem_wr_strb;
	reg	[MW-1:0]			mem_wr_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Registers
	// {{{


	// CMD/control register
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		o_cmd_request <= 1'b0;
	else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[0])
		o_cmd_request <= i_wb_data[7:6] == 2'b01;
	else
		o_cmd_request <= 1'b0;

	always @(posedge i_clk)
	if (i_reset)
		r_cmd <= 7'b0;
	else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[0])
	begin
		if (i_wb_data[7:6] == 2'b01)
			r_cmd <= i_wb_data[6:0];
	end else if (i_cmd_response)
		r_cmd <= { 1'b0, i_resp };

	assign	o_cmd_id = r_cmd[5:0];

	always @(posedge i_clk)
	if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[1])
		o_cmd_type <= i_wb_data[9:8];

	always @(posedge i_clk)
	if (i_reset)
		r_tx_request <= 1'b0;
	else if (!o_tx_en && !i_cmd_busy && i_wb_stb && i_wb_addr == 0
							&& (&i_wb_sel[1:0]))
		r_tx_request <= i_wb_data[10] && (i_wb_data[7:6] == 2'b01)
			&& ((i_wb_data[13] && OPT_DMA) || i_wb_data[11]);
	else if (o_tx_en)
		r_tx_request <= 1'b0;

	always @(posedge i_clk)
	if (i_reset)
		o_tx_en <= 1'b0;
	else if (o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last)
		o_tx_en <= 1'b0;
	else if (!i_cmd_busy && !o_cmd_request)
		o_tx_en <= r_tx_request;

	always @(posedge i_clk)
	if (i_reset)
		r_rx_request <= 1'b0;
	else if (!o_rx_en && !i_cmd_busy && i_wb_stb && i_wb_addr == 0
							&& (&i_wb_sel[1:0]))
	begin
		r_rx_request <= !i_wb_data[10]
			&& (i_wb_data[7:6] == 2'b01)
			&& ((i_wb_data[13] && OPT_DMA) || i_wb_data[11])
			&& i_wb_data[9:8] != 2'b10;
	end else if (o_rx_en)
		r_rx_request <= 1'b0;

	always @(posedge i_clk)
	if (i_reset)
		o_rx_en <= 1'b0;
	else if (i_rx_done)
		o_rx_en <= 1'b0;
	else if (!i_cmd_busy && !o_cmd_request)
		o_rx_en <= r_rx_request;


	always @(posedge i_clk)
	if (i_reset)
		r_fifo <= 1'b0;
	else if (!i_cmd_busy && !o_tx_en && !o_rx_en
			&& i_wb_stb && (i_wb_addr == 0) && i_wb_sel[1])
		r_fifo <= i_wb_data[12];

	// always @(posedge i_clk)
	// if (i_reset || !OPT_DMA)
	//	r_dma <= 1'b0;
	// else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[1:0])
	//	r_dma <= i_wb_data[13];

	always @(posedge i_clk)
	if (i_reset)
		r_cmd_err <= 1'b0;
	else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[0])
	begin
		if (i_wb_data[7:6] == 2'b01)
			r_cmd_err <= 1'b0;
	end else if (i_cmd_err)
		r_cmd_err <= 1'b0;

	always @(posedge i_clk)
	if (i_reset)
		r_cmd_ecode <= 2'b0;
	else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 0 && i_wb_sel[0])
	begin
		if (i_wb_data[7:6] == 2'b01)
			r_cmd_ecode <= 2'b0;
	end else if (i_cmd_busy || i_cmd_err || i_cmd_done)
		r_cmd_ecode <= i_cmd_ercode;

	always @(*)
	begin
		w_cmd_word = 32'h0;
		w_cmd_word[17:16] = r_cmd_ecode;
		w_cmd_word[15] = r_cmd_err;
		w_cmd_word[14] = i_cmd_busy;
		w_cmd_word[13] = 1'b0; // (== r_dma && OPT_DMA)
		w_cmd_word[12] = r_fifo;
		w_cmd_word[11] = (o_tx_en || r_tx_request
				|| o_rx_en || r_rx_request
			||(i_cmd_busy && o_cmd_type == 2'b10));
		w_cmd_word[10] = (o_tx_en || r_tx_request);
		w_cmd_word[9:8] = o_cmd_type;
		w_cmd_word[6:0] = r_cmd;
	end
	// }}}

	// Command argument register
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		r_arg <= 0;
	// else if (o_cmd_request && !i_cmd_busy)
	//	r_arg <= 0;
	else if (i_cmd_response)
		r_arg <= i_arg;
	else if (!i_cmd_busy && i_wb_stb && i_wb_addr == 1)
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
	always @(posedge i_clk)
	if (i_reset)
		lgblk <= 4'h9;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[3])
	begin
		lgblk <= i_wb_data[27:24];
		if (i_wb_data[27:24] >= LGFIFO)
			lgblk <= LGFIFO;
		else if (i_wb_data[27:24] <= 2)
			lgblk <= 2;
	end

	assign	o_length = 1<<lgblk;

	always @(posedge i_clk)
	if (i_reset)
		o_cfg_sample_shift <= 5'h18;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[2])
	begin
		o_cfg_sample_shift <= i_wb_data[20:16];

		if(!OPT_SERDES)
			o_cfg_sample_shift[2:0] <= 3'h0;
	end

	always @(posedge i_clk)
	if (i_reset)
		{ r_clk_shutdown, r_clk90 } <= 2'b00;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[1])
	begin
		{ r_clk_shutdown, r_clk90 } <= i_wb_data[15:14];
		if (i_wb_data[8])
			r_clk90 <= 1'b1;
	end

	assign	o_cfg_clk90 = r_clk90;
	assign	o_cfg_shutdown = r_clk_shutdown;

	always @(posedge i_clk)
	if (i_reset)
		{ o_pp_cmd, o_pp_data } <= 2'b00;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[1])
		{ o_pp_cmd, o_pp_data } <= i_wb_data[13:12];

	always @(posedge i_clk)
	if (i_reset || !OPT_SERDES)
		{ o_cfg_ds, o_cfg_ddr } <= 2'b00;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[1])
		{ o_cfg_ds, o_cfg_ddr } <= i_wb_data[9:8];

	always @(posedge i_clk)
	if (i_reset)
		r_width <= WIDTH_1W;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[1])
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

	always @(posedge i_clk)
	if (i_reset)
		r_ckspeed <= 252;
	else if (i_wb_stb && !o_wb_stall && i_wb_addr == 2 && i_wb_sel[0])
	begin
		r_ckspeed <= i_wb_data[7:0];
		if (!OPT_SERDES && i_wb_data[7:0] <= 2)
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
		w_phy_ctrl[15] = r_clk_shutdown;
		w_phy_ctrl[14] = r_clk90;
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
	// FIFO / Memory handling
	// {{{

	// User write pointer
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		fif_wraddr <= 0;
	else if (i_wb_stb &&  i_wb_we && i_wb_addr == 0 && i_wb_sel[1] && i_wb_data[11])
		fif_wraddr <= 0;
	else if (i_wb_stb && i_wb_we && i_wb_sel[0]
					&& (i_wb_addr == 3 || i_wb_addr == 4))
		fif_wraddr <= fif_wraddr + 1;
	// }}}

	// User read pointer
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		fif_rdaddr <= 0;
	else if (i_wb_stb &&  i_wb_we && i_wb_addr == 0 && i_wb_sel[1] && i_wb_data[11])
		fif_rdaddr <= 0;
	else if (i_wb_stb && !i_wb_we && i_wb_sel[0]
					&& (i_wb_addr == 3 || i_wb_addr == 4))
		fif_rdaddr <= fif_rdaddr + 1;
	// }}}

	// TX

	// o_tx_mem_valid
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		o_tx_mem_valid <= 0;
	else if (!o_tx_en ||(o_tx_mem_valid && i_tx_mem_ready && o_tx_mem_last))
		{ o_tx_mem_valid, tx_pipe_valid } <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid)
		{ o_tx_mem_valid, tx_pipe_valid } <= { tx_pipe_valid, 1'b1 };
	// }}}

	// tx_mem_addr
	// {{{
	always @(posedge i_clk)
	if (i_reset || !o_tx_en)
		tx_mem_addr <= 0;
	else if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid)
		tx_mem_addr <= tx_mem_addr + 1;
	// }}}

	always @(*)
		fif_a_rdaddr = (o_tx_en && !r_fifo) ? tx_mem_addr[LGFIFO-$clog2(MW/8)-1:$clog2(MW/32)] : fif_rdaddr;

	always @(*)
		fif_b_rdaddr = (o_tx_en &&  r_fifo) ? tx_mem_addr[LGFIFO-$clog2(MW/8)-1:$clog2(MW/32)] : fif_rdaddr;

	always @(*)
		pre_tx_last = o_tx_en && (tx_mem_addr[LGFIFO-$clog2(MW/8)-1:$clog2(MW/32)] == (1<<lgblk)-1);

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid || !r_fifo)
		tx_fifo_a <= fifo_a[fif_a_rdaddr];

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid || r_fifo)
		tx_fifo_b <= fifo_b[fif_b_rdaddr];

	always @(posedge i_clk)
	if (!o_tx_mem_valid || i_tx_mem_ready || !tx_pipe_valid || r_fifo)
		tx_fifo_last <= pre_tx_last;

	// o_tx_mem_data
	// {{{
	generate if (MW <= 32)
	begin : NO_TX_SHIFT
		assign	tx_shift = 0;
	end else begin : GEN_TX_SHIFT
		reg	[$clog2(MW/32)-1:0]	r_tx_shift;

		always @(posedge i_clk)
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
	always @(posedge i_clk)
	begin
		mem_wr_strb <= 0;

		if (i_wb_stb && i_wb_we && (i_wb_addr == 3 || i_wb_addr == 4)
				&& (|i_wb_sel))
		begin
			mem_wr_addr <= { (i_wb_addr == 3), fif_wraddr };
			mem_wr_strb <= i_wb_sel;
			mem_wr_data <= i_wb_data;
		end

		if (i_cmd_mem_valid)
		begin
			mem_wr_addr <= { r_fifo, i_cmd_mem_addr };
			mem_wr_strb <= i_cmd_mem_strb;
			mem_wr_data <= i_cmd_mem_data;
		end

		if (i_rx_mem_valid)
		begin
			mem_wr_addr <= { r_fifo, i_rx_mem_addr };
			mem_wr_strb <= i_rx_mem_strb;
			mem_wr_data <= i_rx_mem_data;
		end
	end
	// }}}

	// Actually write to the memories
	// {{{
	always @(posedge i_clk)
	for(ika=0; ika<MW/8; ika=ika+1)
	if (mem_wr_strb[ika] && !mem_wr_addr[LGFIFO-$clog2(MW/8)])
		fifo_a[mem_wr_addr[LGFIFO-$clog2(MW/8)-1:0]][ika*8 +: 8] <= mem_wr_data[ika*8 +: 8];


	always @(posedge i_clk)
	for(ikb=0; ikb<MW/8; ikb=ikb+1)
	if (mem_wr_strb[ikb] && mem_wr_addr[LGFIFO-$clog2(MW/8)])
		fifo_b[mem_wr_addr[LGFIFO-$clog2(MW/8)-1:0]][ikb*8 +: 8] <= mem_wr_data[ikb*8 +: 8];
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
		3'h0: pre_data[31:0] <= w_cmd_word;
		3'h1: pre_data[31:0] <= r_arg;
		3'h2: pre_data[31:0] <= w_phy_ctrl;
		// 3'h3: pre_data <= w_ffta_word;
		// 3'h4: pre_data <= w_fftb_word;
		// 3'h5: DMA address (high, if !OPT_LITTLE_ENDIAN)
		// 3'h6: DMA address (low, if !OPT_LITTLE_ENDIAN)
		// 3'h7: DMA transfer length
		default: begin end
		endcase

		if (!i_wb_stb || i_wb_we)
			pre_data <= 0;

		if (i_wb_addr == 3'h3)
			pre_sel <= 1;
		else if (i_wb_addr == 3'h4)
			pre_sel <= 2;
		if (!i_wb_stb || i_wb_we)
			pre_sel <= 0;

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
`endif	// FORMAL
// }}}
endmodule

