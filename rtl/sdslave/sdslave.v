////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdslave.v
// {{{
// Project:	SD-Card controller
//
// Purpose:
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
// }}}
module	sdslave #(
		// {{{
		parameter [0:0]	OPT_DDR = 1'b1,
		parameter	NUMIO=4,
		parameter	AW = 30, DW=32
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Wishbone interface
		// {{{
		output	wire		o_wb_cyc, o_wb_stb, o_wb_we,
		output	wire [AW-1:0]	o_wb_addr,
		output	wire [DW-1:0]	o_wb_data,
		output	wire [DW/8-1:0]	o_wb_sel,
		input	wire		i_wb_stall,
		input	wire		i_wb_ack,
		input	wire [DW-1:0]	i_wb_data,
		input	wire		i_wb_err,
		// }}}
		// SD slave front-end interface
		// {{{
		input	wire		i_sd_clk,
		//
		input	wire		i_sd_cmd,
		output	wire		o_sd_cmd,
		output	wire		o_sd_cmd_tristate,
		//
		input	wire	[15:0]	i_sd_dat,
		output	wire	[15:0]	o_sd_dat,
		output	wire	[7:0]	o_sd_dat_tristate
		output	wire	[1:0]	o_sd_ds,
		output	wire		o_sd_ds_tristate
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	wire		cfg_ds, cfg_ddr, cfg_cmd_pp, cfg_dat_pp;
	wire	[1:0]	cfg_width;
	wire	[3:0]	cfg_lgblksz;

	wire		w_cmd_en;
	wire		cmd_valid, cmd_err;
	wire	[5:0]	cmd_cmd;
	wire	[31:0]	cmd_arg;

	wire		resp_valid, resp_typ, resp_nocrc;
	wire	[5:0]	resp_cmd;
	wire	[31:0]	resp_arg;
	wire	[95:0]	resp_extended;

	wire		tx_en, tx_busy;
	wire		tx_valid, tx_ready, tx_last;
	wire	[31:0]	tx_data;

	wire		rx_valid, rx_good, rx_fail;
	wire	[31:0]	rx_data;
	// }}}

	assign	sd_reset= i_reset;

	////////////////////////////////////////////////////////////////////////
	//
	// SDS FSM
	// {{{
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDS Command wire control
	// {{{
	sdscmd
	u_cmd (
		// {{{
		.i_clk(i_sd_clk),
		.i_reset(sd_reset),
		.i_cfg_pp(cfg_cmd_pp),
		//
		.o_valid(cmd_valid),
		.o_err(cmd_err),
		.o_cmd(cmd_cmd),
		.o_arg(cmd_arg),
		//
		.i_valid(resp_valid),
		.i_resp(resp_cmd),
		.i_arg(resp_arg),
		.i_typ(resp_typ),
		.i_nocrc(resp_nocrc),
		.i_extended(resp_extended),
		//
		.i_cmdio(i_sd_cmd),
		.o_cmdio(o_sd_cmd),
		.o_cmden(w_cmden),
		.i_collision(1'b0)
		// }}}
	);

	assign	o_sd_cmd_tristate = !w_cmden;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDS TX Frame
	// {{{

	sdstxframe #(
		.OPT_DDR(OPT_DDR), .NUMIO(NUMIO)
	) u_txframe (
		// {{{
		.i_clk(i_sd_clk),
		.i_reset(sd_reset),
		//
		.i_cfg_en(tx_en),
		.i_cfg_pp(cfg_dat_pp),
		.i_cfg_ds(cfg_ds),
		.i_cfg_ddr(cfg_ddr),
		.i_cfg_width(cfg_width),
		//
		.i_busy(tx_busy),
		.i_rxgood(rx_good), .i_rxfail(rx_fail),
		//
		.i_valid(tx_valid),
		.o_ready(tx_ready),
		.i_data(tx_data),
		.i_last(tx_last),
		//
		.o_data(o_sd_dat),
		.o_tristate(o_sd_dat_tristate),
		.o_ds(o_sd_ds),
		//
		.o_done(tx_done)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDS RX Frame
	// {{{

	sdsrxframe #(
		.OPT_DDR(OPT_DDR), .NUMIO(NUMIO)
	) u_rxframe (
		// {{{
		.i_clk(i_sd_clk),
		.i_reset(sd_reset),
		//
		.i_cfg_en(rx_en),
		.i_cfg_lgblksz(cfg_lgblksz),
		.i_cfg_ddr(cfg_ddr),
		.i_cfg_width(cfg_width),
		//
		.i_valid(1'b1), .i_data(i_sd_dat),
		//
		.o_valid(rx_valid), // .i_ready(rx_ready),
			.o_data(rx_data),
			.o_good(rx_good), .o_err(rx_fail)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Alternative data sources
	// {{{
	reg		lcl_valid, lcl_last;
	reg	[3:0]	lcl_addr;
	reg	[31:0]	lcl_ram	[0:31];

	always @(posedge i_sd_clk)
	if (!i_tx_en || tx_src == 2'b00)
		lcl_valid <= 0;
	else if (!lcl_valid || !tx_valid || tx_ready))
		lcl_valid <= !lcl_last;

	always @(posedge i_sd_clk)
	if (!i_tx_en || tx_src == 2'b00)
		lcl_addr <= 0;
	else if (!lcl_last && (!lcl_valid || !tx_valid || tx_ready))
		lcl_addr <= lcl_addr + 1;

	always @(posedge i_sd_clk)
	if (!lcl_valid || !tx_valid || tx_ready))
		lcl_data <= lcl_ram[{ tx_src[1], lcl_addr }];

	always @(posedge i_sd_clk)
	if (!i_tx_en || tx_src == 2'b00)
		lcl_last <= 0;
	else if (!lcl_last && (!lcl_valid || !tx_valid || tx_ready))
	begin
		// S_SCR   =2'b01,  8Bytes
		// S_STATUS=2'b10, 64Bytes
		if (tx_src[0])
			lcl_last <= &lcl_addr[3:0];
		else // if (tx_src[0])
			lcl_last <=  lcl_addr[0];
	end

	assign	tx_valid = lcl_valid || (mem_valid && tx_src == 2'b00);
	assign	mem_ready = (!tx_valid || tx_ready);
	assign	tx_data  = (tx_src == 2'b00) ? mem_data : lcl_data;
	assign	tx_last  = (tx_src == 2'b00) ? mem_last : lcl_last;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDS DMA Handler
	// {{{
	sdsdma #(
		.BUS_WIDTH(DW), .ADDRESS_WIDTH(AW+$clog2(DW/8))
	) u_dma (
		// {{{
		.i_wb_clk(i_clk), .i_wb_reset(i_reset),
		//
		.i_sd_clk(i_sd_clk), .i_sd_reset(sd_reset),
		//
		.i_sd_softreset(dma_reset),	// Follow cfg interface timing
		// DMA Configuration interface
		// {{{
		.i_cfg_valid(dma_cfg_valid),
		.o_cfg_ready(dma_cfg_ready),
		.i_sd_request(dma_request),
		.i_sd_dir(dma_dir),
		.i_sd_abort(dma_abort),
		.i_sd_lglen(cfg_lgblksz),
		.i_sd_addr(dma_addr),

		.o_sd_busy(ign_dma_busy),
		.o_sd_err(dma_err),
		// }}}
		// SD stream interface
		// {{{
		.i_rx_valid(rx_valid), // .o_rx_ready(rx_ready),
			.i_rx_data(rx_data), .i_rx_last(rx_last),
		//
		.o_tx_valid(mem_valid), .i_tx_ready(mem_ready),
			.o_tx_data(mem_data), .o_tx_last(mem_last),
		// }}}
		// Wishbone interface
		// {{{
		.o_dma_cyc(o_wb_cyc), .o_dma_stb(o_wb_stb),
			.o_dma_we(o_wb_we),
		.o_dma_addr(o_wb_addr),
		.o_dma_data(o_wb_data), .o_dma_sel(o_wb_sel),
		//
		.i_dma_stall(i_wb_stall),
		.i_dma_ack(i_wb_ack), .i_dma_data(i_wb_data),
		.i_dma_err(i_wb_err),
		// }}}
		// }}}
	);

	// }}}
endmodule
