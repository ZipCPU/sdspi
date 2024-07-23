////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/testscript/sddmachk.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Drive and demonstrate the DMA controller.
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
`include "../testscript/sdiolib.v"
// }}}

reg	expect_dma_err = 1'b0;

always @(posedge clk)
if (!reset && !expect_dma_err && (u_sdio.u_sdio.dma_err
		|| (u_sdio.u_sdio.s2sd_valid
		    && (u_sdio.u_sdio.s2sd_data != u_sdio.u_sdio.s2sd_data))
		|| (u_sdio.u_sdio.sd2s_valid
		    && (u_sdio.u_sdio.sd2s_data != u_sdio.u_sdio.sd2s_data))))
begin
	$display("ERROR: Unexpected DMA ERR detected");
	error_flag <= 1'b1;
end

always @(posedge clk)
if (!reset && !expect_dma_err && u_sdcard.rx_err && !error_flag)
begin
	$display("ERROR: SDCard (Model) RX ERR (bad CRC) detected");
	error_flag <= 1'b1;
end


task	testscript;
	reg	[31:0]	read_data, ocr_reg, if_cond, op_cond, r6;
	reg	[15:0]	rca;
	reg	[127:0]	CID;
	reg	[31:0]	sample_shift;
begin
	@(posedge clk);
	while(reset !== 1'b0)
		@(posedge clk);
	@(posedge clk);

	// Set clock speed = 25MHz
	// {{{
	// We can start at 25MHz b/c the simulation model allows us to.  We
	// might not do this with real hardware.
	sample_shift = { 11'h0, 5'h08, 16'h0 };
	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_25MHZ | SDPHY_W1 | { 11'h0, 5'h1f, 16'h0 });
	u_bfm.readio(ADDR_SDPHY, read_data);
	if (read_data[18:16] == 3'h0)
		sample_shift = { 11'h0, 5'h8, 16'h0 };
	else if (read_data[17:16] == 2'h0)
		sample_shift = { 11'h0, 5'hc, 16'h0 };
	else
		sample_shift = { 11'h0, 5'ha, 16'h0 };
	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_25MHZ | SDPHY_W1 | sample_shift);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_25MHZ[7:0]);
	u_bfm.readio(ADDR_SDCARD, read_data);
	// }}}

	$display("CMD0: GO-IDLE");
	sdcard_go_idle;

	// IF condition and OP-Cond
	// {{{
	if_cond = 32'h01a5;
	sdcard_send_if_cond(if_cond);
	u_bfm.readio(ADDR_SDCARD, read_data);
	$display("IF-COND: %08x", read_data);
	if (read_data[15] && read_data[17:16] == 2'b00)
	begin
		// No response from card
		do begin
			op_cond = 32'h0ff_8000;
			sdcard_send_op_cond(op_cond);
		end while(1'b0 === op_cond[31]);
	end else begin
		assert(if_cond[7:0] == 8'ha5);

		op_cond = 32'h4000_0000;
		op_cond[24] = 1'b0; // OPT_DUAL_VOLTAGE;
		sdcard_send_op_cond(op_cond);

		do begin
			op_cond = 32'h40ff_8000;
			op_cond[24] = 1'b0; // OPT_DUAL_VOLTAGE;
			sdcard_send_op_cond(op_cond);
		end while(1'b0 === op_cond[31]);
	end
	$display("OP-COND: %08x", op_cond);
	// }}}

	// Assign RCAs
	// {{{
	// CMD2		// All send CID
	sdcard_all_send_cid;
	u_bfm.readio(ADDR_FIFOA, CID[127:96]);
	u_bfm.readio(ADDR_FIFOA, CID[ 95:64]);
	u_bfm.readio(ADDR_FIFOA, CID[ 63:32]);
	u_bfm.readio(ADDR_FIFOA, CID[ 31: 0]);
	$display("READ-CID: %08x:%08x:%08x:%08x",
		CID[127:96], CID[95:64], CID[63:32], CID[31:0]);
	assert(CID[127:8] == u_sdcard.CID);

	// CMD3	SEND_RELATIVE_ADDR
	sdcard_send_relative_addr(r6);
	rca = r6[31:16];
	$display("ASSIGNED-RCA: %04x", rca);
	// }}}

	// CMD7 SELECT/DESELCT Card
	sdcard_select_card(rca);

	// Set up for 1b @ 25MHz
	// {{{
	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_DS | SDPHY_W1 | sample_shift);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_DS[7:0]);
	// }}}

	$display("Set bus width to 4b");
	sdcard_set_bus_width(2'b10);

$display("Set speed to 25MHz");
	// Test at SPEED_DS=25MHZ SDR = 12.5MB/s, or 32b / 32 clocks
	// {{{
	u_bfm.write_f(ADDR_SDPHY, SECTOR_512B | SPEED_DS | SDPHY_W4 | sample_shift);

	sdcard_write_dma(5, 32'h3, MEM_ADDR);

	sdcard_read_dma(4, 32'h4, MEM_ADDR + ((5+3)<<9));
	sdcard_read_dma(1, 32'h3, MEM_ADDR + ((5+3+4)<<9));
	// }}}

	// Test at SPEED_200=200MHZ SDR = 100MB/s, or 32b / 4 clocks
	// {{{
$display("Set speed to 200MHz");
	u_bfm.write_f(ADDR_SDPHY, SECTOR_512B | SPEED_SDR200 | SDPHY_W4 | sample_shift);
	sdcard_write_dma(6, 32'h4, MEM_ADDR);
	sdcard_read_dma(4, 32'h5, MEM_ADDR + ((6+4)<<9));
	// }}}

	// 1 block, sector 0, going right up to the end of memory
	u_bfm.readio(ADDR_SDCARD, read_data);
	sdcard_read_dma(1, 32'h9, MEM_ADDR + (1<<LGMEMSZ)-512);

	repeat(512)
		@(posedge clk);
end endtask
