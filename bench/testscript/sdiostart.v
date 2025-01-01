////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/testscript/sdiostart.v
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
`include "../testscript/sdiolib.v"
// }}}

task	testscript;
	reg	[31:0]	read_data, ocr_reg, if_cond, op_cond, r6, sample_shift;
	reg	[15:0]	rca;
	reg	[127:0]	CID;
	integer		numio;
	reg	[7:0]	max_spd;
begin
	@(posedge clk);
	while(reset !== 1'b0)
		@(posedge clk);
	@(posedge clk);

	sdcard_discover;

$display("Starting with CMD=0x%08x, PHY=0x%08x",
		u_sdio.u_sdio.u_control.w_cmd_word,
		u_sdio.u_sdio.u_control.w_phy_ctrl);

	// Read our capabilities back from the controller
	// {{{
	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_200MHZ
				| SDPHY_WTEST | SPEED_CLKOFF
				| SDPHY_DDR | SDPHY_SHFTMSK);
repeat(5) @(posedge clk);
$display("Post write, CMD=0x%08x, PHY=0x%08x",
		u_sdio.u_sdio.u_control.w_cmd_word,
		u_sdio.u_sdio.u_control.w_phy_ctrl);

	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] > 8'h3);
$display("Done waiting on initial clock change");

	case(read_data[11:10])
	2'b00: numio = 1;
	default: numio = 4;
	endcase

	max_spd = read_data[7:0];
	if (3'h0 == read_data[18:16])
		// OPT_"RAW" (Neither SERDES nor DDR)
		sample_shift = { 11'h0, 5'h08, 16'h0 };
	else if (2'b00 == read_data[17:16])
		// OPT_DDR
		sample_shift = { 11'h0, 5'h0c, 16'h0 };
	else
		// OPT_SERDES
		sample_shift = { 11'h0, 5'h03, 16'h0 };
	// }}}

	// Now set up for the capabilities we will be using
	// {{{
	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_1MHZ | SDPHY_W1 | sample_shift);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_1MHZ[7:0]);
	// }}}

	u_bfm.readio(ADDR_SDCARD, read_data);
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
		op_cond[24] = OPT_1P8V;
		sdcard_send_op_cond(op_cond);

		do begin
			op_cond = 32'h40ff_8000;
			op_cond[24] = OPT_1P8V;
			sdcard_send_op_cond(op_cond);
		end while(1'b0 === op_cond[31]);
	end
	$display("OP-COND: %08x", op_cond);
	// }}}

	if (OPT_1P8V && op_cond[24])
	begin
		sdcard_send_voltage_switch;	// CMD11
	end else
		max_spd = SPEED_50MHZ;

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

	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_DS | SDPHY_W1 | sample_shift);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_DS[7:0]);

	// ACMD51	SEND_SCR -- can be used to determine what bus widths
	//			are supported

	// CMD42	// LOCK_UNLOCK, sets/resets the password of the card

	// ACMD6	SET_BUS_WIDTH, allowable data bus widths are in SCR reg
	sdcard_send_random_block(32'h00);
	sdcard_send_random_block(32'h03);
	sdcard_send_random_block(32'h01);
	sdcard_send_random_block(32'h02);

	sdcard_read_block(32'h00);

	sdcard_set_bus_width(2'b10);

	// Test at SPEED_DS=25MHZ SDR
	// {{{
	if (numio >= 4)
	begin
		u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_DS | SDPHY_W4 | sample_shift);

		// CMD6		// Select drive strength ??
		// CMD19	// Tuning block to determine sampling point
		// sdcard_read_ocr(ocr_reg);

		sdcard_send_random_block(32'h04);
		sdcard_send_random_block(32'h06);
		sdcard_send_random_block(32'h05);
		sdcard_send_random_block(32'h07);

		sdcard_read_block(32'h05);
	end
	// }}}

	// Test at SPEED_HSSDR=50MHZ SDR
	// {{{
	if (numio >= 4 && SPEED_HSSDR[7:0] >= max_spd)
	begin
		u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_HSSDR | SDPHY_W4 | sample_shift);

		sdcard_send_random_block(32'h0b);
		sdcard_send_random_block(32'h09);
		sdcard_send_random_block(32'h0a);
		sdcard_send_random_block(32'h08);

		sdcard_read_block(32'h0a);
		sdcard_read_block(32'h0b);
	end
	// }}}

	// Test at SPEED_SDR100=100MHZ SDR
	// {{{
	if (numio >= 4 && SPEED_HSSDR[7:0] >= max_spd)
	begin
		u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_SDR100 | SDPHY_W4 | sample_shift);

		if (OPT_1P8V && op_cond[24])
			sdcard_send_tuning_block;

		sdcard_send_random_block(32'h0c);
		sdcard_send_random_block(32'h0d);
		sdcard_send_random_block(32'h0e);
		sdcard_send_random_block(32'h0f);

		sdcard_read_block(32'h0d);
		sdcard_read_block(32'h0c);
	end
	// }}}

	// Test at fastest SDR speed=200MHz SDR
	// {{{
	if (numio >= 4 && SPEED_HSSDR[7:0] >= max_spd)
	begin
		u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_SDR200 | SDPHY_W4 | sample_shift);

		if (OPT_1P8V && op_cond[24])
			sdcard_send_tuning_block;

		sdcard_send_random_block(32'h10);
		sdcard_send_random_block(32'h11);
		sdcard_send_random_block(32'h12);
		sdcard_send_random_block(32'h13);

		sdcard_read_block(32'h10);
		sdcard_read_block(32'h11);
	end
	// }}}

	repeat(512)
		@(posedge clk);
end endtask
