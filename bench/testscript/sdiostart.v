////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdiostart.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2023, Gisselquist Technology, LLC
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
	reg	[31:0]	read_data, ocr_reg, if_cond, op_cond, r6;
	reg	[15:0]	rca;
	reg	[127:0]	CID;
begin
	// u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_100KHZ | SDIO_W1);
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_1MHZ | SDIO_W1);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_1MHZ[7:0]);
	u_bfm.readio(ADDR_SDCARD, read_data);
	sdcard_go_idle;

	if_cond = 32'h01a5;
	sdcard_send_if_cond(if_cond);
	u_bfm.readio(ADDR_SDCARD, read_data);
	$display("IF-COND: %08x", read_data);
	if (read_data[15] && read_data[17:16] == 2'b00)
	begin
		// No response from card
		do begin
			op_cond = 32'h0;
			sdcard_send_op_cond(op_cond);
		end while(op_cond[31]);
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

	/*
	if (OPT_DUAL_VOLTAGE && op_cond[24])
	begin
		// CMD11	// Voltage switch command
		// sdcard_switch_voltage;
	end
	*/

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

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_DS | SDIO_W1);
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
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_DS | SDIO_W4);

	// CMD6		// Select drive strength ??
	// CMD19	// Tuning block to determine sampling point
	// sdcard_read_ocr(ocr_reg);

	sdcard_send_random_block(32'h04);
	sdcard_send_random_block(32'h06);
	sdcard_send_random_block(32'h05);
	sdcard_send_random_block(32'h07);

	sdcard_read_block(32'h05);
	// }}}
	//

	// Test at SPEED_HSSDR=50MHZ SDR
	// {{{
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_HSSDR | SDIO_W4);

	sdcard_send_random_block(32'h0b);
	sdcard_send_random_block(32'h09);
	sdcard_send_random_block(32'h0a);
	sdcard_send_random_block(32'h08);

	sdcard_read_block(32'h0a);
	sdcard_read_block(32'h0b);
	// }}}

	// Test at SPEED_SDR100=100MHZ SDR
	// {{{
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_SDR100 | SDIO_W4);

	sdcard_send_random_block(32'h0c);
	sdcard_send_random_block(32'h0d);
	sdcard_send_random_block(32'h0e);
	sdcard_send_random_block(32'h0f);

	sdcard_read_block(32'h0d);
	sdcard_read_block(32'h0c);
	// }}}

	// Test at fastest SDR speed=200MHz SDR
	// {{{
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_SDR200 | SDIO_W4);

	sdcard_send_random_block(32'h10);
	sdcard_send_random_block(32'h11);
	sdcard_send_random_block(32'h12);
	sdcard_send_random_block(32'h13);

	sdcard_read_block(32'h10);
	sdcard_read_block(32'h11);
	// }}}

	repeat(512)
		@(posedge clk);
end endtask
