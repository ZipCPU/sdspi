////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	emmcstart.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Test script to start up an eMMC chip, and get it configured for
//		use.
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
`include "../testscript/emmclib.v"
// }}}

task	testscript;
	reg	[31:0]	read_data, op_cond, r6;
	reg		sector_addressing;
	reg	[15:0]	rca;
	reg	[127:0]	CID, CSD;
begin
	// u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_100KHZ | EMMC_W1);
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_1MHZ | EMMC_W1);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_1MHZ[7:0]);
	u_bfm.readio(ADDR_SDCARD, read_data);
	emmc_go_idle;

	// Send OP COND
	// {{{
	op_cond = 32'hc0ff_8080;		// Request high capacity
	emmc_send_op_cond(op_cond);

	do begin
		// if start_in_low_voltage
		//	op_cond = 32'h8000_0080;
		// else
		op_cond = 32'hc0ff_8080;
		emmc_send_op_cond(op_cond);
	end while(1'b0 === op_cond[31]);
	$display("OP-COND: %08x", op_cond);
	sector_addressing = op_cond[30];
	// }}}

	// Assign RCAs
	// {{{
	// CMD2		// All send CID
	emmc_all_send_cid;
	u_bfm.readio(ADDR_FIFOA, CID[127:96]);
	u_bfm.readio(ADDR_FIFOA, CID[ 95:64]);
	u_bfm.readio(ADDR_FIFOA, CID[ 63:32]);
	u_bfm.readio(ADDR_FIFOA, CID[ 31: 0]);
	$display("READ-CID: %08x:%08x:%08x:%08x",
		CID[127:96], CID[95:64], CID[63:32], CID[31:0]);
	assert(CID[127:8] == u_emmc.CID);

	// CMD3	SEND_RELATIVE_ADDR
	emmc_set_relative_addr(rca);
	$display("ASSIGNED-RCA: %04x", rca);
	// }}}

	// Read the CID register (again, directly)
	// {{{
	emmc_send_cid(rca);
	u_bfm.readio(ADDR_FIFOA, CID[127:96]);
	u_bfm.readio(ADDR_FIFOA, CID[ 95:64]);
	u_bfm.readio(ADDR_FIFOA, CID[ 63:32]);
	u_bfm.readio(ADDR_FIFOA, CID[ 31: 0]);
	$display("READ-CID: %08x:%08x:%08x:%08x",
		CID[127:96], CID[95:64], CID[63:32], CID[31:0]);
	assert(CID[127:8] == u_emmc.CID);
	// }}}

	// Read the CSD register
	// {{{
	emmc_send_csd(rca);
	u_bfm.readio(ADDR_FIFOA, CSD[127:96]);
	u_bfm.readio(ADDR_FIFOA, CSD[ 95:64]);
	u_bfm.readio(ADDR_FIFOA, CSD[ 63:32]);
	u_bfm.readio(ADDR_FIFOA, CSD[ 31: 0]);
	$display("READ-CSD: %08x:%08x:%08x:%08x",
		CSD[127:96], CSD[95:64], CSD[63:32], CSD[31:0]);
	assert(CSD[127:8] == u_emmc.CSD);
	// }}}

	// CMD7 SELECT/DESELCT Card
	// {{{
	emmc_select_card(rca);
	// }}}

	// Test at SPEED_DS=25MHz SDR, 1b
	// {{{
	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_DS | EMMC_W1);
	do begin
		u_bfm.readio(ADDR_SDPHY, read_data);
	end while(read_data[7:0] != SPEED_DS[7:0]);

	emmc_test_bus;

	emmc_send_random_block(32'h00 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h03 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h01 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h02 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h00 * (sector_addressing ? 1 : 512));

	$display("1b 25MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_DS=25MHZ SDR, 4b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b01);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_DS | EMMC_W4);

	emmc_test_bus;

	emmc_send_random_block(32'h04 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h06 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h05 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h07 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h05 * (sector_addressing ? 1 : 512));

	$display("4b 25MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_DS=25MHZ SDR, 8b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_DS | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h04 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h06 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h05 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h07 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h05 * (sector_addressing ? 1 : 512));

	$display("8b 25MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_HSSDR=50MHZ SDR, 1b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b00);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_HSSDR | EMMC_W1);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("1b 50MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_HSSDR=50MHZ SDR, 4b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b01);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_HSSDR | EMMC_W4);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("4b 50MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_HSSDR=50MHZ SDR, 8b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_HSSDR | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("8b 50MHz SDR TRANSFER TEST: Passed");
	// }}}

	// There is no 1b DDR configuration

	// Test at SPEED_HSDDR=50MHZ DDR, 4b
	// {{{
	emmc_switch(8'd185, 8'h1);
	emmc_set_bus_width(1'b0, 1'b1, 2'b01);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_HSDDR | EMMC_W4);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("4b 50MHz DDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_HSDDR=50MHZ DDR, 8b
	// {{{
	emmc_switch(8'd185, 8'h1);
	emmc_set_bus_width(1'b0, 1'b1, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_HSDDR | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("8b 50MHz DDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_SDR100=100MHZ SDR, 1b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b0);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_SDR100 | EMMC_W1);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("1b 100MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_SDR100=100MHZ SDR, 4b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b01);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_SDR100 | EMMC_W4);

	emmc_test_bus;

	emmc_send_random_block(32'h0b * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h09 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h08 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0a * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0b * (sector_addressing ? 1 : 512));

	$display("4b 100MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_SDR100=100MHZ SDR, 8b
	// {{{
	emmc_set_bus_width(1'b0, 1'b0, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_SDR100 | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h0c * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0d * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0e * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h0f * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h0d * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h0c * (sector_addressing ? 1 : 512));

	$display("8b 100MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_SDR200, fastest SDR speed=200MHz SDR, 8b
	// {{{
	emmc_switch(8'd185, 8'h2);
	emmc_set_bus_width(1'b0, 1'b0, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_SDR200 | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h10 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h11 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h12 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h13 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h10 * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h11 * (sector_addressing ? 1 : 512));

	$display("8b 200MHz SDR TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_HS400, fastest DDR speed=200MHz + DS, 8b
	// {{{
	emmc_switch(8'd185, 8'h3);
	emmc_set_bus_width(1'b0, 1'b1, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B|SPEED_HS400 | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h10 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h11 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h12 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h13 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h10 * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h11 * (sector_addressing ? 1 : 512));

	$display("8b 200MHz DDR+DS TRANSFER TEST: Passed");
	// }}}

	// Test at SPEED_HS400 + Enhanced DDR, SPEED=200MHz + EnhDS, 8b
	// {{{
	emmc_set_bus_width(1'b1, 1'b1, 2'b10);

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B|EMMC_DSCMD|SPEED_HS400 | EMMC_W8);

	emmc_test_bus;

	emmc_send_random_block(32'h10 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h11 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h12 * (sector_addressing ? 1 : 512));
	emmc_send_random_block(32'h13 * (sector_addressing ? 1 : 512));

	emmc_read_block(32'h10 * (sector_addressing ? 1 : 512));
	emmc_read_block(32'h11 * (sector_addressing ? 1 : 512));

	$display("8b 200MHz DDR+EDS TRANSFER TEST: Passed");
	// }}}

	repeat(512)
		@(posedge clk);
end endtask
