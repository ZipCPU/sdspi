////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/testscript/sdinsert.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Test inserting an SD card, to make sure the card detection
//		works and clears appropriately.
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
	reg	[31:0]	read_data, ocr_reg, if_cond, op_cond, r6, sample_shift,
			mem_offset;
	reg	[15:0]	rca;
	reg	[127:0]	CID;
	integer		numio;
	reg	[7:0]	max_spd;
begin
	@(posedge clk);
	while(reset !== 1'b0)
		@(posedge clk);
	@(posedge clk);

`ifdef	SDIO_AXI
	read_data = 32'h0; read_data[2] = 1'b1;
	u_bfm.writeio(GPIO_ADDR+8, read_data);		// CLEAR the bit
`else
	read_data = 32'h0;
	read_data[18] = 1'b1;		// Adjust bit 16+2
	read_data[2] = 1'b0;		// Clear it
	u_bfm.writeio(GPIO_ADDR, read_data);
`endif

	mem_offset = 32'h0;
	if (OPT_SDSLAVE)
		mem_offset = (MEM_ADDR + (MEM_ADDR >> 1)) >> 9;
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
		sample_shift = { 11'h0, 5'h0a, 16'h0 };
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

	// We stop here

	repeat(512)
		@(posedge clk);
end endtask
