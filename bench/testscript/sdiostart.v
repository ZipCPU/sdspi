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
// }}}
localparam	[AW+$clog2(DW/8)-1:0]	ADDR_SDCARD =  0,
					ADDR_SDDATA =  4,
					ADDR_FIFOA  =  8,
					ADDR_FIFOB  = 12,
					ADDR_SDPHY  = 16;

localparam [31:0]	SDIO_RNONE = 32'h0,
			SDIO_R1    = 32'h0100,
			SDIO_R2    = 32'h0200,
			SDIO_R1b   = 32'h0300,
			SDIO_WRITE = 32'h0400,
			SDIO_MEM   = 32'h0800,
			SDIO_FIFO  = 32'h1000,
			SDIO_DMA   = 32'h2000,
			SDIO_BUSY  = 32'h4800,
			SDIO_ERR   = 32'h8000;

localparam [31:0]	SDIO_DS    = 32'h00100,
			SDIO_DDR   = 32'h04200,
			SDIO_W1    = 32'h00000,
			SDIO_W4    = 32'h00400,
			SDIO_W8    = 32'h00800;

localparam [31:0]	SPEED_100KHZ   = 32'h00fc,
			SPEED_200KHZ   = 32'h007f,
			SPEED_400KHZ   = 32'h0041,
			SPEED_1MHZ     = 32'h001b,
			SPEED_5MHZ     = 32'h0007,
			SPEED_12MHZ    = 32'h0004,
			SPEED_25MHZ    = 32'h0003,
			SPEED_50MHZ    = 32'h0002;

localparam [31:0]	SPEED_DS     = 32'h03003,
			SPEED_HS     = 32'h03002,
			SPEED_SDR50  = 32'h03401,
			SPEED_DDR50  = 32'h07502,
			SPEED_SDR104 = 32'h07500;

localparam [31:0]	SPEED_SLOW    = SPEED_100KHZ,
			SPEED_DEFAULT = SPEED_DS,
			SPEED_FAST    = SPEED_HS;

localparam [31:0]	SECTOR_16B  = 32'h0400_0000,
			SECTOR_512B = 32'h0900_0000;

localparam [31:0]	SDIO_CMD     = 32'h0000_0040,
			SDIO_READREG  = SDIO_CMD | SDIO_R1 | SDIO_ERR,
			SDIO_WRITEBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
						| SDIO_WRITE | SDIO_MEM)+24,
			SDIO_READBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
						| SDIO_MEM)+17,
			SDIO_READCID = (SDIO_CMD | SDIO_R2 | SDIO_ERR)+2;

reg	r_interrupted;
initial	r_interrupted = 1'b0;
always @(posedge clk)
if (reset)
	r_interrupted <= 1'b0;
else if (interrupt)
	r_interrupted <= 1'b1;

task	sdio_wait_while_busy;
	// {{{
	reg	[31:0]	read_data;
	reg		prior_interrupt;
begin
	r_interrupted = 1'b0;
	u_bfm.readio(ADDR_SDCARD, read_data);
	while(read_data & SDIO_BUSY)
	begin
		do begin
			prior_interrupt = r_interrupted;
			u_bfm.readio(ADDR_SDCARD, read_data);
			// $display("CHECK IF BUSY -- %08x", read_data);
			// if (read_data & SDIO_BUSY) assert(!prior_interrupt);
		end while(read_data & SDIO_BUSY);
		assert(r_interrupted);
	end

	r_interrupted = 1'b0;
end endtask
// }}}

task	sdcard_go_idle;					// CMD0
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send a command 0
	u_bfm.writeio(ADDR_SDDATA, 32'h0); // 0x048040
	u_bfm.writeio(ADDR_SDCARD, SDIO_CMD | SDIO_RNONE | SDIO_ERR);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(!ctrl_reg[15]); // && ctrl_reg[17:16] == 2'b01);
end endtask
// }}}

task	sdcard_all_send_cid;				// CMD2
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD2: ALL_SEND_CID
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.writeio(ADDR_SDCARD, SDIO_READCID);	// 0x08242

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);

	// We leave the CID in the FIFO to be read out later
end endtask
// }}}

task	sdcard_send_relative_addr(output [31:0] r6);
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD3: SEND_RELATIVE_ADDR
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 3);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
	u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	sdcard_select_card(input[15:0] rca);		// CMD 7
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD3: SEND_RELATIVE_ADDR
	u_bfm.writeio(ADDR_SDDATA, { rca, 16'h0 });
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 7);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	sdcard_send_if_cond(inout [31:0] ifcond);	// CMD8
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	sdio_wait_while_busy;

	// Send CMD*: SEND_IF_COND
	u_bfm.writeio(ADDR_SDDATA, ifcond);
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 8); // 0x8148

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
	u_bfm.readio(ADDR_SDDATA, ifcond);
end endtask
// }}}

task	sdcard_send_op_cond(inout [31:0] op_cond);	// ACMD41
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
	sdcard_send_app_cmd;

	// Send a command 41
	u_bfm.writeio(ADDR_SDDATA, op_cond);			// 0x4000_0000
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 32'd41); // 0x8169

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	u_bfm.readio(ADDR_SDDATA, op_cond);
end endtask
// }}}

task	sdcard_set_bus_width(input [1:0] width);	// ACMD6
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
	sdcard_send_app_cmd;

	// Send an ACMD 6
	u_bfm.writeio(ADDR_SDDATA, { 30'h0, width });
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 32'd6);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}

task	sdcard_send_app_cmd;				// CMD 55
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 55); // 0x8177

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(!ctrl_reg[15] && ctrl_reg[17:16] == 2'b01);
	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	sdcard_read_ocr(output [31:0] read_data);	// CMD58
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send a command 58
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.writeio(ADDR_SDCARD, SDIO_READREG + 32'd58);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);

	u_bfm.readio(ADDR_SDDATA, read_data);
	$display("READ-OCR: %08x", read_data);
	if (read_data[31])
		$display("  Card has finished powering up");
	else
		$display("  Card is still powering up");

	if (read_data[30])
		$display("  CCS: High capacity support");
	if (read_data[29])
		$display("  UHS: USH-II card");
	if (read_data[24])
		$display("  S18A: Switching to 1.8V allowed");

	assert(read_data[30:0] == u_sdcard.ocr[30:0]) else error_flag = 1'b1;
end endtask
// }}}

task	sdcard_send_random_block(input[31:0] sector);	// CMD24
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg;
	integer		ik;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h9)
	begin
		phy_reg[27:24] = 4'h9;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end
	u_bfm.writeio(ADDR_SDDATA, sector);
	for(ik=0; ik<512/4; ik=ik+1)
		u_bfm.writeio(ADDR_FIFOA, $random);
	u_bfm.writeio(ADDR_SDCARD, SDIO_WRITEBLK);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}

task	sdcard_read_block(input[31:0] sector);	// CMD17
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg;
	integer		ik;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h9)
	begin
		phy_reg[27:24] = 4'h9;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end

	u_bfm.writeio(ADDR_SDDATA, sector);
	u_bfm.writeio(ADDR_SDCARD, SDIO_READBLK);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);

	for(ik=0; ik<512/4; ik=ik+1)
		u_bfm.writeio(ADDR_FIFOA, $random);
end endtask
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

	u_bfm.writeio(ADDR_SDPHY, SECTOR_16B | SPEED_DS | SDIO_W4);

	// CMD6		// Select drive strength ??
	// CMD19	// Tuning block to determine sampling point
	// sdcard_read_ocr(ocr_reg);

	sdcard_send_random_block(32'h04);
	sdcard_send_random_block(32'h06);
	sdcard_send_random_block(32'h05);
	sdcard_send_random_block(32'h07);

	sdcard_read_block(32'h05);

	repeat(512)
		@(posedge clk);
end endtask
