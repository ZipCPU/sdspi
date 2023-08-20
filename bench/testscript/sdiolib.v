////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdiolib.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Contains a library of routines to be used when writing test
//		scripts for the SDIO controller.
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
			SPEED_50MHZ    = 32'h0002,
			SPEED_100MHZ   = 32'h0001,
			SPEED_200MHZ   = 32'h0000;

localparam [31:0]	SPEED_DS     = 32'h03000|SPEED_25MHZ,	// Push/Pull,
			SPEED_HSSDR  = 32'h03000|SPEED_50MHZ,	// | push/pull
			SPEED_HSDDR  = 32'h07100|SPEED_50MHZ,	// |PP|DDR
			SPEED_DDR50  = 32'h07100|SPEED_50MHZ,	// |PP|DDR
			SPEED_SDR100 = 32'h03000|SPEED_100MHZ,	// |PP
			SPEED_DDR100 = 32'h07100|SPEED_100MHZ,	// |PP|DDR
			SPEED_SDR200 = 32'h03000|SPEED_200MHZ,	// |PP
			SPEED_HS200  = 32'h03000|SPEED_200MHZ,	// |PP|DDR
			SPEED_HS400  = 32'h07300|SPEED_200MHZ;	// |PP|DDR|DS

localparam [31:0]	SPEED_SLOW    = SPEED_100KHZ,
			SPEED_DEFAULT = SPEED_DS,
			SPEED_FAST    = SPEED_HSSDR;

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

	if (1'b0 !== ctrl_reg[15])
	begin
		$display("CMD FAILED CODE (%d) at %t", ctrl_reg[17:16], $time);
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);

	for(ik=0; ik<512/4; ik=ik+1)
		u_bfm.writeio(ADDR_FIFOA, $random);
end endtask
// }}}
