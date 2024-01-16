////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	emmclib.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Contains a library of routines to be used when writing test
//		scripts for the SDIO controller, when driving an eMMC chip.
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
// }}}
localparam	[AW+$clog2(DW/8)-1:0]	ADDR_SDCARD =  0,
					ADDR_SDDATA =  4,
					ADDR_FIFOA  =  8,
					ADDR_FIFOB  = 12,
					ADDR_SDPHY  = 16;

localparam [31:0]	EMMC_RNONE = 32'h000000,
			EMMC_R1    = 32'h000100,
			EMMC_R2    = 32'h000200,
			EMMC_R1b   = 32'h000300,
			EMMC_WRITE = 32'h000400,
			EMMC_MEM   = 32'h000800,
			EMMC_FIFO  = 32'h001000,
			EMMC_DMA   = 32'h002000,
			EMMC_BUSY  = 32'h104800,
			EMMC_ERR   = 32'h008000;

localparam [31:0]	EMMC_DS    = 32'h000100,
			EMMC_DDR   = 32'h004200,
			EMMC_DSCMD = 32'h200300,
			EMMC_W1    = 32'h000000,
			EMMC_W4    = 32'h000400,
			EMMC_W8    = 32'h000800,
			EMMC_WTEST = 32'h000c00,
			EMMC_SHFTMSK=32'h1f0000;

localparam [31:0]	SPEED_100KHZ   = 32'h00fc,
			SPEED_200KHZ   = 32'h007f,
			SPEED_400KHZ   = 32'h0041,
			SPEED_1MHZ     = 32'h001b,
			SPEED_5MHZ     = 32'h0007,
			SPEED_12MHZ    = 32'h0004,
			SPEED_25MHZ    = 32'h0003,
			SPEED_50MHZ    = 32'h0002,
			SPEED_100MHZ   = 32'h0001,
			SPEED_200MHZ   = 32'h0000,
			SPEED_CLKOFF   = 32'h8000;

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

localparam [31:0]	EMMC_CMD      = 32'h0000_0040,
			EMMC_READREG  = EMMC_CMD | EMMC_R1 | EMMC_ERR,
			EMMC_WRITEBLK = (EMMC_CMD | EMMC_R1b | EMMC_ERR
						| EMMC_WRITE | EMMC_MEM)+24,
			EMMC_READBLK = (EMMC_CMD | EMMC_R1 | EMMC_ERR
						| EMMC_MEM)+17,
			EMMC_ALLCID = (EMMC_CMD | EMMC_R2 | EMMC_ERR)+2;

reg	[7:0]	ext_csd	[0:511];

reg	r_interrupted;
initial	r_interrupted = 1'b0;
always @(posedge clk)
if (reset)
	r_interrupted <= 1'b0;
else if (interrupt)
	r_interrupted <= 1'b1;

task	emmc_wait_while_busy;
	// {{{
	reg	[31:0]	read_data;
	reg		prior_interrupt;
begin
	r_interrupted = 1'b0;
	u_bfm.readio(ADDR_SDCARD, read_data);
	while(read_data & EMMC_BUSY)
	begin
		do begin
			prior_interrupt = r_interrupted;
			u_bfm.readio(ADDR_SDCARD, read_data);
			// $display("CHECK IF BUSY -- %08x", read_data);
			// if (read_data & EMMC_BUSY) assert(!prior_interrupt);
		end while(read_data & EMMC_BUSY);
		assert(r_interrupted);
	end

	r_interrupted = 1'b0;
end endtask
// }}}

task	emmc_go_idle;					// CMD0
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send a command 0
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.write_f(ADDR_SDCARD, EMMC_CMD | EMMC_RNONE | EMMC_ERR);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(!ctrl_reg[15]) else $finish; // && ctrl_reg[17:16] == 2'b01);
end endtask
// }}}

task	emmc_all_send_cid;				// CMD2
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD2: ALL_SEND_CID
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.write_f(ADDR_SDCARD, EMMC_ALLCID);	// 0x08242

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]); // else $finish;

	// We leave the CID in the FIFO to be read out later
end endtask
// }}}

task	emmc_set_relative_addr(output [15:0] RCA);
	// {{{
	reg	[31:0]	r1;
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD3: SET_RELATIVE_ADDR
	do begin
		RCA = $random;
	end while(RCA < 2);

	u_bfm.writeio(ADDR_SDDATA, { RCA, 16'h0 });
	u_bfm.write_f(ADDR_SDCARD, EMMC_READREG + 3);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]); // else $finish;
	u_bfm.readio(ADDR_SDDATA, r1);
end endtask
// }}}

task	emmc_select_card(input[15:0] rca);		// CMD 7
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD7: SELECT_DESECT_CARD
	u_bfm.writeio(ADDR_SDDATA, { rca, 16'h0 });
	u_bfm.write_f(ADDR_SDCARD, EMMC_READREG + 7);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]); // else $finish;
	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	emmc_send_op_cond(inout [31:0] op_cond);	// CMD1
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
	// Send a command 1
	u_bfm.writeio(ADDR_SDDATA, op_cond);			// 0x4000_0000
	u_bfm.write_f(ADDR_SDCARD, EMMC_READREG + 32'd1);	// 0x8101

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	u_bfm.readio(ADDR_SDDATA, op_cond);
end endtask
// }}}

task	emmc_send_status(input[15:0] rca, inout [31:0] status_reg); // CMD13
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send a command 1
	u_bfm.writeio(ADDR_SDDATA, { rca, 16'h0 });
	u_bfm.write_f(ADDR_SDCARD, EMMC_READREG + 32'd13);	// 0x810d

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	u_bfm.readio(ADDR_SDDATA, status_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-STATUS"); error_flag = 1'b1; end
end endtask
// }}}

task	emmc_send_csd(input[15:0] rca);			// CMD9
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD9: SEND_CSD
	u_bfm.writeio(ADDR_SDDATA, { rca, 16'h0 });
	u_bfm.write_f(ADDR_SDCARD, (EMMC_CMD | EMMC_R2 | EMMC_ERR) + 9);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-CSD"); error_flag = 1'b1; end
	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	emmc_send_cid(input[15:0] rca);			// CMD10
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD10: SEND_CID
	u_bfm.writeio(ADDR_SDDATA, { rca, 16'h0 });
	u_bfm.write_f(ADDR_SDCARD, (EMMC_CMD | EMMC_R2 | EMMC_ERR) + 10);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-CID"); error_flag = 1'b1; end
	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	emmc_read_ext_csd;				// CMD8
	// {{{
	reg	[31:0]	ctrl_reg, read_reg, uv, phy_reg;
	integer		ik;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h9)
	begin
		phy_reg[27:24] = 4'h9;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end

	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.write_f(ADDR_SDCARD, (EMMC_READREG|EMMC_MEM) + 32'd8);

	emmc_wait_while_busy;

	for(ik=0; ik<512/4; ik=ik+1)
	begin
		u_bfm.readio(ADDR_FIFOA, uv);
		ext_csd[511-(4*ik+0)] = uv[31:24];
		ext_csd[511-(4*ik+1)] = uv[23:16];
		ext_csd[511-(4*ik+2)] = uv[15: 8];
		ext_csd[511-(4*ik+3)] = uv[ 7: 0];
	end

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-EXTCSD"); error_flag = 1'b1; end
end endtask
// }}}

task	emmc_switch(input[7:0] addr, input [7:0] val);	// CMD6
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
	// Send an CMD 6
	u_bfm.writeio(ADDR_SDDATA, { 6'h0, 2'b01, addr, val, 8'h0 });
	u_bfm.write_f(ADDR_SDCARD, EMMC_READREG + 32'd6);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-SWITCH"); error_flag = 1'b1; end
	ext_csd[addr] = val;
end endtask
// }}}

task	emmc_set_bus_width(input enh_ds, input ddr, input [1:0] width);
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
	// Send an ACMD 6
	if (width == 0)
		ddr = 1'b0;
	if (width == 3)
		width = 2;
	if (width != 2'b10)
		enh_ds = 1'b0;
	emmc_switch(8'd183, { enh_ds, 3'h0, 1'b0, ddr, width });
end endtask
// }}}

task	emmc_test_bus;
	// {{{
	reg	[31:0]	ctrl_reg, read_reg, phy_reg;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h3)
	begin
		phy_reg[27:24] = 4'h3;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end

	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	case(phy_reg[11:10])
	2'b01: begin
		u_bfm.writeio(ADDR_FIFOA, 32'h5a00_0000);
		u_bfm.writeio(ADDR_FIFOA, 32'h0000_0000);
		end
	2'b10: begin
		u_bfm.writeio(ADDR_FIFOA, 32'h55aa_0000);
		u_bfm.writeio(ADDR_FIFOA, 32'h0000_0000);
		end
	// 2'b00:
	default: begin	// Also 2'b00 (1b width)
		u_bfm.writeio(ADDR_FIFOA, 32'h8000_0000);
		u_bfm.writeio(ADDR_FIFOA, 32'h0000_0000);
		end
	endcase

	u_bfm.write_f(ADDR_SDCARD, (EMMC_CMD | EMMC_R1 | EMMC_ERR
						| EMMC_WRITE | EMMC_MEM)+19);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-TEST/1"); error_flag = 1'b1; end


	u_bfm.write_f(ADDR_SDCARD, (EMMC_CMD | EMMC_R1 | EMMC_ERR
						| EMMC_WRITE | EMMC_MEM)+14);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-TEST/2"); error_flag = 1'b1; end
end endtask
// }}}

task	emmc_send_random_block(input[31:0] sector);	// CMD24
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
$display("WRITE-BLOCK TO %08x", sector);
	u_bfm.writeio(ADDR_SDDATA, sector);
	for(ik=0; ik<512/4; ik=ik+1)
		u_bfm.writeio(ADDR_FIFOA, $random);
	u_bfm.write_f(ADDR_SDCARD, EMMC_WRITEBLK);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-WRITE"); error_flag = 1'b1; end

	// do begin
	//	emmc_send_status(rca, status_reg);
	// end while(status_reg);
end endtask
// }}}

task	emmc_read_block(input[31:0] sector);	// CMD17
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg, ignore, status_reg;
	integer		ik;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h9)
	begin
		phy_reg[27:24] = 4'h9;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end

$display("READ-BLOCK FROM %08x", sector);
	u_bfm.writeio(ADDR_SDDATA, sector);
	u_bfm.write_f(ADDR_SDCARD, EMMC_READBLK);

	emmc_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	u_bfm.readio(ADDR_SDDATA, status_reg);

	if (1'b0 !== ctrl_reg[15])
	begin
		$display("READ CMD FAILED: CODE (%d) at %t",
				ctrl_reg[17:16], $time);
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16])
		else begin $display("ERR-RDBLOCK/1"); error_flag = 1'b1; end
	assert(32'h0 == (status_reg & 32'hffffe080))
		else begin $display("ERR-RDBLOCK/2"); error_flag = 1'b1; end

	for(ik=0; ik<512/4; ik=ik+1)
		u_bfm.readio(ADDR_FIFOA, ignore);
end endtask
// }}}
