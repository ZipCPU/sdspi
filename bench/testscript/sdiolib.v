////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/testscript/sdiolib.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Contains a library of routines to be used when writing test
//		scripts for the SDIO controller.
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
// }}}
localparam	[ADDRESS_WIDTH-1:0]
				ADDR_SDCARD = SDIO_ADDR + 0,
				ADDR_SDDATA = SDIO_ADDR + 4,
				ADDR_FIFOA  = SDIO_ADDR + 8,
				ADDR_FIFOB  = SDIO_ADDR +12,
				ADDR_SDPHY  = SDIO_ADDR +16,
`ifdef	SDIO_AXI
				ADDR_DMABUS = SDIO_ADDR +20,
`else
				ADDR_DMABUS = SDIO_ADDR +24,
`endif
				ADDR_DMALEN = SDIO_ADDR +28;

localparam [31:0] ADDR_STREAM = { 1'b1, 31'h0 };

localparam [31:0]	SDIO_RNONE    = 32'h0000000,
			SDIO_R1       = 32'h0000100,
			SDIO_R2       = 32'h0000200,
			SDIO_R1b      = 32'h0000300,
			SDIO_WRITE    = 32'h0000400,
			SDIO_MEM      = 32'h0000800,
			SDIO_FIFO     = 32'h0001000,
			SDIO_DMA      = 32'h0002000,
			SDIO_CMDBUSY  = 32'h0004000,
			SDIO_ERR      = 32'h0008000,
			SDIO_REMOVED  = 32'h0040000,
			SDIO_PRESENTN = 32'h0080000,
			SDIO_CARDBUSY = 32'h0100000,
			SDIO_ACK      = 32'h4000000,
			SDIO_BUSY  = (SDIO_CARDBUSY | SDIO_CMDBUSY | SDIO_DMA | SDIO_MEM);

localparam [31:0]	SDPHY_DDR    = 32'h04100,	// Requires CLK90
			SDPHY_DS     = 32'h04300,	// Requires DDR & CLK90
			SDPHY_W1     = 32'h00000,
			SDPHY_W4     = 32'h00400,
			// SDPHY_W8  = 32'h00800,	// 8b SDIO not supported
			SDPHY_WTEST  = 32'h00c00,
			SDPHY_SHFTMSK= 32'h1f0000;

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

localparam [31:0]	SDIO_CMD     = 32'h0000_0040,
			SDIO_READREG  = SDIO_CMD | SDIO_R1 | SDIO_ERR,
			SDIO_READCID = (SDIO_CMD | SDIO_R2 | SDIO_ERR)+2,
			SDIO_WRITEBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
					| SDIO_ACK | SDIO_WRITE | SDIO_MEM)+24,
			SDIO_WRITEDMA = (SDIO_CMD | SDIO_R1 | SDIO_ERR| SDIO_DMA
					| SDIO_ACK | SDIO_WRITE | SDIO_MEM)+25,
			SDIO_READBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
						| SDIO_MEM)+17,
			SDIO_READDMA  = (SDIO_CMD | SDIO_R1 | SDIO_ERR| SDIO_DMA
						| SDIO_MEM)+18;

reg	r_interrupted;
initial	r_interrupted = 1'b0;
always @(posedge clk)
if (reset)
	r_interrupted <= 1'b0;
else if (sdio_interrupt)
	r_interrupted <= 1'b1;

task	sdio_wait_while_busy;
	// {{{
	reg	[31:0]	read_data;
	reg		prior_interrupt;
begin
$display("WAIT-WHILE-BUSY");
	r_interrupted   = 1'b0;
	prior_interrupt = 1'b0;
	u_bfm.readio(ADDR_SDCARD, read_data);
$display("FIRST-CHECK: %08x", read_data);
	if(read_data & SDIO_BUSY)
	begin
		while(!prior_interrupt && (read_data & SDIO_BUSY))
		begin
			prior_interrupt = prior_interrupt || r_interrupted;
			u_bfm.readio(ADDR_SDCARD, read_data);
			// $display("CHECK IF BUSY -- %08x", read_data);
			// if (read_data & SDIO_BUSY) assert(!prior_interrupt);
		end

		if (read_data & SDIO_BUSY)
		begin
			$display("ERROR: INTERRUPTED, but still busy. CMD= %08x, PRIOR=%1d, INT=%1d", read_data, prior_interrupt, r_interrupted);
			error_flag = 1'b1;
		end
		if (1'b1 !== r_interrupted)
		begin
			$display("ERROR: NO INTERRUPT!");
			if (1'b1 !== r_interrupted)
			begin
				$display("ERROR: I");
				error_flag = 1'b1;
			end

			assert(r_interrupted);
		end
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
	u_bfm.write_f(ADDR_SDCARD, SDIO_CMD | SDIO_RNONE | SDIO_ERR);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if (1'b0 !== ctrl_reg[15])
	begin
		$display("ERROR: A");
		error_flag = 1'b1;
	end

	assert(!ctrl_reg[15]);
end endtask
// }}}

task	sdcard_all_send_cid;				// CMD2
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD2: ALL_SEND_CID
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.write_f(ADDR_SDCARD, SDIO_READCID);	// 0x08242

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if (1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: B");
		error_flag = 1'b1;
	end

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
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 3);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if (1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: C");
		error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
	u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	sdcard_send_switch(input [31:0] swcmd);		// CMD6
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (4'h6 !== phy_reg[27:24])
	begin
		phy_reg[27:24] = 4'h6;
		u_bfm.write_f(ADDR_SDPHY, phy_reg);
	end

	// Send CMD6: SEND_SWITCH
	u_bfm.writeio(ADDR_SDDATA, swcmd);
	u_bfm.write_f(ADDR_SDCARD, (SDIO_READREG|SDIO_MEM) + 6);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if (1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: SWITCH-ERR"); error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);

	phy_reg[27:24] = 4'h9;
	u_bfm.write_f(ADDR_SDPHY, phy_reg);
end endtask
// }}}

task	sdcard_select_card(input[15:0] rca);		// CMD 7
	// {{{
	reg	[31:0]	ctrl_reg;
begin
	// Send CMD3: SEND_RELATIVE_ADDR
	u_bfm.writeio(ADDR_SDDATA, { rca, 16'h0 });
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 7);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if (1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: D"); error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	sdcard_send_if_cond(inout [31:0] ifcond);	// CMD8
	// {{{
	reg	[31:0]	ctrl_reg;
begin
$display("SEND-IF-COND");
	sdio_wait_while_busy;

	// Send CMD*: SEND_IF_COND
	u_bfm.writeio(ADDR_SDDATA, ifcond);
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 8); // 0x8148

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: IF-COND, Invalid response");
		error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
	u_bfm.readio(ADDR_SDDATA, ifcond);
end endtask
// }}}

task	sdcard_send_op_cond(inout [31:0] op_cond);	// ACMD41
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
$display("SEND-OP-COND");
	sdcard_send_app_cmd;
	sdio_wait_while_busy;

	// Send a command 41
	u_bfm.writeio(ADDR_SDDATA, op_cond);			// 0x4000_0000
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 32'd41); // 0x8169

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	u_bfm.readio(ADDR_SDDATA, op_cond);
end endtask
// }}}

task	sdcard_send_voltage_switch;	// CMD11
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg;
begin
$display("VOLTAGE-SWITCH");
	assert(OPT_1P8V);
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 11);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || ctrl_reg[17:16] !== 2'b01)
	begin
		$display("ERROR: Voltage switch unsuccessful");
		error_flag = 1'b1;
	end

	assert(!ctrl_reg[15] && ctrl_reg[17:16] == 2'b01);

	$display("Switching voltages!\n");
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	phy_reg[22] = 1;
	u_bfm.write_f(ADDR_SDPHY, phy_reg);
	$display("  ... DONE!\n");

end endtask
// }}}

reg	[31:0]	tuning_pat[0:15];

initial begin
	tuning_pat[ 0]=	32'hFF0FFF00;
	tuning_pat[ 1]=	32'hFFCCC3CC;
	tuning_pat[ 2]=	32'hC33CCCFF;
	tuning_pat[ 3]=	32'hFEFFFEEF;
	tuning_pat[ 4]=	32'hFFDFFFDD;
	tuning_pat[ 5]=	32'hFFFBFFFB;
	tuning_pat[ 6]=	32'hBFFF7FFF;
	tuning_pat[ 7]=	32'h77F7BDEF;
	tuning_pat[ 8]=	32'hFFF0FFF0;
	tuning_pat[ 9]=	32'h0FFCCC3C;
	tuning_pat[10]=	32'hCC33CCCF;
	tuning_pat[11]=	32'hFFEFFFEE;
	tuning_pat[12]=	32'hFFFDFFFD;
	tuning_pat[13]=	32'hDFFFBFFF;
	tuning_pat[14]=	32'hBBFFF7FF;
	tuning_pat[15]=	32'hF77F7BDE;
end

task	sdcard_send_tuning_block;	// CMD19
	// {{{
	reg	[31:0]	ctrl_reg, dat_reg, phy_reg;
	reg	[31:0]	vmask, best_eye, best_ph, first_eye, eyesz, ph, k;
	reg		vld;
begin
$display("SEND-TUNING-BLOCK");
	vmask     = 0;
	best_eye  = 0;
	best_ph   = 0;
	first_eye = 0;
	eyesz     = 0;
	ph        = 0;
	k         = 0;

	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h6)	// 64 bytes, 128bits per lane
	begin
		phy_reg[27:24] = 4'h6;
	end

	for(ph=0; ph<24; ph=ph+1)
	begin
		phy_reg[20:16] = ph;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);

		u_bfm.writeio(ADDR_SDDATA, 32'h0);
		u_bfm.write_f(ADDR_SDCARD, (SDIO_READREG|SDIO_MEM) + 19);

		sdio_wait_while_busy;

		u_bfm.readio(ADDR_SDCARD, ctrl_reg);

		vld = 1;
		if (ctrl_reg[15] !== 1'b0)
			vld = 1'b0;
		else for(k=0; k<16 && vld; k=k+1)
		begin
			u_bfm.readio(ADDR_FIFOA, dat_reg);
			if (tuning_pat[k] !== dat_reg)
				vld = 0;
		end

		if (vld)
		begin
			vmask[ph] = 1'b1;
			if (0 == eyesz)
				first_eye = ph;
			eyesz = eyesz + 1;
			$display("-- %2d is valid", ph);
		end else begin
			$display("-- %2d *INVALID*", ph);
			if (eyesz > best_eye)
			begin
				best_ph = (ph - first_eye)/ 2 + first_eye;
				best_eye = eyesz;
				$display("-- New best at %2d, eyesz=%2d from %2d", best_ph, eyesz, first_eye);
			end

			first_eye = 0;
			eyesz = 0;
		end
	end

	phy_reg[27:24] = 4'h9;		// Return us to 512B blocks
	phy_reg[20:16] = best_ph;	// Use the best phase offset we've found
	u_bfm.write_f(ADDR_SDPHY, phy_reg);

	// u_bfm.readio(ADDR_SDDATA, r6);
end endtask
// }}}

task	sdcard_set_bus_width(input [1:0] width);	// ACMD6
	// {{{
	reg	[31:0]	ctrl_reg, read_reg;
begin
	sdcard_send_app_cmd;

	// Send an ACMD 6
	u_bfm.writeio(ADDR_SDDATA, { 30'h0, width });
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 32'd6);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || ctrl_reg[17:16] !== 2'b01)
	begin
		$display("ERROR: Set-Bus width CMD failure");
		error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}

task	sdcard_send_app_cmd;				// CMD 55
	// {{{
	reg	[31:0]	ctrl_reg;
begin
$display("SEND-APP-CMD");
	u_bfm.writeio(ADDR_SDDATA, 32'h0);
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 55); // 0x8177

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || ctrl_reg[17:16] !== 2'b01)
	begin
		$display("ERROR: APP-CMD FAIL"); error_flag = 1'b1;
	end
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
	u_bfm.write_f(ADDR_SDCARD, SDIO_READREG + 32'd58);

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

	if (read_data[30:0] !== u_sdcard.ocr[30:0])
	begin
		$display("ERROR: F"); error_flag = 1'b1;
	end

	assert(read_data[30:0] == u_sdcard.ocr[30:0]);
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
	u_bfm.write_f(ADDR_SDCARD, SDIO_WRITEBLK);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: WRITEBLK FAIL"); error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}

task	sdcard_discover;
	// {{{
	reg	[31:0]	read_data, phy_reg;
begin
	// See if we can discover and report various settings properly.

	// First, wait for the card to be detected
	// {{{
	// Tell the GPIO to generate a card present signal
`ifdef	SDIO_AXI
	read_data = 32'h0; read_data[2] = 1'b1;
	u_bfm.write_f(GPIO_ADDR+4, read_data);
`else
	read_data = 32'h0; read_data[18] = 1'b1; read_data[2] = 1'b1;
	u_bfm.write_f(GPIO_ADDR, read_data);
`endif

	// Let the card present signal to activate (low)
	u_bfm.readio(ADDR_SDCARD, read_data);
	while (1'b1 === read_data[19])
	begin
		u_bfm.readio(ADDR_SDCARD, read_data);
	end
	// Clear any card removed signal
	if (1'b1 == read_data[18])
	begin
		read_data = 32'h000c_8080;
		u_bfm.write_f(ADDR_SDCARD, read_data);
	end
	// }}}

	// OPT_SERDES, OPT_DDR
	// {{{
	// !OPT_SERDES && OPT_DDR
	// !OPT_SERDES && !OPT_DDR
	u_bfm.readio(ADDR_SDPHY, read_data);
	phy_reg = read_data;
	phy_reg[20:16] = 5'h1f;
	phy_reg[7:0] = 8'h0;
	phy_reg[8] = 1'b0;	// Turn off DDR
	u_bfm.write_f(ADDR_SDPHY, phy_reg);
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	while(phy_reg[7:0] === read_data[7:0])
	begin
		u_bfm.readio(ADDR_SDPHY, phy_reg);
	end

	if (phy_reg[20:16] === 5'h1f)
	begin
		$display("Front-end:      OPT_SERDES");
	end else if (5'h1c === phy_reg[20:16])
	begin
		$display("Front-end:      OPT_DDR");
	end else if (5'h18 === phy_reg[20:16])
	begin
		$display("Front-end:      Raw");
	end else begin
		$display("ERROR: Front end support UNKNOWN, 0x%08x", phy_reg);
		error_flag = 1;
	end
	u_bfm.writeio(ADDR_SDPHY, read_data);
	// }}}

	// Max clock speed
	// {{{
	if (8'h00 === phy_reg[7:0])
	begin
		$display("Max-Speed:      200MHz");
	end else if (phy_reg[7:0] === 8'h01)
	begin
		$display("Max-Speed:      100MHz");
	end else if (phy_reg[7:0] === 8'h02)
	begin
		$display("Max-Speed:       50MHz");
	end else if (phy_reg[7:0] === 8'h03)
	begin
		$display("Max-Speed:       25MHz");
	end else begin
		$display("ERROR: Unexpected maximum speed, 0x%08x", phy_reg);
		error_flag = 1;
	end

	phy_reg[9] = 1'b1;
	phy_reg[8] = 1'b1;
	phy_reg[7:0] = 8'h0;
	u_bfm.writeio(ADDR_SDPHY, phy_reg);
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	while(phy_reg[7:0] === read_data[7:0])
	begin
		u_bfm.readio(ADDR_SDPHY, phy_reg);
	end

	if (phy_reg[8] === 1'b0)
	begin
		$display("DDR:            Unsupported");
	end else if (phy_reg[7:0] === 5'h00)
	begin
		$display("Max DDR Speed:  200MHz (HS400)");
	end else if (phy_reg[7:0] === 8'h01)
	begin
		$display("Max DDR Speed:  100MHz");
	end else if (phy_reg[7:0] === 8'h02)
	begin
		$display("Max DDR Speed:   50MHz");
	end else if (phy_reg[7:0] === 8'h03)
	begin
		$display("Max DDR Speed:   25MHz");
	end else begin
		$display("ERROR: Unexpected maximum speed, 0x%08x", phy_reg);
		error_flag = 1;
	end

	if (phy_reg[9] === 1'b1)
	begin
		$display("Data-Strobe:    Yes, Supported");
	end else if (phy_reg[9] === 1'b0)
	begin
		$display("Data-Strobe:    No support");
	end else begin
		$display("ERR: Invalid data-strobe response");
		error_flag = 1;
	end

	u_bfm.writeio(ADDR_SDPHY, read_data);
	// }}}

	// OPT_DMA (Is the DMA enabled?)
	// {{{
	read_data = 32'hffff_ffff;
	u_bfm.writeio(ADDR_DMALEN, read_data);
	u_bfm.readio(ADDR_DMALEN, read_data);
	if (read_data === 32'h0)
	begin
		$display("DMA      :      No DMA support");
	end else begin
		$display("DMA      :      Yes, Supported");
		read_data = 32'hffff_ffff;
		u_bfm.writeio(ADDR_DMABUS, read_data);
		u_bfm.readio(ADDR_DMABUS, read_data);
		// How ... do we handle AW detection?  Stream detection?
		//	Upper address bits in AXI mode?
		assert(read_data !== 32'h0);
		$display("DMA-MASK :      0x%08x", read_data);
	end
	// }}}

	// OPT_STREAM (Is the DMA+Stream interface enabled?)

	// OPT_HWRESET
	// {{{
	read_data = 32'h00_8080;	// Resets errors and FIFOs
	read_data[25] = 1'b1;		// Hardware reset request
	u_bfm.writeio(ADDR_SDCARD, read_data);
	u_bfm.readio(ADDR_SDCARD, read_data);
	if (1'b1 === read_data[25])
	begin
		$display("HW-Reset :      Yes, Supported");
`ifdef	SDIO_AXI
		u_bfm.readio(GPIO_ADDR+16, read_data);
		assert(1'b1 === read_data[2]);
`else
		u_bfm.readio(GPIO_ADDR, read_data);
		assert(1'b1 === read_data[18]);
`endif
		read_data = 32'h00_8080;	// Clear the reset
		u_bfm.writeio(ADDR_SDCARD, read_data);
		repeat (1536)
			@(posedge clk);
`ifdef	SDIO_AXI
		u_bfm.readio(GPIO_ADDR+16, read_data);
		assert(1'b0 === read_data[2]);
`else
		u_bfm.readio(GPIO_ADDR, read_data);
		assert(1'b0 === read_data[18]);
`endif
		$display("HW-Reset :      Cleared");
	end else begin
		$display("HW-Reset :      No support");
	end
	// }}}

	// OPT_CARD_DETECT (? Can this be detected?)
	// {{{
	// First, "Remove" the card
`ifdef	SDIO_AXI
	read_data = 32'h0; read_data[2] = 1'b1;
	u_bfm.write_f(GPIO_ADDR+8, read_data);
`else
	read_data = 32'h0; read_data[18] = 1'b1; read_data[2] = 1'b0;
	u_bfm.write_f(GPIO_ADDR, read_data);
`endif

	// Verify it is now seen as "removed"
	repeat (5)
		@(posedge clk);
	u_bfm.readio(ADDR_SDCARD, read_data);
	if (1'b1 === read_data[18])
	begin
		// We detected the card was removed.  Good.
		$display("CardDetection:  Supported");
		assert(read_data[19:18] === 2'b11);

		// Now let's see if we can insert it.
`ifdef	SDIO_AXI
		read_data = 32'h0; read_data[2] = 1'b1;
		u_bfm.write_f(GPIO_ADDR+4, read_data);
`else
		read_data = 32'h0; read_data[18] = 1'b1; read_data[2] = 1'b1;
		u_bfm.write_f(GPIO_ADDR, read_data);
`endif
		repeat (1536)
			@(posedge clk);
		u_bfm.readio(ADDR_SDCARD, read_data);
		if (2'b01 === read_data[19:18])
		begin
			$display("CardDetection:  Insertion detected");
		end else begin
			$display("CardDetection:  ERROR!  No insertion detected, 0x%08x", read_data);
			error_flag = 1;
		end
		assert(read_data[19:18] === 2'b01);

		read_data = 32'h00_8080;	// Clear the reset
		read_data[18] = 1'b1;
		u_bfm.write_f(ADDR_SDCARD, read_data);
		repeat(5) @(posedge clk);
		u_bfm.readio(ADDR_SDCARD, read_data);
		if (2'b00 === read_data[19:18])
		begin
			$display("CardDetection:  Insertion acknowledged");
		end else begin
			$display("CardDetection:  ERROR!  No acknowledgment, 0x%08x", read_data);
			error_flag = 1;
		end
	end else begin
		$display("CardDetection:  No support");
	end
	// }}}

	// OPT_CRCTOKEN (? Can this be detected? -- only following a write req)
	$display("Discovery complete");
end endtask
// }}}

task	sdcard_write_dma(input [31:0] nblocks, input[31:0] sector,	// CMD25
				input[31:0] wbaddr);
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg, dummy_data;
	integer		ik;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h9)
	begin
		phy_reg[27:24] = 4'h9;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end

$display("Pre-Writing data to memory");
	for(ik=0; ik<(nblocks<<9); ik=ik+4)
	begin
		dummy_data = $random;
		u_bfm.writeio(wbaddr + ik, dummy_data);
	end

	u_bfm.writeio(ADDR_SDDATA, sector);
	u_bfm.writeio(ADDR_DMABUS, wbaddr);
	u_bfm.writeio(ADDR_DMALEN, nblocks);
$display("Commanding DMA transaction");
	u_bfm.write_f(ADDR_SDCARD, SDIO_WRITEDMA);

$display("Waiting for completion");
	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: H, Write DMA error"); error_flag = 1'b1;
	end
	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}

task	sdcard_write_stream(input [31:0] nblocks, input[31:0] sector);
	// {{{
	reg	[31:0]	ctrl_reg, phy_reg, dummy_data;
	integer		ik;
begin
	u_bfm.readio(ADDR_SDPHY, phy_reg);
	if (phy_reg[27:24] != 4'h9)
	begin
		phy_reg[27:24] = 4'h9;
		u_bfm.writeio(ADDR_SDPHY, phy_reg);
	end

	u_bfm.writeio(SCK_ADDR+4, 32'h01);
	u_bfm.writeio(SCK_ADDR, { 1'b1, 1'b0, nblocks[20:0], 9'h0 });

	u_bfm.writeio(ADDR_SDDATA, sector);
	u_bfm.writeio(ADDR_DMABUS, ADDR_STREAM);
	u_bfm.writeio(ADDR_DMALEN, nblocks);
$display("Commanding DMA stream transaction");
	u_bfm.write_f(ADDR_SDCARD, SDIO_WRITEDMA);

$display("Waiting for completion");
	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);
	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: I, Write DMA error"); error_flag = 1'b1;
	end

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
	u_bfm.write_f(ADDR_SDCARD, SDIO_READBLK);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);

	if (1'b0 !== ctrl_reg[15])
	begin
		$display("CMD FAILED CODE (%d) at %t", ctrl_reg[17:16], $time);
	end

	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: J, Read block ERR"); error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);

	for(ik=0; ik<512/4; ik=ik+1)
		u_bfm.writeio(ADDR_FIFOA, $random);
end endtask
// }}}

task	sdcard_read_dma(input [31:0] nblocks, input[31:0] sector,
				input[31:0] wbaddr);
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
	u_bfm.writeio(ADDR_DMABUS, wbaddr);
	u_bfm.writeio(ADDR_DMALEN, nblocks);
	u_bfm.write_f(ADDR_SDCARD, SDIO_READDMA);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);

	if (1'b0 !== ctrl_reg[15])
	begin
		$display("CMD FAILED CODE (%d) at %t", ctrl_reg[17:16], $time);
	end

	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: K, Read DMA ERR"); error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}

task	sdcard_read_stream(input [31:0] nblocks, input[31:0] sector);
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

	u_bfm.writeio(SCK_ADDR+4, 32'h01);
	u_bfm.writeio(SCK_ADDR, { 1'b0, 1'b0, nblocks[20:0], 9'h0 });

	u_bfm.writeio(ADDR_SDDATA, sector);
	u_bfm.writeio(ADDR_DMABUS, ADDR_STREAM);
	u_bfm.writeio(ADDR_DMALEN, nblocks);
	u_bfm.write_f(ADDR_SDCARD, SDIO_READDMA);

	sdio_wait_while_busy;

	u_bfm.readio(ADDR_SDCARD, ctrl_reg);

	if (1'b0 !== ctrl_reg[15])
	begin
		$display("CMD FAILED CODE (%d) at %t", ctrl_reg[17:16], $time);
	end

	if(1'b0 !== ctrl_reg[15] || 2'b01 !== ctrl_reg[17:16])
	begin
		$display("ERROR: L, Read DMA ERR"); error_flag = 1'b1;
	end

	assert(1'b0 === ctrl_reg[15] && 2'b01 === ctrl_reg[17:16]);
end endtask
// }}}
