////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/testscript/sdstream.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Drive and demonstrate the SD card controller's streaming
//		interface option that can be used with the DMA controller
//	and an (external) DMA source/sink.
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

always @(posedge clk)
if (!reset && stream_error_flag)
begin
	if (!error_flag)
		$display("ERROR: SDCard (Model) DMA stream error detected");
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

	// Start by "inserting" an SD card
	// {{{
`ifdef	SDIO_AXI
	read_data = 32'h0; read_data[2] = 1'b1;
	u_bfm.write_f(GPIO_ADDR+4, read_data);
`else
	read_data = 32'h0; read_data[18] = 1'b1; read_data[2] = 1'b1;
	u_bfm.write_f(GPIO_ADDR, read_data);
`endif
	// }}}

	// Then wait for that card to be detected
	// {{{
	wait(sdio_interrupt);
	u_bfm.readio(ADDR_SDCARD, read_data);
	assert(1'b0 === read_data[19]);
	// Clear the SD card removed bit
	read_data = 32'h0c_8080;
	u_bfm.write_f(ADDR_SDCARD, read_data);
	// }}}

	// Set clock speed = 25MHz
	// {{{
	// We can start at 25MHz b/c the simulation model allows us to.  We
	// might not do this with real hardware.
	sample_shift = { 11'h0, 5'h08, 16'h0 };
	u_bfm.write_f(ADDR_SDPHY, SECTOR_16B | SPEED_25MHZ | SDPHY_W1 | { 11'h0, 5'h1f, 16'h0 });
	u_bfm.readio(ADDR_SDPHY, read_data);
	if (3'h0 == read_data[18:16])
		// OPT_"RAW" (Neither SERDES nor DDR)
		sample_shift = { 11'h0, 5'h08, 16'h0 };
	else if (2'b00 == read_data[17:16])
		// OPT_DDR
		sample_shift = { 11'h0, 5'h0c, 16'h0 };
	else
		// OPT_SERDES
		sample_shift = { 11'h0, 5'h03, 16'h0 };
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
	end
	// else max_spd = SPEED_50MHZ;

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

	// Test at SPEED_DS=25MHZ SDR = 12.5MB/s, or 32b / 32 clocks
	// {{{
	$display("Set speed to DS, i.e. 25MHz");
	u_bfm.write_f(ADDR_SDPHY, SECTOR_512B | SPEED_DS | SDPHY_W4 | sample_shift);

	if (OPT_1P8V && op_cond[24])
		sdcard_send_tuning_block;

	sdcard_write_stream(5, 32'h3);
	sdcard_read_stream(5, 32'h3);
	// }}}

	// Test at SPEED_100=50MHZ SDR = 100MB/s, or 32b / 8 clocks
	// {{{
	if (OPT_SERDES || OPT_DDR)
	begin
		$display("Set speed to SDR50, i.e. 100MHz");
		u_bfm.write_f(ADDR_SDPHY, SECTOR_512B | SPEED_SDR100 | SDPHY_W4 | sample_shift);
		u_bfm.readio(ADDR_SDPHY, read_data);
		while(read_data[7:0] != SPEED_SDR100[7:0])
		begin
			u_bfm.readio(ADDR_SDPHY, read_data);
		end

		if (OPT_1P8V && op_cond[24])
			sdcard_send_tuning_block;

	sdcard_write_stream(7, 32'h2);
	sdcard_read_stream(7, 32'h2);
	end

	// }}}

	// Test at SPEED_200=200MHZ SDR = 100MB/s, or 32b / 4 clocks
	// {{{
	if (OPT_SERDES)
	begin
		$display("Set speed to SDR104, i.e. 200MHz");
		u_bfm.write_f(ADDR_SDPHY, SECTOR_512B | SPEED_SDR200 | SDPHY_W4 | sample_shift);
		u_bfm.readio(ADDR_SDPHY, read_data);
		while(read_data[7:0] != SPEED_SDR200[7:0])
		begin
			u_bfm.readio(ADDR_SDPHY, read_data);
		end

		if (OPT_1P8V && op_cond[24])
			sdcard_send_tuning_block;


		sdcard_write_stream(6, 32'h4);
		sdcard_read_stream(6, 32'h4);
	end
	// }}}

	repeat(512)
		@(posedge clk);
end endtask
