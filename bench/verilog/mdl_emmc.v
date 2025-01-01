////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/mdl_emmc.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	The SDIO SD-Card controller can operate on either SD cards or
//		eMMC cards.  Since the two aren't quite the same, a separate
//	model is needed when interacting with an eMMC card.  One difference,
//	for example, is that the eMMC bus is defined for up to 8 IO pins,
//	whereas the SDIO model only ever uses up to 4 IO pins.  A second
//	key difference, is that the eMMC protocol allows data-strobe qualified
//	IO--something never exploited by the SDIO interface.  Other differences
//	exist at the command layer.
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
`timescale 1ns/1ps
// }}}
module	mdl_emmc #(
		// {{{
		parameter [0:0]	OPT_DUAL_VOLTAGE = 1'b0,
		parameter [0:0]	OPT_HIGH_CAPACITY = 1'b0,
		parameter	LGMEMSZ = 20,	// Log_2(Mem size in bytes)
		parameter	LGBOOTSZ = 17,	// Minimum of 17, for 128kB
		parameter	MAX_BLKLEN = 512,	// Max Blk Size in bytes
		// MEM_HEX: If non-zero, is the name of a hex file to be used
		// to initialize the main memory of the device.
		parameter	MEM_HEX = 0,
		// BOOT_HEX: If non-zero, is the name of a hex file to be used
		// to initialize the first boot memory of the device on
		// startup.
		parameter	BOOT_HEX = 0,
		localparam	LGBLKSZ = $clog2(MAX_BLKLEN/4),
		localparam	MEMSZ = (1<<LGMEMSZ),
		localparam	BOOTSZ = (1<<LGBOOTSZ)
		// }}}
	) (
		// {{{
		input	wire		rst_n,
		input	wire		sd_clk,
		inout	wire		sd_cmd,
		inout	wire	[7:0]	sd_dat,
		output	wire		sd_ds,
		input	wire		i_1p8v
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[3:0]	EMMC_IDLE		= 4'h0,
				EMMC_READY		= 4'h1,
				EMMC_IDENTIFICATION	= 4'h2,
				EMMC_STANDBY		= 4'h3,
				EMMC_TRANSFER		= 4'h4,
				EMMC_SEND_DATA		= 4'h5, // = DATA
				EMMC_RECEIVE_DATA	= 4'h6,	// = RCV
				EMMC_PROGRAMMING	= 4'h7,	// = PRG
				EMMC_DISCONNECT		= 4'h8,
				EMMC_BUS_TEST		= 4'h9,
				//
				EMMC_INACTIVE		= 4'ha,
				EMMC_PRE_IDLE		= 4'hb,
				// EMMC_PRE_BOOT		= 4'hc,
				EMMC_BOOT		= 4'hd,
				EMMC_WAIT_IRQ		= 4'he,
				EMMC_SLEEP		= 4'hf;
	localparam [0:0]	REPLY_48B		= 1'b0,
				REPLY_136B		= 1'b1;

	localparam [31:0]	ERR_ADDRRANGE  = (32'h1 << 31),
				ERR_ILLEGALCMD = (32'h1 << 22),
				ERR_BLOCKLEN   = (32'h1 << 20),
				ERR_SWITCH     = (32'h1 <<  7);
	parameter realtime tPROG = 5120;	// 512ns, or about a half uS
	parameter realtime WRITE_TIME = 512;
	parameter realtime DS_DELAY = 0.4;


	reg	[3:0]	card_state;

	reg		cfg_ddr, cfg_ppull;
	reg	[1:0]	cfg_width;
	reg	[2:0]	cfg_partition;

	wire		cmd_valid, cmd_ds, cmd_crc_err;
	wire	[5:0]	cmd;
	wire	[31:0]	cmd_arg;

	reg		reply_valid, reply_type, reply_crc;
	wire		reply_busy;
	reg	[5:0]	reply;
	reg	[119:0]	reply_data;

	reg	[119:0]	CID, CSD;
	reg	[31:0]	ocr;
	reg		power_up_busy, cmd_alt, card_reset, sector_addressing;
	// reg		r_1p8v_request, r_1p8v;
	reg	[15:0]	RCA;
	reg	[31:0]	R1;
	reg		drive_cmd, card_selected;
	wire		cmd_collision;

	reg		read_en, pending_read, multi_block;
	wire		rx_valid, rx_last, rx_good, rx_err;
	wire	[31:0]	rx_data;

	integer	read_ik;
	reg		write_en, tx_valid, tx_last, pending_write;
	reg	[31:0]	tx_data;
	wire		tx_ready, tx_ds;
	reg	[LGBLKSZ-1:0]	tx_addr;

	reg	[31:0]	mem_buf	[0:(MAX_BLKLEN/4)-1];
	reg	[LGBLKSZ-1:0]	rx_addr;

	reg	[31:0]	mem		[0:(MEMSZ/4)-1];
	reg	[31:0]	boot_mem1	[0:(BOOTSZ/4)-1];
	reg	[31:0]	boot_mem2	[0:(BOOTSZ/4)-1];
	reg	[LGMEMSZ-1:0]	read_posn;
	integer		write_ik;

	reg	[7:0]	ext_csd	[0:511];

	reg	[15:0]	block_len;

	reg		bustest_w, bustest_r, clear_errors, write_ext_csd,
			SQS;
	wire	[31:0]	QSR;

	reg	[6:0]	boot_clk_count;
	reg		boot_mode, boot_active;
	reg		busy_programming;

	reg	err_addr_out_of_range, err_address_misalign,
		err_erase_seq_error,
		err_erase_param, err_wp_violation,
		err_device_is_locked, err_lock_unlock_failed,
		err_com_crc_error, err_illegal_command,
		err_device_ecc_failed, err_cc_error,
		err_generic_error, err_cidcsd_overwrite,
		err_wp_erase_skip, err_erase_reset,
		err_switch_error;
		// err_block_len_error,
		// R1[6] = exception_event;
		// R1[5] = cmd_alt;
	reg	[7:0]	ext_index, next_ext_byte;
	reg	ds_enabled = 1'b0, enhanced_ds_enabled = 1'b0;


	assign	QSR = 32'hffff_ffff;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Initial memory load
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	generate if (BOOT_HEX != 0)
	begin
		initial	begin
			$readmemh(BOOT_HEX, boot_mem1);
		end
	end endgenerate

	generate if (MEM_HEX != 0)
	begin
		initial	begin
			$readmemh(MEM_HEX, boot_mem1);
		end
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Command wire handler
	// {{{

	mdl_sdcmd
	tb_sdcmd (
		// {{{
		.rst_n(rst_n), .sd_clk(sd_clk), .sd_cmd(sd_cmd),
			.sd_ds(cmd_ds),
		//
		.o_cmd_valid(cmd_valid), .o_cmd(cmd), .o_arg(cmd_arg),
			.o_crc_err(cmd_crc_err),
		//
		.i_valid(reply_valid), .i_type(reply_type),
			.o_busy(reply_busy), .i_reply(reply),
			.i_arg(reply_data), .i_use_crc(reply_crc),
		.i_drive(drive_cmd), .o_collision(cmd_collision)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Read data from the host (write operation)
	// {{{

	mdl_sdrx
	tb_sdrx (
		// {{{
		.rst_n(rst_n), .sd_clk(sd_clk), .sd_dat( sd_dat ),
		//
		.i_rx_en(read_en), .i_width(cfg_width), .i_ddr(cfg_ddr),
		.i_len(block_len),
		//
		.o_valid(rx_valid), .o_data(rx_data), .o_last(rx_last),
			.o_good(rx_good), .o_err(rx_err)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Read from the card, send to the host/user (read operation)
	// {{{
	reg	r_crcack, r_crcnak;
	reg	pending_ack, pending_nak;

	mdl_sdtx
	tb_sdtx (
		// {{{
		.rst_n(rst_n && (!boot_mode || sd_cmd === 1'b0)),
			.sd_clk(sd_clk), .sd_dat( sd_dat ), .sd_ds(tx_ds),
		//
		.i_en(write_en),
			.i_width(cfg_width), .i_ddr(cfg_ddr),
			.i_ppull(cfg_ppull),
		//
		.i_crcack(r_crcack), .i_crcnak(r_crcnak),
		//
		.i_valid(tx_valid), .o_ready(tx_ready),
			.i_data(tx_data), .i_last(tx_last)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Boot setup
	// {{{

	initial	boot_mode = 1'b0;

	initial	boot_clk_count = 0;
	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
		boot_clk_count <= 0;
	else if (sd_cmd !== 1'b0 || card_state != EMMC_PRE_IDLE)
		boot_clk_count <= 0;
	else if (!(&boot_clk_count))
		boot_clk_count <= boot_clk_count + 1;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// eMMC FSM
	// {{{


	////////////////////////////////////////////////////////////////////////
	//
	// EXT CSD register support
	// {{{

	initial begin
		for(read_ik=0; read_ik < 512; read_ik=read_ik + 1)
			ext_csd[read_ik] = 8'h0;

		// BOOT INFO
		ext_csd[228] = 8'h7;

		// BOOT_SIZE_MULT
		ext_csd[226] = (8'h1 << (LGBOOTSZ - 17));

		// Sector count
		{ ext_csd[215], ext_csd[214], ext_csd[213], ext_csd[212] }
			= (32'd1<<(LGMEMSZ-9));

		// DRIVER_STRENGTH
		ext_csd[197] = 8'h1;

		// DEVICE_TYPE
		ext_csd[196] = 8'hff;	// Support all speeds up to HS400

		// EXT_CSD_REV
		ext_csd[192] = 8'd8;

		// CMD_SET
		ext_csd[191] = 8'd8;

		// HS_TIMING
		ext_csd[185] = 8'd0;

		// STROBE_SUPPORT
		ext_csd[184] = 8'd1;

		// BUS_WIDTH
		ext_csd[183] = 8'd0;

		// Only some of these registers are properly implemented
	end

	always @(ext_csd[185])
	begin
		ds_enabled = (ext_csd[185][3:0] == 4'h3);
	end

	assign	#DS_DELAY sd_ds = (ds_enabled
				&& (tx_ds || (enhanced_ds_enabled && cmd_ds)));

	// EXT-CSD[183]: cfg_ddr, cfg_width, and enhanced_ds_enabled
	// {{{
	reg	[7:0]	ext_csd_183;
	always @(*)
		ext_csd_183 = ext_csd[183];

	always @(ext_csd[183], boot_mode, ext_csd[177])
	begin
		if (boot_mode)
		begin
			cfg_ddr = (ext_csd[177][4:3] == 2'h2);
			case(ext_csd[177][1:0])
			2'h0: cfg_width = (cfg_ddr) ? 2'h1 : 2'h0;
			2'h1: cfg_width = 2'h1;
			2'h2: cfg_width = 2'h2;
			endcase

			enhanced_ds_enabled = 1'b0;
		end else begin
			case(ext_csd[183][3:0])
			4'h0: begin cfg_ddr = 1'b0; cfg_width = 2'b0; end
			4'h1: begin cfg_ddr = 1'b0; cfg_width = 2'd1; end
			4'h2: begin cfg_ddr = 1'b0; cfg_width = 2'd2; end
			//
			4'h5: begin cfg_ddr = 1'b1; cfg_width = 2'd1; end
			4'h6: begin cfg_ddr = 1'b1; cfg_width = 2'd2; end
			endcase

			enhanced_ds_enabled = ext_csd[183][7];
		end
	end
	// }}}

	always @(*)
	begin
		ext_index = cmd_arg[23:16];
		next_ext_byte = ext_csd[ext_index];
		if (cmd_arg[25:24] == 2'b01)
			next_ext_byte = cmd_arg[15:8];
		else if (cmd_arg[25:24] == 2'b10)
			next_ext_byte = next_ext_byte & (~cmd_arg[15:8]);
		else if (cmd_arg[25:24] == 2'b11)
			next_ext_byte = next_ext_byte | (cmd_arg[15:8]);
	end
			

	// SWITCH command processing to update ext_csd
	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		ext_csd[191] <= 8'd8;
		ext_csd[185] <= 8'h0;
		ext_csd[184] <= 8'd1;
		ext_csd[183] <= 8'h0;
	end else if (boot_mode)
	begin
		ext_csd[183] <= 8'h0;
		if (ext_csd[179][5:3] == 3'h1)
			cfg_partition <= 3'h1;
		else if (ext_csd[179][5:3] == 3'h2)
			cfg_partition <= 3'h2;
		else
			cfg_partition <= 3'h0;

		if (sd_cmd !== 1'b0)
			cfg_partition <= 3'h0;
	end else if (cmd_valid && !cmd_alt && cmd[5:0] == 6'd0)
	begin
		// GO IDLE
		ext_csd[183] <= 8'h0;
	end else if (cmd_valid && !cmd_alt && cmd[5:0] == 6'd6
							&& card_selected)
	begin
		// [25:24] = access
		// [23:16] = index
		// [15: 8] = value
		// [ 7: 3] = 5'h0
		// [ 2: 0] = Cmd Set
		if (cmd_arg[23:16] >= 192)
		begin
			// Read only registers
		end else if (cmd_arg[25:24] != 2'b00)
		begin
			case(ext_index)
			default: ext_csd[ext_index] <= next_ext_byte;
			endcase
		end
	end
	// }}}

	/*
	initial	cmd_alt = 1'b0;
	initial	r_1p8v_request = 1'b0;
	initial	r_1p8v = 1'b0;
	initial	R1  = 32'h0;
	*/
	initial	drive_cmd = 1'b0;
	initial	power_up_busy = 1'b1;
	initial	ocr = 32'h00ff_8000;
	initial	RCA = 16'h1;
	initial	sector_addressing = OPT_HIGH_CAPACITY;
	initial	card_selected = 1'b1;
	initial	cfg_width = 2'b0;	// 1b width
	initial	cfg_ddr = 1'b0;		// SDR
	initial	read_en = 1'b0;
	initial	write_ext_csd = 1'b0;
	initial	bustest_r = 1'b0;
	initial	bustest_w = 1'b0;
	initial	busy_programming = 1'b0;
	initial	cfg_partition = 2'b0;

	initial	begin
		reply_valid = 1'b0;
		reply_type  = REPLY_48B;
		reply = 6'h0;
		reply_data = {(120){1'b0}};

		card_state = EMMC_PRE_IDLE;

		CID[119:112] = $random;		// MFGR ID
		CID[111:106] = $random;		// BIN: Bank index number
		CID[105:104] = $random;		// Device / BGA
		CID[103: 96] = "GT";		// OED/ Application ID
		CID[ 95: 48] = "SM-MDL";	// Product name
		CID[ 47: 40] = $random;		// Product revision
		CID[ 39:  8] = $random;		// Serial number
		CID[  7:  0] = $random;		// Manufacturing date
		// CRC not included in our version

		block_len = 16'd512;
		clear_errors = 1'b0;
	end

	always @(*)
	begin
		if(power_up_busy)
			CSD[119] = 1'b0;
		CSD = 120'h0;
	end


	always @(*)
		ocr[31] = !power_up_busy;
	always @(*)
		ocr[30] = sector_addressing && OPT_HIGH_CAPACITY;

	// CRC error handling
	// {{{
	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
		err_com_crc_error <= 1'b0;
	else if (clear_errors)
	begin
		if (reply_valid && !reply_busy)
			err_com_crc_error <= 1'b0;
	end else if (cmd_crc_err || rx_err)
		err_com_crc_error <= 1'b1;
	// }}}

	// Other error handling
	// {{{
	always @(*)
	begin
		R1 = 32'h0;
		R1[31] = err_addr_out_of_range;
		R1[30] = err_address_misalign;
		// R1[29] = err_block_len_error;
		R1[28] = err_erase_seq_error;
		R1[27] = err_erase_param;
		R1[26] = err_wp_violation;
		R1[25] = err_device_is_locked;
		R1[24] = err_lock_unlock_failed;
		R1[23] = err_com_crc_error;
		R1[22] = err_illegal_command;
		R1[21] = err_device_ecc_failed;
		R1[20] = err_cc_error;
		R1[19] = err_generic_error;
		R1[16] = err_cidcsd_overwrite;
		R1[15] = err_wp_erase_skip;
		R1[13] = err_erase_reset;
		R1[12:9] = card_state;
		// R1[8] = ready_for_data;
		R1[7] = err_switch_error;
		// R1[6] = exception_event;
		R1[5] = cmd_alt;
	end

	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		err_addr_out_of_range	<= 1'b0;
		err_address_misalign	<= 1'b0;
		// err_block_len_error	<= 1'b0;
		err_erase_seq_error	<= 1'b0;
		err_erase_param		<= 1'b0;
		err_wp_violation	<= 1'b0;
		err_device_is_locked	<= 1'b0;
		err_lock_unlock_failed	<= 1'b0;
		// err_com_crc_error	<= 1'b0;
		err_illegal_command	<= 1'b0;
		err_device_ecc_failed	<= 1'b0;
		err_cc_error		<= 1'b0;
		err_generic_error	<= 1'b0;
		err_cidcsd_overwrite	<= 1'b0;
		err_wp_erase_skip	<= 1'b0;
		err_erase_reset		<= 1'b0;
		err_switch_error	<= 1'b0;
	end else if (clear_errors)
	begin
		err_addr_out_of_range	<= 1'b0;
		err_address_misalign	<= 1'b0;
		// err_block_len_error	<= 1'b0;
		err_erase_seq_error	<= 1'b0;
		err_erase_param		<= 1'b0;
		err_wp_violation	<= 1'b0;
		err_device_is_locked	<= 1'b0;
		err_lock_unlock_failed	<= 1'b0;
		// err_com_crc_error	<= 1'b0;
		err_illegal_command	<= 1'b0;
		err_device_ecc_failed	<= 1'b0;
		err_cc_error		<= 1'b0;
		err_generic_error	<= 1'b0;
		err_cidcsd_overwrite	<= 1'b0;
		err_wp_erase_skip	<= 1'b0;
		err_erase_reset		<= 1'b0;
		err_switch_error	<= 1'b0;
	end
	// }}}

	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		// {{{
		card_state <= EMMC_PRE_IDLE;

		cmd_alt <= 1'b0;
		reply_crc <= 1'b1;
		reply_type <= REPLY_48B;
		card_reset <= 0;
		// r_1p8v_request <= 1'b0;
		reply_valid <= 1'b0;
		pending_read  <= 1'b0;
		pending_write <= 1'b0;
		write_ext_csd <= 1'b0;
		clear_errors  <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
		boot_mode <= 1'b1;
		boot_active <= 1'b0;
		cfg_ppull <= 1'b0;
		// }}}
	end else if (card_state == EMMC_INACTIVE)
	begin
		clear_errors  <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
		cfg_ppull <= 1'b0;
	end else if (cmd_valid && cmd[5:0] == 6'h0
			&& cmd_arg != 32'hfffffffa && cmd_arg != 32'hf0f0f0f0)
	begin // CMD0: GO_IDLE_STATE, overrides all other internal states
		// {{{
		if (cmd_arg == 32'hf0f0f0f0)
			card_state <= EMMC_PRE_IDLE;
		else if (cmd_arg == 32'hffff_fffa)
		begin
			card_state <= EMMC_PRE_IDLE;
			boot_mode <= 1;
		end

		reply_valid <= 1'b0;
		// ocr[31] <= power_up_busy;
		ocr[7]  <= 1'b0;
		card_reset <= 1;

		pending_read <= 1'b0;
		read_en <= 1'b0;

		pending_write <= 1'b0;
		write_en <= 1'b0;

		RCA <= 16'h0;
		drive_cmd <= 1'b0;
		card_selected <= 1'b1;
		cfg_width <= 2'b0;
		write_ext_csd <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
		cfg_ppull <= 1'b0;
		// }}}
	end else if (card_state == EMMC_PRE_IDLE)
	begin
		if (sd_cmd !== 1'b0)
			card_state <= EMMC_IDLE;
		else if (boot_clk_count >= 73 && boot_mode)
			card_state <= EMMC_BOOT;
		clear_errors  <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
	/*
	end else if (card_state == EMMC_PRE_BOOT
			&& (!cmd_valid || cmd_crc_err || cmd[5:0] != 6'd1))
	begin
		clear_errors  <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
	*/
	end else if (card_state == EMMC_BOOT)
	begin
		clear_errors  <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
		write_ext_csd <= 1'b0;

		if (sd_cmd !== 1'b0 || (tx_valid && tx_last && read_posn >= BOOTSZ))
		begin
			pending_write <= 1'b0;
			multi_block   <= 1'b0;
			card_state <= EMMC_IDLE;
		end else if (!boot_active)
		begin
			read_posn <= cmd_arg;
			pending_write <= 1'b1;
			multi_block   <= 1'b1;
			boot_active <= 1'b1;
		end
	end else if (cmd_valid && !cmd_crc_err && (card_state != EMMC_IDLE
				|| cmd[5:0] == 6'd1))
	begin
		cmd_alt <= 1'b0;
		reply <= cmd[5:0];
		reply_crc <= 1'b1;	// All replies get CRCs by default
		reply_type <= REPLY_48B;
		card_reset <= 0;
		write_ext_csd <= 1'b0;
		clear_errors  <= 1'b0;
		bustest_w <= 1'b0;
		bustest_r <= 1'b0;
		// r_1p8v_request <= 1'b0;
		casez({ cmd_alt, cmd[5:0] })
		// ACMDs
		{ 1'b1, 6'd41 }: begin	// (An SDIO command ...)
			// {{{
			assert(0);
			end
			// }}}
		// Regular commands
		{ 1'b?, 6'd0 }: begin //! CMD0: Go idle
			// {{{
			if (cmd_arg == 32'hf0f0f0f0)
				card_state <= EMMC_PRE_IDLE;
			else if (cmd_arg == 32'hffff_fffa)
			begin
				card_state <= EMMC_PRE_IDLE;
				boot_mode <= 1;
			end

			reply_valid <= 1'b0;
			// ocr[31] <= power_up_busy;
			ocr[7]  <= 1'b0;
			card_reset <= 1;

			pending_read <= 1'b0;
			read_en <= 1'b0;

			pending_write <= 1'b0;
			write_en <= 1'b0;

			RCA <= 16'h0;
			drive_cmd <= 1'b0;
			card_selected <= 1'b1;
			cfg_width <= 2'b0;
			cfg_ppull <= 1'b0;
			end
			// }}}
		{ 1'b?, 6'd1 }: begin //! CMD1: SEND_OP_COND
			// {{{
			// Possibly card_state <= EMMC_INACTIVE
			if (card_state == EMMC_IDLE || card_state == EMMC_READY)
			begin
				card_state <= EMMC_READY;

				sector_addressing <= (cmd_arg[30:29] == 2'b10) && OPT_HIGH_CAPACITY;
				reply_valid <= #7 1'b1;
				reply <= 6'b111111;
				// r_1p8v_request <= (OPT_DUAL_VOLTAGE && cmd_arg[24] && !power_up_busy);
				reply_data <= { {(120-32){1'b0}}, ocr[31],
					((cmd_arg[30] && sector_addressing)
							&& OPT_HIGH_CAPACITY),
					30'h0ff8080};
				reply_crc  <= 1'b0;
				if (0 == (ocr[23:8] & cmd_arg[23:8]))
				begin
					reply_data[31] <= 1'b0;
					power_up_busy <= 1;
				end else begin
					// 5 command/reply cycles at 100kHz
					// power_up_busy <= #(5*2*48*10000)1'b0;
					// or ... 5 command/reply cycles at 1MHz
					power_up_busy <= #(5*2*48*1000) 1'b0;
				end
			end end
			// }}}
		{ 1'b?, 6'd2 }: begin //! CMD2: ALL_SEND_CID
			// {{{
			if (card_selected && !power_up_busy)
			begin
				reply_valid <= #7 1'b1;
				reply_type <= REPLY_136B;
				reply <= 6'd2;
				reply_data <= CID;
				drive_cmd <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd3 }: begin //! CMD3: SET_RELATIVE_ADDR
			// {{{
			card_state <= EMMC_STANDBY;
			if (!cmd_collision)
			begin
				RCA <= cmd_arg[31:16];

				reply_valid <= #7 1'b1;
				reply <= 6'd3;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;

				card_selected <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd4 }: begin // CMD4: SET_DSR
				// {{{
				card_state <= EMMC_SLEEP;
			end
			// }}}
		{ 1'b?, 6'd5 }: begin // CMD5: SLEEP_AWAKE
			// {{{
			if (cmd_arg[31:16] == RCA)
			begin
				if (card_state == EMMC_SLEEP)
				begin
					card_state <= EMMC_STANDBY;
					cfg_ppull <= 1'b1;
				end else if (card_state == EMMC_STANDBY)
				begin
					card_state <= EMMC_SLEEP;
					cfg_ppull <= 1'b0;
				end
			end end
			// }}}
		{ 1'b?, 6'd6 }: begin //!! CMD6: SWITCH_FUNCTION (+/- DDR, etc)
			// {{{
			if (card_selected)
			begin
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;

				if (cmd_arg[23:16] >= 192 || cmd_arg[25:24] == 2'b00)
					reply_data[31:0] <= R1 | ERR_SWITCH;
			end end
			// }}}
		{ 1'b?, 6'd7 }: begin //! CMD7: SELECT_DESELECT_CARD
			// {{{
			if (cmd_arg[31:16] == RCA)
			begin
				card_selected <= 1'b1;
				reply_valid <= #7 1'b1;
				reply <= 6'd2;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;
				cfg_ppull <= 1'b1;
				if (busy_programming)
					card_state <= EMMC_PROGRAMMING;
				else
					card_state <= EMMC_TRANSFER;
			end else begin
				cfg_ppull <= 1'b0;
				if (busy_programming)
					card_state <= EMMC_DISCONNECT;
				else
					card_state <= EMMC_STANDBY;
				card_selected <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd8 }: begin // CMD8: SEND_EXT_CSD
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_SEND_DATA;
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1 };
				clear_errors  <= 1'b1;
				pending_write <= 1'b1;
				multi_block   <= 1'b0;
				write_ext_csd <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd9 }: begin //! CMD9: SEND_CSD
			// {{{
			if (cmd_arg[31:16] == RCA && card_state == EMMC_STANDBY)
			begin
				card_state <= EMMC_STANDBY;
				// card_state <= EMMC_SEND_DATA;
				reply_valid <= #7 1'b1;
				reply <= 6'h3f;
				reply_type <= REPLY_136B;
				reply_data <= CSD;
				cfg_ppull <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd10 }: begin // CMD10: SEND_CID
			// {{{
			if (cmd_arg[31:16] == RCA && card_state == EMMC_STANDBY)
			begin
				card_state <= EMMC_STANDBY;
				reply_valid <= #7 1'b1;
				reply <= 6'd10;
				reply_type <= REPLY_136B;
				reply_data <= CID;
				cfg_ppull <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd13 }: begin // CMD13: SEND_STATUS
			// {{{
			if (cmd_arg[31:16] == RCA)
			begin
				card_state <= EMMC_STANDBY;
				reply_valid <= #7 1'b1;
				// reply <= 6'd13;
				SQS = cmd_arg[15];
				// HPI = cmd_arg[ 0]
				cfg_ppull <= 1'b0;
				if (!SQS)
				begin
					reply_data <= { {(120-32){1'b0}}, R1};
					clear_errors  <= 1'b1;
				end else
					// Query the QSR (Queue Status Register)
					// which we haven't yet implemented.
					reply_data <= { {(120-32){1'b0}},QSR};
			end end
			// }}}
		{ 1'b?, 6'd12 }: begin //! CMD12: STOP_TRANSMISSION
			// {{{
			if (cmd_arg[31:16] == RCA)
			begin
				multi_block   <= 1'b0;
				pending_read  <= 0;
				pending_write <= 0;
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1 };

				if (card_state == EMMC_SEND_DATA)
					card_state <= EMMC_TRANSFER;
				else if (card_state == EMMC_RECEIVE_DATA)
					card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		{ 1'b?, 6'd14 }: begin //!! CMD14: BUSTEST_R
			// {{{
			if (card_selected && card_state == EMMC_BUS_TEST)
			begin
				card_state    <= EMMC_TRANSFER;
				read_en       <= 0;
				// rx_addr    <= 0;
				reply_valid   <= #7 1'b1;
				reply_data    <= { {(120-32){1'b0}}, R1 };
				pending_write <= (rx_addr != 0);
				multi_block   <= 1'b0;
				bustest_r     <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd15 }: begin // CMD15: GO_INACTIVE_STATE
			// {{{
			if (cmd_arg[31:16] == RCA)
			begin
				card_state <= EMMC_INACTIVE;
				cfg_ppull <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd16 }: begin // CMD16: SET_BLOCKLEN
			// {{{
			if (card_selected)
			begin
				// assert(block_len[31:16] == 0);
				// assert(block_len <= MAX_BLKLEN);
				//
				reply_valid <= #7 1'b1;
				reply <= 6'd16;
				if (block_len > MAX_BLKLEN)
				begin
					reply_data <= { {(120-32){1'b0}},
							R1 | ERR_BLOCKLEN};
				end else begin
					block_len <= cmd_arg[15:0];
					reply_data <= { {(120-32){1'b0}}, R1 };
				end
				clear_errors  <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd17 }: begin //! CMD17: READ_SINGLE_BLOCK
			// {{{
$display("CMD17 request");
			if (card_selected && card_state == EMMC_TRANSFER)
			begin
$display("CMD17 request, card selected, in proper state");
				card_state <= EMMC_SEND_DATA;
				//
				pending_write <= 1'b1;
				multi_block   <= 1'b0;
				// write_en <= 1'b1;
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;
				read_posn <= cmd_arg;

				if (busy_programming)
				begin
$display("READ-CMD-ERR: Already busy");
					reply_data<= { {(120-32){1'b0}},
						R1 | ERR_ILLEGALCMD };
					pending_write <= 1'b0;
				end else if (sector_addressing)
				begin
					read_posn <= cmd_arg << 9;
					if (cmd_arg << 9 >= (40'd1 << LGMEMSZ)
								- block_len)
					begin
						reply_data<= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
						pending_write <= 1'b0;
$display("READ-CMD-ERR: Sector out of bounds");
					end
				end else if (cmd_arg >= (40'd1 << LGMEMSZ)
								- block_len)
				begin
					reply_data<= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
					pending_write <= 1'b0;
$display("READ-CMD-ERR: Small Sector out of bounds");
				end
				rx_addr <= 0;
			end else if (card_selected)
			begin
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				reply_data[31:0] <= R1 | ERR_ILLEGALCMD;
				clear_errors  <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd18 }: begin //! CMD18: READ_MULTIPLE_BLOCK
			// {{{
			if (card_selected && card_state == EMMC_TRANSFER)
			begin
				card_state <= EMMC_SEND_DATA;
				//
				pending_write <= 1'b1;
				multi_block   <= 1'b1;
				// write_en <= 1'b1;
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;

				read_posn <= cmd_arg;
				if (busy_programming)
				begin
					reply_data<= { {(120-32){1'b0}},
						R1 | ERR_ILLEGALCMD };
					pending_write <= 1'b0;
					multi_block   <= 1'b0;
				end else if (sector_addressing)
				begin
					read_posn <= cmd_arg << 9;
					if((cmd_arg << 9) >= (40'd1 << LGMEMSZ)
								- block_len)
					begin
						reply_data<= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
						pending_write <= 1'b0;
						multi_block   <= 1'b0;
					end
				end else if (cmd_arg >= (32'd1 << LGMEMSZ)
								- block_len)
				begin
					reply_data <= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
					pending_write <= 1'b0;
					multi_block   <= 1'b0;
				end
				rx_addr <= 0;
			end else if (card_selected)
			begin
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				reply_data[31:0] <= R1 | ERR_ILLEGALCMD;
				clear_errors  <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd19 }: begin //! CMD19: BUSTEST_W
			// {{{
			if (card_selected && card_state == EMMC_TRANSFER)
			begin
				// Read an undetermined amount from the host,
				// buffer it for later repeating it back to
				// the host.
				card_state <= EMMC_BUS_TEST;
				bustest_w <= 1'b1;
				rx_addr <= 0;
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1 };
			end end
			// }}}
		{ 1'b?, 6'd21 }: begin // CMD21: SEND_TUNING_BLOCK
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_RECEIVE_DATA;
			end end
			// }}}
		// { 1'b?, 6'd23 }: begin // CMD23: SET_BLOCK_COUNT
		{ 1'b?, 6'd24 }: begin //! CMD24: Write block
			// {{{
			if (card_selected && card_state == EMMC_TRANSFER)
			begin
				card_state <= EMMC_PROGRAMMING;
				reply_valid <= #7 1'b1;
				reply <= 6'd24;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;
				multi_block   <= 1'b0;
				busy_programming <= 1'b1;

				pending_read <= 1'b1;
				read_posn <= cmd_arg;
				if (sector_addressing)
				begin
					read_posn <= cmd_arg << 9;
					if((cmd_arg << 9) >= (40'd1 << LGMEMSZ)
								- block_len)
					begin
						reply_data<= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
						pending_read <= 1'b0;
					end
				end else if (cmd_arg >= (32'd1 << LGMEMSZ)
								- block_len)
				begin
					reply_data <= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
					pending_read <= 1'b0;
				end
				rx_addr <= 0;
			end else if (card_selected)
			begin
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				reply_data[31:0] <= R1 | ERR_ILLEGALCMD;
				clear_errors  <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd25 }: begin //! CMD25: WRITE_MULTIPLE_BLOCK
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
				reply_valid <= #7 1'b1;
				reply <= 6'd24;
				reply_data <= { {(120-32){1'b0}}, R1};
				clear_errors  <= 1'b1;
				multi_block   <= 1'b1;
				busy_programming <= 1'b1;

				pending_read <= 1'b1;
				read_posn <= cmd_arg;
				if (sector_addressing)
				begin
					read_posn <= cmd_arg << 9;
					if((cmd_arg << 9) >= (40'd1 << LGMEMSZ)
								- block_len)
					begin
						reply_data<= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
						pending_read <= 1'b0;
						multi_block   <= 1'b0;
						busy_programming <= 1'b0;
					end
				end else if (cmd_arg >= (32'd1 << LGMEMSZ)
								- block_len)
				begin
					reply_data <= { {(120-32){1'b0}},
							R1 | ERR_ADDRRANGE };
					pending_read <= 1'b0;
					multi_block   <= 1'b0;
					busy_programming <= 1'b0;
				end
				rx_addr <= 0;
			end else if (card_selected)
			begin
				reply_valid <= #7 1'b1;
				reply_data <= { {(120-32){1'b0}}, R1};
				reply_data[31:0] <= R1 | ERR_ILLEGALCMD;
				clear_errors  <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd26 }: begin // CMD26: PROGRAM_CID
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		{ 1'b?, 6'd27 }: begin // CMD27: PROGRAM_CSD
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		// { 1'b?, 6'd28 }: begin // CMD28: SET_WRITE_PROT
		// { 1'b?, 6'd29 }: begin // CMD29: CLR_WRITE_PROT
		{ 1'b?, 6'd30 }: begin // CMD30: SEND_WRITE_PROT
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_RECEIVE_DATA;
			end end
			// }}}
		{ 1'b?, 6'd31 }: begin // CMD31: SEND_WRITE_PROT_TYPE
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_RECEIVE_DATA;
			end end
			// }}}
		// { 1'b?, 6'd35 }: begin // CMD35: ERASE_GROUP_START
		// { 1'b?, 6'd36 }: begin // CMD36: ERASE_GROUP_END
		// { 1'b?, 6'd38 }: begin // CMD38: ERASE
		{ 1'b?, 6'd39 }: begin // CMD39: FAST_IO
				// {{{
				card_state <= EMMC_STANDBY;
				cfg_ppull <= 1'b0;
			end
			// }}}
		{ 1'b?, 6'd40 }: begin // CMD40: GO_IRQ_STATE
			// {{{
			if (card_state == EMMC_STANDBY)
			begin
				card_state <= EMMC_WAIT_IRQ;
				cfg_ppull <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd42 }: begin // CMD42: LOCK_UNLOCK
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		// { 1'b?, 6'd44 }: begin // CMD44: QUEUED_TASK_ADDRESS
		{ 1'b?, 6'd46 }: begin // CMD46: EXECUTE_READ_TASK
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_RECEIVE_DATA;
			end end
			// }}}
		{ 1'b?, 6'd47 }: begin // CMD47: EXECUTE_WRITE_TASK
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		// { 1'b?, 6'd48 }: begin // CMD48: CMDQ_TASK_MGMT
		{ 1'b?, 6'd49 }: begin // CMD49: SET_TIME
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		{ 1'b?, 6'd53 }: begin // CMD53: PROTOCOL_RD
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_RECEIVE_DATA;
			end end
			// }}}
		{ 1'b?, 6'd54 }: begin // CMD54: PROTOCOL_WR
			// {{{
			if (card_selected)
			begin
				card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		{ 1'b?, 6'd55 }: begin // CMD55: APP_CMD, ACMD to follow
			// {{{
			reply_valid <= #7 card_selected;
			reply <= 6'd55;
			reply_data <= { {(120-32){1'b0}}, 32'h0120 };
			cmd_alt <= 1'b1;
			end
			// }}}
		{ 1'b?, 6'd56 }: begin // CMD56: GEN_CMD
			// {{{
			if (card_selected)
			begin
				if (cmd_arg[0])
					card_state <= EMMC_RECEIVE_DATA;
				else
					card_state <= EMMC_PROGRAMMING;
			end end
			// }}}
		endcase
	end

	always @(posedge sd_clk)
	if (sd_cmd === 1'b0 && card_state == EMMC_WAIT_IRQ)
		card_state <= EMMC_STANDBY;

	always @(posedge sd_clk)
	if (reply_valid && !reply_busy)
		reply_valid <= 1'b0;

	always @(negedge busy_programming)
	if (card_state == EMMC_PROGRAMMING)
		card_state <= EMMC_TRANSFER;

	// [Program]: Read incoming data from host to chip
	// {{{
	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		pending_read <= 0;
		read_en <= 1'b0;
	end else if (!reply_busy && pending_read)
	begin
		pending_read <= 0;
		read_en <= 1'b1;
	end else if (read_en && (rx_good || rx_err))
	begin
		read_en <= 0;
		if (multi_block)
		begin
			repeat (10)
				@(posedge sd_clk);
			@(posedge sd_clk)
				read_en <= 1'b1;
		end
	end

	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		{ r_crcack, pending_ack } <= 0;
		{ r_crcnak, pending_nak } <= 0;
	end else begin
		{ r_crcack,pending_ack }<= { pending_ack, (read_en && rx_good)};
		{ r_crcnak,pending_nak }<= { pending_nak, (read_en && rx_err) };
	end

	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
		rx_addr <= 0;
	else if ((read_en || bustest_w) && rx_valid)
	begin
		mem_buf[rx_addr] <= rx_data;
		rx_addr <= rx_addr + 1;
	end else if (read_en && rx_good && !bustest_w)
	begin
$display("SETTING MEM[%08x] TO MEM-BUF[0] = %08x", (read_posn/4), mem_buf[0]);
		case(cfg_partition)
		3'h0: begin
			for(write_ik=0; write_ik<block_len/4; write_ik=write_ik+1)
				mem[write_ik+(read_posn/4)] = mem_buf[write_ik];
			end
		3'h1: begin
			for(write_ik=0; write_ik<block_len/4; write_ik=write_ik+1)
				boot_mem1[write_ik+(read_posn/4)] = mem_buf[write_ik];
			end
		3'h2: begin
			for(write_ik=0; write_ik<block_len/4; write_ik=write_ik+1)
				boot_mem1[write_ik+(read_posn/4)] = mem_buf[write_ik];
			end
		default:
			$display("ERROR: MEMORY PARTITION %1d NOT IMPLEMENTED", cfg_partition);
		endcase
		read_posn <= read_posn + block_len;
		rx_addr <= 0;
		if (!multi_block)
		begin
			busy_programming <= 1'b1;
			busy_programming <= #tPROG 1'b0;
		end
	end else if (read_en && rx_err)
	begin
		busy_programming <= 1'b0;
	end
	// }}}

	// Write data from chip to host
	// {{{
	initial	begin
		pending_write = 0;
		write_en = 1'b0;
		tx_valid = 1'b0;
		tx_data  = 0;
		tx_last  = 1'b0;
	end
		
	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		pending_write <= 1'b0;
		write_en <= 1'b0;
		tx_valid <= 1'b0;
		tx_addr <= 0;
		tx_last <= 0;
	end else if (boot_mode && (!boot_active || sd_cmd !== 1'b0
			|| (tx_valid && tx_last && read_posn >= BOOTSZ)))
	begin
$display("Exiting boot mode");
		boot_active <= 1'b0;
		boot_mode   <= 1'b0;

		// At the end of boot mode, exit
		pending_write <= 1'b0;
		multi_block   <= 1'b0;
		write_en <= 1'b0;
		tx_valid <= 1'b0;
		tx_addr <= 0;
		tx_last <= 0;
	end else if (cmd_valid && !cmd_crc_err && card_state != EMMC_IDLE
			&& { cmd_alt, cmd[5:0] } == 7'd12
			&& cmd_arg[31:16] == RCA)
	begin
		pending_write <= 1'b0;
		write_en <= (card_state == EMMC_SEND_DATA);
		tx_valid <= (card_state == EMMC_SEND_DATA);
		tx_data  <= 0;
		tx_last  <= 0;
	end else if (pending_write && !reply_valid && !reply_busy)
	begin
		// We can place a delay here, to simulate read access time
		// if desired ...
		// #50; @(posedge sd_clk) begin
			pending_write <= 1'b0;
			if (bustest_r)
			begin
				// Don't touch the buffer
			end else if (write_ext_csd) // reply == 6'd8)
			begin
				// {{{
				for(read_ik=0; read_ik<512/4;
							read_ik=read_ik+1)
				begin
					mem_buf[read_ik] = {
						ext_csd[4*(127-read_ik) + 3],
						ext_csd[4*(127-read_ik) + 2],
						ext_csd[4*(127-read_ik) + 1],
						ext_csd[4*(127-read_ik) + 0] };
				end
				read_posn <= read_posn + 512;
				// }}}
			end else if (cfg_partition == 3'h1)
			begin
				// Read from Boot partition #1
				for(read_ik=0; read_ik<block_len/4;
							read_ik=read_ik+1)
					mem_buf[read_ik]=boot_mem1[read_ik+(read_posn/4)];

				read_posn <= read_posn + block_len;
			end else if (cfg_partition == 3'h2)
			begin
				// Read from Boot partition #2
				for(read_ik=0; read_ik<block_len/4;
							read_ik=read_ik+1)
					mem_buf[read_ik]=boot_mem1[read_ik+(read_posn/4)];

				read_posn <= read_posn + block_len;
			end else begin
				// Read from device memory
				for(read_ik=0; read_ik<block_len/4;
							read_ik=read_ik+1)
					mem_buf[read_ik]=mem[read_ik+(read_posn/4)];

				read_posn <= read_posn + block_len;
			end
			write_en <= (card_state == EMMC_SEND_DATA);
			tx_valid <= (card_state == EMMC_SEND_DATA);
$display("TX-DATA set to %08x", mem_buf[0]);
			tx_data <= mem_buf[0];
			tx_addr <= 1;
			tx_last <= 0;
		// end
	end else if (write_en && tx_valid && tx_ready)
	begin
		if (tx_last)
		begin
			tx_valid <= 1'b0;
			// Need to keep write_en high until we're done
			// write_en <= 1'b0;
			write_ext_csd <= 1'b0;
			if (multi_block && card_state == EMMC_SEND_DATA)
				pending_write <= #WRITE_TIME 1'b1;
			else
				card_state <= EMMC_TRANSFER;
		end
		tx_data <= mem_buf[tx_addr];
		tx_addr <= tx_addr + 1;
		if (bustest_r)
		begin
			tx_last <= tx_last || (tx_addr + 1 >= rx_addr);
		end else if (write_ext_csd)
		begin
			tx_last <= tx_last || tx_addr == 128;
		end else begin
			tx_last <= tx_last
				|| (tx_addr == (block_len+3)/4-1);
		end
	end else if (write_en && !tx_valid && tx_ready)
	begin
		write_en <= 1'b0;
	end else if (!write_en)
	begin
		tx_valid <= 0;
		tx_data  <= 0;
		tx_last  <= 0;
	end

	// }}}

	// }}}

	reg		busy_indication;
	reg	[2:0]	busy_wait;

	always @(negedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		busy_indication <= 0;
		busy_wait <= 0;
	end else if (!busy_programming || read_en)
	begin
		busy_indication <= 1'b0;
		busy_wait <= 0;
	end else if (busy_wait > 0)
	begin
		busy_wait <= busy_wait - 1;
		busy_indication <= (busy_wait <= 1);
	end else if (!busy_indication)
	begin
		// busy_indication <= 1'b0;
		busy_wait <= 6;
	end

	assign	sd_dat[0] = (busy_indication && !read_en) ? 1'b0 : 1'bz;

	// assign	sd_ds = #DS_DELAY (ds_enabled
	assign	#DS_DELAY sd_ds = (ds_enabled
				&& (tx_ds || (enhanced_ds_enabled && cmd_ds)));

	reg	[31:0]	dbg_mem, dbg_mem_buf;

	always @(posedge sd_clk)
		dbg_mem = mem[32'h400];

	always @(posedge sd_clk)
		dbg_mem_buf = mem_buf[0];
endmodule
