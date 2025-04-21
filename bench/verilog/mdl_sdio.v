////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/mdl_sdio.v
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
`timescale 1ns/1ps
// }}}
module	mdl_sdio #(
		// {{{
		parameter [0:0]	OPT_DUAL_VOLTAGE = 1'b0,
		parameter [0:0]	OPT_HIGH_CAPACITY = 1'b0,
		parameter	LGMEMSZ = 16,	// Log_2(Mem size in bytes)
		localparam	MEMSZ = (1<<LGMEMSZ)
		// }}}
	) (
		// {{{
		input	wire		sd_clk,
		inout	wire		sd_cmd,
		inout	wire	[3:0]	sd_dat,
		input	wire		i_1p8v	// Set if source is 1.8V
		// , output	wire		sd_ds
		// }}}
	);

	// Local declarations
	// {{{
	// Maximum allowable busy time is 250ms.
	// localparam realtime	WRITE_TIME = 250_000_000.0;
	// A 250ms timeout is unreasonable for simulations, however, so we'll
	// drop it to 1250ns or so.  This should at least force a busy of a
	// single clock cycle in 1MHz mode.
	localparam realtime	WRITE_TIME = 1250.0;
	// I have no idea what sort of read time is appropriate.  Therefore,
	// the following is nothing more than a guess.
	localparam realtime	READ_TIME = 400.0;
	// I have even less of an idea of how much time should be expected
	// before sending a tuning response
	localparam realtime	TUNING_TIME = 20.0;
	// REPLY_DELAY is the time from recognizing a command until a reply
	// is requested.  This time should come from the SDIO spec, but my
	// copy is the simplified spec that doesn't contain this value.  Too
	// short, and the SERDES tristate can't clear fast enough.
	localparam realtime	REPLY_DELAY = 7.0;

	reg		cfg_ddr;
	reg	[1:0]	cfg_width;
	wire	[3:0]	ign_dat;

	wire		cmd_valid, cmd_ds, cmd_crc_err;
	wire	[5:0]	cmd;
	wire	[31:0]	cmd_arg;

	reg		reply_valid, reply_type, reply_crc;
	wire		reply_busy;
	reg	[5:0]	reply;
	reg	[119:0]	reply_data;

	reg	[119:0]	CID;
	reg	[31:0]	ocr;
	reg		power_up_busy, cmd_alt, cmd8_sent, card_reset,
			host_supports_hcs, r_1p8v_request, permit_1p8v,set_1p8v;
	reg	[15:0]	RCA;
	reg	[31:0]	R1;
	reg		drive_cmd, card_selected;
	wire		cmd_collision;

	reg		read_en, pending_read, multi_block,
			stop_transmission;
	wire		rx_valid, rx_last, rx_good, rx_err;
	wire	[31:0]	rx_data;

	integer	read_ik;
	reg		write_en, tx_valid, tx_last, pending_write;
	reg	[31:0]	tx_data;
	wire		tx_ready, tx_ds;
	reg	[6:0]	tx_addr;

	reg	[31:0]	mem_buf	[0:127];
	reg	[6:0]	rx_addr;

	reg	[LGMEMSZ-1:0]	read_posn;
	integer		write_ik;

	reg		card_busy, internal_card_busy;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Command wire handler
	// {{{

	mdl_sdcmd
	tb_sdcmd (
		// {{{
		.rst_n(1'b1),		// Not used in SDIO mode
		.sd_clk(sd_clk), .sd_cmd(sd_cmd),
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
	// Read data from the user (write operation)
	// {{{
	mdl_sdrx
	tb_sdrx (
		// {{{
		.rst_n(1'b1),		// Not used in SDIO mode
		.sd_clk(sd_clk), .sd_dat({ ign_dat, sd_dat }),
		//
		.i_rx_en(read_en), .i_width(cfg_width), .i_ddr(cfg_ddr),
		.i_len(16'd512),
		//
		.o_valid(rx_valid), .o_data(rx_data), .o_last(rx_last),
			.o_good(rx_good), .o_err(rx_err)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Read from the card, send to the user (read operation)
	// {{{
	reg	r_crcack, r_crcnak;

	mdl_sdtx
	tb_sdtx (
		// {{{
		.rst_n(1'b1),		// Not used in SDIO mode
		.sd_clk(sd_clk), .sd_dat({ ign_dat, sd_dat }),
			.sd_ds(tx_ds),
		//
		.i_en(write_en), .i_width(cfg_width), .i_ddr(cfg_ddr),
			.i_ppull(1'b0),
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
	// SDIO MEM
	// {{{

	reg	[31:0]	mem	[0:(MEMSZ/4)-1];
	integer	mem_fileid, mem_code;

	/*
	initial	if (MEMFILE != 0)
	begin
		mem_fileid = $fopen(MEMFILE, "rb");
		mem_code = $fread(mem, mem_fileid);
		$fclose(mem_fileid);
	end
	*/

	task mem_blkwrite(input [31:0] posn);
		// {{{
		integer	write_ik;
	begin
		for(write_ik=0; write_ik<512/4; write_ik=write_ik+1)
			mem[write_ik+posn] = mem_buf[write_ik];
	end endtask
	// }}}

	task mem_blkread(input [31:0] posn);
		// {{{
		integer	read_ik;
	begin
		for(read_ik=0; read_ik<512/4; read_ik=read_ik+1)
			mem_buf[read_ik] = mem[read_ik+posn];
	end endtask
	// }}}

/*
	generate if (MEMFILE == 0)
	begin : NO_MEMFILE

	end else begin : USE_MEMFILE
		integer	mem_fileid;

		initial	mem_fileid = $fope(MEMFILE, "rb+");

		task mem_blkwrite(input [31:0] posn);
			// {{{
			integer	read_ik;
			reg	[31:0]	word;
		begin
			$fseek(mem_fileid, posn, 1);
			for(read_ik=0; read_ik<512/4; read_ik=read_ik+1)
			begin
				word[31:24] = $fgetc(mem_fileid);
				word[23:16] = $fgetc(mem_fileid);
				word[15: 8] = $fgetc(mem_fileid);
				word[ 7: 0] = $fgetc(mem_fileid);
				mem_buf[read_ik] = word;
			end
		end endtask
		// }}}

		task mem_blkread(input [31:0] posn);
			// {{{
			integer	read_ik;
			reg	[31:0]	word;
		begin
			$fseek(mem_fileid, posn, 1);
			for(read_ik=0; read_ik<512/4; read_ik=read_ik+1)
			begin
				word[31:24] = $fgetc(mem_fileid);
				word[23:16] = $fgetc(mem_fileid);
				word[15: 8] = $fgetc(mem_fileid);
				word[ 7: 0] = $fgetc(mem_fileid);
				mem_buf[read_ik] = word;
			end
		end endtask
		// }}}

	end endgenerate
*/
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDIO FSM
	// {{{

	initial	ocr = 32'h80ff_8000;
	initial	cmd_alt = 1'b0;
	initial	power_up_busy = 1'b1;
	initial	cmd8_sent = 1'b0;
	initial	host_supports_hcs = OPT_HIGH_CAPACITY;
	initial	r_1p8v_request = 1'b0;
	initial	permit_1p8v = 1'b0;
	initial	RCA = 16'h0;
	initial	R1  = 32'h0;
	initial	drive_cmd = 1'b0;
	initial	card_selected = 1'b1;
	initial	cfg_width = 2'b0;	// 1b width
	initial	cfg_ddr = 1'b0;		// SDR
	initial	read_en = 1'b0;
	initial	stop_transmission = 1'b0;
	initial	multi_block = 1'b0;

	initial	begin
		reply_valid = 1'b0;
		reply_type  = 1'b0;
		reply = 6'h0;
		reply_data = {(120){1'b0}};

		CID[119:112] = $random;		// MFGR ID
		CID[111: 96] = "GT";		// OED/ Application ID
		CID[ 95: 56] = "SM-MODEL";	// Product name
		CID[ 55: 48] = $random;		// Product revision
		CID[ 47: 16] = $random;	// Serial number
		CID[ 15: 12] = 4'h0; // Reserved
		CID[ 11:  0] = $random; // Manufacturing code
		// CRC not included in our version
	end

	always @(*)
		ocr[31] = !power_up_busy;
	always @(*)
		ocr[30] = cmd8_sent && host_supports_hcs && OPT_HIGH_CAPACITY;

	always @(posedge sd_clk)
	if (cmd_valid && !cmd_crc_err)
	begin
		cmd_alt <= 1'b0;
		reply_crc <= 1'b1;	// All replies get CRCs by default
		reply_type <= 1'b0;
		card_reset <= 0;
		casez({ cmd_alt, cmd[5:0] })
		// AMDs
		{ 1'b1, 6'd6 }: begin
			// {{{
			if (card_selected)
			begin
				cfg_width[0] <= (cmd_arg[1:0] == 2'b10);

				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd6;
				reply_data <= { {(120-32){1'b0}}, R1};
			end end
			// }}}
		{ 1'b1, 6'd41 }: begin
			// {{{
			// assert(cmd8_sent);
			host_supports_hcs <= cmd_arg[30] && host_supports_hcs;
			reply_valid <= #REPLY_DELAY 1'b1;
			reply <= 6'b111111;
			r_1p8v_request <= (OPT_DUAL_VOLTAGE && cmd_arg[24] && !power_up_busy);
			reply_data <= { {(120-32){1'b0}}, ocr[31],
				((cmd8_sent && cmd_arg[30]
					&& host_supports_hcs)
						&& OPT_HIGH_CAPACITY),
				ocr[29:25],
				(OPT_DUAL_VOLTAGE && cmd_arg[24] && !power_up_busy),
				ocr[23:8], 8'h0 };
			reply_crc  <= 1'b0;
			if (0 == (ocr[23:8] & cmd_arg[23:8]))
			begin
				reply_data[31] <= 1'b0;
				power_up_busy <= 1;
			end else begin
				// 5 command/reply cycles at 100kHz
				// power_up_busy <= #(5*2*48*10000) 1'b0;
				// or ... 5 command/reply cycles at 1MHz
				power_up_busy <= #(5*2*48*1000) 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd0 }: begin // CMD0: Go idle
			// {{{
			reply_valid <= 1'b0;
			// ocr[31] <= power_up_busy;
			ocr[7]  <= 1'b0;
			cmd8_sent <= 1'b0;
			card_reset <= 1;

			pending_read <= 1'b0;
			read_en <= 1'b0;

			pending_write <= 1'b0;
			write_en <= 1'b0;

			RCA <= 16'h0;
			drive_cmd <= 1'b0;
			card_selected <= 1'b1;
			cfg_width <= 2'b0;
			end
			// }}}
		{ 1'b?, 6'd2 }: begin // CMD2: ALL_SEND_CID
			// {{{
			if (card_selected && !power_up_busy)
			begin
				reply_valid <= #REPLY_DELAY 1'b1;
				reply_type <= 1'b1;
				reply <= 6'd63;
				reply_data <= CID;
				drive_cmd <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd3 }: begin // CMD3: SEND_RELATIVE_ADDR
			// {{{
			if (!cmd_collision)
			begin
				RCA = $random;
				while(RCA == 16'h0)
					RCA = $random;

				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd3;
				reply_data <= { {(120-32){1'b0}}, RCA, 16'h0 };
				drive_cmd <= 1'b1;

				card_selected <= 1'b0;
			end end
			// }}}
		// { 1'b?, 6'd6 }: begin // CMD6: SWITCH_FUNCTION (+/- DDR, etc)
		{ 1'b?, 6'd7 }: begin // CMD7: SELECT_DESELECT_CARD
			// {{{
			card_selected <= (cmd_arg[31:16] == RCA);
			reply_valid <= #REPLY_DELAY (cmd_arg[31:16] == RCA);
			reply <= 6'd7;
			reply_data <= { {(120-32){1'b0}}, R1};
			end
			// }}}
		{ 1'b?, 6'd8 }: begin // CMD8: SEND_IF_COND
			// {{{
			reply_valid <= #REPLY_DELAY 1'b1;
			reply <= 6'd8;
			// assert(cmd_arg[31:12]==20'h0);
			// assert(cmd_arg[11:8]==4'h1);
			reply_data <= { {(120-32){1'b0}}, cmd_arg };

			ocr[7] <= 1'b0;
			if (!cmd8_sent)
			begin
				if (OPT_HIGH_CAPACITY)
					ocr[30] <= 1'b1;
				if (OPT_DUAL_VOLTAGE)
					ocr[7] <= 1'b1;
				cmd8_sent <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd10 }: begin // CMD10: SEND_CID
			// {{{
			if (cmd_arg[31:16] == RCA)
			begin
				reply_valid <= #REPLY_DELAY 1'b1;
				reply_type <= 1'b1;
				reply <= 6'd10;
				reply_data <= CID;
				drive_cmd <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd11 }: begin // CMD11: Voltage switch
			// {{{
			if (r_1p8v_request && OPT_DUAL_VOLTAGE && !power_up_busy)
			begin
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd11;
				reply_data <= { {(120-32){1'b0}}, R1 };
				if (!OPT_DUAL_VOLTAGE || !r_1p8v_request || i_1p8v)
				begin
					// Illegal command
					reply_data[11] <= 1'b1;
				end else
					permit_1p8v <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd12 }: begin // CMD12: STOP_TRANSMISSION
			// {{{
			if (card_selected)
			begin
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd12;
				reply_data <= { {(120-32){1'b0}}, R1 };
				stop_transmission <= 1'b1;
				multi_block <= 1'b0;
				pending_read <= 1'b0;
				pending_write <= 1'b0;
				write_en <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd17 }: begin // CMD17: Read block
			// {{{
			if (card_selected)
			begin
				pending_write <= #READ_TIME 1'b1;
				multi_block <= 1'b0;
				// write_en <= 1'b1;
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd17;
				reply_data <= { {(120-32){1'b0}}, R1};

				if (host_supports_hcs)
					read_posn <= cmd_arg << 7;
				else
					read_posn <= cmd_arg >> 2;
				rx_addr <= 0;
				stop_transmission <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd18 }: begin // CMD18: Read multiple
			// {{{
			if (card_selected)
			begin
				pending_write <= #READ_TIME 1'b1;
				multi_block <= 1'b1;
				// write_en <= 1'b1;
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd18;
				reply_data <= { {(120-32){1'b0}}, R1};

				if (host_supports_hcs)
					read_posn <= cmd_arg << 7;
				else
					read_posn <= cmd_arg >> 2;
				rx_addr <= 0;
				stop_transmission <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd19 }: begin // CMD19: Send tuning block
			// {{{
			if (card_selected)
			begin
				pending_write <= #TUNING_TIME 1'b1;
				multi_block <= 1'b0;
				// write_en <= 1'b1;
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd19;
				reply_data <= { {(120-32){1'b0}}, R1};

				// read_posn <= 0;
				// rx_addr <= 0;
				stop_transmission <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd24 }: begin // CMD24: Write block
			// {{{
			if (card_selected)
			begin
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd24;
				reply_data <= { {(120-32){1'b0}}, R1};

				pending_read <= 1'b1;
				multi_block <= 1'b0;
				stop_transmission <= 1'b0;
				if (host_supports_hcs)
					read_posn <= cmd_arg << 7;
				else
					read_posn <= cmd_arg >> 2;
				rx_addr <= 0;
			end end
			// }}}
		{ 1'b?, 6'd25 }: begin // CMD25: Write multiple
			// {{{
			if (card_selected)
			begin
				reply_valid <= #REPLY_DELAY 1'b1;
				reply <= 6'd25;
				reply_data <= { {(120-32){1'b0}}, R1};

				pending_read <= 1'b1;
				multi_block <= 1'b1;
				stop_transmission <= 1'b0;
				if (host_supports_hcs)
					read_posn <= cmd_arg << 7;
				else
					read_posn <= cmd_arg >> 2;
				rx_addr <= 0;
			end end
			// }}}
		{ 1'b?, 6'd55 }: begin // CMD55: APP_CMD, ACMD to follow
			// {{{
			reply_valid <= #REPLY_DELAY card_selected;
			reply <= 6'd55;
			reply_data <= { {(120-32){1'b0}}, 32'h0120 };
			cmd_alt <= 1'b1;
			end
			// }}}
		endcase
	end

	always @(posedge sd_clk)
	if (reply_valid && !reply_busy)
		reply_valid <= 1'b0;

	////////////////////////////////////////////////////////////////////////
	//
	// 1.8V transition checking
	// {{{
	initial begin
		// Allow the simulation a moment to get settled, to help
		// guarantee we're not looking at the transition of 1p8v
		// from 'x to a valid value.
		//
		// set_1p8v == a 1.8v edge will be valid
		set_1p8v = 1'b0;
		#1;
		wait(!i_1p8v);
		set_1p8v = 1'b1;
	end

	always @(posedge i_1p8v)
	if (set_1p8v)
	begin
		// Now, on a valid transition to 1.8V, we must've been told
		// previously that we were going to do this.
		assert(OPT_DUAL_VOLTAGE);
		assert(r_1p8v_request);
		assert(permit_1p8v);
		assert(!reply_valid);
		// The reply *might* be busy when we activate 1.8V--it might
		// be sending a stop bit.  Therefore, we cannot test for this
		// here.
		// assert(!reply_busy);
		assert(sd_cmd !== 1'b0);
	end

	// It is illegal to ever change from 1.8V back to 3.3V while the card
	// is inserted.
	always @(negedge i_1p8v)
	begin
		// assert(!set_1p8v) else $display("NEGEDGE ASSERTION FAIL at %t", $time);
		if (!set_1p8v) begin end else begin
			$display("NEGEDGE ASSERTION FAIL at %t", $time);
			assert(!set_1p8v);
		end
	end

	// Following a request to switch to 1p8v, we *must* switch to 1p8v.
	always @(posedge sd_clk)
	if (cmd_valid && permit_1p8v)
		assert(i_1p8v);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Write block (host to SD) handling
	// {{{

	always @(posedge sd_clk)
	if (stop_transmission && !cmd_valid)
	begin
		pending_read <= 0;
		read_en <= 0;
	end else if (!reply_busy && pending_read)
	begin
		if (!internal_card_busy)
		begin
			pending_read <= 0;
			read_en <= !stop_transmission && (cmd[5:0] == 6'd24
					|| cmd[5:0] == 6'd25);
		end
	end else if (read_en && (rx_good || rx_err))
	begin
		read_en <= 0;
		if (multi_block && !stop_transmission)
		begin
			internal_card_busy <= 1'b1;
			pending_read <= 1'b1;
			internal_card_busy <= #READ_TIME 1'b0;
		end
	end

	initial	{ r_crcack, r_crcnak } = 2'b0;
	always @(posedge sd_clk)
	begin
		r_crcack <= read_en && rx_good;
		r_crcnak <= read_en && rx_err;
	end

	always @(posedge sd_clk)
	if (read_en && rx_valid)
	begin
		mem_buf[rx_addr] <= rx_data;
		rx_addr <= rx_addr + 1;
	end else if (read_en && rx_good)
	begin
		// mem_blkwrite(read_posn);
		for(write_ik=0; write_ik<512/4; write_ik=write_ik+1)
			mem[write_ik+read_posn] = mem_buf[write_ik];
		read_posn = read_posn + 512/4;
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Read block (SD to host) handling
	// {{{
	initial	begin
		pending_write = 0;
		write_en = 1'b0;
		tx_valid = 1'b0;
		tx_data  = 0;
		tx_last  = 1'b0;
	end

	initial	internal_card_busy = 1'b0;
	always @(posedge sd_clk)
	if (cmd_valid && !cmd_crc_err && { cmd_alt, cmd[5:0] } == 7'd12
						&& card_selected)
	begin // STOP_TRANSMISSION
		// {{{
		write_en <= 1'b0;
		tx_valid <= 1'b0;
		tx_addr <= 0;
		tx_last <= 0;
		// pending_write <= 1'b0;
		// }}}
	end else if (pending_write && !reply_valid && !reply_busy)
	begin
		// {{{
		// We can place a delay here, to simulate read access time
		// if desired ...
		if (!internal_card_busy && !stop_transmission)
		begin
			pending_write <= 1'b0;
			if (cmd[5:0] == 6'd17 || cmd[5:0] == 6'd18)
			begin
				write_en <= 1'b1;
				mem_blkread(read_posn);
				read_posn = read_posn + 512/4;
				tx_valid <= 1'b1;
				tx_data <= mem_buf[0];
				tx_addr <= 1;
				tx_last <= 0;
			end else if (cmd[5:0] == 6'd19)
			begin	// SEND_TUNING_BLOCK
				//
				write_en <= 1'b1;
				tx_valid <= 1'b1;
				mem_buf[112+00] <= 32'hFF0FFF00;
				mem_buf[112+01] <= 32'hFFCCC3CC;
				mem_buf[112+02] <= 32'hC33CCCFF;
				mem_buf[112+03] <= 32'hFEFFFEEF;
				mem_buf[112+04] <= 32'hFFDFFFDD;
				mem_buf[112+05] <= 32'hFFFBFFFB;
				mem_buf[112+06] <= 32'hBFFF7FFF;
				mem_buf[112+07] <= 32'h77F7BDEF;
				mem_buf[112+08] <= 32'hFFF0FFF0;
				mem_buf[112+09] <= 32'h0FFCCC3C;
				mem_buf[112+10] <= 32'hCC33CCCF;
				mem_buf[112+11] <= 32'hFFEFFFEE;
				mem_buf[112+12] <= 32'hFFFDFFFD;
				mem_buf[112+13] <= 32'hDFFFBFFF;
				mem_buf[112+14] <= 32'hBBFFF7FF;
				mem_buf[112+15] <= 32'hF77F7BDE; // End of 512B
				//
				tx_data <= 32'hFF0FFF00;
				tx_addr <= 113;	// So TX_LAST is at 512Bytes
				tx_last <= 0;
			end
		end
		// }}}
	end else if (write_en && tx_valid && tx_ready)
	begin
		tx_data <= mem_buf[tx_addr];
		if (tx_last)
		begin
			tx_valid <= 1'b0;
			tx_addr  <= 0;
			// write_en <= 1'b0;
		end else
			tx_addr <= tx_addr + 1;
		tx_last <= tx_last || (&tx_addr[6:0]);
	end else if (write_en && !tx_valid)
	begin
		if (tx_ready)
		begin
			write_en <= 1'b0;
			if (cmd[5:0] !== 6'd19)
			begin
				// On all reads from the card, we
				// get busy again once complete.
				// 6'd19 is a tuning block, not a card read,
				// so we don't need to get busy when done with
				// it.
				internal_card_busy <= 1'b1;
				internal_card_busy <= #READ_TIME 1'b0;
				if (multi_block)
					pending_write <= 1'b1;
			end
		end
	end else if (!write_en)
	begin
		tx_valid <= 0;
		tx_data  <= 0;
		tx_last  <= 0;
	end
	// }}}

	// }}}

	// Card busy can only change on a clock, so let's make sure we only
	// change it on a clock.
	initial	card_busy <= 1'b0;
	always @(posedge sd_clk)
		card_busy <= internal_card_busy;

	// assign	sd_ds = #DS_DELAY (ds_enabled
	//			&& (tx_ds || (enhanced_ds_enabled && cmd_ds));

	// assign	sd_dat[0] = (card_busy && card_selected && !pending_write) ? 1'b0 : 1'bz;
	reg		busy_indication;
	reg	[2:0]	busy_wait;

	initial	busy_indication = 0;
	initial	busy_wait = 0;
	always @(negedge sd_clk)
	if (!internal_card_busy || write_en || pending_write)
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

endmodule
