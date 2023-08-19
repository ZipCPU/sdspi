////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	mdl_sdio.v
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
// Copyright (C) 2023, Gisselquist Technology, LLC
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
		parameter [0:0]	OPT_DUAL_VOLTAGE = 1'b0,
		parameter [0:0]	OPT_HIGH_CAPACITY = 1'b0,
		parameter	LGMEMSZ = 16,	// Log_2(Mem size in bytes)
		localparam	MEMSZ = (1<<LGMEMSZ)
	) (
		// {{{
		input	wire		sd_clk,
		inout	wire		sd_cmd,
		inout	wire	[3:0]	sd_dat
		// , output	wire		sd_ds
		// }}}
	);

	// Local declarations
	// {{{
	reg		cfg_ddr;
	reg	[1:0]	cfg_width;
	wire	[3:0]	ign_dat;

	wire		cmd_valid, cmd_ds;
	wire	[5:0]	cmd;
	wire	[31:0]	cmd_arg;

	reg		reply_valid, reply_type, reply_crc;
	wire		reply_busy;
	reg	[5:0]	reply;
	reg	[119:0]	reply_data;

	reg	[119:0]	CID;
	reg	[31:0]	ocr;
	reg		power_up_busy, cmd_alt, cmd8_sent, card_reset,
			host_supports_hcs, r_1p8v_request, r_1p8v;
	reg	[15:0]	RCA;
	reg	[31:0]	R1;
	reg		drive_cmd, card_selected;
	wire		cmd_collision;

	reg		read_en, pending_read;
	wire		rx_valid, rx_last, rx_good, rx_err;
	wire	[31:0]	rx_data;

	integer	read_ik;
	reg		write_en, tx_valid, tx_last, pending_write;
	reg	[31:0]	tx_data;
	wire		tx_ready, tx_ds;
	reg	[6:0]	tx_addr;

	reg	[31:0]	mem_buf	[0:127];
	reg	[6:0]	rx_addr;

	reg	[31:0]	mem	[0:(MEMSZ/4)-1];
	reg	[LGMEMSZ-1:0]	read_posn;
	integer		write_ik;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Command wire handler
	// {{{

	mdl_sdcmd
	tb_sdcmd (
		// {{{
		.sd_clk(sd_clk), .sd_cmd(sd_cmd),
			.sd_ds(cmd_ds),
		//
		.o_cmd_valid(cmd_valid), .o_cmd(cmd), .o_arg(cmd_arg),
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

	mdl_sdtx
	tb_sdtx (
		// {{{
		.sd_clk(sd_clk), .sd_dat({ ign_dat, sd_dat }),
			.sd_ds(tx_ds),
		//
		.i_en(write_en), .i_width(cfg_width), .i_ddr(cfg_ddr),
		//
		.i_valid(tx_valid), .o_ready(tx_ready),
			.i_data(tx_data), .i_last(tx_last)
		// }}}
	);

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
	initial	r_1p8v = 1'b0;
	initial	RCA = 16'h0;
	initial	R1  = 32'h0;
	initial	drive_cmd = 1'b0;
	initial	card_selected = 1'b1;
	initial	cfg_width = 2'b0;	// 1b width
	initial	cfg_ddr = 1'b0;		// SDR
	initial	read_en = 1'b0;

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
	if (cmd_valid)
	begin
		cmd_alt <= 1'b0;
		reply_crc <= 1'b1;	// All replies get CRCs by default
		reply_type <= 1'b0;
		card_reset <= 0;
		r_1p8v_request <= 1'b0;
		casez({ cmd_alt, cmd[5:0] })
		// AMDs
		{ 1'b1, 6'd6 }: begin
			// {{{
			if (card_selected)
			begin
				cfg_width[0] <= (cmd_arg[1:0] == 2'b10);

				reply_valid <= #7 1'b1;
				reply <= 6'd6;
				reply_data <= { {(120-32){1'b0}}, R1};
			end end
			// }}}
		{ 1'b1, 6'd41 }: begin
			// {{{
			// assert(cmd8_sent);
			host_supports_hcs <= cmd_arg[30] && host_supports_hcs;
			reply_valid <= #7 1'b1;
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
				reply_valid <= #7 1'b1;
				reply_type <= 1'b1;
				reply <= 6'd2;
				reply_data <= CID;
				drive_cmd <= 1'b0;
			end end
			// }}}
		{ 1'b?, 6'd3 }: begin // CMD3: SEND_RELATIVE_ADDR
			// {{{
			if (!cmd_collision)
			begin
				do begin
					RCA = $random;
				end while(RCA == 16'h0);

				reply_valid <= #7 1'b1;
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
			reply_valid <= #7 (cmd_arg[31:16] == RCA);
			reply <= 6'd2;
			reply_data <= { {(120-32){1'b0}}, R1};
			end
			// }}}
		{ 1'b?, 6'd8 }: begin // CMD8: SEND_IF_COND
			// {{{
			reply_valid <= #7 1'b1;
			reply <= 6'd8;
			assert(cmd_arg[31:12]==20'h0);
			assert(cmd_arg[11:8]==4'h1);
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
		{ 1'b?, 6'd11 }: begin // CMD11: Voltage switch
			// {{{
			if (r_1p8v_request && OPT_DUAL_VOLTAGE && !r_1p8v && !power_up_busy)
			begin
				reply_valid <= #7 1'b1;
				reply <= 6'd11;
				reply_data <= { {(120-32){1'b0}}, 32'h0 };
				cmd_alt <= 1'b1;
			end end
			// }}}
		{ 1'b?, 6'd17 }: begin // CMD19: Read block
			// {{{
			if (card_selected)
			begin
				pending_write <= 1'b1;
				// write_en <= 1'b1;
				reply_valid <= #7 1'b1;
				reply <= 6'd19;
				reply_data <= { {(120-32){1'b0}}, R1};

				read_posn <= cmd_arg;
				rx_addr <= 0;
			end end
			// }}}
		{ 1'b?, 6'd24 }: begin // CMD24: Write block
			// {{{
			if (card_selected)
			begin
				reply_valid <= #7 1'b1;
				reply <= 6'd24;
				reply_data <= { {(120-32){1'b0}}, R1};

				pending_read <= 1'b1;
				read_posn <= cmd_arg;
				rx_addr <= 0;
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
		endcase
	end

	always @(posedge sd_clk)
	if (reply_valid && !reply_busy)
		reply_valid <= 1'b0;

	always @(posedge sd_clk)
	if (!reply_busy && pending_read)
	begin
		pending_read <= 0;
		read_en <= 1'b1;
	end else if (read_en && (rx_good || rx_err))
		read_en <= 0;
		// read_posn <= cmd_arg;

	always @(posedge sd_clk)
	if (read_en && rx_valid)
	begin
		mem_buf[rx_addr] <= rx_data;
		rx_addr <= rx_addr + 1;
	end else if (read_en && rx_good)
	begin
		for(write_ik=0; write_ik<512/4; write_ik=write_ik+1)
			mem[write_ik+read_posn] = mem_buf[write_ik];
	end

	initial	begin
		pending_write = 0;
		write_en = 1'b0;
		tx_valid = 1'b0;
		tx_data  = 0;
		tx_last  = 1'b0;
	end
		
	always @(posedge sd_clk)
	if (pending_write && !reply_valid && !reply_busy)
	begin
		// We can place a delay here, to simulate read access time
		// if desired ...
		// #50; @(posedge sd_clk) begin
			pending_write <= 1'b0;
			write_en <= 1'b1;
			for(read_ik=0; read_ik<512/4; read_ik=read_ik+1)
				mem_buf[read_ik] = mem[read_ik+read_posn];
			tx_valid <= 1'b1;
			tx_data <= mem_buf[0];
			tx_addr <= 1;
			tx_last <= 0;
		// end
	end else if (write_en && tx_valid && tx_ready)
	begin
		if (tx_last)
		begin
			tx_valid <= 1'b0;
			write_en <= 1'b0;
		end
		tx_data <= mem_buf[tx_addr];
		tx_addr <= tx_addr + 1;
		tx_last <= tx_last || (&tx_addr[6:0]);
	end else if (!write_en)
	begin
		tx_valid <= 0;
		tx_data  <= 0;
		tx_last  <= 0;
	end

	// }}}

	// assign	sd_ds = #DS_DELAY (ds_enabled
	//			&& (tx_ds || (enhanced_ds_enabled && cmd_ds));
endmodule
