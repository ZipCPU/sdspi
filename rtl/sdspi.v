////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdspi.v
//
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2016, Gisselquist Technology, LLC
//
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of  the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`define	SDSPI_CMD_ADDRESS	2'h0
`define	SDSPI_DAT_ADDRESS	2'h1
`define	SDSPI_FIFO_A_ADDR	2'h2
`define	SDSPI_FIFO_B_ADDR	2'h3
//
// `define	SDSPI_CMD_ID	3'h0
// `define	SDSPI_CMD_A1	3'h1
// `define	SDSPI_CMD_A2	3'h2
// `define	SDSPI_CMD_A3	3'h3
// `define	SDSPI_CMD_A4	3'h4
// `define	SDSPI_CMD_CRC	3'h5
// `define	SDSPI_CMD_FIFO	3'h6
// `define	SDSPI_CMD_WAIT	3'h7
//
`define	SDSPI_EXPECT_R1		2'b00
`define	SDSPI_EXPECT_R1B	2'b01
`define	SDSPI_EXPECT_R3		2'b10
//
`define	SDSPI_RSP_NONE		3'h0	// No response yet from device
`define	SDSPI_RSP_BSYWAIT	3'h1	// R1b, wait for device to send nonzero
`define	SDSPI_RSP_GETWORD	3'h2	// Get 32-bit data word from device
`define	SDSPI_RSP_GETTOKEN	3'h4 // Write to device, read from FIFO, wait for completion token
`define	SDSPI_RSP_WAIT_WHILE_BUSY		3'h5 // Read from device
`define	SDSPI_RSP_RDCOMPLETE	3'h6
`define	SDSPI_RSP_WRITING	3'h7 // Read from device, write into FIFO
//
module	sdspi(i_clk,
		// Wishbone interface
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data,
			o_wb_ack, o_wb_stall, o_wb_data,
		// SDCard interface
		o_cs_n, o_sck, o_mosi, i_miso,
		// Our interrupt
		o_int,
		// And whether or not we own the bus
		i_bus_grant,
		// And some wires for debugging it all
		o_debug);
	parameter	LGFIFOLN = 7;
	input	i_clk;
	//
	input			i_wb_cyc, i_wb_stb, i_wb_we;
	input		[1:0]	i_wb_addr;
	input		[31:0]	i_wb_data;
	output	reg		o_wb_ack;
	output	wire		o_wb_stall;
	output	reg	[31:0]	o_wb_data;
	//
	output	wire		o_cs_n, o_sck, o_mosi;
	input			i_miso;
	// The interrupt
	output	reg		o_int;
	// .. and whether or not we can use the SPI port
	input			i_bus_grant;
	//
	output	wire	[31:0]	o_debug;

	//
	// Some WB simplifications:
	//
	reg	r_cmd_busy;
	wire	wb_stb, write_stb, cmd_stb; // read_stb
	assign	wb_stb    = ((i_wb_cyc)&&(i_wb_stb)&&(~o_wb_stall));
	assign	write_stb = ((wb_stb)&&( i_wb_we));
	// assign	read_stb  = ((wb_stb)&&(~i_wb_we));
	assign	cmd_stb  = (~r_cmd_busy)&&(write_stb)
				&&(i_wb_addr==`SDSPI_CMD_ADDRESS);


	//
	// Access to our lower-level SDSPI driver, the one that actually
	// uses/sets the SPI ports
	//
	reg	[6:0]	r_sdspi_clk;
	reg		ll_cmd_stb;
	reg	[7:0]	ll_cmd_dat;
	wire		ll_out_stb, ll_idle;
	wire	[7:0]	ll_out_dat;
	llsdspi	lowlevel(i_clk, r_sdspi_clk, r_cmd_busy, ll_cmd_stb, ll_cmd_dat,
			o_cs_n, o_sck, o_mosi, i_miso,
			ll_out_stb, ll_out_dat, ll_idle,
			i_bus_grant);


	// CRC
	reg		r_cmd_crc_stb;
	wire	[7:0]	cmd_crc;
	// State machine
	reg	[2:0]	r_cmd_state, r_rsp_state;
	// Current status, beyond state
	reg		r_have_resp, r_use_fifo, r_fifo_wr,
				ll_fifo_rd_complete, ll_fifo_wr_complete,
				r_fifo_id,
				ll_fifo_wr, ll_fifo_rd,
				r_have_data_response_token, 
				r_have_start_token,
				r_err_token;
	reg	[3:0]	r_read_err_token;
	reg	[1:0]	r_data_response_token;
	reg	[7:0]	fifo_byte;
	reg	[7:0]	r_last_r_one;
	//
	reg	[31:0]	r_data_reg;
	reg	[1:0]	r_data_fil, r_cmd_resp;
	//
	//
	wire		need_reset;
	reg		r_cmd_err;
	reg		r_cmd_sent;
	reg	[31:0]	fifo_a_reg, fifo_b_reg;
	//
	reg		q_busy;
	//
	reg	[7:0]	fifo_a_mem[((1<<(LGFIFOLN+2))-1):0];
	reg	[7:0]	fifo_b_mem[((1<<(LGFIFOLN+2))-1):0];
	reg	[(LGFIFOLN-1):0]	fifo_wb_addr;
	reg	[(LGFIFOLN+1):0]	rd_fifo_sd_addr;
	reg	[(LGFIFOLN+1):0]	wr_fifo_sd_addr;
	//
	reg	[(LGFIFOLN+1):0]	ll_fifo_addr;
	//
	reg		fifo_crc_err;
	reg	[1:0]	ll_fifo_wr_state;
	reg	[7:0]	fifo_a_byte, fifo_b_byte;
	//
	reg	[2:0]	ll_fifo_pkt_state;
	reg		fifo_rd_crc_stb, fifo_wr_crc_stb;
	//
	reg	[3:0]	fifo_rd_crc_count, fifo_wr_crc_count;
	reg	[15:0]	fifo_rd_crc_reg, fifo_wr_crc_reg;
	//
	reg	[3:0]	r_cmd_crc_cnt;
	reg	[7:0]	r_cmd_crc;
	//
	reg		r_cmd_crc_ff;
	//
	reg	[3:0]	r_lgblklen;
	wire	[3:0]	max_lgblklen;
	assign	max_lgblklen = LGFIFOLN;
	//
	reg	[25:0]	r_watchdog;
	reg		r_watchdog_err;
	reg	pre_cmd_state;

	initial	r_cmd_busy = 1'b0;
	initial	r_data_reg = 32'h00;
	initial	r_last_r_one = 8'hff;
	initial	ll_cmd_stb = 1'b0;
	initial	ll_fifo_rd = 1'b0;
	initial	ll_fifo_wr = 1'b0;
	initial	r_rsp_state = 3'h0;
	initial	r_cmd_state = 3'h0;
	initial	r_use_fifo  = 1'b0;
	initial	r_data_fil  = 2'b00;
	initial	r_lgblklen  = LGFIFOLN;
	initial	r_cmd_err   = 1'b0;
	always @(posedge i_clk)
	begin
		if (~ll_cmd_stb)
		begin
			r_have_resp <= 1'b0;
			ll_fifo_wr <= 1'b0;
			ll_fifo_rd <= 1'b0;
			// r_rsp_state <= 3'h0;
			r_cmd_state <= 3'h0;
			r_use_fifo  <= 1'b0;
			r_data_fil <= 2'b00;
			r_cmd_resp <= `SDSPI_EXPECT_R1;
		end

		r_cmd_crc_stb <= 1'b0;
		if (pre_cmd_state)
		begin // While we are actively sending data, and clocking the
			// interface, do:
			//
			// Here we use the transmit command state, or
			// r_cmd_state, to determine where we are at in this
			// process, and we use (ll_cmd_stb)&&(ll_idle) to
			// determine that we have sent a byte.  ll_cmd_dat is
			// set here as well--it's the byte we wish to transmit.
			if (r_cmd_state == 3'h0)
			begin
				r_cmd_state <= r_cmd_state + 3'h1;
				ll_cmd_dat <= r_data_reg[31:24];
				r_cmd_crc_stb <= 1'b1;
			end else if (r_cmd_state == 3'h1)
			begin
				r_cmd_state <= r_cmd_state + 3'h1;
				ll_cmd_dat <= r_data_reg[23:16];
				r_cmd_crc_stb <= 1'b1;
			end else if (r_cmd_state == 3'h2)
			begin
				r_cmd_state <= r_cmd_state + 3'h1;
				ll_cmd_dat <= r_data_reg[15:8];
				r_cmd_crc_stb <= 1'b1;
			end else if (r_cmd_state == 3'h3)
			begin
				r_cmd_state <= r_cmd_state + 3'h1;
				ll_cmd_dat <= r_data_reg[7:0];
				r_cmd_crc_stb <= 1'b1;
			end else if (r_cmd_state == 3'h4)
			begin
				r_cmd_state <= r_cmd_state + 3'h1;
				ll_cmd_dat <= cmd_crc;
			end else if (r_cmd_state == 3'h5)
			begin
				ll_cmd_dat <= 8'hff;
				if (r_have_resp)
				begin
					if (r_use_fifo)
						r_cmd_state <= r_cmd_state+3'h1;
					else
						r_cmd_state <= r_cmd_state+3'h2;
					ll_fifo_rd <= (r_use_fifo)&&(r_fifo_wr);
					if ((r_use_fifo)&&(r_fifo_wr))
						ll_cmd_dat <= 8'hfe;
				end
			end else if (r_cmd_state == 3'h6)
			begin
				ll_cmd_dat <= 8'hff;
				if (ll_fifo_rd_complete)
				begin // If we've finished reading from the
					// FIFO, then move on
					r_cmd_state <= r_cmd_state + 3'h1;
					ll_fifo_rd <= 1'b0;
				end else if (ll_fifo_rd)
					ll_cmd_dat <= fifo_byte;
			end else // if (r_cmd_state == 7)
				ll_cmd_dat <= 8'hff;


			// Here we handle the receive portion of the interface.
			// Note that the IF begins with an if of ll_out_stb.
			// That is, if a byte is ready from the lower level.
			//
			// Here we depend upon r_cmd_resp, the response we are
			// expecting from the SDCard, and r_rsp_state, the
			// state machine for where we are at receive what we
			// are expecting.
			if (pre_rsp_state)
			begin
				case(r_rsp_state)
				`SDSPI_RSP_NONE: begin // Waiting on R1
					if (~ll_out_dat[7])
					begin
						r_last_r_one <= ll_out_dat;
						if (r_cmd_resp == `SDSPI_EXPECT_R1)
						begin // Expecting R1 alone
							r_have_resp <= 1'b1;
							ll_cmd_stb <= (r_use_fifo);
							r_data_reg <= 32'hffffffff;
							ll_fifo_wr<=(r_use_fifo)&&(~r_fifo_wr);
						end else if (r_cmd_resp == `SDSPI_EXPECT_R1B)
						begin // Go wait on R1b
							r_data_reg <= 32'hffffffff;
						end // else wait on 32-bit rsp
					end end
				`SDSPI_RSP_BSYWAIT: begin
					// Waiting on R1b, have R1
					if (nonzero_out)
						r_have_resp <= 1'b1;
					ll_cmd_stb <= (r_use_fifo);
					end 
				`SDSPI_RSP_GETWORD: begin
					// Have R1, waiting on all of R2/R3/R7
					r_data_reg <= { r_data_reg[23:0],
								ll_out_dat };
					r_data_fil <= r_data_fil+2'b01;
					if (r_data_fil == 2'b11)
					begin
						ll_cmd_stb <= (r_use_fifo);
						// r_rsp_state <= 3'h3;
					end end 
				`SDSPI_RSP_WAIT_WHILE_BUSY: begin
					// Wait while device is busy writing
					// if (nonzero_out)
					// begin
						// r_data_reg[31:8] <= 24'h00;
						// r_data_reg[7:0]<= ll_out_dat;
						// // r_rsp_state <= 3'h6;
					// end
					end
				`SDSPI_RSP_RDCOMPLETE: begin
					// Block write command has completed
					ll_cmd_stb <= 1'b0;
					end
				`SDSPI_RSP_WRITING: begin
					// We are reading from the device into
					// our FIFO
					if ((ll_fifo_wr_complete)
						// Or ... we receive an error
						||(r_read_err_token[0]))
					begin
						ll_fifo_wr <= 1'b0;
						ll_cmd_stb <= 1'b0;
					end end
				// `SDSPI_RSP_GETTOKEN:
				default: begin end
				endcase
			end

			if (r_use_fifo)
				r_data_reg <= { 26'h3ffffff, r_data_response_token, r_read_err_token };

			if (r_watchdog_err)
				ll_cmd_stb <= 1'b0;
			r_cmd_err<= (r_cmd_err)|(fifo_crc_err)|(r_watchdog_err)
					|(r_err_token);
		end else if (r_cmd_busy)
		begin
			r_cmd_busy <= (ll_cmd_stb)||(~ll_idle);
		end else if (cmd_stb)
		begin // Command write
			// Clear the error on any write, whether a commanding
			// one or not.  -- provided the user requests clearing
			// it (by setting the bit high)
			r_cmd_err  <= (r_cmd_err)&&(~i_wb_data[15]);
			// In a similar fashion, we can switch fifos even if
			// not in the middle of a command
			r_fifo_id  <= i_wb_data[12];
			//
			// Doesn't matter what this is set to as long as we
			// aren't busy, so we can set it irrelevantly here.
			ll_cmd_dat <= i_wb_data[7:0];
			//
			// Note that we only issue a write upon receiving a
			// valid command.  Such a command is 8 bits, and must
			// start with its high order bits set to zero and one.
			// Hence ... we test for that here.
			if (i_wb_data[7:6] == 2'b01)
			begin // Issue a command
				//
				r_cmd_busy <= 1'b1;
				//
				ll_cmd_stb <= 1'b1;
				r_cmd_resp <= i_wb_data[9:8];
				//
				r_cmd_crc_stb <= 1'b1;
				//
				r_fifo_wr  <= i_wb_data[10];
				r_use_fifo <= i_wb_data[11];
				//
			end else if (i_wb_data[7])
			// If, on the other hand, the command was invalid,
			// then it must have been an attempt to read our
			// internal configuration.  So we'll place that on
			// our data register.
				r_data_reg <= { 8'h00,
					4'h0, max_lgblklen,
					4'h0, r_lgblklen, 1'b0, r_sdspi_clk };
		end else if ((write_stb)&&(i_wb_addr == `SDSPI_DAT_ADDRESS))
		begin // Data write
			r_data_reg <= i_wb_data;
		end
	end

	always @(posedge i_clk)
		pre_cmd_state <= (ll_cmd_stb)&&(ll_idle);

	reg	ready_for_response_token;
	always @(posedge i_clk)
		if (~r_cmd_busy)
			ready_for_response_token <= 1'b0;
		else if (ll_fifo_rd)
			ready_for_response_token <= 1'b1;
	always @(posedge i_clk)
		if (~r_cmd_busy)
			r_have_data_response_token <= 1'b0;
		else if ((ll_out_stb)&&(ready_for_response_token)&&(~ll_out_dat[4]))
			r_have_data_response_token <= 1'b1;

	reg	[2:0]	second_rsp_state;
	always @(posedge i_clk)
		if((r_cmd_resp == `SDSPI_EXPECT_R1)&&(r_use_fifo)&&(r_fifo_wr))
			second_rsp_state <= `SDSPI_RSP_GETTOKEN;
		else if (r_cmd_resp == `SDSPI_EXPECT_R1)
			second_rsp_state <= `SDSPI_RSP_WRITING;
		else if (r_cmd_resp == `SDSPI_EXPECT_R1B)
			second_rsp_state <= `SDSPI_RSP_BSYWAIT;
		else
			second_rsp_state <= `SDSPI_RSP_GETWORD;

	reg	pre_rsp_state, nonzero_out;
	always @(posedge i_clk)
		if (ll_out_stb)
			nonzero_out <= (|ll_out_dat);
	always @(posedge i_clk)
		pre_rsp_state <= (ll_out_stb)&&(r_cmd_sent);

	// Each bit depends upon 8 bits of input
	initial	r_rsp_state = 3'h0;
	always @(posedge i_clk)
		if (~r_cmd_sent)
			r_rsp_state <= 3'h0;
		else if (pre_rsp_state)
		begin
			if ((r_rsp_state == `SDSPI_RSP_NONE)&&(~ll_out_dat[7]))
			begin
				r_rsp_state <= second_rsp_state;
			end else if (r_rsp_state == `SDSPI_RSP_BSYWAIT)
			begin // Waiting on R1b, have R1
				// R1b never uses the FIFO
				if (nonzero_out)
					r_rsp_state <= 3'h6;
			end else if (r_rsp_state == `SDSPI_RSP_GETWORD)
			begin // Have R1, waiting on all of R2/R3/R7
				if (r_data_fil == 2'b11)
					r_rsp_state <= `SDSPI_RSP_RDCOMPLETE;
			end else if (r_rsp_state == `SDSPI_RSP_GETTOKEN)
			begin // Wait on data token response
				if (r_have_data_response_token)
					r_rsp_state <= `SDSPI_RSP_WAIT_WHILE_BUSY;
			end else if (r_rsp_state == `SDSPI_RSP_WAIT_WHILE_BUSY)
			begin // Wait while device is busy writing
				if (nonzero_out)
					r_rsp_state <= `SDSPI_RSP_RDCOMPLETE;
			end
			//else if (r_rsp_state == 3'h6)
			//begin // Block write command has completed
			//	// ll_cmd_stb <= 1'b0;
			// end else if (r_rsp_state == 3'h7)
			// begin // We are reading from the device into
			//	// our FIFO
			// end
		end

	always @(posedge i_clk)
		r_cmd_sent <= (ll_cmd_stb)&&(r_cmd_state >= 3'h5);

	// initial	r_sdspi_clk = 6'h3c;
	initial	r_sdspi_clk = 7'h63;
	always @(posedge i_clk)
	begin
		// Update our internal configuration parameters, unconnected
		// with the card.  These include the speed of the interface,
		// and the size of the block length to expect as part of a FIFO
		// command.
		if ((cmd_stb)&&(i_wb_data[7:6]==2'b11)&&(~r_data_reg[7])
			&&(r_data_reg[15:12]==4'h00))
		begin
			if (|r_data_reg[6:0])
				r_sdspi_clk <= r_data_reg[6:0];
			if (|r_data_reg[11:8])
				r_lgblklen <= r_data_reg[11:8];
		end if (r_lgblklen > max_lgblklen)
			r_lgblklen <= max_lgblklen;
	end

	assign	need_reset = 1'b0;
	always @(posedge i_clk)
		case(i_wb_addr)
		`SDSPI_CMD_ADDRESS:
			o_wb_data <= { need_reset, 11'h00,
					2'h0, r_err_token, fifo_crc_err,
					r_cmd_err, r_cmd_busy, 1'b0, r_fifo_id,
					r_use_fifo, r_fifo_wr, r_cmd_resp,
					r_last_r_one };
		`SDSPI_DAT_ADDRESS:
			o_wb_data <= r_data_reg;
		`SDSPI_FIFO_A_ADDR:
			o_wb_data <= fifo_a_reg;
		`SDSPI_FIFO_B_ADDR:
			o_wb_data <= fifo_b_reg;
		endcase

	always @(posedge i_clk)
		o_wb_ack <= wb_stb;

	initial	q_busy = 1'b1;
	always @(posedge i_clk)
		q_busy <= r_cmd_busy;
	always @(posedge i_clk)
		o_int <= (~r_cmd_busy)&&(q_busy);

	assign	o_wb_stall = 1'b0;

	//
	// Let's work with our FIFO memory here ...
	//
	//
	always @(posedge i_clk)
	begin
		if ((write_stb)&&(i_wb_addr == `SDSPI_CMD_ADDRESS))
		begin // Command write
			// Clear the read/write address
			fifo_wb_addr <= {(LGFIFOLN){1'b0}};
		end else if ((wb_stb)&&(i_wb_addr[1]))
		begin // On read or write, of either FIFO,
			// we increase our pointer
			fifo_wb_addr <= fifo_wb_addr + 1;
			// And let ourselves know we need to update ourselves
			// on the next clock
		end
	end

	// Prepare reading of the FIFO for the WB bus read
	// Memory read #1
	always @(posedge i_clk)
	begin
		fifo_a_reg <= {
			fifo_a_mem[{ fifo_wb_addr, 2'b00 }],
			fifo_a_mem[{ fifo_wb_addr, 2'b01 }],
			fifo_a_mem[{ fifo_wb_addr, 2'b10 }],
			fifo_a_mem[{ fifo_wb_addr, 2'b11 }] };
		fifo_b_reg <= {
			fifo_b_mem[{ fifo_wb_addr, 2'b00 }],
			fifo_b_mem[{ fifo_wb_addr, 2'b01 }],
			fifo_b_mem[{ fifo_wb_addr, 2'b10 }],
			fifo_b_mem[{ fifo_wb_addr, 2'b11 }] };
	end

	// Okay, now ... writing our FIFO ...
	reg	pre_fifo_addr_inc_rd;
	reg	pre_fifo_addr_inc_wr;
	initial	pre_fifo_addr_inc_rd = 1'b0;
	initial	pre_fifo_addr_inc_wr = 1'b0;
	always @(posedge i_clk)
		pre_fifo_addr_inc_wr <= ((ll_fifo_wr)&&(ll_out_stb)
						&&(r_have_start_token));
	always @(posedge i_clk)
		pre_fifo_addr_inc_rd <= ((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle));
	always @(posedge i_clk)
	begin
		if (~r_cmd_busy)
			ll_fifo_addr <= {(LGFIFOLN+2){1'b0}};
		else if ((pre_fifo_addr_inc_wr)||(pre_fifo_addr_inc_rd))
			ll_fifo_addr <= ll_fifo_addr + 1;
	end

	//
	// Look for that start token.  This will be present when reading from 
	// the device into the FIFO.
	// 
	always @(posedge i_clk)
		if (~r_cmd_busy)
			r_have_start_token <= 1'b0;
		else if ((ll_fifo_wr)&&(ll_out_stb)&&(ll_out_dat==8'hfe))
			r_have_start_token <= 1'b1;
	always @(posedge i_clk)
		if (~r_cmd_busy)
			r_read_err_token <= 4'h0;
		else if ((ll_fifo_wr)&&(ll_out_stb)&&(~r_have_start_token)
				&&(ll_out_dat[7:4]==4'h0))
			r_read_err_token <= ll_out_dat[3:0];
	always @(posedge i_clk) // Look for a response to our writing
		if (~r_cmd_busy)
			r_data_response_token <= 2'b00;
		else if ((ready_for_response_token)
				&&(!ll_out_dat[4])&&(ll_out_dat[0]))
			r_data_response_token <= ll_out_dat[3:2];
	initial	r_err_token = 1'b0;
	always @(posedge i_clk)
		if (ll_fifo_rd)
			r_err_token <= (r_err_token)|(r_read_err_token[0]);
		else if (ll_fifo_wr)
			r_err_token <= (r_err_token)|
				((|r_data_response_token)&&(r_data_response_token[1]));
		else if (cmd_stb)
			// Clear the error on any write with the bit high
			r_err_token  <= (r_err_token)&&(~i_wb_data[16])
						&&(~i_wb_data[15]);

	reg	last_fifo_byte;
	initial last_fifo_byte = 1'b0;
	always @(posedge i_clk)
		if (ll_fifo_wr)
			last_fifo_byte <= (ll_fifo_addr == w_blklimit);
		else
			last_fifo_byte <= 1'b0;

	// This is the one (and only allowed) write to the FIFO memory always
	// block.
	//
	// If ll_fifo_wr is true, we'll be writing to the FIFO, and we'll do
	// that here.  This is different from r_fifo_wr, which specifies that
	// we will be writing to the SDCard from the FIFO, and hence READING
	// from the FIFO.
	//
	reg	pre_fifo_a_wr, pre_fifo_b_wr, pre_fifo_crc_a, pre_fifo_crc_b,
		clear_fifo_crc;
	always @(posedge i_clk)
	begin
		pre_fifo_a_wr <= (ll_fifo_wr)&&(ll_out_stb)
				&&(~r_fifo_id)&&(ll_fifo_wr_state == 2'b00);
		pre_fifo_b_wr <= (ll_fifo_wr)&&(ll_out_stb)
				&&( r_fifo_id)&&(ll_fifo_wr_state == 2'b00);
		fifo_wr_crc_stb <= (ll_fifo_wr)&&(ll_out_stb)
			&&(ll_fifo_wr_state == 2'b00)&&(r_have_start_token);
		pre_fifo_crc_a<= (ll_fifo_wr)&&(ll_out_stb)
				&&(ll_fifo_wr_state == 2'b01);
		pre_fifo_crc_b<= (ll_fifo_wr)&&(ll_out_stb)
				&&(ll_fifo_wr_state == 2'b10);
		clear_fifo_crc <= (cmd_stb)&&(i_wb_data[15]);
	end

	initial		fifo_crc_err = 1'b0;
	always @(posedge i_clk)
	begin // One and only memory write allowed
		if ((write_stb)&&(i_wb_addr[1:0]==2'b10))
			{fifo_a_mem[{ fifo_wb_addr, 2'b00 }],
			fifo_a_mem[{  fifo_wb_addr, 2'b01 }],
			fifo_a_mem[{  fifo_wb_addr, 2'b10 }],
			fifo_a_mem[{  fifo_wb_addr, 2'b11 }] }
			<= i_wb_data;
		else if (pre_fifo_a_wr)
			fifo_a_mem[{ ll_fifo_addr }] <= ll_out_dat;

		if ((write_stb)&&(i_wb_addr[1:0]==2'b11))
			{fifo_b_mem[{fifo_wb_addr, 2'b00 }],
			fifo_b_mem[{ fifo_wb_addr, 2'b01 }],
			fifo_b_mem[{ fifo_wb_addr, 2'b10 }],
			fifo_b_mem[{ fifo_wb_addr, 2'b11 }] }
			<= i_wb_data;
		else if (pre_fifo_b_wr)
			fifo_b_mem[{ ll_fifo_addr }] <= ll_out_dat;

		if (~r_cmd_busy)
			ll_fifo_wr_complete <= 1'b0;

		if (~r_cmd_busy)
			ll_fifo_wr_state <= 2'b00;
		else if ((pre_fifo_a_wr)||(pre_fifo_b_wr))
			ll_fifo_wr_state <= (last_fifo_byte)? 2'b01:2'b00;

		if (pre_fifo_crc_a)
		begin
			fifo_crc_err <= fifo_crc_err | (fifo_wr_crc_reg[15:8]!=ll_out_dat);
			ll_fifo_wr_state <= ll_fifo_wr_state + 2'b01;
		end if (pre_fifo_crc_b)
		begin
			fifo_crc_err <= fifo_crc_err | (fifo_wr_crc_reg[7:0]!=ll_out_dat);
			ll_fifo_wr_state <= ll_fifo_wr_state + 2'b01;
			ll_fifo_wr_complete <= 1'b1;
		end else if (clear_fifo_crc)
			fifo_crc_err <= 1'b0;
	end

	always @(posedge i_clk)
	begin // Second memory read, this time for the FIFO
		fifo_a_byte <= fifo_a_mem[ ll_fifo_addr ];
		fifo_b_byte <= fifo_b_mem[ ll_fifo_addr ];
	end

	reg	[(LGFIFOLN-1):0]	r_blklimit;
	wire	[(LGFIFOLN+1):0]	w_blklimit;
	always @(posedge i_clk)
		r_blklimit[(LGFIFOLN-1):0] = (1<<r_lgblklen)-1;
	assign	w_blklimit = { r_blklimit, 2'b11 };

	// Package the FIFO reads up into a packet
	always @(posedge i_clk)
	begin
		fifo_rd_crc_stb <= 1'b0;
		if (r_cmd_busy)
		begin
			if (ll_fifo_pkt_state[2:0] == 3'b000)
			begin
				if((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle))
				begin
					ll_fifo_pkt_state <= ll_fifo_pkt_state + 3'b001;
				end
			end else if (ll_fifo_pkt_state[2:0] == 3'b001)
			begin
				if((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle))
				begin
					ll_fifo_pkt_state <= ll_fifo_pkt_state + 3'b001;
					fifo_byte <= (r_fifo_id)
						? fifo_b_byte : fifo_a_byte;
					fifo_rd_crc_stb <= 1'b1;
				end
			end else if (ll_fifo_pkt_state[2:0] == 3'b010)
			begin
				if((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle))
				begin
					fifo_byte <= (r_fifo_id)
						? fifo_b_byte : fifo_a_byte;
					fifo_rd_crc_stb <= 1'b1;
				end
				if (ll_fifo_addr == 0)
					ll_fifo_pkt_state <= 3'b011;
			end else if (ll_fifo_pkt_state == 3'b011)
			begin // 1st CRC byte
				if((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle))
				begin
					fifo_byte <= fifo_rd_crc_reg[15:8];
					ll_fifo_pkt_state <= 3'b100;
				end
			end else if (ll_fifo_pkt_state == 3'b100)
			begin // 2nd CRC byte
				if((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle))
				begin
					fifo_byte <= fifo_rd_crc_reg[7:0];
					ll_fifo_pkt_state <= 3'b101;
				end
			end else if((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle))
			begin
			// Idle the channel
				ll_fifo_rd_complete <= 1'b1;
				fifo_byte <= 8'hff;
			end
		end else if ((write_stb)&&(i_wb_addr == `SDSPI_CMD_ADDRESS))
		begin
			ll_fifo_pkt_state <= 3'h0;
			ll_fifo_rd_complete <= 1'b0;
			fifo_byte <= (i_wb_data[12]) ? fifo_b_byte : fifo_a_byte;
			fifo_rd_crc_stb <= 1'b1;
		end else begin // Packet state is IDLE (clear the CRC registers)
			ll_fifo_pkt_state <= 3'b111;
			ll_fifo_rd_complete <= 1'b1;
		end
	end

	always @(posedge i_clk)
	begin
		if (~ll_fifo_wr)
			fifo_wr_crc_reg <= 16'h00;
		else if (fifo_wr_crc_stb)
		begin
			fifo_wr_crc_reg[15:8] <=fifo_wr_crc_reg[15:8]^ll_out_dat;
			fifo_wr_crc_count <= 4'h8;
		end else if (|fifo_wr_crc_count)
		begin
			fifo_wr_crc_count <= fifo_wr_crc_count - 4'h1;
			if (fifo_wr_crc_reg[15])
				fifo_wr_crc_reg <= { fifo_wr_crc_reg[14:0], 1'b0 }
					^ 16'h1021;
			else
				fifo_wr_crc_reg <= { fifo_wr_crc_reg[14:0], 1'b0 };
		end
	end

	always @(posedge i_clk)
	begin
		if (~r_cmd_busy)
		begin
			fifo_rd_crc_reg <= 16'h00;
			fifo_rd_crc_count <= 4'h0;
		end else if (fifo_rd_crc_stb)
		begin
			fifo_rd_crc_reg[15:8] <=fifo_rd_crc_reg[15:8]^fifo_byte;
			fifo_rd_crc_count <= 4'h8;
		end else if (|fifo_rd_crc_count)
		begin
			fifo_rd_crc_count <= fifo_rd_crc_count - 4'h1;
			if (fifo_rd_crc_reg[15])
				fifo_rd_crc_reg <= { fifo_rd_crc_reg[14:0], 1'b0 }
					^ 16'h1021;
			else
				fifo_rd_crc_reg <= { fifo_rd_crc_reg[14:0], 1'b0 };
		end
	end

	//
	// Calculate a CRC for the command section of our output
	//
	initial	r_cmd_crc_ff = 1'b0;
	always @(posedge i_clk)
	begin
		if (~r_cmd_busy)
		begin
			r_cmd_crc <= 8'h00;
			r_cmd_crc_cnt <= 4'hf;
			r_cmd_crc_ff <= 1'b0;
		end else if (~r_cmd_crc_cnt[3])
		begin
			r_cmd_crc_cnt <= r_cmd_crc_cnt - 4'h1;
			if (r_cmd_crc[7])
				r_cmd_crc <= { r_cmd_crc[6:0], 1'b0 } ^ 8'h12;
			else
				r_cmd_crc <= { r_cmd_crc[6:0], 1'b0 };
			r_cmd_crc_ff <= (r_cmd_crc_ff)||(r_cmd_crc_stb);
		end else if ((r_cmd_crc_stb)||(r_cmd_crc_ff))
		begin
			r_cmd_crc <= r_cmd_crc ^ ll_cmd_dat;
			r_cmd_crc_cnt <= 4'h7;
			r_cmd_crc_ff <= 1'b0;
		end
	end
	assign	cmd_crc = { r_cmd_crc[7:1], 1'b1 };

	//
	// Some watchdog logic for us.  This way, if we are waiting for the
	// card to respond, and something goes wrong, we can timeout the
	// transaction and ... figure out what to do about it later.  At least
	// we'll have an error indication.
	//
	initial	r_watchdog = 26'h3ffffff;
	initial	r_watchdog_err = 1'b0;
	always @(posedge i_clk)
		if (~r_cmd_busy)
			r_watchdog_err <= 1'b0;
		else if (r_watchdog == 0)
			r_watchdog_err <= 1'b1;
	always @(posedge i_clk)
		if (~r_cmd_busy)
			r_watchdog <= 26'h3fffff;
		else if (|r_watchdog)
			r_watchdog <= r_watchdog - 26'h1;

	assign o_debug = { ((ll_cmd_stb)&&(ll_idle))||(ll_out_stb),
				ll_cmd_stb, ll_idle, ll_out_stb, // 4'h
			o_cs_n, o_sck, o_mosi, i_miso, 	// 4'h
			r_cmd_state, i_bus_grant,	// 4'h
			r_rsp_state, r_cmd_busy,	// 4'h
			ll_cmd_dat,		// 8'b
			ll_out_dat };		// 8'b
endmodule

