////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdiodat.v
// {{{
// Project:	SDIO SD-Card controller, using a shared SPI interface
//
// Purpose:	
//
// Status:
//	This is the beginnings of a (near) top-level SDIO controller.
//	It is far from working, and likely even far fom synthesizing.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2018-2020, Gisselquist Technology, LLC
// {{{
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
`default_nettype	none
// }}}
module	sdiodat #(
		// {{{
		parameter	DW = 32,
		parameter	LGSECTORSZ = 9,	// 512 Bytes
		parameter	LGFIFOLN = LGSECTORSZ-$clog2(DW/8)
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		localparam [1:0] SD_CMD_ADDRESS = 2'b00,
				SD_FIFO_A_ADDR = 2'b10
				SD_FIFO_B_ADDR = 2'b10
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Wishbone interface
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire		i_wb_addr,
		input	wire [DW-1:0]	i_wb_data,
		input	wire [DW/8-1:0]	i_wb_sel,
		//
		output	wire		o_wb_stall,
		output	reg		o_wb_ack,
		output	reg	[31:0]	o_wb_data,
		//
		// Controller interface
		input	wire		i_cmd, i_we, i_adr,
		// SDCard interface
		input	wire		i_stb,	// i_sdio_stb
       		output	wire		o_dir,	// o_sdio_dir
		input	wire	[3:0]	i_data,	// i_sdio_data
		output	wire	[3:0]	o_data,	// o_sdio_data
		//
		output	wire		o_busy,
					o_err
		// }}}
	);

	reg	pre_fifo_addr_inc_rd;
	reg	pre_fifo_addr_inc_wr;
	reg	last_fifo_byte;
	reg	pre_fifo_a_wr, pre_fifo_b_wr, pre_fifo_crc_a, pre_fifo_crc_b,
		clear_fifo_crc;
	reg				fifo_a_wr, fifo_b_wr;
	reg	[3:0]			fifo_a_wr_mask, fifo_b_wr_mask;
	reg	[(LGFIFOLN-1):0]	fifo_a_wr_addr, fifo_b_wr_addr;
	reg	[31:0]			fifo_a_wr_data, fifo_b_wr_data;

	reg	[(LGFIFOLN-1):0]	r_blklimit;
	wire	[(LGFIFOLN+1):0]	w_blklimit;

	assign	write_stb = i_wb_stb && i_wb_we && (i_wb_sel != 0);

	initial last_fifo_byte = 1'b0;
	//
	// Let's work with our FIFO memory here ...
	//

	// fifo_wb_addr
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		fifo_wb_addr <= {(LGFIFOLN){1'b0}};
	else if ((write_stb)&&(wb_addr == SD_CMD_ADDRESS))
	begin // Command write
		// Clear the read/write address
		fifo_wb_addr <= {(LGFIFOLN){1'b0}};
	end else if ((wb_stb)&&(wb_addr[1]))
	begin // On read or write, of either FIFO,
		// we increase our pointer
		fifo_wb_addr <= fifo_wb_addr + 1;
		// And let ourselves know we need to update ourselves
		// on the next clock
	end
	// }}}

	// WB bus read -- actually read the FIFO
	// {{{
	// Memory read #1
	always @(posedge i_clk)
	begin
		fifo_a_reg <= fifo_a[ fifo_wb_addr ];
		fifo_b_reg <= fifo_b[ fifo_wb_addr ];
	end
	// }}}

	// Okay, now ... writing our FIFO ...
	initial	pre_fifo_addr_inc_rd = 1'b0;
	initial	pre_fifo_addr_inc_wr = 1'b0;
	always @(posedge i_clk)
		pre_fifo_addr_inc_wr <= ((ll_fifo_wr)&&(ll_out_stb)
						&&(r_have_start_token));
	always @(posedge i_clk)
		pre_fifo_addr_inc_rd <= ((ll_fifo_rd)&&(ll_cmd_stb)&&(ll_idle));

	// ll_fifo_addr
	// {{{
	always @(posedge i_clk)
	if (!r_cmd_busy)
		ll_fifo_addr <= {(LGFIFOLN+2){1'b0}};
	else if ((pre_fifo_addr_inc_wr)||(pre_fifo_addr_inc_rd))
		ll_fifo_addr <= ll_fifo_addr + 1;
	// }}}

	//
	// Look for that start token.  This will be present when reading from
	// the device into the FIFO.
	//

	// r_have_start_token
	// {{{
	always @(posedge i_clk)
	if (!r_cmd_busy)
		r_have_start_token <= 1'b0;
	else if ((ll_fifo_wr)&&(ll_out_stb)&&(ll_out_dat==8'hfe))
		r_have_start_token <= 1'b1;
	// }}}

	// r_read_er_token
	// {{{
	always @(posedge i_clk)
	if (!r_cmd_busy)
		r_read_err_token <= 4'h0;
	else if ((ll_fifo_wr)&&(ll_out_stb)&&(!r_have_start_token)
			&&(ll_out_dat[7:4]==4'h0))
		r_read_err_token <= ll_out_dat[3:0];
	// }}}

	// r_data_response_token
	// {{{
	always @(posedge i_clk) // Look for a response to our writing
	if (!r_cmd_busy)
		r_data_response_token <= 2'b00;
	else if ((ready_for_response_token)
			&&(!ll_out_dat[4])&&(ll_out_dat[0]))
		r_data_response_token <= ll_out_dat[3:2];
	// }}}

	// r_err_token
	// {{{
	initial	r_err_token = 1'b0;
	always @(posedge i_clk)
	if (ll_fifo_rd)
		r_err_token <= (r_err_token)|(r_read_err_token[0]);
	else if (ll_fifo_wr)
		r_err_token <= (r_err_token)|
			((|r_data_response_token)&&(r_data_response_token[1]));
	else if (cmd_stb)
		// Clear the error on any write with the bit high
		r_err_token  <= (r_err_token)&&(!i_wb_data[16])
						&&(!i_wb_data[15]);
	// }}}

	// last_fifo_byte
	// {{{
	always @(posedge i_clk)
	if (ll_fifo_wr)
		last_fifo_byte <= (ll_fifo_addr == w_blklimit);
	else
		last_fifo_byte <= 1'b0;
	// }}}

	// This is the one (and only allowed) write to the FIFO memory always
	// block.
	//
	// If ll_fifo_wr is true, we'll be writing to the FIFO, and we'll do
	// that here.  This is different from r_fifo_wr, which specifies that
	// we will be writing to the SDCard from the FIFO, and hence READING
	// from the FIFO.
	//
	always @(posedge i_clk)
	begin
		pre_fifo_a_wr <= (ll_fifo_wr)&&(ll_out_stb)
				&&(!r_fifo_id)&&(ll_fifo_wr_state == 2'b00);
		pre_fifo_b_wr <= (ll_fifo_wr)&&(ll_out_stb)
				&&( r_fifo_id)&&(ll_fifo_wr_state == 2'b00);
		fifo_wr_crc_stb <= (ll_fifo_wr)&&(ll_out_stb)
			&&(ll_fifo_wr_state == 2'b00)&&(r_have_start_token);
		pre_fifo_crc_a<= (ll_fifo_wr)&&(ll_out_stb)
				&&(ll_fifo_wr_state == 2'b01);
		pre_fifo_crc_b<= (ll_fifo_wr)&&(ll_out_stb)
				&&(ll_fifo_wr_state == 2'b10);
		clear_fifo_crc <= (new_cmd)&&(wb_data[15]);
	end

	// fifo_a_wr_*
	// {{{
	initial		fifo_crc_err = 1'b0;
	always @(posedge i_clk)
	begin // One and only memory write allowed
		fifo_a_wr <= 1'b0;
		fifo_a_wr_data <= { ll_out_dat, ll_out_dat, ll_out_dat, ll_out_dat };
		if ((write_stb)&&(wb_addr[1:0]==SD_FIFO_A_ADDR))
		begin
			fifo_a_wr <= 1'b1;
			fifo_a_wr_mask <= i_wb_sel;
			fifo_a_wr_addr <= fifo_wb_addr;
			fifo_a_wr_data <= wb_data;
		end else if (pre_fifo_a_wr)
		begin
			fifo_a_wr <= 1'b1;
			fifo_a_wr_addr <= ll_fifo_addr[(LGFIFOLN+1):2];
			case(ll_fifo_addr[$clog2(DW/8)-1:0])
			2'b00: fifo_a_wr_mask <= 4'b0001;
			2'b01: fifo_a_wr_mask <= 4'b0010;
			2'b10: fifo_a_wr_mask <= 4'b0100;
			2'b11: fifo_a_wr_mask <= 4'b1000;
			endcase
		end

		if ((fifo_a_wr)&&(fifo_a_wr_mask[0]))
			fifo_a_mem_0[fifo_a_wr_addr] <= fifo_a_wr_data[7:0];
		if ((fifo_a_wr)&&(fifo_a_wr_mask[1]))
			fifo_a_mem_1[fifo_a_wr_addr] <= fifo_a_wr_data[15:8];
		if ((fifo_a_wr)&&(fifo_a_wr_mask[2]))
			fifo_a_mem_2[fifo_a_wr_addr] <= fifo_a_wr_data[23:16];
		if ((fifo_a_wr)&&(fifo_a_wr_mask[3]))
			fifo_a_mem_3[fifo_a_wr_addr] <= fifo_a_wr_data[31:24];
	end
	// }}}

	// fifo_b_wr_*
	// {{{
	always @(posedge i_clk)
	begin
		fifo_b_wr <= 1'b0;
		fifo_b_wr_data <= { ll_out_dat, ll_out_dat, ll_out_dat, ll_out_dat };
		if ((write_stb)&&(wb_addr[1:0]==SD_FIFO_B_ADDR))
		begin
			fifo_b_wr <= 1'b1;
			fifo_b_wr_mask <= 4'b1111;
			fifo_b_wr_addr <= fifo_wb_addr;
			fifo_b_wr_data <= wb_data;
		end else if (pre_fifo_b_wr)
		begin
			fifo_b_wr <= 1'b1;
			fifo_b_wr_addr <= ll_fifo_addr[(LGFIFOLN+1):2];
			case(ll_fifo_addr[1:0])
			2'b00: fifo_b_wr_mask <= 4'b0001;
			2'b01: fifo_b_wr_mask <= 4'b0010;
			2'b10: fifo_b_wr_mask <= 4'b0100;
			2'b11: fifo_b_wr_mask <= 4'b1000;
			endcase
		end

		if ((fifo_b_wr)&&(fifo_b_wr_mask[0]))
			fifo_b_mem_0[fifo_b_wr_addr] <= fifo_b_wr_data[7:0];
		if ((fifo_b_wr)&&(fifo_b_wr_mask[1]))
			fifo_b_mem_1[fifo_b_wr_addr] <= fifo_b_wr_data[15:8];
		if ((fifo_b_wr)&&(fifo_b_wr_mask[2]))
			fifo_b_mem_2[fifo_b_wr_addr] <= fifo_b_wr_data[23:16];
		if ((fifo_b_wr)&&(fifo_b_wr_mask[3]))
			fifo_b_mem_3[fifo_b_wr_addr] <= fifo_b_wr_data[31:24];

		if (!r_cmd_busy)
			ll_fifo_wr_complete <= 1'b0;

		if (!r_cmd_busy)
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
	// }}}

	// fifo_?_byte
	// {{{
	always @(posedge i_clk)
	begin // Second memory read, this time for the FIFO
		case(ll_fifo_addr[1:0])
		2'b00: begin
			fifo_a_byte<=fifo_a_mem_0[ll_fifo_addr[(LGFIFOLN+1):2]];
			fifo_b_byte<=fifo_b_mem_0[ll_fifo_addr[(LGFIFOLN+1):2]];
			end
		2'b01: begin
			fifo_a_byte<=fifo_a_mem_1[ll_fifo_addr[(LGFIFOLN+1):2]];
			fifo_b_byte<=fifo_b_mem_1[ll_fifo_addr[(LGFIFOLN+1):2]];
			end
		2'b10: begin
			fifo_a_byte<=fifo_a_mem_2[ll_fifo_addr[(LGFIFOLN+1):2]];
			fifo_b_byte<=fifo_b_mem_2[ll_fifo_addr[(LGFIFOLN+1):2]];
			end
		2'b11: begin
			fifo_a_byte<=fifo_a_mem_3[ll_fifo_addr[(LGFIFOLN+1):2]];
			fifo_b_byte<=fifo_b_mem_3[ll_fifo_addr[(LGFIFOLN+1):2]];
			end
		endcase
	end
	// }}}

	// ?_blklimit
	// {{{
	always @(posedge i_clk)
		r_blklimit[(LGFIFOLN-1):0] <= (1<<r_lgblklen)-1;
	assign	w_blklimit = { r_blklimit, 2'b11 };
	// }}}

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
		end else if ((write_stb)&&(wb_addr == SD_CMD_ADDRESS))
		begin
			ll_fifo_pkt_state <= 3'h0;
			ll_fifo_rd_complete <= 1'b0;
			fifo_byte <= (wb_data[12]) ? fifo_b_byte : fifo_a_byte;
			fifo_rd_crc_stb <= 1'b1;
		end else begin // Packet state is IDLE (clear the CRC registers)
			ll_fifo_pkt_state <= 3'b111;
			ll_fifo_rd_complete <= 1'b1;
		end
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Watchdog timer
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	// Some watchdog logic for us.  This way, if we are waiting for the
	// card to respond, and something goes wrong, we can timeout the
	// transaction and ... figure out what to do about it later.  At least
	// we'll have an error indication.
	//
	initial	r_watchdog_err = 1'b0;
	always @(posedge i_clk)
	if (!r_cmd_busy)
		r_watchdog_err <= 1'b0;
	else if (r_watchdog == 0)
		r_watchdog_err <= 1'b1;

	initial	r_watchdog = 26'h3ffffff;
	always @(posedge i_clk)
	if (!r_cmd_busy)
		r_watchdog <= 26'h3fffff;
	else if (|r_watchdog)
		r_watchdog <= r_watchdog - 26'h1;
	// }}}

	// o_debug
	// {{{
	assign o_debug = { ((ll_cmd_stb)&&(ll_idle))||(ll_out_stb),
				ll_cmd_stb, ll_idle, ll_out_stb, // 4'h
			o_cs_n, o_sck, o_mosi, i_miso, 	// 4'h
			r_cmd_state, i_bus_grant,	// 4'h
			r_rsp_state, r_cmd_busy,	// 4'h
			ll_cmd_dat,		// 8'b
			ll_out_dat };		// 8'b
	// }}}

	// Make verilator happy
	// {{{
	// verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_wb_cyc };
	// verilator lint_on  UNUSED
	// }}}
endmodule

