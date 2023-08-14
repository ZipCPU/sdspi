////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdspi.v
// {{{
// Project:	SPI-based SD-Card controller
//
// Purpose:	SD Card controller, using SPI interface with the card and
//		WB interface with the rest of the system.  This is the top
//	level of the SPI based controller.
//
//	See the specification for more information.
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
`timescale 1ns/1ps
`default_nettype	none
// }}}
module	sdspi #(
		// {{{
		parameter [0:0]	OPT_CARD_DETECT = 1'b1,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		//
		// LGFIFOLN
		// {{{
		// LGFIFOLN defines the size of the internal memory in words.
		// An LGFIFOLN of 7 is appropriate for a 2^(7+2)=512 byte FIFO
		parameter			LGFIFOLN = 7,
		// }}}
		parameter			POWERUP_IDLE = 1000,
		// STARTUP_CLOCKS
		// {{{
		// Many SD-Cards require a minimum number of SPI clocks to get
		// them started.  STARTUP_CLOCKS defines this number.  Set this
		// to zero if you don't want to use this initialization
		// sequence.
		parameter			STARTUP_CLOCKS = 75,
		// }}}
		// CKDIV_BITS
		// {{{
		// For my first design, using an 80MHz clock, 7 bits to the
		// clock divider was plenty.  Now that I'm starting to use
		// faster and faster designs, it becomes important to
		// parameterize the number of bits in the clock divider.  More
		// than 8, however, and the interface will need to change.
		parameter			CKDIV_BITS = 8,
		// }}}
		// INITIAL_CLKDIV
		// {{{
		// The SPI frequency is given by the system clock frequency
		// divided by a (clock_divider + 1).  INITIAL_CLKDIV provides
		// an initial value for this clock divider.
		parameter [CKDIV_BITS-1:0]	INITIAL_CLKDIV = 8'h7c,
		// }}}
		// OPT_SPI_ARBITRATION
		// {{{
		// When I originally built this SDSPI controller, it was for an
		// environment where the SPI was shared.  Doing this requires
		// feedback from an arbiter, to know when one SPI device has
		// the bus or not.  This feedback is provided in i_bus_grant.
		// If you don't have an arbiter, just set i_bus_grant to the
		// constant 1'b1 and set OPT_SPI_ARBITRATION to 1'b0 to remove
		// this extra logic.
		parameter [0:0]			OPT_SPI_ARBITRATION = 1'b0,
		// }}}
		//
		//
		parameter [0:0]		OPT_EXTRA_WB_CLOCK = 1'b0,
		//
		//
		//
		localparam	AW = 2, DW = 32
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_sd_reset,
		// Wishbone interface
		// {{{
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire [AW-1:0]	i_wb_addr,
		input	wire [DW-1:0]	i_wb_data,
		input	wire [DW/8-1:0]	i_wb_sel,
		output	wire		o_wb_stall,
		output	reg		o_wb_ack,
		output	reg [DW-1:0]	o_wb_data,
		// }}}
		// SDCard interface
		// {{{
		output	wire		o_cs_n, o_sck, o_mosi,
		input	wire		i_miso, i_card_detect,
		// }}}
		// Our interrupt
		output	reg		o_int,
		// .. and whether or not we can use the SPI port
		input	wire		i_bus_grant,
		// And some wires for debugging it all
		//
		output	reg [DW-1:0]	o_debug
		// }}}
	);

	// Signal / parameter declarations
	// {{{
	localparam [1:0]	SDSPI_CMD_ADDRESS = 2'b00,
				SDSPI_DAT_ADDRESS = 2'b01,
				SDSPI_FIFO_A_ADDR = 2'b10,
				SDSPI_FIFO_B_ADDR = 2'b11;

	localparam	BLKBASE = 16;

	//
	// Command register bit definitions
	//
	localparam	CARD_REMOVED_BIT= 18,
			// CRCERR_BIT	= 16,
			ERR_BIT		= 15,
			FIFO_ID_BIT	= 12,
			USE_FIFO_BIT	= 11,
			FIFO_WRITE_BIT	= 10;
	//
	// Some WB simplifications:
	//
	reg		r_cmd_busy;

	reg			dbg_trigger;

	wire		wb_stb, write_stb, wb_cmd_stb, new_data;
	wire	[AW-1:0]	wb_addr;
	wire	[DW-1:0]	wb_data;
	wire	[3:0]		wb_sel;
	reg	[1:0]	pipe_addr;
	reg		dly_stb;

	reg	[31:0]	fifo_a	[0:((1<<LGFIFOLN)-1)];
	reg	[31:0]	fifo_b	[0:((1<<LGFIFOLN)-1)];
	reg	[(LGFIFOLN-1):0]	fifo_wb_addr;
	reg	[(LGFIFOLN-1):0]	write_fifo_a_addr, write_fifo_b_addr,
					read_fifo_a_addr, read_fifo_b_addr;
	wire	[LGFIFOLN:0]	spi_read_addr, spi_write_addr;
	// reg	[3:0]		write_fifo_a_mask, write_fifo_b_mask;
	reg	[31:0]		write_fifo_a_data, write_fifo_b_data,
				fifo_a_word, fifo_b_word, spi_read_data;
	wire	[31:0]		spi_write_data;
	reg			write_fifo_a, write_fifo_b;
	reg	[31:0]		r_data_reg;
	reg			r_cmd_err;
	reg	[7:0]		r_last_r_one;

	//
	//
	wire		card_removed, card_present;
	//
	reg	[3:0]	r_lgblklen;
	wire	[3:0]	max_lgblklen;
	reg	[25:0]	r_watchdog;
	reg		r_watchdog_err;

	reg	[DW-1:0]	card_status;
	wire		ll_advance;

	reg	[CKDIV_BITS-1:0]	r_sdspi_clk;
	reg		ll_cmd_stb;
	reg	[7:0]	ll_cmd_dat;
	wire		ll_out_stb, ll_idle;
	wire	[7:0]	ll_out_dat;

	reg		r_fifo_id, r_use_fifo, write_to_card;

	wire	w_reset;

	wire		cmd_out_stb;
	wire	[7:0]	cmd_out_byte;
	wire		cmd_sent, cmd_valid, cmd_busy;
	wire	[39:0]	cmd_response;

	reg		rx_start;
	wire		spi_write_to_fifo;
	wire		rx_valid, rx_busy;
	wire	[7:0]	rx_response;

	reg		tx_start;
	wire		spi_read_from_fifo;
	wire		tx_stb;
	wire	[7:0]	tx_byte;
	wire		tx_valid, tx_busy;
	wire	[7:0]	tx_response;

	reg	last_busy;


	// }}}

	// Take an extra wishbone clock?
	// {{{
	generate if (!OPT_EXTRA_WB_CLOCK)
	begin : EXTRA_WB_PASSTHROUGH
		// {{{
		assign	wb_stb    = ((i_wb_stb)&&(!o_wb_stall));
		assign	write_stb = ((wb_stb)&&( i_wb_we) && i_wb_sel != 0);
		// assign	read_stb  = ((wb_stb)&&(!i_wb_we));
		assign	wb_sel = i_wb_sel;
		assign	wb_cmd_stb  = (!r_cmd_busy)&& write_stb && (&i_wb_sel)
				&&(i_wb_addr==SDSPI_CMD_ADDRESS);
		assign	wb_addr = i_wb_addr;
		assign	wb_data = i_wb_data;
		assign	new_data = (i_wb_stb)&&(!o_wb_stall)
				&&(i_wb_we && i_wb_sel != 0)
				&&(i_wb_addr == SDSPI_DAT_ADDRESS);
		// }}}
	end else begin : GEN_EXTRA_WB_CLOCK
		// {{{
		reg		r_wb_stb, r_write_stb, r_wb_cmd_stb, r_new_data;
		reg	[AW-1:0]	r_wb_addr;
		reg	[DW-1:0]	r_wb_data;
		reg	[DW/8-1:0]	r_wb_sel;

		initial	r_wb_stb = 1'b0;
		always @(posedge i_clk)
			r_wb_stb <= ((i_wb_stb)&&(!o_wb_stall));

		initial	r_write_stb = 1'b0;
		always @(posedge i_clk)
			r_write_stb <= ((i_wb_stb)&&(!o_wb_stall)&&(i_wb_we && i_wb_sel != 0));

		initial	r_wb_sel = 1'b0;
		always @(posedge i_clk)
			r_wb_sel <= i_wb_sel;

		initial	r_wb_cmd_stb = 1'b0;
		always @(posedge i_clk)
			r_wb_cmd_stb <= (!r_cmd_busy)&&(i_wb_stb)&&(!o_wb_stall)&&(i_wb_we && i_wb_sel != 0)
					&&(i_wb_addr == SDSPI_CMD_ADDRESS);

		always @(posedge i_clk)
			r_new_data <= (i_wb_stb)&&(!o_wb_stall)
					&&(i_wb_we && i_wb_sel != 0)
					&&(i_wb_addr == SDSPI_DAT_ADDRESS);

		always @(posedge i_clk)
			r_wb_addr <= i_wb_addr;

		always @(posedge i_clk)
			r_wb_data <= i_wb_data;

		assign	wb_stb   = r_wb_stb;
		assign	write_stb= r_write_stb;
		assign	wb_cmd_stb  = r_wb_cmd_stb;
		assign	new_data = r_new_data;
		assign	wb_addr  = r_wb_addr;
		assign	wb_data  = r_wb_data;
		assign	wb_sel   = r_wb_sel;
		// }}}
	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Lower-level SDSPI driver
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	// Access to our lower-level SDSPI driver, the one that actually
	// uses/sets the SPI ports
	//

	llsdspi #(
		// {{{
		.SPDBITS(CKDIV_BITS),
		.STARTUP_CLOCKS(STARTUP_CLOCKS),
		.POWERUP_IDLE(POWERUP_IDLE),
		.OPT_SPI_ARBITRATION(OPT_SPI_ARBITRATION)
		// }}}
	) lowlevel(
		// {{{
		i_clk, i_sd_reset, r_sdspi_clk, r_cmd_busy, ll_cmd_stb,
		ll_cmd_dat, o_cs_n, o_sck, o_mosi, i_miso,
		ll_out_stb, ll_out_dat, ll_idle,
		i_bus_grant
		// }}}
	);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Command controller
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	w_reset = i_sd_reset || r_watchdog_err;

	spicmd
	spicmdi(
		// {{{
		i_clk, w_reset, (wb_cmd_stb && wb_data[7:6] == 2'b01),
			wb_data[9:8], wb_data[5:0], r_data_reg, cmd_busy,
		cmd_out_stb, cmd_out_byte, !ll_advance,
		ll_out_stb, ll_out_dat,
		cmd_sent,
		cmd_valid, cmd_response
		// }}}
	);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Receive data (not commands)
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	spirxdata #(
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN)
	) spirxdatai(
		// {{{
		i_clk, w_reset | r_cmd_err, rx_start,
			r_lgblklen, r_fifo_id, rx_busy,
		ll_out_stb && !cmd_busy, ll_out_dat,
		spi_write_to_fifo, spi_write_addr, spi_write_data,
		rx_valid, rx_response
		// }}}
	);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Transmit/send data (not commands) to the SD card
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	spitxdata #(
		.RDDELAY(2),
		.OPT_LITTLE_ENDIAN(OPT_LITTLE_ENDIAN)
	) spitxdatai(
		// {{{
		i_clk, w_reset | r_cmd_err, tx_start,
			r_lgblklen, r_fifo_id, tx_busy,
		spi_read_from_fifo, spi_read_addr, spi_read_data,
		!ll_advance || cmd_busy, tx_stb, tx_byte,
		ll_out_stb && !cmd_busy, ll_out_dat,
		tx_valid, tx_response
		// }}}
	);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Internal FIFO memory
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	//
	// Let's work with our FIFO memory here ...
	//
	always @(posedge i_clk)
	begin
		if ((write_stb)&&(wb_addr == SDSPI_CMD_ADDRESS))
		begin // Command write
			// Clear the read/write address
			fifo_wb_addr <= {(LGFIFOLN){1'b0}};
		end else if ((wb_stb)&&(wb_addr[1] && wb_sel != 0))
		begin // On read or write, of either FIFO,
			// we increase our pointer
			// if (wb_sel[0])
				fifo_wb_addr <= fifo_wb_addr + 1;
			// And let ourselves know we need to update ourselves
			// on the next clock
		end
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Writes to the FIFO
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	initial	write_fifo_a = 0;
	always @(posedge i_clk)
	if (r_use_fifo && rx_busy && !spi_write_addr[LGFIFOLN])
	begin
		write_fifo_a      <= spi_write_to_fifo;
		write_fifo_a_data <= spi_write_data;
		write_fifo_a_addr <= spi_write_addr[LGFIFOLN-1:0];
		// write_fifo_a_mask <= 4'hf;
	end else begin
		write_fifo_a      <= write_stb &&(wb_addr == SDSPI_FIFO_A_ADDR) &&(&wb_sel);
		write_fifo_a_data <= wb_data;
		write_fifo_a_addr <= fifo_wb_addr;
		// write_fifo_a_mask <= 4'hf;
	end

	initial	write_fifo_b = 0;
	always @(posedge i_clk)
	if (r_use_fifo && rx_busy && spi_write_addr[LGFIFOLN])
	begin
		write_fifo_b      <= spi_write_to_fifo;
		write_fifo_b_data <= spi_write_data;
		write_fifo_b_addr <= spi_write_addr[LGFIFOLN-1:0];
		// write_fifo_b_mask <= 4'hf;
	end else begin
		write_fifo_b      <= write_stb &&(wb_addr == SDSPI_FIFO_B_ADDR) && (&wb_sel);
		write_fifo_b_data <= wb_data;
		write_fifo_b_addr <= fifo_wb_addr;
		// write_fifo_b_mask <= 4'hf;
	end

	always @(posedge i_clk)
	if (write_fifo_a)
		fifo_a[write_fifo_a_addr] <= write_fifo_a_data;

	always @(posedge i_clk)
	if (write_fifo_b)
		fifo_b[write_fifo_b_addr] <= write_fifo_b_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Reads from the FIFO
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
	if (r_use_fifo && tx_busy && !spi_read_addr[LGFIFOLN])
		read_fifo_a_addr = spi_read_addr[LGFIFOLN-1:0];
	else
		read_fifo_a_addr = fifo_wb_addr;

	always @(*)
	if (r_use_fifo && tx_busy && spi_read_addr[LGFIFOLN])
		read_fifo_b_addr = spi_read_addr[LGFIFOLN-1:0];
	else
		read_fifo_b_addr = fifo_wb_addr;

	always @(posedge i_clk)
		fifo_a_word <= fifo_a[read_fifo_a_addr];

	always @(posedge i_clk)
		fifo_b_word <= fifo_b[read_fifo_b_addr];

	always @(posedge i_clk)
	if (!spi_read_addr[LGFIFOLN])
		spi_read_data <= fifo_a_word;
	else
		spi_read_data <= fifo_b_word;

	initial	r_fifo_id = 0;
	always @(posedge i_clk)
	if (!r_cmd_busy && wb_cmd_stb)
		r_fifo_id  <= wb_data[FIFO_ID_BIT];
	// }}}
	// }}}
	// r_cmd_busy, tx_start, rx_start, r_use_fifo, write_to_card
	// {{{
	initial	r_cmd_busy = 0;
	initial	tx_start = 0;
	initial	rx_start = 0;
	always @(posedge i_clk)
	if (i_sd_reset)
	begin
		// {{{
		r_cmd_busy <= 0;
		r_use_fifo <= 0;
		tx_start <= 0;
		rx_start <= 0;
		// }}}
	end else if (!r_cmd_busy)
	begin
		// {{{
		r_cmd_busy <= wb_cmd_stb && (wb_data[7:6] == 2'b01);
		tx_start <= 0;
		rx_start <= 0;
		if (wb_cmd_stb && wb_data[7:6] == 2'b01)
		begin
			write_to_card <= wb_data[FIFO_WRITE_BIT];
			r_use_fifo <= wb_data[USE_FIFO_BIT];
			if (wb_data[USE_FIFO_BIT])
			begin
				tx_start   <= (wb_data[FIFO_WRITE_BIT]);
				rx_start   <= (!wb_data[FIFO_WRITE_BIT]);
			end
		end
		if (r_watchdog_err)
		begin
			r_use_fifo <= 0;
			tx_start <= 0;
			rx_start <= 0;
		end
		// }}}
	end else begin
		// {{{
		if (ll_idle && !ll_cmd_stb && !cmd_busy && !rx_busy && !tx_busy)
		begin
			r_cmd_busy <= 0;
			r_use_fifo <= 0;
		end

		if (r_cmd_err || tx_busy || rx_busy)
		begin
			tx_start <= 0;
			rx_start <= 0;
		end

		if (r_watchdog_err)
		begin
			r_use_fifo <= 0;
			tx_start <= 0;
			rx_start <= 0;
		end
		// }}}
	end
	// }}}

	// r_cmd_err
	// {{{
	initial	r_cmd_err = 0;
	always @(posedge i_clk)
	if (r_watchdog_err)
		r_cmd_err <= 1;
	else if (r_cmd_busy)
	begin
		//
		// A command error is a watchdog error, so nothing needed here
		//
		// if (cmd_valid) r_cmd_err <= |cmd_response[38:33];
		//
		// A transmit error can be discovered as a response to a
		// command
		//
		// if (tx_valid)  r_cmd_err <= 0;
		//
		// However, we can check read response tokens for errors
		if (cmd_valid)
			r_cmd_err <= r_cmd_err || (cmd_response[38:33] != 0);
		if (rx_valid)
			r_cmd_err <= r_cmd_err || rx_response[3];
	end else if (wb_cmd_stb)
		r_cmd_err <= (r_cmd_err)&&(!wb_data[ERR_BIT]);
	// }}}

	// r_data_reg
	// {{{
	always @(posedge i_clk)
	if (!r_cmd_busy)
	begin
		if (new_data)
			r_data_reg <= wb_data;
		else if (wb_cmd_stb && wb_data[7])
			r_data_reg <= {
				4'h0, max_lgblklen,
				1'b0, // Rsrved for: Read data from CMD wire
				3'h0, r_lgblklen,
				{(16-CKDIV_BITS){1'b0}},
				r_sdspi_clk };
	end else begin
		if (cmd_valid)
		begin
			r_data_reg   <= cmd_response[31:0];
			r_last_r_one <= cmd_response[39:32];
		end else if (tx_valid)
			r_data_reg   <= { 24'h0, tx_response[7:0] };
		else if (rx_valid)
			r_data_reg   <= { 24'h0, rx_response[7:0] };
	end
	// }}}

	assign	ll_advance = (!ll_cmd_stb || ll_idle);

	// ll_cmd_stb, ll_cmd_dat
	// {{{
	initial	ll_cmd_stb = 0;
	always @(posedge i_clk)
	begin
		if (ll_advance)
		begin
			if (cmd_busy)
			begin
				ll_cmd_stb <= (ll_cmd_stb || cmd_out_stb);
				ll_cmd_dat <= cmd_out_stb ? cmd_out_byte :8'hff;
			end else begin
				ll_cmd_stb <= (ll_cmd_stb || tx_stb);
				ll_cmd_dat <= tx_stb ? tx_byte : 8'hff;
			end
		end

		if (ll_idle && !cmd_busy && !rx_busy && !tx_busy)
			ll_cmd_stb <= 1'b0;

		if (!r_cmd_busy || i_sd_reset)
			ll_cmd_stb <= 1'b0;
	end
	// }}}

	assign	max_lgblklen = LGFIFOLN+2;

	// r_sdspi_clk, r_lgblklen
	// {{{
	initial	r_sdspi_clk = INITIAL_CLKDIV;
	initial	r_lgblklen = 9;
	always @(posedge i_clk)
	begin
		// Update our internal configuration parameters, unconnected
		// with the card.  These include the speed of the interface,
		// and the size of the block length to expect as part of a FIFO
		// command.
		if ((wb_cmd_stb)&&(wb_data[7:6]==2'b11))
			// &&(!r_data_reg[7])
			// &&(r_data_reg[15:12]==4'h00))
		begin
			if (r_data_reg[CKDIV_BITS-1:0] != 0)
				r_sdspi_clk <= r_data_reg[CKDIV_BITS-1:0];
			if ((r_data_reg[BLKBASE +: 4] >= 3)
				&&(r_data_reg[BLKBASE +: 4] <= max_lgblklen))
				r_lgblklen <= r_data_reg[BLKBASE +: 4];
		end
		// if (r_lgblklen > max_lgblklen)
		//	r_lgblklen <= max_lgblklen;

		if (!card_present)
			r_sdspi_clk <= INITIAL_CLKDIV;
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone return logic
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge i_clk)
		pipe_addr <= wb_addr;

	always @(*)
		card_status = { 8'h00,		// 8b
			2'b0, r_watchdog_err, i_sd_reset,	// 4b
			!card_present, card_removed, 1'b0, 1'b0,
			r_cmd_err, r_cmd_busy, 1'b0, r_fifo_id,	// 4b
			r_use_fifo, write_to_card, 2'b00,	// 4b
			r_last_r_one };	// 8b

	always @(posedge i_clk)
	case(pipe_addr)
	SDSPI_CMD_ADDRESS:
		o_wb_data <= card_status;
	SDSPI_DAT_ADDRESS:
		o_wb_data <= r_data_reg;
	SDSPI_FIFO_A_ADDR:
		o_wb_data <= fifo_a_word;
	SDSPI_FIFO_B_ADDR:
		o_wb_data <= fifo_b_word;
	endcase

	initial	dly_stb = 0;
	always @(posedge i_clk)
	if (!i_wb_cyc)
		dly_stb <= 0;
	else
		dly_stb <= wb_stb;

	initial	o_wb_ack = 0;
	always @(posedge i_clk)
	if (!i_wb_cyc)
		o_wb_ack <= 1'b0;
	else
		o_wb_ack <= dly_stb;

	assign	o_wb_stall = 1'b0;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Interrupt generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	initial	last_busy = 0;
	always @(posedge i_clk)
		last_busy <= r_cmd_busy;

	initial	o_int = 0;
	always @(posedge i_clk)
		o_int <= (!r_cmd_busy)&&(last_busy)
			||(!card_removed && !card_present);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Card detection logic --- is the card even present?
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Depends upon the i_card_detect signal.  Set this signal to 1'b1 if
	// you your device doesn't have it.
	//
	//
	generate if (OPT_CARD_DETECT)
	begin : GEN_CARD_DETECT
		reg	[2:0]	raw_card_present;
		reg	[9:0]	card_detect_counter;
		reg		r_card_removed, r_card_present;

		initial	r_card_removed = 1'b1;
		always @(posedge i_clk)
		if (i_sd_reset)
			r_card_removed <= 1'b1;
		else if (!card_present)
			r_card_removed <= 1'b1;
		else if (wb_cmd_stb && wb_data[CARD_REMOVED_BIT])
			r_card_removed <= 1'b0;

		initial	raw_card_present = 0;
		always @(posedge i_clk)
			raw_card_present <= { raw_card_present[1:0], i_card_detect };

		initial	card_detect_counter = 0;
		always @(posedge i_clk)
		if (i_sd_reset || !raw_card_present[2])
			card_detect_counter <= 0;
		else if (!(&card_detect_counter))
			card_detect_counter <= card_detect_counter + 1;

		initial r_card_present = 1'b0;
		always @(posedge i_clk)
		if (i_sd_reset || !raw_card_present[2])
			r_card_present <= 1'b0;
		else if (&card_detect_counter)
			r_card_present <= 1'b1;

		assign	card_present = r_card_present;
		assign	card_removed = r_card_removed;

	end else begin : NO_CARD_DETECT_SIGNAL

		assign	card_present = 1'b1;
		assign	card_removed = 1'b0;

	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Watchdog protection logic
	// {{{
	////////////////////////////////////////////////////////////////////////
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
	////////////////////////////////////////////////////////////////////////
	//
	// Debug signals
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	initial	dbg_trigger = 0;
	always @(posedge i_clk)
		dbg_trigger <= (cmd_valid)&&(cmd_response[38:33] != 0);

	always @(posedge i_clk)
		o_debug <= { dbg_trigger, ll_cmd_stb,
				(ll_cmd_stb & ll_idle), ll_out_stb, // 4'h
			o_cs_n, o_sck, o_mosi, i_miso, 	// 4'h
			3'b000, i_sd_reset,	// 4'h
			3'b000, r_cmd_busy,	// 4'h
			ll_cmd_dat,		// 8'b
			ll_out_dat };		// 8'b
	// }}}

	// Make verilator happy
	// {{{
	// verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_wb_cyc, i_wb_sel, cmd_sent,
			spi_read_from_fifo };
	// verilator lint_on  UNUSED
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal verification properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
	localparam	F_LGDEPTH = 3;
	wire	[F_LGDEPTH-1:0]	f_nacks, f_nreqs, f_outstanding;
	reg	f_past_valid;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone Bus properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	fwb_slave #(
		.AW(2), .DW(32),
		.F_LGDEPTH(F_LGDEPTH),
		.F_MAX_STALL(1),
		.F_MAX_ACK_DELAY(2),
		.F_OPT_DISCONTINUOUS(1),
		.F_OPT_MINCLOCK_DELAY(1)
	) fwb(
		.i_clk(i_clk), .i_reset(!f_past_valid),
		.i_wb_cyc(i_wb_cyc), .i_wb_stb(i_wb_stb), .i_wb_we(i_wb_we),
		.i_wb_addr(i_wb_addr),.i_wb_data(i_wb_data),.i_wb_sel(i_wb_sel),
			.i_wb_stall(o_wb_stall), .i_wb_ack(o_wb_ack),
			.i_wb_idata(o_wb_data), .i_wb_err(1'b0),
		.f_nreqs(f_nreqs), .f_nacks(f_nacks),
		.f_outstanding(f_outstanding)
	);

	always @(*)
	if (i_wb_cyc)
		assert(f_outstanding == (o_wb_ack ? 1:0) + (dly_stb ? 1:0)
			+ (OPT_EXTRA_WB_CLOCK ? wb_stb : 0));

	////////////////////////////////////////////////////////////////////////
	//
	// Contract checks
	//
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(*)
		assert(!tx_busy || !rx_busy);

	always @(*)
		assert(!tx_start || !rx_start);

	always @(*)
	if (!r_use_fifo)
	begin
		assert(!tx_start && !rx_start);
		assert(!tx_busy && !rx_busy);
		if(!write_to_card)
			assert(!rx_start && !rx_busy);
		else
			assert(!tx_start && !tx_busy);
	end

	always @(*)
	if (tx_busy || rx_busy || cmd_busy)
		assert(r_cmd_busy);

	always @(*)
	begin
		assert(r_lgblklen >= 3);
		assert(r_lgblklen <= 9);
	end

	always @(*)
	if (cmd_busy)
	begin
		assert(spi_read_addr[LGFIFOLN-1:0] <= 1);
		assert(spi_write_addr[LGFIFOLN-1:0] <= 1);
	end

	//
	// Command sequence check
	(* anyseq *)	reg	f_cmd_check_value;
	reg	[1:0]	f_cmd_seq;
	reg	[7:0]	f_cmd_byte;
`ifdef	VERIFIC
	always @(posedge i_clk)
	if (f_cmd_check_value && f_cmd_seq == 0 && cmd_out_stb && !spicmdi.i_ll_busy)
		f_cmd_byte <= cmd_out_byte;

	initial	f_cmd_seq = 0;
	always @(posedge i_clk)
	if (i_sd_reset || r_watchdog_err)
		f_cmd_seq <= 0;
	else if (f_cmd_check_value && f_cmd_seq == 0 && cmd_out_stb && !spicmdi.i_ll_busy)
		f_cmd_seq <= 1;
	else if (!ll_cmd_stb || ll_idle)
		f_cmd_seq <= f_cmd_seq << 1;

	always @(*)
	if (!i_sd_reset && !r_watchdog_err) case(f_cmd_seq)
	0: begin end
	1: begin
		assert(ll_cmd_stb);
		assert(ll_cmd_dat == f_cmd_byte);
		end
	2: begin
		end
	endcase
`endif

	////////////////////////////////////////////////////////////////////////
	//
	// Abstract LLSDSPI properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
//`ifdef	ABSTRACT_LLSDSPI
//`define	LLSDSPI_ASSERT	assume
//`else
//`define	LLSDSPI_ASSERT	assert
//`endif
//	reg	f_first_byte_accepted;
//
//	always @(posedge i_clk)
//	if (!f_past_valid)
//		`LLSDSPI_ASSERT(!ll_out_stb);
//	else if (!$past(r_cmd_busy))
//		`LLSDSPI_ASSERT(!ll_out_stb);
//	else if ($past(ll_out_stb))
//		`LLSDSPI_ASSERT(!ll_out_stb);
//
//	always @(posedge i_clk)
//	if (!r_cmd_busy)
//		f_first_byte_accepted <= 1'b0;
//	else if (ll_cmd_stb && ll_idle)
//		f_first_byte_accepted <= 1'b1;
//
//	always @(posedge i_clk)
//	if (f_past_valid && $past(f_past_valid))
//	begin
//		if ($rose(r_cmd_busy))
//		begin
//			assert($rose(ll_cmd_stb));
//			`LLSDSPI_ASSERT(!ll_out_stb);
//		end else if (r_cmd_busy && f_first_byte_accepted && $past(f_first_byte_accepted))
//			`LLSDSPI_ASSERT(ll_out_stb == $past(ll_idle));
//	end
//
//	always @(posedge i_clk)
//	if (f_past_valid)
//	begin
//		if ($past(i_sd_reset || r_watchdog_err))
//			assert(!r_cmd_busy);
//		else if ($past(r_cmd_busy && ll_out_stb && !ll_idle))
//		begin
//			assert(ll_out_stb);
//			assert($stable(ll_out_dat));
//		end
//	end

	////////////////////////////////////////////////////////////////////////
	//
	// Watchdog checks
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge i_clk)
	if (f_past_valid && $past(r_watchdog_err))
		assert(!cmd_busy && !tx_busy && !rx_busy);

	////////////////////////////////////////////////////////////////////////
	//
	// Cover checks
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
	begin
		cover(cmd_sent && !r_cmd_busy);
		cover(tx_busy  && tx_start);
		cover(rx_busy  && rx_start);
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_sd_reset) && !$past(r_watchdog_err))
	begin
		cover($fell(cmd_busy));
		cover($fell(tx_busy));
		cover($fell(rx_busy));
		cover($fell(r_cmd_busy));
	end

	////////////////////////////////////////////////////////////////////////
	//
	// "Careless" assumptions
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	// always @(*)
	//	assume(!r_watchdog_err);
`endif
// }}}
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	spicmd.v
// {{{
// Project:	SPI-based SD-Card controller
//
// Purpose:	Issues commands and collects responses from the lower level
//		SPI processor.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2019-2023, Gisselquist Technology, LLC
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
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
`default_nettype	none
// }}}
module	spicmd (
		// {{{
		input	wire		i_clk, i_reset,
		//
		input	wire		i_cmd_stb,
		input	wire	[1:0]	i_cmd_type,
		input	wire	[5:0]	i_cmd,
		input	wire	[31:0]	i_cmd_data,
		output	reg		o_busy,
		//
		output	wire		o_ll_stb,
		output	wire	[7:0]	o_ll_byte,
		input	wire		i_ll_busy,
		//
		input	wire		i_ll_stb,
		input	wire	[7:0]	i_ll_byte,
		//
		output	reg		o_cmd_sent,
		output	reg		o_rxvalid,
		output	reg	[39:0]	o_response
		// }}}
	);

	// Signal declarations
	// {{{
	reg		almost_sent;
	reg	[4:0]	crc_valid_sreg;
	reg		crc_busy;
	reg	[4:0]	crc_bit_counter;
	reg	[39:0]	crc_shift_reg, shift_data;
	reg	[7:0]	crc_byte;
	reg		rx_r1_byte, rx_check_busy, rxvalid;
	reg	[2:0]	rx_counter;
	localparam	CRC_POLYNOMIAL = 7'h09; // Was 8'h12
	reg	[6:0]	next_crc_byte;
	// }}}

	// o_busy
	// {{{
	initial	o_busy = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_busy <= 1'b0;
	else if (!o_busy && i_cmd_stb)
		o_busy <= 1'b1;
	// else if (o_rxvalid)
	else if (rxvalid && !rx_check_busy)
		o_busy <= 1'b0;
	// }}}

	// shift_data
	// {{{
	initial	shift_data = -1;
	always @(posedge i_clk)
	if (!o_busy && i_cmd_stb)
		shift_data <= { 2'b01, i_cmd, i_cmd_data };
	else if (!i_ll_busy)
	begin
		shift_data <= { shift_data[31:0], 8'hff };
		if (crc_valid_sreg[0])
			shift_data[39:32] <= crc_byte;
	end
	// }}}

	assign	o_ll_stb  = o_busy;
	assign	o_ll_byte = shift_data[39:32];

	// o_cmd_sent
	// {{{
	initial	o_cmd_sent = 1'b0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		{ o_cmd_sent, almost_sent } <= 2'b00;
	else if (!o_cmd_sent && !i_ll_busy)
		{ o_cmd_sent, almost_sent } <= { almost_sent, crc_valid_sreg[0] };
	// }}}

	// crc_valid_sreg
	// {{{
	initial	crc_valid_sreg = 5'b10000;
	always @(posedge i_clk)
	if (!o_busy)
		crc_valid_sreg <= 5'b10000;
	else if (!i_ll_busy)
		crc_valid_sreg <= crc_valid_sreg >> 1;
	// }}}

	// crc_busy, crc_bit_counter
	// {{{
	initial crc_busy = 1'b0;
	initial	crc_bit_counter = 20;
	always @(posedge i_clk)
	if (!o_busy)
	begin
		crc_bit_counter <= 20;
		crc_busy <= (i_cmd_stb);
	end else if (crc_busy)
	begin
		crc_bit_counter <= crc_bit_counter - 1;
		crc_busy <= (crc_bit_counter > 1);
	end
	// }}}

	// crc_shift_reg
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		crc_shift_reg <= { 2'b01, i_cmd, i_cmd_data };
	else if (crc_busy)
		crc_shift_reg <= crc_shift_reg << 2;
	// }}}

	// next_crc_byte
	// {{{
	always @(*)
	begin
		next_crc_byte = { crc_byte[6:1], 1'b0 };
		if (crc_byte[7] ^ crc_shift_reg[39])
			next_crc_byte = next_crc_byte ^ CRC_POLYNOMIAL;
		if (next_crc_byte[6] ^ crc_shift_reg[38])
			next_crc_byte = (next_crc_byte<<1) ^ CRC_POLYNOMIAL;
		else
			next_crc_byte = (next_crc_byte<<1);
	end
	// }}}

	// crc_byte
	// {{{
	initial	crc_byte = 0;
	always @(posedge i_clk)
	if (!o_busy)
		crc_byte <= 1;
	else if (crc_busy)
		crc_byte <= { next_crc_byte, 1'b1 };
	// }}}

	// rx_r1_byte, rx_counter, rxvalid, rx_check_busy
	// {{{
	initial	rxvalid = 1'b0;
	initial	rx_counter = 1;
	always @(posedge i_clk)
	if (!o_busy)
	begin
		rx_r1_byte <= 1'b0;
		rx_counter <= (i_cmd_type[1]) ? 5 : 1;
		rx_check_busy <= (i_cmd_type == 2'b01);
		rxvalid <= 1'b0;
	end else if (o_cmd_sent && i_ll_stb)
	begin
		if (!rx_r1_byte)
			rx_r1_byte <= (!i_ll_byte[7]);

		if ((rx_r1_byte || !i_ll_byte[7]) && !rxvalid)
		begin
			rx_counter <= rx_counter - 1;
			rxvalid <= (rx_counter <= 1);
		end

		if (rx_r1_byte && i_ll_byte != 0)
			rx_check_busy <= 1'b0;
	end
	// }}}

	// o_rxvalid
	// {{{
	initial	o_rxvalid = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		o_rxvalid <= 0;
	else if (rxvalid && !rx_check_busy)
		o_rxvalid <= 1;
	// }}}

	// o_response
	// {{{
	initial	o_response = -1;
	always @(posedge i_clk)
	if (!o_busy)
		o_response <= -1;
	else if (i_ll_stb)
	begin
		if (!rx_r1_byte)
			o_response[39:32] <= i_ll_byte;
		else
			o_response[31:0] <= { o_response[23:0], i_ll_byte };
	end
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`ifdef	SPICMD
`define	ASSUME	assume
`else
`define	ASSUME	assert
`endif
	reg		f_past_valid;
	reg [2:0]	f_send_seq;
	reg [3:0]	f_rcv_seq;
	reg [5:0]	f_cmd;
	reg	[31:0]	f_data;
	reg	[1:0]	f_type;
	reg	[39:0]	f_rcv_data;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	always @(*)
	if (!o_cmd_sent)
	begin
		assert(!o_rxvalid);
		assert(!rxvalid || !o_busy);
	end else
		assert(&shift_data);

	always @(*)
	if (!rxvalid)
		assert(!o_rxvalid);

	initial	f_send_seq = 0;
	always @(posedge i_clk)
	if (i_reset || (o_busy && rxvalid && !rx_check_busy))
		f_send_seq <= 0;
	else if (!o_busy)
		f_send_seq <= (i_cmd_stb) ? 1:0;
	else if (o_busy && !o_cmd_sent && (f_send_seq < 7))
		f_send_seq <= f_send_seq + (i_ll_busy ? 0:1);

	always @(posedge i_clk)
	if (!o_busy)
		{ f_cmd, f_data, f_type } <= { i_cmd, i_cmd_data, i_cmd_type };

	always @(*)
	if (i_reset || !o_busy)
	begin
		if (!o_busy)
			assert(f_send_seq == 0);
	end else case(f_send_seq)
	1: begin
		assert(shift_data == { 2'b01, f_cmd, f_data });
		assert(o_busy);
		assert({ o_cmd_sent, almost_sent } == 2'b00);
		assert(crc_valid_sreg == 5'b1_0000);
		end
	2: begin
		assert(shift_data == { f_data, 8'hff });
		assert(o_busy);
		assert({ o_cmd_sent, almost_sent } == 2'b00);
		assert(crc_valid_sreg == 5'b1000);
		end
	3: begin
		assert(shift_data == { f_data[23:0], 16'hffff });
		assert(o_busy);
		assert({ o_cmd_sent, almost_sent } == 2'b00);
		assert(crc_valid_sreg == 5'b100);
		end
	4: begin
		assert(shift_data == { f_data[15:0], 24'hffffff });
		assert(o_busy);
		assert({ o_cmd_sent, almost_sent } == 2'b00);
		assert(crc_valid_sreg == 5'b10);
		end
	5: begin
		assert(shift_data == { f_data[7:0], 32'hffffffff });
		assert(o_busy);
		assert({ o_cmd_sent, almost_sent } == 2'b00);
		assert(crc_valid_sreg == 5'b1);
		end
	6: begin
		assert(shift_data == { crc_byte, 32'hffffffff });
		assert(o_busy);
		assert({ o_cmd_sent, almost_sent } == 2'b01);
		assert(crc_valid_sreg == 5'b0);
		end
	7: begin
		assert(shift_data == 40'hff_ffff_ffff);
		assert(crc_valid_sreg == 5'b0);
		assert(o_cmd_sent);
		end
	endcase

	always @(*)
	if (o_busy)
	begin
		assert(f_send_seq != 0);
		assert(f_rcv_seq != 0);
	end

	always @(*)
		assert(crc_bit_counter <= 20);

	always @(*)
	if (o_busy)
		assert(crc_busy == (crc_bit_counter > 0));

	// Got to give the CRC enough time to work
	always @(*)
	if (!o_cmd_sent)
	begin
		// if (crc_bit_counter > 4)
		//	`ASSUME(f_send_seq < 3);
		if (crc_bit_counter > 2)
			`ASSUME(f_send_seq < 4);
		if (crc_bit_counter > 0)
			`ASSUME(f_send_seq < 5);
	end

	always @(*)
	if (o_busy && !o_cmd_sent)
		assume(i_ll_byte == 8'hff);

	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
	if (o_busy && !o_cmd_sent)
	begin
		assert(f_rcv_seq == 1);
		assert(&o_response);
	end

	initial	f_rcv_seq = 0;
	always @(posedge i_clk)
	if (i_reset)
		f_rcv_seq <= 0;
	else if (!o_busy)
		f_rcv_seq <= (i_cmd_stb) ? 1 : 0;
	else if (o_cmd_sent && i_ll_stb)
	begin
		assert(f_rcv_seq != 0);
		if (!rx_r1_byte)
		begin
			if (!i_ll_byte[7]) casez(f_type)
			2'b00: f_rcv_seq <= 3;	// R1 response
			2'b01: f_rcv_seq <= 2;	// R1b response
			2'b1?: f_rcv_seq <= 4;	// R3/R7 response
			endcase
		end else case(f_rcv_seq)
		1: begin assert(0); end // Should never be here
		2: begin
			if (i_ll_byte != 0)
				f_rcv_seq <= 3;
			end
		3: begin end // We'll stop here with R1b
		4: f_rcv_seq <= f_rcv_seq + 1;
		5: f_rcv_seq <= f_rcv_seq + 1;
		6: f_rcv_seq <= f_rcv_seq + 1;
		7: f_rcv_seq <= f_rcv_seq + 1;
		default: begin end
		endcase
	end

	always @(posedge i_clk)
	if (i_ll_stb)
	begin
		if (!rx_r1_byte)
			f_rcv_data[39:32] <= i_ll_byte;
		else
			f_rcv_data[31:0] <= { f_rcv_data[23:0], i_ll_byte };
	end

	always @(*)
	if (!i_reset && o_busy && f_rcv_seq > 0)
	case(f_rcv_seq)
	1: begin
		assert(!rx_r1_byte);
		assert(rx_counter == ((f_type[1]) ? 5 : 1));
		assert(rx_check_busy == (f_type == 2'b01));
		assert(!rxvalid);
		assert(f_send_seq != 0);
		assert(&o_response[31:0]);
		end
	2: begin
		assert(o_cmd_sent);
		assert(rx_r1_byte);
		assert(rx_counter == 0);
		assert(rx_check_busy);
		assert(rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		// assert(&o_response[31:0]);
		end
	3: begin
		assert(o_cmd_sent);
		assert(rx_r1_byte);
		assert(rx_counter == 0);
		assert(!rx_check_busy);
		assert(rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		// assert(&o_response[31:8]);
		end
	4: begin
		assert(rx_r1_byte);
		assert(rx_counter == 4);
		assert(!rx_check_busy);
		assert(!rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		assert(&o_response[31:0]);
		end
	5: begin
		assert(rx_r1_byte);
		assert(rx_counter == 3);
		assert(!rx_check_busy);
		assert(!rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		assert(o_response[7:0] == f_rcv_data[7:0]);
		assert(&o_response[31:8]);
		end
	6: begin
		assert(rx_r1_byte);
		assert(rx_counter == 2);
		assert(!rx_check_busy);
		assert(!rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		assert(o_response[15:0] == f_rcv_data[15:0]);
		assert(&o_response[31:16]);
		end
	7: begin
		assert(rx_r1_byte);
		assert(rx_counter == 1);
		assert(!rx_check_busy);
		assert(!rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		assert(o_response[23:0] == f_rcv_data[23:0]);
		assert(&o_response[31:24]);
		end
	8: begin
		assert(rx_r1_byte);
		assert(rx_counter == 0);
		assert(!rx_check_busy);
		assert(rxvalid);
		assert(o_response[39:32] == f_rcv_data[39:32]);
		end
	default: assert(f_rcv_seq <= 8);
	endcase

	always @(*)
		assert(rxvalid == (rx_counter == 0));
	////////////////////////////////////////////////////////////////////////
	//
	// Cover properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
	begin
		cover(o_cmd_sent);
		cover(f_rcv_seq == 1);
		cover(f_rcv_seq == 2);
		cover(f_rcv_seq == 3);
		cover(f_rcv_seq == 4);
		cover(f_rcv_seq == 5);
		cover(f_rcv_seq == 6);
		cover(f_rcv_seq == 7);
		cover(f_rcv_seq == 8);
	end

	always @(posedge i_clk)
		cover(o_rxvalid && f_cmd == 0 && f_data == 0);

	always @(posedge i_clk)
		cover(o_rxvalid && f_cmd == 0 && f_data == 0
			&& crc_byte == 8'h95);
`endif
// }}}
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	spitxdata.v
// {{{
// Project:	SPI-based SD-Card controller
//
// Purpose:	To handle all of the processing associated with sending data
//		from a memory to our lower-level SPI processor.
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
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
`default_nettype	none
// }}}
module spitxdata #(
		// {{{
		parameter	DW = 32, AW = 8, RDDELAY = 2,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		localparam	CRC_POLYNOMIAL = 16'h1021
		// }}}
	) (
		// {{{
		input	wire	i_clk, i_reset,
		//
		input	wire		i_start,
		input	wire	[3:0]	i_lgblksz,
		input	wire		i_fifo,
		output	reg		o_busy,
		//
		output	reg 		o_read,
		output	reg [AW-1:0]	o_addr,
		input	wire [DW-1:0]	i_data,
		//
		input	wire		i_ll_busy,
		output	reg		o_ll_stb,
		output	wire [7:0]	o_ll_byte,
		//
		input	wire		i_ll_stb,
		input	wire [7:0]	i_ll_byte,
		//
		output	reg 		o_rxvalid,
		output	reg [7:0]	o_response
		// }}}
	);

	// Signal declarations
	// {{{
	reg	[RDDELAY-1:0]		rdvalid;
	reg	[8+DW-1:0]		gearbox;
	reg	[1+(DW/8)-1:0]		fill;
	reg				crc_flag, crc_stb, data_read,
					all_mem_read;

	reg				lastaddr, data_sent, received_token,
					all_idle;
	reg				crc_active;
	reg [$clog2(1+DW/2)-1:0]	crc_fill;
	(* keep *) reg	[DW-1:0]	crc_gearbox;
	reg	[15:0]			crc_data;

	reg				token;
	reg	[2:0]			r_lgblksz_m3;
	reg	[15:0]			next_crc_data;
	// }}}

	// token
	// {{{
	always @(*)
		token = (data_sent && i_ll_stb && i_ll_byte[0] &&!i_ll_byte[4]);
	// }}}

	// o_busy
	// {{{
	initial	o_busy = 0;
	always @(posedge i_clk)
	if (i_reset)
		o_busy <= 0;
	else if (!o_busy)
		o_busy <= i_start;
	else if (all_idle && i_ll_stb && (&i_ll_byte))
		o_busy <= 0;
	// }}}

	// o_rxvalid, received_token
	// {{{
	initial	o_rxvalid = 0;
	initial	received_token = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		{ received_token, o_rxvalid } <= 0;
	else if (token && !received_token)
		{ received_token, o_rxvalid } <= 2'b11;
	else
		o_rxvalid <= 0;
	// }}}

	// all_idle
	// {{{
	initial	all_idle = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		all_idle <= 0;
	else if (received_token && i_ll_stb && (&i_ll_byte))
		all_idle <= 1'b1;
	// }}}

	// o_response
	// {{{
	always @(posedge i_clk)
	if (token)
		o_response <= i_ll_byte;
	// }}}

	// o_read & o_addr
	// 0: arbited read request
	// 1: read data
	// 2: muxed read data

	// rdvalid
	// {{{
	initial	rdvalid = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		rdvalid <= 0;
	else
		rdvalid <= { rdvalid[RDDELAY-2:0], o_read };
	// }}}

	// o_ll_stb
	// {{{
	initial	o_ll_stb = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		o_ll_stb <= 0;
	else if (rdvalid[RDDELAY-1])
		o_ll_stb <= 1;
	else if (data_read && !fill[4])
		o_ll_stb <= 0;
	// }}}

	// fill
	// {{{
	initial	fill = 0;
	always @(posedge i_clk)
	begin
		if (!o_ll_stb && rdvalid[RDDELAY-1])
		begin
			if (OPT_LITTLE_ENDIAN)
				gearbox <= { i_data, 8'hfe };
			else
				gearbox <= { 8'hfe, i_data };
			fill <= 5'h1f;
		end else if (rdvalid[RDDELAY-1])
		begin
			if (OPT_LITTLE_ENDIAN)
				gearbox <= { i_data, gearbox[7:0] };
			else
				gearbox <= { gearbox[DW+8-1:DW], i_data };
			fill <= 5'h1f;
		end else if (crc_stb)
		begin
			if (OPT_LITTLE_ENDIAN)
				gearbox <= { 16'hff, crc_data[7:0], crc_data[15:8], gearbox[7:0] };
			else
				gearbox <= { gearbox[DW+8-1:DW], crc_data, 16'hff };
			fill[3:0] <= 4'hc;
		end else if (o_ll_stb && !i_ll_busy)
		begin
			if (OPT_LITTLE_ENDIAN)
				gearbox <= { 8'hff, gearbox[DW+8-1:8] };
			else
				gearbox <= { gearbox[DW-1:0], 8'hff };
			fill <= fill << 1;
		end

		if (!o_busy)
		begin
			if (OPT_LITTLE_ENDIAN)
				gearbox[7:0] <= 8'hfe;	// Start token
			else
				gearbox[39:32] <= 8'hfe;	// Start token
		end

		if (i_reset)
			fill <= 0;
		else if (!o_busy)
			fill <= (i_start) ? 5'h10 : 0;
	end
	// }}}

	generate if (OPT_LITTLE_ENDIAN)
	begin : GEN_LILEND
		assign	o_ll_byte = gearbox[7:0];
	end else begin : GEN_BIG_ENDIAN
		assign	o_ll_byte = gearbox[39:32];
	end endgenerate

	// crc_stb, o_read
	// {{{
	initial	{ crc_stb, o_read } = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		{ crc_stb, o_read } <= 0;
	else if (!fill[3] || (!fill[2] && (!o_ll_stb || !i_ll_busy)))
	begin
		{ crc_stb, o_read } <= 0;
		if (!o_read && rdvalid == 0 && !data_read)
		begin
			if (!all_mem_read)
				o_read <= 1;
			else
				crc_stb <= (!crc_flag)&&(!crc_stb);
		end
	end else
		{ crc_stb, o_read } <= 0;
	// }}}

	// o_addr
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		o_addr <= { i_fifo, {(AW-1){1'b0}} };
	else if (o_read && !lastaddr)
		o_addr[AW-2:0] <= o_addr[AW-2:0] + 1;
	// }}}

	// all_mem_read
	// {{{
	initial	all_mem_read = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		all_mem_read <= 0;
	else if (o_read && lastaddr)
		all_mem_read <= 1;
	// }}}

	// crc_flag
	// {{{
	initial	crc_flag = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		crc_flag <= 0;
	else if (crc_stb)
		crc_flag <= 1;
	// }}}

	// data_read
	// {{{
	always @(*)
		data_read = crc_flag;
	// }}}

	// data_sent
	// {{{
	initial	data_sent = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		data_sent <= 1'b0;
	else if (data_read && !fill[3] && o_ll_stb && !i_ll_busy)
		data_sent <= 1'b1;
	// }}}

	// r_lgblksz_m3, lastaddr
	// {{{
	initial	r_lgblksz_m3 = 0;
	initial	lastaddr = 0;
	always @(posedge i_clk)
	if (!o_busy)
	begin
		lastaddr <= (i_lgblksz < 4);
		// Verilator lint_off WIDTH
		r_lgblksz_m3 <= i_lgblksz-3;
		// Verilator lint_on WIDTH
	end else if (o_read && !lastaddr)
	begin
		case(r_lgblksz_m3)
		0: begin end // assert(lastaddr);		//   8 bytes
		1: lastaddr <= (&o_addr[1:1]);	//  16 bytes
		2: lastaddr <= (&o_addr[2:1]);	//  32 bytes
		3: lastaddr <= (&o_addr[3:1]);	//  64 bytes
		4: lastaddr <= (&o_addr[4:1]);	// 128 bytes
		5: lastaddr <= (&o_addr[5:1]);	// 256 bytes
		default: lastaddr <= (&o_addr[6:1]);	// 512 bytes
		endcase
	end
	// }}}

	////////////////////////////////////////////////////////////////////////
	//
	// CRC calculation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// crc_fill
	// {{{
	initial	crc_fill = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
	begin
		crc_fill <= 0;
		crc_active <= 0;
	end else if (crc_active || rdvalid[RDDELAY-1])
	begin
		// Verilator lint_off WIDTH
		crc_fill <= crc_fill - (crc_active ? 1:0)
					+ (rdvalid[RDDELAY-1] ? (DW/2):0);
		// Verilator lint_on WIDTH
		if (rdvalid[RDDELAY-1])
			crc_active <= 1;
		else
			crc_active <= (crc_fill > 1);
	end
	// }}}

	// crc_gearbox
	// {{{
	always @(posedge i_clk)
	if (!crc_active)
	begin
		if (OPT_LITTLE_ENDIAN)
			crc_gearbox <= { i_data[7:0], i_data[15:8], i_data[23:16], i_data[31:24] };
		else
			crc_gearbox <= i_data;
	end else
		crc_gearbox <= { crc_gearbox[DW-3:0], 2'b00 };
	// }}}

	// next_crc_data
	// {{{
	always @(*)
	begin
		next_crc_data = crc_data << 1;;

		if (crc_data[15] ^ crc_gearbox[31])
			next_crc_data = next_crc_data ^ CRC_POLYNOMIAL;

		if (next_crc_data[15] ^ crc_gearbox[30])
			next_crc_data = (next_crc_data << 1) ^ CRC_POLYNOMIAL;
		else
			next_crc_data = (next_crc_data << 1);
	end
	// }}}

	// crc_data
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		crc_data <= 0;
	else if (crc_active)
		crc_data <= next_crc_data;
	// }}}
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`ifdef	SPITXDATA
`define	ASSUME	assume
`else
`define	ASSUME	assert
`endif

	reg		f_past_valid;
	reg	[3:0]	f_lgblksz;
	wire	[3:0]	f_lgblksz_m3;


	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	// Reset check(s)
	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(!o_ll_stb);
		assert(!o_busy);
		assert(rdvalid  == 0);
		assert(!o_rxvalid);
	end

	always @(posedge i_clk)
	if (!f_past_valid || !$past(o_busy))
	begin
		assert(rdvalid  == 0);
		assert(!o_rxvalid);
		if (!f_past_valid || $past(i_reset))
			assert(fill == 0);
		else if ($past(i_start))
			assert(fill == 5'h10);
		else
			assert(fill == 0);
		assert(crc_fill == 0);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Data assumptions
	//
	reg	[6:0]	f_last_read;

	initial	f_last_read = 0;
	always @(posedge i_clk)
	begin
		f_last_read <= f_last_read << 1;
		if (o_ll_stb && !i_ll_busy)
			f_last_read[0] <= 1;
	end

`ifdef	VERIFIC
	always @(*)
		`ASSUME($onehot0({f_last_read, (o_ll_stb && !i_ll_busy)}));
`else
	always @(*)
	case({f_last_read, (o_ll_stb && !i_ll_busy)})
	8'h00: begin end
	8'h01: begin end
	8'h02: begin end
	8'h04: begin end
	8'h08: begin end
	8'h10: begin end
	8'h20: begin end
	8'h40: begin end
	8'h80: begin end
	default: `ASSUME(0);
	endcase
`endif


	always @(*)
	if (i_start)
	begin
		`ASSUME(i_lgblksz <= 9);
		`ASSUME(i_lgblksz  > 2);
	end

	always @(*)
	if (crc_fill > 1)
	begin
		assert(o_ll_stb);
		assert(!crc_stb);
	end

	always @(*)
	if (o_read || rdvalid != 0)
		assert(!o_ll_stb || i_ll_busy);

	////////////////////////////////////////////////////////////////////////
	//
	// Induction assertions
	//

	always @(*)
	if (rdvalid != 0)
		assert(crc_fill <= 1);

	always @(*)
	if (o_busy)
		assert(crc_active == (crc_fill > 0));

	always @(*)
	if (o_read)
		assert(rdvalid == 0);
	else
`ifdef	VERIFIC
		assert($onehot0(rdvalid));
`else
	begin
		assert((rdvalid == 0)
			||(rdvalid == 1)
			||(rdvalid == 2)
			||(rdvalid == 4));
	end
`endif

	always @(*)
	if (o_busy && all_idle)
		assert(received_token);

	always @(*)
	if (o_busy && received_token)
		assert(data_sent);

	always @(*)
	if (o_busy && data_sent)
	begin
		assert(data_read);
		assert(!crc_stb);
		assert(!crc_active);
	end

	always @(*)
	if (crc_fill > 5'h0c)
		assert(&fill[4:1]);
	else if (crc_fill > 4)
		assert(&fill[4:2]);
	else if (crc_fill > 0)
		assert(&fill[4:3]);

	always @(*)
		assert(crc_fill <= 5'h10);
	always @(*)
	if (o_busy && data_read)
	begin
		assert(crc_flag);
		assert(!crc_stb);
		assert(!crc_active);
	end

	always @(*)
	if (o_busy && crc_flag)
	begin
		assert(!crc_stb);
		assert(all_mem_read);
		assert(!crc_active);
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset) && $past(o_busy))
		assert($rose(crc_flag) == $past(crc_stb));

	always @(*)
	if (o_busy && all_mem_read)
		assert(lastaddr);

	always @(*)
	if (o_busy && !lastaddr)
		assert(!all_mem_read);

	always @(posedge i_clk)
	if (!o_busy)
		f_lgblksz <= i_lgblksz;

	assign	f_lgblksz_m3 = f_lgblksz - 3;

	always @(*)
	if(o_busy)
		assert(f_lgblksz >= 3);

	always @(*)
	if (o_busy)
		assert(f_lgblksz_m3[2:0] == r_lgblksz_m3);

	always @(*)
	if (o_busy)
	case(r_lgblksz_m3)
	3'h0: assert(lastaddr);			//   8 bytes
	3'h1: assert(lastaddr == (&o_addr[1:0]));	//  16 bytes
	3'h2: assert(lastaddr == (&o_addr[2:0]));	//  32 bytes
	3'h3: assert(lastaddr == (&o_addr[3:0]));	//  64 bytes
	3'h4: assert(lastaddr == (&o_addr[4:0]));	// 128 bytes
	3'h5: assert(lastaddr == (&o_addr[5:0]));	// 256 bytes
	default: assert(lastaddr == (&o_addr[AW-2:0]));
	endcase

	always @(*)
	if (o_ll_stb && !data_sent)
		assert(fill[4]);

	always @(*)
	if (fill != 0)
		assert(fill[DW/8]);

	genvar	k;

	generate for(k=DW/8; k>0; k=k-1)
	begin
		always @(*)
		if ((fill != 0) && !fill[k])
			assert(fill[k-1:0]==0);
	end endgenerate

	always @(posedge i_clk)
	if (f_past_valid && $past(crc_stb))
		assert(!crc_stb);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal property section
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	SPITXDATA
	reg	[5:0]		f_read_seq;
	(* anyseq *) reg	f_read_check;
	reg	[DW-1:0]	f_read_data;

	always @(posedge i_clk)
	if (rdvalid[RDDELAY-1])
		f_read_data <= i_data;

	always @(*)
	if (f_read_seq != 0)
		assume(!f_read_check);

	initial	f_read_seq = 0;
	always @(posedge i_clk)
	if (!o_busy)
		f_read_seq <= 0;
	else if (f_read_check && o_read)
		f_read_seq <= 1;
	else if (rdvalid != 0)
	begin
		if (rdvalid[RDDELAY-1])
			f_read_seq <= 2;
	end else if (!i_ll_busy)
		f_read_seq <= (f_read_seq << 1);

	always @(*)
	if (i_reset || !o_busy || (f_read_seq == 0))
	begin end
	else case(f_read_seq)
	6'h01: begin
		assert(rdvalid != 0);
		assert(fill == 5'h10);
		end
	6'h02: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h1f);
		if (OPT_LITTLE_ENDIAN)
			assert(gearbox[DW+8-1:8] == f_read_data);
		else
			assert(gearbox[DW-1:0] == f_read_data);
		end
	6'h04: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h1e);
		if (OPT_LITTLE_ENDIAN)
			assert(gearbox[DW-1:0] == f_read_data);
		else
			assert(gearbox[8+DW-1:8] == f_read_data);
		end
	6'h08: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h1c);
		if (OPT_LITTLE_ENDIAN)
			assert(gearbox[DW-8-1:0] == f_read_data[31:8]);
		else
			assert(gearbox[8+DW-1:16] == f_read_data[23:0]);
		end
	6'h10: begin
		assert(o_ll_stb);
		assert(rdvalid == 0);
		assert(fill == 5'h18);
		if (OPT_LITTLE_ENDIAN)
			assert(gearbox[DW-16-1:0] == f_read_data[31:16]);
		else
			assert(gearbox[8+DW-1:24] == f_read_data[15:0]);
		end
	6'h20: begin
		assert(o_ll_stb);
		assert(fill[4]);
		if (OPT_LITTLE_ENDIAN)
			assert(gearbox[DW-24-1:0] == f_read_data[31:24]);
		else
			assert(gearbox[8+DW-1:DW] == f_read_data[7:0]);
		end
	default:
`ifdef	VERIFIC
		assert($onehot0(f_read_seq));
`else
		if (f_read_seq)
			assert(0);
`endif
	endcase
`endif
	////////////////////////////////////////////////////////////////////////
	//
	// Cover properties
	//
	always @(posedge i_clk)
	begin
		cover(f_lgblksz == 4 && o_rxvalid);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h00);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h01);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h02);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h03);
		cover(o_busy && f_lgblksz == 4 && lastaddr);
		cover(o_busy && f_lgblksz == 4 && crc_stb);
		cover(o_busy && f_lgblksz == 4 && all_mem_read);
		cover(o_busy && f_lgblksz == 4 && crc_flag);
		cover(o_busy && f_lgblksz == 4 && data_read);
		cover(o_busy && f_lgblksz == 4 && data_sent);
		cover(o_busy && f_lgblksz == 4 && received_token);
		cover(o_busy && f_lgblksz == 4 && all_idle);
		cover(!o_busy && f_lgblksz == 4 && all_idle);
	end
`endif
// }}}
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	spirxdata.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	To handle all of the processing associated with receiving data
//		from an SD card via the lower-level SPI processor, and then
//	issuing write commands to our internal memory store (external to this
//	module).
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2019-2023, Gisselquist Technology, LLC
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
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
`default_nettype	none
// }}}
module spirxdata #(
		// {{{
		parameter	DW = 32, AW = 8,
		parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		localparam	CRC_POLYNOMIAL = 16'h1021
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		//
		input	wire		i_start,
		input	wire	[3:0]	i_lgblksz,
		input	wire		i_fifo,
		output	reg		o_busy,
		//
		input	wire		i_ll_stb,
		input	wire [7:0]	i_ll_byte,
		//
		output	reg 		o_write,
		output	reg [AW-1:0]	o_addr,
		output	reg [DW-1:0]	o_data,
		//
		output	reg 		o_rxvalid,
		output	reg [7:0]	o_response
		// }}}
	);

	// Signal declarations
	// {{{
	reg		error_token, start_token, token, received_token, done,
			lastaddr;
	reg		all_mem_written, lastdata;

	reg	[1:0]	crc_byte;
	reg	[2:0]	r_lgblksz_m3;
	reg	new_data_byte;
	reg	[3:0]	crc_fill;
	reg	[7:0]	crc_gearbox;
	reg	[15:0]	next_crc_data;
	reg	[15:0]	crc_data;
	reg		crc_err, crc_active;
	reg	[2:0]	fill;
	reg	[23:0]	gearbox;
	reg	[15:0]	first_crc_data;
	// }}}

	// error_token
	// {{{
	always @(*)
	begin
		error_token = 0;

		if (i_ll_byte[7:4] == 0)
			error_token = 1;
		if (!i_ll_stb || received_token)
			error_token = 0;
	end
	// }}}

	// start_token
	// {{{
	always @(*)
	begin
		start_token = 0;

		if (!i_ll_byte[0])
			start_token = 1;
		if (!i_ll_stb || received_token)
			start_token = 0;
	end
	// }}}

	// token
	// {{{
	always @(*)
		token = (start_token || error_token);
	// }}}

	// done
	// {{{
	always @(*)
		done = (i_ll_stb && (crc_byte>1));
	// }}}

	// received_token
	// {{{
	initial	received_token = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		received_token <= 0;
	else if (token)
		received_token <= 1;
	// }}}

	// o_busy
	// {{{
	initial	o_busy = 0;
	always @(posedge i_clk)
	if (i_reset)
		o_busy <= 0;
	else if (!o_busy)
		o_busy <= i_start;
	else if (error_token || done)
		o_busy <= 0;
	// }}}

	// o_rxvalid
	// {{{
	initial	o_rxvalid = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		o_rxvalid <= 0;
	else if (error_token || done)
		o_rxvalid <= 1;
	// }}}

	// o_response
	// {{{
	initial	o_response = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		o_response <= 0;
	else if (error_token)
		o_response <= i_ll_byte;
	else if (done)
		o_response <= (crc_err || (crc_data[7:0] != i_ll_byte)) ? 8'h10 : 0;
	// }}}

	// o_write
	// {{{
	initial	o_write = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		o_write <= 0;
	else if (received_token && !all_mem_written)
		o_write <= (&fill) && i_ll_stb;
	else
		o_write <= 0;
	// }}}

	// o_data
	// {{{
	initial	o_data = 0;
	always @(posedge i_clk)
	if (received_token && !all_mem_written)
	begin
		if (OPT_LITTLE_ENDIAN)
			o_data <= { i_ll_byte, gearbox };
		else
			o_data <= { gearbox, i_ll_byte };
	end
	// }}}

	// o_addr
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		o_addr <= { i_fifo, {(AW-1){1'b0}} };
	else if (o_write && !lastaddr)
		o_addr <= o_addr + 1;
	// }}}

	// fill
	// {{{
	initial	fill = 0;
	always @(posedge i_clk)
	begin
		if (i_ll_stb)
		begin
			if (OPT_LITTLE_ENDIAN)
				gearbox <= { i_ll_byte, gearbox[23:8] };
			else
				gearbox <= { gearbox[15:0], i_ll_byte };
		end

		if (!o_busy || !received_token)
			fill <= 0;
		else if ((&fill) && i_ll_stb)
			fill <= 0;
		else if (i_ll_stb)
			fill <= { fill[1:0], 1'b1 };
	end
	// }}}

	// lastdata
	// {{{
	always @(posedge i_clk)
	if (!o_busy)
		lastdata <= 0;
	else if (!lastdata)
		lastdata <= (lastaddr && (&fill));
	// }}}

	// all_mem_written
	// {{{
	initial	all_mem_written = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		all_mem_written <= 0;
	else if (o_write && lastaddr)
		all_mem_written <= 1;
	// }}}

	// crc_byte
	// {{{
	initial	crc_byte = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		crc_byte <= 0;
	else if (i_ll_stb && lastaddr && lastdata)
		crc_byte <= crc_byte + 1;
	// }}}

	// lastaddr, r_lgblksz_m3
	// {{{
	initial	r_lgblksz_m3 = 0;
	initial	lastaddr = 0;
	always @(posedge i_clk)
	if (!o_busy)
	begin
		lastaddr <= (i_lgblksz < 4);
		// Verilator lint_off WIDTH
		r_lgblksz_m3 <= i_lgblksz-3;
		// Verilator lint_on WIDTH
	end else if (o_write && !lastaddr)
	begin
		case(r_lgblksz_m3)
		0: lastaddr <= 1;		//   8 bytes
		1: lastaddr <= (&o_addr[1:1]);	//  16 bytes
		2: lastaddr <= (&o_addr[2:1]);	//  32 bytes
		3: lastaddr <= (&o_addr[3:1]);	//  64 bytes
		4: lastaddr <= (&o_addr[4:1]);	// 128 bytes
		5: lastaddr <= (&o_addr[5:1]);	// 256 bytes
		default: lastaddr <= (&o_addr[6:1]);	// 512 bytes
		endcase
	end
	// }}}

	////////////////////////////////////////////////////////////////////////
	//
	// CRC calculation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// new_data_byte
	// {{{
	always @(*)
		new_data_byte = (i_ll_stb && !all_mem_written);
	// }}}

	// crc_fill, crc_active
	// {{{
	initial	crc_fill   = 0;
	initial	crc_active = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy || !received_token)
	begin
		crc_fill <= 0;
		crc_active <= 0;
	end else if (crc_active || new_data_byte)
	begin
		// Verilator lint_off WIDTH
		crc_fill <= crc_fill - (crc_active ? 1:0)
					+ (new_data_byte ? 4:0);
		// Verilator lint_on WIDTH
		if (new_data_byte)
			crc_active <= 1;
		else
			crc_active <= (crc_fill > 1);
	end
	// }}}

	// crc_gearbox
	// {{{
	always @(posedge i_clk)
	if (!crc_active)
		crc_gearbox <= i_ll_byte;
	else
		crc_gearbox <= { crc_gearbox[8-3:0], 2'b00 };
	// }}}


	// first_crc_data, next_crc_data
	// {{{
	always @(*)
	begin
		first_crc_data = crc_data << 1;;

		if (crc_data[15] ^ crc_gearbox[7])
			first_crc_data = first_crc_data ^ CRC_POLYNOMIAL;

		if (first_crc_data[15] ^ crc_gearbox[6])
			next_crc_data = (first_crc_data << 1) ^ CRC_POLYNOMIAL;
		else
			next_crc_data = (first_crc_data << 1);
	end
	// }}}

	// crc_data
	// {{{
	initial	crc_data = 0;
	always @(posedge i_clk)
	if (!o_busy)
		crc_data <= 0;
	else if (crc_active)
		crc_data <= next_crc_data;
	// }}}

	// crc_err
	// {{{
	initial	crc_err = 0;
	always @(posedge i_clk)
	if (i_reset || !o_busy)
		crc_err <= 0;
	else if (i_ll_stb && (crc_byte == 1))
		crc_err <= (crc_data[15:8] != i_ll_byte);
	// else if (i_ll_stb && (crc_byte == 2)
	//	crc_err <= (crc_data[7:0] != i_ll_byte);
	// }}}
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`ifdef	SPIRXDATA
`define	ASSUME	assume
`else
`define	ASSUME	assert
`endif

	reg		f_past_valid;
	reg	[3:0]	f_lgblksz;
	wire	[3:0]	f_lgblksz_m3;
	reg		f_fifo;


	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	// Reset check(s)
	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(!o_busy);
		assert(o_write  == 0);
	end

	always @(posedge i_clk)
	if (!f_past_valid || !$past(o_busy))
	begin
		assert(!o_rxvalid);
		assert(crc_fill == 0);
	end

	always @(*)
	if (!o_busy)
		assert(!o_write);

	////////////////////////////////////////////////////////////////////////
	//
	// Data assumptions
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	[6:0]	f_last_read;

	initial	f_last_read = 0;
	always @(posedge i_clk)
	begin
		f_last_read <= f_last_read << 1;
		if (i_ll_stb)
			f_last_read[0] <= 1;
	end

`ifdef	VERIFIC
	always @(*)
		`ASSUME($onehot0({f_last_read, i_ll_stb}));
`else
	always @(*)
	case({f_last_read, i_ll_stb})
	8'h00: begin end
	8'h01: begin end
	8'h02: begin end
	8'h04: begin end
	8'h08: begin end
	8'h10: begin end
	8'h20: begin end
	8'h40: begin end
	8'h80: begin end
	default: `ASSUME(0);
	endcase
`endif

	always @(*)
	if (!o_busy && i_start)
	begin
		`ASSUME(i_lgblksz <= 9);
		`ASSUME(i_lgblksz >= 3);
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Induction assertions
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(*)
	if (o_busy)
		assert(crc_active == (crc_fill > 0));

	always @(*)
	if (o_write)
		assert(fill == 0);

	always @(*)
	if (o_busy && crc_byte > 0)
	begin
		assert(lastaddr);
		assert(all_mem_written || o_write);
		assert(lastdata);
	end

	always @(*)
	if (o_busy && all_mem_written)
	begin
		assert(lastaddr);
		assert(lastdata);
	end

	always @(*)
	if (o_busy && lastdata)
	begin
		assert(lastaddr);
		assert((&fill) || o_write || all_mem_written);
	end

	always @(posedge i_clk)
	if (!o_busy)
	begin
		f_lgblksz <= i_lgblksz;
		f_fifo <= i_fifo;
	end

	assign	f_lgblksz_m3 = f_lgblksz - 3;

	always @(*)
	if (o_busy)
	begin
		assert(f_lgblksz >= 3);
		assert(f_lgblksz <= 9);

		assert(o_addr[AW-1] == f_fifo);

		if (!received_token)
		begin
			assert((f_lgblksz == 3) || !lastaddr);
			assert(o_addr[AW-2:0] == 0);
			assert(fill == 0);
		end
	end

	always @(*)
	if (o_busy)
		assert(f_lgblksz_m3[2:0] == r_lgblksz_m3);

	always @(*)
	if (o_busy)
	case(r_lgblksz_m3)
	3'h0: assert(lastaddr);			//   8 bytes
	3'h1: assert(lastaddr == (&o_addr[1:0]));	//  16 bytes
	3'h2: assert(lastaddr == (&o_addr[2:0]));	//  32 bytes
	3'h3: assert(lastaddr == (&o_addr[3:0]));	//  64 bytes
	3'h4: assert(lastaddr == (&o_addr[4:0]));	// 128 bytes
	3'h5: assert(lastaddr == (&o_addr[5:0]));	// 256 bytes
	default: assert(lastaddr == (&o_addr[AW-2:0]));
	endcase

	always @(*)
	if (o_busy)
	case(r_lgblksz_m3)
	3'h0: assert(o_addr[AW-2:1] == 0);	//   8 bytes
	3'h1: assert(o_addr[AW-2:2] == 0);	//  16 bytes
	3'h2: assert(o_addr[AW-2:3] == 0);	//  32 bytes
	3'h3: assert(o_addr[AW-2:4] == 0);	//  64 bytes
	3'h4: assert(o_addr[AW-2:5] == 0);	// 128 bytes
	3'h5: assert(o_addr[AW-2:6] == 0);	// 256 bytes
	default: begin end // assert(lastaddr == (&o_addr[AW-2:0]));
	endcase

	always @(*)
	if (fill != 0)
	begin
		assert(fill[0]);
		if (!fill[1])
			assert(fill[2] == 0);
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CRC checks and properties
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
		assert(crc_fill <= 4);

	always @(*)
	if (o_busy && !received_token)
	begin
		assert(!crc_active);
		assert(crc_fill == 0);
		assert(crc_data == 0);
	end

	always @(*)
	if (crc_active)
	begin
		if (crc_data == 0 && crc_gearbox[7:6] == 0)
			assert(next_crc_data == 0);

		if ((crc_data[15] ^ crc_gearbox[7])
			&&((crc_data[14] ^ crc_gearbox[6])==0))
			assert(next_crc_data == ({ crc_data[13:0], 2'b00 } ^ { CRC_POLYNOMIAL[14:0], 1'b0 }));

		if (((crc_data[15] ^ crc_gearbox[7])==0)
			&&(crc_data[14] ^ crc_gearbox[6]))
			assert(next_crc_data == ({ crc_data[13:0], 2'b00 } ^ CRC_POLYNOMIAL[14:0]));
	end


/*
	reg	[5:0]		f_read_seq;
	(* anyseq *) reg	f_read_check;
	reg	[DW-1:0]	f_read_data;

	always @(posedge i_clk)
	if (rdvalid[RDDELAY-1])
		f_read_data <= i_data;

	always @(*)
	if (f_read_seq != 0)
		assume(!f_read_check);

	initial	f_read_seq = 0;
	always @(posedge i_clk)
	if (!o_busy)
		f_read_seq <= 0;
	else if (f_read_check && o_read)
		f_read_seq <= 1;
	else if (rdvalid != 0)
	begin
		if (rdvalid[RDDELAY-1])
			f_read_seq <= 2;
	end else if (!i_ll_busy)
		f_read_seq <= (f_read_seq << 1);

	always @(*)
	if (i_reset || !o_busy || (f_read_seq == 0))
	begin end
	else case(f_read_seq)
	6'h01: begin
		assert(rdvalid != 0);
		assert(fill == 5'h10);
		end
	6'h02: begin
		assert(rdvalid == 0);
		assert(fill == 5'h1f);
		assert(gearbox[DW-1:0] == f_read_data);
		end
	6'h04: begin
		assert(rdvalid == 0);
		assert(fill == 5'h1e);
		assert(gearbox[8+DW-1:8] == f_read_data);
		end
	6'h08: begin
		assert(rdvalid == 0);
		assert(fill == 5'h1c);
		assert(gearbox[8+DW-1:16] == f_read_data[23:0]);
		end
	6'h10: begin
		assert(rdvalid == 0);
		assert(fill == 5'h18);
		assert(gearbox[8+DW-1:24] == f_read_data[15:0]);
		end
	6'h20: begin
		assert(fill[4]);
		assert(gearbox[8+DW-1:DW] == f_read_data[7:0]);
		end
	default: assert($onehot0(f_read_seq));
	endcase
*/
	// }}}
`ifdef	SPIRXDATA
	////////////////////////////////////////////////////////////////////////
	//
	// Cover properties
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	//
	reg	cvr_packet_received;

	always @(*)
	if (!received_token)
	begin
		cover(error_token);
		cover(start_token);
	end

	initial	cvr_packet_received = 0;
	always @(posedge i_clk)
	if (i_reset || error_token)
		cvr_packet_received <= 0;
	else if (o_rxvalid)
		cvr_packet_received <= 1;

	always @(posedge i_clk)
	begin
		cover(o_rxvalid);
		cover(o_rxvalid && all_mem_written);
		cover(o_rxvalid && f_lgblksz == 4 && all_mem_written);
		cover(o_rxvalid && f_lgblksz == 4 && o_response == 0);
		cover(o_rxvalid && f_lgblksz == 4 && o_response == 0 && all_mem_written);

		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h01);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h02);
		cover(o_busy && f_lgblksz == 4 && o_addr == 8'h03);
		cover(o_busy && crc_byte == 0);
		cover(o_busy && crc_byte == 1);
		cover(o_busy && crc_byte == 2);
		cover(o_busy && crc_byte == 1 && i_ll_stb);
		cover(o_busy && crc_byte == 2 && i_ll_stb);

		cover(cvr_packet_received && !o_busy);
	end
	// }}}
`endif	// SPIRXDATA
`endif	// FORMAL
// }}}
endmodule
////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	llsdspi.v
// {{{
// Project:	SPI-based SD-Card controller
//
// Purpose:	This file implements the "lower-level" interface to the
//		SD-Card controller.  Specifically, it turns byte-level
//	interaction requests into SPI bit-wise interactions.  Further, it
//	handles the request and grant for the SPI wires (i.e., it requests
//	the SPI port by pulling o_cs_n low, and then waits for i_bus_grant
//	to be true before continuing.).  Finally, the speed/clock rate of the
//	communication is adjustable as a division of the current clock rate.
//
//	i_speed
//		This is the number of clocks (minus one) between SPI clock
//		transitions.  Hence a '0' (not tested, doesn't work) would
//		result in a SPI clock that alternated on every input clock
//		equivalently dividing the input clock by two, whereas a '1'
//		would divide the input clock by four.
//
//		In general, the SPI clock frequency will be given by the
//		master clock frequency divided by twice this number plus one.
//		In other words,
//
//		SPIFREQ=(i_clk FREQ) / (2*(i_speed+1))
//
//	i_stb
//		True if the master controller is requesting to send a byte.
//		This will be ignored unless o_idle is false.
//
//	i_byte
//		The byte that the master controller wishes to send across the
//		interface.
//
//	(The external SPI interface)
//
//	o_stb
//		Only true for one clock--when a byte is valid coming in from the
//		interface, this will be true for one clock (a strobe) indicating
//		that a valid byte is ready to be read.
//
//	o_byte
//		The value of the byte coming in.
//
//	o_idle
//		True if this low-level device handler is ready to accept a
//		byte from the incoming interface, false otherwise.
//
//	i_bus_grant
//		True if the SPI bus has been granted to this interface, false
//		otherwise.  This has been placed here so that the interface of
//		the XuLA2 board may be shared between SPI-Flash and the SPI
//		based SDCard.  An external arbiter will determine which of the
//		two gets to control the clock and mosi outputs given their
//		cs_n requests.  If control is not granted, i_bus_grant will
//		remain low as will the actual cs_n going out of the FPGA.
//
//
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
`timescale	1ns/1ps
`default_nettype	none
// }}}
module	llsdspi #(
		// {{{
		parameter	SPDBITS = 7,
				// Minimum startup clocks
				STARTUP_CLOCKS = 150,
				// System clocks to wait before the startup
				// clock sequence
				POWERUP_IDLE = 1000,
		//
		// This core was originally developed for a shared SPI bus
		// implementation on a XuLA2-LX25 board--one that shared flash
		// with the SD-card.  Few cards support such an implementation
		// any more, since per protocol CS high (inactive) and clock
		// and data pins toggling could well put the SD card into it's
		// SD mode instead of SPI mode.
		parameter [0:0]	OPT_SPI_ARBITRATION = 1'b0,
		//
		//
		localparam [0:0]	CSN_ON_STARTUP = 1'b1,
		//
		// The MOSI INACTIVE VALUE *MUST* be 1'b1 to be compliant
		localparam [0:0]	MOSI_INACTIVE_VALUE = 1'b1,
		//
		// Normally, an SPI transaction shuts the clock down when
		// finished.  If OPT_CONTINUOUS_CLOCK is set, the clock will be
		// continuous.  This also means that any driving program must
		// be ready when the SDSPI is idle.
		parameter [0:0]	OPT_CONTINUOUS_CLOCK = 1'b0
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Parameters/setup
		input	wire	[(SPDBITS-1):0]	i_speed,
		// The incoming interface
		input	wire		i_cs,
		input	wire		i_stb,
		input	wire	[7:0]	i_byte,
		// The actual SPI interface
		output	reg		o_cs_n, o_sclk, o_mosi,
		input	wire		i_miso,
		// The outgoing interface
		output	reg		o_stb,
		output	reg	[7:0]	o_byte,
		output	reg		o_idle,
		// And whether or not we actually own the interface (yet)
		input	wire		i_bus_grant
		// }}}
	);

	// Signal / localparam declarations
	// {{{
	localparam [3:0]	LLSDSPI_IDLE    = 4'h0,
				LLSDSPI_HOTIDLE	= 4'h1,
				LLSDSPI_WAIT	= 4'h2,
				LLSDSPI_START	= 4'h3,
				LLSDSPI_END	= 4'hb;
	//
	reg			r_z_counter;
	reg	[(SPDBITS-1):0]	r_clk_counter;
	reg			r_idle;
	reg		[3:0]	r_state;
	reg		[7:0]	r_byte, r_ireg;
	wire			byte_accepted;
	reg			restart_counter;

	wire			bus_grant, startup_hold, powerup_hold;
`ifdef	FORMAL
	reg	f_past_valid;
`endif
	// }}}

	assign	bus_grant = (OPT_SPI_ARBITRATION ? i_bus_grant : 1'b1);

	// Wait for power up
	// {{{
	generate if (POWERUP_IDLE > 0)
	begin : WAIT_FOR_POWERUP
		// {{{
		localparam	POWERUP_BITS = $clog2(POWERUP_IDLE);
		reg	[POWERUP_BITS-1:0]	powerup_counter;
		reg				r_powerup_hold;

		initial powerup_counter = POWERUP_IDLE[POWERUP_BITS-1:0];
		initial	r_powerup_hold = 1;
		always @(posedge i_clk)
		if (i_reset)
		begin
			powerup_counter <= POWERUP_IDLE;
			r_powerup_hold    <= 1;
		end else if (powerup_hold)
		begin
			if (|powerup_counter)
				powerup_counter <= powerup_counter - 1;
			r_powerup_hold <= (powerup_counter > 0);
		end

		assign	powerup_hold = r_powerup_hold;
`ifdef	FORMAL
		always @(*)
		if (!f_past_valid)
			assume(powerup_counter > 2);
		always @(*)
		if (powerup_counter > 0)
			assert(powerup_hold);
`endif
		// }}}
	end else begin : NO_POWERUP_HOLD

		assign	powerup_hold = 0;
	end endgenerate
	// }}}

	// Send a minimum number of start up clocks after power up
	// {{{
	generate if (STARTUP_CLOCKS > 0)
	begin : WAIT_FOR_STARTUP
		// {{{
		localparam	STARTUP_BITS = $clog2(STARTUP_CLOCKS);
		reg	[STARTUP_BITS-1:0]	startup_counter;
		reg				r_startup_hold;

		initial startup_counter = STARTUP_CLOCKS[STARTUP_BITS-1:0];
		initial	r_startup_hold = 1;
		always @(posedge i_clk)
		if (i_reset || powerup_hold)
		begin
			startup_counter <= STARTUP_CLOCKS;
			r_startup_hold    <= 1;
		end else if (startup_hold && r_z_counter && !o_sclk)
		begin
			if (|startup_counter)
				startup_counter <= startup_counter - 1;
			r_startup_hold <= (startup_counter > 0);
		end

		assign	startup_hold = r_startup_hold;
`ifdef	FORMAL
		always @(*)
		if (!f_past_valid)
			assume(startup_counter > 1);
		always @(*)
		if (startup_counter > 0)
			assert(startup_hold);
`endif
		// }}}
	end else begin : NO_STARTUP_HOLD

		assign	startup_hold = 0;

	end endgenerate
	// }}}

	assign	byte_accepted = (i_stb)&&(o_idle);

`ifdef	FORMAL
	// {{{
	always @(*)
	if (powerup_hold)
		assert(startup_hold);
	// }}}
`endif
	////////////////////////////////////////////////////////////////////////
	//
	// Clock divider and speed control
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	initial	r_clk_counter = 0;
	initial	r_z_counter = 1'b1;

	// restart_counter
	// {{{
	always @(*)
	if (OPT_CONTINUOUS_CLOCK || powerup_hold)
		restart_counter = !powerup_hold;
	else begin
		restart_counter = 1'b0;

		if (startup_hold || !i_cs)
			restart_counter = 1'b1;
		else if (!OPT_SPI_ARBITRATION && byte_accepted)
			restart_counter = 1'b1;
		else if (OPT_SPI_ARBITRATION && r_state == LLSDSPI_IDLE)
			restart_counter = 1'b0;
		else if (OPT_SPI_ARBITRATION && r_state == LLSDSPI_WAIT
				&& !bus_grant)
			restart_counter = 1'b0;
		else if (OPT_SPI_ARBITRATION && byte_accepted)
			restart_counter = 1'b1;
		else
			restart_counter = !r_idle;
	end
	// }}}

	// r_clk_counter, r_z_counter
	// {{{
	always @(posedge i_clk)
	begin
		if (!r_z_counter)
		begin
			r_clk_counter <= (r_clk_counter - 1);
			r_z_counter <= (r_clk_counter == 1);
		end else if (restart_counter)
		begin
			r_clk_counter <= i_speed;
			r_z_counter <= (i_speed == 0);
		end
	end
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Control o_stb, o_cs_n, and o_mosi
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// o_cs_n, r_state
	// {{{
	initial	o_cs_n = CSN_ON_STARTUP;
	initial	r_state = LLSDSPI_IDLE;
	always @(posedge i_clk)
	if (i_reset || (!CSN_ON_STARTUP && startup_hold))
	begin
		o_cs_n <= CSN_ON_STARTUP;
		r_state <= LLSDSPI_IDLE;
	end else if (r_z_counter)
	begin
		if (!i_cs)
		begin
			// No request for action.  If anything, a request
			// to close up/seal up the bus for the next transaction
			// Expect to lose arbitration here.
			r_state <= LLSDSPI_IDLE;
			o_cs_n <= 1'b1;
		end else if (r_state == LLSDSPI_IDLE)
		begin
			if (byte_accepted)
			begin
				o_cs_n <= 1'b0;
				if (OPT_SPI_ARBITRATION)
					// Wait for arbitration
					r_state <= LLSDSPI_WAIT;
				else
					r_state <= LLSDSPI_START + (OPT_CONTINUOUS_CLOCK ? 1:0);
			end
		end else if (r_state == LLSDSPI_WAIT)
		begin
			if (bus_grant)
				r_state <= LLSDSPI_START;
		end else if (byte_accepted)
			r_state <= LLSDSPI_START+1;
		else if (o_sclk && r_state >= LLSDSPI_START)
		begin
			r_state <= r_state + 1;
			if (r_state >= LLSDSPI_END)
				r_state <= LLSDSPI_HOTIDLE;
		end

		if (startup_hold)
			o_cs_n <= 1;
	end
	// }}}

	// r_ireg
	// {{{
	always @(posedge i_clk)
	if (r_z_counter && !o_sclk)
		r_ireg <= { r_ireg[6:0], i_miso };
	// }}}

	// o_byte
	// {{{
	always @(posedge i_clk)
	if (r_z_counter && o_sclk && r_state == LLSDSPI_END)
		o_byte <= r_ireg;
	// }}}

	// r_idle
	// {{{
	initial	r_idle  = 0;
	always @(posedge i_clk)
	if (startup_hold || i_reset)
		r_idle <= 0;
	else if (r_z_counter)
	begin
		if (byte_accepted)
			r_idle <= 1'b0;
		else if ((r_state == LLSDSPI_END)
				||(r_state == LLSDSPI_HOTIDLE))
			r_idle <= 1'b1;
		else if (r_state == LLSDSPI_IDLE)
			r_idle <= 1'b1;
		else
			r_idle <= 1'b0;
	end
	// }}}

	// o_sclk
	// {{{
	initial	o_sclk = 1;
	always @(posedge i_clk)
	if (i_reset)
		o_sclk <= 1;
	else if (r_z_counter)
	begin
		if (OPT_CONTINUOUS_CLOCK)
			o_sclk <= !o_sclk;
		else if (restart_counter
			&& (startup_hold || (i_cs && !o_cs_n) || !o_sclk))
			o_sclk <= (r_state == LLSDSPI_WAIT) || !o_sclk;
	end
	// }}}

	// r_byte, o_mosi
	// {{{
	initial	r_byte = -1;
	initial	o_mosi = MOSI_INACTIVE_VALUE;
	always @(posedge i_clk)
	if (i_reset)
	begin
		r_byte <= {(8){MOSI_INACTIVE_VALUE}};
		o_mosi <= MOSI_INACTIVE_VALUE;
	end else if (r_z_counter)
	begin
		if (byte_accepted)
		begin
			o_mosi <= MOSI_INACTIVE_VALUE;
			if (o_cs_n && !OPT_CONTINUOUS_CLOCK)
				r_byte <= i_byte[7:0];
			else begin
				r_byte <= { i_byte[6:0], MOSI_INACTIVE_VALUE };
				o_mosi <= i_byte[7];
			end
		end else if (o_sclk && (!OPT_SPI_ARBITRATION
				|| (bus_grant && r_state != LLSDSPI_WAIT)))
		begin
			r_byte <= { r_byte[6:0], MOSI_INACTIVE_VALUE };
			if (r_state >= LLSDSPI_START && r_state < LLSDSPI_END)
				o_mosi <= r_byte[7];
			else if (!i_cs)
				o_mosi <= MOSI_INACTIVE_VALUE;
		end
	end
	// }}}

	// o_stb
	// {{{
	initial	o_stb  = 1'b0;
	always @(posedge i_clk)
	if (i_reset || startup_hold || !i_cs || !r_z_counter || !o_sclk)
		o_stb <= 1'b0;
	else
		o_stb <= (r_state >= LLSDSPI_END);
	// }}}

	// o_idle
	// {{{
	always @(*)
	begin
		if (OPT_CONTINUOUS_CLOCK)
			o_idle = (r_idle)&&(r_z_counter)&&(o_sclk);
		else
			o_idle = (r_idle)&&(r_z_counter);
	end
	// }}}
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`ifdef	LLSDSPI
`define	ASSUME	assume
`else
`define	ASSUME	assert
`endif

	reg [7:0]	fv_byte;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	////////////////////////////////////////////////////////////////////////
	//
	// Clock division logic (checks)
	////////////////////////////////////////////////////////////////////////
	//
	//
	(* anyconst *) reg constant_speed;

	always @(posedge i_clk)
	if (constant_speed)
	begin
		assume($stable(i_speed));
		assert(r_clk_counter <= i_speed);
	end

	always @(posedge i_clk)
	if (f_past_valid && $past(r_clk_counter != 0))
		assert(r_clk_counter == $past(r_clk_counter - 1));
	else if (f_past_valid)
		assert((r_clk_counter == 0)||(r_clk_counter == $past(i_speed)));

	//
	// And assumptions
	//
	always @(posedge i_clk)
	if (f_past_valid && (i_cs || !r_z_counter))
		assume($stable(i_speed));

	////////////////////////////////////////////////////////////////////////
	//
	// Interface assumptions
	//
	always @(*)
	if (i_stb)
		`ASSUME(i_cs);

	always @(posedge i_clk)
	if (f_past_valid && $past(i_stb && !o_idle))
	begin
		if ($past(i_reset || !i_cs))
			`ASSUME(!i_stb);
		// else
		//	`ASSUME(i_stb);
		if ($past(f_past_valid)&&(!$past(o_idle,2)))
			`ASSUME($stable(i_byte));
	end else if (f_past_valid && $past(i_cs && !o_idle && !i_reset))
		`ASSUME(i_cs);
		// assume(i_cs);

	always @(posedge i_clk)
	if (!f_past_valid)
		assert(o_cs_n == CSN_ON_STARTUP);
	else if ($past(bus_grant && !o_cs_n))
		`ASSUME(bus_grant);
	else if ($past(!bus_grant && o_cs_n))
		`ASSUME(!bus_grant);

	always @(*)
	if (r_state == LLSDSPI_IDLE)
		assert(!CSN_ON_STARTUP || o_cs_n);
	else
		assert(!o_cs_n);


	always @(posedge i_clk)
	if (f_past_valid && CSN_ON_STARTUP && $past(r_state == LLSDSPI_IDLE)
		&& $past(!i_reset)
		&& r_state == LLSDSPI_IDLE)
		assert(o_cs_n);

	always @(*)
	if (r_state == LLSDSPI_WAIT)
		assert(!o_cs_n && o_sclk);
	else
		assert(o_cs_n || bus_grant);

	always @(posedge i_clk)
	if (f_past_valid && $past(!i_reset && i_cs && !o_cs_n && !bus_grant))
		assert(!o_cs_n && o_sclk);

	always @(posedge i_clk)
	if ($past(!o_cs_n && bus_grant) && (!o_cs_n && bus_grant))
	begin
		if (!$fell(o_sclk))
			assume($stable(i_miso));

		cover($changed(i_miso));
	end

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		`ASSUME(!i_stb);
		`ASSUME(!i_cs);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Reset checks
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(startup_hold);
		assert(o_cs_n == CSN_ON_STARTUP);
		assert(o_sclk);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Send byte sequences: from nothing, and from the last byte
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	[17:0]	f_start_seq;
	reg	[16:0]	f_next_seq;

	always @(*)
		assert(r_z_counter == (r_clk_counter == 0));

	always @(posedge i_clk)
	if (byte_accepted)
		fv_byte <= i_byte;

	always @(posedge i_clk)
		assert(r_state <= LLSDSPI_END);

	initial	f_start_seq = 0;
	always @(posedge i_clk)
	if (i_reset || !i_cs)
		f_start_seq <= 0;
	else if (f_start_seq == 0)
	begin
		if (OPT_SPI_ARBITRATION)
		begin
			if (r_state == LLSDSPI_IDLE && byte_accepted)
			begin
			//	if (!o_cs_n && bus_grant)
			//		f_start_seq <= (f_next_seq == 0);
			//	//r_start_seq <= (!o_cs_n && bus_grant) ? 1:0;
			end else if (r_state == LLSDSPI_WAIT && r_z_counter)
				f_start_seq <= (bus_grant ? 1:0);
		end else if (byte_accepted)
		begin
			if (OPT_CONTINUOUS_CLOCK)
				f_start_seq <= (f_next_seq == 0) ? 2:0;
			else
				f_start_seq <= (f_next_seq == 0);
		end
	end else if (r_z_counter)
	begin
		f_start_seq <= f_start_seq << 1;
		if (byte_accepted)
		begin
			f_start_seq[17] <= 0;
			f_start_seq[16] <= 0;
		end else if (f_start_seq[17])
			f_start_seq[17] <= 1;
	end

	always @(*)
	if (!OPT_SPI_ARBITRATION)
		assert(r_state != LLSDSPI_WAIT);

	always @(*)
	if (r_state == LLSDSPI_WAIT)
	begin
		// assert(o_mosi == fv_byte[7]);
		assert(r_byte == fv_byte[7:0]);
		assert(o_sclk);
		assert(!o_stb);
		assert(f_start_seq == 0);
		assert(f_next_seq == 0);
		assert(!r_idle);
		assert(r_z_counter);
	end

	always @(*)
	if (|(f_start_seq & {(8){2'b01}}))
		assert(o_sclk);
	else if (|(f_start_seq & {(8){2'b10}}))
		assert(!o_sclk);

	generate if (!OPT_CONTINUOUS_CLOCK)
	begin
		always @(*)
		if (f_start_seq[17:0] == 18'h001)
			cover(r_state == LLSDSPI_START);
	end endgenerate

	always @(*)
	case(f_start_seq[17:0])
	18'h001: begin
		assert(o_sclk);
		assert(r_byte == fv_byte[7:0] );
		assert(r_state == LLSDSPI_START);
		// assert(o_mosi == fv_byte[7]);
		assert(!OPT_CONTINUOUS_CLOCK);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h002: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[6:0], {(1){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h004: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[6:0], {(1){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h008: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[5:0], {(2){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h010: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[5:0], {(2){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h020: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[4:0], {(3){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h040: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[4:0], {(3){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h080: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[3:0], {(4){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h100: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[3:0], {(4){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h00200: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[2:0], {(5){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h00400: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[2:0], {(5){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h00800: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[1:0], {(6){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h01000: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[1:0], {(6){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h02000: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[0], {(7){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h04000: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[0], {(7){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h08000: begin
		assert(!o_sclk);
		assert(r_byte == { {(8){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_END);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h10000: begin
		assert(o_sclk);
		assert(r_byte == { {(8){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_END);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(r_idle);
		end
	18'h20000: begin
		assert(o_sclk);
		assert(r_state == LLSDSPI_HOTIDLE);
		assert(r_idle);
		end
	default: assert(f_start_seq == 0);
	endcase

	always @(*)
	if (|f_start_seq[16:1])
	begin
		assert(!o_cs_n);
		assert(bus_grant);
	end

	initial	f_next_seq = 0;
	always @(posedge i_clk)
	if (i_reset || !i_cs)
		f_next_seq <= 0;
	else if ((|f_start_seq[17:16]) && byte_accepted)
		f_next_seq <= 1;
	else if ((|f_next_seq[16:15]) && byte_accepted)
		f_next_seq <= 1;
	else if (r_z_counter)
	begin
		f_next_seq <= f_next_seq << 1;
		// if (i_stb)
		//	f_next_seq[16] <= 0;
		// else
		if (f_next_seq[16])
			f_next_seq[16] <= 1;
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset) && !$fell(o_sclk) && !o_cs_n)
		assert($stable(o_mosi));

	always @(*)
	if (|(f_next_seq & {(8){2'b10}}))
		assert(o_sclk);
	else if (|(f_next_seq & {(8){2'b01}}))
		assert(!o_sclk);

	always @(*)
	case(f_next_seq[16:0])
	17'h001: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[6:0], {(1){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h002: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[6:0], {(1){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h004: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[5:0], {(2){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h008: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[5:0], {(2){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h010: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[4:0], {(3){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h020: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[4:0], {(3){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h040: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[3:0], {(4){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h080: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[3:0], {(4){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h100: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[2:0], {(5){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h200: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[2:0], {(5){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h400: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[1:0], {(6){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h0800: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[1:0], {(6){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h1000: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[0], {(7){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h2000: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[0], {(7){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h4000: begin
		assert(!o_sclk);
		assert(r_byte == { {(8){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_END);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h8000: begin
		assert(o_sclk);
		assert(r_byte == { {(8){MOSI_INACTIVE_VALUE}} });
		assert(r_state == LLSDSPI_END);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(r_idle);
		end
	17'h10000: begin
		assert(o_sclk);
		assert(!o_cs_n);
		assert(r_state == LLSDSPI_HOTIDLE);
		assert(r_idle);
		end
	default: assert(f_next_seq == 0);
	endcase

	always @(*)
	if (f_next_seq != 0)
		assert(f_start_seq == 0);

	always @(*)
	if (r_state != LLSDSPI_IDLE)
		assert(({ 1'b0, f_next_seq } | f_start_seq)
			||(OPT_SPI_ARBITRATION && r_state == LLSDSPI_WAIT));

	always @(*)
	if ((f_start_seq==0) && (f_next_seq == 0))
		assert((r_state == LLSDSPI_IDLE)
			||(OPT_SPI_ARBITRATION && r_state == LLSDSPI_WAIT));

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset) && $changed({ o_sclk, o_mosi }))
	begin
		if ($past(i_cs))
		begin
			assert(r_z_counter == $past(i_speed == 0));
			assert(r_clk_counter == $past(i_speed));
		end
	end

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset) && !$past(r_z_counter))
	begin
		assert($stable(o_cs_n));
		assert($stable(o_sclk));
		assert($stable(o_mosi));
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Verify the receiver
	//
	(* anyseq *) reg [7:0] f_rxdata;

	always @(posedge i_clk)
	if (!$past(byte_accepted))
		assume($stable(f_rxdata));

	always @(*)
	case(f_start_seq[17:0] | { f_next_seq, 1'b0 })
	18'h001: begin
		end
	18'h002: begin
		assume(i_miso == f_rxdata[7]);
		end
	18'h004: begin
		// assume(i_miso == f_rxdata[7]);
		assert(r_ireg[0] == f_rxdata[7]);
		end
	18'h008: begin
		// assume(i_miso == f_rxdata[6]);
		assert(r_ireg[0] == f_rxdata[7]);
		end
	18'h010: begin
		assume(i_miso == f_rxdata[6]);
		assert(r_ireg[1:0] == f_rxdata[7:6]);
		end
	18'h020: begin
		assume(i_miso == f_rxdata[5]);
		assert(r_ireg[1:0] == f_rxdata[7:6]);
		end
	18'h040: begin
		assume(i_miso == f_rxdata[5]);
		assert(r_ireg[2:0] == f_rxdata[7:5]);
		end
	18'h080: begin
		assume(i_miso == f_rxdata[4]);
		assert(r_ireg[2:0] == f_rxdata[7:5]);
		end
	18'h100: begin
		assume(i_miso == f_rxdata[4]);
		assert(r_ireg[3:0] == f_rxdata[7:4]);
		end
	18'h200: begin
		assume(i_miso == f_rxdata[3]);
		assert(r_ireg[3:0] == f_rxdata[7:4]);
		end
	18'h400: begin
		assume(i_miso == f_rxdata[3]);
		assert(r_ireg[4:0] == f_rxdata[7:3]);
		end
	18'h800: begin
		assume(i_miso == f_rxdata[2]);
		assert(r_ireg[4:0] == f_rxdata[7:3]);
		end
	18'h1000: begin
		assume(i_miso == f_rxdata[2]);
		assert(r_ireg[5:0] == f_rxdata[7:2]);
		end
	18'h2000: begin
		assume(i_miso == f_rxdata[1]);
		assert(r_ireg[5:0] == f_rxdata[7:2]);
		end
	18'h4000: begin
		assume(i_miso == f_rxdata[1]);
		assert(r_ireg[6:0] == f_rxdata[7:1]);
		end
	18'h8000: begin
		assume(i_miso == f_rxdata[0]);
		assert(r_ireg[6:0] == f_rxdata[7:1]);
		end
	18'h10000: begin
		assume(i_miso == f_rxdata[0]);
		assert(r_ireg == f_rxdata);
		end
	18'h20000: begin
		assume(i_miso == f_rxdata[0]);
		assert(r_ireg == f_rxdata);
		assert(o_byte == f_rxdata);
		end
	endcase

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
		assert(!o_stb);
	else if ($past(i_cs))
	begin
		if ($fell(f_start_seq[16]))
			assert(o_stb);
		else if ($fell(f_next_seq[15]))
			assert(o_stb);
		else
			assert(!o_stb);
	end else
		assert(!o_stb);

	always @(posedge i_clk)
	if (o_cs_n || ((f_start_seq == 0) &&(f_next_seq == 0)))
		assert(o_mosi == MOSI_INACTIVE_VALUE);

	always @(*)
	if (!f_past_valid)
	begin
		if (STARTUP_CLOCKS > 0)
			`ASSUME(startup_hold);
		assert(r_z_counter);
		assert(!r_idle);
		assert(o_cs_n == CSN_ON_STARTUP);
		assert(o_sclk);
	end

	always @(*)
	if (r_state == LLSDSPI_WAIT)
		assert(o_sclk && !o_cs_n);

	always @(*)
	if (!OPT_CONTINUOUS_CLOCK && o_cs_n && r_idle)
		assert(o_sclk);

	always @(*)
	if (startup_hold)
		assert(o_cs_n == CSN_ON_STARTUP);
	else if (!OPT_CONTINUOUS_CLOCK && o_cs_n)
		assert(o_sclk);


`ifdef	VERIFIC
	always @(*)
		assert($onehot0(f_next_seq));
	always @(*)
		assert($onehot0(f_start_seq));
`endif
	////////////////////////////////////////////////////////////////////////
	//
	// Cover checks
	//
`ifdef	LLSDSPI
	(* anyconst *) reg nonzero_speed;

	reg	[2:0]	byte_count;

	always @(posedge i_clk)
	if (i_reset || !i_cs)
		byte_count <= 0;
	else if (i_stb && !o_idle && (!(&byte_count)))
		byte_count <= byte_count + 1;

	always @(posedge i_clk)
	if (f_past_valid)
	begin
		cover(byte_count == 2 && !o_cs_n &&  nonzero_speed);
		cover(byte_count == 2 && !o_cs_n && !nonzero_speed);

		/*
		if (nonzero_speed)
		begin
			assume(i_speed > 0);
			cover($rose(r_state == LLSDSPI_IDLE)&&($past(i_cs,2)));
			cover($past(f_next_seq[16])
					&& r_state == LLSDSPI_IDLE && (!i_cs));
			cover($past(f_next_seq[16]) && f_next_seq[0]);
		end else begin
			cover($rose(r_state == LLSDSPI_IDLE)&&($past(i_cs,2)));
			cover($past(f_next_seq[16])
					&& r_state == LLSDSPI_IDLE && (!i_cs));
			cover($past(f_next_seq[16]) && f_next_seq[0]);
		end
		*/

		cover(o_stb && o_byte == 8'haa && $changed(o_byte));
		cover(o_stb && o_byte == 8'h55 && $changed(o_byte));
	end

	generate if (OPT_CONTINUOUS_CLOCK)
	begin
		always @(*)
		begin
			cover(f_next_seq == 2);
			cover(f_start_seq[3]);
			cover(f_next_seq[15]);
			// cover(f_next_seq[16]); // Won't happen anymore
		end
	end else begin
		always @(*)
			cover(f_next_seq == 1);
	end endgenerate
`endif
	////////////////////////////////////////////////////////////////////////
	//
	// "Careless" constraining assumptions
	//
	always @(posedge i_clk)
	if (f_past_valid)
	begin
		if ($past(i_stb && !i_reset))
			`ASSUME(i_cs);
	end

	always @(*)
	if (!CSN_ON_STARTUP && r_state == 0 && !o_cs_n)
		assume(!i_cs);

	always @(posedge i_clk)
	if (OPT_CONTINUOUS_CLOCK)
	begin
		// if ($past(i_cs && !i_stb))
		//	assume(!i_stb);
		assume(i_stb == i_cs);

		if (!o_sclk && r_state == LLSDSPI_IDLE)
			assume(!i_stb);
	end

`endif // FORMAL
// }}}
endmodule
