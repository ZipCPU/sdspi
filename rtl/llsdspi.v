////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	llsdspi.v
//
// Project:	SD-Card controller, using a shared SPI interface
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
//
// Copyright (C) 2016-2019, Gisselquist Technology, LLC
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
`default_nettype	none
//
module	llsdspi(i_clk, i_speed, i_cs, i_stb, i_byte,
		o_cs_n, o_sclk, o_mosi, i_miso,
		o_stb, o_byte, o_idle, i_bus_grant);
	parameter	SPDBITS = 7,
		STARTUP_CLOCKS = 75;
	parameter [0:0]	OPT_SPI_ARBITRATION = 1'b0;
	//
	input	wire		i_clk;
	// Parameters/setup
	input	wire	[(SPDBITS-1):0]	i_speed;
	// The incoming interface
	input	wire		i_cs;
	input	wire		i_stb;
	input	wire	[7:0]	i_byte;
	// The actual SPI interface
	output	reg		o_cs_n, o_sclk, o_mosi;
	input	wire		i_miso;
	// The outgoing interface
	output	reg		o_stb;
	output	reg	[7:0]	o_byte;
	output	wire		o_idle;
	// And whether or not we actually own the interface (yet)
	input	wire		i_bus_grant;

	localparam [3:0]	LLSDSPI_IDLE    = 4'h0,
				LLSDSPI_HOTIDLE	= 4'h1,
				LLSDSPI_WAIT	= 4'h2,
				LLSDSPI_START	= 4'h3;
//
	reg			r_z_counter;
	reg	[(SPDBITS-1):0]	r_clk_counter;
	reg			r_idle;
	reg		[3:0]	r_state;
	reg		[7:0]	r_byte, r_ireg;
	wire			byte_accepted;

`ifdef	FORMAL
	reg	f_past_valid;
`endif

	reg	startup_hold;
	generate if (STARTUP_CLOCKS > 0)
	begin : WAIT_FOR_STARTUP
		localparam	STARTUP_BITS = $clog2(STARTUP_CLOCKS);
		reg	[STARTUP_BITS-1:0]	startup_counter;
		reg				past_sclk;

		initial	past_sclk = 1;
		always @(posedge i_clk)
			past_sclk <= o_sclk;

`ifndef	FORMAL
		initial startup_counter = STARTUP_CLOCKS[STARTUP_BITS-1:0];
`endif
		initial	startup_hold = 1;
		always @(posedge i_clk)
		if (startup_hold && !past_sclk && o_sclk)
		begin
			if (|startup_counter)
				startup_counter <= startup_counter - 1;
			startup_hold <= (startup_counter > 0);
		end

`ifdef	F_PAST_VALID
		always @(*)
		if (!f_past_valid)
			assume(startup_counter > 1);
`endif
	end else begin

		always @(*)
			startup_hold = 0;

	end endgenerate

	assign	byte_accepted = (i_stb)&&(o_idle);

	////////////////////////////////////////////////////////////////////////
	//
	// Clock divider and speed control
	//
	initial	r_clk_counter = 7'h0;
	initial	r_z_counter = 1'b1;
	always @(posedge i_clk)
	begin
		if (!startup_hold && (!i_cs || (OPT_SPI_ARBITRATION && !i_bus_grant)))
		begin
			// Hold, waiting for some action
			r_clk_counter <= 0;
			r_z_counter <= 1'b1;
		end else if (startup_hold || byte_accepted)
		begin
			r_clk_counter <= i_speed;
			r_z_counter <= (i_speed == 0);
		end else if (!r_z_counter)
		begin
			r_clk_counter <= (r_clk_counter - 1);
			r_z_counter <= (r_clk_counter == 1);
		end else if ((r_state > LLSDSPI_WAIT)&&(!r_idle))
		begin
			if (r_state >= LLSDSPI_START+8)
			begin
				r_clk_counter <= 0;
				r_z_counter <= 1;
			end else begin
				r_clk_counter <= i_speed;
				r_z_counter <= (i_speed == 0);
			end
		end
	end


	////////////////////////////////////////////////////////////////////////
	//
	// Control o_stb, o_cs_n, and o_mosi
	//
	initial	o_stb  = 1'b0;
	initial	o_cs_n = 1'b1;
	initial	o_sclk = 1'b1;
	initial	r_state = LLSDSPI_IDLE;
	initial	r_idle  = 0;
	always @(posedge i_clk)
	begin
		o_stb <= 1'b0;
		o_cs_n <= (startup_hold || !i_cs);
		if (!i_cs)
		begin
			// No request for action.  If anything, a request
			// to close up/seal up the bus for the next transaction
			// Expect to lose arbitration here.
			r_state <= LLSDSPI_IDLE;
			r_idle <= (r_z_counter);
			o_sclk <= 1'b1;
		end else if (!r_z_counter)
			r_idle <= 1'b0;
		else if (r_state == LLSDSPI_IDLE)
		begin
			o_sclk <= 1'b1;
			r_idle <= (!startup_hold);
			if (byte_accepted)
			begin
				r_byte <= i_byte[7:0];
				if (OPT_SPI_ARBITRATION)
					r_state <= (!o_cs_n && i_bus_grant)
						? LLSDSPI_START:LLSDSPI_WAIT;
				else
					r_state <= LLSDSPI_START;
				r_idle <= 1'b0;
				o_mosi <= i_byte[7];
			end
		end else if (r_state == LLSDSPI_WAIT)
		begin
			r_idle <= 1'b0;
			o_sclk <= 1'b1;
			if (!OPT_SPI_ARBITRATION || i_bus_grant)
				r_state <= LLSDSPI_START;
		end else if (r_state == LLSDSPI_HOTIDLE)
		begin
			// The clock is low, the bus is granted, we're just
			// waiting for the next byte to transmit
			o_sclk <= 1'b1;
			if (byte_accepted)
			begin
				r_byte <= { i_byte[6:0], 1'b1 };
				r_state <= LLSDSPI_START+1;
				r_idle <= 1'b0;
				o_mosi <= i_byte[7];
				o_sclk <= 1'b0;
			end else
				r_idle <= 1'b1;
		end else if (o_sclk)
		begin
			o_mosi <= r_byte[7];
			r_byte <= { r_byte[6:0], 1'b1 };
			r_state <= r_state + 1;
			o_sclk <= 1'b0;
			if (r_state >= LLSDSPI_START+8)
			begin
				r_state <= LLSDSPI_HOTIDLE;
				r_idle <= 1'b1;
				o_stb <= 1'b1;
				o_byte <= r_ireg;
				o_sclk <= 1'b1;
			end else
				r_state <= r_state + 1;
		end else begin
			r_ireg <= { r_ireg[6:0], i_miso };
			o_sclk <= 1'b1;
		end

		if (startup_hold)
		begin
			r_idle <= 0;
			o_cs_n <= 1;
			if (r_z_counter)
				o_sclk <= !o_sclk;
		end
	end

	assign o_idle = (r_idle)&&( (i_cs)&&(!OPT_SPI_ARBITRATION || i_bus_grant) );

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
	// Interface assumptions
	//
	always @(*)
	if (!OPT_SPI_ARBITRATION)
		assume(i_bus_grant);

	always @(*)
	if (i_stb)
		`ASSUME(i_cs);

	always @(posedge i_clk)
	if (f_past_valid && $past(i_stb && !o_idle))
	begin
		`ASSUME(i_stb);
		`ASSUME($stable(i_byte));
	end else if ($past(i_cs && !o_idle))
		`ASSUME(i_cs);

	always @(posedge i_clk)
	if (f_past_valid && i_cs)
		`ASSUME($stable(i_speed));

	always @(posedge i_clk)
	if (!r_z_counter)
		`ASSUME($stable(i_speed));

	always @(posedge i_clk)
	if ($past(i_bus_grant && !o_cs_n))
		assume(i_bus_grant);
	else if ($past(!i_bus_grant && o_cs_n))
		assume(!i_bus_grant);

//	always @(posedge i_clk)
//	if ($past(!o_cs_n && i_bus_grant) && (!o_cs_n && i_bus_grant))
//	begin
//		if (!$rose(o_sclk))
//			assume($stable(i_miso));
//	end

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
		assert(r_state <= LLSDSPI_START+8);

	initial	f_start_seq = 0;
	always @(posedge i_clk)
	if (!i_cs)
		f_start_seq <= 0;
	else if (f_start_seq == 0)
	begin
		if (!OPT_SPI_ARBITRATION && byte_accepted)
			f_start_seq <= (f_next_seq == 0);
		if (OPT_SPI_ARBITRATION)
		begin
			if (r_state == LLSDSPI_IDLE && byte_accepted)
			begin
				if (!o_cs_n && i_bus_grant)
					f_start_seq <= (f_next_seq == 0);
				//r_start_seq <= (!o_cs_n && i_bus_grant) ? 1:0;
			end else if (r_state == LLSDSPI_WAIT && r_z_counter)
				f_start_seq <= (i_bus_grant ? 1:0);
		end
	end else if (r_z_counter)
	begin
		f_start_seq <= f_start_seq << 1;
		if (byte_accepted)
			f_start_seq[17] <= 0;
		else if (f_start_seq[17])
			f_start_seq[17] <= 1;
	end

	always @(*)
	if (r_state == LLSDSPI_WAIT)
	begin
		assert(o_mosi == fv_byte[7]);
		assert(r_byte == fv_byte[7:0]);
		assert(o_sclk);
	end

	always @(*)
	if (|(f_start_seq & {(8){2'b01}}))
		assert(o_sclk);
	else if (|(f_start_seq & {(8){2'b10}}))
		assert(!o_sclk);

	always @(*)
	case(f_start_seq[17:0])
	18'h001: begin
		assert(o_sclk);
		assert(r_byte == fv_byte[7:0] );
		assert(r_state == LLSDSPI_START);
		assert(o_mosi == fv_byte[7]);
		cover(r_state == LLSDSPI_START);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h002: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[6:0], 1'b1 });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h004: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[6:0], 1'b1 });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h008: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[5:0], 2'b11 });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h010: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[5:0], 2'b11 });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h020: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[4:0], 3'b111 });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h040: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[4:0], 3'b111 });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h080: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[3:0], 4'hf });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h100: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[3:0], 4'hf });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h200: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[2:0], 5'h1f });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h400: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[2:0], 5'h1f });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h800: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[1:0], 6'h3f });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h1000: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[1:0], 6'h3f });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h2000: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[0], 7'h7f });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h4000: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[0], 7'h7f });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h8000: begin
		assert(!o_sclk);
		assert(r_byte == { 8'hff });
		assert(r_state == LLSDSPI_START+8);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h10000: begin
		assert(o_sclk);
		assert(r_byte == { 8'hff });
		assert(r_state == LLSDSPI_START+8);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(!r_idle);
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
		assert(i_bus_grant);
	end

	initial	f_next_seq = 0;
	always @(posedge i_clk)
	if (!i_cs)
		f_next_seq <= 0;
	else if (f_start_seq[17] && byte_accepted)
		f_next_seq <= 1;
	else if (f_next_seq[16] && byte_accepted)
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

	always @(*)
	if (|(f_next_seq & {(8){2'b10}}))
		assert(o_sclk);
	else if (|(f_next_seq & {(8){2'b01}}))
		assert(!o_sclk);

	always @(*)
	case(f_next_seq[16:0])
	17'h001: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[6:0], 1'b1 });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h002: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[6:0], 1'b1 });
		assert(r_state == LLSDSPI_START+1);
		assert(o_mosi == fv_byte[7]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h004: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[5:0], 2'b11 });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h008: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[5:0], 2'b11 });
		assert(r_state == LLSDSPI_START+2);
		assert(o_mosi == fv_byte[6]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h010: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[4:0], 3'b111 });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h020: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[4:0], 3'b111 });
		assert(r_state == LLSDSPI_START+3);
		assert(o_mosi == fv_byte[5]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h040: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[3:0], 4'hf });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	18'h080: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[3:0], 4'hf });
		assert(r_state == LLSDSPI_START+4);
		assert(o_mosi == fv_byte[4]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h100: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[2:0], 5'h1f });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h200: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[2:0], 5'h1f });
		assert(r_state == LLSDSPI_START+5);
		assert(o_mosi == fv_byte[3]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h400: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[1:0], 6'h3f });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h0800: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[1:0], 6'h3f });
		assert(r_state == LLSDSPI_START+6);
		assert(o_mosi == fv_byte[2]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h1000: begin
		assert(!o_sclk);
		assert(r_byte == { fv_byte[0], 7'h7f });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h2000: begin
		assert(o_sclk);
		assert(r_byte == { fv_byte[0], 7'h7f });
		assert(r_state == LLSDSPI_START+7);
		assert(o_mosi == fv_byte[1]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h4000: begin
		assert(!o_sclk);
		assert(r_byte == { 8'hff });
		assert(r_state == LLSDSPI_START+8);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(!r_idle);
		end
	17'h8000: begin
		assert(o_sclk);
		assert(r_byte == { 8'hff });
		assert(r_state == LLSDSPI_START+8);
		assert(o_mosi == fv_byte[0]);
		assert(!o_cs_n);
		assert(!r_idle);
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
		assume(i_miso == f_rxdata[7]);
		assert(r_ireg[0] == f_rxdata[7]);
		end
	18'h008: begin
		assume(i_miso == f_rxdata[6]);
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
	if (f_past_valid && $rose(f_start_seq[17]))
		assert(o_stb);
	else if (f_past_valid && $rose(f_next_seq[16]))
		assert(o_stb);
	else
		assert(!o_stb);

`ifndef	VERIFIC
`else
	always @(*)
		assert($onehot0(f_next_seq));
	always @(*)
		assert($onehot0(f_start_seq));
`endif
	////////////////////////////////////////////////////////////////////////
	//
	// Cover checks
	//
	(* anyconst *) reg nonzero_speed;
	always @(posedge i_clk)
	if (f_past_valid)
	begin
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

		cover(o_stb && o_byte == 8'haa);
		cover(o_stb && o_byte == 8'h55);
	end

	always @(*)
	if (!f_past_valid)
	begin
		if (STARTUP_CLOCKS > 0)
			assume(startup_hold);
		assert(r_z_counter);
		assert(!r_idle);
		assert(o_cs_n);
		assert(o_sclk);
	end

	always @(*)
	if (r_state == LLSDSPI_WAIT)
		assert(o_sclk && !o_cs_n);

	always @(*)
	if (o_cs_n && r_idle)
		assert(o_sclk);

`endif // FORMAL
endmodule


