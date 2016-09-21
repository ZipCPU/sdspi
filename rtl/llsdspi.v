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
`define	LLSDSPI_IDLE	4'h0
`define	LLSDSPI_HOTIDLE	4'h1
`define	LLSDSPI_WAIT	4'h2
`define	LLSDSPI_START	4'h3
//
module	llsdspi(i_clk, i_speed, i_cs, i_stb, i_byte, 
		o_cs_n, o_sclk, o_mosi, i_miso,
		o_stb, o_byte, o_idle, i_bus_grant);
	parameter	SPDBITS = 7;
	//
	input			i_clk;
	// Parameters/setup
	input		[(SPDBITS-1):0]	i_speed;
	// The incoming interface
	input			i_cs;
	input			i_stb;
	input		[7:0]	i_byte;
	// The actual SPI interface
	output	reg		o_cs_n, o_sclk, o_mosi;
	input			i_miso;
	// The outgoing interface
	output	reg		o_stb;
	output	reg	[7:0]	o_byte;
	output	wire		o_idle;
	// And whether or not we actually own the interface (yet)
	input			i_bus_grant;

	reg			r_z_counter;
	reg	[(SPDBITS-1):0]	r_clk_counter;
	reg			r_idle;
	reg		[3:0]	r_state;
	reg		[7:0]	r_byte, r_ireg;

	wire	byte_accepted;
	assign	byte_accepted = (i_stb)&&(o_idle);

	initial	r_clk_counter = 7'h0;
	always @(posedge i_clk)
	begin
		if ((~i_cs)||(~i_bus_grant))
			r_clk_counter <= 0;
		else if (byte_accepted)
			r_clk_counter <= i_speed;
		else if (~r_z_counter)
			r_clk_counter <= (r_clk_counter - {{(SPDBITS-1){1'b0}},1'b1});
		else if ((r_state != `LLSDSPI_IDLE)&&(r_state != `LLSDSPI_HOTIDLE))
			r_clk_counter <= (i_speed);
		// else 
		//	r_clk_counter <= 16'h00;
	end

	initial	r_z_counter = 1'b1;
	always @(posedge i_clk)
	begin
		if ((~i_cs)||(~i_bus_grant))
			r_z_counter <= 1'b1;
		else if (byte_accepted)
			r_z_counter <= 1'b0;
		else if (~r_z_counter)
			r_z_counter <= (r_clk_counter == 1);
		else if ((r_state != `LLSDSPI_IDLE)&&(r_state != `LLSDSPI_HOTIDLE))
			r_z_counter <= 1'b0;
	end

	initial	r_state = `LLSDSPI_IDLE;
	always @(posedge i_clk)
	begin
		o_stb <= 1'b0;
		o_cs_n <= ~i_cs;
		if (~i_cs)
		begin
			r_state <= `LLSDSPI_IDLE;
			r_idle <= 1'b0;
			o_sclk <= 1'b1;
		end else if (~r_z_counter)
		begin
			r_idle <= 1'b0;
			if (byte_accepted)
			begin // Will only happen within a hot idle state
				r_byte <= { i_byte[6:0], 1'b1 };
				r_state <= `LLSDSPI_START+1;
				o_mosi <= i_byte[7];
			end
		end else if (r_state == `LLSDSPI_IDLE)
		begin
			o_sclk <= 1'b1;
			if (byte_accepted)
			begin
				r_byte <= i_byte[7:0];
				r_state <= (i_bus_grant)?`LLSDSPI_START:`LLSDSPI_WAIT;
				r_idle <= 1'b0;
				o_mosi <= i_byte[7];
			end else begin
				r_idle <= 1'b1;
			end
		end else if (r_state == `LLSDSPI_WAIT)
		begin
			r_idle <= 1'b0;
			if (i_bus_grant)
				r_state <= `LLSDSPI_START;
		end else if (r_state == `LLSDSPI_HOTIDLE)
		begin
			// The clock is low, the bus is granted, we're just
			// waiting for the next byte to transmit
			o_sclk <= 1'b0;
			if (byte_accepted)
			begin
				r_byte <= i_byte[7:0];
				r_state <= `LLSDSPI_START;
				r_idle <= 1'b0;
				o_mosi <= i_byte[7];
			end else
				r_idle <= 1'b1;
		// end else if (r_state == `LLSDSPI_START)
		// begin
			// o_sclk <= 1'b0;
			// r_state <= r_state + 1;
		end else if (o_sclk)
		begin
			o_mosi <= r_byte[7];
			r_byte <= { r_byte[6:0], 1'b1 };
			r_state <= r_state + 1;
			o_sclk <= 1'b0;
			if (r_state >= `LLSDSPI_START+8)
			begin
				r_state <= `LLSDSPI_HOTIDLE;
				r_idle <= 1'b1;
				o_stb <= 1'b1;
				o_byte <= r_ireg;
			end else
				r_state <= r_state + 1;
		end else begin
			r_ireg <= { r_ireg[6:0], i_miso };
			o_sclk <= 1'b1;
		end
	end

	assign o_idle = (r_idle)&&( (i_cs)&&(i_bus_grant) );
endmodule


