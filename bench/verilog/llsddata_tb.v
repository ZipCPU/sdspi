////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	llsddata_tb.v
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
`default_nettype none
`timescale 1ns / 1ns
//
module llsddata_tb;

	reg	i_clk, i_reset, i_pedge, i_nedge, i_stb;
	reg	[7:0]	i_byte;
	wire		wr_busy, rd_busy;
	reg		i_expect_read;
	wire		ign_stb,  rd_stb;
	wire	[7:0]	ign_byte, rd_byte;
	wire		ign_err,  rd_err;
	reg		i_sd_data;
	wire		o_sd_data, ign_sd_data;

	llsddata
	mutwr(i_clk, i_reset, i_pedge, i_nedge,
		i_stb, 8'hff, wr_busy,
		1'b0, ign_stb, ign_byte, ign_err,
		1'b1, o_sd_data);

	llsddata
	mutrd(i_clk, i_reset, i_pedge, i_nedge,
		1'b0, 8'hxx, rd_busy,
		1'b1, rd_stb, rd_byte, rd_err,
		o_sd_data, ign_sd_data);

	initial	i_clk = 1;
	always
		#10 i_clk = ~i_clk;

	initial	i_reset = 1;
	always @(posedge i_clk)
		i_reset = 0;

	initial
	begin
		i_byte = 8'hff;
		i_expect_read = 0;
		i_sd_data = 1;
		i_pedge = 1;
		i_nedge = 1;
	end

	reg	[12:0]	bit_count;
	reg	[12:0]	wr_byte_count, rd_byte_count;
	initial	bit_count = 0;
	initial	wr_byte_count = 0;
	always @(posedge i_clk)
	if (i_pedge)
	begin
		if (bit_count == 0)
			bit_count <= bit_count + (o_sd_data == 0);
		else
			bit_count <= bit_count + 1;

		if (i_stb && !wr_busy)
			wr_byte_count <= wr_byte_count + 1;
	end

	reg	[23:0]	simcount;
	initial	simcount = 0;
	always @(posedge i_clk)
	if (i_reset)
		simcount <= 0;
	else
		simcount <= simcount + 1;

	initial	i_stb = 0;
	always @(posedge i_clk)
	if (simcount == 2)
		i_stb <= 1;
	else if (wr_byte_count == 511 && !wr_busy)
		i_stb <= 0;

	always @(bit_count)
	begin
		if (bit_count > 512*8+32+8)
			$finish;
	end

	always @(posedge i_clk)
	if (i_reset)
		rd_byte_count <= 0;
	else if (rd_stb)
		rd_byte_count <= rd_byte_count + 1;

	initial begin
		$dumpfile("llsddata_tb.vcd");
		$dumpvars(0,llsddata_tb);
	end
endmodule
