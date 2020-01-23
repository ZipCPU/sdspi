////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdrxframe.v
//
// Project:	SDIO SD-Card controller, using a shared SPI interface
//
// Purpose:	
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2018-2020, Gisselquist Technology, LLC
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
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
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
module	sdrxframe(i_clk, i_reset,
		//
		i_length, i_stb, i_data, o_byte, o_wr, o_done, o_err);
	parameter	LGLEN = 15, NCHANS=4;
	//
	input	wire			i_clk, i_reset;
	//
	input	wire	[LGLEN-1:0]	i_length;
	input	wire			i_stb;
	input	wire	[NCHANS-1:0]	i_data;
	output	wire	[7:0]		o_byte;
	output	wire			o_wr;	// Write this value to memory?
	output	wire			o_done;
	output	wire			o_err;
	//
	//

	reg	busy, last;
	reg	[14:0]	count;
	reg	[LGLEN-1:0]		length;

	always @(posedge i_clk)
	if (i_reset)
	begin
		count <= 0;
		busy <= 0;
	end else if (!busy)
	begin
		if ((i_stb)&&(i_data == 0))
		begin
			count <= count + 1;
			busy <= 1;
			length <= i_length;
		end else
			count <= 0;
	end else if (busy)
	begin
		count <= count + 1;
		busy <= (count == length + 16);
		last <= (count == length + 16-1);
	end

	initial	last = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		last <= 1'b0;
	else
		last <= (busy)&&(count == length-1);

	reg	[15:0]		crc	[0:NCHANS-1];
	wire	[NCHANS-1:0]	good, err;

	genvar	k;
	generate for(k=0; k<NCHANS; k=k+1)
	begin
		wire	maskd_bit;

		assign	maskd_bit = crc[k][0] ^ i_data[k];

		initial	crc[k] = 0;
		always @(posedge i_clk)
		if (i_reset)
			crc[k] <= 0;
		else if ((busy)&&(i_stb))
		begin
			if (maskd_bit)
				crc[k] <= { 1'b0, crc[k][15:1] } ^ 16'h8208;
			else
				crc[k] <= { 1'b0, crc[k][15:1] };
		end

		assign	good[k] = (last)&&(crc[k] == 0);
		assign	err[k]  = (last)&&(crc[k] != 0);
	end endgenerate

	initial	o_done = 0;
	always @(posedge i_clk)
	if (i_reset)
		o_done <= 0;
	else
		o_done <= (last)&&(&good);

	// assign	o_done = (&good);
	assign	o_err  = (&err);

	reg	[3:0]	word;

	initial	word = 0;
	always @(posedge i_clk)
	if (i_reset)
		word <= 0;
	else if (i_stb)
		word <= i_data;

	always @(posedge i_clk)
	if (i_reset)
		o_byte <= 0;
	else if ((i_stb)&&(busy)&&(count[0])&&(count <= length))
		o_byte <= { word, i_data };

	always @(posedge i_clk)
		o_wr <= (!i_reset)&&(i_stb)&&(busy)
				&&(count[0])&&(count <= length);

	//
	// Make verilator happy
	// verilator lint_off UNUSED
	// wire	unused;
	// assign	unused = i_wb_cyc;
	// verilator lint_on  UNUSED

`ifdef	FORMAL
	reg	f_past_valid;
	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
		assume(i_stb = (!i_reset));

	reg	[14:0]	f_count;
	initial	f_count = 0;
	always @(posedge i_clk)
	if (i_reset)
		f_count <= 0;
	else if (i_stb)
		f_count <= f_count + 1;

	always @(posedge i_clk)
		assert(f_count == count);

	always @(posedge i_clk)
	if (i_reset)
		assume(i_data == 0);

	always @(*)
		assume(i_length == 4096);
	always @(*)
	if (busy)
		assert(length == 4096);
	always @(*)
		assume(i_stb);

	else if (i_stb)
	begin
		if (f_step < length)
			assume(i_data == 4'hf);
		else
			assert(f_step != length);
	end

	always @(posedge i_clk)
		assert(count < length+16);
	always @(posedge i_clk)
		busy = (count >0)&&(count < length+16);

`endif	// FORMAL
endmodule

