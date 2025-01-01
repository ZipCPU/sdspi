////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/mdl_sdtx.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Model the IO associated with transmitting data from an SD card
//		via the SDIO interface.
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
module mdl_sdtx #(
		parameter realtime FF_HOLD  = 1.25
	) (
		// {{{
		input	wire		rst_n,
		//
		inout	wire		sd_clk,
		inout	wire	[7:0]	sd_dat,
		output	wire		sd_ds,
		input	wire		i_en,
		input	wire	[1:0]	i_width,
		input	wire		i_ddr,
		input	wire		i_ppull,
		//
		input	wire		i_crcack, i_crcnak,
		//
		input	wire		i_valid,
		output	wire		o_ready,
		input	wire	[31:0]	i_data,
		input	wire		i_last
		// }}}
	);

	// Local declarations
	// {{{
	localparam	NCRC = 16;
	localparam [NCRC-1:0]	CRC_POLYNOMIAL = 16'h1021;

	genvar		gk;

	reg	[31:0]	ddr_idata;
	reg	[15:0]	crc	[15:0];
	reg	[79:0]	tx_sreg;
	reg	[5:0]	r_count;
	reg		r_crc, r_active, ds;
	reg		r_ready, r_token;

	wire		w_drive;
	wire	[7:0]	w_dat;
	// }}}

	// r_ready -- set on the posedge of the clock
	// {{{
	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
		r_ready <= 1'b0;
	else if (r_token || i_crcack || i_crcnak || (i_valid && o_ready))
		r_ready <= 1'b0;
	else if (!i_en)
		r_ready <= 1'b1;
	else
		r_ready <= (r_active && r_count <= (1 + i_ddr));
	// }}}

	// ddr_idata
	// {{{
	always @(*)
	if (!i_ddr || i_width[1])
		ddr_idata = i_data;
	else if (i_width[0])
		ddr_idata = {	i_data[31:28], i_data[23:20],
				i_data[27:24], i_data[19:16],
				i_data[15:12], i_data[ 7: 4],
				i_data[11: 8], i_data[ 3: 0] };
	else
		ddr_idata = {	i_data[31], i_data[23],
				i_data[30], i_data[22],
				i_data[29], i_data[21],
				i_data[28], i_data[20],
				i_data[27], i_data[19],
				i_data[26], i_data[18],
				i_data[25], i_data[17],
				i_data[24], i_data[16],
				i_data[15], i_data[ 7],
				i_data[14], i_data[ 6],
				i_data[13], i_data[ 5],
				i_data[12], i_data[ 4],
				i_data[11], i_data[ 3],
				i_data[10], i_data[ 2],
				i_data[ 9], i_data[ 1],
				i_data[ 8], i_data[ 0] };
	// }}}

	// tx_sreg, r_count, r_crc r_active: positive edge of the clock
	// {{{
	// Setup for the positive clock edge
	always @(negedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		tx_sreg <= 0;
		r_count <= 0;
		r_crc   <= 0;
		ds      <= 0;
		r_active<= 0;
		r_token <= 0;
	end else if (r_token)
	begin // We're sending a token
		// {{{
		r_count  <= r_count - 1;
		r_token  <= (r_count > 1);
		r_crc    <= 0;
		r_active <= 0;

		if (r_count > 1)
			ds <= #FF_HOLD 1'b1;

		if (i_width[0]) // 4b
			tx_sreg <= { tx_sreg[75:0], 4'hf };
		else if (i_width[1]) // 8b
			tx_sreg <= { tx_sreg[71:0], 8'hff };
		else
			tx_sreg <= { tx_sreg[78:0], 1'b1 };
		// }}}
	end else if (i_crcack || i_crcnak)
	begin // Receive a request to send a token
		// {{{
		r_token  <= 1;
		r_crc    <= 0;
		r_active <= 0;
		r_count  <= i_ddr ? 6'd10 : 6'd5;

		ds <= #FF_HOLD 1'b1;

		if (i_width[0]) // 4b
		begin
			if (i_ddr)
				tx_sreg  <= #FF_HOLD { 8'h0,
					(i_crcnak) ? 4'hf : 4'h0, 4'hx,
					(i_crcnak) ? 4'h0 : 4'hf, 4'hx,
					(i_crcnak) ? 4'hf : 4'h0, 4'hx,
					4'hf, 4'hx, 40'hff_ffff_ffff };
			else
				tx_sreg  <= #FF_HOLD { 4'h0,
					(i_crcnak) ? 4'hf : 4'h0,
					(i_crcnak) ? 4'h0 : 4'hf,
					(i_crcnak) ? 4'hf : 4'h0,
					4'hf, {(60){1'b1}} };
		end else if (i_width[1]) // 8b
		begin
			if (i_ddr)
				tx_sreg  <= #FF_HOLD { 16'h0,
					(i_crcnak) ? 8'hff : 8'h00, 8'hx,
					(i_crcnak) ? 8'h00 : 8'hff, 8'hx,
					(i_crcnak) ? 8'hff : 8'h00, 8'hx,
					8'hff, 8'hx };
			else
				tx_sreg  <= #FF_HOLD { 8'h0,
					(i_crcnak) ? 8'hff : 8'h00,
					(i_crcnak) ? 8'h00 : 8'hff,
					(i_crcnak) ? 8'hff : 8'h00,
					8'hff, {(40){1'b1}} };
		end else if (i_ddr)
		begin
			tx_sreg  <= #FF_HOLD { 2'h0,
					(i_crcnak) ? 1'b1 : 1'b0, 1'hx,
					(i_crcnak) ? 1'b0 : 1'b1, 1'hx,
					(i_crcnak) ? 1'b1 : 1'b0, 1'hx,
					1'b1, 1'hx, {(70){1'b1}} };
		end else begin
			tx_sreg  <= #FF_HOLD { 1'h0,
					(i_crcnak) ? 1'b1 : 1'b0,
					(i_crcnak) ? 1'b0 : 1'b1,
					(i_crcnak) ? 1'b1 : 1'b0,
					1'b1, {(75){1'b1}} };
		end
		// }}}
	end else if (!i_en)
	begin
		// {{{
		tx_sreg <= 0;
		r_count <= 0;
		r_crc   <= 0;
		ds      <= 0;
		r_active<= 0;
		r_token <= 0;
		// }}}
	end else if (i_valid && o_ready)
	begin // New data
		// {{{
		ds <= #FF_HOLD 1'b1;

		if (!r_active)
		begin // New data, plus a start bit
			// {{{
			if (i_width[0])
			begin // 4b width
				if (i_ddr)
					tx_sreg  <= #FF_HOLD { 4'b0, 4'bx, ddr_idata, 8'hff, 32'hffff_ffff };
				else
					tx_sreg  <= #FF_HOLD { 4'b0, i_data, 12'hfff, 32'hffff_ffff };
				r_count  <= 9 + (i_ddr ? 1:0);
			end else if (i_width[1])
			begin // 8b width
				if (i_ddr)
					tx_sreg  <= #FF_HOLD { 8'b0, 8'bx, ddr_idata, 32'hffff_ffff };
				else
					tx_sreg  <= #FF_HOLD { 8'b0, i_data, 8'hff, 32'hffff_ffff };
				r_count  <= 5 + (i_ddr ? 1:0);
			end else begin // 1b width
				if (i_ddr)
					tx_sreg  <= #FF_HOLD { 1'b0, 1'bx, ddr_idata, 6'h3f, 8'hff, 32'hffff_ffff };
				else
					tx_sreg  <= #FF_HOLD { 1'b0, i_data, 7'h7f, 8'hff, 32'hffff_ffff };
				r_count  <= 33 + (i_ddr ? 1:0);
			end
			// }}}
		end else begin
			tx_sreg  <= #FF_HOLD { ddr_idata, 16'hffff, 32'hffff_ffff };
			r_count  <= (i_width[0]) ? 8 : (i_width[1]) ? 4 : 32;
		end
		r_active <= 1'b1;
		r_crc    <= 1'b0;
		// }}}
	end else if (r_active)
	begin
		ds <= #FF_HOLD 1'b1;

		r_count <= r_count - 1;
		if (i_width[0])
			tx_sreg <= #FF_HOLD { tx_sreg[75:0], 4'hf };
		else if (i_width[1])
			tx_sreg <= #FF_HOLD { tx_sreg[71:0], 8'hff };
		else
			tx_sreg <= #FF_HOLD { tx_sreg[78:0], 1'b1 };

		if (r_crc || (!r_crc && r_count <= 1))
		begin
			if (i_width[0])
				tx_sreg <= #FF_HOLD { crc[3][15],
					crc[2][15], crc[1][15], crc[0][15],
					44'hfff_ffff_ffff, 32'hffff_ffff };
			else if (i_width[1])
				tx_sreg <= #FF_HOLD {
				crc[7][15], crc[6][15], crc[5][15], crc[4][15],
				crc[3][15], crc[2][15], crc[1][15], crc[0][15],
					40'hff_ffff_ffff, 32'hffff_ffff };
			else
				tx_sreg <= #FF_HOLD { crc[0][15], 7'h7f,
					40'hff_ffff_ffff, 32'hffff_ffff };
		end

		if (r_count <= 1)
		begin
			if (!r_crc)
			begin
				r_crc <= 1'b1;
				r_count <= 16 + (i_ddr ? 16:0);
			end else
				r_active <= 0;
		end
	end
	// }}}

	// Negative clock edge
	// {{{
	initial	r_active = 1'b0;
	always @(posedge sd_clk)
	if (rst_n)
		ds <= #FF_HOLD 1'b0;

	always @(posedge sd_clk)
	if (!rst_n)
	begin
	end else if (i_ddr && (r_active || r_token))
	begin
		r_count <= #FF_HOLD r_count - 1;
		if (i_width[0])
			tx_sreg <= #FF_HOLD { tx_sreg[75:0], 4'hf };
		else if (i_width[1])
			tx_sreg <= #FF_HOLD { tx_sreg[71:0], 8'hff };
		else
			tx_sreg <= #FF_HOLD { tx_sreg[78:0], 1'b1 };

		if (r_crc)
		begin
			if (i_width[0])
				tx_sreg <= #FF_HOLD { crc[11][15],
					crc[10][15], crc[9][15], crc[8][15],
					44'hfff_ffff_ffff, 32'hffff_ffff };
			else if (i_width[1])
				tx_sreg <= #FF_HOLD {
				crc[15][15],crc[14][15],crc[13][15],crc[12][15],
				crc[11][15],crc[10][15],crc[ 9][15],crc[ 8][15],
					40'hff_ffff_ffff, 32'hffff_ffff };
			else
				tx_sreg <= #FF_HOLD { crc[8][15], 7'h7f,
					40'hff_ffff_ffff, 32'hffff_ffff };
		end

		if (r_count <= 1)
		begin
			if (r_token)
			begin
				if (r_count > 1)
					r_token <= #FF_HOLD 1'b0;
			end else if (!r_crc)
			begin
				r_crc <= #FF_HOLD 1'b1;
				r_count <= #FF_HOLD 32;
			end else
				r_active <= #FF_HOLD 0;
		end
	end
	// }}}

	assign	w_dat[0] = (i_width[0]) ? tx_sreg[76]
			: (i_width[1]) ? tx_sreg[72] : tx_sreg[79];

	assign	w_dat[3:1] = i_width[0] ? tx_sreg[79:77]
						: tx_sreg[75:73];
	assign	w_dat[7:4] = tx_sreg[79:76];

	assign	w_drive = r_active || r_token;
	assign	sd_dat[0] = (!w_drive || (!i_ppull && w_dat[0])) ? 1'bz : w_dat[0];
	assign	sd_dat[1] = (!w_drive || (!i_ppull && w_dat[1]) || (i_width == 2'b00)) ? 1'bz : w_dat[1];
	assign	sd_dat[2] = (!w_drive || (!i_ppull && w_dat[2]) || (i_width == 2'b00)) ? 1'bz : w_dat[2];
	assign	sd_dat[3] = (!w_drive || (!i_ppull && w_dat[3]) || (i_width == 2'b00)) ? 1'bz : w_dat[3];
	assign	sd_dat[4] = (!w_drive || (!i_ppull && w_dat[4]) || (!i_width[1])) ? 1'bz : w_dat[4];
	assign	sd_dat[5] = (!w_drive || (!i_ppull && w_dat[5]) || (!i_width[1])) ? 1'bz : w_dat[5];
	assign	sd_dat[6] = (!w_drive || (!i_ppull && w_dat[6]) || (!i_width[1])) ? 1'bz : w_dat[6];
	assign	sd_dat[7] = (!w_drive || (!i_ppull && w_dat[7]) || (!i_width[1])) ? 1'bz : w_dat[7];

	assign	sd_ds = ds;

	assign	o_ready = !r_token && (!r_active || (!r_crc && r_ready));
		// ((sd_clk && r_count == 1) || (!sd_clk && r_ready))));

	// CRC generation
	// {{{
	generate for(gk=0; gk<8; gk=gk+1)
	begin : GEN_CRC
		reg	[15:0]	pedge_crc, nedge_crc;	// DEBUG ONLY signals

		always @(posedge sd_clk or negedge rst_n)
		if (!rst_n)
			crc[gk] <= 0;
		else if (!i_en || r_token)
			crc[gk] <= 0;
		else if (!r_crc)
			crc[gk] <= STEPCRC(crc[gk], w_dat[gk]);
		else
			crc[gk] <= crc[gk] << 1;

		always @(negedge sd_clk or negedge rst_n)
		if (!rst_n)
			crc[8+gk] <= 0;
		else if (!i_ddr || !i_en || !i_ddr || r_token)
			crc[8+gk] <= 0;
		else if (!r_crc)
			crc[8+gk] <= STEPCRC(crc[8+gk], w_dat[gk]);
		else
			crc[8+gk] <= crc[8+gk] << 1;

		always @(*) pedge_crc = crc[  gk];
		always @(*) nedge_crc = crc[8+gk];

	end endgenerate
	// }}}

	function automatic [NCRC-1:0] STEPCRC(input [NCRC-1:0] prior,
		// {{{
				input i_bit);
	begin
		if (prior[NCRC-1] ^ i_bit)
			STEPCRC = { prior[NCRC-2:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			STEPCRC = { prior[NCRC-2:0], 1'b0 };
	end endfunction
	// }}}

	// Keep Verilator happy (it won't be w/o timing support, but ...)
	// {{{
	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_last };
	// Verilator lint_on  UNUSED
	// }}}
endmodule
