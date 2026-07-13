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
// Copyright (C) 2016-2026, Gisselquist Technology, LLC
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
		parameter realtime tODLY  = 1.25
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
	reg	[7:0]	tx_out;
	reg	[5:0]	r_count;
	reg		r_crc, r_active, ds;
	reg		r_ready, r_token, r_ddr_started,
			pedge_token, pedge_active, r_start_bit;

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

	// tx_sreg, tx_out, r_count, r_crc r_active: positive edge of the clock
	// {{{
	// Setup for the positive clock edge
	initial	r_active = 1'b0;
	always @(negedge sd_clk or negedge rst_n)
	if (!rst_n)
	begin
		tx_sreg <= -1;
		r_count <= 0;
		r_crc   <= 0;
		ds      <= 0;
		r_active<= 0;
		r_token <= 0;
		r_start_bit <= 1'b0;
	end else if (r_token)
	begin // We're sending a token
		// {{{
		r_count  <= r_count - 1;
		r_token  <= (r_count > 1);
		r_crc    <= 0;
		r_active <= 0;
		r_start_bit <= 1'b0;

		if (r_count > 1)
			ds <= #tODLY 1'b1;

		if (i_width[0]) // 4b
		begin
			tx_sreg <= { tx_sreg[75:0], 4'hf };
			tx_out  <= #tODLY { 4'hf, tx_sreg[75:72] };
		end else if (i_width[1]) // 8b
		begin
			tx_sreg <= { tx_sreg[71:0], 8'hff };
			tx_out  <= #tODLY tx_sreg[71:64];
		end else
		begin
			tx_sreg <= { tx_sreg[78:0], 1'b1 };
			tx_out  <= #tODLY { 7'h7f, tx_sreg[78] };
		end
		// }}}
	end else if (i_crcack || i_crcnak)
	begin // Receive a request to send a token
		// {{{
		r_token  <= 1;
		r_crc    <= 0;
		r_active <= 0;
		r_start_bit <= 1'b0;
		// r_count  <= i_ddr ? 6'd10 : 6'd5;
		// Force us to be busy for longer than the token, to force
		// BOOT to wait until the token has been received and processed
		r_count  <= i_ddr ? (6'd10 + 6'd4) : (6'd5 + 6'd2);

		ds <= #tODLY 1'b1;

		if (i_width[0]) // 4b
		begin
			// {{{
			if (i_ddr)
			begin
				tx_sreg  <= { 8'hee,
					(i_crcnak) ? 4'hf : 4'he, 4'hx,
					(i_crcnak) ? 4'he : 4'hf, 4'hx,
					(i_crcnak) ? 4'hf : 4'he, 4'hx,
					4'hf, 4'hx, 40'hff_ffff_ffff };
			end else begin
				tx_sreg  <= { 4'he,
					(i_crcnak) ? 4'hf : 4'he,
					(i_crcnak) ? 4'he : 4'hf,
					(i_crcnak) ? 4'hf : 4'he,
					4'hf, {(60){1'b1}} };
			end

			tx_out <= #tODLY { 4'hf, 4'he };
			// }}}
		end else if (i_width[1]) // 8b
		begin
			// {{{
			if (i_ddr)
			begin
				tx_sreg  <= { 16'hfefe,
					(i_crcnak) ? 8'hff : 8'hfe, 8'hx,
					(i_crcnak) ? 8'hfe : 8'hff, 8'hx,
					(i_crcnak) ? 8'hff : 8'hfe, 8'hx,
					8'hff, 8'hx };
			end else begin
				tx_sreg  <= { 8'hfe,
					(i_crcnak) ? 8'hff : 8'hfe,
					(i_crcnak) ? 8'hfe : 8'hff,
					(i_crcnak) ? 8'hff : 8'hfe,
					8'hff, {(40){1'b1}} };
			end

			tx_out <= #tODLY { 8'hfe };
			// }}}
		end else if (i_ddr)
		begin // 1b DDR
			// {{{
			tx_sreg  <= { 2'h0,
					(i_crcnak) ? 1'b1 : 1'b0, 1'hx,
					(i_crcnak) ? 1'b0 : 1'b1, 1'hx,
					(i_crcnak) ? 1'b1 : 1'b0, 1'hx,
					1'b1, 1'hx, {(70){1'b1}} };
			tx_out <= #tODLY { 7'h7f, 1'b0 };
			// }}}
		end else begin // 1b SDR
			// {{{
			tx_sreg  <= { 1'h0,
					(i_crcnak) ? 1'b1 : 1'b0,
					(i_crcnak) ? 1'b0 : 1'b1,
					(i_crcnak) ? 1'b1 : 1'b0,
					1'b1, {(75){1'b1}} };
			tx_out <= #tODLY { 7'h7f, 1'b0 };
			// }}}
		end
		// }}}
	end else if (!i_en)
	begin
		// {{{
		tx_sreg <= {(80){1'b1}};
		tx_out  <= #tODLY 8'hff;
		r_count <= 0;
		r_crc   <= 0;
		ds      <= 0;
		r_active<= 0;
		r_token <= 0;
		r_start_bit <= 1'b0;
		// }}}
	end else if (i_valid && o_ready)
	begin // New data
		// {{{
		ds <= #tODLY 1'b1;
		r_start_bit <= 1'b0;

		if (!r_active)
		begin // New data, plus a start bit
			// {{{
			r_start_bit <= 1'b1;
			if (i_width[0])
			begin // 4b width
				if (i_ddr)
					tx_sreg  <= { 4'b0, 4'bx, ddr_idata, 8'hff, 32'hffff_ffff };
				else
					tx_sreg  <= { 4'b0, i_data, 12'hfff, 32'hffff_ffff };
				tx_out <= #tODLY { 4'hf, 4'h0 };
				r_count  <= 9 + (i_ddr ? 1:0);
			end else if (i_width[1])
			begin // 8b width
				if (i_ddr)
					tx_sreg  <= { 8'b0, 8'bx, ddr_idata, 32'hffff_ffff };
				else
					tx_sreg  <= { 8'b0, i_data, 8'hff, 32'hffff_ffff };
				tx_out <= #tODLY 8'h00;
				r_count  <= 5 + (i_ddr ? 1:0);
			end else begin // 1b width
				if (i_ddr)
					tx_sreg  <= { 1'b0, 1'bx, ddr_idata, 6'h3f, 8'hff, 32'hffff_ffff };
				else
					tx_sreg  <= { 1'b0, i_data, 7'h7f, 8'hff, 32'hffff_ffff };
				tx_out <= #tODLY { 7'h7f, 1'b0 };
				r_count  <= 33 + (i_ddr ? 1:0);
			end
			// }}}
		end else begin
			tx_sreg  <= { ddr_idata, 16'hffff, 32'hffff_ffff };
			if (i_width[0])
				tx_out <= #tODLY { 4'hf, ddr_idata[31:28] };
			else if (i_width[1])
				tx_out <= #tODLY ddr_idata[31:24];
			else // if (i_width == 2'b00)
				tx_out <= { 7'h7f, ddr_idata[31] };
			r_count  <= (i_width[0]) ? 8 : (i_width[1]) ? 4 : 32;
		end
		r_active <= 1'b1;
		r_crc    <= 1'b0;
		// }}}
	end else if (r_active)
	begin
		ds <= #tODLY 1'b1;
		r_start_bit <= 1'b0;

		r_count <= r_count - 1;
		// Advance the shift register
		// {{{
		if (i_width[0])
		begin
			tx_sreg <= { tx_sreg[75:0], 4'hf };
			tx_out  <= #tODLY { 4'hf, tx_sreg[75:72] };
		end else if (i_width[1])
		begin
			tx_sreg <= { tx_sreg[71:0], 8'hff };
			tx_out  <= #tODLY tx_sreg[71:64];
		end else begin
			tx_sreg <= { tx_sreg[78:0], 1'b1 };
			tx_out  <= #tODLY { 7'h7f, tx_sreg[78] };
		end
		// }}}

		if (r_crc || (!r_crc && r_count <= 1))
		begin // Insert CRC cycles into the shift register
			// {{{
			if (i_width[0])
			begin
				tx_sreg <= { crc[3][15],
					crc[2][15], crc[1][15], crc[0][15],
					44'hfff_ffff_ffff, 32'hffff_ffff };
				tx_out <= #tODLY { crc[3][15],
					crc[2][15], crc[1][15], crc[0][15] };
			end else if (i_width[1])
			begin
				tx_sreg <= {
				crc[7][15], crc[6][15], crc[5][15], crc[4][15],
				crc[3][15], crc[2][15], crc[1][15], crc[0][15],
					40'hff_ffff_ffff, 32'hffff_ffff };
				tx_out <= #tODLY {
				crc[7][15], crc[6][15], crc[5][15], crc[4][15],
				crc[3][15], crc[2][15], crc[1][15], crc[0][15]};
			end else begin
				tx_sreg <= { crc[0][15], 7'h7f,
					40'hff_ffff_ffff, 32'hffff_ffff };
				tx_out <= #tODLY { 7'h7f, crc[0][15] };
			end
			// }}}
		end

		if (r_count <= 1)
		begin
			if (!r_crc)
			begin
				r_crc <= 1'b1;
				r_count <= 16 + (i_ddr ? 16:0);
			end else
				r_active <= #tODLY 1'b0;
		end
	end
	// }}}

	// Negative clock edge
	// {{{
	always @(posedge sd_clk)
	if (rst_n)
		ds <= #tODLY 1'b0;

	always @(posedge sd_clk or negedge rst_n)
	if (!rst_n)
		{ pedge_active, pedge_token } <= 1'b0;
	else
		{ pedge_active, pedge_token } <= { r_active, r_token };

	always @(posedge sd_clk)
	if (!rst_n)
		r_ddr_started <= 1'b0;
	else if (!r_active || !i_ddr || !i_en)
		// No token check here, since DDR doesn't do tokens
		r_ddr_started <= 1'b0;
	else if (r_active && r_start_bit)
		r_ddr_started <= 1'b1;

	always @(posedge sd_clk)
	if (!rst_n)
	begin
	end else if (i_ddr && ((r_active && (r_ddr_started || r_start_bit))
							|| r_token))
	begin
		r_count <= r_count - 1;
		if (i_width[0])			// 4b
		begin
			tx_sreg <= { tx_sreg[75:0], 4'hf };
			tx_out  <= #tODLY { 4'hf, tx_sreg[75:72] };
		end else if (i_width[1])	// 8b
		begin
			tx_sreg <= { tx_sreg[71:0], 8'hff };
			tx_out  <= #tODLY tx_sreg[71:64];
		end else begin
			tx_sreg <= { tx_sreg[78:0], 1'b1 };
			tx_out  <= #tODLY { 7'h7f, tx_sreg[78] };
		end

		if (r_crc)
		begin // Insert the CRC
			// {{{
			if (i_width[0])
			begin
				tx_sreg <= { crc[11][15],
					crc[10][15], crc[9][15], crc[8][15],
					44'hfff_ffff_ffff, 32'hffff_ffff };
				tx_out <= #tODLY { 4'hf, crc[11][15],
					crc[10][15], crc[9][15], crc[8][15] };
			end
			else if (i_width[1])
			begin
				tx_sreg <= {
				crc[15][15],crc[14][15],crc[13][15],crc[12][15],
				crc[11][15],crc[10][15],crc[ 9][15],crc[ 8][15],
					40'hff_ffff_ffff, 32'hffff_ffff };
				tx_out <= #tODLY {
				crc[15][15],crc[14][15],crc[13][15],crc[12][15],
				crc[11][15],crc[10][15],crc[ 9][15],crc[ 8][15]
					};
			end else begin
				tx_sreg <= { crc[8][15], 7'h7f,
					40'hff_ffff_ffff, 32'hffff_ffff };
				tx_out  <= #tODLY { 7'h7f, crc[8][15] };
			end
			// }}}
		end

		if (r_count <= 1)
		begin // Clear r_token, set r_crc, or clear r_active
			// {{{
			if (r_token)
			begin
				if (r_count > 1)
					r_token <= #tODLY 1'b0;
			end else if (!r_crc)
			begin
				r_crc <= #tODLY 1'b1;
				r_count <= 32;
			end else
				r_active <= #tODLY 0;
			// }}}
		end
	end
	// }}}

	assign	w_dat[0] = tx_out[0];
		// (i_width[0]) ? tx_out[76]
		//	: (i_width[1]) ? tx_out[72] : tx_out[79];

	assign	w_dat[3:1] = tx_out[3:1];
			// i_width[0] ? tx_sreg[79:77] : tx_sreg[75:73];
	assign	w_dat[7:4] = tx_out[7:4];
			// tx_sreg[79:76];

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

	assign	o_ready = (!pedge_token) && (!pedge_active || (!r_crc && r_ready));
		// ((sd_clk && r_count == 1) || (!sd_clk && r_ready))));

	// CRC generation
	// {{{
	generate for(gk=0; gk<8; gk=gk+1)
	begin : GEN_CRC
		reg	[15:0]	pedge_crc, nedge_crc;	// DEBUG ONLY signals
		reg		lcl_dat, lcl_active;

		// lcl_dat
		// {{{
		// Ideally, we'd use w_dat to handle our CRC's but ... we can't.
		// If tODLY is any larger than one clock tick, w_dat will not
		// match pos/negedge sd_clk.  tx_sreg will, however, so we use
		// that.  We'll trim it just a touch here to make sure that our
		// pin is active in the current mode, but otherwise simply
		// reference the appropriate tx_sreg pin.
		//
		// This is a bit of a challenge, since tx_sreg[79:76] contains
		// the pins for 4b mode, tx_sreg[79:72] contains the pins for
		// 8b mode, and tx_sreg[79] contains the (single) pin value for
		// 1b mode--so it takes a bit of a lookup.  tx_out doesn't have
		// this problem, but tx_out already has the delay applied to it.
		always @(*)
		if (i_width[0])
			lcl_dat = (gk < 4) ? tx_sreg[76+gk] : 1'b0;
		else if (i_width[1])
			lcl_dat = tx_sreg[72+gk];
		else
			lcl_dat = (gk == 0) ? tx_sreg[79] : 1'b0;
		// }}}

		// lcl_active
		// {{{
		always @(*)
		if (!i_en || !r_active)
			lcl_active = 1'b0;
		else if (i_width[0])
			lcl_active = (gk < 4);
		else if (i_width[1])
			lcl_active = 1;
		else
			lcl_active = (gk == 0);
		// }}}

		// The positive edge CRC fill register
		// {{{
		always @(posedge sd_clk or negedge rst_n)
		if (!rst_n)
			crc[gk] <= 0;
		else if (!lcl_active || r_token || pedge_token)
			crc[gk] <= 0;
		else if (!r_crc)
			crc[gk] <= STEPCRC(crc[gk], lcl_dat);
		else
			crc[gk] <= crc[gk] << 1;
		// }}}

		// Negative edge CRC calculation
		// {{{
		always @(negedge sd_clk or negedge rst_n)
		if (!rst_n)
			crc[8+gk] <= 0;
		else if (!r_ddr_started || !lcl_active || r_token)
			crc[8+gk] <= 0;
		else if (!r_crc)
			crc[8+gk] <= STEPCRC(crc[8+gk], lcl_dat);
		else
			crc[8+gk] <= crc[8+gk] << 1;
		// }}}

		// pedge_crc and nedge_crc are used for generating useful
		//  simulation traces only.  They are not used for synthesis.
		//  As such, nothing reads these.  They are completely ignored.
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
