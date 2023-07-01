////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	tb_txframe.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	An ad-hoc test of the transmitter.  Engages the SDIO transmitter
//		in each and every mode, while also verifying that the CRCs
//	are properly detected and decoded.
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
module	tb_txframe;
	// Local declarations
	// {{{
	parameter [0:0]	OPT_SERDES = 1'b1;
	reg		clk, reset;
	reg	[7:0]	cfg_spd;
	reg	[1:0]	cfg_width;
	reg		cfg_ddr, tx_en;
	wire		ckstb, hlfck;
	wire	[7:0]	ckwide;
	reg		S_VALID, S_LAST;
	reg	[31:0]	S_DATA;
	wire		tx_valid;
	wire	[31:0]	tx_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Clock generator
	// {{{
	sdckgen
	u_ckgen (
		// {{{
		.i_clk(clk), .i_reset(reset),
		//
		.i_cfg_clk90(1'b1), .i_cfg_ckspd(cfg_spd),
		.i_cfg_shutdown(1'b0),
		//
		.o_ckstb(ckstb), .o_hlfck(hlfck), .o_ckwide(ckwide)
		// }}}
	);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Module under test
	// {{{
	wire	S_READY;

	sdtxframe #(
		.OPT_SERDES(OPT_SERDES)
	) u_txframe (
		// {{{
		.i_clk(clk), .i_reset(reset),
		//
		.i_cfg_spd(cfg_spd), .i_cfg_width(cfg_width),
		.i_cfg_ddr(cfg_ddr),
		//
		//
		.i_en(tx_en), .i_ckstb(ckstb), .i_hlfck(hlfck),
		//
		.S_VALID(S_VALID), .S_READY(S_READY), .S_DATA(S_DATA),
			.S_LAST(S_LAST),
		//
		.tx_valid(tx_valid), .tx_data(tx_data)
		// }}}
	);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Test script
	// {{{

	task	send_packet(input [8:0] length);
		// {{{
		integer	counter;
	begin
		counter = 0;
		@(posedge clk)
		begin
			tx_en <= 0;
			S_VALID <= 0;
			S_DATA  <= $random;
			S_LAST  <= 0;
		end

		@(posedge clk)
			tx_en <= 1;
		@(posedge clk)
			S_VALID <= 1;

		do begin
			@(posedge clk)
			if (!S_VALID || S_READY)
			begin
				S_VALID <= !S_LAST;
				S_DATA  <= $random;
				S_LAST  <= (counter+1) >= length;
				counter <= counter + 1;
			end

			wait(!clk);
		end while(counter < length);

		do begin
			@(posedge clk)
			if (S_READY)
				S_VALID <= 0;

			wait(!clk);
		end while(S_VALID);

		wait(!tx_valid);
		@(posedge clk);
		@(posedge clk)
			tx_en <= 1'b0;
		@(posedge clk);
		@(posedge clk);
	end endtask
	// }}}

	initial begin
		$dumpfile("tb_txframe.vcd");
		$dumpvars(0, tb_txframe);

		clk = 0;
		reset = 1;
		forever
			#5 clk = !clk;
	end

	initial begin
		tx_en     = 1'b0;
		cfg_spd   = 8'h1;
		cfg_width = 2'b10;
		cfg_ddr   = 1'b1;
		S_VALID   = 1'b0;
		S_DATA    = 32'b0;
		S_LAST    = 1'b0;

		@(posedge clk)
		begin
			reset <= 1'b0;
		end

		// HS200
		// {{{
		@(posedge clk)
		begin
			$display("TEST # 1: SPD=0,8W,DDR");
			cfg_spd   = 8'h0;
			cfg_width = 2'b10;
			cfg_ddr   = 1'b1;
		end send_packet(128);

		@(posedge clk)
		begin
			$display("TEST # 2: SPD=0,4W,DDR");
			cfg_spd   = 8'h0;
			cfg_width = 2'b01;
			cfg_ddr   = 1'b1;
		end send_packet(128);

		@(posedge clk)
		begin
			$display("TEST # 3: SPD=0,1W,DDR");
			cfg_spd   = 8'h0;
			cfg_width = 2'b00;
			cfg_ddr   = 1'b1;
		end send_packet(128);
		// }}}

		// DDR100
		// {{{
		@(posedge clk)
		begin
			$display("TEST # 4: SPD=1,8W,DDR");
			cfg_spd   = 8'h1;
			cfg_width = 2'b10;
			cfg_ddr   = 1'b1;
		end send_packet(5);

		@(posedge clk)
		begin
			$display("TEST # 5: SPD=1,4W,DDR");
			cfg_spd   = 8'h1;
			cfg_width = 2'b01;
			cfg_ddr   = 1'b1;
		end send_packet(5);

		@(posedge clk)
		begin
			$display("TEST # 6: SPD=1,1W,DDR");
			cfg_spd   = 8'h1;
			cfg_width = 2'b00;
			cfg_ddr   = 1'b1;
		end send_packet(5);
		// }}}

		// SDR100
		// {{{
		@(posedge clk)
		begin
			$display("TEST # 7: SPD=1,8W,SDR");
			cfg_spd   = 8'h1;
			cfg_width = 2'b10;
			cfg_ddr   = 1'b0;
		end send_packet(5);

		@(posedge clk)
		begin
			$display("TEST # 8: SPD=1,4W,SDR");
			cfg_spd   = 8'h1;
			cfg_width = 2'b01;
			cfg_ddr   = 1'b0;
		end send_packet(5);

		@(posedge clk)
		begin
			$display("TEST # 9: SPD=1,1W,SDR");
			cfg_spd   = 8'h1;
			cfg_width = 2'b00;
			cfg_ddr   = 1'b0;
		end send_packet(5);
		// }}}

		// DDR50
		// {{{
		@(posedge clk)
		begin
			$display("TEST #10: SPD=2,8W,DDR");
			cfg_spd   = 8'h2;
			cfg_width = 2'b10;
			cfg_ddr   = 1'b1;
		end send_packet(5);

		@(posedge clk)
		begin
			$display("TEST #11: SPD=2,4W,DDR");
			cfg_spd   = 8'h2;
			cfg_width = 2'b01;
			cfg_ddr   = 1'b1;
		end send_packet(5);

		@(posedge clk)
		begin
			$display("TEST #12: SPD=2,1W,DDR");
			cfg_spd   = 8'h2;
			cfg_width = 2'b00;
			cfg_ddr   = 1'b1;
		end send_packet(5);
		// }}}

		// SDR50
		// {{{
		@(posedge clk)
		begin
			$display("TEST #13: SPD=2,8W,SDR");
			cfg_spd   = 8'h2;
			cfg_width = 2'b10;
			cfg_ddr   = 1'b0;
		end send_packet(128);

		@(posedge clk)
		begin
			$display("TEST #14: SPD=2,4W,SDR");
			cfg_spd   = 8'h2;
			cfg_width = 2'b01;
			cfg_ddr   = 1'b0;
		end send_packet(128);

		@(posedge clk)
		begin
			$display("TEST #15: SPD=2,1W,SDR");
			cfg_spd   = 8'h2;
			cfg_width = 2'b00;
			cfg_ddr   = 1'b0;
		end send_packet(128);
		// }}}

		// SDRSLOW
		// {{{
		@(posedge clk)
		begin
			$display("TEST #16: SPD=3,8W,SDR");
			cfg_spd   = 8'h3;
			cfg_width = 2'b10;
			cfg_ddr   = 1'b0;
		end send_packet(128);

		@(posedge clk)
		begin
			$display("TEST #17: SPD=3,4W,SDR");
			cfg_spd   = 8'h3;
			cfg_width = 2'b01;
			cfg_ddr   = 1'b0;
		end send_packet(128);

		@(posedge clk)
		begin
			$display("TEST #18: SPD=3,1W,SDR");
			cfg_spd   = 8'h3;
			cfg_width = 2'b00;
			cfg_ddr   = 1'b0;
		end send_packet(128);
		// }}}

		$finish;
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CRC checking
	// {{{
	genvar	gk;
	reg	crc_fsm, pc, ph, dc, dh, pre_fsm;
	parameter		NCRC = 16;
	parameter [NCRC-1:0]	CRC_POLYNOMIAL  = 16'h1021;
	reg	[7:0]		rail_fail, eval_rail;
	reg			crc_fail;

	initial	crc_fail = 1'b0;

	initial	{ crc_fsm, pre_fsm } = 1'b0;
	always @(posedge clk)
	if (reset)
		{ crc_fsm, pre_fsm } <= 0;
	else if (pc && !tx_valid)
		{ crc_fsm, pre_fsm } <= 0;
	else if (pre_fsm && !crc_fsm)
		crc_fsm <= 1'b1;
	else if (!tx_data[0] && pc && !crc_fsm)
		{ crc_fsm, pre_fsm } <= { 1'b0, 1'b1 };

	initial	{ pc, ph } = 2'b00;
	always @(posedge clk)
		{ dc, dh, pc, ph } <= { pc, ph, ckstb, hlfck };

	always @(posedge clk)
	if (S_VALID && S_READY && 0)
	begin
		if (cfg_width == 2'b00 && !cfg_ddr)
			$display("DATA: %08x %1d", S_DATA, S_LAST);
		else if (cfg_width == 2'b00 && cfg_ddr)
			$display("DATA: %04x %1d", {S_DATA[31], S_DATA[29],
				 S_DATA[27], S_DATA[25],
				 S_DATA[23], S_DATA[21],
				 S_DATA[19], S_DATA[17],
				 S_DATA[15], S_DATA[13],
				 S_DATA[11], S_DATA[ 9],
				 S_DATA[ 7], S_DATA[ 5],
				 S_DATA[ 3], S_DATA[ 1] }, S_LAST);
		else if (0 && cfg_width == 2'b01 && !cfg_ddr)
			$display("DATA: %02x %1d",
				{S_DATA[28], S_DATA[24], S_DATA[20],S_DATA[16],
				 S_DATA[12], S_DATA[ 8], S_DATA[ 4],S_DATA[ 0]},
				S_LAST);
		else if (0 && cfg_width == 2'b01 && cfg_ddr)
			$display("DATA: %01x %1d", {S_DATA[28], S_DATA[20],
				 S_DATA[12], S_DATA[ 4] }, S_DATA, S_LAST);
		else if (0 && cfg_width == 2'b10 && !cfg_ddr)
			$display("DATA: %1x %1d", { S_DATA[24],S_DATA[16],
					S_DATA[ 8],S_DATA[ 0] }, S_LAST);
		else if (cfg_width == 2'b10 && cfg_ddr)
			$display("DATA: %08x -> %1x %1d", S_DATA, { S_DATA[24],
					S_DATA[ 8] }, S_LAST);
	end

	generate for(gk=0; gk<8; gk=gk+1)
	begin : GEN_CRC_CHECK
		reg	[3:0]	delay;
		wire	[3:0]	vector;
		reg	[15:0]	crcfill, hlffill, psreg, nsreg;
		reg	[15:0]	crcpast	[0:15];
		reg	[15:0]	hlfpast	[0:15];
		reg		validpin;
		integer		sp, sh;

		initial	rail_fail[gk] = 1'b0;
		initial	eval_rail[gk] = 1'b0;

		assign	vector = { tx_data[gk+24], tx_data[gk+16],
				tx_data[gk+8], tx_data[gk] };

		initial	validpin = 0;
		always @(posedge clk)
		if (gk == 0 || cfg_width == 2'b10)
			validpin <= 1;
		else if (gk < 4 && cfg_width != 2'b00)
			validpin <= 1;
		else
			validpin <= 0;

		// always @(*) delay = vector;
		always @(posedge clk) delay <= vector;

		always @(posedge clk)
		if (!crc_fsm || !validpin)
		begin
			crcfill <= 0;
			eval_rail[gk] <= 1'b0;
			rail_fail[gk] <= 1'b0;
		end else if (crc_fsm && tx_valid && pc)
		begin
			if (cfg_spd == 0)
				crcfill <= STEPCRC(
					STEPCRC(crcfill, delay[3]), delay[1]);
			else
				crcfill <= STEPCRC(crcfill, delay[3]);
		end else if (crc_fsm && !tx_valid && pc && validpin)
		begin
			assert(crcfill == 0);
			eval_rail[gk] <= 1'b1;
			rail_fail[gk] <= (crcfill != 0);
		end

		reg	[15:0]	last_half;

		always @(posedge clk)
		if (crc_fsm && tx_valid && ph)
			last_half <= hlffill;

		always @(posedge clk)
		if (!crc_fsm || !validpin || !cfg_ddr)
			hlffill <= 0;
		else if (crc_fsm && tx_valid && ph)
		begin
			if (cfg_spd == 0)
			begin
				hlffill <= STEPCRC(
					STEPCRC(hlffill, delay[2]), delay[0]);
			end else
				hlffill <= STEPCRC(crcfill, delay[2]);
		end else if (crc_fsm && !tx_valid && pc && validpin)
		begin
			if (cfg_spd < 2)
				assert(hlffill == 0);
			else
				assert(last_half == 0);
			eval_rail[gk] <= 1'b1;
			if (last_half != 0)
				rail_fail[gk] <= 1'b1;
		end
	end endgenerate

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
	// }}}
endmodule
