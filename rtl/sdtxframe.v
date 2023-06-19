////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdtxframe.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Given a frame of data from memory, formats it for sending to
//		the front end.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2018-2023, Gisselquist Technology, LLC
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
module	sdtxframe #(
		parameter		NCRC = 16,
		parameter [0:0]		OPT_SERDES = 1'b1,
		parameter [NCRC-1:0]	CRC_POLYNOMIAL  = 16'h1021
	) (
		// {{{
		input	wire			i_clk, i_reset,
		//
		input	wire	[7:0]		i_cfg_spd,
		input	wire	[1:0]		i_cfg_width,
		input	wire			i_cfg_ddr,
		//
		input	wire			i_en, i_ckstb,
		//
		input	wire			S_VALID,
		output	wire			S_READY,
		input	wire	[31:0]		S_DATA,
		input	wire			S_LAST,
		//
		output	wire			tx_valid,
		input	wire			tx_ready,
		output	wire	[31:0]		tx_data
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[1:0]	P_IDLE = 2'b00,
				P_DATA = 2'b01,
				P_CRC  = 2'b10,
				P_LAST = 2'b11;

	localparam [1:0]	WIDTH_1W = 2'b00,
				WIDTH_4W = 2'b01,
				WIDTH_8W = 2'b10;

	reg		cfg_ddr;
	reg	[1:0]	cfg_width, cfg_period;


	wire		start_packet;
	reg		pre_valid;
	reg	[1:0]	pstate;
	wire		pre_ready;
	reg	[31:0]	pre_data;
	reg	[1:0]	pre_count;


	integer		ik, jk;
	reg	[NCRC- 1:0]	crc_1w_reg;
	reg	[NCRC* 4-1:0]	di_crc_4w, nxt_crc_4w, new_crc_4w, crc_4w_reg;
	reg	[NCRC* 8-1:0]	di_crc_8w, nxt_crc_8w, new_crc_8w, crc_8w_reg;
	reg	[NCRC*16-1:0]	di_crc_8d, nxt_crc_8d, new_crc_8d, crc_8d_reg;

	reg		ck_valid;
	wire		ck_ready;
	reg	[4:0]	ck_counts;
	reg	[31:0]	ck_data, ck_sreg;
	// }}}
	// Steps: #1, Packetizer: breaks incoming signal into wires
	// 	#2, add CRC
	//	#3, split across clocks
	//
	////////////////////////////////////////////////////////////////////////
	//
	// Configuration
	// {{{
	initial	cfg_period = 2'b00;
	always @(posedge i_clk)
	if (i_reset || !OPT_SERDES)
		cfg_period <= 2'b00;
	else if (pstate == P_IDLE)
	begin
		if (i_cfg_ddr && i_cfg_spd == 0)
			cfg_period <= 2'b10;	// Four clock periods
		else if (i_cfg_ddr
			||(!i_cfg_ddr && i_cfg_spd == 0))
			cfg_period <= 2'b01;	// Two clock periods
		else
			cfg_period <= 2'b00;	// One clock period
	end

	always @(posedge i_clk)
	if (i_reset)
		cfg_width <= WIDTH_1W;
	else if (pstate == P_IDLE)
		cfg_width <= i_cfg_width;

	always @(posedge i_clk)
	if (i_reset)
		cfg_ddr <= 1'b0;
	else if (pstate == P_IDLE)
		cfg_ddr <= i_cfg_ddr;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Packetizer
	// {{{

	assign	start_packet = i_en && S_VALID && i_ckstb && !tx_valid && !ck_valid;
	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		pstate    <= P_IDLE;
		pre_valid <= 1'b0;
		pre_data  <= {(32){1'b1}};
		// }}}
	end else case(pstate)
	P_IDLE: begin
		// {{{
		pstate <= P_IDLE;
		pre_valid <= 0;
		pre_data <= {(32){1'b1}};
		if (start_packet)
		begin
			pstate <= P_DATA;
			/*
			pre_valid <= 1;
			case(cfg_width)
			WIDTH_1W: pre_data <= { {(24){1'b1}}, 8'hfe };
			WIDTH_4W: pre_data <= { {(24){1'b1}}, 8'hf0 };
			WIDTH_8W: pre_data <= { {(24){1'b1}}, 8'h00 };
			default: pre_data  <= { {(24){1'b1}}, 8'h00 };
			endcase
			*/
		end end
		// }}}
	P_DATA: if (S_VALID && S_READY)
		// {{{
		begin
			pstate <= P_DATA;
			pre_valid <= 1;
			pre_data <= S_DATA;

			if (S_LAST)
				pstate <= P_CRC;
		end
		// }}}
	P_CRC: if (pre_ready)
		// {{{
		begin
			pre_valid <= 1'b1;
			if (pre_count == 0)
				pstate <= P_LAST;

			case(cfg_width)
			WIDTH_1W: pre_data <= { crc_1w_reg[NCRC-2:0], 1'b1, 16'hffff };
			WIDTH_4W: if (cfg_ddr)
				pre_data <= crc_8w_reg[8*NCRC-1:8*NCRC-32];
				else
				pre_data <= crc_4w_reg[4*NCRC-1:4*NCRC-32];
			WIDTH_8W: if (cfg_ddr)
				pre_data <= crc_8d_reg[16*NCRC-1:16*NCRC-32];
				else
				pre_data <= crc_8w_reg[8*NCRC-1:8*NCRC-32];
			default: pre_data  <= crc_8w_reg[8*NCRC-1:8*NCRC-32];
			endcase
		end
		// }}}
	P_LAST: if (pre_ready)
		begin
			pre_valid <= 0;
			pre_data <= {(32){1'b1}};
			pstate <= P_IDLE;
		end
	endcase

	always @(posedge i_clk)
	if (i_reset || pstate == P_DATA)
	begin
		case(cfg_width)
		WIDTH_1W: pre_count <= 0;
		WIDTH_4W: pre_count <= 1;
		default: // WIDTH_8W
			pre_count <= 2;
		endcase
	end else if (pre_ready && pre_count != 0)
		pre_count <= pre_count - 1;

	assign	S_READY = pre_ready && (pstate == P_DATA);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CRC calculation
	// {{{

	// di_crc*, new_crc*, next_crc*: Advance CRC calculations combinatorialy
	// {{{
	always @(*)
	begin
		// Pre-load the CRC fills
		for(ik=0; ik<NCRC; ik=ik+1)
		begin
			for(jk=0; jk<4; jk=jk+1)
				di_crc_4w[jk*NCRC+ik] = crc_4w_reg[ik*4+jk];

			for(jk=0; jk<8; jk=jk+1)
				di_crc_8w[jk*NCRC+ik] = crc_8w_reg[ik*8+jk];

			for(jk=0; jk<16; jk=jk+1)
				di_crc_8d[jk*NCRC+ik] = crc_8d_reg[ik*16+jk];
		end

		// Advance the CRCs based on S_DATA
		for(ik=0; ik<4; ik=ik+1)
		begin
			new_crc_4w[ik*NCRC +: NCRC] = 
				APPLYCRC8(di_crc_4w[ik*NCRC +: NCRC],
			  		{ S_DATA[28+ik],S_DATA[24+ik],
						S_DATA[20+ik],S_DATA[16+ik],
						S_DATA[12+ik],S_DATA[ 8+ik],
						S_DATA[ 4+ik],S_DATA[   ik] });
		end

		for(ik=0; ik<8; ik=ik+1)
		begin
			new_crc_8w[ik*NCRC +: NCRC] = 
				APPLYCRC4(di_crc_8w[ik*NCRC +: NCRC],
			  		{ S_DATA[24+ik], S_DATA[16+ik],
						S_DATA[8+ik], S_DATA[ik] });
		end

		for(ik=0; ik<8; ik=ik+1)
		begin
			// Odd CRC (positive edge)
			new_crc_8d[ik*NCRC*2 +: NCRC] = 
				APPLYCRC2(di_crc_8d[ik*NCRC*2   +: NCRC],
			  		{ S_DATA[24+ik], S_DATA[8+ik] });

			// Even CRC (negitive edge)
			new_crc_8d[ik*NCRC*2 + NCRC +: NCRC] = 
				APPLYCRC2(di_crc_8d[ik*NCRC*2+NCRC +: NCRC],
			  		{ S_DATA[16+ik], S_DATA[ik] });
		end

		// Order the bits to place the results back in line
		for(ik=0; ik<NCRC; ik=ik+1)
		begin
			for(jk=0; jk<4; jk=jk+1)
				nxt_crc_4w[ik*4+jk] = new_crc_4w[jk*NCRC+ik];
			for(jk=0; jk<8; jk=jk+1)
				nxt_crc_8w[ik*8+jk] = new_crc_8w[jk*NCRC+ik];
			for(jk=0; jk<16; jk=jk+1)
				nxt_crc_8d[ik*16+jk] = new_crc_8d[jk*NCRC+ik];
		end
	end
	// }}}

	// crc_*_reg: Advance CRC calculations sequentially w/ new data
	// {{{
	always @(posedge i_clk)
	if (i_reset || pstate == P_IDLE || pstate == P_LAST)
	begin
		crc_1w_reg <= 0;
		crc_4w_reg <= 0;
		crc_8w_reg <= 0;
		crc_8d_reg <= 0;
	end else if (S_VALID && S_READY)
	begin
		crc_1w_reg <= {(NCRC   ){1'b1}};
		crc_4w_reg <= {(NCRC* 4){1'b1}};
		crc_8w_reg <= {(NCRC* 8){1'b1}};
		crc_8d_reg <= {(NCRC*16){1'b1}};

		case(cfg_width)
		WIDTH_1W: crc_1w_reg <= APPLYCRC32(crc_1w_reg, S_DATA);
		WIDTH_4W: if (cfg_ddr)
				crc_8w_reg <= nxt_crc_8w;
			else
				crc_4w_reg <= nxt_crc_4w;
		WIDTH_8W: if (cfg_ddr)
				crc_8w_reg <= nxt_crc_8w;
			else
				crc_8d_reg <= nxt_crc_8d;
		default: begin end
		endcase
	end else if (i_ckstb && pstate != P_DATA)
	begin
		crc_1w_reg <= {(NCRC){1'b1}};
		crc_4w_reg <= { crc_4w_reg[ 4*NCRC-32-1:0], 32'hffff_ffff };
		crc_8w_reg <= { crc_8w_reg[ 8*NCRC-32-1:0], 32'hffff_ffff };
		crc_8d_reg <= { crc_8d_reg[16*NCRC-32-1:0], 32'hffff_ffff };
	end
	// }}}

	function automatic [NCRC-1:0] STEPCRC(reg [NCRC-1:0] prior,
		// {{{
				input i_data);
	begin
		if (prior[NCRC-1] ^ i_data)
			STEPCRC = { prior[NCRC-2:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			STEPCRC = { prior[NCRC-2:0], 1'b0 };
	end endfunction
	// }}}

	function automatic [NCRC-1:0] APPLYCRC32(reg [NCRC-1:0] prior,
		// {{{
			input [31:0] i_data);
		integer		ck;
		reg	[NCRC-1:0]	fill;
	begin
		fill = prior;
		for(ck=0; ck<32; ck=ck+1)
			fill = STEPCRC(fill, i_data[31-ck]);

		APPLYCRC32 = fill;
	end endfunction
	// }}}

	function automatic [NCRC-1:0] APPLYCRC8(reg [NCRC-1:0] prior,
		// {{{
			input [7:0] i_data);
		integer		ck;
		reg	[NCRC-1:0]	fill;
	begin
		fill = prior;
		for(ck=0; ck<8; ck=ck+1)
			fill = STEPCRC(fill, i_data[7-ck]);
		APPLYCRC8 = fill;
	end endfunction
	// }}}

	function automatic [NCRC-1:0] APPLYCRC4(reg [NCRC-1:0] prior,
		// {{{
			input [3:0] i_data);
		integer		ck;
		reg	[NCRC-1:0]	fill;
	begin
		fill = prior;
		for(ck=0; ck<4; ck=ck+1)
			fill = STEPCRC(fill, i_data[3-ck]);
		APPLYCRC4 = fill;
	end endfunction
	// }}}

	function automatic [NCRC-1:0] APPLYCRC2(reg [NCRC-1:0] prior,
		// {{{
			input [1:0] i_data);
		integer		ck;
		reg	[NCRC-1:0]	fill;
	begin
		fill = prior;
		for(ck=0; ck<2; ck=ck+1)
			fill = STEPCRC(fill, i_data[1-ck]);
		APPLYCRC2 = fill;
	end endfunction
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Clock divider, data shift register
	// {{{

	always @(posedge i_clk)
	if (i_reset)
		ck_valid <= 0;
	else if (start_packet)
		ck_valid <= 1;
	else if (pre_valid && ck_ready)
		ck_valid <= 1;
	else if (pre_valid && (!ck_valid || ck_ready))
		ck_valid <= 1;
	else if (ck_ready)
		ck_valid <= 0;

	always @(posedge i_clk)
	if (i_reset) // pstate == P_IDLE)
	begin
		// {{{
		ck_sreg  <= {(32){1'b1}};
		ck_data  <= {(32){1'b1}};
		ck_counts<= 0;
		// }}}
	end else if (pre_valid && (!ck_valid || ck_ready))
	begin
		case(cfg_period)
		2'b00: begin // One clock period of data
			// {{{
			case(cfg_width)
			WIDTH_1W: begin
				ck_sreg <= { pre_data[30: 0], 1'b1 };
				ck_data <= { (4){7'h7f, pre_data[31] } };
				ck_counts <= 31;
				end
			WIDTH_4W: begin
				ck_sreg <= { pre_data[27: 0], 4'hf };
				ck_data <= { (4){ 4'hf, pre_data[31:28] } };
				ck_counts <= 7;
				end
			default: begin // WIDTH_8W
				ck_sreg <= { pre_data[23: 0], 8'hff };
				ck_data <= { (4){pre_data[31:24] } };
				ck_counts <= 3;
				end
			endcase end
			// }}}
		2'b01: begin // Two clock periods of data
			// {{{
			case(cfg_width)
			WIDTH_1W: begin
				ck_sreg <= { pre_data[29: 0], 2'b11 };
				ck_data <= { {(2){7'h7f, pre_data[31] }},
						{(2){7'h7f, pre_data[30] }} };
				ck_counts <= 15;
				end
			WIDTH_4W: begin
				ck_sreg <= { pre_data[23: 0], 8'hff };
				ck_data <= { {(2){4'hf, pre_data[31:28] }},
						{(2){4'hf, pre_data[27:24] }} };
				ck_counts <= 3;
				end
			default: begin // WIDTH_8W
				ck_sreg <= { pre_data[15: 0], 16'hffff };
				ck_data <= { {(2){pre_data[31:24] }},
						{(2){pre_data[23:16]}} };
				ck_counts <= 31;
				end
			endcase end
			// }}}
		default: begin // Four clock periods of data
			// {{{
			case(cfg_width)
			WIDTH_1W: begin
				ck_sreg <= { pre_data[27: 0], 4'hf };
				ck_data <= { 7'h7f, pre_data[31],
						7'h7f, pre_data[30],
						7'h7f, pre_data[29],
						7'h7f, pre_data[28] };
				ck_counts <= 7;
				end
			WIDTH_4W: begin
				ck_sreg <= { pre_data[15: 0], 16'hffff };
				ck_data <= { 4'hf, pre_data[31:28],
					4'hf, pre_data[27:24],
					4'hf, pre_data[23:20],
					4'hf, pre_data[19:16] };
				ck_counts <= 3;
				end
			default: begin // WIDTH_8W
				ck_sreg <= {(32){1'b1}};
				ck_data <= pre_data;
				ck_counts <= 31;
				end
			endcase end
			// }}}
		endcase
	end else if (i_ckstb && ck_counts > 0)
	begin // Transition to idle
		ck_counts <= ck_counts - 1;
		case(cfg_period)
		2'b00: begin // One clock period of data
			// {{{
			case(cfg_width)
			WIDTH_1W: begin
				ck_sreg <= { ck_sreg[30: 0], 1'b1 };
				ck_data <= { (4){7'h7f, ck_sreg[31] }};
				end
			WIDTH_4W: begin
				ck_sreg <= { ck_sreg[27: 0], 4'hf };
				ck_data <= { (4){4'hf, ck_sreg[31:28] } };
				end
			default: begin // WIDTH_8W
				ck_sreg <= { ck_sreg[23: 0], 8'hff };
				ck_data <= { (4){ck_sreg[31:24] } };
				end
			endcase end
			// }}}
		2'b01: begin // Two clock periods of data
			// {{{
			case(cfg_width)
			WIDTH_1W: begin
				ck_sreg <= { ck_sreg[29: 0], 2'b11 };
				ck_data <= { {(2){ck_sreg[31], 7'h7f }},
						{(2){ck_sreg[30],7'h7f}} };
				end
			WIDTH_4W: begin
				ck_sreg <= { ck_sreg[23: 0], 8'hff };
				ck_data <= { {(2){ck_sreg[31:28], 4'hf }},
						{(2){ck_sreg[27:24], 4'hf }} };
				end
			default: begin // WIDTH_8W
				ck_sreg <= { ck_sreg[15: 0], 16'hffff };
				ck_data <= { {(2){ck_sreg[31:24] }},
						{(2){ck_sreg[23:16]}} };
				end
			endcase end
			// }}}
		default: begin // Four clock periods of data
			// {{{
			case(cfg_width)
			WIDTH_1W: begin
				ck_sreg <= { ck_sreg[27: 0], 4'hf };
				ck_data <= { 7'h7f, ck_sreg[31],
						7'h7f, ck_sreg[30],
						7'h7f, ck_sreg[29],
						7'h7f, ck_sreg[28] };
				end
			WIDTH_4W: begin
				ck_sreg <= { ck_sreg[15: 0], 16'hffff };
				ck_data <= { 4'hf, ck_sreg[31:28],
					4'hf, ck_sreg[27:24],
					4'hf, ck_sreg[23:20],
					4'hf, ck_sreg[19:16] };
				end
			default: begin // WIDTH_8W
				ck_sreg <= {(32){1'b1}};
				ck_data <= ck_sreg;
				end
			endcase end
			// }}}
		endcase
	end else begin
		if (ck_ready)
		begin // Fully idle
			ck_valid <= 0;
			if (i_ckstb)
			begin
				ck_data  <= -1;
				ck_sreg  <= -1;
			end
		end

		if (start_packet)
		case(cfg_period)
		2'b00: begin // One clock period
			case(cfg_width)
			WIDTH_1W: ck_data  <= {(4){ 8'hfe }};
			WIDTH_4W: ck_data  <= {(4){ 8'hf0 }};
			default:  ck_data  <= {(4){ 8'h00 }};
			endcase end
		default: begin // 2-4 clock periods
			case(cfg_width)
			WIDTH_1W: ck_data  <= { {(2){8'hff}}, {(2){8'hfe}} };
			WIDTH_4W: ck_data  <= { {(2){8'hff}}, {(2){8'hf0}} };
			default:  ck_data  <= { {(2){8'hff}}, {(2){8'h00}} };
			endcase end
		endcase
	end

	assign	pre_ready = (ck_counts == 0) && (!ck_valid || ck_ready);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Final outputs
	// {{{
	assign	tx_valid = ck_valid;
	assign	ck_ready = tx_ready;
	assign	tx_data  = ck_data;
	// }}}

	//
	// Make verilator happy
	// verilator lint_off UNUSED
	// wire	unused;
	// assign	unused = i_wb_cyc;
	// verilator lint_on  UNUSED
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
	reg	f_past_valid;
	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

`endif	// FORMAL
// }}}
endmodule

