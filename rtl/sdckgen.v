////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdckgen.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Generate a digitally divided pre-serdes clock.  The serdes this
//		will feed is an 8:1 serdes.  Hence, we generate 8 outputs per
//	clock period.  This allows us to generate a clock with a 90 degree
//	offset, without needing to actually offset the clock by 90 degrees.
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
`timescale	1ns/1ps
`default_nettype	none
// }}}
module	sdckgen #(
		// {{{
		parameter [0:0]	OPT_SERDES = 0,
		parameter [0:0]	OPT_DDR = 0,
		// To hit 100kHz from a 100MHz clock, we'll need to divide by
		// 4, and then by another 250.  Hence, we'll need Lg(256)-2
		// bits.  (The first three are special)
		localparam	LGMAXDIV = 8
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		//
		input	wire			i_cfg_clk90,
		input	wire	[LGMAXDIV-1:0]	i_cfg_ckspd,
		input	wire			i_cfg_shutdown,
		//
		output	reg			o_ckstb,
		output	reg			o_hlfck,
		output	reg	[7:0]		o_ckwide,
		output	wire			o_clk90,
		output	reg	[LGMAXDIV-1:0]	o_ckspd
		// }}}
	);

	// Local declarations
	// {{{
	localparam		NCTR = LGMAXDIV+2;
	reg			nxt_stb, nxt_clk;
	reg	[NCTR-1:0]	nxt_counter, counter;
	reg			clk90;
	reg	[LGMAXDIV-1:0]	ckspd;
	wire			w_clk90;
	wire	[LGMAXDIV-1:0]	w_ckspd;
	reg	[LGMAXDIV-1:0]	new_ckspd;
	// }}}

	// nxt_stb, nxt_clk, nxt_counter
	// {{{
	always @(*)
	begin
		nxt_stb = 1'b0;
		nxt_clk = 1'b0;
		nxt_counter = counter;

		{ nxt_stb, nxt_counter[NCTR-3:0] } = counter[NCTR-3:0] - 1;

		if (nxt_stb)
		begin
			// Advance the top two bits
			{ nxt_clk, nxt_counter[NCTR-1:NCTR-2] }
						= nxt_counter[NCTR-1:NCTR-2] +1;

			if ((OPT_DDR || OPT_SERDES) && ckspd <= 1)
			begin
				nxt_clk = 1;
				nxt_counter[NCTR-3:0] = 0;
			end else if (ckspd <= 2)
			begin
				nxt_clk = counter[NCTR-1];
				nxt_counter[NCTR-3:0] = 0;
			end else
				nxt_counter[NCTR-3:0] = ckspd-3;
		end

		if (nxt_clk)
		begin
			if ((OPT_DDR || OPT_SERDES) && new_ckspd <= 1)
				nxt_counter = {2'b11, {(NCTR-2){1'b0}} };
			else if (new_ckspd <= 2)
				nxt_counter = { 2'b01, {(NCTR-2){1'b0}} };
			else begin
				nxt_counter[NCTR-1:NCTR-2] = 0;
				nxt_counter[NCTR-3:0] = new_ckspd-3;
			end end
	end

	always @(posedge i_clk)
	if (i_reset)
	begin
		if (OPT_SERDES)
			counter <= 0;
		else if (OPT_DDR)
			counter <= { 2'b11, {(NCTR-2){1'b0}} };
		else
			counter <= { 2'b01, {(NCTR-2){1'b0}} };
	end else if (nxt_clk && i_cfg_shutdown)
		counter <= { 2'b11, {(NCTR-2){1'b0}} };
	else
		counter <= nxt_counter;
	// }}}

	// w_clk90, w_ckspd: Register the requested clock speed
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		clk90 <= 0;
	else
		clk90 <= w_clk90;
	assign	o_clk90 = clk90;

	initial	ckspd = (OPT_SERDES) ? 8'd0 : (OPT_DDR) ? 8'd1 : 8'd2;
	always @(posedge i_clk)
	if (i_reset)
		ckspd <= (OPT_SERDES) ? 8'd0 : (OPT_DDR) ? 8'd1 : 8'd2;
	else
		ckspd <= w_ckspd;

	always @(*)
	if (OPT_SERDES)
		new_ckspd = i_cfg_ckspd;
	else if (OPT_DDR && i_cfg_ckspd <= 1 && !i_cfg_clk90)
		new_ckspd = 1;
	else if (i_cfg_ckspd <= 2 && (OPT_DDR || !i_cfg_clk90))
		new_ckspd = 2;
	else if (i_cfg_ckspd <= 3)
		new_ckspd = 3;
	else
		new_ckspd = i_cfg_ckspd;

	assign	w_clk90 = (nxt_clk) ? i_cfg_clk90 : clk90;
	assign	w_ckspd = (nxt_clk) ? new_ckspd   : ckspd;
	// }}}

	// o_ckstb, o_ckwide
	// {{{
	initial	o_ckstb  = 1;
	initial	o_hlfck  = (OPT_SERDES || OPT_DDR);
	initial	o_ckwide = 0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		o_ckstb  <= 1;
		o_hlfck  <= (OPT_SERDES || OPT_DDR);
		o_ckwide <= 0;
	end else if (nxt_clk && i_cfg_shutdown)
	begin
		o_ckstb  <= 1'b0;
		o_hlfck  <= 1'b0;
		o_ckwide <= 8'h0;
	end else if (OPT_SERDES && w_ckspd == 0)
	begin
		o_ckstb  <= !i_cfg_shutdown;
		o_hlfck  <= !i_cfg_shutdown;
		o_ckwide <= (i_cfg_clk90) ? 8'h66 : 8'h33;
		if (i_cfg_shutdown)
			o_ckwide <= 8'h0;
	end else if ((OPT_SERDES || OPT_DDR) && w_ckspd <= 1)
	begin
		o_ckstb  <= 1'b1;
		o_hlfck  <= 1'b1;
		o_ckwide <= (OPT_SERDES && w_clk90) ? 8'h3c : 8'h0f;
	end else if (w_ckspd == 2)
	begin
		{ o_ckstb, o_hlfck } <= (!nxt_counter[NCTR-1]) ? 2'b10 : 2'b01;
		if (w_clk90 && (OPT_SERDES || OPT_DDR))
			o_ckwide <= (!nxt_counter[NCTR-1]) ? 8'h0f : 8'hf0;
		else
			o_ckwide <= (!nxt_counter[NCTR-1]) ? 8'h00 : 8'hff;
	end else begin
		o_ckstb <= nxt_clk;
		o_hlfck <= (counter == {2'b01, {(NCTR-2){1'b0}} });
		if (w_clk90)
			o_ckwide <= {(8){nxt_counter[NCTR-1]
						^ nxt_counter[NCTR-2]}};
		else
			o_ckwide <= {(8){nxt_counter[NCTR-1]}};
	end
	// }}}

	always @(posedge i_clk)
		o_ckspd <= w_ckspd;
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
	reg	f_past_valid, f_en;
	wire	f_pending_reset, f_pending_half;

	initial	f_past_valid = 1'b0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);
	////////////////////////////////////////////////////////////////////////
	//
	// Clock properties
	// {{{
	initial	f_en = 1'b1;
	always @(posedge i_clk)
	if (i_reset)
		f_en <= 1'b1;
	else if (nxt_clk)
		f_en <= !i_cfg_shutdown;

	fclk #(
		.OPT_SERDES(OPT_SERDES),
		.OPT_DDR(OPT_DDR)
	) u_ckprop (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		//
		.i_en(f_en),
		.i_ckspd(o_ckspd),
		.i_clk90(clk90),
		//
		.i_ckstb(o_ckstb),
		.i_hlfck(o_hlfck),
		.i_ckwide(o_ckwide),
		//
		.f_pending_reset(f_pending_reset),
		.f_pending_half(f_pending_half)
		// }}}
	);

	always @(*)
	if (!i_reset && !o_hlfck && !o_ckstb && !f_pending_reset)
		assert(f_pending_half == (counter[NCTR-1:NCTR-2] < 2'b10));
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Induction properties
	// {{{
	always @(posedge i_clk)
	if (f_past_valid)
	begin
		if ($past(!i_reset && i_cfg_shutdown))
		begin
			assert(!o_ckstb);
		end

		if (ckspd == 0)
		begin
			assert(OPT_SERDES);
			assert(o_ckstb || $past(i_cfg_shutdown));
			assert(counter == 0
				||counter == {2'b11,{(NCTR-2){1'b0}} });
		end

		if (ckspd == 1)
		begin
			assert(OPT_SERDES || OPT_DDR);
			if (!OPT_SERDES)
			begin
				assert(!clk90);
			end
			assert(counter == {2'b11,{(NCTR-2){1'b0}} });
		end

		if (ckspd == 2)
			assert(counter == 0
				|| counter == {2'b01,{(NCTR-2){1'b0}} }
				|| counter == {2'b10,{(NCTR-2){1'b0}} }
				|| counter == {2'b11,{(NCTR-2){1'b0}} });
		if (ckspd >= 3)
			assert(counter[NCTR-3:0] <= (ckspd-3));
	end

	always @(*)
	if (!i_reset && o_ckstb && o_hlfck)
		assert(ckspd <= 1 || (o_ckwide == 0 && nxt_clk));

	always @(*)
	if (!i_reset)
	case(o_ckwide)
	8'h00: if (nxt_clk)
		begin
			assert(counter == {2'b11,{(NCTR-2){1'b0}} }
					|| ckspd == 0);
		end else if(!clk90)
		begin
			assert(counter[NCTR-1] == 1'b0);
		end else if(clk90)
		begin
			assert(counter[NCTR-1:NCTR-2] == 2'b00
				||counter[NCTR-1:NCTR-2] == 2'b11);
		end
	8'h0f: assert((!clk90 && ckspd == 1 && o_ckstb && o_hlfck)
			||(clk90 && ckspd == 2 && o_ckstb));
	8'hf0: assert(clk90 && ckspd == 2 && !o_ckstb && o_hlfck);
	8'hff: if(!clk90) assert(counter[NCTR-1] == 1'b1);
		else
			assert(counter[NCTR-1:NCTR-2] == 2'b01
				|| counter[NCTR-1:NCTR-2] == 2'b10);
	8'h3c: assert( clk90 && ckspd == 1 && o_ckstb && o_hlfck);
	8'h33: assert(!clk90 && ckspd == 0 && o_ckstb && o_hlfck);
	8'h66: assert( clk90 && ckspd == 0 && o_ckstb && o_hlfck);
	default: assert(0);
	endcase
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Cover checks
	// {{{
	reg		cvr_active, cvr_clk90;
	reg	[7:0]	cvr_spd, cvr_count;

	always @(posedge i_clk)
	if (!cvr_active)
	begin
		cvr_spd <= i_cfg_ckspd;
		cvr_clk90 <= i_cfg_clk90;
	end

	initial	cvr_active = 0;
	always @(posedge i_clk)
	if (i_reset)
		cvr_active <= 1'b0;
	else if (cvr_spd != o_ckspd || cvr_spd != i_cfg_ckspd || !f_en
			|| cvr_clk90 != i_cfg_clk90 || cvr_clk90 != clk90)
		cvr_active <= 0;
	else if (o_ckstb)
		cvr_active <= 1;

	always @(posedge i_clk)
	if (i_reset || !cvr_active)
		cvr_count <= 8'b0;
	else if (o_ckstb && !(&cvr_count))
		cvr_count <= cvr_count + 1;

	always @(posedge i_clk)
	if (!i_reset)
	begin
		cover(cvr_spd == 2 && !clk90 && cvr_count > 2);
		cover(cvr_spd == 3 &&  clk90 && cvr_count > 2);
		cover(cvr_spd == 3 && !clk90 && cvr_count > 2);
		cover(cvr_spd == 4 &&  clk90 && cvr_count > 2);
		cover(cvr_spd == 4 && !clk90 && cvr_count > 2);
		cover(cvr_spd == 5 &&  clk90 && cvr_count > 2);
		cover(cvr_spd == 5 && !clk90 && cvr_count > 2);
		cover(cvr_spd == 6 &&  clk90 && cvr_count > 2);
		cover(cvr_spd == 6 && !clk90 && cvr_count > 2);
	end

	generate if (OPT_SERDES)
	begin : CVR_SERDES

		always @(posedge i_clk)
		if (!i_reset)
		begin
			cover(cvr_spd == 0 &&  clk90 && cvr_count > 5);
			cover(cvr_spd == 1 &&  clk90 && cvr_count > 5);
			cover(cvr_spd == 1 && !clk90 && cvr_count > 5);
			cover(cvr_spd == 2 &&  clk90 && cvr_count > 5);
			cover(cvr_spd == 2 && !clk90 && cvr_count > 5);
		end

	end else if (OPT_DDR)
	begin : CVR_DDR

		always @(posedge i_clk)
		if (!i_reset)
		begin
			cover(cvr_spd == 1 && !clk90 && cvr_count > 5);
			cover(cvr_spd == 2 &&  clk90 && cvr_count > 5);
			cover(cvr_spd == 2 && !clk90 && cvr_count > 5);
		end

	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// "Careless" assumptions
	// {{{
	always @(*)
	if (!i_reset && i_cfg_clk90 && !OPT_SERDES)
	begin
		if (OPT_DDR)
		begin
			assume(1 || i_cfg_ckspd >= 2);
		end else begin
			assume(1 || i_cfg_ckspd >= 3);
		end
	end

	always @(*)
	if (i_cfg_ckspd == 0)
		assume(i_cfg_clk90);

	// }}}
`endif	// FORMAL
// }}}
endmodule

