////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	fclk.v
// {{{
// Project:	SD-Card controller
//
// Purpose:
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2024, Gisselquist Technology, LLC
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
`default_nettype none
// }}}
module	fclk #(
		parameter	[0:0]	OPT_SERDES = 1'b0,
					OPT_DDR    = 1'b0
	) (
		// {{{
		input	wire		i_clk, i_reset,
		//
		input	wire		i_en,
		input	wire	[7:0]	i_ckspd,
		input	wire		i_clk90,
		//
		input	wire		i_ckstb, i_hlfck,
		input	wire	[7:0]	i_ckwide,
		//
		output	reg		f_pending_reset,
		output	reg		f_pending_half
		// }}}
	);

`ifdef	CKGEN
`define	SLAVE_ASSUME	assert
`else
`define	SLAVE_ASSUME	assume
`endif

	reg		f_past_tick, f_past_valid;
	reg		last_reset, last_en, last_pending;
	reg	[7:0]	last_ckspd;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	initial	f_pending_reset = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		f_pending_reset <= 1'b1;
	else if (i_ckstb || i_hlfck)
		f_pending_reset <= 1'b0;

	initial	f_pending_half = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		f_pending_half <= 1'b0;
	else if (i_ckstb)
		f_pending_half <= !i_hlfck;
	else if (i_hlfck)
		f_pending_half <= 1'b0;

	initial	f_past_tick = 0;
	always @(posedge i_clk)
		f_past_tick <= i_ckstb || i_hlfck;

	// always @(*)
	// if (!i_reset && !f_pending_reset && i_en && i_ckspd == 2)
	//	`SLAVE_ASSUME({ i_ckstb, i_hlfck } == (f_pending_half ? 2'b01:2'b10));

	always @(posedge i_clk)
	if (!i_reset && !f_pending_reset)
	begin
		if (f_pending_half)
			`SLAVE_ASSUME(!i_ckstb);
		else if (i_hlfck)
			`SLAVE_ASSUME(i_ckstb);
	end

	always @(posedge i_clk)
	if (!i_reset)
	case(i_ckspd)
	0: begin
		// {{{
		`SLAVE_ASSUME(OPT_SERDES);
		`SLAVE_ASSUME(f_pending_reset || !f_pending_half);
		if (i_ckwide == 0)
		begin
			`SLAVE_ASSUME(f_pending_reset || (!i_ckstb && !i_hlfck));
		end else begin
			`SLAVE_ASSUME(i_ckstb && i_hlfck);
		end

		if (i_clk90)
		begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'h66);
		end else begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'h33);
		end end
		// }}}
	1: begin
		// {{{
		if (i_ckwide == 0)
		begin
			`SLAVE_ASSUME(f_pending_reset || (!i_ckstb && !i_hlfck));
		end else begin
			`SLAVE_ASSUME(i_ckstb && i_hlfck);
		end

		if (!f_pending_reset)
			`SLAVE_ASSUME(!f_pending_half);

		if (i_clk90)
		begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'h3c);
			`SLAVE_ASSUME(OPT_SERDES);
		end else begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'h0f);
			`SLAVE_ASSUME(OPT_SERDES || OPT_DDR);
		end end
		// }}}
	2: begin
		// {{{
		if (i_clk90)
		begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'h0f || i_ckwide == 8'hf0);
			if (i_en)
			begin
				`SLAVE_ASSUME(i_ckwide != 0);
			end
			`SLAVE_ASSUME(OPT_SERDES || OPT_DDR);
			if (!f_pending_reset && f_pending_half)
			begin
				`SLAVE_ASSUME(i_ckwide == 8'hf0);
			end
			if (i_ckwide == 8'h00)
			begin
				`SLAVE_ASSUME(!i_ckstb && !i_hlfck);
			end else if (i_ckwide == 8'h0f)
			begin
				`SLAVE_ASSUME(i_ckstb);
			end else begin
				`SLAVE_ASSUME(i_hlfck);
			end
		end else begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'hff);
			if (!f_pending_reset)
			begin
				if (i_ckstb)
				begin
					`SLAVE_ASSUME(1 || i_ckwide == 8'h00);
				end else if (i_hlfck)
				begin
					`SLAVE_ASSUME(1 || i_ckwide == 8'hff);
				end else if (f_pending_half)
				begin
					`SLAVE_ASSUME(1 || i_ckwide == 8'h00);
				end else // if (!f_pending_half)
					`SLAVE_ASSUME(1 || i_ckwide == 8'hff);
			end
			if (i_ckwide == 8'hff)
				`SLAVE_ASSUME(i_hlfck);
		end end
		// }}}
	default: begin
			`SLAVE_ASSUME(i_ckwide == 0 || i_ckwide == 8'hff);
			// if (f_past_tick && !f_pending_reset)
			//	`SLAVE_ASSUME(!i_ckstb && !i_hlfck);
			if (!f_pending_reset && !i_clk90 && last_en && i_en)
			begin
				if (i_ckstb)
				begin
					`SLAVE_ASSUME(i_ckwide == 8'h00);
				end else if (i_hlfck)
				begin
					`SLAVE_ASSUME(i_ckwide == 8'hff);
				end else if (f_pending_half)
				begin
					`SLAVE_ASSUME(i_ckwide == 8'h00);
				end else // if (!f_pending_half)
					`SLAVE_ASSUME(i_ckwide == 8'hff);
			end
		end
	endcase

	always @(posedge i_clk)
	if (!OPT_SERDES && !OPT_DDR)
		assert(!i_ckstb || !i_hlfck);

	always @(posedge i_clk)
	if (f_past_valid && !last_reset && (last_en || i_ckstb || i_hlfck))
	begin
		case(i_ckspd)
		0: `SLAVE_ASSUME(!i_en || (i_ckstb && i_hlfck));
		1: `SLAVE_ASSUME(!i_en || (i_ckstb && i_hlfck));
		default:
			`SLAVE_ASSUME(!i_ckstb || !i_hlfck);
		endcase
	end

	always @(posedge i_clk)
		last_reset <= i_reset;

	always @(posedge i_clk)
		last_en <= i_en;

	always @(posedge i_clk)
		last_ckspd <= i_ckspd;

	always @(posedge i_clk)
		last_pending <= f_pending_reset;

	always @(posedge i_clk)
	if (!i_reset && f_past_valid && !last_reset && last_en && i_en
			&& i_ckspd == last_ckspd)
	begin
		if (i_ckspd <= 1)
			assert(i_ckstb && i_hlfck);
	end

	always @(posedge i_clk)
	if (!i_reset && f_past_valid && f_past_tick && !last_pending
			&& last_ckspd == i_ckspd && i_ckspd >= 3)
	begin
		if (f_past_tick && !f_pending_reset)
			`SLAVE_ASSUME(!i_ckstb && !i_hlfck);
	end
endmodule
