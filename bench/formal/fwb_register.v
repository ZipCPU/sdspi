////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	fwb_register.v
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	While it may be fairly easy to verify that a core follows the
//		bus protocol, it's another thing to prove that the answers it
//	returns are the right ones.
//
//	This core is meant to be a complement to the fwb_slave logic, for slaves
//	that consist of a series of registers.  This core will test whether a
//	register can be written to using Wishbone, and/or read back properly
//	later.  It assumes a register having a single clock latency.
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
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype none
// }}}
module	fwb_register #(
		// {{{
		parameter		AW = 4,
		parameter		DW = 32,
		parameter [AW-1:0]	ADDR = 0,
		parameter [DW-1:0]	MASK = -1,
		parameter [DW-1:0]	FIXED_BIT_MASK = 0
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		//
		input	wire			i_wb_stb, i_wb_we,
		input	wire	[AW-1:0]	i_wb_addr,
		input	wire	[DW-1:0]	i_wb_data,
		input	wire	[DW/8-1:0]	i_wb_sel,
		input	wire			i_wb_ack,
		input	wire	[DW-1:0]	i_wb_return,
		input	wire	[DW-1:0]	i_register
		// }}}
	);

	// Local register, reset assumption
	// {{{
	integer			ik;
	reg			f_past_valid, past_reset;
	reg	[DW-1:0]	freg, non_ro_write, mask_write;
	wire	[DW-1:0]	error_mask;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);

	always @(*)
		assert((MASK & FIXED_BIT_MASK) == 0);
	// }}}

	// freg
	// {{{
	always @(*)
	begin
		mask_write = (i_reset || past_reset) ? i_register : freg;
		for(ik=0; ik<DW/8; ik=ik+1)
		if (i_wb_sel[ik])
			mask_write[ik*8 +: 8] = i_wb_data[ik*8 +: 8];

		non_ro_write = (mask_write & ~FIXED_BIT_MASK)
				| (freg & FIXED_BIT_MASK);
	end

	initial	past_reset = 1'b1;
	always @(posedge i_clk)
		past_reset <= i_reset;

	always @(posedge i_clk or posedge i_reset)
	if (i_reset)
		freg <= i_register;
	else if (i_wb_stb && i_wb_we && i_wb_addr == ADDR)
		freg <= non_ro_write;
	else if (past_reset)
		freg <= i_register;
	// }}}

	// Comparing freg against i_register
	// {{{
	assign	error_mask = (freg ^ i_register) & MASK;

	always @(posedge i_clk)
	if (!i_reset && !past_reset)
		assert(error_mask == 0);
	// }}}

	// Verifying wb_ack
	// {{{
	always @(posedge i_clk)
	if (!i_reset && $past(!i_reset && i_wb_stb))
		assert(i_wb_ack);
	else if (!i_reset)
		assert(!i_wb_ack);
	// }}}

	// Verifying i_wb_return
	// {{{
	always @(posedge i_clk)
	if (!i_reset && $past(!i_reset && i_wb_stb && !i_wb_we
			&& i_wb_addr == ADDR))
		assert(i_wb_return == $past(i_register));
	// }}}

endmodule
