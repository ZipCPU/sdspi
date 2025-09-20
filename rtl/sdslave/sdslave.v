////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rtl/sdslave.v
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
module	sdslave #(
		parameter	NUMIO=4
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Wishbone interface
		// {{{
		output	wire		o_wb_cyc, o_wb_stb, o_wb_we,
		output	wire [AW-1:0]	o_wb_addr,
		output	wire [DW-1:0]	o_wb_data,
		output	wire [DW/8-1:0]	o_wb_sel,
		input	wire		i_wb_stall,
		input	wire		i_wb_ack,
		input	wire [DW-1:0]	i_wb_data,
		input	wire		i_wb_err,
		// }}}
		// SD slave front-end interface
		// {{{
		input	wire		i_sd_clk,
		input	wire		i_sd_cmd,
		input	wire	[15:0]	i_sd_data,
		output	wire	[15:0]	o_sd_data,
		output	wire	[7:0]	o_sd_tristate
		output	wire	[1:0]	o_sd_ds,
		output	wire		o_sd_ds_tristate
		// }}}
		// }}}
	);
