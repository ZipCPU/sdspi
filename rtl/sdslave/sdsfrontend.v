////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdsfrontend.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	This is the "front-end" for the SDIO slave controller.
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
`timescale		1ns / 1ps
`default_nettype	none
// }}}
module	sdsfrontend #(
		// {{{
		// parameter [0:0]	OPT_SERDES = 1'b0,
		// parameter [0:0]	OPT_DDR = 1'b0,
		parameter [0:0]	OPT_DS = 1'b0,
		// parameter [0:0]	OPT_COLLISION = 1'b0,
		parameter	NUMIO = 8
		// }}}
	) (
		// {{{
		// Control signals
		// Tx path
		// {{{
		input	wire		i_tx_cmd, i_tx_cmd_tristate,

		input	wire	[15:0]	i_tx_data,
		input	wire	[7:0]	i_tx_data_tristate,

		input	wire		i_tx_ds,
		// input	wire		i_ds_tristate,
		// }}}
		// Rx path
		// {{{
		output	wire			o_rx_cmd,
		output	wire			o_cmd_collision,
		//
		output	wire	[15:0]		o_rx_dat,
		// }}}
		// I/O ports
		// {{{
		input	wire			i_ck,
		output	wire			o_ds,
		//
		inout	wire			io_cmd,
		inout	wire	[NUMIO-1:0]	io_dat
		// }}}
		// output	wire	[31:0]	o_debug
		// }}}
	);

	// Local declarations
	// {{{
	genvar		gk;
	wire		i_cmd, o_cmd, cmd_tristate;
	wire	[1:0]		ign_cmd_mine, w_cmd_wide;

	wire	[NUMIO-1:0]	o_dat, dat_tristate, i_dat;

	wire			ds_tristate;
	// }}}

	xsdddr #(
		.OPT_BIDIR(1'b1)
	) u_cmd (
		.i_clk(i_ck),
		.i_en(!i_tx_cmd_tristate),
		.i_data({(2){i_tx_cmd}}),
		.io_pin_tristate(cmd_tristate),
		.i_pin(i_cmd),
		.o_mine(ign_cmd_mine),
		.o_pin(o_cmd),			// Connect to pin
		.o_wide(w_cmd_wide)
	);

	assign	o_rx_cmd = w_cmd_wide[1];

	assign	o_cmd_collision = 1'b0;

	generate for(gk=0; gk<NUMIO; gk=gk+1)
	begin : GEN_IO_DDR
		wire	[1:0]	ign_dat_mine;

		xsdddr #(
			.OPT_BIDIR(1'b1)
		) u_dat (
			.i_clk(i_ck),
			.i_en(!i_tx_data_tristate[gk]),
			.i_data({ i_tx_data[gk+8], i_tx_data[gk] }),
			.io_pin_tristate(dat_tristate[gk]),
			.i_pin(i_dat[gk]),	// IOBUFT
			.o_mine(ign_dat_mine),
			.o_pin(o_dat[gk]),	// IOBUFT
			.o_wide({ o_rx_dat[gk+8], o_rx_dat[gk] })
		);

		// Verilator lint_off UNUSED
		wire	unused_dat;
		assign	unused_dat = &{ 1'b0, ign_dat_mine };
		// Verilator lint_on  UNUSED
	end for(gk=NUMIO; gk<8; gk=gk+1)
	begin : UNUSED_DDR
		assign	{ o_rx_dat[gk+8], o_rx_dat[gk] } = 2'b00;

		// Verilator lint_off UNUSED
		wire	unused_dat;
		assign	unused_dat = &{ 1'b0, i_tx_data[gk+8], i_tx_data[gk],
				i_tx_data_tristate[gk] };
		// Verilator lint_on  UNUSED
	end endgenerate

	////////////////////////////////////////////////////////////////////////
	//
	// Datastrobe support
	// {{{
	////////////////////////////////////////////////////////////////////////
	//

	generate if (OPT_DS)
	begin : GEN_DATASTROBE
		wire	[1:0]	ign_ds_mine;
		wire	[1:0]	ign_ds_wide;

		xsdddr #(
			.OPT_BIDIR(1'b0)
		) u_ds (
			.i_clk(i_ck),
			.i_en(OPT_DS),
			.i_data(i_tx_ds),
			.io_pin_tristate(ds_tristate),
			.i_pin(1'b0),
			.o_mine(ign_ds_mine),
			.o_pin(o_ds),
			.o_wide(ign_ds_wide)
		);

	end else begin : NO_DATASTROBE
		assign	o_ds = 1'b0;
		assign	ds_tristate = 1'b1;

		// Verilator lint_off UNUSED
		wire	unused_ds;
		assign	unused_ds = &{ 1'b0, i_tx_ds, ds_tristate };
		// Verilator lint_on  UNUSED
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// IO buffers
	// {{{

	IOBUF
	u_cmdbuf(
		.T(cmd_tristate),
		.I(o_cmd),
		.IO(io_cmd),
		.O(i_cmd)
	);

	generate for(gk=0; gk<NUMIO; gk=gk+1)
	begin : GEN_IOBUF
		IOBUF
		u_datbuf(
			.T(dat_tristate[gk]),
			.I(o_dat[gk]),
			.IO(io_dat[gk]),
			.O(i_dat[gk])
		);
	end endgenerate
	// }}}



	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, w_cmd_wide[0], ign_cmd_mine };
	// Verilator lint_on  UNUSED
endmodule

`ifdef	VERILATOR
// Verilator lint_off DECLFILENAME
module	IOBUF(input T, input I, inout IO, output O);
	assign	IO = (T) ? 1'bz : I;
	assign	O = IO;
endmodule
// Verilator lint_on  DECLFILENAME
`endif
