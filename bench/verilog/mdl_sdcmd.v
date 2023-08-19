////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	mdl_sdcmd.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Models how the CMD wire is handled from the standpoint of the
//		SD card, not the controller.  As such, this model is only
//	used in a simulation testbench--not in the design itself.
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
`default_nettype	none
`timescale 1ns/1ps
// }}}
module	mdl_sdcmd #(
		parameter realtime FF_HOLD = 1.2
	) (
		// {{{
		input	wire		sd_clk,
		inout	wire		sd_cmd,
		output	wire		sd_ds,
		//
		output	reg		o_cmd_valid,
		output	reg	[5:0]	o_cmd,
		output	reg	[31:0]	o_arg,
		//
		input	wire		i_valid, i_type,
		output	wire		o_busy,
		input	wire	[5:0]	i_reply,
		input	wire	[119:0]	i_arg,
		input	wire		i_use_crc,
		input	wire		i_drive,
		output	reg		o_collision
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[6:0]	CRC_POLYNOMIAL = 7'h09;

	reg		r_incoming;
	reg	[7:0]	icount;
	reg	[47:0]	ireg;

	reg		r_active, r_outgoing, r_cmd;
	reg	[135:0]	oreg;
	reg	[7:0]	ocount;
	reg		ds;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Receive command from the controller
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	initial	o_cmd_valid = 1'b1;
	initial o_cmd = 6'h0;
	initial o_arg = 32'h0;
	initial	r_incoming = 1'b0;
	always @(posedge sd_clk)
	begin
		o_cmd_valid <= 1'b0;
		if (r_incoming)
		begin
			icount <= icount + 1;
			ireg <= { ireg[46:0], (sd_cmd !== 1'b0) };
			if (icount >= 48)
			begin
				o_cmd_valid <= 1'b1;
				o_cmd <= ireg[45:40];
				o_arg <= ireg[39: 8];
				r_incoming <= 1'b0;
				icount <= 0;
			end
		end else if (sd_cmd === 1'b0 && !r_outgoing)
		begin
			r_incoming <= 1'b1;
			icount <= 1;
			ireg <= { 48'h0 };
		end else begin
			r_incoming <= 1'b0;
			icount <= 0;
			ireg <= { 48'h0 };
		end
	end

	always @(posedge sd_clk)
	if (r_incoming && icount == 48)
	begin
		assert(ireg[47:46] == 2'b01);
		assert(ireg[0]);
		assert(ireg[7:1] == CMDCRC(ireg[47:8]));
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Return a result to the controller
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	initial	o_collision = 0;
	initial	ocount = 0;
	initial	oreg   = 0;
	initial	r_outgoing = 0;
	initial	ds = 0;
	always @(posedge sd_clk)
	if (i_valid && !o_busy)
	begin
		ocount <= (i_type) ? 136 : 48;
		r_outgoing <= 1;
		o_collision <= 1'b0;

		if (i_type)
		begin
			// 136b response
			oreg <= { 2'b00, i_reply, i_arg, REGCRC(i_arg), 1'b1 };
		end else begin
			oreg <= { 2'b00, i_reply, i_arg[31:0],
					CMDCRC({2'b00, i_reply, i_arg[31:0] }),
				 1'b1, {(136-48){1'b1}} };
			if (!i_use_crc)
				oreg[95:89] <= 7'h7f;
		end
	end else begin
		if (ocount > 0)
			ocount <= ocount - 1;
		oreg <= { oreg[134:0], 1'b1 };

		r_outgoing <= (ocount != 0);
		if (!o_collision)
		begin
			o_collision <= r_active && !i_drive
					&& (sd_cmd === 1'b0 && r_cmd);
		end
	end

	always @(negedge sd_clk)
	if (r_outgoing)
		ds <= #FF_HOLD 1'b1;

	always @(posedge sd_clk)
		ds <= #FF_HOLD 1'b0;

	always @(negedge sd_clk)
	if (r_outgoing)
	begin
		r_cmd    <= #FF_HOLD oreg[135];
		r_active <= #FF_HOLD 1'b1;
	end else begin
		r_active <= 1'b0;
		r_cmd    <= 1'b1;
	end

	assign	sd_cmd = (r_active && (i_drive || !r_cmd)) ? r_cmd : 1'bz;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CRC handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	function automatic [6:0] STEPCRC(input [6:0] fill, input i_bit);
		// {{{
	begin
		if (fill[6] ^ i_bit)
			STEPCRC = { fill[5:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			STEPCRC = { fill[5:0], 1'b0 };
	end endfunction
	// }}}

	function automatic [6:0] CMDCRC(input [39:0] cmd);
		// {{{
		reg	[6:0]	fill;
		integer		icrc;
	begin
		fill = 0;

		for(icrc=0; icrc<8+32; icrc=icrc+1)
			fill = STEPCRC(fill, cmd[39-icrc]);

		CMDCRC = fill;
	end endfunction
	// }}}

	function automatic [6:0] REGCRC(input [119:0] cmd);
		// {{{
		reg	[6:0]	fill;
		integer		icrc;
	begin
		fill = 0;

		for(icrc=0; icrc<120; icrc=icrc+1)
			fill = STEPCRC(fill, cmd[119-icrc]);

		REGCRC = fill;
	end endfunction
	// }}}

	// }}}

	assign	o_busy = r_incoming || r_outgoing;
endmodule
