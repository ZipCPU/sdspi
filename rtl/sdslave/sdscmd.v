////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rtl/sdslave/sdscmd.v
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
module	sdscmd #(
		// NUMIO ... unused
		// OPT_DS ... eventually, to support enhanced commands
		// OPT_DDDR ... unused, commands are always SDR
	) (
		// {{{
		// i_clk is the SD clock, from the SDIO master
		// i_reset is ... a power on reset?
		input	wire		i_clk, i_reset,
		input	wire		i_cfg_pp, // Push-pull config
		// Decoded commands from the master
		// {{{
		output	reg		o_valid, o_err,
		output	reg	[5:0]	o_cmd,
		output	reg	[31:0]	o_arg,
		// }}}
		// Responses, to be returned to the master
		// {{{
		input	wire		i_valid,
		input	wire	[5:0]	i_resp,
		input	wire	[31:0]	i_arg,
		// input wire		i_typ,
		// input wire	[95:0]	i_extended,
		// }}}
		// Interface to the front end
		// {{{
		input	wire		i_cmdio,
		output	wire		o_cmdio,
		output	reg		o_cmden
		// }}}
		// }}}
	);

	localparam	[1:0]	CMD_IDLE = 2'b00,
				CMD_RX = 2'b01,
				CMD_TX = 2'b10;

	reg	[1:0]		state;
	reg	[47:0]		sreg;
	reg	[6:0]		cmdcrc;
	reg	[5:0]		count;

	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		state <= CMD_IDLE;
		sreg  <= 48'h0;
		count <= 0;
		cmdcrc <= 0;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		o_cmden <= 1'b0;
		o_cmd   <= 0;
		o_arg   <= 0;
		// }}}
	end else case(state)
	CMD_IDLE: begin
		// {{{
		sreg  <= 48'h0;
		count <= 0;
		cmdcrc <= 0;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		o_cmden <= 1'b0;
		if (!i_cmdio)
		begin
			state <= CMD_RX;
			count <= 1;
		end else if (i_valid)
		begin
			state <= CMD_TX;
			count <= 1;
			o_cmden <= 1'b1;
			sreg[47:0] <= { 2'b00, i_resp, i_arg, 8'hff };
		end end
		// }}}
	CMD_RX: begin
		// {{{
		sreg <= { sreg[46:0], i_cmdio };
		cmdcrc <= STEPCRC(cmdcrc, i_cmdio);
		count <= count + 1;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		if (count == 47)
		begin
			o_valid <= (cmdcrc == 0) &&  i_cmdio && sreg[45];
			o_err   <= (cmdcrc != 0) || !i_cmdio ||!sreg[45];
			o_cmd   <= sreg[44:39];
			o_arg   <= sreg[38: 7];
			sreg <= 48'hffff_ffff_ffff;
			state   <= CMD_IDLE;
		end end
		// }}}
	CMD_TX: begin
		sreg <= { sreg[46:0], 1'b1 };
		o_cmden <= !sreg[46] || i_cfg_pp;
		cmdcrc <= STEPCRC(cmdcrc, sreg[47]);
		count <= count + 1;
		if (count == 39)
		begin
			sreg <= { cmdcrc, 41'h1ff_ffff_ffff };
			o_cmden <= !cmdcrc[6] || i_cfg_pp;
		end if (count >= 48)
		begin
			state <= CMD_IDLE;
			o_cmden <= 1'b0;
		end end
	default: begin
		sreg  <= 48'h0;
		count <= 0;
		cmdcrc <= 0;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		end
	endcase

	assign	o_cmdio = sreg[47];

	localparam [6:0]	CRC_POLY = 7'h09;

	function automatic [6:0] STEPCRC(input[6:0] fill, input i_bit);
		// {{{
	begin
		if (fill[6] ^ i_bit)
			STEPCRC = { fill[5:0], 1'b0 } ^ CRC_POLY;
		else
			STEPCRC = { fill[5:0], 1'b0 };
	end endfunction
	// }}}
endmodule
