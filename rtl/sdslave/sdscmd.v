////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdscmd.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Command wire interface controller.  Listens for 48b commands
//		from the host, decodes them, and checks their CRCs.  Commands
//	are then forwarded to the FSM controller.  Replies, received from the
//	FSM controller, are then sent to the PHY for transmission over the
//	same command wire.  CRCs are inserted.  Collisions are checked.
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
`default_nettype none
// }}}
module	sdscmd #(
		parameter	IODELAY = 2
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
		input	wire		i_typ, i_nocrc,
		input	wire	[95:0]	i_extended,
		output	wire		o_busy,
		// }}}
		// Interface to the front end
		// {{{
		input	wire		i_cmdio,
		output	wire		o_cmdio,
		output	wire		o_cmden,
		input	wire		i_collision
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[1:0]	CMD_IDLE = 2'b00,
				CMD_RX = 2'b01,
				CMD_TX = 2'b10;

	reg			long_message, r_cmden;
	reg	[1:0]		state;
	reg	[136:0]		sreg;
	reg	[6:0]		cmdcrc;
	reg	[7:0]		count;
	wire	[6:0]		stepped;
	// }}}

	assign	stepped = STEPCRC(cmdcrc, sreg[135]);

	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		state <= CMD_IDLE;
		sreg  <= {(137){1'b1}};
		count <= 0;
		cmdcrc <= 0;
		long_message <= 1;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		r_cmden <= 1'b0;
		o_cmd   <= 0;
		o_arg   <= 0;
		// }}}
	end else case(state)
	CMD_IDLE: begin
		// {{{
		sreg  <= {(137){1'b1}};
		count <= 0;
		cmdcrc <= 0;
		long_message <= 1'b0;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		r_cmden <= 1'b0;
		if (!i_cmdio)
		begin
			state <= CMD_RX;
			count <= 1;
		end else if (i_valid)
		begin
			state <= CMD_TX;
			count <= 1;
			r_cmden <= 1'b1;
			long_message <= i_typ;
			sreg[136:96] <= { 1'b0, 2'b00, i_resp, i_arg };
			if (i_typ)
				sreg[95:0] <= { i_extended };
		end end
		// }}}
	CMD_RX: begin
		// {{{
		// sreg <= { sreg[142:96], i_cmdio, {(96){1'b1}} };
		sreg <= { sreg[135:88], i_cmdio, {(88){1'b1}} };
		cmdcrc <= STEPCRC(cmdcrc, i_cmdio);
		count <= count + 1;
		long_message <= 1'b0;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		if (count == 47)
		begin
			o_valid <= (cmdcrc == 0) &&  i_cmdio && sreg[133];
			o_err   <= (cmdcrc != 0) || !i_cmdio ||!sreg[133];
			o_cmd   <= sreg[132:127];
			o_arg   <= sreg[126: 95];
			sreg <= {(137){1'b1}};
			state   <= CMD_IDLE;
		end end
		// }}}
	CMD_TX: begin
		// {{{
		sreg <= { sreg[135:0], 1'b1 };
		r_cmden <= !sreg[134] || i_cfg_pp;
		if (i_nocrc)
			cmdcrc <= -1;
		else if (!long_message || count > 8)
			cmdcrc <= stepped;
		count <= count + 1;
		if ((!long_message && count == 40) || (count == 136-8))
		begin
			sreg[135:136-7] <= stepped;
			r_cmden <= !stepped[6] || i_cfg_pp;
		end if ((!long_message && count >= 48+IODELAY)
				|| (count >= 136+IODELAY)
				|| i_collision)
		begin
			// We need to wait an extra IODELAY clocks here, so
			// that upon returning to idle we don't "detect" our
			// last bit as a potential stop bit.
			state <= CMD_IDLE;
			r_cmden <= 1'b0;
		end end
		// }}}
	default: begin
		// {{{
		sreg  <= {(137){1'b1}};
		count <= 0;
		cmdcrc <= 0;
		long_message <= 1'b0;
		o_valid <= 1'b0;
		o_err   <= 1'b0;
		end
		// }}}
	endcase

	assign	o_cmdio = sreg[135];
	assign	o_cmden = r_cmden && !i_collision;
	assign	o_busy = (state != CMD_IDLE);

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
