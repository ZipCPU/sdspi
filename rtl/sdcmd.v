////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdcmd.v
// {{{
// Project:	SDIO SD-Card controller, using a shared SPI interface
//
// Purpose:	Bi-directional command line processor.  This generates the
//		command line inputs to the PHY, and receives its outputs.
//	Commands are requested from the CPU, and responses gathered and
//	returned to the register set.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2023, Gisselquist Technology, LLC
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
module	sdcmd #(
		// {{{
		// parameter	MW = 32,
		// parameter [0:0]	OPT_LITTLE_ENDIAN = 1'b0,
		parameter	LGTIMEOUT = 22,
		parameter	LGLEN = 9,
		parameter	MW = 32
		// }}}
	) (
		// {{{
		input	wire			i_clk, i_reset,
		// Configuration bits
		input	wire			i_cfg_ds,	// Use ASYNC
		input	wire			i_cfg_dbl,	// 2Bits/Clk
		input	wire			i_ckstb,
		// Controller interface
		// {{{
		input	wire			i_cmd_request,
		input	wire	[1:0]		i_cmd_type,
		input	wire	[5:0]		i_cmd,
		input	wire	[31:0]		i_arg,

		output	wire			o_busy,
		output	reg			o_done,
		output	reg			o_err,
		output	reg	[1:0]		o_ercode,
		// }}}
		// Send to the front end
		// {{{
		output	wire			o_cmd_en,
		// output	wire		o_pp_cmd,	// From CFG reg
		output	wire	[1:0]		o_cmd_data,
		// }}}
		// Receive from the front end
		// {{{
		input	wire	[1:0]		i_cmd_strb,
		input	wire	[1:0]		i_cmd_data,
		input	wire			i_dat_busy,

		input	wire			S_ASYNC_VALID,
		input	wire	[1:0]		S_ASYNC_DATA,
		// }}}
		// Return the result
		output	reg			o_cmd_response,
		output	reg	[5:0]		o_resp,
		output	reg	[31:0]		o_arg,
		// Writes to memory
		// {{{
		output	reg			o_mem_valid,
		output	wire	[MW/8-1:0]	o_mem_strb,
		output	wire	[LGLEN-1:0]	o_mem_addr,	// Word address
		output	reg	[MW-1:0]	o_mem_data	// Outgoing data
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam [1:0]	R_NONE = 2'b00,
				R_R1   = 2'b01,
				R_R2   = 2'b10,
				R_R1b  = 2'b11;

	localparam [1:0]	ECODE_TIMEOUT = 2'b00,
				ECODE_OKAY    = 2'b01,
				ECODE_BADCRC  = 2'b10,
				ECODE_FRAMEERR= 2'b11;

	localparam	[6:0]	CRC_POLYNOMIAL = 7'h09;

	reg		active;
	reg	[5:0]	srcount;
	reg	[47:0]	tx_sreg;

	reg		waiting_on_response, cfg_ds, cfg_dbl, r_frame_err;
	reg	[1:0]	cmd_type;
	reg	[7:0]	resp_count;
	wire		frame_err, w_done, crc_err;
	reg	[LGLEN+$clog2(MW/32)-1:0]	mem_addr;
	reg	[39:0]	rx_sreg;

	reg			rx_timeout;
	reg	[LGTIMEOUT-1:0]	rx_timeout_counter;

	reg	[6:0]	crc_fill;
	reg		r_busy;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Send 48b command to card
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	if (i_reset)
	begin
		active <= 0;
		srcount <= 0;
	end else if (i_cmd_request && !o_busy)
	begin
		srcount <= 48;
		active  <= 1;
		// sreg <= { i_cmd, i_arg, CMDCRC({ i_cmd, i_arg }), 1'b1 };
	end else if (i_ckstb)
	begin
		if (cfg_dbl)
		begin
			// sreg <= { sreg[45:0], 2'b11 };
			active <= (srcount > 2);
			srcount <= srcount - 2;
		end else begin
			// sreg <= { sreg[46:0], 2'b1 };
			active <= (srcount > 1);
			srcount <= srcount - 1;
		end
	end
	
	always @(posedge i_clk)
	if (i_cmd_request && !o_busy)
		tx_sreg <= { 2'b01, i_cmd, i_arg,
				CMDCRC({ 2'b01, i_cmd, i_arg }), 1'b1 };
	else if (i_ckstb)
	begin
		if (cfg_dbl)
			tx_sreg <= { tx_sreg[45:0], 2'b11 };
		else
			tx_sreg <= { tx_sreg[46:0], 1'b1 };
	end

	assign	o_cmd_en = active;
	assign	o_cmd_data = tx_sreg[47:46];

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Receive response from card
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// waiting_on_response
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		waiting_on_response <= 1'b0;
	else if (i_cmd_request && !o_busy)
		waiting_on_response <= (i_cmd_type != R_NONE);
	else if (waiting_on_response)
	begin
		if (cmd_type[0] && resp_count >= 48)
			waiting_on_response <= 1'b0;
		if (resp_count >= 136)
			waiting_on_response <= 1'b0;
		if (rx_timeout)
			waiting_on_response <= 1'b0;
	end
	// }}}

	// cfg_ds, cfg_dbl, cmd_type
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		{ cfg_ds, cfg_dbl, cmd_type } <= 4'b0;
	else if (i_cmd_request && !o_busy)
		{ cfg_ds, cfg_dbl, cmd_type } <= { i_cfg_ds, i_cfg_dbl, i_cmd_type };
	// }}}

	// resp_count
	// {{{
	always @(posedge i_clk)
	if (i_reset || !waiting_on_response || active)
		resp_count <= 0;
	else if (cfg_ds)
	begin
		if (S_ASYNC_VALID)
			resp_count <= resp_count + 2;
	end else
		resp_count <= resp_count + (i_cmd_strb[1] ? 1:0)
				+ (i_cmd_strb[0] ? 1:0);
	// }}}

	// rx_sreg
	// {{{
	always @(posedge i_clk)
	if (i_reset || !waiting_on_response || active)
		rx_sreg <= 0;
	else if (cfg_ds)
	begin
		if (S_ASYNC_VALID)
			rx_sreg <= { rx_sreg[37:0], S_ASYNC_DATA[1:0] };
	end else if (i_cmd_strb[1])
	begin
		if (i_cmd_strb[0])
			rx_sreg <= { rx_sreg[37:0], i_cmd_data[1:0] };
		else
			rx_sreg <= { rx_sreg[38:0], i_cmd_data[1] };
	end
	// }}}

	assign	w_done = (waiting_on_response)
			&&((cmd_type == R_R2 && resp_count == 136)
			|| (cmd_type[1] && resp_count == 48));

	// o_cmd_response
	// {{{
	always @(posedge i_clk)
	if (i_reset || !waiting_on_response || cmd_type == R_NONE)
		o_cmd_response <= 1'b0;
	else if (cmd_type[1])
		o_cmd_response <= (resp_count == 48);
	else // if (cmd_type == R_R2)
		o_cmd_response <= (resp_count == 136);
	// }}}

	// o_resp, o_arg
	// {{{
	always @(posedge i_clk)
	if (i_reset || !waiting_on_response)
		o_resp <= 6'b0;
	else if (resp_count == 8)
		o_resp <= rx_sreg[5:0];

	always @(posedge i_clk)
	if (i_reset || !waiting_on_response)
		o_arg <= 32'b0;
	else if (cmd_type == R_R2)
	begin
		o_arg  <= 32'h0;
	end else
		o_arg <= rx_sreg[8 +: 32];
	// }}}

	//////////
	//
	// Writes to memory
	//

	// o_mem_valid
	// {{{
	always @(posedge i_clk)
	if (i_reset || cmd_type != R_R2 || !waiting_on_response)
		o_mem_valid <= 1'b0;
	else
		o_mem_valid <= (resp_count[4:0] == 8 && resp_count[7:5] != 0);
	// }}}

	// o_mem_strb
	// {{{
	generate if (MW==32)
	begin
		assign	o_mem_strb = 4'hf;
	end else begin
		reg	[MW/8-1:0]	r_mem_strb;

		initial	r_mem_strb = 0;
		always @(posedge i_clk)
		if (i_reset || cmd_type != R_R2 || !waiting_on_response)
			r_mem_strb <= { 4'hf, {(MW/8-1){1'b0}} };
		else if (o_mem_valid)
			r_mem_strb<= { r_mem_strb[3:0], r_mem_strb[MW/32-1:4] };

		assign	o_mem_strb = r_mem_strb;
	end endgenerate
	// }}}

	// o_mem_addr
	// {{{
	always @(posedge i_clk)
	if (i_reset || cmd_type != R_R2 || !waiting_on_response)
		mem_addr <= 0;
	else if (o_mem_valid)
		mem_addr <= mem_addr + 1;

	assign	o_mem_addr = mem_addr[LGLEN-1:$clog2(MW/32)];
	// }}}

	// o_mem_data
	// {{{
	always @(posedge i_clk)
	if (resp_count[4:0] == 8 && resp_count[7:5] != 0)
		o_mem_data <= {(MW/32){rx_sreg[31:0]}};
	// }}}

	// Frame error detection
	// {{{
	always @(posedge i_clk)
	if (i_reset || !waiting_on_response)
		r_frame_err <= 1'b0;
	else if (i_cmd_request && !o_busy)
		r_frame_err <= 1'b0;
	else if (resp_count == 2 && rx_sreg[1:0] != 2'b00)
		r_frame_err <= 1'b1;
	
	assign	frame_err = r_frame_err || (waiting_on_response
			&&((cmd_type[1] && !rx_sreg[0] && resp_count == 48)
			||((cmd_type==R_R2&& !rx_sreg[0] && resp_count == 136))));
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// RX Timeout handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	if (i_reset || !waiting_on_response || active)
	begin
		rx_timeout <= 0;
		rx_timeout_counter <= -1;
	end else if ((cmd_type == R_R1b && i_dat_busy)
			|| (i_cfg_ds && S_ASYNC_VALID)
			|| (!i_cfg_ds && i_cmd_strb != 0))
	begin
		rx_timeout <= 0;
		rx_timeout_counter <= -1;
	end else if (i_ckstb)
	begin
		if (rx_timeout_counter != 0)
			rx_timeout_counter <= rx_timeout_counter - 1;
		if (rx_timeout_counter <= 1)
			rx_timeout <= 1;
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CRC handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	if (i_reset || !waiting_on_response)
		crc_fill <= 0;
	else if ((cfg_ds && S_ASYNC_VALID)
			|| (!cfg_ds && i_cmd_strb == 2'b11))
	begin
		if (resp_count >= 6)
			crc_fill <= STEPCRC(STEPCRC(crc_fill,
					rx_sreg[7]), rx_sreg[6]);
	end else if (!i_cfg_ds && i_cmd_strb[1] && resp_count >= 7)
		crc_fill <= STEPCRC(crc_fill, rx_sreg[7]);

	assign	crc_err = w_done && (crc_fill != resp_count[7:1]);


	function automatic [6:0] STEPCRC(input [6:0] fill,
							input i_data);
	begin
		if (fill[6] ^ i_data)
			STEPCRC = { fill[5:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			STEPCRC = { fill[5:0], 1'b0 };
	end endfunction

	function automatic [6:0] CMDCRC(input [39:0] cmd);
		reg	[6:0]	fill;
		integer		icrc;
	begin
		fill = 0;

		for(icrc=0; icrc<8+32; icrc=icrc+1)
			fill = STEPCRC(fill, cmd[39-icrc]);

		CMDCRC = fill;
	end endfunction

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// ERR handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	if (i_reset || active || (i_cmd_request && !o_busy))
	begin
		o_err <= 1'b0;
		o_ercode <= 2'b00;
	end else if (rx_timeout)
	begin
		o_err <= 1'b1;
		o_ercode <= ECODE_TIMEOUT;
	end else if (w_done)
	begin
		{ o_err, o_ercode } <= { 1'b0, ECODE_OKAY };
		if (frame_err)
			{ o_err, o_ercode } <= { 1'b1, ECODE_FRAMEERR };
		if (crc_err)
			{ o_err, o_ercode } <= { 1'b1, ECODE_BADCRC };
	end
	// }}}

	always @(posedge i_clk)
		o_done <= (rx_timeout || w_done) && !i_reset;

	always @(posedge i_clk)
	if (i_reset)
		r_busy <= 1'b0;
	else if (i_cmd_request && !o_busy)
		r_busy <= 1'b1;
	else if (!active && !waiting_on_response
			&& (cmd_type != R_R1b || !i_dat_busy))
		r_busy <= 1'b0;

	assign	o_busy = r_busy || !i_ckstb;

	//
	// Make verilator happy
	// {{{
	// verilator coverage_off
	// verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, R_R1 };
	// verilator lint_on  UNUSED
	// verilator coverage_on
	// }}}
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
`endif	// FORMAL
// }}}
endmodule

