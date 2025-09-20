////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rtl/sdslave/sdsfsm.v
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
module	sdsfsm #(
		parameter [0:0]	OPT_HIGH_CAPACITY = 1'b1
	) (
		// {{{
		input	wire		i_clk, i_reset,
		//
		input	wire		i_cmd_valid, i_cmd_err,
		input	wire	[5:0]	i_cmd,
		input	wire	[31:0]	i_arg,
		//
		output	wire		o_rsp_valid,
		output	wire	[5:0]	o_resp,
		output	wire	[31:0]	o_resp_data,
		// output wire		o_resp_typ,
		// output wire	[95:0]	o_resp_extra,
		//
		output	wire		o_rx_en,
					// o_rx_abort (?)
		output	wire [AW-1:0]	o_rx_addr,
		input	wire		i_rx_done,
		input	wire		i_rx_dma_done,
		input	wire		i_rx_dma_err,
	//
		output	wire		o_tx_en,
					// o_tx_abort (?)
		output	wire [AW-1:0]	o_tx_addr,
		input	wire		o_tx_done,
		input	wire		i_tx_dma_done,
		input	wire		i_tx_dma_err,
	//
		output	wire		o_busy,
	//
		output	wire		o_cfg_pp,
		output	wire		o_cfg_ds,
		output	wire		o_cfg_ddr,
		output	wire	[3:0]	o_cfg_lgblksz,
		output	wire	[1:0]	o_cfg_width
		// }}}
	);

	assign	o_cfg_ds = 1'b0;
	assign	o_cfg_width = { 1'b0, r_width };
	assign	ocr = 32'h00ff_8000 | { !power_up_busy,
			cmd8_sent && hcs_support && OPT_HIGH_CAPACITY,
				30'h0 };

	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		cmd_aux <= 1'b0;
		r_width <= 1'b0;
		o_rsp_valid <= 1'b0;
		o_cfg_pp <= 1'b0;
		o_cfg_ddr <= 1'b0;
		o_cfg_lgblksz <= 4'h9;
		r_width <= 1'b0;
		card_selected <= 1'b0;
		multiblock <= 1'b0;
		// }}}
	end else begin
		o_rsp_valid <= 1'b0;

		if (i_cmd_valid)
		begin
			o_resp <= i_cmd;
			o_resp_data <= i_arg;
			//
			case(cmd_aux, i_cmd[5:0])
			// ACMD6
			{ 1'b1, 6'd41 }: begin // ACMD41
					// LOTS OF WORK TO DO HERE!
					// TODO
					// FIXME!
					// Need to do proper voltage capability
					// declaration and high capacity
					// andling here.
				end
			{ 1'b?, 6'd0 }: begin	// CMD0: Go Idle
				// {{{
				o_rsp_valid <= 1'b1;
				o_cfg_pp <= 1'b0;
				o_cfg_ds <= 1'b0;
				o_cfg_ddr <= 1'b0;
				o_cfg_lgblksz <= 4'h9;
				r_width <= 1'b0;
				card_selected <= 1'b0;
				end
				// }}}
			// CMD2: ALL_SEND_CID
			// CMD3: SEND_RELATIVE_ADDR
			// CMD6: SWITCH_FUNCTION (+/- DDR, etc)
			{ 1'b0, 6'd07 }: begin // CMD7: SELECT_DESELECT_CARD
				// {{{
				card_selected <= (i_arg[31:16] == RCA);
				o_rsp_valid   <= (i_arg[31:16] == RCA);
				o_resp_arg    <= R1;
				end
				// }}}
			{ 1'b0, 6'd08 }: begin // CMD8: SEND_IF_COND
				// {{{
				// card_selected <= 1'b0;
				o_rsp_valid <= card_selected;
				if (!cmd8_sent)
				begin
					hcs_support <= 1'b0;
					if (OPT_HIGH_CAPACITY)
						hcs_support <= i_arg[30];
					if (OPT_DUAL_VOLTAGE)
						dual_voltage_hosto <= i_arg[30];
					cmd8_sent <= 1'b1;
				end end
				// }}}
			// CMD10: SEND_CID
			// CMD11: VOLTAGE_SWITCH
			{ 1'b0, 6'd12 }: begin // CMD12: STOP_TRANSMISSION
					// {{{
					o_rx_en <= 1'b0;
					o_tx_en <= 1'b0;
					multiblock <= 1'b0;
				end
				// }}}
			{ 1'b0, 6'd17 }: begin // CMD17: Read block
					// {{{
					// o_tx_en <= 1'b1	// Must be in another FSM
					multiblock <= 1'b0;
					dma_addr <= i_arg[AW-1:0];
					o_cfg_lgblksz <= 4'h9;
					if (hcs_support)
						dma_addr <= { i_arg[AW-9:0], 8'h0 };
				end
				// }}}
			// CMD18: Read multiple
			// CMD19: Send tuning block
			{ 1'b0, 6'd24 }: begin // CMD24: Write block
					// {{{
					o_rx_en <= 1'b1;
					multiblock <= 1'b0;
					dma_addr <= i_arg[AW-1:0];
					o_cfg_lgblksz <= 4'h9;
					if (hcs_support)
						dma_addr <= { i_arg[AW-9:0], 8'h0 };
				end
				// }}}
			// CMD25: Write multiple
			{ 1'b?, 6'd55 }: begin	// CMD55: APP_CMD, ACMD to follow
				// {{{
				o_rsp_valid <= card_selected;
				o_resp_data <= 32'h0120;
				cmd_aux <= 1'b1;
				end
				// }}}
			default: begin end
			endcase
		end // if (i_cmd_valid)
	end

	assign	o_cfg_ds = 1'b0;

endmodule
