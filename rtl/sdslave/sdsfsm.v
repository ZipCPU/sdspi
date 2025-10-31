////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdsfsm.v
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
module	sdsfsm #(
		// {{{
		parameter	ADDRESS_WIDTH = 32,
		parameter [0:0]	OPT_HIGH_CAPACITY = 1'b1,
		parameter [0:0]	OPT_1P8V = 1'b0,
		parameter [0:0]	OPT_UHSII = 1'b0,
		parameter [119:0]	CID = { 64'hdadd_3519_2347_291a,
						56'habca_dead_51da_d1 },
		parameter [15:0]	OCR_VOLTAGE = 16'hff_80
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Configuration / PHY control
		// {{{
		output	reg		o_cfg_cmd_pp,
		output	reg		o_cfg_dat_pp,
		output	wire		o_cfg_ds,
		output	reg		o_cfg_ddr,
		output	reg	[3:0]	o_cfg_lgblksz,
		output	wire	[1:0]	o_cfg_width,
		// }}}
		// Command / response
		// {{{
		input	wire		i_cmd_valid, i_cmd_err,
		input	wire	[5:0]	i_cmd,
		input	wire	[31:0]	i_arg,
		//
		output	reg		o_resp_valid,
		output	reg	[5:0]	o_resp,
		output	reg	[31:0]	o_resp_data,
		output	reg		o_resp_typ, o_resp_nocrc,
		output	reg	[95:0]	o_resp_extra,
		//
		input	wire		i_collision,
		input	wire		i_cmd_busy,
		// }}}
		// RX (host -> slave) control
		// {{{
		output	reg		o_rx_en,
					// o_rx_abort (?)
		// output	wire [AW-1:0]	o_rx_addr,
		input	wire		i_rx_good, i_rx_err,
		// }}}
		// TX (slave -> host) control
		// {{{
		output	reg		o_tx_en,
		output	wire		o_tx_busy,
		output	reg	[1:0]	o_tx_src,
		// input wire		i_tx_busy,
		input	wire		i_tx_done,
		// }}}
		// DMA (to/from memory) control
		// {{{
		// Caution: DMA requests *must* cross clock boundaries, from
		// SD clock to AXI clock.
		//
		// Rule: Can request again while the DMA is busy, but *only*
		// if the address is continuous.  Once
		//	o_dma_request && i_cfg_ready are both true, the request
		// has been accepted and o_dma_request may be dropped.
		// Likewise, once (o_dma_abort && i_cfg_ready) are both true,
		// the abort request may be dropped.  DMA busy is a return
		// (potentially from the other clock domain), and its useful
		// for knowing when actions are ongoing.
		//
		output reg		o_cfg_valid,
		input	wire		i_cfg_ready,
		output reg		o_dma_request,
		output reg		o_dma_dir,	// Direction
		output	reg		o_dma_abort, o_dma_reset,
		output reg [ADDRESS_WIDTH-1:0]	o_dma_addr,	// Byte-wise address
		//
		input	wire		i_s2mm_done, i_s2mm_err,
		input	wire		i_mm2s_done, i_mm2s_err
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam		AW = ADDRESS_WIDTH;
	// Possible transmit data sources
	localparam	[1:0]	S_MEM = 2'b00,
				S_SCR = 2'b01,
				S_STATUS = 2'b10,
				S_TUNING = 2'b11;

	// Verilator lint_off UNUSED

	localparam [0:0]	D_DEV2HOST = 1'b0,
				D_HOST2DEV = 1'b1;

	// Card status bit definitions
	localparam	[3:0]	ST_IDLE  = 4'h0,
				ST_READY = 4'h1,
				ST_IDENT = 4'h2,
				ST_STBY  = 4'h3,
				ST_TRAN  = 4'h4,
				ST_DATA  = 4'h5,
				ST_RCV   = 4'h6,
				ST_PRG   = 4'h7,
				ST_DIS   = 4'h8;
				// All other states are reserved
				// ST_IOMODE = 15

	localparam [31:0]	OUT_OF_RANGE     = 32'h8000_0000,
				ADDRESS_ERROR    = 32'h4000_0000,
				BLOCK_LEN_ERROR  = 32'h2000_0000,
				ERASE_SEQ_ERROR  = 32'h1000_0000,
				ERASE_PARAM      = 32'h0800_0000,
				WP_VIOLATION     = 32'h0400_0000,
				CARD_IS_LOCKED   = 32'h0200_0000,
				LOCK_UNLOCK_FAILED = 32'h0100_0000,
				COM_CRC_ERROR    = 32'h0080_0000,
				ILLEGAL_COMMAND  = 32'h0040_0000,
				CARD_ECC_FAILED  = 32'h0020_0000,
				CC_ERROR         = 32'h0010_0000,
				ERROR            = 32'h0008_0000,
				// Reserved
				// Reserved
				CSD_OVERWRITE    = 32'h0001_0000,
				WP_ERASE_SKIP    = 32'h0000_8000,
				CARD_ECC_DISABLED= 32'h0000_4000,
				ERASE_RESET      = 32'h0000_2000,
				// State, bits 12:9
				READY_FOR_DATA   = 32'h0000_0100,
				// Reserved
				FX_DATA          = 32'h0000_0040,
				APP_CMD          = 32'h0000_0020,
				// Reserved
				AKE_SEQ_ERROR    = 32'h0000_0008;
				// Last 3 bits are also reserved
				// Verilator lint_on UNUSED

	reg	[3:0]	r_state;
	reg		r_multiblock, app_cmd, r_width, hcs_support, r_inactive,
			first_command, w_valid_blksz, my_cid,
			s2mm_busy, mm2s_busy;
	wire	[31:0]	R1;
	reg	[15:0]	RCA;
	wire	[15:0]	next_rca;
	reg	[1:0]	bufcount;
	reg	[3:0]	w_new_blksz;
	wire		transfer_complete, operation_complete;

	wire	[127:0]	CSD;

	reg		new_dma_request, new_tx_en, new_rx_en;
	reg	[1:0]	new_bufcount;
	wire	[119:0]	w_CID;
	reg		r_reply_active;
	// }}}

	assign	R1 = { 19'h0, r_state, 3'h0, app_cmd, 5'h0 };
	// assign	R1 = 32'h0; // | sd_state | BUS_ERR | CRC_ERR | CMD_ERR | APPCMD
	assign	CSD = 128'h0;
	assign	w_CID = CID;

	assign	o_cfg_ds = 1'b0;
	assign	o_cfg_width = { 1'b0, r_width };

	// Setup commands:
	//	CMD0		Go IDLE
	//	CMD8		SEND_IF_COND
	//	CMD55		SEND_APP_CMD
	//	ACMD41		SEND_OP_COND
	//	CMD11		SEND_VOLTAGE_SWITCH
	//	CMD2	128b	ALL_SEND_CID
	//	CMD3		SEND_RCA
	//	CMD7		SELECT_CARD	(or unselect)
	//	CMD6		SWITCH_CMD	-- Switch clocks / modes
	//	CMD9		READ_CSD
	//	CMD10		SEND_CID
	//	CMD19		SEND_TUNING_BLOCK
	//	ACMD51		READ_SCR
	//	ACMD6		SET_BUS_WIDTH
	//
	//
	//

	assign	next_rca = (&RCA) ? 16'h1 : (RCA + 16'h1);
	assign	transfer_complete = (o_tx_en && i_tx_done && !r_multiblock)
			||(o_rx_en && (i_rx_good || i_rx_err) && !r_multiblock);
	assign	operation_complete =
		((r_state == ST_DIS || r_state == ST_PRG) && !o_dma_request && bufcount == ((i_s2mm_done || i_s2mm_err)? 1:0))
		|| !r_multiblock && ((o_tx_en && i_tx_done)
				||(s2mm_busy && !o_rx_en && !bufcount[1]
						&& (i_s2mm_done || i_s2mm_err))
				|| (o_dma_reset && !o_tx_en));

	// w_valid_blksz, w_new_blksz
	// {{{
	always @(*)
	begin
		w_valid_blksz = 1'b0;
		w_new_blksz = o_cfg_lgblksz;
		casez(i_arg)
		32'b0000_0000_0000_0000_0000_0000_0000_0100: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd2;
			end
		32'b0000_0000_0000_0000_0000_0000_0000_1000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd3;
			end
		32'b0000_0000_0000_0000_0000_0000_0001_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd4;
			end
		32'b0000_0000_0000_0000_0000_0000_0010_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd5;
			end
		32'b0000_0000_0000_0000_0000_0000_0100_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd6;
			end
		32'b0000_0000_0000_0000_0000_0000_1000_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd7;
			end
		32'b0000_0000_0000_0000_0000_0001_0000_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd8;
			end
		32'b0000_0000_0000_0000_0000_0010_0000_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd9;
			end
		32'b0000_0000_0000_0000_0000_0100_0000_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd10;
			end
		32'b0000_0000_0000_0000_0000_1000_0000_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd11;
			end
		32'b0000_0000_0000_0000_0001_0000_0000_0000: begin
			w_valid_blksz = 1;
			w_new_blksz   = 4'd12;
			end
		default: begin end
		endcase
	end
	// }}}

	// Notes: DMA Controls
	// {{{
	// -- READ/TX (Read from memory, write to SDIO)
	//	SOURCES: CSD, or memory
	//	For MEMORY ...
	//	1. dma_request, dma_write <= 2'b10
	//	2. i_cfg_ready, dma_request <= 0
	//	3. i_dma_busy
	//	4. !i_dma_busy
	//	5. loaded <= loaded + 1, o_tx_en = card_selected
	//	6. if (multiblock && dma_loaded < 2) dma_request <= !DMA_ERR;
	//	7. If TX done, dma_loaded <= dma_loaded - 1
	//	8. if (CMD12) dma_abort <= 1; multiblock=0; o_tx_en <= 0
	//		o_tx_abort <= 1'b0
	//	What if DMA_ERR?	DMA_ERR = 1
	//		if DMA_ERR, dma_loaded <= 0
	//
	// -- RX/WRITE (Read from SDIO, write to memory)
	//	1. rx_en, dma_write <= 1
	//	2. rx_done => rx_en <= 0,
	//	3. if (!rx_err)
	//		loaded <= loaded + 1
	//	4. dma_request <= 1; dma_write <= 1
	//	5. if (dma_ready) dma_request <= 0
	//	6. if (multiblock && loaded < 2) rx_en <= 1;
	//	7. busy = ((!multi && loaded != 0) || (loaded > 1))
	//		&& card_selected && dma_active && dma_write
	//	What if DMA_ERR?
	//	What if BAD CRC?
	//
	//	o_cfg_pp <= (card_selected || tx_en) && cfg_pp
	// }}}

	initial	r_state = ST_IDLE;
	initial	r_reply_active = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		r_state <= ST_IDLE;
		app_cmd <= 1'b0;
		r_width <= 1'b0;
		o_resp_valid <= 1'b0;
		o_resp_nocrc <= 1'b0;	// (Most) everything gets a CRC
		o_cfg_cmd_pp <= 1'b0;
		o_cfg_dat_pp <= 1'b0;
		o_cfg_ddr <= 1'b0;
		o_cfg_lgblksz <= 4'h9;
		r_width <= 1'b0;
		// r_multiblock <= 1'b0;
		RCA <= 16'hdad0;
		// rca_assigned <= 1'b0;
		my_cid <= 1'b0;
		r_inactive <= 1'b0;
		first_command <= 1'b1;
		r_reply_active <= 1'b0;
		// }}}
	end else begin
		o_resp_valid <= 1'b0;
		o_resp_typ   <= 1'b0;	// 32b/48b response
		o_resp_nocrc <= 1'b0;	// (Most) everything gets a CRC
		if (i_collision)
			my_cid <= 1'b0;
		o_resp_extra <= { CID[87:0], 8'hff };

		if (!o_dma_request && r_state == ST_PRG && bufcount == 0)
			r_state <= ST_TRAN;
		if (!o_dma_request && r_state == ST_DIS && bufcount == 0)
			r_state <= ST_STBY;

		if (transfer_complete && r_state == ST_RCV && !r_multiblock)
			r_state <= ST_PRG;
		if (operation_complete && (r_state == ST_PRG
							|| r_state == ST_DATA))
			r_state <= ST_TRAN;
		if (operation_complete && r_state == ST_DIS)
			r_state <= ST_STBY;

		if (i_cmd_valid && !r_inactive)
			first_command <= 1'b0;

		if (!i_cmd_busy && !o_resp_valid)
			r_reply_active <= 1'b0;

		if (r_inactive)
		begin
		end else if (i_cmd_err)
		begin
			if (r_state != ST_READY && r_state != ST_IDENT
				|| r_state != ST_STBY && r_state != ST_IDLE)
			begin
				o_resp_valid <= 1'b1;
				o_resp <= i_cmd;
				o_resp_data  <= R1 | COM_CRC_ERROR;
			end
		end else if (i_cmd_valid)
		begin
			o_resp <= i_cmd;
			o_resp_data <= i_arg;
			// o_resp_typ <= 1'b0;
			app_cmd <= 1'b0;
			//
			casez({ app_cmd, i_cmd[5:0] })
			// ACMD6
			{ 1'b1, 6'd41 }: begin // ACMD41
				// {{{
				if (r_state == ST_IDLE)
				// if (r_state == ST_READY)
				begin
					if (0 != i_arg[23:0] && !i_arg[30])
						hcs_support <= 1'b0;
					if (0 != i_arg[23:0])
						r_state <= ST_READY;
					o_resp_valid <= 1'b1;
					o_resp_data <= {
						// 1'b0, 1'b0, 6'h3f,
						1'b1, // Initialization complete
						hcs_support && i_arg[30],
						OPT_UHSII,
						4'h0,
						OPT_1P8V && i_arg[24],
						// OCR
						OCR_VOLTAGE & i_arg[23:8],
						8'h00
						};
					o_resp_nocrc <= 1'b1;
				end end
				// }}}
			{ 1'b1, 6'd6 }: begin // ACMD6: SET_BUS_WIDTH
				// {{{
				o_resp_data   <= R1;
				if (r_state == ST_TRAN)
				begin
					o_resp_valid <= 1'b1;
					case(i_arg[1:0])
					2'b00: r_width <= 1'b0;
					2'b10: r_width <= 1'b1;
					default: o_resp_data <= R1
						| ILLEGAL_COMMAND;
					endcase

					if (i_arg[31:2] != 30'h0)
						o_resp_data <= R1
							| ILLEGAL_COMMAND;
				end end
				// }}}
			{ 1'b1, 6'd13 }: begin // ACMD13: SD_STATUS
				// {{{
				if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					// o_tx_en <= 1'b1	// Must be in another FSM
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					o_resp_data[12:9] <= ST_DATA;
					// The SD Status result is 64 bytes
					o_cfg_lgblksz <= 4'h6;
					r_reply_active <= 1'b1;
				end end
				// }}}
			// { 1'b1, 6'd22 }: begin // ACMD22: SEND_NUM_WR_BLOCKS
			//	r_state <= ST_DATA
			// { 1'b1, 6'd23 }: begin // ACMD23: SET_WR_BLK_ERASE_COUNT
			//	r_state <= ST_TRAN
			// { 1'b1, 6'd42 }: begin // ACMD42: SET_CLR_CARD_DETECT
			//		r_state <= ST_TRAN
			{ 1'b1, 6'd51 }: begin // ACMD51: SEND_SCR
				// {{{
				if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					// o_tx_en <= 1'b1	// Must be in another FSM
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					o_resp_data[12:9] <= ST_DATA;
					// The SD Config Register is 8bytes
					o_cfg_lgblksz <= 4'h3;
					r_reply_active <= 1'b1;
				end end
				// }}}
			{ 1'b?, 6'd0 }: begin	// CMD0: GO_IDLE
				// {{{
				r_state <= ST_IDLE;
				// No response to the GO_IDLE command
				o_resp_valid <= 1'b0;
				o_cfg_cmd_pp <= 1'b0;
				o_cfg_dat_pp <= 1'b0;
				// o_cfg_ds <= 1'b0;
				o_cfg_ddr <= 1'b0;
				o_cfg_lgblksz <= 4'h9;
				r_width <= 1'b0;
				// rca_assigned <= 1'b0;
				my_cid <= 1'b0;
				first_command <= 1'b1;
				end
				// }}}
			{ 1'b0, 6'd02 }: begin // CMD2: ALL_SEND_CID
				// {{{
				my_cid <= 1'b0; // Avoid collisions
				if (r_state == ST_READY)
				begin
					o_resp_valid <= 1'b1;
					r_state <= ST_IDENT;
					{ o_resp_data, o_resp_extra } <= { CID, 8'hff };
					o_resp_typ <= 1'b1;
					my_cid <= 1'b1;
				end end
				// }}}
			{ 1'b0, 6'd03 }: begin // CMD3: SEND_RELATIVE_ADDR
				// {{{
				if (my_cid && (r_state == ST_IDENT
							|| r_state == ST_STBY))
				begin
					r_state <= ST_STBY;
					o_resp_valid <= 1'b1;
					// rca_assigned <= 1'b1;
					RCA <= next_rca;
					o_resp_data <= { next_rca, 16'h0 };
				end end
				// }}}
			// { 1'b0, 6'd4 }: begin // CMD4: SET_DSR, r_state <= ST_STBY
			// CMD6: SWITCH_FUNCTION (+/- DDR, etc)
			// 	r_state <= ST_DATA
			{ 1'b0, 6'd07 }: begin // CMD7: SELECT_DESELECT_CARD
				// {{{
				o_resp_data <= R1;
				if (r_state == ST_IDLE || r_state == ST_READY)
				begin
				end else if (i_arg[31:16] == RCA
					&& (r_state == ST_DIS
							|| r_state == ST_STBY))
				begin
					if (!o_dma_reset && bufcount != 0)
					begin
			 			r_state <= ST_PRG;
						o_resp_data[12:9] <= ST_PRG;
					end else begin
			 			r_state <= ST_TRAN;
						o_resp_data[12:9] <= ST_TRAN;
					end
					o_resp_valid   <= 1'b1;
				end else if (i_arg[31:16] != RCA
						&& (r_state == ST_TRAN
						  || r_state == ST_DATA
						  || r_state == ST_PRG))
				begin // Not selected
					if (r_state == ST_PRG)
					begin
			 			r_state <= ST_DIS;
						o_resp_data[12:9] <= ST_DIS;
					end else begin
			 			r_state <= ST_STBY;
						o_resp_data[12:9] <= ST_STBY;
					end
					// We don't respond, and instead allow
					// the newly selected card to respond
					o_resp_valid   <= 1'b0;
				end end
				// }}}
			{ 1'b0, 6'd08 }: begin // CMD8: SEND_IF_COND
				// {{{
				if (r_state == ST_IDLE)
				begin
					o_resp_valid <= (i_arg[11:8] == 4'h1);
					// Keep the check pattern
					o_resp_data[7:0] <= i_arg[7:0];
					// Keep the Voltage supplied indicator
					o_resp_data[11:8] <= i_arg[11:8];
					o_resp_data[31:12] <= 20'h0;
					if (first_command)
					begin
						hcs_support <= OPT_HIGH_CAPACITY;
					end
				end end
				// }}}
			{ 1'b0, 6'd9 }: begin // CMD9: SEND_CSD
				// {{{
				{ o_resp_data, o_resp_extra } <= CSD;
				o_resp_typ <= 1'b1;
				if (r_state == ST_STBY && i_arg[31:16] == RCA)
				begin
					r_state <= ST_STBY;
					o_resp_valid <= 1'b1;
				end end
				// }}}
			{ 1'b0, 6'd10 }: begin // CMD10: SEND_CID
				// {{{
				{ o_resp_data, o_resp_extra } <= { CID, 8'hff };
				o_resp_typ <= 1'b1;
				if (r_state == ST_STBY && i_arg[31:16] == RCA)
				begin
					r_state <= ST_STBY;
					o_resp_valid <= 1'b1;
				end end
				// }}}
			{ 1'b0, 6'd12 }: begin // CMD12: STOP_TRANSMISSION
					// {{{
				if (r_state == ST_DATA || r_state == ST_RCV)
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					r_reply_active <= 1'b1;
					if (r_state == ST_RCV)
					begin
						r_state <= ST_PRG;
						o_resp_data[12:9] <= ST_PRG;
					end else begin
						r_state <= ST_TRAN;
						o_resp_data[12:9] <= ST_TRAN;
					end
					// o_dma_abort <= mm2s_busy;
					// o_rx_en <= 1'b0;
					// o_tx_en <= 1'b0;
					// r_multiblock <= 1'b0;
				end end
				// }}}
			{ 1'b0, 6'd13 }: begin // CMD13: SEND_STATUS
				// {{{
				// No state transition
				if (i_arg[31:16] == RCA && r_state != ST_IDLE)
				begin
					o_resp_valid <= 1;
					o_resp_data  <= R1;
					// If CQ enabled && i_arg[15],
					//	send task status
				end end
				// }}}
			{ 1'b0, 6'd15 }: begin // CMD15: GO_INACTIVE_STATE
				// {{{
				if (r_state != ST_IDLE && i_arg[31:16] == RCA)
				begin
					r_state <= ST_IDLE;
					r_inactive <= 1'b1;
					o_resp_valid <= 1'b0;
				end end
				// }}}
			{ 1'b0, 6'd16 }: begin // CMD16: SET_BLOCKLEN
				// {{{
				if (r_state == ST_TRAN)
				begin
					o_resp_valid <= 1'b1;
					o_resp_data  <= R1;
					if (w_valid_blksz)
						o_cfg_lgblksz <= w_new_blksz;
				end end
				// }}}
			{ 1'b0, 6'd17 }: begin // CMD17: READ_SINGLE_BLOCK
				// {{{
				if (r_state == ST_PRG || r_state == ST_RCV
					|| (r_state == ST_TRAN && o_tx_busy))
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1 | ILLEGAL_COMMAND;
				end else if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					o_resp_data[12:9] <= ST_DATA;
					o_cfg_lgblksz <= 4'h9;
					r_reply_active <= 1'b1;
				end end
				// }}}
			{ 1'b0, 6'd18 }: begin // CMD18: READ_MULTIPLE_BLOCK
				// {{{
				if (r_state == ST_PRG || r_state == ST_RCV
					|| (r_state == ST_TRAN && o_tx_busy))
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1 | ILLEGAL_COMMAND;
				end else if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					if (!o_tx_busy)
					begin
						o_resp_data[12:9] <= ST_DATA;
						o_cfg_lgblksz <= 4'h9;
						r_reply_active <= 1'b1;
					end else
						o_resp_data <= R1
							| ILLEGAL_COMMAND;
				end end
				// }}}
			{ 1'b0, 6'd19 }: begin // CMD19: SEND_TUNING_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					o_resp_data[12:9] <= ST_DATA;
					r_reply_active <= 1'b1;
					// The SD Config Register is 8bytes
					if (r_width)
						// 128 clocks, 4b/clk => 64B
						o_cfg_lgblksz <= 4'h6;
					else
						// 128 clocks, 1b/clk, = 16B
						o_cfg_lgblksz <= 4'h4;
				end end
				// }}}
			// CMD20: SPEED_CLASS_CONTROL
			//	if (r_state == ST_TRAN)
			//		r_state <= ST_DATA;
			// { 1'b0, 6'd23 }: // CMD23: SET_BLOCK_COUNT
			//	r_state <= ST_TRAN
			{ 1'b0, 6'd24 }: begin // CMD24: WRITE_BLOCK
				// {{{
				if (r_state == ST_PRG || r_state == ST_RCV
					|| (r_state == ST_TRAN && o_tx_busy))
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1 | ILLEGAL_COMMAND;
				end else if (r_state == ST_TRAN)
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					r_state <= ST_RCV;
					o_resp_data[12:9] <= ST_RCV;
					o_cfg_lgblksz <= 4'h9;
				end end
				// }}}
			{ 1'b0, 6'd25 }: begin // CMD25: WRITE_MULTIPLE_BLOCK
				// {{{
				if (r_state == ST_PRG || r_state == ST_RCV
					|| (r_state == ST_TRAN && o_tx_busy))
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1 | ILLEGAL_COMMAND;
				end else if (r_state == ST_TRAN)
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					r_state <= ST_RCV;
					o_resp_data[12:9] <= ST_RCV;
					o_cfg_lgblksz <= 4'h9;
				end end
				// }}}
			// CMD26: (Reserved CMD) r_state <= ST_RCV
			// CMD27: PROGRAM_CSD, r_state <= ST_RCV
			// CMD30: SEND_WRITE_PROTECT, r_state <= ST_DATA;
			// { 1'b0, 6'd32 }: // CMD32: ERASE_WR_BLK_START
			//	r_state <= ST_TRAN
			// { 1'b0, 6'd33 }: // CMD33, ERASE_WR_BLK_END
			//	r_state <= ST_TRAN
			// CMD38: ERASE
			// CMD39: (Reserved)
			// CMD41: (Reserved)
			// CMD42: LOCK_UNLOCK, r_state <= ST_RCV
			// CMD43: Q_MANAGEMENT
			// CMD44: Q_TASK_INFO_A
			// CMD45: Q_TASK_INFO_B
			// CMD46: Q_RD_TASK
			// CMD47: Q_WR_TASK
			// CMD48: READ_EXTR_SINGLE, r_state <= ST_DATA;
			// CMD49: WRITE_EXTR_SINGLE, r_state <= ST_RCV
			{ 1'b?, 6'd55 }: begin	// CMD55: APP_CMD, ACMD follows
				// {{{
				// No state change
				if (r_state != ST_STBY || i_arg[31:16] == RCA)
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					app_cmd <= 1'b1;
				end end
				// }}}
			// CMD56: GEN_CMD
			//	if (card_selected && i_arg[0])
			//		r_state <= ST_DATA;
			//	else if (card_selected && !i_arg[0])
			//		r_state <= ST_RCV;
			// CMD58: READ_EXTR_MULTI
			//	if (card_selected) r_state <= ST_DATA;
			// CMD59: WRITE_EXTR_MULTI
			//	if (card_selected) r_state <= ST_RCV;
			default: if (r_state != ST_STBY && r_state != ST_IDLE)
				begin
					o_resp_valid <= 1'b1;
					o_resp_data <= R1 | ILLEGAL_COMMAND;
				end
			endcase
		end // if (i_cmd_valid)
	end

	assign	o_cfg_ds = 1'b0;

	always @(*)
	begin
		new_bufcount = bufcount;
		case({ !o_dma_abort &&((mm2s_busy && i_mm2s_done) || i_rx_good),
					(i_s2mm_done || i_tx_done)})
		2'b10: new_bufcount = bufcount + 1;
		2'b01: if (bufcount > 0) new_bufcount = bufcount - 1;
		default: new_bufcount = bufcount;
		endcase

		new_dma_request = o_dma_request;
		if (!o_cfg_valid || i_cfg_ready)
		begin
			new_dma_request = 1'b0;
			case(o_dma_dir)
			D_DEV2HOST:
				if (r_multiblock && !o_dma_request
							&& new_bufcount < 2)
					new_dma_request = (o_tx_src == S_MEM);
			D_HOST2DEV: if (!s2mm_busy && new_bufcount > 0)
					new_dma_request = 1'b1;
			endcase
		end

		new_rx_en = o_rx_en && !i_rx_good && !i_rx_err;
		new_tx_en = o_tx_en && !i_tx_done;

		case(o_dma_dir)
		D_DEV2HOST: if (!o_tx_en && new_bufcount > 0
							&& !r_reply_active
							&& r_state == ST_DATA)
			// Once a page is loaded, send it, but first
			// guarantee an empty cycle
			new_tx_en = 1'b1;
		D_HOST2DEV: if (r_multiblock && !o_rx_en && bufcount < 2)
			new_rx_en = 1'b1;
		endcase

	end

	initial	o_dma_request = 1'b0;
	initial	o_dma_abort   = 1'b0;
	initial	o_dma_reset   = 1'b1;
	initial	bufcount = 2'b0;
	initial	o_tx_en = 1'b0;
	initial	o_rx_en = 1'b0;
	initial	mm2s_busy = 1'b0;
	initial	s2mm_busy = 1'b0;
	initial	r_multiblock = 1'b0;
	always @(posedge i_clk)
	if (i_reset || r_inactive)
	begin
		// {{{
		if (i_reset)
			o_cfg_valid <= 1'b0;
		else if (!o_dma_reset)
			o_cfg_valid <= 1'b1;
		else if (i_cfg_ready)
			o_cfg_valid <= 1'b0;

		if (i_reset || i_cfg_ready)
			o_dma_request <= 1'b0;
		if (i_reset)
		begin
			o_dma_abort <= 1'b0;
		end else if (o_dma_request)
		begin
			o_dma_abort <= 1'b1;
			o_cfg_valid <= 1'b1;
		end else if (i_cfg_ready)
		begin
			o_dma_abort <= 1'b0;

			if (o_dma_abort)
				o_cfg_valid <= 1'b1;
		end

		o_dma_dir <= 1'b0;
		o_dma_addr <= 0;
		o_dma_reset <= 1'b1;

		o_rx_en <= 1'b0;
		o_tx_en <= 1'b0;

		o_tx_src <= S_MEM;

		bufcount <= 2'b0;
		mm2s_busy <= 1'b0;
		s2mm_busy <= 1'b0;
		r_multiblock <= 1'b0;
		// }}}
	end else begin
		if (o_dma_reset && bufcount == 0 && (!o_tx_en || i_tx_done))
			o_tx_src <= S_MEM;

		if (i_cfg_ready)
		begin
			o_cfg_valid   <= (o_dma_request || o_dma_abort);
			o_dma_request <= 1'b0;
			o_dma_abort   <= 1'b0;
			// o_dma_reset   <= 1'b0;
			if (o_dma_abort)
			begin
				s2mm_busy <= 1'b0;
				mm2s_busy <= 1'b0;
			end
		end

		o_dma_request <= new_dma_request;
		o_tx_en <= new_tx_en;
		o_rx_en <= new_rx_en;

		// HOST2DEV operation complete, reset interface
		if (s2mm_busy && (i_s2mm_done || i_s2mm_err))
		begin
			s2mm_busy <= 1'b0;
			if (bufcount < 2 && !r_multiblock && !o_rx_en)
			begin
				o_dma_reset <= 1;
				o_cfg_valid <= 1'b1;
			end
		end

		if ((s2mm_busy && i_s2mm_done) || (i_mm2s_done && mm2s_busy))
			o_dma_addr <= o_dma_addr + (1<<o_cfg_lgblksz);

		if (new_dma_request)
		begin
			if (o_dma_dir == D_DEV2HOST)
				mm2s_busy <= 1'b1;
			else
				s2mm_busy <= 1'b1;
		end else begin
			if (mm2s_busy && (i_mm2s_done || i_mm2s_err))
				mm2s_busy <= 1'b0;

			if (s2mm_busy && (i_s2mm_done || i_s2mm_err))
				s2mm_busy <= 1'b0;
		end

		if (i_tx_done && !r_multiblock)
		begin
			o_dma_reset <= 1;
			o_cfg_valid <= 1'b1;
		end

		bufcount <= new_bufcount;
		if (o_dma_reset)
			bufcount <= 0;

		if (!o_dma_request && new_dma_request)
		begin
			o_cfg_valid <= 1'b1;
			case(o_dma_dir)
			D_DEV2HOST: mm2s_busy <= 1'b1;
			D_HOST2DEV: s2mm_busy <= 1'b1;
			endcase
		end

		if (!r_multiblock
			&& (new_bufcount == 0 && !mm2s_busy && !s2mm_busy)
			&& !o_tx_en && !o_rx_en)
		begin
			if (!o_dma_reset)
				o_cfg_valid <= 1'b1;
			o_dma_reset <= 1'b1;
			bufcount <= 0;
		end

		if (i_cmd_valid)
		begin
			casez({ app_cmd, i_cmd[5:0] })
			// ACMD6
			{ 1'b1, 6'd13 }: begin // ACMD13: SD_STATUS
				// {{{
				if (r_state == ST_TRAN)
				begin
					// Need to wait for the reply to
					//  complete before we enable the
					//  transmit side
					// o_tx_en  <= 1'b1;
					o_tx_src <= S_STATUS;	// 512b/64B vector
					bufcount <= 1;
					o_dma_dir <= D_DEV2HOST;
				end end
				// }}}
			{ 1'b1, 6'd51 }: begin // ACMD51: SEND_SCR
				// {{{
				if (r_state == ST_TRAN)
				begin
					// Need to wait for the reply to
					//  complete before we enable the
					//  transmit side
					// o_tx_en  <= 1'b0;
					o_tx_src <= S_SCR;	// 64b/8B vector
					bufcount <= 1;
					o_dma_dir    <= D_DEV2HOST;
				end end
				// }}}
			{ 1'b?, 6'd0 }: begin // CMD0
				// {{{
				o_cfg_valid <= 1'b1;
				// o_dma_request <= 1'b0;
				o_dma_abort <= (mm2s_busy || s2mm_busy);
				o_dma_reset <= 1'b1; // mm2s_busy || s2mm_busy;
				o_tx_en <= 1'b0;
				o_rx_en <= 1'b0;
				r_multiblock <= 1'b0;
				bufcount <= 2'b00;
				if (!o_dma_request)
					o_dma_request <= 1'b0;
				if(!o_dma_request && (!s2mm_busy || i_s2mm_done || i_s2mm_err))
					s2mm_busy <= 1'b0;
				if(!o_dma_request && (!mm2s_busy || i_mm2s_done || i_mm2s_err))
					mm2s_busy <= 1'b0;
				end
				// }}}
			{ 1'b0, 6'd07 }: begin // CMD7: SELECT/DESELECT_CARD
				// {{{
				if (r_state == ST_IDLE || r_state == ST_READY)
				begin
				end else if (i_arg[31:16] == RCA
					&& (r_state == ST_DIS
							|| r_state == ST_STBY))
				begin
					// No change to anything here ...
				end else if (i_arg[31:16] != RCA
						&& (r_state == ST_TRAN
						  || r_state == ST_DATA
						  || r_state == ST_PRG))
				begin // Not selected
					o_rx_en <= 1'b0;
					o_tx_en <= 1'b0;
					r_multiblock <= 1'b0;
					o_dma_abort <= o_dma_abort || mm2s_busy;
					if (mm2s_busy)
						o_cfg_valid <= 1'b1;

					if (r_state == ST_DATA)
						bufcount <= 0;

					if(!o_dma_request)
						o_dma_request <= 1'b0;
					if(!o_dma_request && (!s2mm_busy || i_s2mm_done || i_s2mm_err))
						s2mm_busy <= 1'b0;
					if(!o_dma_request && (!mm2s_busy || i_mm2s_done || i_mm2s_err))
						mm2s_busy <= 1'b0;
				end end
				// }}}
			{ 1'b0, 6'd12 }: begin // CMD12: STOP_TRANSMISSION
				// {{{
				if (r_state == ST_DATA || r_state == ST_RCV)
				begin
					if (mm2s_busy || o_dma_dir == D_DEV2HOST)
						o_cfg_valid <= 1'b1;
					o_dma_abort <= mm2s_busy;
					o_rx_en <= 1'b0;
					o_tx_en <= 1'b0;
					r_multiblock <= 1'b0;
					o_dma_reset<= o_dma_reset || (o_dma_dir == D_DEV2HOST);
					if (r_state == ST_DATA)
						bufcount <= 0;
					if (!o_dma_request && o_dma_dir == D_DEV2HOST)
						o_dma_request <= 1'b0;
					if(!o_dma_request && (!mm2s_busy || i_mm2s_done || i_mm2s_err))
						mm2s_busy <= 1'b0;
				end end
				// }}}
			{ 1'b0, 6'd15 }: begin // CMD15: GO_INACTIVE_STATE
				// {{{
				if (r_state != ST_IDLE && r_state != ST_READY
					&& r_state != ST_IDENT
					&& i_arg[31:16] == RCA)
				begin
					if (mm2s_busy && !i_mm2s_done && !i_mm2s_err)
						o_dma_abort <= 1'b1;
					else
						o_dma_reset <= o_dma_reset || o_tx_en || o_rx_en || mm2s_busy;
					o_tx_en <= 1'b0;
					o_rx_en <= 1'b0;
					r_multiblock <= 1'b0;
					o_cfg_valid <= 1'b1;
					bufcount <= 2'b00;
					if(!o_dma_request)
						o_dma_request <= 1'b0;
					if(!o_dma_request && (!s2mm_busy || i_s2mm_done || i_s2mm_err))
						s2mm_busy <= 1'b0;
					if(!o_dma_request && (!mm2s_busy || i_mm2s_done || i_mm2s_err))
						mm2s_busy <= 1'b0;
				end end
				// }}}
			{ 1'b0, 6'd17 }: begin // CMD17: READ_SINGLE_BLOCK
				// {{{
				if (r_state == ST_TRAN && !o_tx_busy)
				begin
`ifdef	FORMAL
					assert(!o_dma_request);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					r_multiblock <= 1'b0;
					o_cfg_valid <= 1'b1;
					o_dma_request    <= 1'b1;
					mm2s_busy    <= 1'b1;
					o_dma_abort  <= 1'b0;
					o_dma_reset  <= 1'b0;
					o_dma_dir    <= D_DEV2HOST;
					// o_tx_en   <= 1'b0;
					// o_rx_en   <= 1'b0;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 9'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			{ 1'b0, 6'd18 }: begin // CMD18: READ_MULTIPLE_BLOCK
				// {{{
				if (r_state == ST_TRAN && !o_tx_busy)
				begin
`ifdef	FORMAL
					assert(!o_dma_request);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					r_multiblock <= 1'b1;
					o_cfg_valid  <= 1'b1;
					o_dma_request<= 1'b1;
					o_dma_abort  <= 1'b0;
					o_dma_reset  <= 1'b0;
					o_dma_dir    <= D_DEV2HOST;
					mm2s_busy    <= 1'b1;
					// o_tx_en   <= 1'b0;
					// o_rx_en   <= 1'b0;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 9'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			{ 1'b0, 6'd19 }: begin // CMD19: SEND_TUNING_BLOCK
				// {{{
				if (r_state == ST_TRAN && !o_tx_busy)
				begin
					// o_tx_en  <= 1'b1;
					o_tx_src <= S_TUNING;
					bufcount <= 1;
					o_dma_dir    <= D_DEV2HOST;
				end end
				// }}}
			{ 1'b0, 6'd24 }: begin // CMD24: WRITE_BLOCK
				// {{{
				if (r_state == ST_TRAN && !o_tx_busy)
				begin
`ifdef	FORMAL
					assert(!o_dma_request);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					o_rx_en <= 1'b1;
					o_dma_request <= 1'b0;
					o_dma_reset   <= 1'b0;
					o_cfg_valid   <= 1'b1;
					o_dma_dir <= D_HOST2DEV;
					r_multiblock <= 1'b0;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 9'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			{ 1'b0, 6'd25 }: begin // CMD24: WRITE_MULTIBLOCK
				// {{{
				if (r_state == ST_TRAN && !o_tx_busy)
				begin
`ifdef	FORMAL
					assert(!o_dma_request);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					o_rx_en <= 1'b1;
					o_dma_request <= 1'b0;
					o_dma_dir <= D_HOST2DEV;
					o_dma_reset   <= 1'b0;
					o_cfg_valid   <= 1'b1;
					r_multiblock <= 1'b1;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 9'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			default: begin end
			endcase
		end
	end

	assign	o_tx_busy = !o_tx_en
			&& ((o_dma_dir == D_HOST2DEV
					&& (r_state == ST_PRG) && bufcount > 0)
				|| (o_dma_dir == D_HOST2DEV
					&& (r_state == ST_RCV) && bufcount > 1)
				// ||(o_cfg_valid && !i_cfg_ready
				//	&& (o_dma_reset || o_dma_abort))
				||(r_state == ST_TRAN && mm2s_busy));

	always @(*)
	if (o_rx_en)
		assert(!o_tx_busy);
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
	reg		f_past_valid;
	reg	[3:0]	f_cmdcount;
	(* anyconst *)	reg	f_nvr_multi;

	initial	f_past_valid = 1'b0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);

	////////////////////////////////////////////////////////////////////////
	//
	// Command assumptions
	// {{{

	always @(posedge i_clk)
	if (i_reset)
		assume(!i_cmd_valid && !i_cmd_err);
	else if ($past(i_cmd_valid || i_cmd_err))
		assume(!i_cmd_valid && !i_cmd_err);

	always @(posedge i_clk)
		assume(!i_cmd_valid || !i_cmd_err);

	always @(posedge i_clk)
	if (f_past_valid && !$past(i_reset))
	begin
		/*
		if (!$past(i_cmd_err) && o_resp_valid && (o_resp == 6'd12
				|| o_resp == 6'd17
				|| o_resp == 6'd18
				|| o_resp == 6'd19))
		begin
			assert(r_reply_active);
		end
		*/

		if (r_reply_active)
		begin
			if (!$past(r_reply_active))
				assert(o_resp_valid);
			assume(!i_cmd_valid && !i_cmd_err);
		end

		assume($rose(i_cmd_busy) == $past(o_resp_valid));
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// DMA / IP assumptions
	// {{{
	always @(posedge i_clk)
	if (!i_reset)
	begin
		if (o_dma_request)
		begin
			assume(!i_s2mm_done && !i_s2mm_err);
			assume(!i_mm2s_done && !i_mm2s_err);
		end

		if (!mm2s_busy)
			assume(!i_mm2s_done && !i_mm2s_err);
		if (!s2mm_busy)
			assume(!i_s2mm_done && !i_s2mm_err);

		if (!o_tx_en)
			assume(!i_tx_done);
		if (!o_rx_en)
			assume(!i_rx_good && !i_rx_err);
	end

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(!o_dma_request);
	end else begin
		if (o_dma_abort)
			assert(o_cfg_valid);
		if (o_dma_reset && r_state != ST_IDLE)
			assert(!s2mm_busy);

		if ($changed(o_dma_reset))
			assert(o_cfg_valid);
		if ($changed(o_dma_request))
			assert(o_cfg_valid);
		if ($changed(o_dma_abort))
			assert(o_cfg_valid);

		if ($past(o_cfg_valid && !i_cfg_ready))
		begin
			// assert(!$fell(o_dma_reset));
			assert(!$fell(o_dma_request));
			assert(!$fell(o_dma_abort));
			assert(!$fell(o_dma_reset));

			assert(!$past(o_dma_request) || $stable(o_dma_dir) || (r_inactive && o_dma_abort));
			assert(!$past(o_dma_request) || !o_dma_request
					|| (r_inactive && o_dma_abort)
					|| $stable(o_dma_addr));
			assert(!$fell(o_dma_abort));
		end
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// State machine checks
	// {{{
	always @(posedge i_clk)
	if (!i_reset) case(r_state)
	ST_IDLE: begin
		// {{{
		assert(!o_dma_request || o_dma_abort || r_inactive);
		// assert(o_dma_reset);
		assert(r_inactive || o_dma_reset);
		// assert(!s2mm_busy || r_inactive);	// !!!
		assert(!$rose(s2mm_busy));
		assert(!mm2s_busy || o_dma_abort);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(o_cfg_width == 2'b00 || r_inactive);
		assert(!r_multiblock);
		assert(!o_tx_busy);
		end
		// }}}
	ST_READY: begin
		// {{{
		assert(!o_dma_request);
		assert(o_dma_reset);
		assert(!s2mm_busy);
		assert(!mm2s_busy);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(o_cfg_width == 2'b00);
		assert(!r_multiblock);
		assert(!o_tx_busy);
		end
		// }}}
	ST_IDENT: begin
		// {{{
		assert(!o_dma_request);
		assert(o_dma_reset);
		assert(!s2mm_busy);
		assert(!mm2s_busy);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(o_cfg_width == 2'b00);
		assert(!r_multiblock);
		assert(!o_tx_busy);
		end
		// }}}
	ST_STBY: begin
		// {{{
		assert(!o_dma_request || o_dma_abort);
		// assert(o_dma_reset);
		assert(!s2mm_busy);
		assert(!mm2s_busy || o_dma_abort);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(!r_multiblock);
		assert(!o_tx_busy);
		assert(RCA != 0);
		end
		// }}}
	ST_TRAN: begin
		// {{{
		assert(!s2mm_busy);
		if (mm2s_busy)
		begin
			assert(o_tx_busy);
			assert(o_dma_abort);
		end else
			assert(!o_dma_request);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(!o_tx_busy || mm2s_busy);
		assert(RCA != 0);
		assert(!r_multiblock);
cover($past(r_state) == ST_PRG);
		end
		// }}}
	ST_DATA: begin
		// {{{
		assert(o_dma_dir == D_DEV2HOST || o_tx_src != S_MEM);
		if (!r_multiblock)
		begin
			assert(!o_tx_en || !mm2s_busy);
			assert(bufcount + (mm2s_busy ? 1:0) <= 1);
		end
		// assert(!o_dma_reset || (!mm2s_busy && o_tx_src != S_MEM));
		assert(o_dma_reset == (o_tx_src != S_MEM));
		assert(!s2mm_busy);
		assert(!o_rx_en);
		if (o_tx_en && $stable(o_tx_en))
			assert($stable(o_tx_src));
		assert(RCA != 0);
		if (o_tx_src != S_MEM)
			assert(!o_dma_request && !mm2s_busy);
		case(o_tx_src)
		S_MEM: begin end
		S_SCR: assert(!mm2s_busy && o_cfg_lgblksz == 3);
		S_STATUS: assert(!mm2s_busy && o_cfg_lgblksz == 6);
		S_TUNING: begin
			assert(!mm2s_busy);
			if (r_width)
			begin
				assert(o_cfg_lgblksz == 6);
			end else
				assert(o_cfg_lgblksz == 4);
			end
		default: assert(0);
		endcase end
		// }}}
	ST_RCV: begin
		// {{{
		assert(o_dma_dir == D_HOST2DEV);
		assert(!mm2s_busy);
		assert(!o_tx_en);
		assert(RCA != 0);
		assert(!o_dma_reset || (!s2mm_busy && !o_rx_en && bufcount==0));
		end
		// }}}
	ST_PRG: begin
		// {{{
		assert(o_tx_busy || bufcount == 0);
		assert(o_dma_dir == D_HOST2DEV);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(!r_multiblock);
		assert(RCA != 0);
		assert(!mm2s_busy);
		assert(!o_dma_reset || (bufcount == 0 && !s2mm_busy));
		end
		// }}}
	ST_DIS: begin
		// {{{
		assert(!mm2s_busy);
		assert(!o_tx_busy);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(!o_tx_busy);
		assert(!r_multiblock);
		assert(o_dma_dir == D_HOST2DEV);
		assert(!o_dma_reset || (bufcount == 0 && !s2mm_busy));
		assert(RCA != 0);
		end
		// }}}
	default: assert(0);
	endcase

	always @(posedge i_clk)
	if (f_nvr_multi)
	begin
		assume(!i_cmd_valid || app_cmd
			|| (i_cmd[5:0] != 6'd18 && i_cmd[5:0] != 6'd25));
		assert(!r_multiblock);
		assert(bufcount <= 1);
		if (bufcount == 1)
		begin
			assert(!mm2s_busy);
			assert(!o_rx_en);
		end
	end

	always @(posedge i_clk)
	if (!i_reset && !$past(i_reset))
	begin
		if (o_tx_en)
			assert(bufcount > 0);
		if (o_tx_src != S_MEM)
		begin
			assert(o_dma_dir == D_DEV2HOST);
			assert(o_dma_reset);
			assert(!o_dma_request);
			assert(!mm2s_busy);
			assert(!s2mm_busy);
			assert(bufcount <= 1);
		end
		assert(bufcount <= 2);
		if (bufcount == 2)
		begin
			if (o_dma_dir == D_HOST2DEV)
			begin
				assert(o_tx_busy || r_state == ST_DIS);
			end else if (o_dma_dir == D_DEV2HOST)
			begin
				assert(!o_dma_request);
				assert(!mm2s_busy);
			end
		end

		if (o_dma_request && (!r_inactive || !o_dma_abort))
		begin
			assert(s2mm_busy || mm2s_busy);
			assert(s2mm_busy == (o_dma_dir == D_HOST2DEV));
		end

		if (bufcount > 0 && $past(bufcount > 0)
					&& o_dma_dir == D_DEV2HOST
					&& r_state == ST_DATA)
		begin
			assert(o_tx_en || $past(o_tx_en || r_reply_active));
		end

		assert(!o_cfg_cmd_pp);
		assert(!o_cfg_dat_pp);
		if (mm2s_busy || s2mm_busy)
		begin
			assert(o_tx_src == S_MEM);
			assert($stable(o_cfg_dat_pp));
			assert($stable(o_cfg_cmd_pp));
			// assert($stable(o_cfg_ds));
			assert($rose(mm2s_busy || s2mm_busy)
					|| $stable(o_cfg_lgblksz)
					|| r_state == ST_IDLE);
			assert($stable(o_cfg_width) || r_state == ST_IDLE);
		end

		if ($past(o_cfg_valid && !i_cfg_ready && o_dma_request)
				&& !o_dma_abort && !o_dma_reset)
			assert(o_dma_request && $stable(o_dma_addr));

		assert(!o_tx_en || !o_rx_en);

		if ($past(o_tx_en))
		begin
			if ($past(i_tx_done))
			begin
				assert(!o_tx_en);
			end else if (!$past(i_cmd_valid)
				|| $past((i_cmd != 6'h0) &&(i_cmd != 6'd12)
					&&(i_cmd != 6'd7)
					&&(i_cmd != 6'd15)))
			begin
				assert(o_tx_en);
			end
		end

		if (s2mm_busy && !r_inactive && r_state != ST_IDLE)
			assert(bufcount > 0);

		if ($past(o_rx_en))
		begin
			if ($past(i_rx_good || i_rx_err))
			begin
				assert(!o_rx_en);
			end else if (!$past(i_cmd_valid)
				|| $past(i_cmd != 6'h0 && i_cmd != 6'd12
					&& i_cmd != 6'd07 && i_cmd != 6'd15))
			begin
				assert(o_rx_en);
			end
		end

		if ($past(o_rx_en && (i_rx_good || i_rx_err)))
			assert(!o_rx_en);
	end

	// Let's make sure we never get stuck in certain states ...
	always @(posedge i_clk)
	if (f_past_valid || $past(i_reset) || i_reset || r_inactive)
	begin
	end else if (!$past(i_cmd_valid)) case($past(r_state))
	ST_DATA: if ($past(bufcount == 0) && $past(!mm2s_busy)
				&& !$past(o_tx_en) && !$past(r_multiblock))
		assert(r_state == ST_TRAN);
	ST_PRG: if ($past(bufcount == 0) && $past(!s2mm_busy))
		assert(r_state == ST_TRAN);
	ST_DIS: if ($past(bufcount == 0) && $past(!s2mm_busy))
		assert(r_state == ST_STBY);
	default: begin end
	endcase

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Quick coverage checks
	// {{{
	reg	[3:0]	cvr_single_read, cvr_single_write;
	reg	[3:0]	cvr_multi_read, cvr_multi_write;

	initial	cvr_single_read = 4'b0;
	always @(posedge i_clk)
	if (i_reset || r_multiblock
			|| (i_cmd_valid && i_cmd == 6'd0)
			|| (i_cmd_valid && !app_cmd && i_cmd == 6'd12))
		cvr_single_read <= 4'b0;
	else if (o_tx_en && i_tx_done && !cvr_single_read[3] && o_tx_src == S_MEM)
		cvr_single_read <= cvr_single_read + 1;

	initial	cvr_single_write = 4'b0;
	always @(posedge i_clk)
	if (i_reset || r_multiblock
			|| (i_cmd_valid && i_cmd == 6'd0)
			|| (i_cmd_valid && !app_cmd && i_cmd == 6'd12))
		cvr_single_write <= 4'b0;
	else if (s2mm_busy && i_s2mm_done && !cvr_single_write[3])
		cvr_single_write <= cvr_single_write + 1;

	initial	cvr_multi_read = 4'b0;
	always @(posedge i_clk)
	if (i_reset || (i_cmd_valid && i_cmd[5:0] == 6'd0)
			// No single block reads allowed
			|| (i_cmd_valid && !app_cmd && i_cmd[5:0] == 6'd17))
		cvr_multi_read <= 4'b0;
	else if (o_tx_en && i_tx_done && !cvr_multi_read[3] && o_tx_src == S_MEM)
		cvr_multi_read <= cvr_multi_read + 1;

	initial	cvr_multi_write = 4'b0;
	always @(posedge i_clk)
	if (i_reset || (i_cmd_valid && i_cmd[5:0] == 6'd0)
			// No single block writes allowed
			|| (i_cmd_valid && !app_cmd && i_cmd[5:0] == 6'd24))
		cvr_multi_write <= 4'b0;
	else if (s2mm_busy && i_s2mm_done && !cvr_multi_write[3])
		cvr_multi_write <= cvr_multi_write + 1;

	always @(posedge i_clk)
	if (!i_reset)
	begin
		cover(r_state == ST_TRAN);

		cover(s2mm_busy);
		cover(s2mm_busy && !o_dma_request);
		cover(s2mm_busy && i_s2mm_done);
		cover(s2mm_busy && i_s2mm_done && r_state == ST_RCV);
		cover(s2mm_busy && i_s2mm_done && r_state == ST_PRG);
		cover(s2mm_busy && i_s2mm_done && r_state == ST_DIS);

		cover(cvr_single_read  > 0);	//	16
		cover(cvr_multi_read   > 0);	//	16
		cover(cvr_single_write > 0);	//
		cover(cvr_multi_write  > 0);	//	16

		cover(r_state == ST_TRAN && cvr_single_read  > 0);	//	16
		cover(r_state == ST_TRAN && cvr_multi_read   > 0);	//	16
		cover(r_state == ST_TRAN && cvr_single_write > 0);	//	16
		cover(r_state == ST_TRAN && cvr_multi_write  > 0);	//	16

		cover(cvr_single_read[2]);	//	31
		cover(cvr_multi_read[2]);	//	22
		cover(cvr_single_write[2]);	//	30
		cover(cvr_multi_write[2]);	//	25

		cover(r_state == ST_TRAN && !mm2s_busy && cvr_single_read[2]);	// 31
		cover(r_state == ST_TRAN && !mm2s_busy && cvr_multi_read[2]);	// 23
		cover(r_state == ST_TRAN && !s2mm_busy && cvr_single_write[2]);	// 30
		cover(r_state == ST_TRAN && !s2mm_busy && cvr_multi_write[2]);	// 25

		cover(cvr_single_read[3]);	//	51
		cover(cvr_multi_read[3]);	//	30
		cover(cvr_single_write[3]);	//	50
		cover(cvr_multi_write[3]);	//	37

		cover(r_state == ST_TRAN && !mm2s_busy && cvr_single_read[3]);	// 51
		cover(r_state == ST_TRAN && !mm2s_busy && cvr_multi_read[3]);	// 31
		cover(r_state == ST_TRAN && !s2mm_busy && cvr_single_write[3]);	// 50
		cover(r_state == ST_TRAN && !s2mm_busy && cvr_multi_write[3]);	// 37
		cover(r_state == ST_PRG  && !s2mm_busy && cvr_multi_write[3]
			&& bufcount == 0);	// Step 37
		cover(r_state == ST_DIS  && !s2mm_busy && cvr_multi_write[3]
			&& bufcount == 0);	// Step 37
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// "Careless" assumptions
	// {{{
	always @(*)
		assume(!i_mm2s_err && !i_s2mm_err && !i_rx_err);
	always @(*)
		assume(!o_dma_reset || !o_cfg_valid || !i_cmd_valid);

	always @(posedge i_clk)
	if (o_cfg_valid)
	begin
		assume(i_cfg_ready || $past(i_cfg_ready)
						|| $past(i_cfg_ready,2));
	end

	always @(posedge i_clk)
	if ($past(o_cfg_valid && !i_cfg_ready))
		assume(!i_cmd_valid && !i_cmd_err);

	always @(posedge i_clk)
	if (!i_reset && r_state == ST_IDLE)
		assume(!i_cmd_valid || !s2mm_busy);
	// }}}
`endif
// }}}
endmodule
