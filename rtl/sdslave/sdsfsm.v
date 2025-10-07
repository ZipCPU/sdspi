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
		// {{{
		parameter	ADDRESS_WIDTH = 32,
		parameter [0:0]	OPT_HIGH_CAPACITY = 1'b1,
		parameter [0:0]	OPT_1P8V = 1'b0,
		parameter [0:0]	OPT_UHSII = 1'b0,
		parameter [127:0]	CID = { 64'hdadd_3519_2347_291a,
						64'habca_dead_519d_dad1 },
		parameter [15:0]	OCR_VOLTAGE = 16'hff_80
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Configuration / PHY control
		// {{{
		output	reg		o_cfg_pp,
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
		// }}}
		// RX (host -> slave) control
		// {{{
		output	wire		o_rx_en,
					// o_rx_abort (?)
		// output	wire [AW-1:0]	o_rx_addr,
		input	wire		i_rx_good, i_rx_err,
		// }}}
		// TX (slave -> host) control
		// {{{
		output	wire		o_tx_en, o_tx_busy,
		output	wire	[1:0]	o_tx_src,
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
				S_STATUS = 2'b10;

	// Verilator lint_off UNUSED

	// Card status bit definitions
	localparam	[3:0]	ST_IDLE  = 0,
				ST_READY = 1,
				ST_IDENT = 2,
				ST_STBY  = 3,
				ST_TRAN  = 4,
				ST_DATA  = 5,
				ST_RCV   = 6,
				ST_PRG   = 7,
				ST_DIS   = 8;
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
	reg	[15:0]	RCA, next_rca;
	reg	[1:0]	bufcount;
	reg	[3:0]	w_new_blksz;
	wire		transfer_complete, operation_complete;

	wire	[127:0]	CSD;
	// }}}

	assign	R1 = { 19'h0, r_state, 3'h0, app_cmd, 5'h0 };
	assign	CSD = 128'h0;

	assign	o_cfg_ds = 1'b0;
	assign	o_cfg_width = { 1'b0, r_width };
	assign	R1 = 32'h0; // | sd_state | BUS_ERR | CRC_ERR | CMD_ERR | APPCMD

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
	assign	transfer_complete = (o_tx_en && i_tx_done);
	assign	operation_complete = (o_tx_en && i_tx_done) || o_dma_reset;

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

	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		r_state <= ST_IDLE;
		app_cmd <= 1'b0;
		r_width <= 1'b0;
		o_resp_valid <= 1'b0;
		o_resp_nocrc <= 1'b0;	// (Most) everything gets a CRC
		o_cfg_pp <= 1'b0;
		o_cfg_ddr <= 1'b0;
		o_cfg_lgblksz <= 4'h9;
		r_width <= 1'b0;
		r_multiblock <= 1'b0;
		RCA <= 16'hdad0;
		// rca_assigned <= 1'b0;
		my_cid <= 1'b0;
		r_inactive <= 1'b0;
		first_command <= 1'b0;
		o_cfg_valid <= 1'b0;
		// }}}
	end else begin
		o_resp_valid <= 1'b0;
		o_resp_typ   <= 1'b0;	// 32b/48b response
		o_resp_nocrc <= 1'b0;	// (Most) everything gets a CRC
		if (i_collision)
			my_cid <= 1'b0;
		o_resp_extra <= CID[95:0];

		if (!o_dma_request && r_state == ST_PRG && bufcount == 0
				&& !r_multiblock)
			r_state <= ST_TRAN;

		if (transfer_complete && r_state == ST_RCV && !r_multiblock)
			r_state <= ST_PRG;
		if (operation_complete && (r_state == ST_PRG
							|| r_state == ST_DATA))
			r_state <= ST_TRAN;
		if (operation_complete && r_state == ST_DIS)
			r_state <= ST_STBY;

		if (i_cmd_valid && !r_inactive)
			first_command <= 1'b0;

		if (i_cmd_err && !r_inactive && (r_state != ST_STBY
					&& r_state != ST_IDLE))
		begin
			o_resp_valid <= 1'b1;
			o_resp_data  <= R1 | COM_CRC_ERROR;
		end else if (i_cmd_valid && !r_inactive)
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
				begin
					if (0 != i_arg[23:0] && !i_arg[30])
						hcs_support <= 1'b0;
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
				end end
				// }}}
			{ 1'b?, 6'd0 }: begin	// CMD0: GO_IDLE
				// {{{
				r_state <= ST_IDLE;
				// No response to the GO_IDLE command
				o_resp_valid <= 1'b0;
				o_cfg_pp <= 1'b0;
				o_cfg_ds <= 1'b0;
				o_cfg_ddr <= 1'b0;
				o_cfg_lgblksz <= 4'h9;
				r_width <= 1'b0;
				// rca_assigned <= 1'b0;
				my_cid <= 1'b0;
				first_command <= 1'b0;
				end
				// }}}
			{ 1'b0, 6'd02 }: begin // CMD2: ALL_SEND_CID
				// {{{
				my_cid <= 1'b0; // Avoid collisions
				if (r_state == ST_IDLE)
				begin
					o_resp_valid <= 1'b1;
					{ o_resp_data, o_resp_extra } <= CID;
					o_resp_typ <= 1'b1;
					my_cid <= 1'b1;
				end end
				// }}}
			{ 1'b0, 6'd03 }: begin // CMD3: SEND_RELATIVE_ADDR
				// {{{
				if (my_cid && (r_state == ST_IDLE
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
				if (i_arg[31:16] == RCA && (r_state == ST_DIS
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
							&& r_state != ST_IDLE)
				begin // Not selected
					if (!o_dma_reset && bufcount != 0)
					begin
			 			r_state <= ST_DIS;
						o_resp_data[12:9] <= ST_DIS;
					end else begin
			 			r_state <= ST_STBY;
						o_resp_data[12:9] <= ST_STBY;
					end
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
				{ o_resp_data, o_resp_extra } <= CID;
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
					if (r_state == ST_RCV)
						r_state <= ST_PRG;
					else
						r_state <= ST_TRAN;
					// o_dma_abort <= mm2s_busy;
					// o_rx_en <= 1'b0;
					// o_tx_en <= 1'b0;
					r_multiblock <= 1'b0;
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
				if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					// o_tx_en <= 1'b1	// Must be in another FSM
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					o_resp_data[12:9] <= ST_DATA;
					r_multiblock <= 1'b0;
					o_cfg_lgblksz <= 4'h9;
				end end
				// }}}
			{ 1'b0, 6'd18 }: begin // CMD18: READ_MULTIPLE_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
			 		r_state <= ST_DATA;
					// o_tx_en <= 1'b1	// Must be in another FSM
					o_resp_valid <= 1'b1;
					o_resp_data <= R1;
					o_resp_data[12:9] <= ST_DATA;
					r_multiblock <= 1'b1;
					// dma_addr <= i_arg[AW-1:0];
					o_cfg_lgblksz <= 4'h9;
					// if (hcs_support)
					//	dma_addr <= { i_arg[AW-9:0], 8'h0 };
				end end
				// }}}
			// CMD19: SEND_TUNING_BLOCK
			//	r_state <= ST_DATA;
			// CMD20: SPEED_CLASS_CONTROL
			//	if (r_state == ST_TRAN)
			//		r_state <= ST_DATA;
			// { 1'b0, 6'd23 }: // CMD23: SET_BLOCK_COUNT
			//	r_state <= ST_TRAN
			{ 1'b0, 6'd24 }: begin // CMD24: WRITE_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
					o_resp_valid <= 1'b1;
					r_state <= ST_RCV;
					// o_rx_en <= 1'b1;
					r_multiblock <= 1'b0;
					// dma_addr <= i_arg[AW-1:0];
					o_cfg_lgblksz <= 4'h9;
					// if (hcs_support)
						// dma_addr <= { i_arg[AW-9:0], 8'h0 };
				end end
				// }}}
			{ 1'b0, 6'd25 }: begin // CMD25: WRITE_MULTIPLE_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
					o_resp_valid <= 1'b1;
					r_state <= ST_RCV;
					// o_rx_en <= 1'b1;
					r_multiblock <= 1'b1;
					// dma_addr <= i_arg[AW-1:0];
					o_cfg_lgblksz <= 4'h9;
					// if (hcs_support)
						// dma_addr <= { i_arg[AW-9:0], 8'h0 };
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

	localparam [0:0]	D_DEV2HOST = 1'b0,
				D_HOST2DEV = 1'b1;

	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		o_cfg_valid <= 1'b0;
		o_dma_request <= 1'b0;
		o_dma_dir <= 1'b0;
		o_dma_addr <= 0;
		o_dma_reset <= 1'b1;

		o_rx_en <= 1'b1;
		o_tx_en <= 1'b1;

		o_tx_src <= S_MEM;
		// }}}
	end else begin
		if (o_dma_reset)
			o_tx_src <= S_MEM;

		if (i_cfg_ready)
		begin
			o_cfg_valid   <= (o_dma_request || o_dma_abort);
			o_dma_request <= 1'b0;
			o_dma_abort   <= 1'b0;
			// o_dma_reset   <= 1'b0;
		end

		if (i_s2mm_done || i_s2mm_err)
		begin
			s2mm_busy <= 1'b0;
			if (bufcount < 2 && !r_multiblock)
			begin
				o_dma_reset <= 1;
				o_cfg_valid <= 1'b1;
			end
		end

		if (i_s2mm_done || i_mm2s_done)
			o_dma_addr <= o_dma_addr + (1<<o_cfg_lgblksz);

		if (i_mm2s_done || i_mm2s_err)
			mm2s_busy <= 1'b0;

		if (i_s2mm_done || i_s2mm_err)
			s2mm_busy <= 1'b0;

		if (i_tx_done && !r_multiblock)
		begin
			o_dma_reset <= 1;
			o_cfg_valid <= 1'b1;
		end

		case({ (i_mm2s_done || i_rx_good), (i_s2mm_done || i_tx_done)})
		2'b10: bufcount <= bufcount + 1;
		2'b01: if (bufcount > 0) bufcount <= bufcount - 1;
		default: begin end
		endcase

		if (i_tx_done) o_tx_en <= 1'b0;
		if (i_rx_good || i_rx_err) o_rx_en <= 1'b0;

		if (r_multiblock) //  && !dma_err)
		begin
			case(o_dma_dir)
			D_DEV2HOST: begin
				if (!o_tx_en && bufcount > 0)
					o_tx_en <= 1'b1;
				if (!o_dma_request && bufcount < 2)
				begin
					o_cfg_valid <= 1'b1;
					o_dma_request <= 1'b1;
					mm2s_busy <= 1'b1;
				end end
			D_HOST2DEV: begin
				if (!o_rx_en && bufcount < 2)
					o_rx_en <= 1'b1;
				if (!o_dma_request && bufcount > 0)
				begin
					o_cfg_valid <= 1'b1;
					o_dma_request <= 1'b1;
					s2mm_busy <= 1'b1;
				end end
			endcase
		end

		if (!r_multiblock && !mm2s_busy && !s2mm_busy && !o_tx_en && !o_rx_en)
		begin
			if (!o_dma_reset)
				o_cfg_valid <= 1'b1;
			o_dma_reset <= 1'b1;
			bufcount <= 0;
		end

		if (i_cmd_valid && !r_inactive)
		begin
			casez({ app_cmd, i_cmd[5:0] })
			// ACMD6
			{ 1'b1, 6'd13 }: begin // ACMD13: SD_STATUS
				// {{{
				if (r_state == ST_TRAN)
				begin
					o_tx_en  <= 1'b1;
					o_tx_src <= S_STATUS;	// 512b/64B vector
				end end
				// }}}
			{ 1'b1, 6'd51 }: begin // ACMD51: SEND_SCR
				// {{{
				if (r_state == ST_TRAN)
				begin
					o_tx_en  <= 1'b1;
					o_tx_src <= S_SCR;	// 64b/8B vector
				end end
				// }}}
			{ 1'b1, 6'd0 }: begin // CMD0
				// {{{
				o_cfg_valid <= 1'b1;
				o_dma_request <= 1'b0;
				o_dma_abort <= (mm2s_busy || s2mm_busy);
				o_tx_en <= 1'b0;
				o_rx_en <= 1'b0;
				o_dma_reset <= 1'b1;
				end
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
					o_dma_reset<= (o_dma_dir == D_DEV2HOST);
				end end
				// }}}
			{ 1'b0, 6'd15 }: begin // CMD15: GO_INACTIVE_STATE
				// {{{
				if (r_state != ST_IDLE && i_arg[31:16] == RCA)
				begin
					// r_state <= ST_IDLE;
					// r_inactive <= 1'b1;
					// o_resp_valid <= 1'b0;
					o_dma_abort <= mm2s_busy;
					o_dma_reset <= (o_tx_en || o_rx_en);
					o_tx_en <= 1'b0;
					o_rx_en <= 1'b0;
					r_multiblock <= 1'b0;
					o_cfg_valid <= 1'b1;
				end end
				// }}}
			{ 1'b0, 6'd17 }: begin // CMD17: READ_SINGLE_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
`ifdef	FORMAL
					assert(!o_dma_req);
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
						o_dma_addr <= { i_arg[AW-9:0], 8'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			{ 1'b0, 6'd18 }: begin // CMD17: READ_MULTIPLE_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
`ifdef	FORMAL
					assert(!o_dma_req);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					r_multiblock <= 1'b1;
					o_cfg_valid <= 1'b1;
					o_dma_request    <= 1'b1;
					o_dma_abort  <= 1'b0;
					o_dma_reset  <= 1'b0;
					o_dma_dir    <= D_DEV2HOST;
					mm2s_busy    <= 1'b1;
					// o_tx_en   <= 1'b0;
					// o_rx_en   <= 1'b0;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 8'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			{ 1'b0, 6'd24 }: begin // CMD24: WRITE_BLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
`ifdef	FORMAL
					assert(!o_dma_req);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					o_rx_en <= 1'b1;
					o_dma_request <= 1'b0;
					o_dma_dir <= D_HOST2DEV;
					r_multiblock <= 1'b0;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 8'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			{ 1'b0, 6'd25 }: begin // CMD24: WRITE_MULTIBLOCK
				// {{{
				if (r_state == ST_TRAN)
				begin
`ifdef	FORMAL
					assert(!o_dma_req);
					assert(!o_rx_en);
					assert(!o_tx_en);
					assert(!mm2s_busy);
					assert(!s2mm_busy);
					assert(bufcount == 0);
`endif
					o_rx_en <= 1'b1;
					o_dma_request <= 1'b0;
					o_dma_dir <= D_HOST2DEV;
					r_multiblock <= 1'b1;
					if (hcs_support)
						o_dma_addr <= { i_arg[AW-9:0], 8'h0 };
					else
						o_dma_addr <= i_arg[AW-1:0];
				end end
				// }}}
			default: begin end
			endcase
		end
	end

	assign	o_tx_busy = !o_tx_en && !o_dma_reset && o_dma_dir == D_HOST2DEV
			&& (r_state != ST_DIS) && bufcount > 0;
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

	initial	f_past_valid;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(*)
	if (!f_past_valid)
		assume(i_reset);

	////////////////////////////////////////////////////////////////////////
	//
	// Command assumptions
	// {{{

	always @(poesdge i_clk)
	if (i_reset)
		assume(!i_cmd_valid);
	else if ($past(i_cmd_valid))
		assume(!i_cmd_valid);

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
			assume(!i_mm2s_done && !i_mm2s_err);
		end
	end

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(!o_dma_request);
	end else begin
		if ($changed(o_dma_reset))
			assert(o_cfg_valid);
		if ($changed(o_dma_request))
			assert(o_cfg_valid);
		if ($changed(o_dma_abort))
			assert(o_cfg_valid);

		if ($past(o_cfg_valid && !o_cfg_ready))
		begin
			// assert(!$fell(o_dma_reset));
			assert(!$fell(o_dma_request));
			assert(!$fell(o_dma_abort));
			assert(!$fell(o_dma_reset));

			assert($stable(o_dma_dir));
			assert($stable(o_dma_addr));
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
		assert(!o_dma_request);
		// assert(o_dma_reset);
		assert(!o_dma_reset);
		assert(!s2mm_busy);
		assert(!mm2s_busy);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(o_cfg_width == 2'b00);
		end
	// ST_READY:
	// ST_IDENT:
	ST_STBY: begin
		assert(!o_dma_request);
		assert(!o_dma_reset);
		assert(!s2mm_busy);
		assert(!mm2s_busy);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		end
	ST_TRAN: begin
		assert(!o_dma_request);
		assert(!o_dma_reset);
		assert(!s2mm_busy);
		assert(!mm2s_busy);
		assert(bufcount == 0);
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(!o_tx_busy);
		assert(RCA != 0);
		end
	ST_DATA: begin
		assert(o_dma_dir == D_DEV2HOST);
		assert(!mm2s_busy);
		assert(!o_rx_en);
		assert($stable(o_tx_src));
		if (o_tx_src != S_MEM)
			assert(!o_dma_request);
		case(o_tx_src)
		S_MEM: begin end
		S_SCR: assert(o_cfg_lgblksz == 3);
		S_STATUS: assert(o_cfg_lgblksz == 6);
		default: assert(0);
		endcase
		end
	ST_RCV: begin
		assert(o_dma_dir == D_HOST2DEV);
		assert(!s2mm_busy);
		assert(!o_tx_en);
		end
	ST_PRG: begin
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(o_tx_busy);
		end
	ST_DIS: begin
		assert(!o_tx_en);
		assert(!o_rx_en);
		assert(!o_tx_busy);
		end
	default: assert(0);
	endcase

	always @(posedge i_clk)
	if (!i_reset)
	begin
		assert(bufcount <= 2);
		if (bufcount == 2)
		begin
			assert(r_multiblock);
			assert(!o_dma_req);
			assert(!mm2s_busy);
			assert(o_dma_dir == D_DEV2HOST || o_tx_busy);
		end

		if (o_dma_request)
		begin
			assert(s2mm_busy || mm2s_busy);
			assert(s2mm_busy == (o_dma_dir == D_HOST2DEV));
		end

		if (bufcount > 0 && o_dma_dir == D_DEV2HOST)
			assert(o_tx_en || $past(o_tx_en));

		if (mm2s_busy || s2mm_busy)
		begin
			assert($stable(o_cfg_pp));
			// assert($stable(o_cfg_ds));
			assert($stable(o_cfg_lgblksz));
			assert($stable(o_cfg_width));
			assert(o_dma_request || $stable(o_dma_addr));
		end

		assert(!o_tx_en || !o_rx_en);

		if ($past(o_tx_en))
		begin
			if ($past(i_tx_done))
			begin
				assert(!o_tx_en);
			end else if (!$past(i_cmd_valid)
				|| $past((i_cmd != 6'h0) &&(i_cmd != 6'd12)))
			begin
				assert(o_tx_en);
			end
		end

		if ($past(o_rx_en))
		begin
			if ($past(i_rx_good || i_rx_err))
			begin
				assert(!o_rx_en);
			end else if (!$past(i_cmd_valid)
				|| $past((i_cmd != 6'h0) &&(i_cmd != 6'd12)))
			begin
				assert(o_rx_en);
			end
		end

		if ($past(o_rx_en && (i_rx_good || i_rx_err)))
			assert(!o_rx_en);
	end

	// }}}
`endif
// }}}
endmodule
