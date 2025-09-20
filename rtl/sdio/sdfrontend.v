////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdfrontend.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	This is the "front-end" for the SDIO controller.  It's designed
//		to support all modes up to HS400 if OPT_SERDES is enabled,
//	or just the backwards compatibility modes (up to 50MHz) if not.
//
// Status:
//	OPT_DDR=0	(Without IDDR and ODDR hardware elements)
//		Verified in H/W for 1, 4, and 8b.  (Not yet for DDR)
//		No support for data strobe.
//	OPT_DDR=1	(With IDDR/ODDR hardware elements, but no SERDES)
//		Verified in H/W for 1 and 4b.  (Not yet for DDR)
//		No support for data strobe.
//	OPT_SERDES=1
//		Verified in H/W for 1b, 4b, and 8b.
//		Hardware data strobe support verification still pending,
//		although all simulation tests pass.
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
module	sdfrontend #(
		// {{{
		parameter [0:0]	OPT_SERDES = 1'b0,
		parameter [0:0]	OPT_DDR = 1'b0,
		parameter [0:0]	OPT_DS = OPT_SERDES,
		parameter [0:0]	OPT_COLLISION = 1'b0,
		parameter [0:0]	OPT_CRCTOKEN = 1'b1,
		parameter 	BUSY_CLOCKS = 4,
		parameter	HWBIAS = (OPT_SERDES ? 7 : 0),
		parameter	NUMIO = 8
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_hsclk,
		// Configuration
		input	wire		i_reset,
		input	wire		i_cfg_ddr,
		input	wire		i_cfg_ds, i_cfg_dscmd,
		input	wire	[4:0]	i_sample_shift,
		// Control signals
		// Tx path
		// {{{
		// MSB "first" incoming data.
		input	wire	[7:0]	i_sdclk,
		// Verilator lint_off SYNCASYNCNET
		input	wire		i_cmd_en, i_cmd_tristate,
		// Verilator lint_on  SYNCASYNCNET
		input	wire	[1:0]	i_cmd_data,
		//
		input	wire		i_data_en, i_rx_en, i_data_tristate,
		input	wire	[31:0]	i_tx_data,
		// }}}
		output	wire		o_data_busy,
		// Synchronous Rx path
		// {{{
		output	wire	[1:0]	o_cmd_strb,
		output	wire	[1:0]	o_cmd_data,
		output	wire		o_cmd_collision,
		//
		output	wire		o_crcack, o_crcnak,
		//
		output	wire	[1:0]	o_rx_strb,
		output	wire	[15:0]	o_rx_data,
		// }}}
		// Async Rx path
		// {{{
		output	wire		MAC_VALID,
		output	wire	[1:0]	MAC_DATA,
		output	wire		MAD_VALID,
		output	wire	[31:0]	MAD_DATA,
		// output	wire		MAD_LAST,
		// }}}
		// I/O ports
		// {{{
		output	wire			o_ck,
		input	wire			i_ds,
		//
`ifdef	VERILATOR
		output	wire			io_cmd_tristate,
		output	wire			o_cmd,
		input	wire			i_cmd,
		//
		output	wire	[NUMIO-1:0]	io_dat_tristate,
		output	wire	[NUMIO-1:0]	o_dat,
		input	wire	[NUMIO-1:0]	i_dat,
`else
		inout	wire			io_cmd,
		inout	wire	[NUMIO-1:0]	io_dat,
`endif
		// }}}
		output	wire	[31:0]	o_debug
		// }}}
	);

	// Local declarations
	// {{{
	genvar		gk;
	reg		dat0_busy, wait_for_busy;
	reg	[$clog2(BUSY_CLOCKS+1)-1:0]	busy_count;
	wire		raw_cmd;
	wire	[NUMIO-1:0]	raw_iodat;
	wire		w_cmd_collision;
`ifndef	VERILATOR
	wire			io_cmd_tristate, i_cmd, o_cmd;
	wire	[NUMIO-1:0]	io_dat_tristate, i_dat, o_dat;
`endif
	reg		last_ck, sync_ack, sync_nak;
	wire	[7:0]	w_pedges, next_pedge, next_nedge, next_dedge;
	wire		async_ack, async_nak;
	reg	[4:0]	acknak_sreg;

	reg	ackd, ck_ack, ck_nak, pipe_ack, pipe_nak;
	// }}}

	// Common setup
	// {{{
	initial	last_ck = 1'b0;
	always @(posedge i_clk)
		last_ck <= i_sdclk[0];

	assign	next_pedge = ~{ last_ck, i_sdclk[7:1] } &  i_sdclk[7:0];
	assign	next_nedge =  { last_ck, i_sdclk[7:1] } & ~i_sdclk[7:0];
	assign	next_dedge = next_pedge | (i_cfg_ddr ? next_nedge : 8'h0);
	// }}}
	generate if (!OPT_SERDES && !OPT_DDR)
	begin : GEN_NO_SERDES
		// {{{
		// This is sort of the "No-PHY" option.  Maximum speed, when
		// using this option, is the incoming clock speed/2.  Without
		// SERDES support, there's no support for the DS (data strobe)
		// pin either.  Think of this as a compatibility mode.
		//
		// Fastest clock supported = incoming clock speed / 2.  That's
		// also the fastest clock supported *if you get lucky*.  It's
		// possible that there won't be enough sub-clock resolution
		// to land close enough to the middle of the eye at this
		// frequency.
		//
		reg		resp_started;
		reg	[1:0]	io_started;
		reg		r_cmd_data, r_cmd_strb, r_rx_strb;
		reg	[7:0]	r_rx_data;
		wire	[HWBIAS+3:0]	wide_dedge, wide_pedge, wide_cmdedge;
		reg	[HWBIAS+3:0]	ck_sreg, pck_sreg, ck_psreg;
		reg		sample_ck, cmd_sample_ck, sample_pck;

		assign	o_ck = i_sdclk[7];

		assign	io_cmd_tristate = i_cmd_tristate || o_cmd_collision;
		assign	o_cmd = i_cmd_data[1];
		assign	raw_cmd = i_cmd;

		assign	o_dat = i_tx_data[24 +: NUMIO];

		assign	io_dat_tristate = {(NUMIO){i_data_tristate}};

		assign	w_cmd_collision = OPT_COLLISION && io_cmd_tristate
				&& i_cmd_en && !i_cmd && |next_pedge;

		if (OPT_COLLISION)
		begin : GEN_COLLISION
			reg	r_collision;

			always @(posedge i_clk)
			if (i_reset || !i_cmd_en)
				r_collision <= 1'b0;
			else if (w_cmd_collision)
				r_collision <= 1'b1;

			assign	o_cmd_collision = r_collision;
		end else begin : NO_COLLISION
			assign	o_cmd_collision = 1'b0;

			// Verilator lint_off UNUSED
			wire	unused_collision;
			assign	unused_collision = &{ 1'b0, w_cmd_collision };
			// Verilator lint_on  UNUSED
		end

		// sample_ck
		// {{{
		assign	wide_dedge = { ck_sreg[HWBIAS+2:0], |next_dedge };

		initial	ck_sreg = 0;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
			ck_sreg <= 0;
		else
			ck_sreg <= wide_dedge[HWBIAS+2:0];

		initial	sample_ck = 0;
		always @(*)
		if (i_data_en)
			sample_ck = 0;
		else
			// Verilator lint_off WIDTH
			sample_ck = wide_dedge[HWBIAS +: 4] >> i_sample_shift[4:3];
			// Verilator lint_on  WIDTH
		// }}}

		// sample_pck
		// {{{
		assign	wide_pedge = { ck_psreg[HWBIAS+2:0], |next_pedge };

		initial	ck_sreg = 0;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
			ck_psreg <= 0;
		else
			ck_psreg <= wide_pedge[HWBIAS+2:0];

		initial	sample_ck = 0;
		always @(*)
		if (i_data_en)
			sample_pck = 0;
		else
			// Verilator lint_off WIDTH
			sample_pck = wide_pedge[HWBIAS +: 4] >> i_sample_shift[4:3];
			// Verilator lint_on  WIDTH
		// }}}

		// cmd_sample_ck: When do we sample the command line?
		// {{{
		assign	wide_cmdedge = { pck_sreg[HWBIAS+2:0], |next_pedge };

		always @(posedge i_clk)
		if (i_reset || i_cmd_en)
			pck_sreg <= 0;
		else
			pck_sreg <= wide_cmdedge[HWBIAS+2:0];

		always @(*)
		if (i_cmd_en)
			cmd_sample_ck = 0;
		else
			// Verilator lint_off WIDTH
			cmd_sample_ck = wide_cmdedge[HWBIAS +: 4] >> i_sample_shift[4:3];
			// Verilator lint_on  WIDTH
		// }}}

		assign	raw_iodat = i_dat;

		// CRC TOKEN detection
		// {{{
		always @(posedge i_clk)
		if(i_reset || i_data_en || i_cfg_ds || !OPT_CRCTOKEN)
			acknak_sreg <= -1;
		else if (acknak_sreg[4] && sample_pck)
			acknak_sreg <= { acknak_sreg[3:0], raw_iodat[0] };

		initial	{ sync_ack, sync_nak } = 2'b00;
		always @(posedge i_clk)
		if(i_reset || i_data_en || i_cfg_ds || !OPT_CRCTOKEN)
		begin
			sync_ack <= 1'b0;
			sync_nak <= 1'b0;
		end else begin
			sync_ack <= (acknak_sreg == 5'b00101);
			sync_nak <= (acknak_sreg == 5'b01011);
		end
		// }}}

		always @(posedge i_clk)
		if (i_reset || i_cmd_en || i_cfg_dscmd)
			resp_started <= 1'b0;
		else if (!i_cmd && cmd_sample_ck)
			resp_started <= 1'b1;

		always @(posedge i_clk)
		if (i_reset || i_data_en || !i_rx_en || i_cfg_ds)
			io_started <= 2'b0;
		else if (i_cfg_ddr && io_started[0] && sample_ck)
			io_started <= 2'b11;
		else if (!i_dat[0] && sample_pck)
			io_started <= (i_cfg_ddr) ? 2'b01 : 2'b11;

		// dat0_busy, wait_for_busy
		// {{{
		/*
		if (OPT_CRCTOKEN)
		begin : GEN_WAIT_ON_TOKEN
			reg		wait_for_token;
			reg	[3:0]	token_sreg;
			always @(posedge i_clk)
			if (i_reset || i_data_en || i_rx_en || !wait_for_token)
				token_sreg <= -1;
			else if (sample_pck)
				token_sreg <= { token_sreg[2:0], i_dat[0] };

			always @(posedge i_clk)
			if (i_reset || i_rx_en)
				wait_for_token <= 1'b0;
			else if (data_en)
				wait_for_token <= 1'b1;
			else if (!token_sreg[3])
				wait_for_token <= 1'b0;

			always @(posedge i_clk)
			if (i_reset || !wait_for_token)
				{ r_crcnak, r_crcack } <= 2'b00;

		end else begin
		end
		*/

		// busy_count: SD Clock cycles to wait before busy asserted
		// {{{
		initial	busy_count = 0;
		always @(posedge i_clk)
		if (i_reset || i_cmd_en || i_data_en)
			busy_count <= BUSY_CLOCKS;
		else if (sample_pck && busy_count > 0)
			busy_count <= busy_count-1;
		// }}}

		initial	{ dat0_busy, wait_for_busy } = 2'b01;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
		begin
			// *MUST* clear busy on i_data_en, else we'd overwrite
			// the busy bit anyway by transmitting
			dat0_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (dat0_busy && !i_dat[0] && !wait_for_busy)
		begin
			// If busy is already set, keep it set until D0 rises
			dat0_busy <= 1'b1;
			// wait_for_busy <= 1'b0;
		end else if (i_cmd_en)
		begin
			dat0_busy <= 1'b0;	// Should already be zero
			wait_for_busy <= 1'b1;
		end else if (wait_for_busy)
		begin
			dat0_busy <= 1'b1;
			wait_for_busy <= (busy_count > 1);
		end else if (i_dat[0])
			dat0_busy <= 1'b0;

		assign	o_data_busy = dat0_busy;
		// }}}

		always @(posedge i_clk)
		begin
			if (i_cmd_en || !cmd_sample_ck || i_cfg_dscmd)
				r_cmd_strb <= 1'b0;
			else if (!i_cmd || resp_started)
				r_cmd_strb <= 1'b1;
			else
				r_cmd_strb <= 1'b0;

			if (i_data_en || sample_ck == 0 || i_cfg_ds)
				r_rx_strb <= 1'b0;
			else if (io_started[1])
				r_rx_strb <= 1'b1;
			else
				r_rx_strb <= 1'b0;

			if (cmd_sample_ck)
				r_cmd_data <= i_cmd;
			if (sample_ck)
			begin
				r_rx_data <= 0;
				r_rx_data[NUMIO-1:0] <= i_dat;
			end
		end

		assign	o_cmd_strb = { r_cmd_strb, 1'b0 };
		assign	o_cmd_data = { r_cmd_data, 1'b0 };
		assign	o_rx_strb  = { r_rx_strb, 1'b0 };
		assign	o_rx_data  = { r_rx_data, 8'h0 };

		reg	[7:0]	w_out;
		always @(*)
		begin
			w_out = 0;
			w_out[NUMIO-1:0] = i_dat;
		end

		assign	o_debug = {
				i_cmd_en || i_data_en,
				5'h0,
				i_sdclk[7], 1'b0,
				i_cmd_en, i_cmd_data[1], i_cmd,
					(io_cmd_tristate) ? i_cmd: o_cmd,//w_cmd
					r_cmd_strb, r_cmd_data,		// 2b
				i_data_en, r_rx_strb, r_rx_data,	// 10b
				//
				((i_data_en) ? i_tx_data[31:24] : w_out) // 8b
				};

		// Keep Verilator happy
		// {{{
		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused_no_serdes;
		assign	unused_no_serdes = &{ 1'b0,
				i_cfg_ds, i_ds,
				i_sdclk[6:0], i_tx_data[23:0],
				i_cmd_data[0], i_hsclk,i_sample_shift
				};
		// Verilator lint_on  UNUSED
		// Verilator coverage_on
		// }}}
		// }}}
	end else if (!OPT_SERDES && OPT_DDR)
	begin : GEN_IODDR_IO
		// {{{
		// Notes:
		// {{{
		// The idea is, if we only have DDR elements and no SERDES
		// elements, can we do better than with just IOs?
		//
		// The answer is, Yes.  Even though we aren't going to run at
		// 2x the clock speed w/o OPT_SERDES, we can output a DDR clk,
		// and we can also access sub-sample timing via IDDR elements.
		// Even in DDR mode, however, there will be no possibility of
		// two outputs per clock.
		//
		// Fastest clock supported = incoming clock speed
		//	Practically, you won't be likely to achieve this unless
		//	you get really lucky, but it is technically the fastest
		//	speed this version supports.
		// A more realistic speed will be the incoming clock speed / 2,
		//	and done with more reliability than the non-DDR mode.
		// }}}

		// Local declarations
		// {{{
		wire	[1:0]	w_cmd;
		wire	[15:0]	w_dat;

		reg	[5:0]	ck_sreg;
		reg	[6:0]	pck_sreg, ck_psreg;
		reg	[1:0]	sample_ck, cmd_sample_ck, sample_pck;
		reg		resp_started, r_last_cmd_enabled,
				r_cmd_strb, r_cmd_data, r_rx_strb;
		wire	[1:0]	my_cmd_data;
		reg	[1:0]	io_started;
		reg	[7:0]	r_rx_data;
		reg	[1:0]	busy_delay;
		wire	[HWBIAS+7:0]	wide_pedge, wide_dedge, wide_cmdedge;
		// Verilator lint_off UNUSED
		wire		io_clk_tristate, ign_clk;
		assign		ign_clk = o_ck;
		// Verilator lint_on  UNUSED
		// }}}

		// Clock
		// {{{
		xsdddr #(.OPT_BIDIR(1'b0))
		u_clk_oddr(
			.i_clk(i_clk), .i_en(1'b1),
			.i_data({ i_sdclk[7], i_sdclk[3] }),
			.io_pin_tristate(io_clk_tristate),
			.o_pin(o_ck),
			.i_pin(ign_clk),
			// Verilator lint_off PINCONNECTEMPTY
			.o_mine(),
			.o_wide()
			// Verilator lint_on  PINCONNECTEMPTY
		);
		// }}}

		// CMD
		// {{{
		always @(posedge i_clk)
			r_last_cmd_enabled <= i_cmd_en;

		xsdddr #(.OPT_BIDIR(1'b1))
		u_cmd_ddr(
			.i_clk(i_clk),
			.i_en(!i_cmd_tristate && !o_cmd_collision),
			.i_data({(2){ i_reset || i_cmd_data[1] }}),
			.io_pin_tristate(io_cmd_tristate),
			.o_pin(o_cmd),
			.i_pin(i_cmd),
			.o_mine(my_cmd_data),
			.o_wide(w_cmd)
		);

		assign	raw_cmd = i_cmd;

		assign	w_cmd_collision = OPT_COLLISION && i_cmd_en
				&& |(my_cmd_data & ~w_cmd);

		if (OPT_COLLISION)
		begin : GEN_COLLISION
			reg	r_collision;

			always @(posedge i_clk)
			if (i_reset || !i_cmd_en)
				r_collision <= 1'b0;
			else if (w_cmd_collision)
				r_collision <= 1'b1;

			assign	o_cmd_collision = r_collision;
		end else begin : NO_COLLISION
			assign	o_cmd_collision = 1'b0;

			// Verilator lint_off UNUSED
			wire	unused_collision;
			assign	unused_collision = &{ 1'b0, w_cmd_collision };
			// Verilator lint_on  UNUSED
		end
		// }}}

		// DATA
		// {{{
		for(gk=0; gk<NUMIO; gk=gk+1)
		begin : DRIVE_DDR_IO

			xsdddr #(.OPT_BIDIR(1'b1))
			u_dat_ddr(
				.i_clk(i_clk),
				.i_en(!i_data_tristate),
				.i_data({(2){ i_reset || i_tx_data[24+gk] }}),
				.io_pin_tristate(io_dat_tristate[gk]),
				.o_pin(o_dat[gk]),
				.i_pin(i_dat[gk]),
				// Verilator lint_off PINCONNECTEMPTY
				.o_mine(),
				// Verilator lint_on  PINCONNECTEMPTY
				.o_wide({ w_dat[gk+8], w_dat[gk] })
			);

			assign	raw_iodat[gk] = i_dat[gk];

		end for(gk=NUMIO; gk<8; gk=gk+1)
		begin : NO_DDR_IO
			assign	{ w_dat[8+gk], w_dat[gk] } = 2'b00;
		end
		// }}}

		// sample_ck
		// {{{
		assign	wide_dedge = { ck_sreg[HWBIAS+5:0], |next_dedge[7:4], |next_dedge[3:0] };

		initial	ck_sreg = 0;
		always @(posedge i_clk)
		if (i_data_en || i_cfg_ds)
			ck_sreg <= 0;
		else
			ck_sreg <= wide_dedge[HWBIAS+5:0];

		initial	sample_ck = 0;
		always @(*)
		if (i_data_en || !i_rx_en || i_cfg_ds)
			sample_ck = 0;
		else
			// Verilator lint_off WIDTH
			sample_ck = wide_dedge[HWBIAS +: 8] >> i_sample_shift[4:2];
			// Verilator lint_on  WIDTH
		// }}}

		// sample_pck -- positive edge data sampl clock
		// {{{
		assign	wide_pedge = { ck_psreg[HWBIAS+5:0], |next_pedge[7:4], |next_pedge[3:0] };

		initial	ck_psreg = 0;
		always @(posedge i_clk)
		if (i_data_en)
			ck_psreg <= 0;
		else
			ck_psreg <= wide_pedge[HWBIAS+5:0];

		initial	sample_pck = 0;
		always @(*)
		if (i_data_en)
			sample_pck = 0;
		else
			// Verilator lint_off WIDTH
			sample_pck = wide_pedge[HWBIAS +: 8] >> i_sample_shift[4:2];
			// Verilator lint_on  WIDTH
		// }}}

		// cmd_sample_ck: When do we sample the command line?
		// {{{
		assign	wide_cmdedge = { pck_sreg[HWBIAS+5:0], |next_pedge[7:4], |next_pedge[3:0] };

		always @(posedge i_clk)
		if (i_reset || i_cmd_en || r_last_cmd_enabled || i_cfg_dscmd)
			pck_sreg <= 0;
		else
			pck_sreg <= wide_cmdedge[HWBIAS + 5:0];

		always @(*)
		if (i_cmd_en || r_last_cmd_enabled || i_cfg_dscmd)
			cmd_sample_ck = 0;
		else
			// Verilator lint_off WIDTH
			cmd_sample_ck = wide_cmdedge[HWBIAS +: 8] >> i_sample_shift[4:2];
			// Verilator lint_on  WIDTH
		// }}}

		// CRC TOKEN detection
		// {{{
		always @(posedge i_clk)
		if(i_reset || i_data_en || i_cfg_ds || !OPT_CRCTOKEN)
			acknak_sreg <= -1;
		else if (acknak_sreg[4])
		begin
			if (sample_pck[1:0] == 2'b11 && acknak_sreg[3])
				acknak_sreg <= { acknak_sreg[2:0], w_dat[8], w_dat[0] };
			else if (sample_pck[1])
				acknak_sreg <= { acknak_sreg[3:0], w_dat[8] };
			else if (sample_pck[0])
				acknak_sreg <= { acknak_sreg[3:0], w_dat[0] };
		end

		initial	{ sync_ack, sync_nak } = 2'b00;
		always @(posedge i_clk)
		if(i_reset || i_data_en || i_cfg_ds || !OPT_CRCTOKEN)
		begin
			sync_ack <= 1'b0;
			sync_nak <= 1'b0;
		end else begin
			sync_ack <= (acknak_sreg == 5'b00101);
			sync_nak <= (acknak_sreg == 5'b01011);
		end
		// }}}


		always @(posedge i_clk)
		if (i_reset || i_cmd_en || r_last_cmd_enabled || i_cfg_dscmd)
			resp_started <= 1'b0;
		else if ((cmd_sample_ck != 0) && (cmd_sample_ck & w_cmd)==0)
			resp_started <= 1'b1;

		always @(posedge i_clk)
		if (i_reset || i_data_en || !i_rx_en || i_cfg_ds)
			io_started <= 2'b0;
		else if (sample_pck != 0
				&& ((sample_pck & { w_dat[8], w_dat[0] }) == 0))
		begin
			io_started[0] <= 1'b1;
			if (!i_cfg_ddr)
				io_started[1] <= 1'b1;
			else if (sample_pck[1] && sample_ck[0])
				io_started[1] <= 1'b1;
		end else if (io_started == 2'b01 && sample_ck != 0)
			io_started <= 2'b11;

		// dat0_busy, wait_for_busy, busy_delay
		// {{{
		initial	busy_count = (OPT_CRCTOKEN) ? 3'h0 : 3'h4;
		always @(posedge i_clk)
		if (i_reset || i_cmd_en || i_data_en)
			// Clock periods to wait until busy is active
			busy_count <= BUSY_CLOCKS;
		else if (sample_pck != 0 && busy_count > 0)
			busy_count <= busy_count - 1;

		initial	busy_delay = -1;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
			// System clock cycles to wait until busy can be read
			busy_delay <= -1;
		else if (busy_delay != 0)
			busy_delay <= busy_delay - 1;

		initial	{ dat0_busy, wait_for_busy } = 2'b01;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
		begin
			dat0_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (dat0_busy && !wait_for_busy &&
			((sample_pck == 0)
			|| (sample_pck & {w_dat[8],w_dat[0]})!=sample_pck))
		begin
			// Still busy ...
			dat0_busy <= 1'b1;
			wait_for_busy <= 1'b0;
		end else if (i_cmd_en)
		begin
			dat0_busy <= 1'b0;	// Should already be zero
			wait_for_busy <= 1'b1;
		end else if (wait_for_busy)
		begin
			dat0_busy <= 1'b1;
			wait_for_busy <= (busy_delay > 0) || (busy_count > 1);
		end else if ((sample_pck != 0)
				&& (sample_pck & {w_dat[8],w_dat[0]})!=2'b0)
			dat0_busy <= 1'b0;

		assign	o_data_busy = dat0_busy;
		// }}}

		always @(posedge i_clk)
		begin

			// The command response
			// {{{
			if (i_reset || i_cmd_en || cmd_sample_ck == 0 || i_cfg_dscmd)
			begin
				r_cmd_strb <= 1'b0;
				// r_cmd_data <= r_cmd_data;
			end else if (resp_started)
			begin
				r_cmd_strb <= 1'b1;
				r_cmd_data <= |(cmd_sample_ck & w_cmd);
			end else if ((cmd_sample_ck[1] && !w_cmd[1])
					||(cmd_sample_ck[0] && !w_cmd[0]))
			begin
				r_cmd_strb <= 1'b1;
				r_cmd_data <= 1'b0;
			end else
				r_cmd_strb <= 1'b0;
			// }}}

			// The data response
			// {{{
			if (i_data_en || sample_ck == 0 || i_cfg_ds)
				r_rx_strb <= 1'b0;
			else if (io_started[1])
				r_rx_strb <= 1'b1;
			else
				r_rx_strb <= 1'b0;
			// }}}

			if (sample_ck[1])
				r_rx_data <= w_dat[15:8];
			else
				r_rx_data <= w_dat[7:0];
		end

		assign	o_cmd_strb = { r_cmd_strb, 1'b0 };
		assign	o_cmd_data = { r_cmd_data, 1'b0 };
		assign	o_rx_strb  = { r_rx_strb, 1'b0 };
		assign	o_rx_data  = { r_rx_data, 8'h0 };

		reg	[7:0]	w_out;
		always @(*)
		begin
			w_out = 0;
			w_out[NUMIO-1:0] = w_dat[8 +: NUMIO]& w_dat[0 +: NUMIO];
		end

		assign	o_debug = {
				i_cmd_en || i_data_en, 2'h0, i_rx_en,
				sample_ck, i_sdclk[7], i_sdclk[3],
				i_cmd_en, i_cmd_data[1:0],
					(&w_cmd), r_cmd_strb, r_cmd_data,
				i_data_en, r_rx_strb, r_rx_data,
				//
				((i_data_en) ? i_tx_data[31:24] : w_out)
				};


		// Keep Verilator happy
		// {{{
		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused_ddr;
		assign	unused_ddr = &{ 1'b0, i_hsclk,
				i_cfg_ds, i_ds, i_tx_data[23:0],
				i_sdclk[6:4], i_sdclk[2:0],
				i_sample_shift[1:0] };
		// Verilator lint_on  UNUSED
		// Verilator coverage_on
		// }}}
		// }}}
	end else begin : GEN_WIDE_IO
		// {{{
		// Generic PHY handler, designed to support up to HS400.
		// Outputs 4 data periods per incoming clock, and 8 clock values
		// per incoming clock--hence the outgoing clock may have a
		// 90 degree shift from the data.  When dealing with non-DS
		// data, the clock edge is detected on output, and a sample
		// controller decides when to sample it on input.
		//
		// Fastest clock supported = incoming clock speed * 2

		// Local declarations
		// {{{
		reg		r_last_cmd_enabled;
		reg	[1:0]	w_cmd_data;
		reg	[15:0]	r_rx_data;
		wire	[15:0]	w_rx_data;
		// wire	[7:0]	next_ck_sreg, next_ck_psreg;
		reg	[HWBIAS+24:0]	ck_sreg, ck_psreg;
		wire	[7:0]	wide_cmd_data;
		reg	[7:0]	sample_ck, sample_pck;
		reg	[1:0]	r_cmd_data;
		reg		busy_strb;
		reg	[1:0]	r_rx_strb;
		reg	[1:0]	start_io;
		reg	[1:0]	io_started;
		reg		resp_started;
		reg	[1:0]	r_cmd_strb;
		reg	[HWBIAS+24:0]	pck_sreg;
		reg	[7:0]	cmd_sample_ck;
		wire		busy_pin;
		reg	[1:0]	busy_delay, itok;
		wire	[HWBIAS+31:0]	wide_pedge, wide_dedge, wide_cmdedge;
		// Verilator lint_off UNUSED
		wire	[7:0]	my_cmd_data;
		// Verilator lint_on  UNUSED
		reg		r_cmd_tristate, r_data_tristate;
		// }}}

		// Clock
		// {{{
		// Verilator lint_off UNUSED
		wire		io_clk_tristate, ign_clk_raw;
		wire	[7:0]	ign_clk_mine, ign_clk_wide;
		// Verilator lint_on  UNUSED

		xsdserdes8x #(.OPT_BIDIR(1'b0))
		u_clk_oserdes(
			.i_clk(i_clk),
			.i_hsclk(i_hsclk),
			.i_reset(i_reset),
			.i_en(1'b1),
			.i_data(i_sdclk),
			.io_tristate(io_clk_tristate),
			.o_pin(o_ck),
			.i_pin(1'b0),
			// Verilator lint_off PINCONNECTEMPTY
			.o_raw(ign_clk_raw), .o_mine(ign_clk_mine),
			.o_wide(ign_clk_wide)
			// Verilator lint_on  PINCONNECTEMPTY
		);
		// }}}

		// assign	next_ck_sreg  = (i_data_en) ? 8'h0 : next_dedge;
		// assign	next_ck_psreg = (i_data_en) ? 8'h0 : next_pedge;

		// sample_ck
		// {{{
		assign	wide_dedge = { ck_sreg[HWBIAS+23:0], next_dedge };

		// We use this for busy detection, as well as reception
		always @(posedge i_clk)
		if (i_reset || i_data_en)
				//||(!wait_for_busy && (!i_rx_en || i_cfg_ds)))
			ck_sreg <= 0;
		else
			ck_sreg <= wide_dedge[HWBIAS+23:0];

		initial	sample_ck = 0;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
				//||(!wait_for_busy && (!i_rx_en || i_cfg_ds)))
			sample_ck <= 0;
		else
			// Verilator lint_off WIDTH
			sample_ck <= wide_dedge[HWBIAS +: 32] >> i_sample_shift;
			// Verilator lint_on  WIDTH
		// }}}

		// sample_pck
		// {{{
		assign	wide_pedge = { ck_psreg[HWBIAS+23:0], next_pedge };

		always @(posedge i_clk)
		if (i_reset || i_data_en)
			ck_psreg <= 0;
		else
			ck_psreg <= wide_pedge[HWBIAS + 23:0];

		initial	sample_pck = 0;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
			sample_pck <= 0;
		else
			// Verilator lint_off WIDTH
			sample_pck <= wide_pedge[HWBIAS +: 32] >> i_sample_shift;
			// Verilator lint_on  WIDTH
		// }}}

		always @(posedge i_clk)
			r_data_tristate <= i_data_tristate;

		for(gk=0; gk<NUMIO; gk=gk+1)
		begin : GEN_WIDE_DATIO
			// {{{
			reg	[7:0]	out_pin;
			wire	[7:0]	in_pin;
			integer		ik;
			reg	[1:0]	lcl_data;

			always @(*)
			for(ik=0; ik<4; ik=ik+1)
				out_pin[ik*2 +: 2] = {(2){i_tx_data[ik*8+gk]}};

			xsdserdes8x #(
				.OPT_BIDIR(1'b1)
			) io_serdes(
				.i_clk(i_clk),
				.i_hsclk(i_hsclk),
				.i_reset(i_reset),
				.i_en(!r_data_tristate),
				.i_data(out_pin),
				.io_tristate(io_dat_tristate[gk]),
				.o_pin(o_dat[gk]),
				.i_pin(i_dat[gk]),
				// Verilator lint_off PINCONNECTEMPTY
				.o_mine(),
				// Verilator lint_on  PINCONNECTEMPTY
				.o_raw(raw_iodat[gk]), .o_wide(in_pin)
			);

			if (gk == 0)
			begin : GEN_START_SIGNAL
				always @(*)
				begin
					start_io[1] = (|sample_pck[7:4])
						&&(0 == (sample_pck[7:4]&in_pin[7:4]));
					start_io[0] = (|sample_pck[3:0])
						&&(0 == (sample_pck[3:0]&in_pin[3:0]));

					itok[1] = |(sample_pck[7:4] & in_pin[7:4]);
					itok[0] = |(sample_pck[3:0] & in_pin[3:0]);
				end

				assign	busy_pin = !in_pin[0];
			end

			always @(*)
			begin
				lcl_data[1] = |(sample_ck[7:4]&in_pin[7:4]);
				lcl_data[0] = |(sample_ck[3:0]&in_pin[3:0]);
			end

			assign	w_rx_data[8+gk] = lcl_data[1];
			assign	w_rx_data[  gk] = lcl_data[0];
			// }}}
		end

		for(gk=NUMIO; gk<8; gk=gk+1)
		begin : NULL_DATIO
			// {{{
			assign	w_rx_data[8+gk] = 1'b1;
			assign	w_rx_data[  gk] = 1'b1;

			// Keep Verilator happy
			// {{{
			// Verilator coverage_off
			// Verilator lint_off UNUSED
			wire	unused_outputs;
			assign	unused_outputs = &{ 1'b0 }; // , out_pin
			// Verilator lint_on  UNUSED
			// Verilator coverage_on
			// }}}
			// }}}
		end

		// o_rx_strb, o_rx_data
		// {{{
		always @(posedge i_clk)
		if (i_reset || i_data_en || !i_rx_en || i_cfg_ds)
			io_started <= 2'b0;
		else if (!io_started[1])
		begin
			if (start_io[1] && (|sample_ck[3:0]))
				io_started <= 2'b11;
			else if (io_started[0])
				io_started[1] <= |sample_ck;
			else begin // if (!io_started[0] && start_io[0])
				io_started[0] <= |start_io;
				if (!i_cfg_ddr)
					io_started[1] <= |start_io;
			end
		end

		always @(posedge i_clk)
		if (i_reset || i_cfg_ds || !i_rx_en || i_data_en)
			r_rx_strb <= 2'b0;
		else if (sample_ck == 0)
			r_rx_strb <= 2'b0;
		else if (io_started == 0
				&& (|sample_pck[7:4]) && (|sample_pck[3:0])
				&& start_io[1])
		begin // Two clocks received (200MHz), one is the start clock
			r_rx_strb <= 2'b10;
		end else if (io_started[1])
		begin // The full start clock has been received
			r_rx_strb[1] <= (|sample_ck);
			r_rx_strb[0] <= (|sample_ck[7:4]) && (|sample_ck[3:0]);
		end else if (io_started[0]&& (|sample_ck[7:4])) // PEDGE+NEDGE
		begin // past PEDGE, this clock has the NEGEDGE on the left
			// The second clock edge is received on the left, first
			// data clock on the right.  (100MHz clock)
			//
			// io_started == 2'b10 will *ONLY* happen in cfg_ddr
			//
			r_rx_strb[1] <= (|sample_ck[3:0]);
			r_rx_strb[0] <= 1'b0;
		end else
			r_rx_strb <= 2'b0;

		always @(posedge i_clk)
		if (i_reset || i_cfg_ds)
			r_rx_data <= 16'h0;
		else begin
			if (io_started[1] && |sample_ck[7:4])
				r_rx_data[15:8] <= w_rx_data[15:8];
			else
				r_rx_data[15:8] <= w_rx_data[ 7:0];

			r_rx_data[7:0] <= w_rx_data[7:0];
		end

		assign	o_rx_strb = r_rx_strb;
		assign	o_rx_data = r_rx_data;
		// }}}

		// busy_strb
		always @(*)
			busy_strb = (|sample_pck);

		// o_data_busy, dat0_busy, wait_for_busy, busy_delay
		// {{{

		// busy_count: SD clock cycles to wait before busy is asserted
		// {{{
		always @(posedge i_clk)
		if (i_reset || i_cmd_en || i_data_en)
			busy_count <= BUSY_CLOCKS;
		else if (busy_strb != 0 && busy_count > 0)
			busy_count <= busy_count - 1;
		// }}}

		// busy_delay
		// {{{
		// System clock cycles to wait until busy can be read
		always @(posedge i_clk)
		if (i_reset || i_data_en)
			busy_delay <= -1;
		else if (busy_delay != 0)
			busy_delay <= busy_delay - 1;
		// }}}

		initial	{ dat0_busy, wait_for_busy } = 2'b01;
		always @(posedge i_clk)
		if (i_reset || i_data_en)
		begin
			dat0_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (dat0_busy && !wait_for_busy && busy_pin)
		begin
			// dat0_busy <= 1'b1;
		end else if (i_cmd_en)
		begin
			dat0_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (wait_for_busy)
		begin
			dat0_busy <= 1'b1;
			wait_for_busy <= (busy_delay > 0)||(busy_count > 1);
		end else if (!busy_pin)
			// Once busy is released, we don't become busy again
			// until we reset
			dat0_busy <= 1'b0;

		assign	o_data_busy = dat0_busy;
		// }}}

		// CRC TOKEN detection
		// {{{
		localparam	[4:0]	ACK_TOKEN = 5'b00101,
					NAK_TOKEN = 5'b01011;

		always @(posedge i_clk)
		if(i_reset || i_rx_en || i_cfg_ds || !OPT_CRCTOKEN)
			acknak_sreg <= 0;
		else if (i_data_en)
			acknak_sreg <= -1;
		else if (acknak_sreg[4])
		begin
			if ((|sample_pck[7:4] && |sample_pck[3:0])
							&& acknak_sreg[3])
				acknak_sreg <= { acknak_sreg[2:0], itok[1], itok[0] };
			else if (|sample_pck[7:4])
				acknak_sreg <= { acknak_sreg[3:0], itok[1] };
			else if (|sample_pck[3:0])
				acknak_sreg <= { acknak_sreg[3:0], itok[0] };
		end

		initial	{ sync_ack, sync_nak } = 2'b00;
		always @(posedge i_clk)
		if(i_reset || i_data_en || i_cfg_ds || !OPT_CRCTOKEN)
		begin
			sync_ack <= 1'b0;
			sync_nak <= 1'b0;
		end else begin
			sync_ack <= acknak_sreg == 5'b00101;
			sync_nak <= acknak_sreg == 5'b01011;
		end
		// }}}

		////////////////////////////////////////////////////////////////
		//
		// CMD
		// {{{
		always @(posedge i_clk)
			r_cmd_tristate <= i_cmd_tristate;
		always @(posedge i_clk)
			r_last_cmd_enabled <= i_cmd_en;

		assign	wide_cmdedge = { pck_sreg[HWBIAS+23:0], next_pedge };

		always @(posedge i_clk)
		if (i_reset || i_cfg_dscmd || i_cmd_en || !r_cmd_tristate)
			pck_sreg <= 0;
		else
			pck_sreg <= wide_cmdedge[HWBIAS+23:0];

		always @(posedge i_clk)
		if (i_reset || i_cfg_dscmd || i_cmd_en || r_last_cmd_enabled || !r_cmd_tristate)
			cmd_sample_ck <= 0;
		else
			// Verilator lint_off WIDTH
			cmd_sample_ck <= wide_cmdedge[HWBIAS +: 32] >> i_sample_shift;
			// Verilator lint_on  WIDTH

		xsdserdes8x #(
			.OPT_BIDIR(1'b1)
		) cmd_serdes(
			.i_clk(i_clk),
			.i_hsclk(i_hsclk),
			.i_reset(i_reset),
			.i_en(!r_cmd_tristate && !o_cmd_collision),
			.i_data({ {(4){i_cmd_data[1]}}, {(4){i_cmd_data[0]}} }),
			.io_tristate(io_cmd_tristate),
			.o_pin(o_cmd),
			.i_pin(i_cmd),
			.o_mine(my_cmd_data),
			.o_raw(raw_cmd), .o_wide(wide_cmd_data)
		);

		assign	w_cmd_collision = OPT_COLLISION && i_cmd_en
					&& i_cmd_tristate && !i_cfg_dscmd
					&& my_cmd_data[0] != wide_cmd_data[7]
					&& (|next_pedge);

		if (OPT_COLLISION)
		begin : GEN_COLLISION
			reg	r_collision;

			always @(posedge i_clk)
			if (i_reset || !i_cmd_en || i_cfg_dscmd)
				r_collision <= 1'b0;
			else if (w_cmd_collision)
				r_collision <= 1'b1;

			assign	o_cmd_collision = r_collision;
		end else begin : NO_COLLISION
			assign	o_cmd_collision = 1'b0;

			// Verilator lint_off UNUSED
			wire	unused_collision;
			assign	unused_collision = &{ 1'b0, w_cmd_collision };
			// Verilator lint_on  UNUSED
		end

		// resp_started
		// {{{
		always @(posedge i_clk)
		if (i_reset || i_cmd_en || i_cfg_dscmd || !r_cmd_tristate)
			resp_started <= 1'b0;
		else if (((|cmd_sample_ck[7:4])&&((cmd_sample_ck[7:4] & wide_cmd_data[7:4])==0))
			||((|cmd_sample_ck[3:0])&&((cmd_sample_ck[3:0] & wide_cmd_data[3:0])==0)))
			resp_started <= 1'b1;
		// }}}

		// o_cmd_strb
		// {{{
		always @(posedge i_clk)
		if (i_reset || i_cmd_en || i_cfg_dscmd || !r_cmd_tristate)
			r_cmd_strb <= 2'b00;
		else if (resp_started)
		begin
			r_cmd_strb[1] <= (|cmd_sample_ck);
			r_cmd_strb[0] <= (|cmd_sample_ck[7:4])
						&&(|cmd_sample_ck[3:0]);
		end else begin
			r_cmd_strb[1] <= (((|cmd_sample_ck[7:4])&&((cmd_sample_ck[7:4] & wide_cmd_data[7:4])==0))
				||((|cmd_sample_ck[3:0])&&((cmd_sample_ck[3:0] & wide_cmd_data[3:0])==0)));
			r_cmd_strb[0] <= (|cmd_sample_ck[7:4])
				&& ((cmd_sample_ck[7:4] & wide_cmd_data[7:4])==0)
				&& (|cmd_sample_ck[3:0]);
		end

		assign	o_cmd_strb = r_cmd_strb;
		// }}}

		// o_cmd_data
		// {{{
		always @(*)
		begin
			if (resp_started)
			begin
				if (|cmd_sample_ck[7:4])
					w_cmd_data[1] = |(cmd_sample_ck[7:4] & wide_cmd_data[7:4]);
				else
					w_cmd_data[1] = |(cmd_sample_ck[3:0] & wide_cmd_data[3:0]);
			end else begin // if (!resp_started)
				w_cmd_data[1] = 1'b0;
			end

			w_cmd_data[0] = |(cmd_sample_ck[3:0]
						& wide_cmd_data[3:0]);
		end

		always @(posedge i_clk)
			r_cmd_data <= w_cmd_data;

		assign	o_cmd_data = r_cmd_data;
		// }}}

		// }}}

		reg	[31:0]	r_debug;
		reg	[11:0]	r_dbg_timeout;
		reg	[7:0]	r_dbg_cmd_counter;

		always @(posedge i_clk)
		if (i_reset)
			r_dbg_timeout <= 0;
		else if (i_cmd_en != r_debug[27])
			r_dbg_timeout <= 120;
		else if ({ i_rx_en, i_data_en } != r_debug[15:14])
			r_dbg_timeout <= -1;	// 512B * (8b/4IO) * (2clk/IO)
		else if (r_dbg_timeout > 0)
			r_dbg_timeout <= r_dbg_timeout - 1;

		always @(posedge i_clk)
		if (i_reset)
			r_dbg_cmd_counter <= 0;
		else if (i_cmd_en || |(o_cmd_strb & ~o_cmd_data))
			r_dbg_cmd_counter <= 0;
		else if (!r_dbg_cmd_counter[7] && |o_cmd_strb)
			r_dbg_cmd_counter <= r_dbg_cmd_counter + 1;

		always @(posedge i_clk)
		begin
			r_debug <= 32'h0;

			r_debug[27:25] <= { i_cmd_en, i_cmd_tristate,
						i_cmd_data[0] };
			if (!i_cmd_en)
			begin
				r_debug[26] <= wide_cmd_data[7];
				r_debug[25] <= wide_cmd_data[0];
			end

			r_debug[24:20] <= { i_data_tristate, i_tx_data[3:0] };

			if (!r_dbg_cmd_counter[7])
				r_debug[19:18] <= o_cmd_strb;
			if (o_cmd_strb == 0)
				r_debug[17:16] <= r_debug[17:16];
			else
				r_debug[17:16] <= o_cmd_data;

			r_debug[15:14] <= { i_rx_en, i_data_en };
			r_debug[13:12] <= { sync_ack, sync_nak };
			if (i_rx_en && i_cfg_ddr)
				r_debug[13:12] <= { |sample_pck, |sample_ck };
			if (i_rx_en && i_cfg_ddr && !i_data_en)
				r_debug[14] <= ^io_started;

			r_debug[11:10] <= r_debug[11:10];
			if (|sample_pck[7:4])
				r_debug[11] <= itok[1];
			if (|sample_pck[3:0])
				r_debug[10] <= itok[0];

			r_debug[ 7: 0] <= r_debug;
			if (i_rx_en)
				r_debug[ 9: 8] <= o_rx_strb;
			if (o_rx_strb != 0 || o_cmd_strb != 0)
				r_debug[ 7: 0] <= { o_rx_data[11:8], o_rx_data[3:0] };

			if (0 && r_dbg_timeout == 0)
			begin
				r_debug[9:8] <= 2'b00;
				r_debug[19:16] <= 4'hf;
				r_debug[7:0] <= 8'hff;

				r_debug[4:0] <= acknak_sreg;
			end
		end

		assign	o_debug = r_debug;

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_serdes;
		assign	unused_serdes = &{ 1'b0, ign_clk_mine,
				ign_clk_raw, ign_clk_wide,
				w_rx_data[15:9], w_rx_data[7:1] };
		// Verilator lint_on  UNUSED
		// }}}
		// }}}
	end endgenerate
	////////////////////////////////////////////////////////////////////////
	//
	// TX ACK/NAK handling
	// {{{
	always @(posedge i_clk)
	if (i_reset || i_data_en || !i_cfg_ds || !OPT_DS || !OPT_CRCTOKEN)
	begin
		{ ck_ack, pipe_ack } <= 0;
		{ ck_nak, pipe_nak } <= 0;
	end else begin
		{ ck_ack, pipe_ack } <= { pipe_ack, async_ack };
		{ ck_nak, pipe_nak } <= { pipe_nak, async_nak };
	end

	initial	ackd = 0;
	always @(posedge i_clk)
	if (i_reset || i_data_en || !OPT_CRCTOKEN)
	begin
		ackd <= 0;
	end else if (sync_ack || sync_nak || ck_ack || ck_nak)
		ackd <= 1'b1;

	assign	o_crcack = OPT_CRCTOKEN && (sync_ack || ck_ack) && !ackd;
	assign	o_crcnak = OPT_CRCTOKEN && (sync_nak || ck_nak) && !ackd;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Datastrobe support
	// {{{
	////////////////////////////////////////////////////////////////////////
	//

	generate if (OPT_DS && NUMIO == 8)
	begin : GEN_DATASTROBE
		// Notes
		// {{{
		// This *SHOULD* work with OPT_DDR.  However, I have no evidence
		// that the IDDR element can be bypassed, which would be
		// required by the logic below.  If it cannot be bypassed,
		// this logic then makes no sense.
		//
		// Also: The DS (data strobe) needs to be delayed by a quarter
		// clock before entering this module.  Since the clock frequency
		// isn't fully known until run time, it should instead be
		// delayed by a quarter of the maximum clock period that will
		// be used.
		//
		// When working with DS based outputs, an asynchronous FIFO is
		// used to clock incoming data.  Further, when using the
		// asynchronous FIFO, the start of any incoming data is quietly
		// stripped off, guaranteeing that all data will be aligned on
		// the left/MSB sample.
		// }}}

		// Local declarations
		// {{{
		wire		afifo_reset_n, cmd_ds_en;
		reg		af_started_p, af_started_n, acmd_started;
		reg		af_count_p, af_count_n, acmd_count,
				af_waiting;
		wire	[3:0]	ign_afifo_full, afifo_empty;
		wire	[31:0]	af_data;
		wire	[1:0]	acmd_empty, ign_acmd_full;
		wire	[1:0]	af_cmd;
		// }}}

		// Need to keep this from triggering on CRC tokens, which
		//   might also toggle the DS.  Either that, or ... we need
		//   to clear after the CRC tokens.
		assign	afifo_reset_n = i_cfg_ds && !i_data_en && i_rx_en;
		assign	cmd_ds_en = i_cfg_dscmd && !i_cmd_en;

		// Async command port
		// {{{
		// The rule here is that only the positive edges of the
		// data strobe will qualify the CMD pin;
		always @(posedge i_ds or negedge cmd_ds_en)
		if (!cmd_ds_en)
			acmd_started <= 0;
		else if (!raw_cmd)
			acmd_started <= 1;

		always @(posedge i_ds or negedge cmd_ds_en)
		if (!cmd_ds_en)
			acmd_count <= 0;
		else if (acmd_started || !raw_cmd)
			acmd_count <= acmd_count + 1;

		afifo #(
			.LGFIFO(4), .WIDTH(1), .WRITE_ON_POSEDGE(1'b1)
		) u_pcmd_fifo_0 (
			// {{{
			.i_wclk(i_ds), .i_wr_reset_n(cmd_ds_en),
			.i_wr((acmd_started || !raw_cmd)&& acmd_count == 1'b0),
				.i_wr_data(raw_cmd),
			.o_wr_full(ign_acmd_full[0]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(cmd_ds_en),
			.i_rd(acmd_empty == 2'b0), .o_rd_data(af_cmd[1]),
			.o_rd_empty(acmd_empty[0])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(1), .WRITE_ON_POSEDGE(1'b1)
		) u_pcmd_fifo_1 (
			// {{{
			.i_wclk(i_ds), .i_wr_reset_n(cmd_ds_en),
			.i_wr(acmd_count), .i_wr_data(raw_cmd),
			.o_wr_full(ign_acmd_full[1]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(cmd_ds_en),
			.i_rd(acmd_empty == 2'b0), .o_rd_data(af_cmd[0]),
			.o_rd_empty(acmd_empty[1])
			// }}}
		);

		assign	MAC_VALID = (acmd_empty == 2'h0);
		assign	MAC_DATA  = af_cmd;
		// }}}

		// ACK/NAK checking
		// {{{
		if (OPT_CRCTOKEN)
		begin : GEN_ASYNCTOKEN
			wire		acknak_reset;
			reg	[4:0]	atok_sreg;

			assign		acknak_reset = i_reset || i_data_en;

			always @(posedge i_ds or posedge acknak_reset)
			if (acknak_reset)
				atok_sreg <= -1;
			else if (atok_sreg[4])
				atok_sreg <= { atok_sreg, raw_iodat[0] };

			assign	async_ack = (atok_sreg == 5'b00101);
			assign	async_nak = (atok_sreg == 5'b01011);
		end else begin : NO_ASYNCTOKEN
			assign	async_ack = 1'b0;
			assign	async_nak = 1'b0;
		end
		// }}}

		// af_started_*, af_count_*
		// {{{
		always @(posedge i_ds or negedge afifo_reset_n)
		if (!afifo_reset_n)
			af_started_p <= 0;
		else if (raw_iodat[0] == 0)
			af_started_p <= 1;

		always @(posedge i_ds or negedge afifo_reset_n)
		if (!afifo_reset_n)
			af_count_p <= 0;
		else if (af_started_p)
			af_count_p <= af_count_p + 1;

		always @(negedge i_ds or negedge afifo_reset_n)
		if (!afifo_reset_n)
			af_started_n <= 0;
		else if (af_started_p)
			af_started_n <= 1;

		always @(negedge i_ds or negedge afifo_reset_n)
		if (!afifo_reset_n)
			af_count_n <= 0;
		else if (af_started_n)
			af_count_n <= af_count_n + 1;
		// }}}


		afifo #(
			.LGFIFO(4), .WIDTH(NUMIO), .WRITE_ON_POSEDGE(1'b1)
		) u_pedge_fifo_0 (
			// {{{
			.i_wclk(i_ds), .i_wr_reset_n(afifo_reset_n),
			.i_wr(af_started_p && af_count_p == 1'b0),
				.i_wr_data(raw_iodat),
			.o_wr_full(ign_afifo_full[0]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[24 +: NUMIO]),
			.o_rd_empty(afifo_empty[0])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(NUMIO), .WRITE_ON_POSEDGE(1'b0)
		) u_nedge_fifo_1 (
			// {{{
			.i_wclk(i_ds), .i_wr_reset_n(afifo_reset_n),
			.i_wr(af_started_n && af_count_n == 1'b0),
				.i_wr_data(raw_iodat),
			.o_wr_full(ign_afifo_full[1]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[16 +: NUMIO]),
			.o_rd_empty(afifo_empty[1])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(NUMIO), .WRITE_ON_POSEDGE(1'b1)
		) u_pedge_fifo_2 (
			// {{{
			.i_wclk(i_ds), .i_wr_reset_n(afifo_reset_n),
			.i_wr(af_count_p == 1'b1),
				.i_wr_data(raw_iodat),
			.o_wr_full(ign_afifo_full[2]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[8 +: NUMIO]),
			.o_rd_empty(afifo_empty[2])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(NUMIO), .WRITE_ON_POSEDGE(1'b0)
		) u_nedge_fifo_3 (
			// {{{
			.i_wclk(i_ds), .i_wr_reset_n(afifo_reset_n),
			.i_wr(af_count_n == 1'b1),
				.i_wr_data(raw_iodat),
			.o_wr_full(ign_afifo_full[3]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[0 +: NUMIO]),
			.o_rd_empty(afifo_empty[3])
			// }}}
		);

		for(gk=NUMIO; gk<8; gk=gk+1)
		begin : ASSIGN_UNUSED_DATA
			// {{{
			// Technically, the data strobe is only permitted
			// if/when we are using all 8 IOs.  Still, there's no
			// reason why we can't build the front end to use
			// however many IOs we have.  Of course ... the rest
			// of the design assumes 8 IOs whenever the ASYNC
			// interface is active, so ... defining these here
			// doesn't mean these will work with the rest of the
			// design.
			//
			assign	af_data[     gk] = 1'b0;
			assign	af_data[ 8 + gk] = 1'b0;
			assign	af_data[16 + gk] = 1'b0;
			assign	af_data[24 + gk] = 1'b0;
			// }}}
		end

		assign	MAD_VALID = (afifo_empty == 4'h0); // af_flush && !afifo_empty[0]
		assign	MAD_DATA  = af_data;

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_ds;
		assign	unused_ds = &{ 1'b0,
				ign_afifo_full, ign_acmd_full
				};
		// Verilator lint_on  UNUSED
		// }}}
	end else begin : NO_DATASTROBE
		assign	MAC_VALID = 1'b0;
		assign	MAC_DATA  = 2'b0;
		assign	MAD_VALID = 1'b0;
		assign	MAD_DATA  = 32'h0;

		assign	async_ack = 1'b0;
		assign	async_nak = 1'b0;

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_ds;
		assign	unused_ds = &{ 1'b0, raw_cmd, raw_iodat, i_ds,
				i_cfg_ds, i_cfg_dscmd
				};
		// Verilator lint_on  UNUSED
		// }}}
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// IO buffers --- if not using Verilator
	// {{{
`ifndef	VERILATOR

	IOBUF
	u_cmdbuf( .T(io_cmd_tristate), .I(o_cmd), .IO(io_cmd), .O(i_cmd));

	generate for(gk=0; gk<NUMIO; gk=gk+1)
	begin : GEN_IOBUF
		IOBUF
		u_datbuf(
			.T(io_dat_tristate[gk]),
			.I(o_dat[gk]),
			.IO(io_dat[gk]),
			.O(i_dat[gk])
		);
	end endgenerate
// else
	// If we are using Verilator, then the IO buffers are handled
	// externally as part of the C++ model.
`endif
	// }}}
endmodule
