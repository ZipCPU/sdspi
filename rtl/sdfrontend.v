////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdfrontend.v
// {{{
// Project:	SDIO SD-Card controller, using a shared SPI interface
//
// Purpose:	
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
module	sdfrontend #(
		parameter [0:0]	OPT_SERDES = 1'b1,
		parameter	NUMIO = 8
	) (
		// {{{
		input	wire		i_clk, i_hsclk,
		// Configuration
		input	wire		i_cfg_ddr,
		input	wire	[4:0]	i_sample_shift,
		// Control signals
		// Tx path
		// {{{
		input	wire	[7:0]	i_sdclk,
		//
		input	wire		i_cmd_en,
		input	wire		i_pp_cmd,	// Push/pull cmd lines
		input	wire	[1:0]	i_cmd_data,
		output	wire		o_cmd_busy,
		//
		input	wire		i_data_en,
		input	wire		i_pp_data,	// Push/pull data lines
		input	wire	[31:0]	i_tx_data,
		input	wire		i_afifo_reset_n,
		// }}}
		// Synchronous Rx path
		// {{{
		output	wire	[1:0]	o_cmd_strb,
		output	wire	[1:0]	o_cmd_data,
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
		inout	wire			io_cmd,
		inout	wire			io_ds,
		inout	wire	[NUMIO-1:0]	io_dat
		// }}}
		// }}}
	);

	localparam	[0:0]	OPT_ODDR = 1'b0;
	genvar		gk;
	reg		cmd_busy, wait_for_busy;

	generate if (!OPT_SERDES && !OPT_ODDR)
	begin : GEN_NO_SERDES
		// {{{
		wire		next_edge;
		reg		resp_started, io_started, last_clk;
		reg		r_cmd_data, r_cmd_strb, r_rx_strb;
		reg	[7:0]	r_rx_data;


		assign	o_ck = i_sdclk[7];

		assign	io_cmd = (i_cmd_en && (i_pp_cmd || !i_cmd_data[1]))
						? i_cmd_data[1] : 1'bz;

		for(gk=0; gk<NUMIO; gk=gk+1)
		begin
			assign	io_dat[gk] = (i_data_en
					&& (i_pp_data || !i_tx_data[24+gk]))
				? i_tx_data[24+gk] : 1'bz;
		end

		assign	next_edge = last_clk && !i_sdclk[7];

		always @(posedge i_clk)
		if (i_cmd_en)
			resp_started <= 1'b0;
		else if (!io_cmd)
			resp_started <= 1'b1;

		always @(posedge i_clk)
		if (i_data_en)
			io_started <= 1'b0;
		else if (!io_cmd)
			io_started <= 1'b1;

		// cmd_busy, wait_for_busy
		// {{{
		always @(posedge i_clk)
		if (i_reset)
		begin
			cmd_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (i_cmd_en)
		begin
			cmd_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (wait_for_busy && !r_cmd_data[0])
		begin
			cmd_busy <= 1'b1;
			wait_for_busy <= 1'b0;
		end else if (!wait_for_busy && r_cmd_data[0])
			cmd_busy <= 1'b0;

		assign	o_cmd_busy = cmd_busy;
		// }}}

		initial	last_clk = 1'b0;
		always @(posedge i_clk)
		begin
			last_clk <= i_sdclk[0];

			if (i_cmd_en)
				r_cmd_strb <= 1'b0;
			else if (!io_cmd || resp_started)
				r_cmd_strb <= 1'b1;
			else
				r_cmd_strb <= 1'b0;

			if (i_data_en)
				r_rx_strb <= 1'b0;
			else if (next_edge && (io_started || io_dat == 0))
				r_rx_strb <= 1'b1;
			else
				r_rx_strb <= 1'b0;

			r_cmd_data <= io_cmd;
			r_rx_data <= io_dat;
		end

		assign	o_cmd_strb = { r_cmd_strb, 1'b0 };
		assign	o_cmd_data = { r_cmd_data, 1'b0 };
		assign	o_rx_strb  = { r_rx_strb, 1'b0 };
		assign	o_rx_data  = { r_rx_data, 8'h0 };

		// No asynchronous outputs w/o OPT_SERDES
		assign	MAC_VALID = 1'b0;
		assign	MAC_DATA  = 2'h0;
		assign	MAD_VALID = 1'b0;
		assign	MAD_DATA  = 32'h0;

		// Keep Verilator happy
		// {{{
		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused_no_serdes;
		assign	unused_no_serdes = &{ 1'b0, i_afifo_reset_n,
				i_sdclk[6:0], i_tx_data[23:0],
				i_cmd_data[0], i_hsclk, i_cfg_ddr,i_sample_shift
				};
		// Verilator lint_on  UNUSED
		// Verilator coverage_on
		// }}}
		// }}}
	end else if (!OPT_SERDES && OPT_ODDR)
	begin : GEN_IODDR_IO
		// {{{
		// Not built.
		// The idea is, if we only have DDR elements, can we do better
		// than with just IOs?
		// }}}
	end else begin : GEN_WIDE_IO
		// {{{
		reg		af_started_p, af_started_n, acmd_started;
		reg		af_count_p, af_count_n, acmd_count;
		wire	[3:0]	ign_afifo_full, afifo_empty;
		wire	[31:0]	af_data;
		wire	[1:0]	acmd_empty, ign_acmd_full, w_cmd_data;
		wire	[15:0]	w_rx_data;
		reg	[15:0]	r_rx_data;
		reg		last_ck;
		wire	[7:0]	next_ck_sreg;
		reg	[23:0]	ck_sreg;
		wire	[7:0]	next_pedge, next_nedge, wide_cmd_data;
		reg	[7:0]	sample_ck;
		wire	[1:0]	af_cmd;
		reg	[1:0]	r_cmd_data;
		reg	[1:0]	busy_strb, busy_data;

		assign	next_pedge = { ~{last_ck, i_sdclk[7:1] } &  i_sdclk[7:0] };
		assign	next_nedge = i_cfg_ddr ? {  {last_ck, i_sdclk[7:1] } & ~i_sdclk[7:0] } : 8'h0;

		assign	next_ck_sreg = (i_data_en) ? 8'h0
				: { next_pedge | next_nedge };

		always @(posedge i_clk)
		if (!i_afifo_reset_n || i_data_en)
			{ last_ck, ck_sreg } <= 0;
		else begin
			last_ck <= i_sdclk[0];
			ck_sreg <= { ck_sreg[15:0], next_ck_sreg };
		end

		always @(posedge i_clk)
		if (i_reset || i_data_en || i_afifo_reset_n)
			sample_ck <= 0;
		else
			// Verilator lint_off WIDTH
			sample_ck <= { ck_sreg[23:0], next_ck_sreg } >> i_sample_shift;
			// Verilator lint_on  WIDTH

		always @(posedge i_clk)
		if (i_reset || i_data_en || i_cmd_en)
			busy_ck <= 0;
		else
			// Verilator lint_off WIDTH
			busy_ck <= { ck_sreg[23:0], next_ck_sreg } >> i_sample_shift;
			// Verilator lint_on  WIDTH


		for(gk=0; gk<NUMIO; gk=gk+1)
		begin : GEN_WIDE_DATIO
			// {{{
			wire		out_en;
			wire	[7:0]	out_pin, in_pin;
			integer		ikin, ikout;

			always @(*)
			for(ikout=0; ikout<4; ikout++)
				out_pin[ikout*2 +: 2] = {(2){i_tx_data[ikout*8+gk]}};

			assign	out_en = i_data_en &&(i_pp_data || !out_pin[3]);

			xsdserdes8x #(
				.OPT_BIDIR(1'b1)
			) io_serdes(
				.i_clk(i_clk),
				.i_hsclk(i_hsclk),
				.i_en(out_en),
				.i_data(out_pin),
				.io_pin(io_dat[gk]),
				.o_wide(in_pin)
			);

			always @(*)
			begin
				w_rx_data[8+gk] = 0;
				if (|sample_ck[7:4])
				begin
					for(ikin=4; ikin<8; ikin++)
					if (sample_ck[ikin])
						r_rx_data[8 + gk] = in_pin[ikin];
				end

				if (|sample_ck[3:0])
				begin
					for(ikin=0; ikin<4; ikin++)
					if (sample_ck[ikin])
						r_rx_data[gk] = in_pin[ikin];
				end
			end

			assign	w_rx_data = r_rx_data;
			// }}}
		end for(gk=NUMIO; gk<8; gk=gk+1)
		begin : NULL_DATIO
			// {{{
			integer		ik;

			assign	w_rx_data[8+gk] = 1'b1;
			assign	w_rx_data[  gk] = 1'b1;

			// Keep Verilator happy
			// {{{
			// Verilator coverage_off
			// Verilator lint_off UNUSED
			wire	unused_outputs;
			assign	unused_outputs = &{ 1'b0 }; // , out_pin };
			// Verilator lint_on  UNUSED
			// Verilator coverage_on
			// }}}
			// }}}
		end

		// o_rx_strb, o_rx_data
		// {{{
		reg	[1:0]	busy_strb;
		always @(posedge i_clk)
		begin
			o_rx_strb[1] <= (|sample_ck);
			o_rx_strb[0] <= (|sample_ck[7:4]) && (|sample_ck[3:0]);

			if (sample_ck[7:4] == 0)
				o_rx_data <= { w_rx_data[7:0], 8'h0 };
			else
				o_rx_data <= w_rx_data;

			if (i_afifo_reset_n)
				o_rx_strb <= 2'b0;
		end
		// }}}

		// cmd_busy, wait_for_busy, busy_strb
		// {{{
		always @(*)
		begin
			busy_strb[1] = (|busy_ck);
			busy_strb[0] = (|busy_ck[7:4]) && (|busy_ck[3:0]);

			if (busy_ck[7:4] == 0)
				busy_data = { w_rx_data[0], 1'h0 };
			else
				busy_data = { w_rx_data[8], w_rx_data[0] };
		end

		always @(posedge i_clk)
		if (i_reset)
		begin
			cmd_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (i_cmd_en)
		begin
			cmd_busy <= 1'b0;
			wait_for_busy <= 1'b1;
		end else if (wait_for_busy)
		begin
			if ((busy_strb[0] && !busy_data[0])
				||(busy_strb == 2'b10 && !busy_data[1]))
			begin
				cmd_busy <= 1'b1;
				wait_for_busy <= 1'b0;
			end
		end else if ((busy_strb[0] && busy_data[0])
				|| (busy_strb == 2'b10 && busy_data[1]))
			cmd_busy <= 1'b0;

		assign	o_cmd_busy = cmd_busy;
		// }}}

		////////////////////////////////////////////////////////////////
		//
		// CMD
		// {{{

		// assign	next_pedge = { ~{last_ck, i_sdclk[7:1] } &  i_sdclk[7:0] };

		reg	[23:0]	pck_sreg;
		reg	[7:0]	cmd_sample_ck;

		always @(posedge i_clk)
		if (!i_afifo_reset_n || i_data_en)
			pck_sreg <= 0;
		else
			pck_sreg <= { pck_sreg[15:0], next_pedge };

		always @(posedge i_clk)
		if (!i_afifo_reset_n || i_data_en)
			cmd_sample_ck <= 0;
		else
			// Verilator lint_off WIDTH
			cmd_sample_ck <= { pck_sreg[23:0], next_pedge } >> i_sample_shift;
			// Verilator lint_on  WIDTH


		xsdserdes8x #(
			.OPT_BIDIR(1'b1)
		) cmd_serdes(
			.i_clk(i_clk),
			.i_hsclk(i_hsclk),
			.i_en(i_cmd_en && (i_pp_cmd || !i_cmd_data[1])),
			.i_data({ {(4){i_cmd_data[1]}}, {(4){i_cmd_data[0]}} }),
			.io_pin(io_cmd),
			.o_wide(wide_cmd_data)
		);

		integer	ikcmd;

		always @(*)
		begin
			w_cmd_data[1:0] = 0;
			if (|sample_ck[7:4])
			begin
				for(ikcmd=4; ikcmd<8; ikcmd++)
				if (cmd_sample_ck[ikcmd])
					w_cmd_data[1] = wide_cmd_data[ikcmd];
			end

			if (|sample_ck[3:0])
			begin
				for(ikcmd=0; ikcmd<4; ikcmd++)
				if (cmd_sample_ck[ikcmd])
					w_cmd_data[gk] = wide_cmd_data[ikcmd];
			end
		end

		always @(posedge i_clk)
		if (sample_ck[7:4] == 0)
			r_cmd_data <= { w_cmd_data[0], 1'b0 };
		else
			r_cmd_data <= w_cmd_data;

		assign	o_cmd_data = r_cmd_data;

		// }}}
		////////////////////////////////////////////////////////////////
		//
		// Data strobe based inputs
		// {{{

		// Async command port
		// {{{
		// The rule here is that only the positive edges of the
		// data strobe will qualify the CMD pin;
		always @(posedge io_ds or negedge i_cmd_en)
		if (!i_cmd_en)
			acmd_started <= 0;
		else if (!io_cmd)
			acmd_started <= 1;

		always @(posedge io_ds or negedge i_cmd_en)
		if (!i_afifo_reset_n)
			acmd_count <= 0;
		else if (acmd_started || !io_cmd)
			acmd_count <= acmd_count + 1;

		afifo #(
			.LGFIFO(4), .WIDTH(1), .WRITE_ON_POSEDGE(1'b1)
		) u_pcmd_fifo_0 (
			// {{{
			.i_wclk(io_ds), .i_wr_reset_n(i_cmd_en),
			.i_wr((acmd_started || !io_cmd)&& acmd_count == 1'b0),
				.i_wr_data(io_cmd),
			.o_wr_full(ign_afifo_full[0]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(i_cmd_en),
			.i_rd(acmd_empty == 2'b0), .o_rd_data(af_cmd[1]),
			.o_rd_empty(acmd_empty[0])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(1), .WRITE_ON_POSEDGE(1'b1)
		) u_pcmd_fifo_1 (
			// {{{
			.i_wclk(io_ds), .i_wr_reset_n(i_cmd_en),
			.i_wr(acmd_count), .i_wr_data(io_cmd),
			.o_wr_full(ign_acmd_full[1]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(i_cmd_en),
			.i_rd(acmd_empty == 2'b0), .o_rd_data(af_cmd[0]),
			.o_rd_empty(acmd_empty[1])
			// }}}
		);

		assign	MAC_VALID = (acmd_empty == 2'h0);
		assign	MAC_DATA  = af_cmd;
		// }}}

		// af_started_*, af_count_*
		// {{{
		always @(posedge io_ds or negedge i_afifo_reset_n)
		if (!i_afifo_reset_n)
			af_started_p <= 0;
		else if (io_dat[0] == 0)
			af_started_p <= 1;

		always @(posedge io_ds or negedge i_afifo_reset_n)
		if (!i_afifo_reset_n)
			af_count_p <= 0;
		else if (af_started_p)
			af_count_p <= af_count_p + 1;

		always @(negedge io_ds or negedge i_afifo_reset_n)
		if (!i_afifo_reset_n)
			af_started_n <= 0;
		else if (af_started_p)
			af_started_n <= 1;

		always @(negedge io_ds or negedge i_afifo_reset_n)
		if (!i_afifo_reset_n)
			af_count_n <= 0;
		else if (af_started_n)
			af_count_n <= af_count_n + 1;
		// }}}


		afifo #(
			.LGFIFO(4), .WIDTH(8), .WRITE_ON_POSEDGE(1'b1)
		) u_pedge_fifo_0 (
			// {{{
			.i_wclk(io_ds), .i_wr_reset_n(i_afifo_reset_n),
			.i_wr(af_started_p && af_count_p == 1'b0),
				.i_wr_data(io_dat),
			.o_wr_full(ign_afifo_full[0]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(i_afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[31:24]),
			.o_rd_empty(afifo_empty[0])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(8), .WRITE_ON_POSEDGE(1'b0)
		) u_nedge_fifo_1 (
			// {{{
			.i_wclk(io_ds), .i_wr_reset_n(i_afifo_reset_n),
			.i_wr(af_started_n && af_count_n == 1'b0),
				.i_wr_data(io_dat),
			.o_wr_full(ign_afifo_full[1]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(i_afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[23:16]),
			.o_rd_empty(afifo_empty[1])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(8), .WRITE_ON_POSEDGE(1'b1)
		) u_pedge_fifo_2 (
			// {{{
			.i_wclk(io_ds), .i_wr_reset_n(i_afifo_reset_n),
			.i_wr(af_count_p == 1'b1),
				.i_wr_data(io_dat),
			.o_wr_full(ign_afifo_full[2]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(i_afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[15:8]),
			.o_rd_empty(afifo_empty[2])
			// }}}
		);

		afifo #(
			.LGFIFO(4), .WIDTH(8), .WRITE_ON_POSEDGE(1'b0)
		) u_nedge_fifo_3 (
			// {{{
			.i_wclk(io_ds), .i_wr_reset_n(i_afifo_reset_n),
			.i_wr(af_count_n == 1'b1),
				.i_wr_data(io_dat),
			.o_wr_full(ign_afifo_full[3]),
			//
			.i_rclk(i_clk), .i_rd_reset_n(i_afifo_reset_n),
			.i_rd(MAD_VALID), .o_rd_data(af_data[ 7: 0]),
			.o_rd_empty(afifo_empty[3])
			// }}}
		);

		/*
		reg		prior_af_return, af_flush;

		always @(posedge i_clk)
		if (!i_afifo_reset_n)
			prior_af_return <= 0;
		else if (MAD_VALID)
			prior_af_return <= 1;

		always @(posedge i_clk)
		if (!i_afifo_reset_n || !prior_af_return)
			af_flush <= 1'b0;
		else if (afifo_empty == 4'h0)
			af_flush <= 1'b0;
		else
			af_flush <= !afifo_empty[0];
		*/

		assign	MAD_VALID = (afifo_empty == 4'h0); // af_flush && !afifo_empty[0]
		assign	MAD_DATA  = af_data;
		// assign	MAD_LAST  = af_flush && (afifo_empty != 4'h0);
		// }}}
		// }}}
	end endgenerate
endmodule

