////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/streamchk.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Used for verifying the operation of a stream.  This IP has two
//		registers.  The first sets the direction of the stream and
//	a mux switch (since the SD card has two stream source/sinks: the SDIO
//	and the EMMC).  The second register sets a SEED, which is then used
//	to determine either outgoing data, or the "correct" data upon receive.
//	What happens next depends on the direction.  If configured as a SINK,
//	data arriving that don't match the SEED, or data arriving after the
//	length has been exhausted will be declared an error.  Likewise, a
//	VALID from the source will also be declared an error.  If configured
//	as a SOURCE, the outgoing data will be drawn from the SEED and LAST
//	will be set at the end of the transaction.
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
`timescale 1ns/1ps
// }}}
module	streamchk #(
		parameter	SW = 32
	) (
		// {{{
`ifdef	SDIO_AXI
		// {{{
		input	wire		S_AXI_ACLK, S_AXI_ARESETN,
		//
		input	wire		S_AXI_AWVALID,
		output	wire		S_AXI_AWREADY,
		input	wire	[2:0]	S_AXI_AWADDR,
		input	wire	[2:0]	S_AXI_AWPROT,
		//
		input	wire		S_AXI_WVALID,
		output	wire		S_AXI_WREADY,
		input	wire	[31:0]	S_AXI_WDATA,
		input	wire	[3:0]	S_AXI_WSTRB,
		//
		output	reg		S_AXI_BVALID,
		input	wire		S_AXI_BREADY,
		output	wire	[1:0]	S_AXI_BRESP,
		//
		input	wire		S_AXI_ARVALID,
		output	wire		S_AXI_ARREADY,
		input	wire	[2:0]	S_AXI_ARADDR,
		input	wire	[2:0]	S_AXI_ARPROT,
		//
		output	wire		S_AXI_RVALID,
		input	wire		S_AXI_RREADY,
		output	wire	[31:0]	S_AXI_RDATA,
		output	wire	[1:0]	S_AXI_RRESP,
		// }}}
`else
		// {{{
		input	wire		i_clk, i_reset,
		//
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[0:0]	i_wb_addr,	// 0=LN, 1=SEED
		input	wire	[31:0]	i_wb_data,
		input	wire	[3:0]	i_wb_sel,
		output	wire		o_wb_stall,
		output	wire		o_wb_ack,
		output	reg	[31:0]	o_wb_data,
		// }}}
`endif
		input	wire		S_VALID,
		output	wire		S_READY,
		input	wire [SW-1:0]	S_DATA,
		input	wire		S_LAST,
		//
		output	wire		M_VALID,
		input	wire		M_READY,
		output	wire [SW-1:0]	M_DATA,
		output	wire		M_LAST,
		//
		output	wire		o_dev,
		output	reg		o_err		// ERR detected flag
		// }}}
	);

	// Local declarations
	// {{{
	localparam [0:0]	D_SINK = 1'b0,
				D_SOURCE = 1'b1;

	wire		clk, reset;
	wire		bus_write, bus_read;
	wire	[0:0]	bus_waddr, bus_raddr;
	wire	[31:0]	bus_wdata;
	reg	[31:0]	bus_rdata;
	wire	[3:0]	bus_wstrb;

	reg		r_dir, r_dev;
	reg	[29:0]	r_len;
	reg	[31:0]	new_len, old_len;
	reg [SW-1:0]	r_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Bus handling
	// {{{
`ifdef	SDIO_AXI
	// {{{
	wire		skd_awvalid, skd_wvalid, skd_arvalid,
			bus_write_ready, bus_read_ready;
	reg		bvalid, rvalid;
	reg	[31:0]	rdata;

	assign	clk = S_AXI_ACLK;
	assign	reset = !S_AXI_ARESETN;

	sdskid #(
		.DW(1), .OPT_OUTREG(1'b0)
	) u_awskd (
		.i_clk(clk), .i_reset(reset),
		.i_valid(S_AXI_AWVALID), .o_ready(S_AXI_AWREADY),
			.i_data(S_AXI_AWADDR[2]),
		.o_valid(skd_awvalid), .i_ready(bus_write_ready),
			.o_data(bus_waddr)
	);

	sdskid #(
		.DW(4+32)
	) u_wskd (
		.i_clk(clk), .i_reset(reset),
		.i_valid(S_AXI_WVALID), .o_ready(S_AXI_WREADY),
			.i_data({ S_AXI_WSTRB, S_AXI_WDATA }),
		.o_valid(skd_wvalid), .i_ready(bus_write_ready),
			.o_data({ bus_wstrb, bus_wdata })
	);

	sdskid #(
		.DW(1)
	) u_arskd (
		.i_clk(clk), .i_reset(reset),
		.i_valid(S_AXI_ARVALID), .o_ready(S_AXI_ARREADY),
			.i_data(S_AXI_ARADDR[2]),
		.o_valid(skd_arvalid), .i_ready(bus_read_ready),
			.o_data(bus_raddr)
	);

	assign	bus_write_ready = (!S_AXI_BVALID || S_AXI_BREADY) && (skd_awvalid && skd_wvalid);
	assign	bus_read_ready = (!S_AXI_RVALID || S_AXI_RREADY) && skd_arvalid;
	assign	bus_write = bus_write_ready && bus_wstrb != 0;
	assign	bus_read  = bus_read_ready;

	always @(posedge clk)
	if (reset)
		bvalid <= 0;
	else if (bus_write_ready)
		bvalid <= 1;
	else if (S_AXI_BREADY)
		bvalid <= 0;

	assign	S_AXI_BVALID = bvalid;

	always @(posedge clk)
	if (reset)
		rvalid <= 0;
	else if (bus_read_ready)
		rvalid <= 1;
	else if (S_AXI_RREADY)
		rvalid <= 0;

	assign	S_AXI_RVALID = rvalid;

	// S_AXI_RDATA
	// {{{
	always @(posedge clk)
	if (reset)
		rdata <= 0;
	else if (bus_read_ready)
		rdata <= bus_rdata;
	else if (S_AXI_RREADY)
		rdata <= 0;

	assign	S_AXI_RDATA = rdata;
	// }}}

	assign	S_AXI_BRESP = 2'b00;
	assign	S_AXI_RRESP = 2'b00;

	// Keep Verilator happy w/ the bus
	// {{{
	// Verilator lint_off UNUSED
	wire	unused_bus;
	assign	unused_bus = &{ 1'b0, S_AXI_AWPROT, S_AXI_ARPROT,
			S_AXI_AWADDR[1:0], S_AXI_ARADDR[1:0] };
	// Verilator lint_on  UNUSED
	// }}}
	// }}}
`else
	// {{{
	reg	wb_ack;

	assign	clk = i_clk;
	assign	reset = i_reset;
	assign	bus_write = i_wb_stb && i_wb_we && (i_wb_sel != 0);
	assign	bus_waddr = i_wb_addr;
	assign	bus_wdata = i_wb_data;
	assign	bus_wstrb = i_wb_sel;

	assign	bus_read  = i_wb_stb && !i_wb_we && (i_wb_sel != 0);
	assign	bus_raddr = i_wb_addr;

	assign	o_wb_stall = 1'b0;
	always @(posedge clk)
	if (reset)
		wb_ack <= 1'b0;
	else
		wb_ack <= i_wb_stb && !o_wb_stall;
	assign	o_wb_ack = wb_ack;

	always @(posedge clk)
	if (reset)
		o_wb_data <= 0;
	else if (bus_read)
		o_wb_data <= bus_rdata;
	else
		o_wb_data <= 0;

	// Keep Verilator happy w/ the bus
	// {{{
	// Verilator lint_off UNUSED
	wire	unused_bus;
	assign	unused_bus = &{ 1'b0, i_wb_cyc };
	// Verilator lint_on  UNUSED
	// }}}
	// }}}
`endif
	// }}}
	////////////////////////////////////////////////////////////////////////
	//

	// r_len, r_dir, r_dev
	// {{{

	always @(*)
	begin
		new_len = { 2'b00, bus_wdata[29:0] };
		new_len[$clog2(SW/8)-1:0] = 0;
		old_len = { 2'b0, r_len } << $clog2(SW/8);

		if (!bus_wstrb[0])
			new_len[ 7: 0] = old_len[ 7: 0];
		if (!bus_wstrb[1])
			new_len[15: 8] = old_len[15: 8];
		if (!bus_wstrb[2])
			new_len[23:16] = old_len[23:16];
		if (!bus_wstrb[3])
			new_len[31:24] = old_len[31:24];
		if (!bus_write || bus_waddr != 0)
			new_len = old_len;

		new_len = new_len >> $clog2(SW/8);
	end

	always @(posedge clk)
	if (reset)
		r_len <= 0;
	else begin
		if ((r_len != 0)&&((S_VALID && S_READY)||(M_VALID && M_READY)))
			r_len <= r_len - 1;

		if (bus_write && bus_waddr == 0)
			r_len <= new_len[29:0];
	end

	always @(posedge clk)
	if (reset)
		{ r_dir, r_dev } <= 2'b00;
	else if (bus_write && bus_waddr == 0 && bus_wstrb[3])
	begin
		r_dir <= bus_wdata[31];
		r_dev <= bus_wdata[30];
	end

	assign	o_dev = r_dev;
	// }}}

	// o_err
	// {{{
	initial	o_err = 1'b0;
	always @(posedge clk)
	if (reset)
		o_err <= 0;
	else begin
		if (bus_write && bus_waddr == 0 && bus_wstrb[0])
			o_err <= 1'b0;

		if (S_VALID && r_dir != D_SINK)
			o_err <= 1'b1;

		if (S_VALID && S_READY)
		begin
			if (r_data != S_DATA)
				o_err <= 1'b1;
			if (r_len == 0)
				o_err <= 1'b1;
			// if (S_LAST != (r_len == 1))
			//	o_err <= 1'b1;
		end

		if (M_READY && r_dir != D_SOURCE)
			o_err <= 1'b1;
	end
	// }}}

	// r_data
	// {{{
	always @(posedge clk)
	if (reset)
		r_data <= 1;
	else begin
		if ((S_VALID && S_READY) || (M_VALID && M_READY))
			r_data <= STEP(r_data);

		if (bus_write && bus_waddr == 1)
		begin
			r_data <= NEWV(r_data, bus_wdata, bus_wstrb);
		end
	end
	// }}}

	// bus_rdata (MUST be combinatorial)
	// {{{
	always @(*)
	begin
		bus_rdata = r_data[31:0];
		if (bus_read && bus_raddr == 0)
		begin
			bus_rdata[31] = r_dir;
			bus_rdata[30:0] = r_len << $clog2(SW/8);
		end
	end
	// }}}

	assign	S_READY = (r_dir == D_SINK);
	assign	M_VALID = (r_dir == D_SOURCE) && (r_len != 0);
	assign	M_DATA  = (r_dir == D_SOURCE) ? r_data : {(SW){1'b0}};
	assign	M_LAST  = (r_dir == D_SOURCE) && (r_len <= 1);

	localparam	[SW-1:0] POLY = { {(SW-24){1'b0}}, 24'h4c0001 };
	function automatic [SW-1:0]	STEP(input [SW-1:0] sreg);
		// {{{
	begin
		// sreg = sreg + 1;
		if (sreg[22])
			sreg = (sreg << 1) ^ POLY;
		else
			sreg = (sreg << 1);
		STEP = sreg;
	end endfunction
	// }}}

	// localparam	[PLEN-1:0] POLYFILL = 
	function automatic [SW-1:0]	NEWV(input [SW-1:0] old,
			input[31:0] nw, input [3:0] strb);
		// {{{
		reg	[SW-1:0]	v;
		integer			ik;
	begin
		v = 0;
		v[31:0] = old;
		for(ik=0; ik<4; ik=ik+1)
		if (strb[ik])
			v[8*ik +: 8] = nw[8*ik +: 8];

		NEWV = v;
	end endfunction
	// }}}
endmodule
