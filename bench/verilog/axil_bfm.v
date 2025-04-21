////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/axil_bfm.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Drive an AXI-Lite bus via commands from a test script.  This is
//		*NOT* a sufficient method to test whether or not an AXI-Lite
//	peripheral is bus compliant.  However, it can be used to verify
//	whether or not an AXI-Lite peripheral behaves as it should to a subset
//	of possible/potential bus interactions.
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
module	axil_bfm #(
		// {{{
		parameter AW = 5,
		localparam	ADDR_WIDTH=AW,
		parameter DW = 32,
		parameter LGFIFO = 4,
		parameter [0:0]	OPT_DEBUG = 1'b0
		// }}}
	) (
		// {{{
		input	wire			i_aclk, i_aresetn,
		//
		output	reg			AXIL_AWVALID,
		input	wire			AXIL_AWREADY,
		output	reg	[AW-1:0]	AXIL_AWADDR,
		output	reg	[2:0]		AXIL_AWPROT,
		//
		output	reg			AXIL_WVALID,
		input	wire			AXIL_WREADY,
		output	reg	[DW-1:0]	AXIL_WDATA,
		output	reg	[DW/8-1:0]	AXIL_WSTRB,
		//
		input	wire			AXIL_BVALID,
		output	wire			AXIL_BREADY,
		input	wire	[1:0]		AXIL_BRESP,
		//
		output	reg			AXIL_ARVALID,
		input	wire			AXIL_ARREADY,
		output	reg	[AW-1:0]	AXIL_ARADDR,
		output	reg	[2:0]		AXIL_ARPROT,
		//
		input	wire			AXIL_RVALID,
		output	wire			AXIL_RREADY,
		input	wire	[DW-1:0]	AXIL_RDATA,
		input	wire	[1:0]		AXIL_RRESP
		// }}}
	);

	// Local declarations
	// {{{
	localparam	AXILLSB = $clog2(DW/8);

	reg	[31:0]		aw_outstanding, aw_requests;
	wire	[31:0]		aw_committed;
	reg	[31:0]		w_outstanding, w_requests;
	wire	[31:0]		w_committed;
	reg	[31:0]		ar_outstanding, ar_requests;
	wire	[31:0]		ar_committed;
	reg	[LGFIFO:0]	awfifo_fill, awfifo_rdaddr, awfifo_wraddr;
	reg	[LGFIFO:0]	 wfifo_fill,  wfifo_rdaddr,  wfifo_wraddr;
	reg	[LGFIFO:0]	arfifo_fill, arfifo_rdaddr, arfifo_wraddr;
	reg	[AW-1:0]	awfifo	[0:((1<<LGFIFO)-1)];
	reg	[(DW/8)+DW-1:0]	 wfifo	[0:((1<<LGFIFO)-1)];
	reg	[AW-1:0]	arfifo	[0:((1<<LGFIFO)-1)];
	wire			aw_full, w_full, ar_full;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// User interface tasks
	// {{{
	// These will queue tasks into the request FIFO

	initial	{ awfifo_rdaddr, awfifo_wraddr } = 0;
	initial	{  wfifo_rdaddr,  wfifo_wraddr } = 0;
	initial	{ arfifo_rdaddr, arfifo_wraddr } = 0;

	task writeio(input [ADDR_WIDTH-1:0] addr, input [DW-1:0] dat);
		// {{{
	begin
		if (OPT_DEBUG)
			$display("BFM:WRITE @0x%04x <-- %08x", addr, dat);

		if (i_aresetn !== 1'b1)
		begin
			wait(i_aresetn == 1'b1);
			@(posedge i_aclk);
			@(posedge i_aclk);
		end

		while(
			((awfifo_wraddr[LGFIFO-1:0] == awfifo_rdaddr[LGFIFO-1:0])
				&&(awfifo_wraddr[LGFIFO] != awfifo_rdaddr[LGFIFO]))
			||((wfifo_wraddr[LGFIFO-1:0] == wfifo_rdaddr[LGFIFO-1:0])
				&&(wfifo_wraddr[LGFIFO] != wfifo_rdaddr[LGFIFO])))
		begin
			@(posedge i_aclk);
		end

		awfifo[awfifo_wraddr[LGFIFO-1:0]] = addr;
		wfifo[  wfifo_wraddr[LGFIFO-1:0]] = { 4'hf, dat };
		awfifo_wraddr = awfifo_wraddr + 1;
		 wfifo_wraddr =  wfifo_wraddr + 1;
		awfifo_fill   = awfifo_fill + 1;
		 wfifo_fill   =  wfifo_fill + 1;
	end endtask
	// }}}

	task write_f(input [ADDR_WIDTH-1:0] addr, input [DW-1:0] dat);
		// {{{
	begin
		writeio(addr, dat);

		// Now wait for the last read to finish
		while(aw_outstanding > 0 || w_outstanding > 0
				|| awfifo_fill != 0 || wfifo_fill != 0)
			@(posedge i_aclk);
	end endtask
	// }}}

	task readio(input [ADDR_WIDTH-1:0] addr, output [DW-1:0] dat);
		// {{{
		reg			returned, err_flag;
		reg	[LGFIFO:0]	read_busaddr;
		reg	[31:0]		return_count;
	begin
		while(i_aresetn !== 1'b1)
		begin
			wait(i_aresetn === 1'b1);
			@(posedge i_aclk);
			@(posedge i_aclk);
		end

		err_flag = 1'b0;
		while((arfifo_wraddr[LGFIFO-1:0] == arfifo_rdaddr[LGFIFO-1:0])
			&&(arfifo_wraddr[LGFIFO] != arfifo_rdaddr[LGFIFO]))
		begin
			@(posedge i_aclk);
		end

		arfifo[arfifo_wraddr[LGFIFO-1:0]] = addr;
		arfifo_wraddr = arfifo_wraddr + 1;
		return_count = 0;
		return_count[LGFIFO-1:0] = arfifo_wraddr[LGFIFO-1:0] - arfifo_rdaddr[LGFIFO-1:0];
		return_count = return_count + ar_outstanding;
// $display("READ ARFIFO(0x%08x, %6d, %6d, %6d,count=%2d) at %t", addr, arfifo_wraddr, arfifo_rdaddr, ar_outstanding, return_count, $time);


		do begin
			@(posedge i_aclk)
			if (AXIL_RVALID === 1'b1 && AXIL_RREADY === 1'b1)
			begin
				return_count <= return_count - 1;
				dat <= AXIL_RDATA;
				err_flag <= (AXIL_RRESP != 2'b00);
			end
			wait(!i_aclk);
		end while(return_count >= 1);

		if (OPT_DEBUG)
			$display("BFM:READ  @0x%04x --> %08x at %t", addr, dat, $time);
	end endtask
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Issue requests
	// {{{
	assign	aw_committed = aw_outstanding + (AXIL_AWVALID ? 1:0);
	assign	w_committed  =  w_outstanding + (AXIL_WVALID  ? 1:0);
	assign	ar_committed = ar_outstanding + (AXIL_ARVALID ? 1:0);
	assign	aw_full = (&aw_committed);
	assign	 w_full = (& w_committed);
	assign	ar_full = (&ar_committed);

	always @(posedge i_aclk)
	if (!i_aresetn)
		aw_outstanding <= 0;
	else case({ AXIL_AWVALID && AXIL_AWREADY, AXIL_BVALID && AXIL_BREADY })
	2'b10: aw_outstanding <= aw_outstanding + 1;
	2'b01: aw_outstanding <= (aw_outstanding > 0)
					? (aw_outstanding - 1) : 0;
	default: begin end
	endcase

	always @(posedge i_aclk)
	if (!i_aresetn)
		w_outstanding <= 0;
	else case({ AXIL_WVALID && AXIL_WREADY,
				AXIL_BVALID && AXIL_BREADY })
	2'b10: w_outstanding <= w_outstanding + 1;
	2'b01: w_outstanding <= (w_outstanding > 0)
					? (w_outstanding - 1) : 0;
	default: begin end
	endcase

	always @(posedge i_aclk)
	if (!i_aresetn)
		ar_outstanding <= 0;
	else case({ AXIL_ARVALID && AXIL_ARREADY,
					AXIL_RVALID && AXIL_RREADY })
	2'b10: ar_outstanding <= ar_outstanding + 1;
	2'b01: ar_outstanding <= (ar_outstanding > 0)
					? (ar_outstanding - 1) : 0;
	default: begin end
	endcase

	always @(*) awfifo_fill = awfifo_wraddr - awfifo_rdaddr;
	always @(*)  wfifo_fill =  wfifo_wraddr -  wfifo_rdaddr;
	always @(*) arfifo_fill = arfifo_wraddr - arfifo_rdaddr;

	always @(*) aw_requests = awfifo_fill + aw_outstanding;
	always @(*)  w_requests =  wfifo_fill +  w_outstanding;
	always @(*) ar_requests = arfifo_fill + ar_outstanding;

	// AW* issue
	// {{{
	always @(posedge i_aclk)
	if (!i_aresetn)
	begin
		// {{{
		awfifo_rdaddr <= 0;
		AXIL_AWVALID <= 0;
		AXIL_AWADDR  <= 0;
		AXIL_AWPROT  <= 0;
		// }}}
	end else if ((!AXIL_AWVALID || AXIL_AWREADY) && (awfifo_fill != 0))
	begin
		// {{{
		AXIL_AWVALID <= 1'b1;
		AXIL_AWADDR  <= awfifo[awfifo_rdaddr[LGFIFO-1:0]];
		awfifo_rdaddr  <= awfifo_rdaddr + 1;
		// }}}
	end else if (AXIL_AWREADY)
	begin
		// {{{
		AXIL_AWVALID <= 1'b0;
		AXIL_AWADDR  <= 0;
		// }}}
	end
	// }}}

	// W* issue
	// {{{
	always @(posedge i_aclk)
	if (!i_aresetn)
	begin
		// {{{
		wfifo_rdaddr <= 0;
		AXIL_WVALID <= 0;
		AXIL_WDATA  <= 0;
		AXIL_WSTRB  <= 0;
		// }}}
	end else if ((!AXIL_WVALID || AXIL_WREADY) && (wfifo_fill != 0))
	begin
		// {{{
		AXIL_WVALID <= 1'b1;
		{ AXIL_WSTRB, AXIL_WDATA } <= wfifo[wfifo_rdaddr[LGFIFO-1:0]];
		wfifo_rdaddr  <= wfifo_rdaddr + 1;
		// }}}
	end else if (AXIL_WREADY)
	begin
		// {{{
		AXIL_WVALID <= 1'b0;
		AXIL_WDATA  <= 0;
		AXIL_WSTRB  <= 0;
		// }}}
	end
	// }}}

	// AR* issue
	// {{{
	wire	[AW-1:0]	next_araddr;
	assign	next_araddr = arfifo[arfifo_rdaddr[LGFIFO-1:0]];

	always @(posedge i_aclk)
	if (!i_aresetn)
	begin
		// {{{
		arfifo_rdaddr <= 0;
		AXIL_ARVALID <= 0;
		AXIL_ARADDR  <= 0;
		AXIL_ARPROT  <= 0;
		// }}}
	end else if ((!AXIL_ARVALID || AXIL_ARREADY) && (arfifo_fill != 0))
	begin
		// {{{
		AXIL_ARVALID <= 1'b1;
		AXIL_ARADDR  <= arfifo[arfifo_rdaddr[LGFIFO-1:0]];
		arfifo_rdaddr  <= arfifo_rdaddr + 1;
		// }}}
	end else if (AXIL_ARREADY)
	begin
		// {{{
		AXIL_ARVALID <= 1'b0;
		AXIL_ARADDR  <= 0;
		AXIL_ARPROT  <= 0;
		// }}}
	end
	// }}}

	// }}}

	assign	AXIL_BREADY = 1'b1;
	assign	AXIL_RREADY = 1'b1;
endmodule

