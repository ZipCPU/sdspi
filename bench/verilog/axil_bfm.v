////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	axil_bfm.v
// {{{
// Project:	SDIO SD-Card controller
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
// Copyright (C) 2023, Gisselquist Technology, LLC
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
		parameter AW = 5,
		parameter DW = 32,
		parameter LGFIFO = 4,
		parameter [0:0]	OPT_DEBUG = 1'b0
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
		input	reg			AXIL_BVALID,
		output	wire			AXIL_BREADY,
		input	reg	[1:0]		AXIL_BRESP,
		//
		output	reg			AXIL_ARVALID,
		input	wire			AXIL_ARREADY,
		output	reg	[AW-1:0]	AXIL_ARADDR,
		output	reg	[2:0]		AXIL_ARPROT,
		//
		input	reg			AXIL_RVALID,
		output	wire			AXIL_RREADY,
		input	reg	[DW-1:0]	AXIL_RDATA,
		input	reg	[1:0]		AXIL_RRESP
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

	initial	{ fifo_rdaddr, fifo_wraddr } = 0;

	task writeio(input [ADDR_WIDTH-1:0] addr, input [DW-1:0] dat);
		// {{{
	begin
		if (OPT_DEBUG)
			$display("BFM:WRITE @0x%04x <-- %08x", addr, dat);
		wait(i_aresetn);

		while(
			((awfifo_wraddr[LGFIFO-1:0] == awfifo_rdaddr[LGFIFO-1:0])
				&&(awfifo_wraddr[LGFIFO] != awfifo_rdaddr[LGFIFO]))
			||((wfifo_wraddr[LGFIFO-1:0] == wfifo_rdaddr[LGFIFO-1:0])
				&&(wfifo_wraddr[LGFIFO] != wfifo_rdaddr[LGFIFO]))
		begin
			@(posedge i_aclk);
		end

		awfifo[awfifo_wraddr[LGFIFO-1:0]] = addr;
		wfifo[  wfifo_wraddr[LGFIFO-1:0]] = { 4'hf, data };
		awfifo_wraddr = awfifo_wraddr + 1;
		 wfifo_wraddr =  wfifo_wraddr + 1;
	end endtask
	// }}}

	task write_f(input [ADDR_WIDTH-1:0] addr, input [DW-1:0] dat);
		// {{{
	begin
		write_f(addr, dat);

		// Now wait for the last read to finish
		while(aw_outstanding > 0 || w_outstanding > 0
				|| awfifo_fill != 0 || wfifo_fill != 0)
			@(posedge i_aclk);

	end endtask
	// }}}

	task readio(input [ADDR_WIDTH-1:0] addr, output [DW-1:0] dat);
		reg			returned, err_flag;
		reg	[LGFIFO:0]	read_busaddr;
		reg	[31:0]		return_count;
	begin
		err_flag = 1'b0;

		while((arfifo_wraddr[LGFIFO-1:0] == arfifo_rdaddr[LGFIFO-1:0])
			&&(arfifo_wraddr[LGFIFO] != arfifo_rdaddr[LGFIFO]))
		begin
			@(posedge i_aclk);
		end

		arfifo[arfifo_wraddr[LGFIFO-1:0]] = addr;
		arfifo_wraddr = arfifo_wraddr + 1;
		return_count  = arfifo_wraddr - arfifo_rdaddr + ar_outstanding;


		do begin
			@(posedge i_aclk)
			if (S_AXIL_RVALID && S_AXIL_RREADY)
			begin
				return_count <= return_count - 1;
				dat <= S_AXIL_RDATA;
			end
			wait(!i_aclk);
		end while(return_count > 1);

		if (OPT_DEBUG)
			$display("BFM:READ  @0x%04x --> %08x", addr, dat);
	end endtask

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Issue requests
	// {{{
	assign	aw_committed = aw_outstanding + (S_AXIL_AWVALID ? 1:0);
	assign	w_committed  =  w_outstanding + (S_AXIL_WVALID  ? 1:0);
	assign	ar_committed = ar_outstanding + (S_AXIL_ARVALID ? 1:0);
	assign	aw_full = (&aw_committed);
	assign	 w_full = (& w_committed);
	assign	ar_full = (&ar_committed);

	always @(posedge i_aclk)
	if (!i_aresetn)
		aw_outstanding <= 0;
	else case({ S_AXIL_AWVALID && S_AXIL_AWREADY, S_AXIL_BVALID && S_AXIL_BREADY })
	2'b10: aw_outstanding <= aw_outstanding + 1;
	2'b01: aw_outstanding <= (aw_outstanding > 0)
					? (aw_outstanding - 1) : 0;
	default: begin end
	endcase

	always @(posedge i_aclk)
	if (!i_aresetn)
		w_outstanding <= 0;
	else case({ S_AXIL_WVALID && S_AXIL_WREADY,
				S_AXIL_BVALID && S_AXIL_BREADY })
	2'b10: w_outstanding <= w_outstanding + 1;
	2'b01: w_outstanding <= (w_outstanding > 0)
					? (w_outstanding - 1) : 0;
	default: begin end
	endcase

	always @(posedge i_aclk)
	if (i_aresetn)
		ar_outstanding <= 0;
	else case({ S_AXIL_ARVALID && S_AXIL_ARREADY,
					S_AXIL_RVALID && S_AXIL_RREADY })
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

	always @(posedge i_aclk)
	if (!i_aresetn)
	begin
		// {{{
		awfifo_rdaddr <= 0;
		S_AXIL_AWVALID <= 0;
		S_AXIL_AWADDR  <= 0;
		// }}}
	end else if ((!S_AXIL_AWVALID || S_AXIL_AWREADY) && (awfifo_fill != 0))
	begin
		// {{{
		S_AXIL_AWVALID <= 1'b1;
		S_AXIL_AWADDR  <= awfifo[awfifo_rdaddr];
		awfifo_rdaddr  <= awfifo_rdaddr + 1;
		// }}}
	end else if (S_AXIL_AWREADY)
	begin
		// {{{
		S_AXIL_AWVALID <= 1'b0;
		S_AXIL_AWADDR  <= 0;
		// }}}
	end

	always @(posedge i_aclk)
	if (!i_aresetn)
	begin
		// {{{
		wfifo_rdaddr <= 0;
		S_AXIL_WVALID <= 0;
		S_AXIL_WDATA  <= 0;
		S_AXIL_WSTRB  <= 0;
		// }}}
	end else if ((!S_AXIL_WVALID || S_AXIL_WREADY) && (wfifo_fill != 0))
	begin
		// {{{
		S_AXIL_WVALID <= 1'b1;
		{ S_AXIL_WSTRB, S_AXIL_WDATA } <= wfifo[wfifo_rdaddr];
		wfifo_rdaddr  <= wfifo_rdaddr + 1;
		// }}}
	end else if (S_AXIL_WREADY)
	begin
		// {{{
		S_AXIL_WVALID <= 1'b0;
		S_AXIL_WDATA  <= 0;
		S_AXIL_WSTRB  <= 0;
		// }}}
	end


	always @(posedge i_aclk)
	if (!i_aresetn)
	begin
		// {{{
		arfifo_rdaddr <= 0;
		S_AXIL_ARVALID <= 0;
		S_AXIL_ARADDR  <= 0;
		// }}}
	end else if ((!S_AXIL_ARVALID || S_AXIL_ARREADY) && (arfifo_fill != 0))
	begin
		// {{{
		S_AXIL_ARVALID <= 1'b1;
		S_AXIL_ARADDR  <= arfifo[arfifo_rdaddr];
		arfifo_rdaddr  <= arfifo_rdaddr + 1;
		// }}}
	end else if (S_AXIL_ARREADY)
	begin
		// {{{
		S_AXIL_ARVALID <= 1'b0;
		S_AXIL_ARADDR  <= 0;
		// }}}
	end

	// }}}
endmodule

