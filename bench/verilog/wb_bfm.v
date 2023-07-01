////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	wb_bfm.v
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	Drive a Wishbone bus via commands from a test script.
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
module	wb_bfm #(
		parameter AW = 5,
		parameter DW = 32,
		parameter LGFIFO = 4,
		parameter [0:0]	OPT_DEBUG = 1'b0
	) (
		// {{{
		input	wire			i_clk, i_reset,
		//
		output	reg			o_wb_cyc, o_wb_stb, o_wb_we,
		output	reg	[AW-1:0]	o_wb_addr,
		output	reg	[DW-1:0]	o_wb_data,
		output	reg	[DW/8-1:0]	o_wb_sel,
		input	wire			i_wb_stall,
		input	wire			i_wb_ack,
		input	wire	[DW-1:0]	i_wb_data,
		input	wire			i_wb_err
		// }}}
	);

	// Local declarations
	// {{{
	localparam	WBLSB = $clog2(DW/8);
	localparam	ADDR_WIDTH = AW + WBLSB;

	reg	[31:0]		bus_outstanding, req_outstanding;
	wire	[31:0]		bus_committed;
	reg	[LGFIFO:0]	fifo_fill, fifo_rdaddr, fifo_wraddr;
	reg	[1+(DW/8)+AW+DW-1:0]	fifo	[0:((1<<LGFIFO)-1)];
	wire			bus_full;

	reg			next_we;
	reg	[DW/8-1:0]	next_sel;
	reg	[AW-1:0]	next_addr;
	reg	[DW-1:0]	next_dat;
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
		wait(!i_reset);

		while((fifo_wraddr[LGFIFO-1:0] == fifo_rdaddr[LGFIFO-1:0])
				&&(fifo_wraddr[LGFIFO] != fifo_rdaddr[LGFIFO]))
		begin
			@(posedge i_clk);
		end

		fifo[fifo_wraddr[LGFIFO-1:0]] = { 1'b1,4'hf, addr[ADDR_WIDTH-1:WBLSB],dat };
		fifo_wraddr = fifo_wraddr + 1;
	end endtask
	// }}}

	task write_f(input [ADDR_WIDTH-1:0] addr, input [DW-1:0] dat);
		// {{{
	begin
		write_f(addr, dat);

		// Now wait for the last read to finish
		while(o_wb_cyc || fifo_fill == 0)
			@(posedge i_clk);

	end endtask
	// }}}

	task readio(input [ADDR_WIDTH-1:0] addr, output [DW-1:0] dat);
		reg	returned, err_flag;
		reg	[LGFIFO:0]	read_busaddr;
	begin
		err_flag = 1'b0;

		while((fifo_wraddr[LGFIFO-1:0] == fifo_rdaddr[LGFIFO-1:0])
				&&(fifo_wraddr[LGFIFO] != fifo_rdaddr[LGFIFO]))
		begin
			@(posedge i_clk);
		end

		fifo[fifo_wraddr[LGFIFO-1:0]] = { 1'b0, {(DW/8){1'b1}},
			addr[ADDR_WIDTH-1:WBLSB], {(DW){1'b0}} };
		fifo_wraddr = fifo_wraddr + 1;
		read_busaddr= fifo_wraddr;


		do begin
			@(posedge i_clk)
				err_flag <= (o_wb_cyc && i_wb_err);
			wait(!i_clk);
		end while(!err_flag && (!o_wb_cyc || fifo_rdaddr != read_busaddr));

		do begin
			@(posedge i_clk)
			begin
				if (o_wb_cyc && i_wb_ack)
					dat <= i_wb_data;
				if (o_wb_cyc && i_wb_err)
					err_flag <= 1'b1;
			end wait(!i_clk);
		end while(!err_flag && o_wb_cyc);

		if (OPT_DEBUG)
			$display("BFM:READ  @0x%04x --> %08x", addr, dat);
	end endtask

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Issue requests
	// {{{
	assign	bus_committed = bus_outstanding + (o_wb_stb ? 1:0);
	assign	bus_full = (&bus_committed);

	always @(posedge i_clk)
	if (i_reset || !o_wb_cyc || i_wb_err)
		bus_outstanding <= 0;
	else case({ (o_wb_stb && !i_wb_stall), i_wb_ack })
	2'b10: bus_outstanding <= bus_outstanding + 1;
	2'b01: bus_outstanding <= (bus_outstanding > 0)
					? (bus_outstanding - 1) : 0;
	default: begin end
	endcase

	always @(*)
		{ next_we, next_sel, next_addr, next_dat } = fifo[fifo_rdaddr[LGFIFO-1:0]];
	always @(*)
		fifo_fill = fifo_wraddr - fifo_rdaddr;

	always @(*)
		req_outstanding = fifo_fill + bus_outstanding;

	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		fifo_rdaddr <= 0;
		o_wb_cyc  <= 0;
		o_wb_stb  <= 0;
		o_wb_we   <= 0;
		o_wb_addr <= 0;
		o_wb_data <= 0;
		o_wb_sel  <= 0;
		// }}}
	end else if (o_wb_cyc && i_wb_err)
	begin
		// {{{
		fifo_rdaddr <= fifo_wraddr;
		o_wb_cyc  <= 0;
		o_wb_stb  <= 0;
		o_wb_we   <= 0;
		o_wb_addr <= 0;
		o_wb_data <= 0;
		o_wb_sel  <= 0;
		// }}}
	end else if ((!o_wb_stb || !i_wb_stall) && (fifo_rdaddr != fifo_wraddr)
			&& (!o_wb_cyc || (o_wb_we == next_we)))
	begin
		// {{{
		o_wb_cyc  <= 1'b1;
		o_wb_we   <= next_we;
		o_wb_addr <= next_addr;
		o_wb_data <= next_dat;
		o_wb_sel  <= next_sel;

		if (!bus_full)
		begin
			o_wb_stb  <= 1'b1;
			fifo_rdaddr <= fifo_rdaddr + 1;
		end else begin
			o_wb_stb  <= 1'b0;
		end
		// }}}
	end else begin
		// {{{
		if (!i_wb_stall)
			o_wb_stb <= 1'b0;

		if (o_wb_cyc && i_wb_ack)
		begin

			if (bus_outstanding == 1 && !o_wb_stb)
				o_wb_cyc <= 1'b0;
			else if (bus_outstanding == 0 && o_wb_stb && !i_wb_stall)
				o_wb_cyc <= 1'b0;
		end

		if (bus_outstanding == 0 && !o_wb_stb)
		begin
			o_wb_cyc  <= 1'b0;
			o_wb_stb  <= 1'b0;
			o_wb_we   <= 1'b0;
			o_wb_addr <= 0;
			o_wb_data <= 0;
			o_wb_sel  <= 0;
		end
		// }}}
	end
	// }}}
endmodule

