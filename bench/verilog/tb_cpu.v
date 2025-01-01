////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/tb_cpu.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	A SOC based top-level Verilog configuration.  It's designed
//		for use when testing the ZipCPU software when using the
//	Veri1ator SDIO/eMMC model(s).
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
`timescale 1ns/1ns
// }}}
module	tb_cpu #(
		// {{{
		parameter	ADDRESS_WIDTH        = 28,	//Width in bytes
		parameter	BUS_WIDTH            = 32,
		// Verilator lint_off WIDTH
		parameter [0:0]	OPT_PIPELINED        = 1'b1,
		parameter	OPT_LGICACHE         = 12,
		parameter	OPT_LGDCACHE         = 12,
		parameter	OPT_MPY              = 3,
		parameter [0:0]	OPT_DIV              = 1'b1,
		parameter [0:0]	OPT_SHIFTS           = 1'b1,
		parameter [0:0]	OPT_LOCK             = 1'b1,
		parameter [0:0]	OPT_EARLY_BRANCHING  = 1'b1,
		parameter [0:0]	OPT_LOWPOWER         = 1'b1,
		parameter [0:0]	OPT_DISTRIBUTED_REGS = 1'b1,
		parameter [0:0]	OPT_USERMODE         = 1'b1,
		parameter [0:0]	OPT_CLKGATE          = 1'b1,
		parameter [0:0]	OPT_DBGPORT          = 1'b1,
		parameter [0:0]	OPT_TRACE_PORT       = 1'b1,
		parameter [0:0]	OPT_CIS              = 1'b1,
		parameter [0:0]	OPT_DMA              = 1'b1,
		parameter	MEM_FILE = "cputest",
		parameter	CONSOLE_FILE = "console.txt",
		parameter	LGMEMSZ = ADDRESS_WIDTH-2,
		//
		parameter[0:0]	OPT_SERDES = 1'b0,
		parameter [0:0]	OPT_DDR = 1'b1,
		parameter [0:0]	DUMP_TO_VCD = 1'b0,
		parameter	VCD_FILE = "dump.vcd"
		// Verilator lint_on  WIDTH
		// }}}
	) (
		// {{{
		input	wire	i_clk, i_reset, i_hsclk,
		// Sim control input(s)
		// {{{
		input	wire	i_sim_cyc, i_sim_stb, i_sim_we,
		input	wire	[ADDRESS_WIDTH-2:0]	i_sim_addr,
		input	wire	[31:0]			i_sim_data,
		input	wire	[3:0]			i_sim_sel,
		output	wire				o_sim_stall,
		output	wire				o_sim_ack,
		output	wire	[31:0]			o_sim_data,
		output	wire				o_sim_err,
		// }}}
		input	wire				i_sim_int,
		//
		// "Profiler" support.  This is a simulation only port.
		// {{{
		output	wire				o_prof_stb,
		output	wire	[ADDRESS_WIDTH-1:0]	o_prof_addr,
		output	wire	[31:0]			o_prof_ticks,
		// }}}
		// The SDIO peripheral
		// {{{
		output	wire			sdio_ck,
		//
		output	wire			sdio_cmd_tristate,
		output	wire			sdio_cmd_data,
		input	wire			sdio_cmd_idata,
		//
		output	wire	[3:0]		sdio_io_tristate,
		output	wire	[3:0]		sdio_io_data,
		input	wire	[3:0]		sdio_io_idata,
		//
		output	wire	[31:0]		sdio_debug,
		// }}}
		// A similar eMMC peripheral
		// {{{
		output	wire			emmc_ck,
		input	wire			emmc_ds,
		//
		output	wire			emmc_cmd_tristate,
		output	wire			emmc_cmd_data,
		input	wire			emmc_cmd_idata,
		//
		output	wire	[7:0]		emmc_io_tristate,
		output	wire	[7:0]		emmc_io_data,
		input	wire	[7:0]		emmc_io_idata,
		//
		output	wire	[31:0]		emmc_debug
		// }}}
		// }}}
	);

	// Local declarations
	// {{{
	localparam	WBLSB = $clog2(BUS_WIDTH/8);
	parameter [31:0] RESET_ADDRESS = { {(32-ADDRESS_WIDTH){1'b0}}, MEMORY_ADDR, {(WBLSB){1'b0}} };
	localparam	WAW = ADDRESS_WIDTH-WBLSB;
	parameter [WAW-1:0]	SCOPE_ADDR   = { 4'b0001, {(WAW-4){1'b0}} };
	parameter [WAW-1:0]	CONSOLE_ADDR = { 4'b0010, {(WAW-4){1'b0}} };
	parameter [WAW-1:0]	SDIO_ADDR    = { 4'b0100, {(WAW-4){1'b0}} };
	parameter [WAW-1:0]	EMMC_ADDR    = { 4'b0101, {(WAW-4){1'b0}} };
	parameter [WAW-1:0]	MEMORY_ADDR  = { 2'b01, {(WAW-2){1'b0}} };
	localparam	LGFIFO = 4;

	wire	cpu_int;
	// Verilator lint_off UNUSED
	wire	cpu_halted, cpu_op_stall, cpu_pf_stall, cpu_i_count, cpu_gie;
	// Verilator lint_on  UNUSED

	wire			scope_int;
	wire	[31:0]		cpu_trace;

	wire		dbg_cyc, dbg_stb, dbg_we, dbg_stall, dbg_ack, dbg_err;
	wire	[ADDRESS_WIDTH+1-$clog2(32/8)-1:0]	dbg_addr;
	wire	[31:0]	dbg_data, dbg_idata;
	wire	[3:0]	dbg_sel;

	////////////////////////////////////////////////////////////////////////
	//
	// CPU bus declarations
	// {{{
	wire			cpu_cyc, cpu_stb, cpu_we,
					cpu_ack, cpu_stall, cpu_err;
	wire	[WAW-1:0]	cpu_addr;
	wire	[BUS_WIDTH-1:0]	cpu_data, cpu_idata;
	wire [BUS_WIDTH/8-1:0]	cpu_sel;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// DMA bus declarations
	// {{{
	wire			sddma_cyc, sddma_stb, sddma_we,
					sddma_ack, sddma_stall, sddma_err;
	wire	[WAW-1:0]	sddma_addr;
	wire	[BUS_WIDTH-1:0]	sddma_data, sddma_idata;
	wire [BUS_WIDTH/8-1:0]	sddma_sel;

	wire			emdma_cyc, emdma_stb, emdma_we,
					emdma_ack, emdma_stall, emdma_err;
	wire	[WAW-1:0]	emdma_addr;
	wire	[BUS_WIDTH-1:0]	emdma_data, emdma_idata;
	wire [BUS_WIDTH/8-1:0]	emdma_sel;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Memory bus declarations
	// {{{
	wire	mem_cyc, mem_stb, mem_we, mem_ack, mem_stall, mem_err;
	wire	[WAW-1:0]		mem_addr;
	wire	[BUS_WIDTH-1:0]		mem_data, mem_idata;
	wire	[BUS_WIDTH/8-1:0]	mem_sel;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Console bus declarations
	// {{{
	wire	conw_cyc, conw_stb, conw_we, conw_ack, conw_stall, conw_err;
	wire	[WAW-1:0]		conw_addr;
	wire	[BUS_WIDTH-1:0]		conw_data, conw_idata;
	wire	[BUS_WIDTH/8-1:0]	conw_sel;

	wire	con_cyc, con_stb, con_we, con_ack, con_stall, con_err;
	wire	[ADDRESS_WIDTH-3-$clog2(32/8)-1:0]	con_addr;
	wire	[31:0]	con_data, con_idata;
	wire	[3:0]	con_sel;
	reg	r_con_ack;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDIO & eMMC bus declarations
	// {{{
	wire				sdiow_cyc, sdiow_stb, sdiow_we,
					sdiow_ack, sdiow_stall, sdiow_err;
	wire	[WAW-1:0]		sdiow_addr;
	wire	[BUS_WIDTH-1:0]		sdiow_data, sdiow_idata;
	wire	[BUS_WIDTH/8-1:0]	sdiow_sel;

	wire				sdio_cyc, sdio_stb, sdio_we,
					sdio_ack, sdio_stall, sdio_err;
	wire	[WAW+WBLSB-$clog2(32/8)-5:0]	sdio_addr;
	wire	[31:0]			sdio_data, sdio_idata;
	wire	[3:0]			sdio_sel;

	wire		sdio_int;
	wire		ign_sdio_1p8v;

	wire		sdio_detect;
	assign		sdio_detect = 1;

	////////////////////

	wire				emmcw_cyc, emmcw_stb, emmcw_we,
					emmcw_ack, emmcw_stall, emmcw_err;
	wire	[WAW-1:0]		emmcw_addr;
	wire	[BUS_WIDTH-1:0]		emmcw_data, emmcw_idata;
	wire	[BUS_WIDTH/8-1:0]	emmcw_sel;

	wire				emmc_cyc, emmc_stb, emmc_we,
					emmc_ack, emmc_stall, emmc_err;
	wire	[WAW+WBLSB-$clog2(32/8)-5:0]	emmc_addr;
	wire	[31:0]			emmc_data, emmc_idata;
	wire	[3:0]			emmc_sel;

	wire		emmc_int;
	wire		ign_emmc_1p8v;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Optional SCOPE
	// {{{
	wire	scopew_cyc, scopew_stb, scopew_we, scopew_ack, scopew_stall,
		scopew_err;
	wire	[ADDRESS_WIDTH-$clog2(BUS_WIDTH/8)-1:0]	scopew_addr;
	wire	[BUS_WIDTH-1:0]		scopew_data, scopew_idata;
	wire	[BUS_WIDTH/8-1:0]	scopew_sel;
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// External sim port: Either controls ZipCPU or wide WB bus
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Only required if we are using Verilator.  Other test benches won't
	// use this input port
	wire			sim_cyc, sim_stb, sim_we,
				sim_stall, sim_ack, sim_err;
	wire	[ADDRESS_WIDTH+1-$clog2(32/8)-1:0]	sim_addr;
	wire	[31:0]		sim_data, sim_idata;
	wire	[32/8-1:0]	sim_sel;

	wire			simw_cyc, simw_stb, simw_we,
				simw_stall, simw_ack, simw_err;
	wire	[ADDRESS_WIDTH+1-$clog2(BUS_WIDTH/8)-1:0]	simw_addr;
	wire	[BUS_WIDTH-1:0]	simw_data, simw_idata;
	wire [BUS_WIDTH/8-1:0]	simw_sel;

	wbxbar #(
		// {{{
		.NM(1), .NS(2), .AW(ADDRESS_WIDTH+1-$clog2(32/8)), .DW(32),
		.SLAVE_ADDR({
			{ 1'b0, {(ADDRESS_WIDTH-$clog2(32/8)){1'b0}} },
			{ 1'b1, {(ADDRESS_WIDTH-$clog2(32/8)){1'b0}} }}), // CPU
		.SLAVE_MASK({
			{ 1'b0, {(ADDRESS_WIDTH-$clog2(32/8)){1'b0}} },
			{ 1'b1, {(ADDRESS_WIDTH-$clog2(32/8)){1'b0}} }})  // CPU
		// }}}
	) simxbar (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// One master: the SIM bus input
		// {{{
		.i_mcyc(i_sim_cyc), .i_mstb(i_sim_stb), .i_mwe(i_sim_we),
		.i_maddr(i_sim_addr), .i_mdata(i_sim_data), .i_msel(i_sim_sel),
		//
		.o_mstall(o_sim_stall), .o_mack(o_sim_ack),.o_mdata(o_sim_data),
			.o_merr(o_sim_err),
		// }}}
		// Two slaves: The wide bus the ZipCPU masters, and the ZipCPU's
		// debug port
		// {{{
		.o_scyc({  sim_cyc, dbg_cyc  }),
		.o_sstb({  sim_stb, dbg_stb  }),
		.o_swe({   sim_we,  dbg_we   }),
		.o_saddr({ sim_addr,dbg_addr }),
		.o_sdata({ sim_data,dbg_data }),
		.o_ssel({  sim_sel, dbg_sel  }),
		//
		.i_sstall({ sim_stall, dbg_stall }),
		.i_sack({   sim_ack,   dbg_ack   }),
		.i_sdata({  sim_idata, dbg_idata }),
		.i_serr({   sim_err,   dbg_err   })
		// }}}
		// }}}
	);

	assign	simw_cyc   = sim_cyc;
	assign	simw_we    = sim_we;
	assign	sim_ack    = simw_ack;
	assign	sim_err    = simw_err;

	generate if (BUS_WIDTH == 32)
	begin : NO_EXPAND_SIMBUS
		// {{{
		assign	simw_stb  = sim_stb;
		assign	simw_addr = sim_addr;
		assign	simw_data = sim_data;
		assign	simw_sel  = sim_sel;
		assign	sim_stall = simw_stall;
		assign	sim_idata = simw_idata;
		// }}}
	end else begin : GEN_EXPAND_SIMBUS
		// {{{
		wire			fifo_full, fifo_empty;
		wire	[LGFIFO:0]	fifo_fill;
		wire	[$clog2(BUS_WIDTH/8)-$clog2(32/8)-1:0]	fifo_addr;
		wire	[BUS_WIDTH-1:0]	wide_idata;

		assign	simw_stb   = sim_stb    && !fifo_full;
		assign	sim_stall  = simw_stall ||  fifo_full;

		assign	simw_addr = sim_addr[ADDRESS_WIDTH+1-$clog2(32/8)-1:$clog2(BUS_WIDTH/32)];
		assign	simw_sel  = { sim_sel, {(BUS_WIDTH/8-4){1'b0}} } >> (4*simw_addr[$clog2(BUS_WIDTH/8)-1:2]);
		assign	simw_data = { sim_data, {(BUS_WIDTH-32){1'b0}} } >> (32*simw_addr[$clog2(BUS_WIDTH/8)-1:2]);

		sfifo #(
			// {{{
			.LGFLEN(LGFIFO),
			.OPT_READ_ON_EMPTY(1'b1),
			.BW($clog2(BUS_WIDTH/32))
			// }}}
		) u_simaddr_fifo (
			// {{{
			.i_clk(i_clk), .i_reset(i_reset),
			.i_wr(simw_stb && !sim_stall),
			.i_data(simw_addr[$clog2(BUS_WIDTH/8)-1:2]),
			.o_full(fifo_full), .o_fill(fifo_fill),
			.i_rd(simw_ack), .o_data(fifo_addr),
			.o_empty(fifo_empty)
			// }}}
		);

		assign	wide_idata = simw_idata << (32*fifo_addr);
		assign	sim_idata  = wide_idata[BUS_WIDTH-1:BUS_WIDTH-32];

		// Verilator lint_off UNUSED
		wire	unused_sim_expander;
		assign	unused_sim_expander = &{ 1'b0,
			fifo_fill, fifo_empty, wide_idata[BUS_WIDTH-32-1:0],
			sim_addr[$clog2(BUS_WIDTH/32):0] };
		// Verilator lint_on  UNUSED
		// }}}
	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// The CPU itself
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	localparam		RESET_DURATION = 10;
	localparam	[0:0]	OPT_SIM = 1'b1;
`ifdef	VERILATOR
	localparam	[0:0]	OPT_PROFILER = 1'b1;
`else
	localparam	[0:0]	OPT_PROFILER = 1'b0;
`endif

	zipsystem #(
		// {{{
		.RESET_ADDRESS(RESET_ADDRESS),
		.ADDRESS_WIDTH(ADDRESS_WIDTH),
		.BUS_WIDTH(BUS_WIDTH),
		.OPT_PIPELINED(OPT_PIPELINED),
		.OPT_EARLY_BRANCHING(OPT_EARLY_BRANCHING),
		.OPT_LGICACHE(OPT_LGICACHE),
		.OPT_LGDCACHE(OPT_LGDCACHE),
		.START_HALTED(1'b0),
		.OPT_DISTRIBUTED_REGS(OPT_DISTRIBUTED_REGS),
		.OPT_MPY(OPT_MPY),
		.OPT_DIV(OPT_DIV),
		.OPT_SHIFTS(OPT_SHIFTS),
		.OPT_LOCK(OPT_LOCK),
		.OPT_CIS(OPT_CIS),
		.OPT_USERMODE(OPT_USERMODE),
		.OPT_DBGPORT(OPT_DBGPORT),
		.OPT_TRACE_PORT(OPT_TRACE_PORT),
		.OPT_PROFILER(OPT_PROFILER),
		.OPT_LOWPOWER(OPT_LOWPOWER),
		.OPT_SIM(OPT_SIM),
		.OPT_CLKGATE(OPT_CLKGATE),
		.RESET_DURATION(RESET_DURATION),
		// ZipSystem only parameters
		.OPT_DMA(OPT_DMA),
		.OPT_ACCOUNTING(1'b1),
		.EXTERNAL_INTERRUPTS(4)
		// }}}
	) u_cpu (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// Master bus
		// {{{
		.o_wb_cyc(cpu_cyc), .o_wb_stb(cpu_stb),
		.o_wb_we(cpu_we),
		.o_wb_addr(cpu_addr[WAW-1:0]), .o_wb_data(cpu_data[BUS_WIDTH-1:0]),
			.o_wb_sel(cpu_sel[BUS_WIDTH/8-1:0]),
		.i_wb_stall(cpu_stall), .i_wb_ack(cpu_ack),
			.i_wb_data(cpu_idata[BUS_WIDTH-1:0]),
			.i_wb_err(cpu_err),
		// }}}
		.i_ext_int({ emmc_int, sdio_int, scope_int, i_sim_int }),
		.o_ext_int(cpu_int),
		// Debug control port
		// {{{
		.i_dbg_cyc(dbg_cyc), .i_dbg_stb(dbg_stb),
			.i_dbg_we(dbg_we), .i_dbg_addr(dbg_addr[6:0]),
			.i_dbg_data(dbg_data), .i_dbg_sel(dbg_sel),
		.o_dbg_stall(dbg_stall), .o_dbg_ack(dbg_ack),
			.o_dbg_data(dbg_idata),
		// }}}
		.o_cpu_debug(cpu_trace),
		// (Optional) Profiler
		// {{{
		.o_prof_stb(o_prof_stb),
		.o_prof_addr(o_prof_addr),
		.o_prof_ticks(o_prof_ticks)
		// }}}
		// }}}
	);

	assign	cpu_halted   = u_cpu.cpu_has_halted;
	assign	cpu_op_stall = u_cpu.cpu_op_stall;
	assign	cpu_pf_stall = u_cpu.cpu_pf_stall;
	assign	cpu_i_count  = u_cpu.cpu_i_count;
	assign	cpu_gie      = u_cpu.cpu_gie;

	assign	dbg_err = 1'b0;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// The wide bus interconnect
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Verilator tracing_off
	wbxbar #(
		// {{{
`ifdef	VERILATOR
		.NM(1+3),
`else
		.NM(3),
`endif
		.NS(5),
		.AW(ADDRESS_WIDTH-$clog2(BUS_WIDTH/8)), .DW(BUS_WIDTH),
		.SLAVE_ADDR({
			EMMC_ADDR, SDIO_ADDR,
			CONSOLE_ADDR,
			SCOPE_ADDR,
			MEMORY_ADDR }),
		.SLAVE_MASK({
			{ 4'b1111, {(WAW-4){1'b0}} },	// eMMC
			{ 4'b1111, {(WAW-4){1'b0}} },	// SDIO
			{ 4'b1111, {(WAW-4){1'b0}} },	// Console
			{ 4'b1111, {(WAW-4){1'b0}} },	// Scope
			{ 2'b11,   {(WAW-2){1'b0}} } })	// Memory
		// }}}
	) main_crossbar (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// Slave ports from the various bus masters
		// {{{
`ifdef	VERILATOR
		// Two bus masters: the external SIM input, and the CPU
		.i_mcyc({   emdma_cyc, sddma_cyc, simw_cyc,   cpu_cyc }),
		.i_mstb({   emdma_stb, sddma_stb, simw_stb,   cpu_stb }),
		.i_mwe({    emdma_we,  sddma_we, simw_we,    cpu_we }),
		.i_maddr({  emdma_addr, sddma_addr, simw_addr[ADDRESS_WIDTH-WBLSB-1:0], cpu_addr }),
		.i_mdata({  emdma_data, sddma_data, simw_data,  cpu_data }),
		.i_msel({   emdma_sel, sddma_sel, simw_sel,   cpu_sel }),
		.o_mstall({ emdma_stall, sddma_stall, simw_stall, cpu_stall }),
		.o_mack({   emdma_ack, sddma_ack, simw_ack,   cpu_ack }),
		.o_mdata({  emdma_idata, sddma_idata, simw_idata, cpu_idata }),
		.o_merr({   emdma_err, sddma_err, simw_err,   cpu_err }),
`else
		// With no external CPU input, there is no simulation port
		.i_mcyc({ emdma_cyc, sddma_cyc, cpu_cyc }),
			.i_mstb({  emdma_stb,  sddma_stb,  cpu_stb }),
			.i_mwe({   emdma_we,   sddma_we,   cpu_we }),
			.i_maddr({ emdma_addr, sddma_addr, cpu_addr }),
			.i_mdata({ emdma_data, sddma_data, cpu_data }),
			.i_msel({  emdma_sel,  sddma_sel,  cpu_sel }),
		.o_mstall({ emdma_stall, sddma_stall, cpu_stall }),
			.o_mack({  emdma_ack,   sddma_ack,   cpu_ack }),
			.o_mdata({ emdma_idata, sddma_idata, cpu_idata }),
			.o_merr({  emdma_err,   sddma_err,   cpu_err }),
`endif
		// }}}
		// Master port ... to control the slaves w/in this design
		// {{{
		.o_scyc({    emmcw_cyc,  sdiow_cyc, conw_cyc,  scopew_cyc,  mem_cyc  }),
		.o_sstb({    emmcw_stb,  sdiow_stb, conw_stb,  scopew_stb,  mem_stb  }),
		.o_swe({      emmcw_we,   sdiow_we,  conw_we,   scopew_we,   mem_we   }),
		.o_saddr({  emmcw_addr, sdiow_addr,conw_addr, scopew_addr, mem_addr }),
		.o_sdata({  emmcw_data, sdiow_data, conw_data, scopew_data, mem_data }),
		.o_ssel({    emmcw_sel,  sdiow_sel,  conw_sel,  scopew_sel,  mem_sel  }),
		//
		.i_sstall({emmcw_stall,sdiow_stall,conw_stall, scopew_stall, mem_stall }),
		.i_sack({    emmcw_ack,  sdiow_ack,  conw_ack,   scopew_ack,   mem_ack   }),
		.i_sdata({ emmcw_idata,sdiow_idata,conw_idata, scopew_idata, mem_idata }),
		.i_serr({    emmcw_err,  sdiow_err,  conw_err,   scopew_err,   mem_err   })
		// }}}
		// }}}
	);
	// Verilator tracing_on

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Memory
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Verilator tracing_off
	memdev #(
		// {{{
		.LGMEMSZ(LGMEMSZ),
		.DW(BUS_WIDTH),
		.HEXFILE(MEM_FILE),
		.OPT_ROM(1'b0)
		// }}}
	) u_mem (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		.i_wb_cyc(mem_cyc), .i_wb_stb(mem_stb), .i_wb_we(mem_we),
		.i_wb_addr(mem_addr[LGMEMSZ-WBLSB-1:0]), .i_wb_data(mem_data),
			.i_wb_sel(mem_sel),
		.o_wb_stall(mem_stall), .o_wb_ack(mem_ack),
		.o_wb_data(mem_idata)
		// }}}
	);
	// Verilator tracing_on

	assign	mem_err = 1'b0;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Console
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg		con_write_en;
	reg	[7:0]	con_write_byte;

	integer	sim_console;

	// Verilator tracing_off
	wbdown #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH-3),
		.WIDE_DW(BUS_WIDTH), .SMALL_DW(32)
		// }}}
	) u_condown (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// The "Wide" connection
		// {{{
		.i_wcyc(conw_cyc), .i_wstb(conw_stb), .i_wwe(conw_we),
		.i_waddr(conw_addr[WAW-4:0]),
			.i_wdata(conw_data), .i_wsel(conw_sel),
		//
		.o_wstall(conw_stall), .o_wack(conw_ack), .o_wdata(conw_idata),
			.o_werr(conw_err),
		// }}}
		// The downsized connection
		// {{{
		.o_cyc(con_cyc), .o_stb(con_stb), .o_we(con_we),
		.o_addr(con_addr), .o_data(con_data), .o_sel(con_sel),
		//
		.i_stall(con_stall), .i_ack(con_ack), .i_data(con_idata),
			.i_err(con_err)
		// }}}
		// }}}
	);
	// Verilator tracing_on

	assign	con_stall = 1'b0;

	initial	r_con_ack = 1'b0;
	always @(posedge i_clk)
		r_con_ack <= !i_reset && con_stb;
	assign	con_ack = r_con_ack;

	initial	begin
		sim_console = $fopen(CONSOLE_FILE);
	end

	// Make sure we can read the outgoing console data from the trace
	initial	con_write_en = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		con_write_en <= 1'b0;
	else if (con_stb && con_we && con_addr[1:0] == 2'b11 && con_sel[0])
		con_write_en <= 1'b1;
	else
		con_write_en <= 1'b0;

	initial	con_write_byte = 8'h0;
	always @(posedge i_clk)
	if (con_stb && con_we && con_addr[1:0] == 2'b11 && con_sel[0])
		con_write_byte <= con_data[7:0];

	always @(posedge i_clk)
	if (!i_reset && con_write_en)
	begin
		$fwrite(sim_console, "%1s", con_write_byte);
		$write("%1s", con_write_byte);
	end

	assign	con_idata = 32'h0;
	assign	con_err   = 1'b0;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Test Stream generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	reg		sdrx_valid;
	wire		sdrx_ready;
	reg	[31:0]	sdrx_data;

	wire		sdtx_valid, sdtx_last;
	reg		sdtx_ready, sdtx_start;
	wire	[31:0]	sdtx_data;
	reg	[31:0]	sdtx_prior;

	reg		emrx_valid;
	wire		emrx_ready;
	reg	[31:0]	emrx_data;

	wire		emtx_valid, emtx_last;
	reg		emtx_ready, emtx_start;
	wire	[31:0]	emtx_data;
	reg	[31:0]	emtx_prior;

	// SDRX (Stream data to the SD card)
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		sdrx_valid <= 1'b0;
	else
		sdrx_valid <= 1'b1;

	always @(posedge i_clk)
	if (i_reset)
		sdrx_data <= 32'b1;
	else if (sdrx_valid && sdrx_ready)
	begin
		if (sdrx_data == 0)
			sdrx_data <= 32'h1;
		else if (sdrx_data[18])
			sdrx_data <= (sdrx_data << 1) ^ 32'h72d31;
		else
			sdrx_data <= (sdrx_data << 1);
	end
	// }}}

	// SD-TX (from the SD card)
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		sdtx_ready <= 1'b1;
	else if (sdtx_valid && !sdtx_ready)
		sdtx_ready <= 1'b1;
	else if (sdtx_valid && sdtx_ready)
		sdtx_ready <= sdtx_data[30];

	always @(posedge i_clk)
	if (i_reset)
		sdtx_start <= 1'b1;
	else if (sdtx_valid && sdtx_ready)
	begin
		sdtx_start <= sdtx_last;
	end

	always @(posedge i_clk)
	if (sdtx_valid && sdtx_ready)
		sdtx_prior <= sdtx_data;

	always @(posedge i_clk)
	if (sdtx_valid && sdtx_ready && !sdtx_start)
	begin
		if (sdtx_prior[18])
		begin
			assert(sdtx_data == ({ sdtx_prior[30:0], 1'b0 }
								^ 32'h72d31));
		end else begin
			assert(sdtx_data == { sdtx_prior[30:0], 1'b0 });
		end
	end
	// }}}

	// EMRX (Stream data to the eMMC card)
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		emrx_valid <= 1'b0;
	else
		emrx_valid <= 1'b1;

	always @(posedge i_clk)
	if (i_reset)
		emrx_data <= 32'b1;
	else if (emrx_valid && emrx_ready)
	begin
		if (emrx_data == 0)
			emrx_data <= 32'h1;
		else if (emrx_data[18])
			emrx_data <= (emrx_data << 1) ^ 32'h68c79;
		else
			emrx_data <= (emrx_data << 1);
	end
	// }}}

	// EMTX (from the eMMC card)
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		emtx_ready <= 1'b1;
	else if (emtx_valid && !emtx_ready)
		emtx_ready <= 1'b1;
	else if (emtx_valid && emtx_ready)
		emtx_ready <= emtx_data[30];

	always @(posedge i_clk)
	if (i_reset)
		emtx_start <= 1'b1;
	else if (emtx_valid && emtx_ready)
		emtx_start <= emtx_last;

	always @(posedge i_clk)
	if (emtx_valid && emtx_ready)
		emtx_prior <= emtx_data;

	always @(posedge i_clk)
	if (emtx_valid && emtx_ready && !emtx_start)
	begin
		if (emtx_prior[18])
		begin
			assert(emtx_data == ({ emtx_prior[30:0], 1'b0 }
								^ 32'h72d31));
		end else begin
			assert(emtx_data == { emtx_prior[30:0], 1'b0 });
		end
	end
	// }}}

	// Verilator lint_off UNUSED
	wire	unused_stream = &{ 1'b0, emtx_prior[31], sdtx_prior[31] };
	// Verilator lint_on  UNUSED
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SDIO controller
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	wbdown #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH-4),
		.WIDE_DW(BUS_WIDTH), .SMALL_DW(32)
		// }}}
	) u_sdiodown (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// The "Wide" connection
		// {{{
		.i_wcyc(sdiow_cyc), .i_wstb(sdiow_stb), .i_wwe(sdiow_we),
		.i_waddr(sdiow_addr[WAW-5:0]),
			.i_wdata(sdiow_data), .i_wsel(sdiow_sel),
		//
		.o_wstall(sdiow_stall), .o_wack(sdiow_ack),
			.o_wdata(sdiow_idata), .o_werr(sdiow_err),
		// }}}
		// The downsized connection
		// {{{
		.o_cyc(sdio_cyc), .o_stb(sdio_stb), .o_we(sdio_we),
		.o_addr(sdio_addr), .o_data(sdio_data), .o_sel(sdio_sel),
		//
		.i_stall(sdio_stall), .i_ack(sdio_ack), .i_data(sdio_idata),
			.i_err(sdio_err)
		// }}}
		// }}}
	);

	sdio_top #(
		// {{{
		.LGFIFO(9), .NUMIO(4), .ADDRESS_WIDTH(ADDRESS_WIDTH),
		.DW(BUS_WIDTH), .SW(32),
		.OPT_DMA(OPT_DMA),
		.OPT_LITTLE_ENDIAN(1'b0),
		.OPT_ISTREAM(1'b1),
		.OPT_OSTREAM(1'b1),
		.OPT_EMMC(1'b0),
		.MW(BUS_WIDTH),
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR), .OPT_CARD_DETECT(1'b1),
		.LGTIMEOUT(18)
		// }}}
	) u_sdio (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset), .i_hsclk(i_hsclk),
		// Control interface
		// {{{
		.i_wb_cyc(sdio_cyc), .i_wb_stb(sdio_stb), .i_wb_we(sdio_we),
		.i_wb_addr(sdio_addr[2:0]), .i_wb_data(sdio_data),
			.i_wb_sel(sdio_sel),
		.o_wb_stall(sdio_stall), .o_wb_ack(sdio_ack),
			.o_wb_data(sdio_idata),
		// }}}
		// DMA interface
		// {{{
		.o_dma_cyc(sddma_cyc),
		.o_dma_stb(sddma_stb),
		.o_dma_we(sddma_we),
		.o_dma_addr(sddma_addr),
		.o_dma_data(sddma_data),
		.o_dma_sel(sddma_sel),
		.i_dma_stall(sddma_stall),
		.i_dma_ack(sddma_ack),
		.i_dma_data(sddma_idata),
		.i_dma_err(sddma_err),
		// }}}
		// External DMA streaming interface
		// {{{
		.s_valid(sdrx_valid),
		.s_ready(sdrx_ready),
		.s_data(sdrx_data),
		//
		.m_valid(sdtx_valid),
		.m_ready(sdtx_ready),
		.m_data(sdtx_data),
		.m_last(sdtx_last),
		// }}}
		// IO interface
		// {{{
		.o_ck(sdio_ck), .i_ds(1'b0),
		//
		.io_cmd_tristate(sdio_cmd_tristate),
		.o_cmd(sdio_cmd_data),
		.i_cmd(sdio_cmd_idata),
		//
		.io_dat_tristate(sdio_io_tristate),
		.o_dat(sdio_io_data),
		.i_dat(sdio_io_idata),
		// }}}
		.i_card_detect(sdio_detect), .o_1p8v(ign_sdio_1p8v),
			.o_int(sdio_int),
		.o_debug(sdio_debug)
		// }}}
	);

	assign	sdio_err = 1'b0;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// eMMC controller
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	wbdown #(
		// {{{
		.ADDRESS_WIDTH(ADDRESS_WIDTH-4),
		.WIDE_DW(BUS_WIDTH), .SMALL_DW(32)
		// }}}
	) u_emmcdown (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset),
		// The "Wide" connection
		// {{{
		.i_wcyc(emmcw_cyc), .i_wstb(emmcw_stb), .i_wwe(emmcw_we),
		.i_waddr(emmcw_addr[WAW-5:0]),
			.i_wdata(emmcw_data), .i_wsel(emmcw_sel),
		//
		.o_wstall(emmcw_stall), .o_wack(emmcw_ack),
			.o_wdata(emmcw_idata), .o_werr(emmcw_err),
		// }}}
		// The downsized connection
		// {{{
		.o_cyc(emmc_cyc), .o_stb(emmc_stb), .o_we(emmc_we),
		.o_addr(emmc_addr), .o_data(emmc_data), .o_sel(emmc_sel),
		//
		.i_stall(emmc_stall), .i_ack(emmc_ack), .i_data(emmc_idata),
			.i_err(emmc_err)
		// }}}
		// }}}
	);

	sdio_top #(
		// {{{
		.LGFIFO(9), .NUMIO(8), .ADDRESS_WIDTH(ADDRESS_WIDTH),
		.DW(BUS_WIDTH), .SW(32),
		.OPT_DMA(OPT_DMA),
		.OPT_LITTLE_ENDIAN(1'b0),
		.OPT_ISTREAM(1'b1),
		.OPT_OSTREAM(1'b1),
		.OPT_EMMC(1'b1),
		.MW(BUS_WIDTH),
		.OPT_SERDES(OPT_SERDES), .OPT_DDR(OPT_DDR), .OPT_CARD_DETECT(1'b0),
		.LGTIMEOUT(18)
		// }}}
	) u_emmc (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset), .i_hsclk(i_hsclk),
		// Control interface
		// {{{
		.i_wb_cyc(emmc_cyc), .i_wb_stb(emmc_stb), .i_wb_we(emmc_we),
		.i_wb_addr(emmc_addr[2:0]), .i_wb_data(emmc_data),
			.i_wb_sel(emmc_sel),
		.o_wb_stall(emmc_stall), .o_wb_ack(emmc_ack),
			.o_wb_data(emmc_idata),
		// }}}
		// DMA interface
		// {{{
		.o_dma_cyc(emdma_cyc),
		.o_dma_stb(emdma_stb),
		.o_dma_we(emdma_we),
		.o_dma_addr(emdma_addr),
		.o_dma_data(emdma_data),
		.o_dma_sel(emdma_sel),
		.i_dma_stall(emdma_stall),
		.i_dma_ack(emdma_ack),
		.i_dma_data(emdma_idata),
		.i_dma_err(emdma_err),
		// }}}
		// External DMA streaming interface
		// {{{
		.s_valid(emrx_valid),
		.s_ready(emrx_ready),
		.s_data(emrx_data),
		//
		.m_valid(emtx_valid),
		.m_ready(emtx_ready),
		.m_data(emtx_data),
		.m_last(emtx_last),
		// }}}
		// IO interface
		// {{{
		.o_ck(emmc_ck), .i_ds(emmc_ds),
		//
		.io_cmd_tristate(emmc_cmd_tristate),
		.o_cmd(emmc_cmd_data),
		.i_cmd(emmc_cmd_idata),
		//
		.io_dat_tristate(emmc_io_tristate),
		.o_dat(emmc_io_data),
		.i_dat(emmc_io_idata),
		// }}}
		.i_card_detect(1'b1), .o_1p8v(ign_emmc_1p8v),
			.o_int(emmc_int),
		.o_debug(emmc_debug)
		// }}}
	);

	assign	emmc_err = 1'b0;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// (Optional) WB Scope
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Verilator tracing_off
	generate if (OPT_TRACE_PORT)
	begin : GEN_WBSCOPE
		// {{{
		wire		scope_cyc, scope_stb, scope_we;
		// Verilator lint_off UNUSED
		wire	[ADDRESS_WIDTH-3-$clog2(32/8)-1:0]	scope_addr;
		// Verilator lint_on  UNUSED
		wire	[31:0]	scope_data, scope_idata;
		wire	[3:0]	scope_sel;
		wire		scope_stall, scope_ack, scope_err;

		wbdown #(
			// {{{
			.ADDRESS_WIDTH(ADDRESS_WIDTH-3),
			.WIDE_DW(BUS_WIDTH), .SMALL_DW(32)
			// }}}
		) u_scopedown (
			// {{{
			.i_clk(i_clk), .i_reset(i_reset),
			// The "Wide" connection
			// {{{
			.i_wcyc(scopew_cyc), .i_wstb(scopew_stb), .i_wwe(scopew_we),
			.i_waddr(scopew_addr[WAW-4:0]),
				.i_wdata(scopew_data), .i_wsel(scopew_sel),
			//
			.o_wstall(scopew_stall), .o_wack(scopew_ack),
				.o_wdata(scopew_idata), .o_werr(scopew_err),
			// }}}
			// The downsized connection
			// {{{
			.o_cyc(scope_cyc), .o_stb(scope_stb), .o_we(scope_we),
			.o_addr(scope_addr), .o_data(scope_data),
				.o_sel(scope_sel),
			//
			.i_stall(scope_stall), .i_ack(scope_ack),
				.i_data(scope_idata), .i_err(scope_err)
			// }}}
			// }}}
		);

		wbscope #(
			.LGMEM(12)
		) u_scope (
			// {{{
			.i_data_clk(i_clk), .i_trigger(1'b0), .i_ce(1'b1),
			.i_data(cpu_trace),
			//
			.i_wb_clk(i_clk),
			.i_wb_cyc(scope_cyc), .i_wb_stb(scope_stb),
			.i_wb_we(scope_we),   .i_wb_addr(scope_addr[0]),
			.i_wb_data(scope_data), .i_wb_sel(scope_sel),
			//
			.o_wb_stall(scope_stall),
			.o_wb_ack(scope_ack),
			.o_wb_data(scope_idata),
			//
			.o_interrupt(scope_int)
			// }}}
		);

		assign	scope_err = 1'b0;
		// }}}
	end else begin : NO_SCOPE
		// {{{
		reg	r_scope_ack;

		initial	r_scope_ack = 1'b0;
		always @(posedge i_clk)
			r_scope_ack <= scopew_stb && !i_reset;

		assign	scopew_stall = 1'b0;
		assign	scopew_ack   = r_scope_ack;
		assign	scopew_idata = {(BUS_WIDTH){1'b0}};
		assign	scopew_err   = 1'b0;

		assign	scope_int = 1'b0;

		// Verilator coverage_off
		// Verilator lint_off UNUSED
		wire	unused_scope;
		assign	unused_scope = &{ 1'b0, scopew_cyc, scopew_we,
					scopew_data, scopew_sel, cpu_trace };
		// Verilator lint_on  UNUSED
		// Verilator coverage_on

		// }}}
	end endgenerate
	// Verilator tracing_on

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// (Optional) VCD generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
`ifndef	VERILATOR
	initial if (DUMP_TO_VCD)
	begin
		$dumpfile(VCD_FILE);
		$dumpvars(0, wb_tb);
	end
`endif
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Test bench watchdog
	// {{{

	localparam	TB_WATCHDOG_TIMEOUT = 1_000_00;	// 1ms
	reg	[$clog2(TB_WATCHDOG_TIMEOUT+2)-1:0]	watchdog_counter;

	initial	watchdog_counter = 0;
	always @(posedge i_clk)
	// if (i_reset)
	//	watchdog_counter <= 0;
	// else
	if (|(cpu_stb & ~cpu_stall))
		watchdog_counter <= 0;
	else
		watchdog_counter <= watchdog_counter + 1;

	always @(posedge i_clk)
	if (watchdog_counter > TB_WATCHDOG_TIMEOUT)
	begin
		$display("\nERROR: Watchdog timeout!");
		$finish;
	end

	always @(posedge i_clk)
	if (!i_reset && cpu_int)
		$display("\nCPU Halted without error: PASS\n");
	// }}}

	// Keep Verilator happy
	// {{{
	// Verilator coverage_off
	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0,
		DUMP_TO_VCD, VCD_FILE,
		conw_addr, scopew_addr,
`ifdef	VERILATOR
		simw_addr,
`endif
		sdiow_addr, emmcw_addr, 	// Unused bits
		sdio_addr, emmc_addr,
		ign_sdio_1p8v, ign_emmc_1p8v,
		con_addr, mem_addr, dbg_addr,
		con_cyc, con_data[31:8], con_sel[3:1], scope_int };
	// Verilator lint_on  UNUSED
	// Verilator coverage_on
	// }}}
endmodule
