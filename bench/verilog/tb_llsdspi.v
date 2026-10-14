////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/verilog/tb_llsdspi.v
// Project:	SD-Card controller
// Purpose:	Check the minimum SPI startup clocks before the first byte.
// License:	GPL, v3 or later
//
////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module	tb_llsdspi;
	reg		clk = 0, reset = 1;
	reg	[2:0]	done;
	genvar		gk;

	always #5 clk = !clk;

	initial begin
		repeat (5)
			@(negedge clk);
		reset = 0;
		wait (&done);
		$display("PASS SPI startup clocks");
		$finish;
	end

	initial begin
		#1000000;
		$fatal(1, "SPI startup timeout");
	end

	generate for (gk=0; gk<3; gk=gk+1)
	begin : TEST_STARTUP
		localparam	STARTUP_CLOCKS = (gk == 0) ? 75
						: (gk == 1) ? 128 : 256;
		wire		cs_n, sclk, mosi;
		integer		startup_edges = 0;
		reg	[7:0]	tx_byte = 0;

		llsdspi #(
			.SPDBITS(8), .STARTUP_CLOCKS(STARTUP_CLOCKS),
			.POWERUP_IDLE(1000)
		) dut (
			.i_clk(clk), .i_reset(reset), .i_speed(8'd124),
			.i_cs(1'b1), .i_stb(1'b1), .i_byte(8'h40),
			.o_cs_n(cs_n), .o_sclk(sclk), .o_mosi(mosi),
			.i_miso(1'b1), .i_bus_grant(1'b1),
			.o_stb(), .o_byte(), .o_idle()
		);

		always @(posedge sclk)
		if (!reset && cs_n)
		begin
			if (mosi !== 1'b1)
				$fatal(1, "MOSI low during startup");
			startup_edges = startup_edges + 1;
		end

		initial begin
			done[gk] = 0;
			wait (!reset);
			@(negedge cs_n);
			$display("STARTUP_CLOCKS=%0d observed startup clocks=%0d",
					STARTUP_CLOCKS, startup_edges);
			if (startup_edges < STARTUP_CLOCKS)
				$fatal(1, "Fewer startup clocks than requested");
			repeat (8)
			begin
				@(posedge sclk);
				tx_byte = { tx_byte[6:0], mosi };
			end
			if (tx_byte !== 8'h40)
				$fatal(1, "Incorrect first byte: %02x", tx_byte);
			done[gk] = 1;
		end
	end endgenerate
endmodule
