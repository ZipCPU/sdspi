////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/testscript/emmcboot.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	Demonstrate the eMMC BOOT capability.
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
`include "../testscript/emmclib.v"
// }}}

task	testscript;
	reg	[31:0]	read_data, src_data, mem_data, b_addr, phy_data;
	integer		iw;
begin
	@(posedge clk);
	while(reset !== 1'b0)
		@(posedge clk);
	@(posedge clk);

	b_addr = 32'h0;
	b_addr[ADDRESS_WIDTH-1:0] = BOOT_ADDR;

	////////////////////////////////////////////////////////////////////////
	//
	// Boot method #1: Automatic boot on startup
	// {{{
	////////////////////////////////////////////////////////////////////////
	$display("BOOT TEST #1: Automatic boot following a system RESET");

	u_mcchip.randmize_boot;
	if (OPT_DMA)
	begin
		$display("  Waiting ...");
		wait(emmc_interrupt);
	end else begin
		loading = 2'b00;
		b_addr = 32'h0;
		b_addr[ADDRESS_WIDTH-1:0] = BOOT_ADDR;

		// Request the first block
		// {{{
		if (blk == 0)
		begin
			read_data = 32'h0 | EMMC_MEM;
			u_bfm.writeio(ADDR_SDCARD, read_data);
		end
		// }}}

		for(blk=0; 1'b0 === error_flag
				&& blk<(1<<(EMMC_LGBOOTSZ-9)); blk=blk+1)
		begin
			// Wait for a block to be ready
			// {{{
			wait(emmc_interrupt);

			u_bfm.readio(ADDR_SDCARD, read_data);
			while(read_data & EMMC_MEM)
				u_bfm.readio(ADDR_SDCARD, read_data);

			error_flag = error_flag || read_data[15];
			// }}}

			// (Possibly) Request the next block
			// {{{
			if (blk + 1 < (1<<(EMMC_LGBOOTSZ-9)) )
			begin
				read_data = 32'h0 | EMMC_FIFO;
				if (blk[0] == 1'b0)
					read_data = read_data | EMMC_FIFO;
				u_bfm.writeio(ADDR_SDCARD, read_data);
			end
			// }}}

			// Transfer data
			// {{{
			for(iw=0; 1'b0 === error_flag && iw<(1<<7); iw=iw+1)
			begin
				u_bfm.readio(ADDR_FIFOA + (blk[0]? 4:0),
						read_data);
				u_bfm.writeio(b_addr + { iw, 2'b00 },
						read_data);
			end
			b_addr = b_addr + 512;
			// }}}
		end

		b_addr = 32'h0;
		b_addr[ADDRESS_WIDTH-1:0] = BOOT_ADDR;
	end

	u_bfm.readio(ADDR_SDCARD, read_data);
	$display("  CMD: 0x%08x", read_data);

	if (read_data[28:27] !== 2'b0)
	begin
		$display("ERROR: BOOT ERROR!");
		error_flag = 1'b1;
		$display("  Final CMD word: 0x%08x", read_data);
	end else begin
		if (1'b0 !== read_data[24])
		begin
			$display("ERROR: DMA  ERROR, but no BOOT ERROR!");
			error_flag = 1'b1;
		end if (1'b0 !== read_data[22])
		begin
			$display("ERROR: READ ERROR, but no BOOT ERROR!");
			error_flag = 1'b1;
		end if (1'b0 !== read_data[21])
		begin
			$display("ERROR: CMD  ERROR, but no BOOT ERROR!");
			error_flag = 1'b1;
		end

		if (error_flag)
			$display("  Final CMD word: 0x%08x", read_data);
	end

	if (!error_flag)
	begin
		for(iw=0; error_flag === 1'b0 && iw<(1<<(EMMC_LGBOOTSZ-4)); iw=iw+1)
		begin
			u_mcchip.read_bootword(iw, src_data);
`ifdef	SDIO_AXI
			read8(b_addr + { iw, 2'b00 },mem_data[31:24]);
			read8(b_addr + { iw, 2'b01 },mem_data[23:16]);
			read8(b_addr + { iw, 2'b10 },mem_data[15: 8]);
			read8(b_addr + { iw, 2'b11 },mem_data[ 7: 0]);
`else
			u_mem.read8(b_addr + { iw, 2'b00 },mem_data[31:24]);
			u_mem.read8(b_addr + { iw, 2'b01 },mem_data[23:16]);
			u_mem.read8(b_addr + { iw, 2'b10 },mem_data[15: 8]);
			u_mem.read8(b_addr + { iw, 2'b11 },mem_data[ 7: 0]);
`endif

			if (mem_data !== src_data)
			begin
				$write("Boot data mismatch: ");
				$write("MEM[%02x] = 0x%08x != ",
					b_addr + iw*4, mem_data);
				$display("EMMC[%02x] = 0x%08x", iw*4, src_data);
				$display("b_addr = 0x%08x", b_addr);
				error_flag = 1'b1;
			end
		end if (!error_flag)
			$display("  Success!");
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Boot method #2: Automatic boot following HW reset
	// {{{
	////////////////////////////////////////////////////////////////////////

	$display("BOOT TEST #2: Automatic boot following a user commanded RESET");
	if (!error_flag)
	begin
		// Need to re-randomize memory, lest the last random memory
		// get counted valid again
		u_mcchip.randmize_boot;

		// Control setup
		// {{{
		// Set the H/W reset
		read_data[25] = 1'b1;
		// Clear any errors
		read_data[15] = 1'b1;
		read_data[7:0] = 8'h0;	// Clear any potential commands
		// Make sure we expect an acknowledgment
		read_data[26] = 1'b1;
		u_bfm.writeio(ADDR_SDCARD, read_data);

		// Now, while the device is in reset ...

		// Set up the PHY
		u_bfm.readio(ADDR_SDCARD, phy_data);
		// Speed will get reset, needs to be reset here
		phy_data[7:0] = BOOT_SPEED;
		// Whether or not to use DDR
		phy_data[ 8] = BOOT_MODE[2];
		phy_data[14] = BOOT_MODE[2];	// CLK90, always when using DDR
		// Whether or not to use DS (The eMMC spec disallows this ...)
		phy_data[9] = BOOT_MODE[3];	// HS400 mode w/ DS
		// ... as will the width
		phy_data[11:10] = BOOT_MODE[1:0];
		// ... and the block size (512Bytes)
		phy_data[27:24] = 9;
		u_bfm.writeio(ADDR_SDPHY, phy_data);

		// Set up the DMA
		// {{{
		// Increment the address, so we can test writing to unaligned
		// addresses
		b_addr = b_addr + 1;
		u_bfm.writeio(ADDR_DMABUS, b_addr);
		u_bfm.writeio(ADDR_DMALEN, BOOT_BLOCKS);
		// }}}

		// Clear the H/W reset
		read_data[25] = 1'b0;
		// Make sure we expect an acknowledgment
		read_data[26] = BOOT_TOKEN;
		// Also, make sure boot is activated when we release from reset
		// read_data[9:6] = 4'b1100;
		// Clear any errors
		// read_data[15] = 1'b1;
		// All put together now ...
		read_data[15:0] = 16'ha300;
		read_data[13:11] = { OPT_DMA, 1'b0, !OPT_DMA };
		u_bfm.writeio(ADDR_SDCARD, read_data);
		// }}}

		if (OPT_DMA)
		begin
			$display("  Waiting ...");
			wait(emmc_interrupt);
		end else begin
			loading = 2'b00;
			b_addr = 32'h0;
			b_addr[ADDRESS_WIDTH-1:0] = BOOT_ADDR;

			// Request the first block
			// {{{
			if (blk == 0)
			begin
				read_data = 32'h0 | EMMC_MEM;
				u_bfm.writeio(ADDR_SDCARD, read_data);
			end
			// }}}

			for(blk=0; 1'b0 === error_flag
				&& blk<(1<<(EMMC_LGBOOTSZ-9)); blk=blk+1)
			begin
				// Wait for a block to be ready
				// {{{
				wait(emmc_interrupt);

				u_bfm.readio(ADDR_SDCARD, read_data);
				while(read_data & EMMC_MEM)
					u_bfm.readio(ADDR_SDCARD, read_data);

				error_flag = error_flag || read_data[15];
				// }}}

				// (Possibly) Request the next block
				// {{{
				if (blk + 1 < (1<<(EMMC_LGBOOTSZ-9)) )
				begin
					read_data = 32'h0 | EMMC_FIFO;
					if (blk[0] == 1'b0)
						read_data = read_data | EMMC_FIFO;
					u_bfm.writeio(ADDR_SDCARD, read_data);
				end
				// }}}

				// Transfer data
				// {{{
				for(iw=0; 1'b0 === error_flag && iw<(1<<7); iw=iw+1)
				begin
					u_bfm.readio(ADDR_FIFOA + (blk[0]? 4:0),
							read_data);
					u_bfm.writeio(b_addr + { iw, 2'b00 },
							read_data);
				end
				b_addr = b_addr + 512;
				// }}}
			end

			b_addr = 32'h0;
			b_addr[ADDRESS_WIDTH-1:0] = BOOT_ADDR;
		end

		u_bfm.readio(ADDR_SDCARD, read_data);
		$display("  CMD: 0x%08x", read_data);

		if (read_data[28:27] !== 2'b0)
		begin
			$display("ERROR: BOOT ERROR!");
			error_flag = 1'b1;
			$display("  Final CMD word: 0x%08x", read_data);
		end else begin
			if (1'b0 !== read_data[24])
			begin
				$display("ERROR: DMA  ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end if (1'b0 !== read_data[22])
			begin
				$display("ERROR: READ ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end if (1'b0 !== read_data[21])
			begin
				$display("ERROR: CMD  ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end

			if (error_flag)
				$display("  Final CMD word: 0x%08x", read_data);
		end

		// Send a CMD0 : Go IDLE
		// {{{
		// This is to check if we properly produce 74 clocks following
		// any reset, before any command takes place.
		emmc_go_idle;
		// }}}


		if (!error_flag)
		begin
			for(iw=0; error_flag === 1'b0 && iw<(1<<(EMMC_LGBOOTSZ-2)); iw=iw+1)
			begin
				u_mcchip.read_bootword(iw, src_data);
`ifdef	SDIO_AXI
				read8(b_addr + { iw, 2'b00 },mem_data[31:24]);
				read8(b_addr + { iw, 2'b01 },mem_data[23:16]);
				read8(b_addr + { iw, 2'b10 },mem_data[15: 8]);
				read8(b_addr + { iw, 2'b11 },mem_data[ 7: 0]);
`else
				u_mem.read8(b_addr+{iw,2'b00 },mem_data[31:24]);
				u_mem.read8(b_addr+{iw,2'b01 },mem_data[23:16]);
				u_mem.read8(b_addr+{iw,2'b10 },mem_data[15: 8]);
				u_mem.read8(b_addr+{iw,2'b11 },mem_data[ 7: 0]);
`endif

				if (mem_data !== src_data)
				begin
					$display("Boot data mismatch: MEM[%02x] = 0x%08x != EMMC[%02x] = 0x%08x", iw*4, mem_data, iw*4, src_data);
					error_flag = 1'b1;
				end
			end
			if (!error_flag)
				$display("  Success!");
		end
	end else
		$display("  -- Skipped");
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Boot method #3: User commanded boot following CMD0
	// {{{
	////////////////////////////////////////////////////////////////////////

	$display("BOOT TEST #3: Boot mode following CMD0");
	if (!error_flag)
	begin
		// Need to re-randomize memory, lest the last random memory
		// get counted valid again
		u_mcchip.randmize_boot;

		// Control setup
		// {{{
		u_bfm.readio(ADDR_SDCARD, read_data);

		// Set up the PHY
		// {{{
		u_bfm.readio(ADDR_SDPHY, phy_data);
		// Speed will get reset, needs to be reset here
		phy_data[7:0] = BOOT_SPEED;
		// Whether or not to use DDR
		phy_data[ 8] = BOOT_MODE[2];
		phy_data[14] = BOOT_MODE[2];	// CLK90, always when using DDR
		// Whether or not to use DS
		phy_data[9] = BOOT_MODE[3];	// HS400 mode w/ DS (Disallowed by spec)
		// ... as will the width
		phy_data[11:10] = BOOT_MODE[1:0];
		// Must use the clock shutdown, to deal w/ bus idle issues
		phy_data[15] = 1'b1;
		// ... and the block size (512Bytes)
		phy_data[27:24] = 9;
		u_bfm.writeio(ADDR_SDPHY, phy_data);
		// }}}

		// Set up the DMA
		// {{{
		// Keep testing our ability to do unaligned DMA copies
		b_addr = b_addr + 1;
		u_bfm.writeio(ADDR_DMABUS, b_addr);
		u_bfm.writeio(ADDR_DMALEN, BOOT_BLOCKS);
		// }}}

		// CMD0 into the PRE-IDLE state
		u_bfm.writeio(ADDR_SDDATA, 32'hf0f0_f0f0);
		u_bfm.write_f(ADDR_SDCARD, EMMC_CMD | EMMC_RNONE | EMMC_ERR);

		// Wait for the CMD0 to complete
		wait(emmc_interrupt);

		// Now start the boot -- Null command expecting R1b response
		read_data = EMMC_ERR | EMMC_DMA | EMMC_R1b;	// w/ CMD=8'h00, is BOOT
		// Make sure whether or not we expect a boot token
		//  acknowledgment matches how our model is set up
		read_data[26] = BOOT_TOKEN;
		read_data[13:11] = { OPT_DMA, 1'b0, !OPT_DMA };
		u_bfm.write_f(ADDR_SDCARD, read_data);
		// }}}

		$display("  Waiting ...");

		wait(emmc_interrupt);

		u_bfm.readio(ADDR_SDCARD, read_data);
		$display("  CMD: 0x%08x", read_data);

		if (read_data[28:27] !== 2'b0)
		begin
			$display("ERROR: BOOT ERROR!");
			error_flag = 1'b1;
		end else begin
			if (1'b0 !== read_data[24])
			begin
				$display("ERROR: DMA  ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end if (1'b0 !== read_data[22])
			begin
				$display("ERROR: READ ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end if (1'b0 !== read_data[21])
			begin
				$display("ERROR: CMD  ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end

			if (error_flag)
				$display("  Final CMD word: 0x%08x", read_data);
		end

		if (!error_flag)
		begin
			for(iw=0; error_flag === 1'b0 && iw<(1<<(EMMC_LGBOOTSZ-2)); iw=iw+1)
			begin
				u_mcchip.read_bootword(iw, src_data);
`ifdef	SDIO_AXI
				read8(b_addr + { iw, 2'b00 },mem_data[31:24]);
				read8(b_addr + { iw, 2'b01 },mem_data[23:16]);
				read8(b_addr + { iw, 2'b10 },mem_data[15: 8]);
				read8(b_addr + { iw, 2'b11 },mem_data[ 7: 0]);
`else
				u_mem.read8(b_addr+{ iw,2'b00},mem_data[31:24]);
				u_mem.read8(b_addr+{ iw,2'b01},mem_data[23:16]);
				u_mem.read8(b_addr+{ iw,2'b10},mem_data[15: 8]);
				u_mem.read8(b_addr+{ iw,2'b11},mem_data[ 7: 0]);
`endif

				if (mem_data !== src_data)
				begin
					$display("Boot data mismatch: MEM[%02x] = 0x%08x != EMMC[%02x] = 0x%08x", iw*4, mem_data, iw*4, src_data);
					error_flag = 1'b1;
				end
			end if (!error_flag)
				$display("  Success!");
		end
	end else
		$display("  -- Skipped");
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Boot method #4: User commanded ALT-boot following CMD0/FFFF-FFFA
	// {{{
	////////////////////////////////////////////////////////////////////////

	$display("BOOT TEST #4: Alt-boot, following CMD0/FFFF-FFFA");
	if (!error_flag)
	begin
		// Need to re-randomize memory, lest the last random memory
		// get counted valid again
		u_mcchip.randmize_boot;


		// Control setup
		// {{{
		u_bfm.readio(ADDR_SDCARD, read_data);

		// Set up the PHY
		// {{{
		u_bfm.readio(ADDR_SDPHY, phy_data);
		// Speed will get reset, needs to be reset here
		phy_data[7:0] = BOOT_SPEED;
		// Whether or not to use DDR
		phy_data[ 8] = BOOT_MODE[2];
		phy_data[14] = BOOT_MODE[2];	// CLK90, always when using DDR
		// Whether or not to use DS
		phy_data[9] = BOOT_MODE[3];	// HS400 mode w/ DS (Disallowed by spec)
		// ... as will the width
		phy_data[11:10] = BOOT_MODE[1:0];
		// Must use the clock shutdown, to deal w/ bus idle issues
		phy_data[15] = 1'b1;
		// ... and the block size (512Bytes)
		phy_data[27:24] = 9;
		u_bfm.writeio(ADDR_SDPHY, phy_data);
		// }}}

		// Set up the DMA
		// {{{
		// Keep testing our ability to do unaligned DMA copies
		b_addr = b_addr + 1;
		u_bfm.writeio(ADDR_DMABUS, b_addr);
		u_bfm.writeio(ADDR_DMALEN, BOOT_BLOCKS);
		// }}}

		// CMD0/F0F0-F0F0 into the PRE-IDLE state
		// {{{
		//   This should also start our boot, while leaving CMD0 high
		u_bfm.writeio(ADDR_SDDATA, 32'hf0f0_f0f0);
		u_bfm.write_f(ADDR_SDCARD, EMMC_CMD | EMMC_RNONE | EMMC_ERR);

		// Wait for the command to finish
		wait(emmc_interrupt);
		// }}}

		// CMD0/FFFF-FFFA into the BOOT state
		// {{{
		//   This should also start our boot, while leaving CMD0 high
		u_bfm.writeio(ADDR_SDDATA, 32'hffff_fffa);
		read_data = EMMC_CMD | EMMC_RNONE | EMMC_ERR;
		read_data[13:11] = { OPT_DMA, 1'b0, !OPT_DMA };
		// Boot token's aren't (yet) supported for alternate boot
		// read_data[26] = BOOT_TOKEN;
		u_bfm.write_f(ADDR_SDCARD, read_data);
		// }}}

		// Wait for the command to finish
		$display("  Waiting ...");
		wait(emmc_interrupt);
		// }}}

		u_bfm.readio(ADDR_SDCARD, read_data);
		$display("  CMD: 0x%08x", read_data);

		if (read_data[28:27] !== 2'b0)
		begin
			$display("ERROR: BOOT ERROR!");
			error_flag = 1'b1;
		end else begin
			if (1'b0 !== read_data[24])
			begin
				$display("ERROR: DMA  ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end if (1'b0 !== read_data[22])
			begin
				$display("ERROR: READ ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end if (1'b0 !== read_data[21])
			begin
				$display("ERROR: CMD  ERROR, but no BOOT ERROR!");
				error_flag = 1'b1;
			end

			if (error_flag)
				$display("  Final CMD word: 0x%08x", read_data);
		end
	end else
		$display(" -- Skipped");

	if (!error_flag)
	begin
		for(iw=0; error_flag === 1'b0 && iw<(1<<(EMMC_LGBOOTSZ-2)); iw=iw+1)
		begin
			u_mcchip.read_bootword(iw, src_data);
`ifdef	SDIO_AXI
			read8(b_addr + { iw, 2'b00 },mem_data[31:24]);
			read8(b_addr + { iw, 2'b01 },mem_data[23:16]);
			read8(b_addr + { iw, 2'b10 },mem_data[15: 8]);
			read8(b_addr + { iw, 2'b11 },mem_data[ 7: 0]);
`else
			u_mem.read8(b_addr + { iw, 2'b00 },mem_data[31:24]);
			u_mem.read8(b_addr + { iw, 2'b01 },mem_data[23:16]);
			u_mem.read8(b_addr + { iw, 2'b10 },mem_data[15: 8]);
			u_mem.read8(b_addr + { iw, 2'b11 },mem_data[ 7: 0]);
`endif

			if (mem_data !== src_data)
			begin
				$display("Boot data mismatch: MEM[%02x] = 0x%08x != EMMC[%02x] = 0x%08x", iw*4, mem_data, iw*4, src_data);
				error_flag = 1'b1;
			end
		end if (!error_flag)
			$display("  Success!");
	end
	// }}}

	// Boot failures: if DMA error, will BOOT error also get set?

	repeat(512)
		@(posedge clk);
end endtask
