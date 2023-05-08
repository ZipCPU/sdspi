////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	autotest_tb.cpp
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	Exercise all of the functionality contained within the Verilog
//		core, from bring up through read to write and read-back.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2023, Gisselquist Technology, LLC
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
// }}}
// Include files
// {{{
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <Vsdspi.h>
#include "testb.h"
#include "wb_tb.h"
#include "sdspisim.h"
// }}}

// MACRO definitions
// {{{
// #define	OPT_LITTLE_ENDIAN

#define	SDSPI_CMD_ADDR	0
#define	SDSPI_DATA_ADDR	1
#define	SDSPI_FIFO_A	2
#define	SDSPI_FIFO_B	3

#define	READAUX	0x80
#define	SETAUX	0xc0

#define	SDSPI_CMD		0x000040
#define	SDSPI_ACMD		(SDSPI_CMD + 55)
#define	SDSPI_READREG		0x000200
#define	SDSPI_FIFO_OP		0x000800
#define	SDSPI_WRITEOP		0x000c00
#define	SDSPI_FIFO_ID		0x001000
#define	SDSPI_BUSY		0x004000
#define	SDSPI_ERROR		0x008000
#define	SDSPI_CLEARERR		0x008000
#define	SDSPI_REMOVED		0x040000
#define	SDSPI_PRESENTN		0x080000
#define	SDSPI_RESET		0x100000
#define	SDSPI_WATCHDOG		0x200000
#define	SDSPI_GO_IDLE		((SDSPI_REMOVED|SDSPI_CLEARERR|SDSPI_CMD)+0)
#define	SDSPI_READ_SECTOR	((SDSPI_CMD|SDSPI_CLEARERR|SDSPI_FIFO_OP)+17)
#define	SDSPI_WRITE_SECTOR	((SDSPI_CMD|SDSPI_CLEARERR|SDSPI_WRITEOP)+24)
// }}}

class	SDSPI_TB : public WB_TB<Vsdspi> {
	SDSPISIM	*m_sdspi;
public:
	unsigned	OCR(void) { return m_sdspi->OCR(); }

	SDSPI_TB(const char *sdcard_image) {
		// {{{
		if (0 != access(sdcard_image, R_OK)) {
			fprintf(stderr, "Cannoot open %s for reading\n", sdcard_image);
			exit(EXIT_FAILURE);
		} if (0 != access(sdcard_image, W_OK)) {
			fprintf(stderr, "Cannot open %s for writing\n", sdcard_image);
			exit(EXIT_FAILURE);
		}

		m_sdspi = new SDSPISIM(true);
		m_sdspi->load(sdcard_image);
		// }}}
	}

	virtual	void	tick(void) {
		// {{{
		TESTB<Vsdspi>::tick();

		core()->i_miso = (*m_sdspi)(core()->o_cs_n,
				core()->o_sck, core()->o_mosi);
		// }}}
	}

	Vsdspi *core(void) {
		return m_core;
	}

	unsigned	read_aux(void) {
		// {{{
		wb_write(SDSPI_CMD_ADDR, READAUX);
		return wb_read(SDSPI_DATA_ADDR);
		// }}}
	}

	unsigned	set_aux(unsigned aux) {
		// {{{
		wb_write(SDSPI_DATA_ADDR, aux);
		wb_write(SDSPI_CMD_ADDR, SETAUX);
		return wb_read(SDSPI_DATA_ADDR);
		// }}}
	}

	void	wait_while_busy(void) {
		// {{{
		while(!core()->o_int)
			tick();
		// }}}
	}

	unsigned	sdcmd(int cmd, unsigned arg = 0) {
		// {{{
		wb_write(SDSPI_DATA_ADDR, arg);
		wb_write(SDSPI_CMD_ADDR, cmd);

		wait_while_busy();
		return wb_read(SDSPI_CMD_ADDR);
		// }}}
	}

	unsigned	read(int cmd, unsigned arg, int ln, unsigned *data) {
		// {{{
		unsigned	fifo_addr;
		unsigned	lglen, r;

		if (cmd & SDSPI_FIFO_ID)
			fifo_addr = SDSPI_FIFO_B;
		else
			fifo_addr = SDSPI_FIFO_A;

		assert((cmd & SDSPI_WRITEOP) == SDSPI_FIFO_OP);

		for(lglen = 4; (1<<lglen) < ln; lglen++)
			;
		set_aux(lglen << 16);

fprintf(stderr, "LGLEN = %d, LN = %d\n", lglen, ln);
		assert((1<<lglen) == ln);

		wb_write(SDSPI_DATA_ADDR, arg);
		wb_write(SDSPI_CMD_ADDR, cmd);
		wait_while_busy();

		r = wb_read(SDSPI_CMD_ADDR);
		wb_read(fifo_addr, (1<<(lglen-2)), data, 0);

		return	r;
		// }}}
	}

	unsigned	write(int cmd, unsigned arg, int ln, unsigned *data) {
		// {{{
		unsigned	fifo_addr;
		unsigned	lglen;

		if (cmd & SDSPI_FIFO_ID)
			fifo_addr = SDSPI_FIFO_B;
		else
			fifo_addr = SDSPI_FIFO_A;

		assert((cmd & SDSPI_WRITEOP) == SDSPI_WRITEOP);

		for(lglen = 4; (1<<lglen) < ln; lglen++)
			;
		set_aux(lglen << 16);

		assert((1<<lglen) == ln);

		wb_write(fifo_addr, (1<<(lglen-2)), data, 0);

		wb_write(SDSPI_DATA_ADDR, arg);
		wb_write(SDSPI_CMD_ADDR, cmd);
		wait_while_busy();
		return	wb_read(SDSPI_CMD_ADDR);
		// }}}
	}

	////////////////////////////////////////////////////////////////////////
	//
	unsigned read_ocr(void) {
		// {{{
		unsigned	r;
		r = sdcmd(SDSPI_READREG | SDSPI_CLEARERR | SDSPI_CMD + 58,0);
		TBASSERT((*this),r == 0);

		r = wb_read(SDSPI_DATA_ADDR);
		fprintf(stderr, "R   : 0x%08x\nOCR: 0x%08x\n", r, m_sdspi->OCR());
		TBASSERT((*this), (r == m_sdspi->OCR()));
		return r;
		// }}}
	}

	unsigned read_csd(unsigned *data) {
		// {{{
		unsigned	r;
		r = read(SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD+9, 0,
				16, data);

		for(int k=0; k<4; k++) {
			unsigned v;
			v = 0;
#ifdef	OPT_LITTLE_ENDIAN
			for(int i=0; i<4; i++)
				v = (v<<8) | m_sdspi->CSD(k*4+3-i);
#else
			for(int i=0; i<4; i++)
				v = (v<<8) | m_sdspi->CSD(k*4+i);
#endif

			TBASSERT((*this), v == data[k]);
		}
		return r;
		// }}}
	}

	unsigned read_cid(unsigned *data) {
		// {{{
		unsigned	r;
		r = read(SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD+10, 0,
				16, data);
		for(int k=0; k<4; k++) {
			unsigned v;
			v = 0;
#ifdef	OPT_LITTLE_ENDIAN
			for(int i=0; i<4; i++)
				v = (v<<8) | m_sdspi->CID(k*4+3-i);
#else
			for(int i=0; i<4; i++)
				v = (v<<8) | m_sdspi->CID(k*4+i);
#endif

			TBASSERT((*this), v == data[k]);
		} return r;
		// }}}
	}

};

int	main(int argc, char **argv) {
	const char	SDIMAGE_FILENAME[] = "sdcard.img";
	const char	VCD_FILENAME[] = "trace.vcd";
	SDSPI_TB	tb(SDIMAGE_FILENAME);
	unsigned	v;

	unsigned	boot_sector[128], test_sector[128], buf[128];

	tb.opentrace(VCD_FILENAME);

	tb.core()->i_sd_reset    = 1;
	tb.core()->i_bus_grant   = 1;
	tb.core()->i_card_detect = 1;
	tb.tick();
	tb.tick();
	tb.tick();
	tb.core()->i_sd_reset = 0;
	tb.tick();
	v = tb.read_aux();
	if (SDSPI_PRESENTN & tb.wb_read(SDSPI_CMD_ADDR)) {
		printf("Waiting for the card assertion to be registered\n");
		while(SDSPI_PRESENTN & tb.wb_read(SDSPI_CMD_ADDR))
			;
	}

	//
	// GO_IDLE
	printf("SEND_GO_IDLE\n");
	assert(0x01 == tb.sdcmd(SDSPI_GO_IDLE));
	//
	// SEND_IF_COND
	printf("SEND_IF_COND\n");
	assert(0 == tb.sdcmd(SDSPI_READREG | SDSPI_CMD + 8, 0x01a5));
printf("Data response = 0x%08x\n", tb.wb_read(SDSPI_DATA_ADDR));
	assert(0x01a5 == tb.wb_read(SDSPI_DATA_ADDR));
	//
	// Wait for the card to start up
	do {
		assert(0 == (tb.sdcmd(SDSPI_ACMD) & 0x01));
		assert(0 == ((v = tb.sdcmd(SDSPI_CMD + 41, 0x40000000))&~1));
	} while(v & 1);

	//
	// Read the OCR register
	tb.read_ocr();
	printf("OCR: 0x%08x\n", (v = tb.wb_read(SDSPI_DATA_ADDR)));
	assert(v == tb.OCR());

	// Speed up our interface
	tb.set_aux(1);

	//
	// Read the CSD register
	tb.read_csd(test_sector);
fprintf(stderr, "Read\n");
	printf("CSD: ");
	for(int k=0; k<4; k++)
		printf("%08x%c", test_sector[k], (k < 3) ? ':':'\n');

	//
	// Read the CID register
	tb.read_cid(test_sector);
	printf("CID: ");
	for(int k=0; k<4; k++)
		printf("%08x%c", test_sector[k], (k < 3) ? ':':'\n');
	//
	// Read the original boot sector
	tb.read(SDSPI_READ_SECTOR, 0, 512, boot_sector);
	//
	// Write random data to the boot sector
	for(unsigned k=0; k<128; k++)
		test_sector[k] = rand();
	tb.write(SDSPI_WRITE_SECTOR | SDSPI_FIFO_ID, 0, 512, test_sector);

	//
	// Read the random data back
	tb.read(SDSPI_READ_SECTOR, 0, 512, buf);
	//
	// Check that it was correctly written
	for(unsigned k=0; k<128; k++) {
fprintf(stderr, "BUF[%3d] = 0x%08x, TST[%3d] = 0x%08x\n", k, buf[k], k, test_sector[k]);
	}
	for(unsigned k=0; k<128; k++) {
fprintf(stderr, "BUF[%d] = 0x%08x\n", k, buf[k]);
fprintf(stderr, "TST[%d] = 0x%08x\n", k, test_sector[k]);
		assert(buf[k] == test_sector[k]);
	}
	//
	// Restore the boot sector
	tb.write(SDSPI_WRITE_SECTOR, 0, 512, boot_sector);
	//
	// Read it back again
	tb.read(SDSPI_READ_SECTOR | SDSPI_FIFO_ID, 0, 512, buf);
	//
	// Check that it was properly stored
	for(unsigned k=0; k<128; k++)
		assert(buf[k] == boot_sector[k]);

	printf("SUCCESS!\n");
}
