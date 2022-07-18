////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sdcard.c
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	Provides low-level access to the SD-Card SPI controller.
//		Primary functions provided here are sdcard_init() to initialize
//	the card, sdcard_read(sector, buffer) and sdcard_write(sector, buffer)
//	to read from and write to the card respectively.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2022, Gisselquist Technology, LLC
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
// }}}
// #define	STDIO_DEBUG
#ifdef	STDIO_DEBUG
#include <stdio.h>
#include <stdlib.h>

#define	txstr(A)	printf("%s", A)
#define	txhex(A)	printf("%08x", A)

#else
// Embedded I/O functions txstr() and txhex
#include "txfns.h"
#endif

#include "board.h"

#include "sdcard.h"
#include "zipsys.h"

#include <zipcpu.h>


#define	SDSPI_READREG	0x0200	// Expect an R3 or R7 response (R1 + 32-bits)
#define	SDSPI_WAIT_WHILE_BUSY	{ int r; do { r = _sdcard->sd_ctrl; if (r & (SDSPI_ERROR|SDSPI_PRESENTN)) break; } while(r & SDSPI_BUSY); }

//
// Provide DMA support
//
// If INLUDE_DMA_CONTROLLER is set, then we'll build to use the DMA to transfer
// data to and from the SDSPI controller
//
#ifdef	INCLUDE_DMA_CONTROLLER
const int	SDUSEDMA = 1;
#else
const int	SDUSEDMA = 0;
#endif

//
// SCOPE
//
// If defined, references the address of a WBSCOPE on the bus which can be used
// for debugging various interactions
#define		SCOPE	_scope_sdcard

//
//
// Book recommends sequence of: CMD0, (CMD8), CMD58, ACMD41(wait), CMD58
//	for startup
//

int	sdcard_err = 0;
int	sdcard_ocr = 0;
char	sdcard_csd[16];
char	sdcard_cid[16];

//
// SDDEBUG
//
// If set will produce verbose output via txstr() and txhex().  These functions
// are roughly equivalent to printf("%s",X) and printf("%08x", X) respectively.
static const int	SDDEBUG = 0;

//
// SDINFO
//
// If set, will produce output indicating the results of transactions.
static const int	SDINFO  = 0;

#ifdef	_BOARD_HAS_SDSPI

int	sdcard_read_ocr(void) {
	unsigned	v;

	if (SDDEBUG)
		txstr("SDCARD: CMD58, READ-OCR\n");

	//
	// CMD58 : READ-OCR
	// After the card is ready, we must send a READ_OCR command next
	// The card will respond with an R7
	//
	_sdcard->sd_data = 0x0;
	_sdcard->sd_ctrl = (SDSPI_CMD|SDSPI_READREG) + 58; // 0x027a;
	SDSPI_WAIT_WHILE_BUSY;

	if ((v = _sdcard->sd_ctrl)!= 0) {
		sdcard_err |= SDERR_INIT | 0x10000;
		if (SDDEBUG) {
			txstr("READ-OCR: Ctrl != 0, ");
			txhex(v);
			txstr("\r\n");
		}
	}


	sdcard_ocr = _sdcard->sd_data;
	if (SDINFO) {
		txstr("READ-OCR: OCR = "); txhex(sdcard_ocr); txstr("\n");
	// printf("    OCR = 0x%08x%s\n", v, (v!=0x40ff8000)?"Shouldnt this be 0x40ff8000?":"");

		if (sdcard_ocr & 0x80000000)
			txstr("  Card is still powering up\n");
		if (sdcard_ocr & 0x40000000)
			txstr("  CCS: High capacity support\n");
		if (sdcard_ocr & 0x20000000)
			txstr("  UHS: USH-II card\n");
		if (sdcard_ocr & 0x01000000)
			txstr("  S18A: Switching to 1.8V allowed\n");
#ifdef	STDIO_DEBUG
		int	mxv = 0, mnv = 0;
		if (sdcard_ocr & 0x00800000) {
			if ((mxv == 0)||(mxv <= 36))
				mxv = 36;
			if ((mnv == 0)||(mnv >= 35))
				mxv = 35;
		} if (sdcard_ocr & 0x00400000) {
			if ((mxv == 0)||(mxv <= 35))
				mxv = 35;
			if ((mnv == 0)||(mnv >= 34))
				mxv = 34;
		} if (sdcard_ocr & 0x00200000) {
			if ((mxv == 0)||(mxv <= 34))
				mxv = 34;
			if ((mnv == 0)||(mnv >= 33))
				mxv = 33;
		} if (sdcard_ocr & 0x00100000) {
			if ((mxv == 0)||(mxv <= 33))
				mxv = 33;
			if ((mnv == 0)||(mnv >= 32))
				mxv = 32;
		} if (sdcard_ocr & 0x00080000) {
			if ((mxv == 0)||(mxv <= 32))
				mxv = 32;
			if ((mnv == 0)||(mnv >= 31))
				mxv = 31;
		} if (sdcard_ocr & 0x00040000) {
			if ((mxv == 0)||(mxv <= 31))
				mxv = 31;
			if ((mnv == 0)||(mnv >= 30))
				mxv = 30;
		} if (sdcard_ocr & 0x00020000) {
			if ((mxv == 0)||(mxv <= 30))
				mxv = 30;
			if ((mnv == 0)||(mnv >= 29))
				mxv = 29;
		} if (sdcard_ocr & 0x00010000) {
			if ((mxv == 0)||(mxv <= 29))
				mxv = 29;
			if ((mnv == 0)||(mnv >= 28))
				mxv = 28;
		} if (sdcard_ocr & 0x00008000) {
			if ((mxv == 0)||(mxv <= 28))
				mxv = 28;
			if ((mnv == 0)||(mnv >= 27))
				mxv = 27;
		}

		printf("  Voltage ranges supported: %d.%dV - %d.%dV\n",
			(mxv/10), (mxv%10), (mnv/10), (mnv%10));
#else
		if (sdcard_ocr & 0x00800000)
			txstr("  3.6-3.5 V allowed\n");
		if (sdcard_ocr & 0x00400000)
			txstr("  3.5-3.4 V allowed\n");
		if (sdcard_ocr & 0x00200000)
			txstr("  3.4-3.3 V allowed\n");
		if (sdcard_ocr & 0x00100000)
			txstr("  3.3-3.2 V allowed\n");
		if (sdcard_ocr & 0x00080000)
			txstr("  3.2-3.1 V allowed\n");
		if (sdcard_ocr & 0x00040000)
			txstr("  3.1-3.0 V allowed\n");
		if (sdcard_ocr & 0x00020000)
			txstr("  3.0-2.9 V allowed\n");
		if (sdcard_ocr & 0x00010000)
			txstr("  2.9-2.8 V allowed\n");
		if (sdcard_ocr & 0x00008000)
			txstr("  2.8-2.7 V allowed\n");
#endif
	}

	return sdcard_ocr;
}

//
// Read the SCR register
//
int	sdcard_read_scr(unsigned *scr) {
	unsigned	v;

	if (SDDEBUG)
		txstr("READ-SCR\n");

	// The SCR register is 512 bytes, and we can read it at fast speed
	_sdcard->sd_data = SECTOR_8B | SPEED_FAST;
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	// 
	_sdcard->sd_data = 0;
	_sdcard->sd_ctrl = (SDSPI_CLEARERR|SDSPI_ACMD); // Go to alt command set
	SDSPI_WAIT_WHILE_BUSY;

	if (_sdcard->sd_ctrl & (SDSPI_ERROR | SDSPI_REMOVED)) {
		sdcard_err |= SDERR_INIT | 0x4000000;
		return -1;
	}

	_sdcard->sd_data = 0x0;	// Read from position zero
	_sdcard->sd_ctrl = (SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD)+51;
	SDSPI_WAIT_WHILE_BUSY;

	if ((v = _sdcard->sd_ctrl) != 0)
		sdcard_err |= SDERR_INIT | 0x4000000;
	if ((v = _sdcard->sd_data) != 0xffffffff)
		sdcard_err |= SDERR_INIT | 0x8000000;

	scr[0] = _sdcard->sd_fifo[0];
	scr[1] = _sdcard->sd_fifo[0];

	if (SDINFO) {
		txstr("SCR: ");
		txhex(scr[0]); txstr(":");
		txhex(scr[1]); txstr("\n");
	}

	if (_sdcard->sd_ctrl & SDSPI_ERROR)
		return -1;
	return 0;
}

//
// Read the CSD register
//
int	sdcard_read_csd(char *csd) {
	unsigned	*ucsd = (unsigned *)csd;
	int		i;
	unsigned	v;

	if (sdcard_err != 0)
		return -1;

	if (SDDEBUG)
		txstr("READ-CSD\n");

	//
	// Adjust the FIFO for a 4-word length
	//
	_sdcard->sd_ctrl = SDSPI_READAUX;
	// 16 byte block length
	_sdcard->sd_data = SECTOR_16B;
	// Write config data, read last config data
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	//
	// CMD9 : SEND_CSD_COND
	// Send CSD condition to FIFO #0 (Requires FIFO support)
	//
	_sdcard->sd_data = 0;
	_sdcard->sd_ctrl = (SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD) + 9; // 0x08849;
	SDSPI_WAIT_WHILE_BUSY;

	if ((v = _sdcard->sd_ctrl) != 0) {
		txstr("READ-CSD: Invalid CSD return control value, ");
		txhex(v);
		txstr("\n");
		sdcard_err |= SDERR_INIT | 0x20000;
zip_halt();
	}
	if ((v = _sdcard->sd_data) != 0) {
		if (v & 0x010)
			txstr("READ-CSD: Return indicates CRC ERR, ");
		else
			txstr("READ-CSD: Unexpected return data, ");
		txhex(v);
		txstr("\n");
		sdcard_err |= SDERR_INIT | 0x40000;
zip_halt();
	}

	for(i=0; i<4; i++)
		ucsd[i] = _sdcard->sd_fifo[0];

	if (SDINFO) {
		txstr("CSD: ");
		txhex(ucsd[0]);
		txstr(":");
		txhex(ucsd[1]);
		txstr(":");
		txhex(ucsd[2]);
		txstr(":");
		txhex(ucsd[3]);
		txstr("\n");
	}

	// 40,0e,00,32, 5b,59,00,00, e8,37,7f,80, 0a,40,00,23,
	// CSD_STRUCTURE = 2'b01	(CSD version 2.0)
	// 6'h00 (Reserved)
	//
	// TAAC = 0x0e (always 0x0e in v2)
	//
	// NSAC = 0x00 (always 0 in v2)
	//
	// TRAN_SPEED=0x32 (25MHz max operating speed)
	//
	// CCC = 0x5b5
	// READ_BL_LEN = 0x9, max block read length is 512 bytes
	//
	// READ_BL_PARTIAL	= 0
	// WRITE_BLK_MISALIGN	= 0
	// READ_BLK_MISALIGN	= 0
	// DSR_IMP		= 0
	// 2'b00 (reserved)
	// C_SIZE		= 22'h00e837=>(59447+1)/2MB = 29,724MB
	//
	//
	// 1'b0 (reserved)
	// ERASE_BLK_EN		= 1'b1 (Host can erase units of 512 bytes)
	// SECTOR_SIZE		= 7'b11_1111_1	(128 write blocks, 64kB ea)
	// WP_GRP_SIZE		= 7'h00 (one erase sector)
	//
	// WP_GRP_ENABLE	= 1'b0 (No group write protection possible)
	// 2'b00 (reserved)
	// R2W_FACTOR		= 3'b010 (writes are 4x slower than reads)
	// WRITE_BL_LEN		= 4'h9 (512 bytes)
	// WRITE_BL_PARTIAL	= 1'b0 (Only 512 byte units may be written)
	// 5'h00 (reserved)
	//
	// FILE_FORMAT_GRP	= 1'b0 (HD type file system w/partition tbl)
	// COPY			= 1'b0 (Contents are original, not copied)
	// PERM_WRITE_PROTECT	= 1'b0 (No permanent write protect)
	// TMP_WRITE_PROTECT	= 1'b0 (No temporary write protect)
	// FILE_FORMAT		= 2'b00 (As above, HD typ fmt, w/ partition tbl)
	// 2'b00 (reserved)
	//
	// CRC	= { 7'h11, 1'b1 }
	//
	// Derived values:
	// 	BLOCK_LEN = 2^READ_BL_LEN = 512

	return 0;
}

int	sdcard_read_cid(char *cid) {
	unsigned	*ucid = (unsigned *)cid;
	int		i;
	unsigned	v;

	if (SDDEBUG)
		txstr("READ-CID\n");

	// CMD10 : SEND_CID_COND
	// Send CID condition to FIFO #1 (Requires reading from FIFO)
	//
	// Adjust the FIFO for a 4-word length
	//
	_sdcard->sd_ctrl = SDSPI_READAUX;
	// 16 byte block length
	_sdcard->sd_data = SECTOR_16B;
	// Write config data, read last config data
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	_sdcard->sd_data = 0x0;
	_sdcard->sd_ctrl = (SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD)+10; // 0x0984a;
	SDSPI_WAIT_WHILE_BUSY;

	if ((v = _sdcard->sd_ctrl) != 0x0) { // Expecting 0x0
		txstr("READ-CID: Expecting a control response of 0x0, not ");
		txhex(v);
		txstr("\r\n");
		sdcard_err |= SDERR_INIT | 0x100000;
	}
	if ((v = _sdcard->sd_data) != 0) {
		if (v & 0x10)
			txstr("READ-CID: Rcvd a CRC err, ");
		else
			txstr("READ-CID: Expecting a data field of 0x..f00, not ");
		txhex(v);
		txstr("\r\n");
		sdcard_err |= SDERR_INIT | 0x200000;
	}

	// 0x0824d; // CMD thirteen -- SEND_STATUS
	_sdcard->sd_data = 0x0;
	_sdcard->sd_ctrl = (SDSPI_CLEARERR|SDSPI_READREG|SDSPI_CMD)+ 13;

	SDSPI_WAIT_WHILE_BUSY;

	// Read out the CID
	for(i=0; i<4; i++)
		ucid[i] = _sdcard->sd_fifo[0];

	if (1 || SDINFO) {
		txstr("CID: ");
		txhex(ucid[0]); txstr(":");
		txhex(ucid[1]); txstr(":");
		txhex(ucid[2]); txstr(":");
		txhex(ucid[3]); txstr("\n");
	}

	if ((v = _sdcard->sd_ctrl) != 0) { // 0
		txstr("READ-CID: 2nd read, expecting control value of 0, not ");
		txhex(v);
		txstr("\r\n");
		// sdcard_err |= SDERR_INIT | 0x400000;
	}

	if ((v = _sdcard->sd_data) != 0x00ffffff) { // Finally, read the cards status
		txstr("READ-CID: 2nd cmd, expecting a data value of -1, not ");
		txhex(v);
		txstr("\r\n");
		// sdcard_err |= SDERR_INIT | 0x800000;
	}

#ifdef	STDIO_DEBUG
	if (SDINFO) {
		unsigned sn, md;

		//
		// Here's an example of how this might break down
		//
		// 03,53,44,53, 44,33,32,47, 30,7c,13,03, 66,00,ea,25,
		// MID = 0x03; // Manufacturer ID
		// OID = 0x5344; // OEM/Application ID
		// PNM = 0x5344333247 = "SD32G" // Product Name
		// PRV = 0x30;	// Product Revision
		// PSN = 0x7c130366; // Product Serial Number
		// Reserved= 4'h0
		// MDT = 0x0ea // Manufacturing Date, (October, 2014)
		// CRC = 0x25 = {7'h12, 1'b1}

		sn = (cid[9]&0x0ff);
		sn = (sn<<8) | (cid[10]&0x0ff);
		sn = (sn<<8) | (cid[11]&0x0ff);
		sn = (sn<<8) | (cid[12]&0x0ff);
		md = (cid[13] & 0x0f);
		md = (md << 8) | (cid[14] & 0x0ff);

		printf("CID:\n"
"\tManufacturer ID:  0x%02x\n"
"\tApplication ID:   %c%c\n"
"\tProduct Name:     %5c\n"
"\tProduct Revision: %x.%x\n"
"\tSerial Number:    0x%0x\n",
		cid[0]&0x0ff,
		cid[1], cid[2],
		cid[3], cid[4], cid[5], cid[6], cid[7],
		(cid[8]>>4)&0xf, cid[8]&0x0f,
		sn);
		printf(
"\tYear of Man.:     %d\n"
"\tMonth of Man.:    %d\n",
	((md>>4)+2000), md&0x0f);
	}
#endif

	return (sdcard_err == 0) ? 0 : -1;
}



int	sdcard_init(void) {
	// int		*data = debug_data;
	int		i, j;
	unsigned	v;

#ifdef	SCOPE
	const unsigned	SCOPEDELAY = 0x100;

	SCOPE->s_ctrl = WBSCOPE_DISABLE | SCOPEDELAY;
#endif
	if (SDDEBUG)
		txstr("SDCARD-INIT\n");

	// Start us out slow, with a known sector length
	_sdcard->sd_data = SECTOR_512B | SPEED_SLOW;
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	// Clear any prior pending errors
	_sdcard->sd_data = 0;
	_sdcard->sd_ctrl = SDSPI_CLEARERR|SDSPI_READAUX;

	sdcard_err = 0;
	if (_sdcard->sd_ctrl & SDSPI_PRESENTN) {
		txstr("SDCARD: No card present\n");
		sdcard_err = -1;
		return -1;
	}

	/////////////////
	//
	// CMD0: Command zero, reset the card interaction
	//
	// Reset the card by sending a command zero
	//
	/////////////////
	//
	//
	if (SDDEBUG)
		txstr("SDCARD: CMD0 GO_IDLE_STATE\n");

	_sdcard->sd_data = 0;
	_sdcard->sd_ctrl = SDSPI_GO_IDLE; // CMD zero
	SDSPI_WAIT_WHILE_BUSY;

	v = _sdcard->sd_ctrl;
	if (v & SDSPI_ERROR) {
#ifdef	SCOPE
		SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
		txstr("No response from card to reset command\n");
		*_gpio = GPIO_SET(GPIO_SD_RESET);
		sdcard_err |= SDERR_INIT | 0x800;
		return -1;
	}
	if (SDDEBUG) {
		txstr("Response: ");
		txhex(v);
		txstr("\r\n");
	}


	if (0) {
		//
		// ENABLE AT YOUR PERIL!!!
		//
		// LEXAR's CHIP DOESN'T WORK FOLLOWING A CMD1
		//
		//

		//
		// CMD1 : SEND_OP_COND
		// send operational conditions (i.e. the voltage we'll be
		// operating at)
		//
		if (SDDEBUG)
			txstr("SDCARD: CMD1 SEND_OP_COND\n");
		_sdcard->sd_data = 0x40000000;	// We support high capacity cards
		_sdcard->sd_ctrl = SDSPI_CMD+1;
		SDSPI_WAIT_WHILE_BUSY;

		if ((v = _sdcard->sd_ctrl)!= 1) {
			if (v != 2) {
				//
				// The SIM returns a 2 for this command
				//
				txstr("SDCARD: SEND_OP_COND returned ");
				txhex(v);
				txstr(", not 1\r\n");
				// sdcard_err |= SDERR_INIT | 0x400;
			}
		}


		if ((v = _sdcard->sd_data)!= -1) {
			txstr("SDCARD: SEND_OP_COND data returned ");
			txhex(v);
			txstr(", not -1\r\n");
			sdcard_err |= SDERR_INIT | 0x800;

#ifdef	SCOPE
			SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
			return -1;
		}

		if (SDDEBUG) {
			txstr("Response: ");
			txhex(_sdcard->sd_ctrl);
			txstr("\r\n");
		}
	}

	/////////////////
	//
	// CMD8 : SEND_IF_COND
	//
	// Send interface condition
	//
	/////////////////
	//
	//
	if (SDDEBUG)
		txstr("SDCARD: CMD8 SEND_IF_COND (3.3v)\n");
	_sdcard->sd_data = 0x001a5;
	_sdcard->sd_ctrl = (SDSPI_CMD|SDSPI_READREG)+8;
	SDSPI_WAIT_WHILE_BUSY;

	if ((v = _sdcard->sd_data)!= 0x01a5) {
#ifdef	SCOPE
		SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
		txstr("SDCARD: SEND_IF_COND status ");
		txhex(v);
		txstr(", != 0x01a5 as expected\r\n");
		sdcard_err |= SDERR_INIT | 0x2000;

		return -1;
	}

	if (SDDEBUG) {
		txstr("Response: ");
		txhex(_sdcard->sd_ctrl);
		txstr(" : ");
		txhex(_sdcard->sd_data);
		txstr("\r\n");
	}

	/////////////////
	//
	// ACMD41
	//
	// Issue ACMD41 until the card has fully started
	//
	/////////////////
	//
	//
	{	int	dev_busy;
		const int	MAX_ITERATIONS = 1500;

		if (SDDEBUG)
			txstr("SDCARD: ACMD41, Wait for startup\n");

		int	iterations = 0;
		do {
			// Now we need to issue an ACMD41 until such time as
			// the in_idle_state turns to zero
			//
			// First step, issue the ACMD prefix command
			//
			_sdcard->sd_data = 0;
			_sdcard->sd_ctrl = SDSPI_ACMD;	// 0x70 = 'd55 + 'h40
			SDSPI_WAIT_WHILE_BUSY;

			//
			// Send the ACMD41 itself
			//
			_sdcard->sd_data = 0x40000000; // Support high capacity
			_sdcard->sd_ctrl = SDSPI_CMD + 41; // 0x69; // 0x040+41;
			SDSPI_WAIT_WHILE_BUSY;

			dev_busy = _sdcard->sd_ctrl&1;
			iterations++;

			if (SDDEBUG && (_sdcard->sd_ctrl == 1))
				txstr(".");
		} while(dev_busy && (iterations < MAX_ITERATIONS));

		if (iterations >= MAX_ITERATIONS)
			sdcard_err |= SDERR_INIT | 0x4000;
	}

	//
	// Check the response, to double check things are "working" like they
	// should be.
	{
		unsigned vc, vd;

		vc = _sdcard->sd_ctrl;
		vd = _sdcard->sd_data;

		if (vc != 0)
			sdcard_err |= SDERR_INIT | 0x4000;
		else if (vd != 0xffffffff)
			sdcard_err |= SDERR_INIT | 0x8000;

		if (SDDEBUG || sdcard_err & 0xc000) {
			txstr("ACMD41 Response: ");
			txhex(vc);
			txstr(" : ");
			txhex(vd);
			txstr("\r\nI/O setup: ");
			_sdcard->sd_ctrl = SDSPI_READAUX;
			vd = _sdcard->sd_data;
			txhex(vd);
			if (sdcard_err & 0x0c000) {
				txstr("\r\nERR: Card still idle\r\n");
				zip_halt();
			} else
				txstr("\r\n");
		}

		if (sdcard_err & 0xc000) {
#ifdef	SCOPE
			SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
			return -1;
		}
	}

	/////////////////
	//
	// Read the OCR register
	//
	/////////////////
	//
	//
	sdcard_read_ocr();

	//
	// Change the clock speed to high speed
	//
	_sdcard->sd_data = 0x0401;	// 16 byte block length, 25MHz clock
	// Write config data, read last config data
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	/*
	// Bits
	//	19:16	Sector size we can support
	//	11: 8	LGBLKLEN
	//	 3: 0	SDSPI CLK divider
	*/

	if (sdcard_err != 0) {
		txstr("SDCARD: ERR != 0, ERR = ");
		txhex(sdcard_err);
		txstr("\r\n");
		return -1;
	}

	/////////////////
	//
	// Read the CSD register
	//
	/////////////////
	//
	//
	if ((v = sdcard_read_csd(sdcard_csd)) != 0) {
		txstr("SDCARD: READCSD returned ");
		txhex(v);
		txstr("\r\n");
		return -1;
	}

	/////////////////
	//
	// Read the CID register
	//
	/////////////////
	//
	//
	if ((v = sdcard_read_cid(sdcard_cid)) != 0) {
		txstr("SDCARD: READCID returned ");
		txhex(v);
		txstr("\r\n");
		return -1;
	}

	return 0;
}

//
//
// DUMP Control register
//
//
static void	sdcard_dump_ctrlreg(const unsigned rv) {
	if (SDDEBUG) {
		txstr("CONTROL-REG:  "); txhex(rv);
		if (rv & SDSPI_WATCHDOG)
			txstr("\r\n  Watchdog-err");
		if (rv & SDSPI_RESET)
			txstr("\r\n  Needs reset");
		if (rv & SDSPI_REMOVED)
			txstr("\r\n  Removed");
		if (rv & SDSPI_PRESENTN)
			txstr("\r\n  Not present");
		// if (rv & SDSPI_ERRTOK)
		//	txstr("\r\n  Error token");
		// if (rv & SDSPI_CRCERR)
		//	txstr("\r\n  CRC Error");
		if (rv & SDSPI_ERROR)
			txstr("\r\n  Generic Error");
		if (rv & SDSPI_BUSY)
			txstr("\r\n  Busy");
		if (rv & SDSPI_ALTFIFO)
			txstr("\r\n  Alternate FIFO");
		if (rv & SDSPI_FIFO_OP)
			txstr("\r\n  Memory operation");

		txstr("\r\n  R1 : "); txhex(rv & 0x0ff);
		txstr("\r\n");
	}
}


//
//
// DUMP R1
//
//
static void	sdcard_dump_r1(const unsigned rv) {
	if (SDDEBUG) {
		txstr("R1 Decode:  "); txhex(rv & 0x0ff);
		if (rv & 0x01)
			txstr("\r\n  Card is still in idle");
		if (rv & 0x02)
			txstr("\r\n  Erase reset");
		if (rv & 0x04)
			txstr("\r\n  Illegal command");
		if (rv & 0x08)
			txstr("\r\n  Com CRC error");
		if (rv & 0x10)
			txstr("\r\n  Erase sequence err");
		if (rv & 0x20)
			txstr("\r\n  Address error");
		if (rv & 0x40)
			txstr("\r\n  Parameter error");
		txstr("\r\n");
	}
}

//
//
// DUMP R2
//
//
static void	sdcard_dump_r2(const unsigned rv) {
	if (SDDEBUG) {
		unsigned	uv = rv >> 16;
		txstr("R2 Decode:  "); txhex(uv & 0x0ffff);
		if (uv & 0x01)
			txstr("\r\n  Card is locked");
		if (uv & 0x02)
			txstr("\r\n  Unlock command failed");
		if (uv & 0x04)
			txstr("\r\n  Error");
		if (uv & 0x08)
			txstr("\r\n  CC Error");
		if (uv & 0x10)
			txstr("\r\n  Card ECC failed");
		if (uv & 0x20)
			txstr("\r\n  WP violation");
		if (uv & 0x40)
			txstr("\r\n  Erase param");
		if (uv & 0x80)
			txstr("\r\n  Out of range | CSD overwrite");
		txstr("\r\n");
		sdcard_dump_r1(uv >> 8);
	}
}


//
//
// DUMP SECTOR
//
//
static void	sdcard_dump_sector(const unsigned *ubuf) {
	int	j;

	for(j=0; j<512/4; j++) {
		txhex(ubuf[j]);
		if ((j&7)==7)
			txstr("\r\n");
		else
			txstr(" ");
	} txstr("\r\n");
}

static	unsigned	sdcard_get_r2(void) {
	if (SDDEBUG) {
		_sdcard->sd_ctrl = SDSPI_ERROR | SDSPI_READREG | SDSPI_CMD + 13;
		SDSPI_WAIT_WHILE_BUSY;

		txstr("SD_WRITE.STAT: Status return, "); txhex(_sdcard->sd_ctrl);
		txstr(" : "); txhex(_sdcard->sd_data);
		txstr("\r\n");

		sdcard_dump_r2(_sdcard->sd_ctrl);
	}
}
//
//
// READ SECTOR
//
//
int	sdcard_read(int sector, char *buf) {
	unsigned	*ubuf = (unsigned *)buf;
	int		j;

	if (SDDEBUG) {
		txstr("SDCARD-READ: ");
		txhex(sector);
		txstr("\r\n");
	}
	if (_sdcard->sd_ctrl & SDSPI_REMOVED) {
		txstr("ERR: SD-Card was removed\n");
		return -1;
	}

	// 512 byte block length, 25MHz clock
	//
	_sdcard->sd_data = SECTOR_512B | SPEED_FAST;
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	//
	// Issue the read command
	//
	_sdcard->sd_data = sector;
	_sdcard->sd_ctrl = SDSPI_READ_SECTOR;	// CMD 17, into FIFO 0
	SDSPI_WAIT_WHILE_BUSY;

#ifdef	INCLUDE_DMA_CONTROLLER
	if (SDUSEDMA && ((_zip->z_dma.d_ctrl & DMA_BUSY) == 0)) {
		_zip->z_dma.d_len= 512/sizeof(int);
		_zip->z_dma.d_rd = (unsigned *)&_sdcard->sd_fifo[0];
		_zip->z_dma.d_wr = &ubuf[0];
		_zip->z_dma.d_ctrl = DMACCOPY | DMA_CONSTSRC;
		while(_zip->z_dma.d_ctrl & DMA_BUSY)
			;
		CLEAR_CACHE;
	} else
#endif
		for(j=0; j<512/4; j++)
			ubuf[j] = _sdcard->sd_fifo[0];

	if (SDDEBUG && SDINFO)
		sdcard_dump_sector(ubuf);

	if (_sdcard->sd_ctrl & (SDSPI_ERROR | SDSPI_REMOVED)) {
		txstr("READ ERR!\r\n");
//		sdcard_get_r2();
		return -1;
	} return 0;
}

//
//
// WRITE SECTOR
//
//
int	sdcard_write(const int sector, const char *buf) {
	const unsigned *ubuf = (unsigned *)buf;
	unsigned	vc, vd;

	if (SDDEBUG) {
		txstr("SDCARD-WRITE: ");
		txhex(sector);
		txstr("\r\n");
	}

	if (_sdcard->sd_ctrl & SDSPI_REMOVED) {
		txstr("ERR: SD-Card was removed\n");
		return -1;
	}

	// For our next test, let us write and then read sector 2.
	_sdcard->sd_data = SECTOR_512B | SPEED_FAST;	// 128 word block length, 20MHz clock

	// Write config data, read last config data
	// This also resets our FIFO to the beginning, so we can start
	// writing into it from the beginning.
	_sdcard->sd_ctrl = SDSPI_SETAUX;

	if (SDDEBUG && SDINFO)
		sdcard_dump_sector(ubuf);

#ifdef	INCLUDE_DMA_CONTROLLER
	if (SDUSEDMA && ((_zip->z_dma.d_ctrl & DMA_BUSY) == 0)) {
		_zip->z_dma.d_len= 512/sizeof(int);
		_zip->z_dma.d_rd = (unsigned *)&ubuf[0];
		_zip->z_dma.d_wr = (unsigned *)&_sdcard->sd_fifo[0];
		_zip->z_dma.d_ctrl = DMACCOPY | DMA_CONSTDST;
		while(_zip->z_dma.d_ctrl & DMA_BUSY)
			;
	} else
#endif
		for(int i=0; i<512/4; i++)
			_sdcard->sd_fifo[0] = ubuf[i];

	_sdcard->sd_data = sector;
	_sdcard->sd_ctrl = SDSPI_WRITE_SECTOR;

	SDSPI_WAIT_WHILE_BUSY;

	vc = _sdcard->sd_ctrl;
	vd = _sdcard->sd_data;

	if (SDDEBUG) {
		if ((vc & SDSPI_ERROR) || ((vd & 0x01f) != 0x05)) {
			txstr("WRITE ERR!  Response token = ");
			txhex(vc);
			txstr(" : ");
			txhex(vd);
			txstr("\r\n");
		}

		if (SDINFO)
			sdcard_get_r2();
	}

	if (vc & SDSPI_ERROR) {
		_sdcard->sd_ctrl = SDSPI_CLEARERR;
		return -1;
	}
	return 0;
}


#endif	// _BOARD_HAS_SDSPI
