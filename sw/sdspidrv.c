////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sw/sdspidrv.c
// {{{
// Project:	SD-Card controller
//
// Purpose:	Provides low-level access to the SD-Card SPI controller.
//		Primary functions provided here are sdspi_init() to initialize
//	the card, sdspi_read(sector, buffer) and sdspi_write(sector, buffer)
//	to read from and write to the card respectively.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2016-2024, Gisselquist Technology, LLC
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
// }}}
#define	STDIO_DEBUG
#include <stdlib.h>
#include <stdint.h>
typedef	uint8_t		BYTE;
typedef	uint16_t	WORD;
typedef	uint32_t	DWORD, LBA_t, UINT;
#include <diskio.h>

#ifdef	STDIO_DEBUG
#include <stdio.h>
#include <stdlib.h>

#define	txstr(A)	printf("%s", A)
#define	txhex(A)	printf("%08x", A)
#define	txdecimal(A)	printf("%d", A)

#else
// Embedded I/O functions txstr() and txhex
#include "txfns.h"
#endif

#include "sdspidrv.h"

#define	SDSPI_SETAUX	0x0000ff
#define	SDSPI_READAUX	0x0000bf
#define	SDSPI_CMD		0x000040
#define	SDSPI_ACMD		(0x040+55) // CMD55
#define	SDSPI_FIFO_OP	0x000800	// Read only
#define	SDSPI_WRITEOP	0x000c00	// Write to the FIFO
#define	SDSPI_ALTFIFO	0x001000
#define	SDSPI_BUSY		0x004000
#define	SDSPI_ERROR		0x008000
#define	SDSPI_CLEARERR	0x008000
// #define	SDSPI_CRCERR	0x010000
// #define	SDSPI_ERRTOK	0x020000
#define	SDSPI_REMOVED	0x040000
#define	SDSPI_PRESENTN	0x080000
#define	SDSPI_RESET		0x100000	// Read only
#define	SDSPI_WATCHDOG	0x200000	// Read only
#define	SDSPI_GO_IDLE	((SDSPI_REMOVED|SDSPI_CLEARERR|SDSPI_CMD)+0)
#define	SDSPI_READ_SECTOR	((SDSPI_CMD|SDSPI_CLEARERR|SDSPI_FIFO_OP)+17)
#define	SDSPI_WRITE_SECTOR	((SDSPI_CMD|SDSPI_CLEARERR|SDSPI_WRITEOP)+24)

#define	SDERR_READ	0x001
#define	SDERR_WRITE	0x002
#define	SDERR_INIT	0x010

#define	SPEED_25MHZ	0x01
#define	SPEED_17MHZ	0x02
#define	SPEED_12MHZ	0x03
#define	SPEED_10MHZ	0x04
#define	SPEED_400KHZ	0x7c
#define	SPEED_200KHZ	0xf9
#define	SPEED_100KHZ	0x1f3
#define	SPEED_SLOW	SPEED_400KHZ
#define	SPEED_FAST	SPEED_25MHZ
// #define	SPEED_FAST	SPEED_17MHZ
// #define	SPEED_FAST	SPEED_400KHZ

#define	SECTOR_8B	0x030000	// Used by SCR register
#define	SECTOR_16B	0x040000	// CSD and CID registers
#define	SECTOR_512B	0x090000	// 512-byte disk sectors
//
//
// Book recommen ds sequence of: CMD0, (CMD8), CMD58, ACMD41(wait), CMD58
//	for startup
//

// #include "zipsys.h"

// #include <zipcpu.h>


#define	SDSPI_READREG	0x0200	// Expect an R3 or R7 response (R1 + 32-bits)

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

typedef	struct SDSPI_S {
	volatile unsigned	sd_ctrl, sd_data, sd_fifo[2];
} SDSPI;

typedef	struct SDSPIDRV_S {
	SDSPI		*d_dev;
	char		d_SCR[8];
	char		d_CSD[16], d_CID[16];
	uint32_t	d_OCR;
	uint32_t	d_sector_count, d_block_size;
} SDSPIDRV;

#define	SDSPI_WAIT_WHILE_BUSY(DEV)	{ int r; do { r = DEV->d_dev->sd_ctrl; if (r & (SDSPI_ERROR|SDSPI_PRESENTN)) break; } while(r & SDSPI_BUSY); }

//
// SCOPE
//
// If defined, references the address of a WBSCOPE on the bus which can be used
// for debugging various interactions
// #define		SCOPE	_scope_sdcard

//
//
// Book recommends sequence of: CMD0, (CMD8), CMD58, ACMD41(wait), CMD58
//	for startup
//

int	sdcard_err = 0;

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

static	int	sdspi_read_ocr(SDSPIDRV *dev) {
	// {{{
	unsigned	v;

	if (SDDEBUG)
		txstr("SDCARD: CMD58, READ-OCR\n");

	//
	// CMD58 : READ-OCR
	// After the card is ready, we must send a READ_OCR command next
	// The card will respond with an R7
	//
	dev->d_dev->sd_data = 0x0;
	dev->d_dev->sd_ctrl = (SDSPI_CMD|SDSPI_READREG) + 58; // 0x027a;
	SDSPI_WAIT_WHILE_BUSY(dev);

	if ((v = dev->d_dev->sd_ctrl)!= 0) {
		sdcard_err |= SDERR_INIT | 0x10000;
		if (SDDEBUG) {
			txstr("READ-OCR: Ctrl != 0, ");
			txhex(v);
			txstr("\r\n");
		}
	}


	dev->d_OCR = dev->d_dev->sd_data;
	if (SDINFO) {
		txstr("READ-OCR: OCR = "); txhex(dev->d_OCR); txstr("\n");
	// printf("    OCR = 0x%08x%s\n", v, (v!=0x40ff8000)?"Shouldnt this be 0x40ff8000?":"");

		if (dev->d_OCR & 0x80000000)
			txstr("  Card is still powering up\n");
		if (dev->d_OCR & 0x40000000)
			txstr("  CCS: High capacity support\n");
		if (dev->d_OCR & 0x20000000)
			txstr("  UHS: USH-II card\n");
		if (dev->d_OCR & 0x01000000)
			txstr("  S18A: Switching to 1.8V allowed\n");
#ifdef	STDIO_DEBUG
		int	mxv = 0, mnv = 0;
		if (dev->d_OCR & 0x00800000) {
			if ((mxv == 0)||(mxv <= 36))
				mxv = 36;
			if ((mnv == 0)||(mnv >= 35))
				mnv = 35;
		} if (dev->d_OCR & 0x00400000) {
			if ((mxv == 0)||(mxv <= 35))
				mxv = 35;
			if ((mnv == 0)||(mnv >= 34))
				mnv = 34;
		} if (dev->d_OCR & 0x00200000) {
			if ((mxv == 0)||(mxv <= 34))
				mxv = 34;
			if ((mnv == 0)||(mnv >= 33))
				mnv = 33;
		} if (dev->d_OCR & 0x00100000) {
			if ((mxv == 0)||(mxv <= 33))
				mxv = 33;
			if ((mnv == 0)||(mnv >= 32))
				mnv = 32;
		} if (dev->d_OCR & 0x00080000) {
			if ((mxv == 0)||(mxv <= 32))
				mxv = 32;
			if ((mnv == 0)||(mnv >= 31))
				mnv = 31;
		} if (dev->d_OCR & 0x00040000) {
			if ((mxv == 0)||(mxv <= 31))
				mxv = 31;
			if ((mnv == 0)||(mnv >= 30))
				mnv = 30;
		} if (dev->d_OCR & 0x00020000) {
			if ((mxv == 0)||(mxv <= 30))
				mxv = 30;
			if ((mnv == 0)||(mnv >= 29))
				mnv = 29;
		} if (dev->d_OCR & 0x00010000) {
			if ((mxv == 0)||(mxv <= 29))
				mxv = 29;
			if ((mnv == 0)||(mnv >= 28))
				mnv = 28;
		} if (dev->d_OCR & 0x00008000) {
			if ((mxv == 0)||(mxv <= 28))
				mxv = 28;
			if ((mnv == 0)||(mnv >= 27))
				mnv = 27;
		}

		printf("  Voltage ranges supported: %d.%dV - %d.%dV\n",
			(mxv/10), (mxv%10), (mnv/10), (mnv%10));
#else
		if (dev->d_OCR & 0x00800000)
			txstr("  3.6-3.5 V allowed\n");
		if (dev->d_OCR & 0x00400000)
			txstr("  3.5-3.4 V allowed\n");
		if (dev->d_OCR & 0x00200000)
			txstr("  3.4-3.3 V allowed\n");
		if (dev->d_OCR & 0x00100000)
			txstr("  3.3-3.2 V allowed\n");
		if (dev->d_OCR & 0x00080000)
			txstr("  3.2-3.1 V allowed\n");
		if (dev->d_OCR & 0x00040000)
			txstr("  3.1-3.0 V allowed\n");
		if (dev->d_OCR & 0x00020000)
			txstr("  3.0-2.9 V allowed\n");
		if (dev->d_OCR & 0x00010000)
			txstr("  2.9-2.8 V allowed\n");
		if (dev->d_OCR & 0x00008000)
			txstr("  2.8-2.7 V allowed\n");
#endif
	}

	return dev->d_OCR;
}
// }}}

//
// Read the SCR register
static	int	sdspi_read_scr(SDSPIDRV *dev) {
	// {{{
	unsigned	v;

	if (SDDEBUG)
		txstr("READ-SCR\n");

	// The SCR register is 64 bits, and we can read it at fast speed
	dev->d_dev->sd_data = SECTOR_8B | SPEED_FAST;
	dev->d_dev->sd_ctrl = SDSPI_SETAUX;

	//
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_ctrl = (SDSPI_CLEARERR|SDSPI_ACMD); // Go to alt command set
	SDSPI_WAIT_WHILE_BUSY(dev);

	if (dev->d_dev->sd_ctrl & (SDSPI_ERROR | SDSPI_REMOVED)) {
		sdcard_err |= SDERR_INIT | 0x4000000;
		return -1;
	}

	dev->d_dev->sd_data = 0x0;	// Read from position zero
	dev->d_dev->sd_ctrl = (SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD)+51;
	SDSPI_WAIT_WHILE_BUSY(dev);

	if ((v = dev->d_dev->sd_ctrl) != 0)
		sdcard_err |= SDERR_INIT | 0x4000000;
	if ((v = dev->d_dev->sd_data) != 0xffffffff)
		sdcard_err |= SDERR_INIT | 0x8000000;

	if (SDINFO) txstr("SCR: ");

	for(v=0; v<8; v+=4) {
		unsigned	u;

		u = dev->d_dev->sd_fifo[0];
		if (SDINFO) { txhex(u); if (v+4 < 8) txstr(":");
			else txstr("\r\n"); }
		dev->d_SCR[v+3] = u & 0x0ff; u>>= 8;
		dev->d_SCR[v+2] = u & 0x0ff; u>>= 8;
		dev->d_SCR[v+1] = u & 0x0ff; u>>= 8;
		dev->d_SCR[v+0] = u & 0x0ff;
	}

	if (dev->d_dev->sd_ctrl & SDSPI_ERROR)
		return -1;
	return 0;
}
// }}}

//
// Read the CSD register
//
static	int	sdspi_read_csd(SDSPIDRV *dev) {
	// {{{
	unsigned	*ucsd = (unsigned *)dev->d_CSD;
	int		i;
	unsigned	v;

	if (sdcard_err != 0)
		return -1;

	if (SDDEBUG)
		txstr("READ-CSD\n");

	//
	// Adjust the FIFO for a 4-word length
	//
	dev->d_dev->sd_ctrl = SDSPI_READAUX;
	// 16 byte block length
	dev->d_dev->sd_data = SECTOR_16B;
	// Write config data, read last config data
	dev->d_dev->sd_ctrl = SDSPI_SETAUX;

	//
	// CMD9 : SEND_CSD_COND
	// Send CSD condition to FIFO #0 (Requires FIFO support)
	//
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_ctrl = (SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD) + 9; // 0x08849;
	SDSPI_WAIT_WHILE_BUSY(dev);

	if ((v = dev->d_dev->sd_ctrl) != 0) {
		txstr("READ-CSD: Invalid CSD return control value, ");
		txhex(v);
		txstr("\n");
		sdcard_err |= SDERR_INIT | 0x20000;
		return sdcard_err;
	} if ((v = dev->d_dev->sd_data) != 0) {
		if (v & 0x010)
			txstr("READ-CSD: Return indicates CRC ERR, ");
		else
			txstr("READ-CSD: Unexpected return data, ");
		txhex(v);
		txstr("\n");
		sdcard_err |= SDERR_INIT | 0x40000;
		return sdcard_err;
	}

	for(i=0; i<4; i++)
		ucsd[i] = dev->d_dev->sd_fifo[0];

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
// }}}

static	int	sdspi_read_cid(SDSPIDRV *dev) {
	// {{{
	unsigned	*ucid = (unsigned *)dev->d_CID;
	int		i;
	unsigned	v;

	if (SDDEBUG)
		txstr("READ-CID\n");

	// CMD10 : SEND_CID_COND
	// Send CID condition to FIFO #1 (Requires reading from FIFO)
	//
	// Adjust the FIFO for a 4-word length
	//
	dev->d_dev->sd_ctrl = SDSPI_READAUX;
	// 16 byte block length
	dev->d_dev->sd_data = SECTOR_16B;
	// Write config data, read last config data
	dev->d_dev->sd_ctrl = SDSPI_SETAUX;

	dev->d_dev->sd_data = 0x0;
	dev->d_dev->sd_ctrl = (SDSPI_CLEARERR|SDSPI_FIFO_OP|SDSPI_CMD)+10; // 0x0984a;
	SDSPI_WAIT_WHILE_BUSY(dev);

	if ((v = dev->d_dev->sd_ctrl) != 0x0) { // Expecting 0x0
		txstr("READ-CID: Expecting a control response of 0x0, not ");
		txhex(v);
		txstr("\r\n");
		sdcard_err |= SDERR_INIT | 0x100000;
	}
	if ((v = dev->d_dev->sd_data) != 0) {
		if (v & 0x10)
			txstr("READ-CID: Rcvd a CRC err, ");
		else
			txstr("READ-CID: Expecting a data field of 0x..f00, not ");
		txhex(v);
		txstr("\r\n");
		sdcard_err |= SDERR_INIT | 0x200000;
	}

	// 0x0824d; // CMD thirteen -- SEND_STATUS
	dev->d_dev->sd_data = 0x0;
	dev->d_dev->sd_ctrl = (SDSPI_CLEARERR|SDSPI_READREG|SDSPI_CMD)+ 13;

	SDSPI_WAIT_WHILE_BUSY(dev);

	// Read out the CID
	for(i=0; i<4; i++)
		ucid[i] = dev->d_dev->sd_fifo[0];

	if (1 || SDINFO) {
		txstr("CID: ");
		txhex(ucid[0]); txstr(":");
		txhex(ucid[1]); txstr(":");
		txhex(ucid[2]); txstr(":");
		txhex(ucid[3]); txstr("\n");
	}

	if ((v = dev->d_dev->sd_ctrl) != 0) { // 0
		txstr("READ-CID: 2nd read, expecting control value of 0, not ");
		txhex(v);
		txstr("\r\n");
		// sdcard_err |= SDERR_INIT | 0x400000;
	}

	if ((v = dev->d_dev->sd_data) != 0x00ffffff) { // Finally, read the cards status
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

		sn = (dev->d_CID[9]&0x0ff);
		sn = (sn<<8) | (dev->d_CID[10]&0x0ff);
		sn = (sn<<8) | (dev->d_CID[11]&0x0ff);
		sn = (sn<<8) | (dev->d_CID[12]&0x0ff);
		md = (dev->d_CID[13] & 0x0f);
		md = (md << 8) | (dev->d_CID[14] & 0x0ff);

		printf("CID:\n"
"\tManufacturer ID:  0x%02x\n"
"\tApplication ID:   %c%c\n"
"\tProduct Name:     %c%c%c%c%c\n"
"\tProduct Revision: %x.%x\n"
"\tSerial Number:    0x%0x\n",
		dev->d_CID[0]&0x0ff,
		dev->d_CID[1], dev->d_CID[2],
		dev->d_CID[3], dev->d_CID[4], dev->d_CID[5],
				dev->d_CID[6], dev->d_CID[7],
		(dev->d_CID[8]>>4)&0xf, dev->d_CID[8]&0x0f,
		sn);
		printf(
"\tYear of Man.:     %d\n"
"\tMonth of Man.:    %d\n",
	((md>>4)+2000), md&0x0f);
	}
#endif

	return (sdcard_err == 0) ? 0 : -1;
}
// }}}


//
//
// DUMP Control register
//
//
static void	sdspi_dump_ctrlreg(const unsigned rv) {
	// {{{
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
// }}}


//
//
// DUMP R1
//
//
static void	sdspi_dump_r1(const unsigned rv) {
	// {{{
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
// }}}

static void	sdspi_dump_r2(const unsigned rv) {
	// {{{
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
		sdspi_dump_r1(uv >> 8);
	}
}
// }}}

static void	sdspi_dump_sector(const unsigned *ubuf) {
	// {{{
	int	j;

	for(j=0; j<512/4; j++) {
		txhex(ubuf[j]);
		if ((j&7)==7)
			txstr("\r\n");
		else
			txstr(" ");
	} txstr("\r\n");
}
// }}}

static	unsigned	sdspi_get_r2(SDSPIDRV *dev) {
	// {{{
	dev->d_dev->sd_ctrl = SDSPI_ERROR | SDSPI_READREG | SDSPI_CMD + 13;
	SDSPI_WAIT_WHILE_BUSY(dev);

	if (SDDEBUG) {
		txstr("SD_WRITE.STAT: Status return, "); txhex(dev->d_dev->sd_ctrl);
		txstr(" : "); txhex(dev->d_dev->sd_data);
		txstr("\r\n");

		sdspi_dump_r2(dev->d_dev->sd_ctrl);
	}
}
// }}}

SDSPIDRV *sdspi_init(SDSPI *dev) {
	// {{{
	// int		*data = debug_data;
	int		i, j;
	unsigned	v;
	SDSPIDRV *dv = (SDSPIDRV *)malloc(sizeof(SDSPIDRV));
	dv->d_dev = dev;
	dv->d_sector_count = 0;
	dv->d_block_size   = 0;

#ifdef	SCOPE
	const unsigned	SCOPEDELAY = 0x100;

	SCOPE->s_ctrl = WBSCOPE_DISABLE | SCOPEDELAY;
#endif
	if (SDDEBUG)
		txstr("SDCARD-INIT\n");

	// Start us out slow, with a known sector length
	dev->sd_data = SECTOR_512B | SPEED_SLOW;	// 128 word block length, 400kHz clock
	dev->sd_ctrl = SDSPI_SETAUX; // Write config data, read last config data

	// Clear any prior pending errors
	dev->sd_data = 0;
	dev->sd_ctrl = SDSPI_CLEARERR|SDSPI_READAUX;

	sdcard_err = 0;
	if (dev->sd_ctrl & SDSPI_PRESENTN) {
		txstr("SDCARD: No card present\n");
		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		sdcard_err = -1;
		return dv;
	}

	//////////////////////////////////////////////////////////////////////
	//
	// CMD0: Command zero, reset the card interaction
	// {{{
	// Reset the card by sending a command zero
	//
	/////////////////
	//
	//
	if (SDDEBUG)
		txstr("SDCARD: CMD0 GO_IDLE_STATE\n");

	dev->sd_data = 0;
	dev->sd_ctrl = SDSPI_GO_IDLE; // CMD zero
	SDSPI_WAIT_WHILE_BUSY(dv);

	v = dev->sd_ctrl;
	if (v & SDSPI_ERROR) {
#ifdef	SCOPE
		SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
		txstr("No response from card to reset command\n");
#ifdef	GPIO_SD_RESET
		*_gpio = GPIO_SET(GPIO_SD_RESET);
#endif
		sdcard_err |= SDERR_INIT | 0x800;
		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		return dv;
	}

	if (SDDEBUG) {
		txstr("Response: ");
		txhex(v);
		txstr("\r\n");
	}

	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// CMD1 : SEND_OP_COND
	// {{{
	//

	if (0) { // CMD1: SEND_OP_COND
		// {{{
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
		dev->sd_data = 0x40000000;	// We support high capacity cards
		dev->sd_ctrl = SDSPI_CMD+1;
		SDSPI_WAIT_WHILE_BUSY(dv);

		if ((v = dev->sd_ctrl)!= 1) {
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


		if ((v = dev->sd_data)!= -1) {
			txstr("SDCARD: SEND_OP_COND data returned ");
			txhex(v);
			txstr(", not -1\r\n");
			sdcard_err |= SDERR_INIT | 0x800;

#ifdef	SCOPE
			SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
			dv->d_sector_count = 0;
			dv->d_block_size   = 0;
			return dv;
		}

		if (SDDEBUG) {
			txstr("Response: ");
			txhex(dev->sd_ctrl);
			txstr("\r\n");
		}
		// }}}
	}
	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// CMD8 : SEND_IF_COND, Send interface condition
	// {{{
	/////////////////
	//
	//
	if (SDDEBUG)
		txstr("SDCARD: CMD8 SEND_IF_COND (3.3v)\n");
	dev->sd_data = 0x001a5;
	dev->sd_ctrl = (SDSPI_CMD|SDSPI_READREG)+8;
	SDSPI_WAIT_WHILE_BUSY(dv);

	if ((v = dev->sd_data)!= 0x01a5) {
#ifdef	SCOPE
		SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
		txstr("SDCARD: SEND_IF_COND status ");
		txhex(v);
		txstr(", != 0x01a5 as expected\r\n");
		sdcard_err |= SDERR_INIT | 0x2000;

		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		return dv;
	}

	if (SDDEBUG) {
		txstr("Response: ");
		txhex(dev->sd_ctrl);
		txstr(" : ");
		txhex(dev->sd_data);
		txstr("\r\n");
	}
	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// ACMD41: Issue ACMD41 until the card has fully started
	// {{{
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
			dev->sd_data = 0;
			dev->sd_ctrl = SDSPI_ACMD;	// 0x70 = 'd55 + 'h40
			SDSPI_WAIT_WHILE_BUSY(dv);

			//
			// Send the ACMD41 itself
			//
			dev->sd_data = 0x40000000; // Support high capacity
			dev->sd_ctrl = SDSPI_CMD + 41; // 0x69; // 0x040+41;
			SDSPI_WAIT_WHILE_BUSY(dv);

			dev_busy = dev->sd_ctrl&1;
			iterations++;

			if (SDDEBUG && (dev->sd_ctrl == 1))
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

		vc = dev->sd_ctrl;
		vd = dev->sd_data;

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
			dev->sd_ctrl = SDSPI_READAUX;
			vd = dev->sd_data;
			txhex(vd);
			if (sdcard_err & 0x0c000) {
				txstr("\r\nERR: Card still idle\r\n");
				// zip_halt();
			} else
				txstr("\r\n");
		}

		if (sdcard_err & 0xc000) {
#ifdef	SCOPE
			SCOPE->s_ctrl = WBSCOPE_TRIGGER | SCOPEDELAY;
#endif
			txstr("SDCard Err :");
			txhex(sdcard_err);
			txstr("\n");
			dv->d_sector_count = 0;
			dv->d_block_size   = 0;
			return dv;
		}
	}
	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// Read the OCR register
	// {{{
	/////////////////
	//
	//
	sdspi_read_ocr(dv);

	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// Change the clock speed to high speed
	// {{{
	dev->sd_data = 0x0401;	// 16 byte block length, 25MHz clock
	// Write config data, read last config data
	dev->sd_ctrl = SDSPI_SETAUX;

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
		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		return dv;
	}

	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// Read the CSD register
	// {{{
	/////////////////
	//
	//
	if ((v = sdspi_read_csd(dv)) != 0) {
		txstr("SDCARD: READCSD returned ");
		txhex(v);
		txstr("\r\n");
		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		return dv;
	}

	if (0x00 == (dv->d_CSD[0] & 0xc0)) {
		// {{{
		// Standard capacity card, CSD v1
		//
		unsigned	C_SIZE, C_SIZE_MULT, READ_BL_LEN,
				BLOCK_LEN, BLOCKNR, SECTOR_SIZE,
				WRITE_BL_LEN;

		C_SIZE  =  dv->d_CSD[6] & 0x0ff;
		C_SIZE |= (dv->d_CSD[7] & 0x0ff) | (C_SIZE << 8);
		C_SIZE |= (dv->d_CSD[8] & 0x0ff) | (C_SIZE << 8);

		C_SIZE >>= 6;
		C_SIZE &= 0x0fff;

		C_SIZE_MULT  = (dv->d_CSD[ 9]& 0x0ff);
		C_SIZE_MULT |= (dv->d_CSD[10]& 0x0ff) | (C_SIZE_MULT << 8);

		C_SIZE_MULT >>= 7;
		C_SIZE_MULT &= 0x07;

		READ_BL_LEN = (dv->d_CSD[ 5]& 0x0f);

		BLOCK_LEN = (1ul<<(READ_BL_LEN));
		// gbl_sector_size = BLOCK_LEN;
		BLOCKNR  = (C_SIZE+1ul) * (1ul << (C_SIZE_MULT+2));
		dv->d_sector_count = BLOCKNR;

		// Get the size of an erasable sector
		SECTOR_SIZE  = (dv->d_CSD[10]& 0x0ff);
		SECTOR_SIZE |= (dv->d_CSD[11]& 0x0ff) | (SECTOR_SIZE << 8);
		SECTOR_SIZE >>= 7;
		SECTOR_SIZE &= 0x07f;


		WRITE_BL_LEN  = (dv->d_CSD[12]& 0x03);
		WRITE_BL_LEN |= (dv->d_CSD[13]& 0x0ff) | (WRITE_BL_LEN << 8);
		WRITE_BL_LEN >>= 6;
		WRITE_BL_LEN &= 0x0f;
#ifdef	STDIO_DEBUG
		if (SDDEBUG) {
			printf("LO:\n"
			"  C_SIZE_MULT  = %6d\n"
			"  C_SIZE       = %6d\n"
			"  READ_BL_LEN  = %6d\n"
			"  SECTOR_SIZE  = %6d\n"
			"  WRITE_BL_LEN = %6d\n",
					C_SIZE_MULT, C_SIZE,
					READ_BL_LEN, SECTOR_SIZE,
					WRITE_BL_LEN);
		}
#endif

		dv->d_block_size = (SECTOR_SIZE + 2)*WRITE_BL_LEN;
		// }}}
	} else if (0x40 == (dv->d_CSD[0] & 0xc0)) {
		// {{{
		// High capacity and extended capacity cards, CSD v2
		//
		unsigned	C_SIZE, READ_BL_LEN,
				BLOCK_LEN, BLOCKNR;

		READ_BL_LEN = 9;

		BLOCK_LEN = (1 << (READ_BL_LEN));

		C_SIZE = (dv->d_CSD[7]& 0x0ff);
		C_SIZE = (dv->d_CSD[8]& 0x0ff) | (C_SIZE << 8);
		C_SIZE = (dv->d_CSD[9]& 0x0ff) | (C_SIZE << 8);
		C_SIZE = 0x03fffff;

		// User data capacity is given by (C_SIZE+1)*512kB
		// Hence we have (C_SIZE+1)*512*1024 / 512 sectors, or...
		dv->d_sector_count = (C_SIZE+1ul) * 1024;
#ifdef	STDIO_DEBUG
		if (SDDEBUG) {
			printf("HS: C_SIZE = %d\n", C_SIZE);
			printf("  From ... %02x%02x%02x\n",
				dv->d_CSD[7]&0x0ff,dv->d_CSD[8]&0x0ff,
				dv->d_CSD[9]&0x0ff);
			printf("  Card Size = %lu\n", (unsigned long)dv->d_sector_count * 512ul);
		}
#endif

		dv->d_block_size = 512;
		// }}}
	} else {
		// {{{
#ifdef	STDIO_DEBUG
		printf("Unrecognizable CSD: %02x %02x %02x ...\n",
			dv->d_CSD[0] & 0x0ff, dv->d_CSD[1] & 0x0ff,
			dv->d_CSD[2] & 0x0ff);
#else
		txstr("Unrecognizable CSD type\n");
#endif	// STDIO_DEBUG
		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		// }}}
	}

	// }}}
	//////////////////////////////////////////////////////////////////////
	//
	// Read the CID register
	// {{{
	/////////////////
	//
	//
	if ((v = sdspi_read_cid(dv)) != 0) {
		txstr("SDCARD: READCID returned ");
		txhex(v);
		txstr("\r\n");
		dv->d_sector_count = 0;
		dv->d_block_size   = 0;
		return dv;
	}

	// }}}

	return 0;
}
// }}}

int	sdspi_read(SDSPIDRV *dev, const unsigned sector, const unsigned count, char *buf) {
	// {{{
	unsigned	*ubuf = (unsigned *)buf, k;
	int		j, st = 0;

	if (SDDEBUG) {
		txstr("SDCARD-READ: ");
		txhex(sector);
		if (count > 1) {
			txstr(", +");
			txhex(count);
		} txstr("\r\n");
	} if (sector + count > dev->d_sector_count)
		return RES_PARERR;
	for(k=0; k<count; k++) {
		unsigned	*ubuf = (unsigned *)&buf[k*512];

		if (dev->d_dev->sd_ctrl & SDSPI_REMOVED) {
			txstr("ERR: SD-Card was removed\n");
			st = RES_ERROR;
			break;
		}

		// 512 byte block length, 25MHz clock
		//
		// Write config data, read last config data
		dev->d_dev->sd_data = SECTOR_512B | SPEED_FAST;
		dev->d_dev->sd_ctrl = SDSPI_SETAUX;

		//
		// Issue the read command
		//
		dev->d_dev->sd_data = sector+k; // sector to read from
		dev->d_dev->sd_ctrl = SDSPI_READ_SECTOR;	// CMD 17, into FIFO 0
		SDSPI_WAIT_WHILE_BUSY(dev);

#ifdef	INCLUDE_DMA_CONTROLLER
		if (SDUSEDMA && ((_zip->z_dma.d_ctrl & DMA_BUSY) == 0)) {
			_zip->z_dma.d_len= 512/sizeof(char);
			_zip->z_dma.d_rd = (char *)&dev->d_dev->sd_fifo[0];
			_zip->z_dma.d_wr = &buf[k*512];
			_zip->z_dma.d_ctrl = DMAREQUEST|DMACLEAR|DMA_DSTWIDE
					| DMA_CONSTSRC|DMA_SRCWORD;
			while(_zip->z_dma.d_ctrl & DMA_BUSY)
				;
			CLEAR_DCACHE;
		} else
#endif
			for(j=0; j<512/4; j++)
				ubuf[j] = dev->d_dev->sd_fifo[0];

		if (SDDEBUG && SDINFO)
			sdspi_dump_sector(ubuf);

		if (dev->d_dev->sd_ctrl & (SDSPI_ERROR | SDSPI_REMOVED)) {
			txstr("READ ERR!\r\n");
//			sdspi_get_r2(dev);
			st = -1;
			break;
		}
	} return st;
}
// }}}

int	sdspi_write(SDSPIDRV *dev, const unsigned sector, const unsigned count, const char *buf) {
	// {{{
	unsigned	vc, vd, st = 0, k;

	if (SDDEBUG) {
		txstr("SDCARD-WRITE: ");
		txhex(sector);
		if (count > 1) {
			txstr(", +");
			txhex(count);
		} txstr("\r\n");
	}

	for(k=0; k<count; k++) {
		const unsigned *ubuf = (unsigned *)(&buf[k*512]);

		if (dev->d_dev->sd_ctrl & SDSPI_REMOVED) {
			txstr("ERR: SD-Card was removed\n");
			st = -1;
			break;
		}

		// For our next test, let us write and then read sector 2.
		//	 128 word block length, 20MHz clock
		dev->d_dev->sd_data = SECTOR_512B | SPEED_FAST;

		// Write config data, read last config data
		// This also resets our FIFO to the beginning, so we can start
		// writing into it from the beginning.
		dev->d_dev->sd_ctrl = SDSPI_SETAUX;

		if (SDDEBUG && SDINFO)
			sdspi_dump_sector(ubuf);

#ifdef	INCLUDE_DMA_CONTROLLER
		if (SDUSEDMA && ((_zip->z_dma.d_ctrl & DMA_BUSY) == 0)) {
			_zip->z_dma.d_len= 512/sizeof(char);
			_zip->z_dma.d_rd = (char *)&ubuf;
			_zip->z_dma.d_wr = (char *)&dev->d_dev->sd_fifo[0];
			_zip->z_dma.d_ctrl = DMAREQUEST|DMACLEAR|DMA_SRCWIDE
						| DMA_CONSTDST|DMA_DSTWORD;
			while(_zip->z_dma.d_ctrl & DMA_BUSY)
				;
		} else
#endif
			for(int i=0; i<512/4; i++)
				dev->d_dev->sd_fifo[0] = ubuf[i];

		dev->d_dev->sd_data = sector+k;
		dev->d_dev->sd_ctrl = SDSPI_WRITE_SECTOR;

		SDSPI_WAIT_WHILE_BUSY(dev);

		vc = dev->d_dev->sd_ctrl;
		vd = dev->d_dev->sd_data;

		if (SDDEBUG) {
			if ((vc & SDSPI_ERROR) || ((vd & 0x01f) != 0x05)) {
				txstr("WRITE ERR!  Response token = ");
				txhex(vc);
				txstr(" : ");
				txhex(vd);
				txstr("\r\n");
			}

			if (SDINFO)
				sdspi_get_r2(dev);
		}

		if (vc & SDSPI_ERROR) {
			dev->d_dev->sd_ctrl = SDSPI_CLEARERR;
			st = -1;
			break;
		}
	}
	return 0;
}
// }}}

int	sdspi_ioctl(SDSPIDRV *dev, char cmd, char *buf) {
	// {{{
	int		dstat;
	unsigned	vc;

	vc = dev->d_dev->sd_ctrl;
	if (vc & SDSPI_PRESENTN)
		return RES_ERROR;
	if (vc & SDSPI_REMOVED)
		return	RES_NOTRDY;

	switch(cmd) {
	case CTRL_SYNC: {
			SDSPI_WAIT_WHILE_BUSY(dev);
			return	RES_OK;
		} break;
	case GET_SECTOR_COUNT:
		{	DWORD	*w = (DWORD *)buf;
			*w = dev->d_sector_count;
			return RES_OK;
		} break;
		break;
	case GET_SECTOR_SIZE:
		{	WORD	*w = (WORD *)buf;
			*w = 512;	// *MUST* be
			return RES_OK;
		} break;
	case GET_BLOCK_SIZE:
		{	DWORD	*w = (DWORD *)buf;
			*w = dev->d_block_size; // (gbl_erase_sector_size / gbl_sector_size);
			return RES_OK;
		} break;
	}

	return	RES_PARERR;
}
// }}}
