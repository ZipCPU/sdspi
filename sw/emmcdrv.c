////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sw/emmcdrv.c
// {{{
// Project:	SD-Card controller
//
// Purpose:	
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
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
typedef	uint8_t  BYTE;
typedef	uint16_t WORD;
typedef	uint32_t DWORD, LBA_t, UINT;
#include <diskio.h>
#include "emmcdrv.h"

// tx* -- debugging output functions
// {{{
// Debugging isn't quite as simple as using printf(), since I want to guarantee
// that I can still compile and build this into a memory that isn't large enough
// to hold a printf function.  The txstr(), txchr(), and txhex() functions fit
// that low memory footprint need.  For cases where these are not sufficient,
// we use the STDIO_DEBUG flag to determine if the regular ?rintf() functions
// are available.
#ifndef	TXFNS_H
#include <stdio.h>

#define	txchr(A)		putchar(A)
#define	txstr(A)		fputs(A, stdout)
#define	txhex(A)		printf("%08x", A)
#define	txdecimal(A)		printf("%d", A)
#define	STDIO_DEBUG
#else
// extern	void	txstr(const char *);
// extern	void	txhex(unsigned);
// extern	void	txdecimal(int);
#define	printf(...)
#endif
// }}}

// WBScope
// {{{
// The following defines may be useful when a WBScope has been attached
// to the device.  They are mostly useful for debugging.  If no WBScope
// is attached, these macros are given null definitions.
#ifdef	_BOARD_HAS_EMMCSCOPE
#define	SET_SCOPE	_emmcscope->s_ctrl = 0x04000100
#define	TRIGGER_SCOPE	_emmcscope->s_ctrl = 0xff000100
#else
#define	SET_SCOPE
#define	TRIGGER_SCOPE
#endif
// }}}

// EMMCINFO & EMMCDEBUG
// {{{
// These are designed to be compile time constants, to allow the compiler to
// remove the logic they generate for space reasons.
//	EMMCDEBUG: Set to turn on debugging output.  Ideally, debugging output
//		should not be necessary in a *working* design, so its use is
//		primary until the design starts working.  This debugging
//		output will tell you (among other things) when the design
//		issues commands, what command are issued, which routines are
//		called.  etc.
//	EMMCINFO: Set to turns on a verbose reporting.  This will dump values of
//		registers, together with their meanings.  When reading,
//		it will dump sectors read.  Often requires EMMCDEBUG.
static	const int	EMMCINFO = 1, EMMCDEBUG=1;
// }}}
// Compile time DMA controls
// {{{
// OPT_SDIODMA will be defined if the design was built in an environment that
// (may have potentially) had a hardware DMA built into it.  (We'll still
// check.)  If not, EXTDMA controls whether or not calls to an _external_
// DMA should be generated.  These would be calls to the ZipCPU's DMA.  To
// avoid all external DMA calls, simply set EXTDMA to zero.
#ifdef	OPT_SDIODMA
static	const	int	EXTDMA = 0;
#else
static	const	int	EXTDMA = 1;
#endif
// }}}

// EMMCMULTI
// {{{
// EMMCMULTI: Controls whether the read multiple block or write multiple block
// commands will be used.  Set to 1 to use these commands, 0 otherwise.
// EMMCMULTI *must* be set to use the internal DMA, otherwise only block level
// commands will be issued.
static	const	int	EMMCMULTI = 1;
// }}}
// }}}

#define	NEW_MUTEX
#define	GRAB_MUTEX
#define	RELEASE_MUTEX
#ifndef	CLEAR_DCACHE
#define	CLEAR_DCACHE
#endif

typedef	struct	EMMCDRV_S {
	EMMC		*d_dev;
	uint32_t	d_CID[4], d_OCR;
	char		d_SCR[8], d_CSD[16], d_EXCSD[512];
	uint16_t	d_RCA;	// Relative Card Address
			// d_sector_count: the number of 512B blocks / device
	uint32_t	d_sector_count,
			// d_block_size = 512 (or at least better be ...)
			d_block_size,
			// d_DMA: A true/false flag indicating if we are to use
			// the DMA (true) or not (false).
			d_DMA;
} EMMCDRV;

static	const	uint32_t
		// Command bit enumerations
		SDIO_RNONE    = 0x00000000,
		SDIO_R1       = 0x00000100,
		SDIO_R2       = 0x00000200,
		SDIO_R1b      = 0x00000300,
		SDIO_WRITE    = 0x00000400,
		SDIO_MEM      = 0x00000800,
		SDIO_FIFO     = 0x00001000,
		SDIO_DMA      = 0x00002000,
		SDIO_CMDBUSY  = 0x00004000,
		SDIO_ERR      = 0x00008000,
		SDIO_CMDTMOUT = 0x00000000,
		SDIO_CMDEOKAY = 0x00010000,
		SDIO_CMDCRCER = 0x00020000,
		SDIO_CMDFRMER = 0x00030000,
		SDIO_CMDECODE = 0x00030000,
		// SDIO_REMOVED  = 0x00040000,
		// SDIO_PRESENTN = 0x00080000,
		SDIO_CARDBUSY = 0x00100000,
		SDIO_BUSY     = (SDIO_CARDBUSY|SDIO_CMDBUSY|SDIO_DMA|SDIO_MEM),
		SDIO_CMDERR   = 0x00200000,
		SDIO_RXERR    = 0x00400000,	// RX Error present
		SDIO_RXECODE  = 0x00800000,	// RX Error code
		SDIO_DMAERR   = 0x01000000,
		SDIO_HWRESET  = 0x02000000,
		SDIO_ACK      = 0x04000000,	// Expect a CRC ACK token
		SDIO_RESET    = 0x52000000,
		// PHY enumerations
		SDPHY_DDR      = 0x00004100,	// Requires CK90
		SDPHY_DS       = 0x00004300,	// Requires DDR & CK90
		SDPHY_W1       = 0x00000000,
		SDPHY_W4       = 0x00000400,
		SDPHY_W8	      = 0x00000800,
		SDPHY_WBEST    = 0x00000c00,
		SDPHY_PPDAT    = 0x00001000,	// Push pull drive for data
		SDPHY_PPCMD    = 0x00002000,	// Push pull drive for cmd wire
		SDPHY_PUSHPULL = SDPHY_PPDAT | SDPHY_PPCMD,
		SDIOCK_CK90   = 0x00004000,
		SDIOCK_SHUTDN = 0x00008000,
		SDPHY_PHASEMSK= 0x001f0000,
		// IO clock speeds
		SDIOCK_100KHZ = 0x000000fc,
		SDIOCK_200KHZ = 0x0000007f,
		SDIOCK_400KHZ = 0x00000041,
		SDIOCK_1MHZ   = 0x0000001b,
		SDIOCK_5MHZ   = 0x00000007,
		SDIOCK_12MHZ  = 0x00000004,
		SDIOCK_25MHZ  = 0x00000003,
		SDIOCK_50MHZ  = 0x00000002,
		SDIOCK_100MHZ = 0x00000001,
		SDIOCK_200MHZ = 0x00000000,
		SDIOCK_MASK   = 0x000000ff,
		SDPHY_1P2V    = 0x00400000,
		SDIOCK_DS     = SDIOCK_25MHZ | SDPHY_W4 | SDPHY_PUSHPULL,
		SDIOCK_HS     = SDIOCK_50MHZ | SDPHY_W4 | SDPHY_PUSHPULL,
		// Speed abbreviations
		SDIOCK_SDR50  = SDIOCK_50MHZ  | SDPHY_W4 | SDPHY_PUSHPULL | SDPHY_1P2V,
		SDIOCK_DDR50  = SDIOCK_50MHZ  | SDPHY_W4 | SDPHY_PUSHPULL | SDPHY_DDR | SDPHY_1P2V,
		SDIOCK_SDR104 = SDIOCK_100MHZ | SDPHY_W4 | SDPHY_PUSHPULL | SDPHY_1P2V,
		SDIOCK_SDR200 = SDIOCK_200MHZ | SDPHY_W4 | SDPHY_PUSHPULL | SDPHY_1P2V,
		// SDIOCK_HS400= SDIOCK_200MHZ | SDPHY_W4 | SDPHY_PUSHPULL | SDPHY_DS,
		//
		SPEED_SLOW   = SDIOCK_400KHZ,
		SPEED_DEFAULT= SDIOCK_DS,
		SPEED_FAST   = SDIOCK_HS,
		//
		SECTOR_4B    = 0x02000000,
		SECTOR_16B   = 0x04000000,
		SECTOR_512B  = 0x09000000,
		SECTOR_MASK  = 0x0f000000,
		//
		SDIO_CMD     = 0x00000040,
		SDIO_READREG  = SDIO_CMD | SDIO_R1 | SDIO_ERR,
		SDIO_READREGb = SDIO_CMD | SDIO_R1b,
		SDIO_READR2  = (SDIO_CMD | SDIO_R2),
		SDIO_WRITEBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_ACK | SDIO_WRITE | SDIO_MEM) + 24,
		SDIO_WRMULTI = (SDIO_CMD | SDIO_R1
				| SDIO_ACK | SDIO_WRITE | SDIO_MEM) + 25,
		SDIO_WRDMA = SDIO_WRMULTI | SDIO_DMA,
		SDIO_READBLK  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
					| SDIO_MEM) + 17,
		SDIO_RDMULTI  = (SDIO_CMD | SDIO_R1
					| SDIO_MEM) + 18,
		SDIO_READDMA  = SDIO_RDMULTI | SDIO_DMA,
		SDIO_READCID  = (SDIO_CMD | SDIO_R2 | SDIO_ERR) + 2,
		SDIO_R1ERR    = 0xfff80080;
static	const	uint32_t
		SDIO_RESET_FIFO = 0x080;

static	void	emmc_wait_while_busy(EMMCDRV *dev);
static	void	emmc_go_idle(EMMCDRV *dev);
static	void	emmc_all_send_cid(EMMCDRV *dev);
static	uint32_t emmc_send_rca(EMMCDRV *dev);
static	void	emmc_select_card(EMMCDRV *dev);	// CMD7
static	uint32_t emmc_send_op_cond(EMMCDRV *dev, uint32_t opcond); // ACMD41
static	void	emmc_send_app_cmd(EMMCDRV *dev);  // CMD 55
static	uint32_t emmc_read_ocr(EMMCDRV *dev, uint32_t width);	  // CMD 58
static	void	emmc_dump_cid(EMMCDRV *dev);
static	void	emmc_dump_scr(EMMCDRV *dev);
static	void	emmc_dump_ocr(EMMCDRV *dev);
static	unsigned emmc_get_r1(EMMCDRV *dev);
static	void	emmc_dump_r1(unsigned);
static	int	emmc_write_block(EMMCDRV *dev, uint32_t sector, uint32_t *buf);	  // CMD 24
static	int	emmc_read_block(EMMCDRV *dev, uint32_t sector, uint32_t *buf);	  // CMD 17

extern	EMMCDRV *emmc_init(EMMC *dev);
extern	int	emmc_write(EMMCDRV *dev, const unsigned sector, const unsigned count, const char *buf);
extern	int	emmc_read(EMMCDRV *dev, const unsigned sector, const unsigned count, char *buf);
extern	int	emmc_ioctl(EMMCDRV *dev, char cmd, char *buf);

#define	PANIC
// #define PANIC zip_halt()

void	emmc_wait_while_busy(EMMCDRV *dev) {
	// {{{

	// Could also do a system call and yield to the scheduler while waiting
	// if (EMMC_OS) {
	//	os_wait(dev->d_int);
	// } else if (EMMC_INT) {
	//	// Can wait for an interrupt here, such as by calling an
	//	// external wait_int function with our interrupt ID.
	//	wait_int(dev->d_int);
	// } else {

	// Busy wait implementation
	uint32_t	st;

	st = dev->d_dev->sd_cmd;
	while(st & SDIO_BUSY)
		st = dev->d_dev->sd_cmd;

	// }
}
// }}}

void	emmc_go_idle(EMMCDRV *dev) {			// CMD0
	// {{{
	dev->d_dev->sd_phy = SECTOR_512B | SDIOCK_400KHZ;
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_CMD | SDIO_RNONE | SDIO_ERR;

	emmc_wait_while_busy(dev);

	if (EMMCDEBUG && EMMCINFO) {
		unsigned	c = dev->d_dev->sd_cmd;
		unsigned	r = dev->d_dev->sd_data;

		txstr("CMD0:    SEND_GO_IDLE\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
}
// }}}

void	emmc_all_send_cid(EMMCDRV *dev) {		// CMD2
	// {{{
	if (EMMCDEBUG)	txstr("READ-CID\n");

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READR2)+2;

	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;

	dev->d_CID[0] = dev->d_dev->sd_fifa;
	dev->d_CID[1] = dev->d_dev->sd_fifa;
	dev->d_CID[2] = dev->d_dev->sd_fifa;
	dev->d_CID[3] = dev->d_dev->sd_fifa;

	if (EMMCDEBUG && (c & SDIO_ERR)) {
		TRIGGER_SCOPE;
		txstr("  SD-ERR: "); txhex(c); txstr("\n");
	}

	if (EMMCINFO) {
		unsigned r = dev->d_dev->sd_data;

		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
		emmc_dump_cid(dev);
	}
}
// }}}

void	emmc_send_cid(EMMCDRV *dev) {			// CMD10
	// {{{
	if (EMMCDEBUG)	txstr("SEND-CID\n");

	dev->d_dev->sd_data = (dev->d_RCA << 16);
	dev->d_dev->sd_cmd  = (SDIO_ERR|SDIO_READR2) + 10;

	emmc_wait_while_busy(dev);

	dev->d_CID[0] = dev->d_dev->sd_fifa;
	dev->d_CID[1] = dev->d_dev->sd_fifa;
	dev->d_CID[2] = dev->d_dev->sd_fifa;
	dev->d_CID[3] = dev->d_dev->sd_fifa;

	if (EMMCINFO) {
		unsigned c = dev->d_dev->sd_cmd;
		unsigned r = dev->d_dev->sd_data;

		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
		emmc_dump_cid(dev);
	}
}
// }}}

void	emmc_dump_cid(EMMCDRV *dev) {
	// {{{
	txstr("CID: ");
	txhex(dev->d_CID[0]); txstr(":");
	txhex(dev->d_CID[1]); txstr(":");
	txhex(dev->d_CID[2]); txstr(":");
	txhex(dev->d_CID[3]); txstr("\n");

	txstr("CID:\n" "\tManufacturer ID:  0x"); txhex(dev->d_CID[0]); txstr("\n");
	txstr("\tBank Index #   :  0x"); txhex((dev->d_CID[1]>>2)&0x03f); txstr("\n");
	txstr("\tDevice/BGA     :  0x"); txhex((dev->d_CID[1])&0x03); txstr("\n");
	txstr("\tApplication ID :  0x"); txhex((dev->d_CID[2])&0x0ff); txstr("\n");
	txstr("\tProduct Name   :  ");
	for(int k=3; k<3+6; k++) {
		unsigned	ch = dev->d_CID[k/4];
// txstr(" "); txhex(ch); txstr(" ");
		ch >>= (24-8*(k&3));
		ch &= 0x0ff;
// txstr(" "); txhex(ch); txstr(" ");
		if (isgraph(ch))
			txchr(ch);
		else
			txchr('.');
	} txstr("\n");
	txstr("\tProduct Revision: ");
		txhex((dev->d_CID[9] >>  4)&0x0f); txstr(".");
		txhex((dev->d_CID[9])&0x0f); txstr("\n");
	/*
	txstr(""\tSerial Number:    0x%02x%02x%02x%02x\n",
		(buf[9] >>  4)&0x0f,
		(buf[9])&0x0f,
		(buf[10])&0x0ff,
		(buf[11])&0x0ff,
		(buf[12])&0x0ff,
		(buf[13])&0x0ff);
	*/
}
// }}}

uint32_t emmc_send_rca(EMMCDRV *dev) {				// CMD3
	// {{{
	unsigned	c, r;

	if (EMMCDEBUG)	txstr("SEND-RCA\n");

	dev->d_RCA = 0x1;
	dev->d_dev->sd_data = (dev->d_RCA << 16);
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READREG)+3;

	emmc_wait_while_busy(dev);

	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;

	if (EMMCDEBUG && EMMCINFO) {
		if (!EMMCDEBUG) txstr("CMD3:    SEND-RCA\n");

		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}

	return dev->d_RCA;
}
// }}}

void	emmc_select_card(EMMCDRV *dev) {			// CMD7
	// {{{
	unsigned c, r;

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = SDIO_READREGb + 7;

	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;

	if (EMMCDEBUG && ((c & SDIO_ERR) || SDINFO)) {
		if (c & SDIO_ERR)
			TRIGGER_SCOPE;
		txstr("CMD7:    SELECT_CARD ("); txhex(dev->d_RCA);txstr(")\n");
		if (c & SDIO_ERR) {
			txstr("  SD-ERR: "); txhex(c); txstr("\n");
			sdio_dump_err(c);
		}
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
}
// }}}

uint32_t emmc_send_op_cond(EMMCDRV *dev, uint32_t opcond) {	// CMD1
	// {{{
	unsigned	c, r;

	dev->d_dev->sd_data = opcond;
	dev->d_dev->sd_cmd = SDIO_READREG+1;
	emmc_wait_while_busy(dev);

	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;

	dev->d_OCR = r;
	if (EMMCDEBUG && EMMCINFO) {
		txstr("CMD1:    SEND_OP_COND : "); txhex(opcond); txstr("\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}

	return r;
}
// }}}

void	emmc_send_app_cmd(EMMCDRV *dev) {			// CMD 55
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_READREG+55;

	emmc_wait_while_busy(dev);

	if (EMMCDEBUG && EMMCINFO) {
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD55:   SEND_APP_CMD\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
}
// }}}

void	emmc_dump_ocr(EMMCDRV *dev) {
	// {{{
	if (EMMCINFO) {
		txstr("READ-OCR: OCR = "); txhex(dev->d_OCR); txstr("\n");
		if (0 == (dev->d_OCR & 0x80000000))
			txstr("  Card is still powering up\n");
		if (dev->d_OCR & 0x40000000)
			txstr("  CCS: High capacity support\n");
		if (dev->d_OCR & 0x20000000)
			txstr("  UHS: UHS-II card\n");
		if (dev->d_OCR & 0x01000000)
			txstr("  S18A: Switching to 1.8V allowed\n");
#ifdef	STDIO_DEBUG
		int	mxv = 0, mnv = 0;
		if (dev->d_OCR & 0x00800000) {
			if (mxv == 0||mxv <= 36)
				mxv = 36;
			if (mnv == 0||mnv >= 35)
				mnv = 35;
		} if (dev->d_OCR & 0x00400000) {
			if (mxv == 0||mxv <= 35)
				mxv = 35;
			if (mnv == 0||mnv >= 34)
				mnv = 34;
		} if (dev->d_OCR & 0x00200000) {
			if (mxv == 0||mxv <= 34)
				mxv = 34;
			if (mnv == 0||mnv >= 33)
				mnv = 33;
		} if (dev->d_OCR & 0x00100000) {
			if (mxv == 0||mxv <= 33)
				mxv = 33;
			if (mnv == 0||mnv >= 32)
				mnv = 32;
		} if (dev->d_OCR & 0x00080000) {
			if (mxv == 0||mxv <= 32)
				mxv = 32;
			if (mnv == 0||mnv >= 31)
				mnv = 31;
		} if (dev->d_OCR & 0x00040000) {
			if (mxv == 0||mxv <= 31)
				mxv = 31;
			if (mnv == 0||mnv >= 30)
				mnv = 30;
		} if (dev->d_OCR & 0x00020000) {
			if (mxv == 0||mxv <= 30)
				mxv = 30;
			if (mnv == 0||mnv >= 29)
				mnv = 29;
		} if (dev->d_OCR & 0x00010000) {
			if (mxv == 0||mxv <= 29)
				mxv = 29;
			if (mnv == 0||mnv >= 28)
				mnv = 28;
		} if (dev->d_OCR & 0x00008000) {
			if (mxv == 0||mxv <= 28)
				mxv = 28;
			if (mnv == 0||mnv >= 27)
				mnv = 27;
		} printf("  Voltage ranges supported: %d.%dV - %d.%dV\n",
			(mxv/10), (mxv%10), (mnv/10), (mnv%10));
#else
		if (dev->d_OCR & 0x00800000) txstr("  3.6-3.5 V allowed\n");
		if (dev->d_OCR & 0x00400000) txstr("  3.5-3.4 V allowed\n");
		if (dev->d_OCR & 0x00200000) txstr("  3.4-3.3 V allowed\n");
		if (dev->d_OCR & 0x00100000) txstr("  3.3-3.2 V allowed\n");
		if (dev->d_OCR & 0x00080000) txstr("  3.2-3.1 V allowed\n");
		if (dev->d_OCR & 0x00040000) txstr("  3.1-3.0 V allowed\n");
		if (dev->d_OCR & 0x00020000) txstr("  3.0-2.9 V allowed\n");
		if (dev->d_OCR & 0x00010000) txstr("  2.9-2.8 V allowed\n");
		if (dev->d_OCR & 0x00008000) txstr("  2.8-2.7 V allowed\n");
		if (dev->d_OCR & 0x00000080) txstr("  1.7-1.95 V allowed\n");
#endif
	}
}
// }}}

void emmc_read_csd(EMMCDRV *dev) {	  // CMD 9
	// {{{
	if (EMMCDEBUG)	txstr("READ-CSD\n");

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = (SDIO_CMD | SDIO_R2 | SDIO_ERR)+9;

	emmc_wait_while_busy(dev);

	if (EMMCDEBUG && EMMCINFO) {
		// {{{
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD9:    SEND_CSD\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
		if (c & SDIO_ERR) {
			TRIGGER_SCOPE;
			txstr("  SD-ERR: "); txhex(c); txstr("\n");
			emmc_dump_err(c);
		}
	}
	// }}}

	if (EMMCINFO) txstr("  CSD :    ");
	for(int k=0; k<16; k+= sizeof(uint32_t)) {
		unsigned	uv;

		uv = dev->d_dev->sd_fifa;
		if (EMMCINFO) { txhex(uv); if (k < 12) txstr(":"); }
		dev->d_CSD[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 0] = uv;
	}

	unsigned	CSD_STRUCTURE;

	CSD_STRUCTURE = (dev->d_CSD[0]>>6)&3;

	if (EMMCINFO && EMMCDEBUG) {
		txstr("\n  ");
		for(int k=0; k<16; k++) {
			unsigned	v;

			v = (dev->d_CSD[k] >> 4)&0x0f;
			if (v < 10)
				txchr('0'+v);
			else
				txchr('A'+v-10);

			v = dev->d_CSD[k] & 0x0f;
			if (v < 10)
				txchr('0'+v);
			else
				txchr('A'+v-10);

			if (k<15)
				txstr(":");
		} txstr("\n");
	}

	if (EMMCINFO) {
		txstr("\n");
		txstr("  CSD_STRUCTURE     : "); txhex(CSD_STRUCTURE); txstr("\n");
	}

	unsigned	SPEC_VERS, TAAC, NSAC, TRAN_SPEED, CCC, READ_BL_LEN,
			READ_BL_PARTIAL, WRITE_BLK_MISALIGN, READ_BLK_MISALIGN,
			DSR_IMP, C_SIZE, C_SIZE_MULT, ERASE_GRP_SIZE,
			ERASE_GRP_MULT, WP_GRP_SIZE, WP_GRP_ENABLE,
			DEFAULT_ECC, R2W_FACTOR, WRITE_BL_LEN, WRITE_BL_PARTIAL,
			CONTENT_PROT_APP, FILE_FORMAT_GRP, COPY,
			PERM_WRITE_PROTECT, TMP_WRITE_PROTECT, FILE_FORMAT,
			ECC, CRC;

	SPEC_VERS = (dev->d_CSD[0]>>2)&15;
	TAAC = dev->d_CSD[1] & 0x0ff;
	NSAC = dev->d_CSD[2] & 0x0ff;
	TRAN_SPEED = dev->d_CSD[3] & 0x0ff;
	CCC = ((dev->d_CSD[4] & 0x0ff) << 4) | ((dev->d_CSD[5] >> 4) & 0x0f);
	READ_BL_LEN = (dev->d_CSD[5] & 0x0f);
	READ_BL_PARTIAL = (dev->d_CSD[6] & 0x80) ? 1:0;
	WRITE_BLK_MISALIGN = (dev->d_CSD[6] & 0x40) ? 1:0;
	READ_BLK_MISALIGN  = (dev->d_CSD[6] & 0x20) ? 1:0;
	DSR_IMP  = (dev->d_CSD[6] & 0x10) ? 1:0;
	C_SIZE      = ((dev->d_CSD[6] & 0x0f)<<10)
				| ((dev->d_CSD[7] & 0x0ff)<<2)
				| ((dev->d_CSD[8] >> 6)&3);
	C_SIZE_MULT = ((dev->d_CSD[9] & 0x03)<<1) |((dev->d_CSD[10]>> 7)&1);
	ERASE_GRP_SIZE = (dev->d_CSD[10]>>2) & 0x01f;
	ERASE_GRP_MULT = ((dev->d_CSD[10]&3)<<3)|((dev->d_CSD[11]>>5)&7);
	WP_GRP_SIZE   = (dev->d_CSD[11]& 0x01f);
	WP_GRP_ENABLE = (dev->d_CSD[12]& 0x080) ? 1:0;
	DEFAULT_ECC    = (dev->d_CSD[12] >> 5) & 3;
	R2W_FACTOR     = (dev->d_CSD[12] >> 2) & 7;
	WRITE_BL_LEN   =((dev->d_CSD[12] & 0x03) << 2) |((dev->d_CSD[13]>>6)&3);
	WRITE_BL_PARTIAL= (dev->d_CSD[13] >> 5)&1;
	CONTENT_PROT_APP= dev->d_CSD[13] & 1;
	FILE_FORMAT_GRP = (dev->d_CSD[14] & 0x80) ? 1:0;
	COPY = (dev->d_CSD[14] & 0x40) ? 1:0;
	PERM_WRITE_PROTECT = (dev->d_CSD[14] & 0x20) ? 1:0;
	TMP_WRITE_PROTECT  = (dev->d_CSD[14] & 0x10) ? 1:0;
	FILE_FORMAT        = (dev->d_CSD[14] >> 2) & 3;
	ECC                =  dev->d_CSD[14] & 3;

	if (EMMCINFO) {
		txstr("  TAAC              : "); txhex(TAAC); txstr("\n");
		txstr("  NSAC              : "); txhex(NSAC); txstr("\n");
		txstr("  TRAN_SPEED        : "); txhex(TRAN_SPEED); txstr("\n");
		txstr("  CCC               : "); txhex(CCC); txstr("\n");
		txstr("  READ_BL_LEN       : "); txhex(READ_BL_LEN); txstr("\n");
		txstr("  READ_BL_PARTIAL   : "); txhex(READ_BL_PARTIAL); txstr("\n");
		txstr("  WRITE_BLK_MISALIGN: "); txhex(WRITE_BLK_MISALIGN); txstr("\n");
		txstr("  READ_BLK_MISALIGN : "); txhex(READ_BLK_MISALIGN); txstr("\n");
		txstr("  DSR_IMP           : "); txhex(DSR_IMP); txstr("\n");
		txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\n");
		// txstr("  VDD_R_CURR_MIN    : "); txhex((dev->d_CSD[8]>>3)&0x07); txstr("\n");
		// txstr("  VDD_R_CURR_MAX    : "); txhex(dev->d_CSD[8]&0x07); txstr("\n");
		// txstr("  VDD_W_CURR_MIN    : "); txhex((dev->d_CSD[9]>>5)&0x07); txstr("\n");
		// txstr("  VDD_W_CURR_MAX    : "); txhex((dev->d_CSD[9]>>2)&0x07); txstr("\n");
		txstr("  C_SIZE_MULT       : "); txhex(C_SIZE_MULT); txstr("\n");
		txstr("  ERASE_GRP_SIZE    : "); txhex(ERASE_GRP_SIZE); txstr("\n");
		txstr("  ERASE_GRP_MULT    : "); txhex(ERASE_GRP_MULT); txstr("\n");
		txstr("  WP_GRP_SIZE       : "); txhex(WP_GRP_SIZE); txstr("\n");
		txstr("  WP_GRP_ENABLE     : "); txhex(WP_GRP_ENABLE); txstr("\n");
		txstr("  R2W_FACTOR        : "); txhex(R2W_FACTOR); txstr("\n");
		txstr("  WRITE_BL_LEN      : "); txhex(WRITE_BL_LEN); txstr("\n");
		txstr("  WRITE_BL_PARTIAL  : "); txhex(WRITE_BL_PARTIAL); txstr("\n");
		txstr("  FILE_FORMAT_GRP   : "); txhex(FILE_FORMAT_GRP); txstr("\n");
		txstr("  COPY              : "); txhex(COPY); txstr("\n");
		txstr("  PERM_WRITE_PROTECT: "); txhex(PERM_WRITE_PROTECT); txstr("\n");
		txstr("  TMP_WRITE_PROTECT : "); txhex(TMP_WRITE_PROTECT); txstr("\n");
		txstr("  FILE_FORMAT       : "); txhex(FILE_FORMAT); txstr("\n");
	/*
		if (0 == FILE_FORMAT_GRP) {
			if (0 == FILE_FORMAT)
				txstr("  (Has parition tbl)");
			else if (1 == FILE_FORMAT)
				txstr("  (DOS FAT w/ boot sector, no partition)");
			else if (2 == FILE_FORMAT)
				txstr("  (Universal file format)");
			else
				txstr("  (Others/unknown)");
		} else
			txstr("  (Reserved)");
	*/
		// txstr("  Size  = "); txdecimal(BLOCKNR); txstr(" blocks of ");
		// txdecimal(BLOCK_LEN); txstr(" bytes each");
	}
}
// }}}

static	void	emmc_decode_cmd(unsigned cmd) {
	// {{{
	txstr("  Cmd:     "); txhex(cmd); txstr("\n");
	if ((cmd & 0xc0)==0x40) {
		txstr("   ");
		txhex(cmd & 0x0ff);
		txstr(": CMD");
		txdecimal(cmd & 0x3f);
		txstr("\n");
	} else if ((cmd & 0xc0)==0x00) {
		txstr("   ");
		txhex(cmd & 0x0ff);
		txstr(": REPLY ");
		txdecimal(cmd & 0x3f);
		txstr("\n");
	} else {
		txstr("   ");
		txhex(cmd & 0x0ff); txstr(": ILLEGAL\n");
	}

	if (SDIO_RNONE== (cmd & SDIO_R1b))
		txstr("   No reply expected\n");
	else if (SDIO_R1 == (cmd & SDIO_R1))
		txstr("   R1  reply expected\n");
	else if (SDIO_R2 == (cmd & SDIO_R1))
		txstr("   R2  reply expected\n");
	else //
		txstr("   R1b reply expected\n");

	if (cmd & SDIO_MEM) {
		if (cmd & SDIO_WRITE)
			txstr("   MEM Write (TX) command\n");
		else
			txstr("   MEM Read  (RX) command\n");
	} if ((cmd & SDIO_MEM) || ((cmd & SDIO_R1b) == SDIO_R2)) {
		if (cmd & SDIO_FIFO)
			txstr("   FIFO B\n");
		else
			txstr("   FIFO A\n");
	} if (cmd & SDIO_BUSY) {
		txstr("   Busy\n");
	} if (cmd & SDIO_CMDBUSY) {
		txstr("   CMDBusy\n");
	} if (cmd & SDIO_ERR) {
		if (cmd & SDIO_CMDERR) {
			txstr("   CMD Err: ");
			switch((cmd >> 16)&3) {
			case 0: txstr("Timeout\n"); break;
			case 1: txstr("Okay\n"); break;
			case 2: txstr("Bad CRC\n"); break;
			case 3: txstr("Frame Err\n"); break;
			}
		} if (cmd & SDIO_RXERR) {
			txstr("   RX Err:  ");
			if (cmd & SDIO_RXCRCERR) {
				txstr(" CRC Err\n");
			} else
				txstr(" Watchdog Err\n");
		}
	} else
		txstr("   No ERR\n");
}
// }}}

void emmc_send_ext_csd(EMMCDRV *dev) {	  // CMD 8
	// {{{
	if (EMMCDEBUG)	txstr("SEND-EXT-CSD\n");

	dev->d_dev->sd_phy = (dev->d_dev->sd_phy & (0x0ffffff)) | SECTOR_512B;
	dev->d_dev->sd_data = 0;	// Stuff bits
	dev->d_dev->sd_cmd = (SDIO_CMD | SDIO_R1 | SDIO_ERR | SDIO_MEM)+8;
	emmc_decode_cmd(dev->d_dev->sd_cmd);

	emmc_wait_while_busy(dev);

	if (EMMCDEBUG && EMMCINFO) {
		// {{{
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD8:    SEND_EXT_CSD\n");
		// txstr("  Cmd:     "); txhex(c); txstr("\n");
		emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	}
	// }}}

	// STOP transmission
	// {{{
	if (0) {
		// Not required
		dev->d_dev->sd_data = dev->d_RCA << 16;
		dev->d_dev->sd_cmd = SDIO_READREG+12;
		emmc_wait_while_busy(dev);

		if (EMMCDEBUG && EMMCINFO) {
			// {{{
			unsigned	c, r;

			c = dev->d_dev->sd_cmd;
			r = dev->d_dev->sd_data;

			txstr("CMD12:   STOP_TRANSMISSION\n");
			txstr("  Cmd:     "); txhex(c); txstr("\n");
			txstr("  Data:    "); txhex(r); txstr("\n");
		}
		// }}}
	}
	// }}}

	// Read data into dev->d_EXCSD[]
	// {{{
	for(int k=0; k<512; k+= sizeof(uint32_t)) {
		unsigned	uv;

		uv = dev->d_dev->sd_fifa;
		// if (EMMCINFO) { txhex(uv); if (k < 12) txstr(":"); }
		dev->d_EXCSD[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_EXCSD[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_EXCSD[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_EXCSD[k + 0] = uv;
	}
	// }}}

	if (EMMCINFO && EMMCDEBUG) {
		// {{{
		txstr("\n  ");
		for(int k=0; k<512; k++) {
			unsigned	v;

			v = (dev->d_EXCSD[k] >> 4)&0x0f;
			if (v < 10)
				txchr('0'+v);
			else
				txchr('A'+v-10);

			v = dev->d_EXCSD[k] & 0x0f;
			if (v < 10)
				txchr('0'+v);
			else
				txchr('A'+v-10);

			if (k+1 >= 512) {
			} else if (15 == (k&15)) {
				txstr("\n  ");
			} else if (7 == (k&7))
				txstr(" ");
			else
				txstr(":");
		} txstr("\n");
	}
	// }}}

	for(int k=0; k<512; k++) {
		if (isgraph(dev->d_EXCSD[k]))
			txchr(dev->d_EXCSD[k]);
		else
			txchr('.');
	} txchr('\n');

	unsigned	SEC_COUNT;

	dev->d_sector_count = dev->d_EXCSD[212]
			| (dev->d_EXCSD[213] << 8)
			| (dev->d_EXCSD[214] << 16)
			| (dev->d_EXCSD[215] << 24);

	if (EMMCINFO) {
		txstr("  S_CMD_SET     : "); txhex(dev->d_EXCSD[504]); txstr("\n");
		txstr("  BKOPS_SUPPORT : "); txhex(dev->d_EXCSD[502]); txstr("\n");
		txstr("  MAX_PKD_READS : "); txhex(dev->d_EXCSD[501]); txstr("\n");
		txstr("  MAX_PKD_WRITES: "); txhex(dev->d_EXCSD[500]); txstr("\n");
		txstr("  DATA_TAG_SPRT : "); txhex(dev->d_EXCSD[499]); txstr("\n");
		txstr("  EXT_SUPPORT   : "); txhex(dev->d_EXCSD[494]); txstr("\n");
		txstr("  SUPPORTED_MODS: "); txhex(dev->d_EXCSD[493]); txstr("\n");
		txstr("  CMDQ_SUPPORT  : "); txhex(dev->d_EXCSD[308]); txstr("\n");
		txstr("  SECTOR_COUNT  : "); txhex(dev->d_sector_count); txstr("\n");
		uint32_t cache_size = dev->d_EXCSD[ 249] | (dev->d_EXCSD[ 250] << 8) |
						(dev->d_EXCSD[ 251] << 16) | (dev->d_EXCSD[ 252] << 24);
		txstr("  CACHE_SIZE    : "); txhex(cache_size); txstr("\n");
		txstr("  DEVICE_TYPE   : "); txhex(dev->d_EXCSD[196]); txstr("\n");
		if (dev->d_EXCSD[196] & 0x80)
			txstr("    HS400 DDR @ 200MHz, 1.2V\n");
		if (dev->d_EXCSD[196] & 0x40)
			txstr("    HS400 DDR @ 200MHz, 1.8V\n");
		if (dev->d_EXCSD[196] & 0x20)
			txstr("    HS200 SDR @ 200MHz, 1.2V\n");
		if (dev->d_EXCSD[196] & 0x10)
			txstr("    HS200 SDR @ 200MHz, 1.8V\n");
		if (dev->d_EXCSD[196] & 0x08)
			txstr("    HS    DDR @  52MHz, 1.2V\n");
		if (dev->d_EXCSD[196] & 0x04)
			txstr("    HS    DDR @  52MHz, 1.8 or 3V\n");
		if (dev->d_EXCSD[196] & 0x02)
			txstr("    HS        @  52MHz\n");
		if (dev->d_EXCSD[196] & 0x01)
			txstr("    HS        @  26MHz\n");
		txstr("  CSD_STRUCTURE : "); txhex(dev->d_EXCSD[194]); txstr("\n");
		txstr("  EXT_CSD_REV   : "); txhex(dev->d_EXCSD[192]); txstr("\n");
		txstr("  CMD_SET       : "); txhex(dev->d_EXCSD[191]); txstr("\n");
		txstr("  POWER_CLASS   : "); txhex(dev->d_EXCSD[187]); txstr("\n");
		txstr("  HS_TIMING     : "); txhex(dev->d_EXCSD[185]); txstr("\n");
		txstr("  STROBE_SUPPORT: "); txhex(dev->d_EXCSD[184]); txstr("\n");
		txstr("  BUS_WIDTH     : "); txhex(dev->d_EXCSD[183]); txstr("\n");
		txstr("  DATA_SECTOR_SZ: "); txhex(dev->d_EXCSD[ 61]); txstr("\n");
		txstr("  CACHE         : "); txhex(dev->d_EXCSD[ 33]); txstr("\n");

	}
}
// }}}

// Dump R1
void emmc_dump_r1(const unsigned rv) {
	// {{{
	if (EMMCINFO) {
		txstr("EMMC R1 Decode:  "); txhex(rv);
		if (rv & 0x80000000)
			txstr("\n  OUT_OF_RANGE");
		if (rv & 0x40000000)
			txstr("\n  ADDRESS_MISALIGN");
		if (rv & 0x20000000)
			txstr("\n  BLOCK_LEN_ERROR");
		if (rv & 0x10000000)
			txstr("\n  ERASE_SEQ_ERROR");
		if (rv & 0x08000000)
			txstr("\n  ERASE_PARAM");
		if (rv & 0x04000000)
			txstr("\n  WP_VIOLATION");
		if (rv & 0x02000000)
			txstr("\n  DEVICE_IS_LOCKED");
		if (rv & 0x01000000)
			txstr("\n  LOCK_UNLOCK_FAILED");
		if (rv & 0x00800000)
			txstr("\n  COM_CRC_ERROR");
		if (rv & 0x00400000)
			txstr("\n  ILLEGAL_COMMAND");
		if (rv & 0x00200000)
			txstr("\n  DEVICE_ECC_FAILED");
		if (rv & 0x00100000)
			txstr("\n  CC_ERROR (Internal card controller err)");
		if (rv & 0x00080000)
			txstr("\n  ERROR (General or unknown error)");
		if (rv & 0x00010000)
			txstr("\n  CSD_OVERWRITE");
		if (rv & 0x00008000)
			txstr("\n  WP_ERASE_SKIP");
		if (rv & 0x00002000)
			txstr("\n  ERASE_RESET");
		switch((rv >> 9)&0x0f) {
		case 0: txstr("\n  STATE: idle");  break;
		case 1: txstr("\n  STATE: ready"); break;
		case 2: txstr("\n  STATE: ident"); break;
		case 3: txstr("\n  STATE: stdby"); break;
		case 4: txstr("\n  STATE: tran");  break;
		case 5: txstr("\n  STATE: data");  break;
		case 6: txstr("\n  STATE: rcv");   break;
		case 7: txstr("\n  STATE: prg");   break;
		case 8: txstr("\n  STATE: dis");   break;
		case 9: txstr("\n  STATE: btst");   break;
		case 10: txstr("\n  STATE: slp");   break;
		default: txstr("\n  STATE: (reserved)"); break;
		}
		if (rv & 0x00000100)
			txstr("\n  READY_FOR_DATA");
		if (rv & 0x00000080)
			txstr("\n  SWITCH_ERROR");
		if (rv & 0x00000040)
			txstr("\n  EXCEPTION_EVENT");
		if (rv & 0x00000020)
			txstr("\n  APP_CMD");
		txstr("\n");
	}
}
// }}}

unsigned emmc_get_r1(EMMCDRV *dev) {	// CMD13=send_status
	// {{{
	unsigned	vc, vd;

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd  = (SDIO_ERR|SDIO_READREG) + 13;
	if (EMMCDEBUG)
		txstr("CMD13:   SEND_STATUS\n");
	emmc_wait_while_busy(dev);

	vc = dev->d_dev->sd_cmd;
	vd = dev->d_dev->sd_data;
	if (EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(vc); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(vd); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");

		if (EMMCINFO && (0 == (vc & 0x8000)))
			emmc_dump_r1(vd);
	}
	return	vd;
}
// }}}

static void emmc_dump_sector(const unsigned *ubuf) {
	// {{{
	for(int j=0; j<512/4; j++) {
		txstr("0x"); txhex(ubuf[j]);
		if (3 == (j&3))
			txstr("\n");
		else
			txstr(" ");
	} txstr("\n");
}
// }}}

int	emmc_write_block(EMMCDRV *dev, uint32_t sector, uint32_t *buf){// CMD 24
	// {{{
	unsigned	dev_stat, card_stat, err = 0;

	if (EMMCDEBUG) {
		txstr("EMMC-WRITE(BLK): ");
		txhex(sector);
		txstr("\n");
	}


	GRAB_MUTEX;

	{
		uint32_t	phy = dev->d_dev->sd_phy;

		if (SECTOR_512B != (phy & SECTOR_MASK)) {
			phy &= ~SECTOR_MASK;
			phy |= SECTOR_512B;
			dev->d_dev->sd_phy = phy;
		}
	}

	// Load up the FIFO
#ifdef	INCLUDE_DMA_CONTROLLER
	if (SDEXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
		_zip->z_dma.d_len = 512;
		_zip->z_dma.d_rd  = (char *)buf;
		_zip->z_dma.d_wr  = (char *)&dev->d_dev->sd_fifa;
		_zip->z_dma.d_ctrl= DMAREQUEST|DMACLEAR|DMA_SRCWIDE
				|DMA_CONSTDST|DMA_DSTWORD;
		while(_zip->z_dma.d_ctrl & DMA_BUSY)
			;
	} else
#endif
		for(int k=0; k<512/sizeof(uint32_t); k++)
			dev->d_dev->sd_fifa = buf[k];

	// Set up the device address
	// {{{
	if (dev->d_OCR & 0x40000000)
		// High capacity card
		dev->d_dev->sd_data = sector;
	else
		dev->d_dev->sd_data = sector*512;
	// }}}

	// Send the write command
	dev->d_dev->sd_cmd = SDIO_WRITEBLK;

	// ... and wait while busy
	emmc_wait_while_busy(dev);

	dev_stat  = dev->d_dev->sd_cmd;
	card_stat = dev->d_dev->sd_data;

	RELEASE_MUTEX;

	// Return an error code if necessary
	// {{{
	if (dev_stat & (SDIO_ERR|SDIO_RXERR)) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;

		err = 1;
		if (EMMCDEBUG) {
			txstr("SDIO-WRITE -> ERR: \n");
			txhex(dev_stat); txstr("\n");
			if (SDINFO)
				sdio_dump_err(dev_stat);
		}
	} else if (card_stat & SDIO_R1ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;

		if (EMMCDEBUG) {
			if (card_stat & SDIO_ERR) {
				txstr("EMMC ERR: EMMC write response = ");
				txhex(card_stat);
				txstr("\n");
			}

			if (EMMCINFO)
				emmc_dump_r1(card_stat);
		} dev->d_dev->sd_cmd = SDIO_ERR;

		err = 1;
	}
	// }}}

	if (err) {
		if (EMMCDEBUG)
			txstr("EMMC-READ -> ERR\n");
		return RES_ERROR;
	} return RES_OK;
}
// }}}

int	emmc_read_block(EMMCDRV *dev, uint32_t sector, uint32_t *buf){// CMD 17
	// {{{
	int	err = 0, dev_stat, card_stat;

	if (EMMCDEBUG) {
		txstr("EMMC-READ(BLK): ");
		txhex(sector);
		txstr("\n");
	}

	// No removal check.  EMMC Chip can't be removed.

	GRAB_MUTEX;

	// Check the PHY configuration
	// {{{
	{
		uint32_t	phy = dev->d_dev->sd_phy;

		if (SECTOR_512B != (phy & SECTOR_MASK)) {
			phy &= ~SECTOR_MASK;
			phy |= SECTOR_512B;
			dev->d_dev->sd_phy = phy;
		}
	}
	// }}}

	dev->d_dev->sd_data = sector;
	dev->d_dev->sd_cmd = SDIO_READBLK;

	emmc_wait_while_busy(dev);

	// Check for errors
	dev_stat  = dev->d_dev->sd_cmd;
	card_stat = dev->d_dev->sd_data;

#ifdef	INCLUDE_DMA_CONTROLLER
	if (SDEXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
		_zip->z_dma.d_len = 512;
		_zip->z_dma.d_rd  = (char *)&dev->d_dev->sd_fifa;
		_zip->z_dma.d_wr  = (char *)buf;
		_zip->z_dma.d_ctrl= DMAREQUEST|DMACLEAR|DMA_DSTWIDE
					| DMA_CONSTSRC|DMA_SRCWORD;
		while(_zip->z_dma.d_ctrl & DMA_BUSY)
			;
		CLEAR_DCACHE;
	} else
#endif
		for(int k=0; k<512/sizeof(uint32_t); k++)
			buf[k] = dev->d_dev->sd_fifa;

	if (EMMCDEBUG && EMMCINFO)
		emmc_dump_sector(buf);

	RELEASE_MUTEX;

	// Error handling
	// {{{
	if (dev_stat & SDIO_ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		err = 1;
		if (EMMCDEBUG) {
			txstr("\tEMMC-READ -> ERR: ");
			txhex(dev_stat);
			txstr(":");
			txhex(card_stat);
			txstr("\n");
			if (EMMCINFO)
				sdio_dump_err(dev_stat);
		}
	} else if (card_stat & SDIO_R1ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		// If the card declares an error, return an error status
		if (EMMCDEBUG) {
			txstr("\tR1-ERR\n");
			if (EMMCINFO)
				emmc_dump_r1(card_stat);
		}
		err = 1;
	}
	// }}}

	return 0;
}
// }}}

void	emmc_best_width(EMMCDRV *dev) {
	// {{{
	unsigned	const	SWITCH_WRITE_BYTE = (3 << 24),
				WIDTH_INDEX = 183;
	unsigned	v, c, r;

	// Section 6.6.4
		// SDPHY_W1       = 0x00000000,
		// SDPHY_W4       = 0x00000400,
		// SDPHY_W8	      = 0x00000800,
		// SDPHY_WBEST    = 0x00000c00,
	
	if (EMMCDEBUG) txstr("Testing 1b width\n");
	// {{{
	dev->d_dev->sd_phy = (dev->d_dev->sd_phy & ~(SECTOR_MASK | SDPHY_WBEST))
				| SDPHY_W1 | SECTOR_4B;
	dev->d_dev->sd_data = SWITCH_WRITE_BYTE
				| (WIDTH_INDEX << 16)
				| (0 << 8);
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b | SDIO_ERR) + 6;
	emmc_wait_while_busy(dev);
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD6:    SWITCH\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_ERR) {
		txstr("EMMC PANIC!  Err response to switch-cmd\n");
		PANIC;
	} if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1 ERR response to SWITCH (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	dev->d_dev->sd_fifa = 0x80000000;
	dev->d_dev->sd_fifb = 0;
	// CMD19 = BUSTEST_W
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_WRITE | SDIO_MEM) + 19;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD19:   BUSTEST_W\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_ERR) {
		// Rx/CRC errors are expected
		txstr("EMMC PANIC!  Err response to bus test write\n");
	} if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1ERR response to CMD19 (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	// CMD14 = BUSTEST_R, FIFO B
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_MEM | SDIO_FIFO) + 14;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD14:   BUSTEST_R\n");
	emmc_wait_while_busy(dev);

	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	v = dev->d_dev->sd_fifb;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
		txstr(" FIFB:     "); txhex(v); txstr("\n");
	} if (c & SDIO_CMDERR) {
		txstr("EMMC PANIC!  CMDErr response to bus test read\n");
		PANIC;
	} if (r & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1ERR response to CMD14 (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	// Now check the result
	if ((v & 0xc0000000) != 0x40000000) {
		txstr("EMMC PANIC!  Failed single-bit bus test (");
		txhex(v); txstr(").  No fallback available.\n");
		PANIC;
	}
	// }}}
	if (EMMCDEBUG) txstr("Testing 4b width\n");
	// {{{
	dev->d_dev->sd_phy = (dev->d_dev->sd_phy & ~(SECTOR_MASK | SDPHY_WBEST))
			| SDPHY_W4 | SECTOR_4B;
	dev->d_dev->sd_data = SWITCH_WRITE_BYTE
				| (WIDTH_INDEX << 16)
				| (1 << 8);
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b | SDIO_ERR) + 6;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD6:    SWITCH\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_ERR) {
		txstr("EMMC PANIC!  Err response to switch-cmd\n");
		PANIC;
	} if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1 ERR response to SWITCH (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	dev->d_dev->sd_fifa = 0xa5000000;
	dev->d_dev->sd_fifb = 0;
	// CMD19 = BUSTEST_W
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_WRITE | SDIO_MEM) + 19;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD19:   BUSTEST_W\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_ERR) {
		// Rx/CRC errors are expected
		txstr("EMMC PANIC!  Err response to bus test write\n");
	} if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1ERR response to CMD19 (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	// CMD14 = BUSTEST_R, FIFO B
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1 | SDIO_ERR | SDIO_MEM
				| SDIO_FIFO) + 14;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD14:   BUSTEST_R\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	v = dev->d_dev->sd_fifb;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
		txstr(" FIFB:     "); txhex(v); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_CMDERR) {
		txstr("EMMC PANIC!  CMDErr response to bus test read\n");
		PANIC;
	} if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1ERR response to CMD14 (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	// Now check the result -- ignoring all but the first two bits
	if ((v & 0xff000000) != 0x5a000000) {
		txstr("  4b fail! ("); txhex(v);
		txstr(") -- falling back to 1b\n");
		dev->d_dev->sd_phy = (dev->d_dev->sd_phy
						& ~(SECTOR_MASK | SDPHY_WBEST))
				| SDPHY_W1 | SECTOR_512B;
		dev->d_dev->sd_data = SWITCH_WRITE_BYTE
					| (WIDTH_INDEX << 16)
					| (0 << 8);
		dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b | SDIO_ERR) + 6;
		emmc_wait_while_busy(dev);
		if (dev->d_dev->sd_cmd & SDIO_ERR) {
			txstr("EMMC PANIC!  Err response to switch-cmd\n");
			PANIC;
		} if (dev->d_dev->sd_data & SDIO_R1ERR) {
			txstr("EMMC PANIC!  R1 ERR response to SWITCH (");
			txhex(dev->d_dev->sd_data); txstr(")\n");
			PANIC;
		}

		return;
	} else if (EMMCINFO) {
		txstr("  Success!\n");
	}
	// }}}
	if (EMMCDEBUG) txstr("Testing 8b width\n");
	// {{{
	dev->d_dev->sd_phy = (dev->d_dev->sd_phy & ~(SECTOR_MASK | SDPHY_WBEST))
				| SDPHY_W8 | SECTOR_4B;
	dev->d_dev->sd_data = SWITCH_WRITE_BYTE
				| (WIDTH_INDEX << 16)
				| (2 << 8);
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b | SDIO_ERR) + 6;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD6:    SWITCH\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_ERR) {
		txstr("EMMC PANIC!  Err response to switch-cmd\n");
		PANIC;
	} if (r & SDIO_R1ERR) {
		r = emmc_get_r1(dev);
		if (r & SDIO_R1ERR) {
			txstr("EMMC PANIC!  R1 ERR response to SWITCH (");
			txhex(r); txstr(")\n");
			PANIC;
		}
	}

	dev->d_dev->sd_fifa = 0xa55a0000;
	dev->d_dev->sd_fifb = 0;
	// CMD19 = BUSTEST_W
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_WRITE | SDIO_MEM) + 19;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD19:   BUSTEST_W\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_ERR) {
		// Rx/CRC errors are expected
		txstr("EMMC PANIC!  Err response to bus test write\n");
	}
	/*
	if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1ERR response to CMD19 (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}
	*/

	// CMD14 = BUSTEST_R, FIFO B
	dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1 | SDIO_ERR | SDIO_MEM
				| SDIO_FIFO) + 14;
	if (EMMCINFO && EMMCDEBUG)
		txstr("CMD14:   BUSTEST_R\n");
	emmc_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;
	v = dev->d_dev->sd_fifb;
	if (EMMCINFO && EMMCDEBUG) {
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		// emmc_decode_cmd(c);
		txstr("  Data:    "); txhex(r); txstr("\n");
		txstr("  PHY:     "); txhex(dev->d_dev->sd_phy); txstr("\n");
		txstr(" FIFB:     "); txhex(v); txstr("\n");
	} if (dev->d_dev->sd_cmd & SDIO_CMDERR) {
		txstr("EMMC PANIC!  CMDErr response to bus test read\n");
		PANIC;
	} if (dev->d_dev->sd_data & SDIO_R1ERR) {
		txstr("EMMC PANIC!  R1ERR response to CMD14 (");
		txhex(dev->d_dev->sd_data); txstr(")\n");
		PANIC;
	}

	// Now check the result -- ignoring all but the first two bits
	if ((v & 0xffff0000) != 0x5aa50000) {
		txstr("  8b fail! ("); txhex(v);
		txstr(") -- falling back to 4b\n");
		dev->d_dev->sd_phy = (dev->d_dev->sd_phy
						& ~(SECTOR_MASK | SDPHY_WBEST))
				| SDPHY_W4 | SECTOR_512B;
		dev->d_dev->sd_data = SWITCH_WRITE_BYTE
					| (WIDTH_INDEX << 16)
					| (1 << 8);
		dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b | SDIO_ERR) + 6;
		emmc_wait_while_busy(dev);
		if (dev->d_dev->sd_cmd & SDIO_ERR) {
			txstr("EMMC PANIC!  Err response to switch-cmd\n");
			PANIC;
		} if (dev->d_dev->sd_data & SDIO_R1ERR) {
			txstr("EMMC PANIC!  R1 ERR response to SWITCH (");
			txhex(dev->d_dev->sd_data); txstr(")\n");
			PANIC;
		}

		return;
	} else if (EMMCINFO) {
		txstr("  Success!\n");
	}
	// }}}
}
// }}}

EMMCDRV *emmc_init(EMMC *dev) {
	// {{{
	unsigned	ifcond, op_cond;
	EMMCDRV	*dv = (EMMCDRV *)malloc(sizeof(EMMCDRV));

	if (NULL == dv) {
		txstr("PANIC!  No memory for driver\n");
		PANIC;
	}

	dv->d_dev = dev;
	dv->d_OCR = 0;
	dv->d_RCA = 0;
	dv->d_sector_count = 0;
	dv->d_block_size   = 0;

	// Determine if the DMA is present, and mark it for use if so
	// {{{
	dv->d_dev->sd_dma_length = -1;
	dv->d_DMA = 0;
	if (dv->d_dev->sd_dma_length != 0)
		dv->d_DMA = 1;
	// }}}

	// NEW_MUTEX;
	GRAB_MUTEX;

	// Start by resetting the interface--in case we're being called
	// to restart from an uncertain state.
	dv->d_dev->sd_cmd = SDIO_RESET;

	dv->d_dev->sd_phy = SDIOCK_400KHZ | SECTOR_512B;
	while(SDIOCK_400KHZ != (dv->d_dev->sd_phy & SDIOCK_MASK))
		;

	emmc_go_idle(dv);

	// Query OP_COND
	// supported_op_cond = emmc_send_op_cond(dv,0);

	do {
		op_cond = emmc_send_op_cond(dv,0x40ff8080);
	} while(0 == (op_cond & 0x80000000));

	if (EMMCINFO || ((op_cond & 0x3ffffff) != 0x0ff8080)) {
		emmc_dump_ocr(dv);
	}

	emmc_all_send_cid(dv);		// CMD2
	emmc_send_rca(dv);

	emmc_send_cid(dv);		// CMD10, Read CID (again)
	emmc_read_csd(dv);		// CMD9, Read card specific data

	{
		unsigned phy = dv->d_dev->sd_phy;

		// We can move up to 25MHz with no further hassles.
		// Going faster requires coordination with the eMMC chip
		phy &= ~SDIOCK_MASK;
		phy |= SDIOCK_25MHZ | SDPHY_PUSHPULL;
		dv->d_dev->sd_phy = phy;
	}

	// CMD4 (SET_DSR), CMD9 (SEND_CSD), CMD10 (SEND_CID), CMD39(FAST_IO) ?
	// CMD40 (GO_IRQ_STATE)
	// CMD5 SLEEP_AWAKE ... when to issue this?
	// CMD6 SWITCH command set ... when to issue this?

	emmc_select_card(dv);		// Move to transfer state
	emmc_send_ext_csd(dv);		// CMD8, Read extended CSD info

	// 6.6.4 Bus testing procedure ... via CMD19(BUSTEST_W)
	//		and CMD14(BUSTEST_R)
	emmc_best_width(dv);

	RELEASE_MUTEX;

	// dv->d_sector_count = 0;
	dv->d_block_size   = 512;

	if (EMMCINFO && EMMCDEBUG) {
		txstr("EMMC SUMMARY:  ");
		txstr("Block size:   ");
		txdecimal(dv->d_block_size);
		txstr("\nSector count: "); txdecimal(dv->d_sector_count);
		txstr("\n");
	}

	return	dv;
}
// }}}

int	emmc_write(EMMCDRV *dev, const unsigned sector,
			const unsigned count, const char *buf) {
	// {{{
	unsigned	card_stat, dev_stat, err = 0;

	if (0 == count)
		return	RES_OK;

	if (!EMMCMULTI) {
		for(unsigned k=0; k<count; k++) {
			unsigned	st;

			st = emmc_write_block(dev, sector+k,
					(uint32_t *)&buf[k*512]);
			if (0 != st)
				return RES_ERROR;
		} return RES_OK;
	}

	if (EMMCDEBUG) {
		// {{{
		txstr("EMMC-WRITE(MNY): ");
		txhex(sector);
		txstr(", ");
		txhex(count);
		txstr(", ");
		txhex(buf);
		txstr("-- [DEV ");
		txhex(dev->d_dev->sd_cmd);
		txstr("]\n");
	}
	// }}}


	GRAB_MUTEX;

	// Make sure our device is idle
	if(dev->d_dev->sd_cmd & SDIO_BUSY)
		emmc_wait_while_busy(dev);

	// Make sure we're configured for 512B sectors
	// {{{
	{
		unsigned phy = dev->d_dev->sd_phy;

		if (SECTOR_512B != (phy & SECTOR_MASK)) {
			// Here, we don't care if the clock is shut down as
			// well, since we control the write speed.
			phy &= ~SECTOR_MASK;
			phy |= SECTOR_512B;
			dev->d_dev->sd_phy = phy;
		}
	}
	// }}}

	if (dev->d_OCR & 0x40000000)
		// High capacity card
		dev->d_dev->sd_data = sector;
	else
		dev->d_dev->sd_data = sector*512;

	// Check for the DMA's existence
	dev->d_dev->sd_dma_length = count;
	if (count == dev->d_dev->sd_dma_length) { // DMA is present
		// {{{

		// Set up the DMA
		// {{{
		dev->d_dev->sd_dma_addr = (char *)buf;
		// Always transfer in units of 512 bytes
		// dev->d_dev->sd_dma_length = count;
		//	Already set above, in order to determine if the DMA
		//	was present
		// }}}

		// Command the transfer, setting SDIO_DMA to use the DMA
		// {{{
		dev->d_dev->sd_cmd  = SDIO_WRMULTI | SDIO_DMA;
		// }}}

		// The DMA will end the transfer with a STOP_TRANSMISSION
		// command *IF* count > 1.  The R1 value from the STOP_TRANS
		// command will still be in the data register after the
		// entire command completes.  We can read it here to get the
		// status from the transmission.
		// }}}
	} else {
		for(unsigned s=0; s<count; s++) { // Foreach sector
			// Load the data into alternating buffers
			// {{{
#ifdef	INCLUDE_DMA_CONTROLLER
			if (EXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
				_zip->z_dma.d_len = 512;
				_zip->z_dma.d_rd  = (char *)&buf[s*512];
				_zip->z_dma.d_wr  = (s&1)
					? (char *)&dev->d_dev->sd_fifb
					: (char *)&dev->d_dev->sd_fifa;
				_zip->z_dma.d_ctrl= DMAREQUEST|DMACLEAR|DMA_SRCWIDE
						| DMA_CONSTDST|DMA_DSTWORD;
				while(_zip->z_dma.d_ctrl & DMA_BUSY)
					;
			} else
#endif
			{
				unsigned *src;
				src = (unsigned *)&buf[s*512];

				if (s&1) {
					for(int w=0; w<512/sizeof(uint32_t); w++)
						dev->d_dev->sd_fifb = src[w];
				} else {
					for(int w=0; w<512/sizeof(uint32_t); w++)
						dev->d_dev->sd_fifa = src[w];
				}
			}
			// }}}

			if (s == 0) { // Issue WRITE_MULTIPLE_BLOCK cmd
				// {{{

				// Issue a write-multiple command
				dev->d_dev->sd_cmd  = SDIO_ERR | SDIO_WRMULTI;

				emmc_wait_while_busy(dev);

				// Check for errors
				dev_stat  = dev->d_dev->sd_cmd;
				card_stat = dev->d_dev->sd_data;
				if (dev_stat & SDIO_ERR)
					break;
				if (card_stat & SDIO_R1ERR)
					break;

				// Don't wait.  Go around again and load the
				// next block of data before checking if the
				// write has completed.
				// }}}
			} else { // Send the next block
				// {{{
				// Wait for the last write to complete
				emmc_wait_while_busy(dev);

				// Then send another block of data
				dev->d_dev->sd_cmd = (SDIO_WRITE | SDIO_MEM)
					| ((s&1) ? SDIO_FIFO : 0);
				// }}}
			}
		}

		// Wait for the final write to complete
		emmc_wait_while_busy(dev);

		// Send a (final) STOP_TRANSMISSION request
		dev->d_dev->sd_data = 0;
		dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b | SDIO_ERR) + 12;
	}

	emmc_wait_while_busy(dev);

	dev_stat  = dev->d_dev->sd_cmd;
	card_stat = dev->d_dev->sd_data;

	RELEASE_MUTEX;

	// Error handling
	// {{{
	if (err) {
		// If we had any write failures along the way, return
		// an error status.

		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
	} else if (dev_stat & SDIO_ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		err = 1;
		if (EMMCDEBUG) {
			txstr("\tEMMC-WRITE -> ERR: ");
			txhex(dev_stat);
			txstr(":");
			txhex(card_stat);
			txstr("\n");
			if (EMMCINFO)
				sdio_dump_err(dev_stat);
		}
	} else if (card_stat & SDIO_R1ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		// If the card declares an error, return an error status
		if (EMMCDEBUG) {
			txstr("\tR1-ERR\n");
			if (EMMCINFO)
				emmc_dump_r1(card_stat);
		}
		err = 1;
	}
	// }}}

	if (err) {
		if (EMMCDEBUG)
			txstr("EMMC-WRITE -> R1 ERR\n");
		return	RES_ERROR;
	} return RES_OK;
}
// }}}

int	emmc_read(EMMCDRV *dev, const unsigned sector,
				const unsigned count, char *buf) {
	// {{{
	unsigned	err = 0, dev_stat, card_stat;

	if (0 == count)
		return RES_OK;

	if (!EMMCMULTI) {
		// {{{
		for(unsigned k=0; k<count; k++) {
			unsigned	st;

			st = emmc_read_block(dev, sector+k,
						(uint32_t *)(&buf[k*512]));
			if (0 != st) {
				// No need for error reporting here.  If
				// necessary, it took place in emmc_read_block
				return RES_ERROR;
			}
		} return RES_OK;
	}
	// }}}

	if (EMMCDEBUG) {
		// {{{
		txstr("EMMC-READ(MNY): ");
		txhex(sector);
		txstr(", ");
		txhex(count);
		txstr(", ");
		txhex(buf);
		txstr("-- [DEV ");
		txhex(dev->d_dev->sd_cmd);
		txstr("]\n");
	}
	// }}}

	GRAB_MUTEX;

	// Make sure our device is idle
	if(dev->d_dev->sd_cmd & SDIO_BUSY)
		emmc_wait_while_busy(dev);

	// Make sure our PHY is set up properly
	// {{{
	{
		unsigned	phy;

		phy = dev->d_dev->sd_phy;
		if (SECTOR_512B != (phy & (SECTOR_MASK | SDIOCK_SHUTDN))) {
			// Read multiple *requires* the clock be shut down
			// between pages, to make sure the device doesn't try
			// to produce data before we are ready for it.
			phy &= ~SECTOR_MASK;
			phy |= SECTOR_512B | SDIOCK_SHUTDN;
			dev->d_dev->sd_phy = phy;
		}
	}
	// }}}

	// Set the data sector
	// {{{
	if (dev->d_OCR & 0x40000000)
		// High capacity card
		dev->d_dev->sd_data = sector;
	else
		dev->d_dev->sd_data = sector*512;
	// }}}

	dev->d_dev->sd_dma_length = count;
	if (count == dev->d_dev->sd_dma_length) {
		// Activate the EMMC DMA
		// {{{
		dev->d_dev->sd_dma_addr = buf;
		dev->d_dev->sd_cmd = SDIO_ERR | SDIO_READDMA;
		// }}}
	} else {
		// Issue the read multiple command
		// {{{
		dev->d_dev->sd_cmd  = SDIO_ERR | SDIO_RDMULTI;
		// }}}

		// Read each sector
		// {{{
		for(unsigned s=0; s<count; s++) {
			// Wait until we have a block to read
			emmc_wait_while_busy(dev);

			// Send the next (or last) command
			// {{{
			if (0 != (err_stat = dev->d_dev->sd_cmd & SDIO_ERR)) {
				err = 1;
			} if (s + 1 < count && !err) {
				// Immediately start the next read request
				dev->d_dev->sd_cmd  = SDIO_MEM
					| ((s&1) ? 0 : SDIO_FIFO);
			} else {
				// Send a STOP_TRANSMISSION request
				dev->d_dev->sd_data = 0;
				dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b |SDIO_ERR) + 12;
			}
			// }}}

			// Now copy out the data that we've read
			// {{{
#ifdef	INCLUDE_DMA_CONTROLLER
			if (EXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
				_zip->z_dma.d_len = 512;
				_zip->z_dma.d_rd  = (s&1)
					? (char *)&dev->d_dev->sd_fifb
					: (char *)&dev->d_dev->sd_fifa;
				_zip->z_dma.d_wr  = (char *)&buf[s*512];
				_zip->z_dma.d_ctrl= DMAREQUEST|DMACLEAR|DMA_DSTWIDE
						| DMA_CONSTSRC|DMA_SRCWORD;
				while(_zip->z_dma.d_ctrl & DMA_BUSY)
					;
			} else
#endif
			{
				unsigned *dst;
				dst = (unsigned *)&buf[s*512];

				if (s&1) {
					for(int w=0; w<512/sizeof(uint32_t); w++)
						dst[w] = dev->d_dev->sd_fifb;
				} else {
					for(int w=0; w<512/sizeof(uint32_t); w++)
						dst[w] = dev->d_dev->sd_fifa;
				}
			}
			// }}}
		}
		// }}}
	}

	// Check the results of the STOP_TRANSMISSION request
	emmc_wait_while_busy(dev);
	dev_stat  = dev->d_dev->sd_cmd;
	card_stat = dev->d_dev->sd_data;

	RELEASE_MUTEX;
	CLEAR_DCACHE;

	// Error handling
	// {{{
	if (err) {
		// If we had any read failures along the way, return
		// an error status.

		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		if (EMMCDEBUG) {
			txstr("\tSD-MidREAD -> ERR: ");
			txhex(err_stat);
			txstr("\n");
			if (SDINFO)
				sdio_dump_err(err_stat);
		}
	} else if (dev_stat & SDIO_ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		err = 1;
		if (EMMCDEBUG) {
			txstr("\tEMMC-READ -> ERR: ");
			txhex(dev_stat);
			txstr(":");
			txhex(card_stat);
			txstr("\n");
			if (EMMCINFO)
				emmc_dump_err(dev_stat);
			sdio_get_r1(dev);
		}
		// If the stop transmission command didn't receive
		// a proper response, return an error status
	} else if (card_stat & SDIO_R1ERR) {
		// Immediately trigger the scope (if not already triggered)
		// to avoid potentially losing any more data.
		TRIGGER_SCOPE;
		// If the card declares an error, return an error status
		if (EMMCDEBUG) {
			txstr("\tR1-ERR\n");
			if (EMMCINFO)
				emmc_dump_r1(card_stat);
		}
		err = 1;
	}
	// }}}

	if (err) {
		if (EMMCDEBUG)
			txstr("EMMC-READ -> ERR\n");
		return RES_ERROR;
	} return RES_OK;
}
// }}}

int	emmc_ioctl(EMMCDRV *dev, char cmd, char *buf) {
	// {{{
	int		dstat;
	unsigned	vc;

	vc = dev->d_dev->sd_cmd;

	switch(cmd) {
	case CTRL_SYNC: {
			GRAB_MUTEX;
			emmc_wait_while_busy(dev);
			RELEASE_MUTEX;
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
	case MMC_GET_SDSTAT:
		return RES_OK;
		break;
	}

	return	RES_PARERR;
}
// }}}
