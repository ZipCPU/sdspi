////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdiodrv.c
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	This is the software driver for the SDIO controller, used for
//		interacting with an SD Card in SDIO (4b) mode.
//
// Entry points: This driver has four external entry points, which can be used
//	to define it's operation:
//
//	1. sdio_init
//		This should be called first.  It will generate the driver's
//		data structure, and attempt to interact with the card.
//		It needs to be passed the hardware address of the device in
//		the address map.
//	2.  sdio_write(dev, sector, count, buf)
//		Writes "count" sectors of data to the device, starting at
//		the sector numbered "sector".  The data are sourced from the
//		*buf pointer, which *must* either be word aligned or the CPU
//		must be able to handle unaligned accesses.
//
//		If SDMULTI is set, the write multiple blocks command will be
//		used for better performance.
//
//	3.  sdio_read(dev, sector, count, buf)
//		Reads "count" sectors of data to the device, starting at
//		the sector numbered "sector".  The data are saved into the
//		*buf pointer, which *must* either be word aligned or the CPU
//		must be able to handle unaligned accesses.
//
//		If SDMULTI is set, the read multiple blocks command will be
//		used for better performance.  This requires that the clock
//		shutdown bit be set, to avoid losing data.
//
//	4. sdio_ioctl
//
// Issues:
//	- This controller only handles 3.3V mode.  Even if hardware exists for
//	  switching to 1.8V, this driver doesn't (yet) enable it.
//	- The controller doesn't really recover well from a failed init.
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
typedef	uint8_t  BYTE;
typedef	uint16_t WORD;
typedef	uint32_t DWORD, LBA_t, UINT;
#include <diskio.h>
#include "sdiodrv.h"

#ifndef	TXFNS_H
#include <stdio.h>

#define	txchr(A)		putchar(A)
#define	txstr(A)		printf("%s",A)
#define	txhex(A)		printf("%08x", A)
#define	txdecimal(A)		printf("%d", A)
#else
// extern	void	txstr(const char *);
// extern	void	txhex(unsigned);
// extern	void	txdecimal(int);
#endif

static	const int	SDINFO = 1, SDDEBUG=1, SDEXTDMA=0;

// SDMULTI: Controls whether the read multiple block or write multiple block
// commands will be used.  Set to 1 to use these commands, 0 otherwise.
static	const int	SDMULTI = 1;
// }}}

/*
typedef	struct SDIO_S {
	volatile uint32_t	sd_cmd, sd_data, sd_fifa, sd_fifb, sd_phy;
} SDIO;
*/
#define	NEW_MUTEX
#define	GRAB_MUTEX
#define	RELEASE_MUTEX

typedef	struct	SDIODRV_S {
	SDIO		*d_dev;
	uint32_t	d_CID[4], d_OCR;
	char		d_SCR[8], d_CSD[16];
	uint16_t	d_RCA;
	uint32_t	d_sector_count, d_block_size;
} SDIODRV;

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
		SDIO_CMDECODE = 0x00030000,
		SDIO_REMOVED  = 0x00040000,
		SDIO_PRESENTN = 0x00080000,
		SDIO_CARDBUSY = 0x00100000,
		SDIO_BUSY     = 0x00104800,
		SDIO_CMDERR   = 0x00200000,
		SDIO_RXERR    = 0x00400000,
		SDIO_RXECODE  = 0x00800000,
		// PHY enumerations
		SDIO_DDR      = 0x00004100,	// Requires CK90
		SDIO_DS       = 0x00004300,	// Requires DDR & CK90
		SDIO_W1       = 0x00000000,
		SDIO_W4       = 0x00000400,
		SDIO_W8	      = 0x00000800,
		SDIO_WBEST    = 0x00000c00,
		SDIO_PPDAT    = 0x00001000,	// Push pull drive for data
		SDIO_PPCMD    = 0x00002000,	// Push pull drive for cmd wire
		SDIO_PUSHPULL = SDIO_PPDAT | SDIO_PPCMD,
		SDIOCK_CK90   = 0x00004000,
		SDIOCK_SHUTDN = 0x00008000,
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
		SDIOCK_1P2V   = 0x00400000,
		SDIOCK_DS     = SDIOCK_25MHZ | SDIO_W4 | SDIO_PUSHPULL,
		SDIOCK_HS     = SDIOCK_50MHZ | SDIO_W4 | SDIO_PUSHPULL,
		// Speed abbreviations
		SDIOCK_SDR50  = SDIOCK_50MHZ  | SDIO_W4 | SDIO_PUSHPULL | SDIOCK_1P2V,
		SDIOCK_DDR50  = SDIOCK_50MHZ  | SDIO_W4 | SDIO_PUSHPULL | SDIO_DDR | SDIOCK_1P2V,
		SDIOCK_SDR104 = SDIOCK_100MHZ | SDIO_W4 | SDIO_PUSHPULL | SDIOCK_1P2V,
		SDIOCK_SDR200 = SDIOCK_200MHZ | SDIO_W4 | SDIO_PUSHPULL | SDIOCK_1P2V,
		// SDIOCK_HS400= SDIOCK_200MHZ | SDIO_W4 | SDIO_PUSHPULL | SDIO_DS,
		//
		SPEED_SLOW   = SDIOCK_100KHZ,
		SPEED_DEFAULT= SDIOCK_DS,
		SPEED_FAST   = SDIOCK_HS,
		//
		SECTOR_16B   = 0x04000000,
		SECTOR_512B  = 0x09000000,
		//
		SDIO_CMD     = 0x00000040,
		SDIO_R1ERR   = 0xff800000,
		SDIO_READREG  = SDIO_CMD | SDIO_R1,
		SDIO_READREGb = SDIO_CMD | SDIO_R1b,
		SDIO_READR2  = (SDIO_CMD | SDIO_R2),
		SDIO_WRITEBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_WRITE | SDIO_MEM) + 24,
		SDIO_READBLK  = (SDIO_CMD | SDIO_R1
					| SDIO_MEM) + 17;

static	void	sdio_wait_while_busy(SDIODRV *dev);
static	void	sdio_go_idle(SDIODRV *dev);
static	void	sdio_all_send_cid(SDIODRV *dev);
static	void	sdio_dump_cid(SDIODRV *dev);
static	uint32_t sdio_send_rca(SDIODRV *dev);
static	void	sdio_select_card(SDIODRV *dev);	// CMD7
static	uint32_t sdio_send_if_cond(SDIODRV *dev, uint32_t ifcond); // CMD8
static	uint32_t sdio_send_op_cond(SDIODRV *dev, uint32_t opcond); // ACMD41
static	void	sdio_set_bus_width(SDIODRV *dev, uint32_t width); // CMD6
static	void	sdio_send_app_cmd(SDIODRV *dev);  // CMD 55
static	uint32_t sdio_read_ocr(SDIODRV *dev, uint32_t width);	  // CMD 58
static	void	sdio_dump_scr(SDIODRV *dev);
static	void	sdio_dump_ocr(SDIODRV *dev);
static	unsigned sdio_get_r1(SDIODRV *dev);
static	void	sdio_dump_r1(unsigned);
static	int	sdio_write_block(SDIODRV *dev, uint32_t sector, uint32_t *buf);	  // CMD 24
static	int	sdio_read_block(SDIODRV *dev, uint32_t sector, uint32_t *buf);	  // CMD 17

extern	SDIODRV *sdio_init(SDIO *dev);
extern	int	sdio_write(SDIODRV *dev, const unsigned sector, const unsigned count, const char *buf);
extern	int	sdio_read(SDIODRV *dev, const unsigned sector, const unsigned count, char *buf);
extern	int	sdio_ioctl(SDIODRV *dev, char cmd, char *buf);


void	sdio_wait_while_busy(SDIODRV *dev) {
	// {{{

	// Could also do a system call and yield to the scheduler while waiting
	// if (SDIO_OS) {
	//	os_wait(dev->d_int);
	// } else if (SDIO_INT) {
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

void	sdio_go_idle(SDIODRV *dev) {				// CMD0
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_REMOVED | SDIO_CMD | SDIO_RNONE | SDIO_ERR;

	sdio_wait_while_busy(dev);
}
// }}}

void	sdio_all_send_cid(SDIODRV *dev) {	// CMD2
	// {{{
	if (SDDEBUG)	txstr("READ-CID\n");

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READR2)+2;

	sdio_wait_while_busy(dev);

	dev->d_CID[0] = dev->d_dev->sd_fifa;
	dev->d_CID[1] = dev->d_dev->sd_fifa;
	dev->d_CID[2] = dev->d_dev->sd_fifa;
	dev->d_CID[3] = dev->d_dev->sd_fifa;

	if (SDINFO)
		sdio_dump_cid(dev);
}
// }}}

void	sdio_send_cid(SDIODRV *dev) {	// CMD10
	// {{{
	if (SDDEBUG)	txstr("SEND-CID\n");

	dev->d_dev->sd_data = (dev->d_RCA << 16);
	dev->d_dev->sd_cmd  = (SDIO_ERR|SDIO_READR2) + 10;

	sdio_wait_while_busy(dev);

	dev->d_CID[0] = dev->d_dev->sd_fifa;
	dev->d_CID[1] = dev->d_dev->sd_fifa;
	dev->d_CID[2] = dev->d_dev->sd_fifa;
	dev->d_CID[3] = dev->d_dev->sd_fifa;

	if (SDINFO)
		sdio_dump_cid(dev);
}
// }}}

void	sdio_dump_cid(SDIODRV *dev) {
	// {{{
	txstr("CID: ");
	txhex(dev->d_CID[0]); txstr(":");
	txhex(dev->d_CID[1]); txstr(":");
	txhex(dev->d_CID[2]); txstr(":");
	txhex(dev->d_CID[3]); txstr("\n");

#ifdef	STDIO_DEBUG
	unsigned sn, md;

	sn = dev->d_CID[2];
	sn = (sn << 8) | (dev->d_CID[3] >> 24) & 0x0ff;
	md = (dev->d_CID[3] >>  8) & 0x0fff;

	printf("CID:\n"
"\tManufacturer ID:  0x%02x\n"
"\tApplication ID:   %c%c\n"
"\tProduct Name:     %5c\n"
"\tProduct Revision: %x.%x\n"
"\tSerial Number:    0x%0x\n",
		(dev->d_CID[0] >> 24)&0x0ff,
		(dev->d_CID[0] >> 16)&0x0ff,
		(dev->d_CID[0] >>  8)&0x0ff,
		(dev->d_CID[0]      )&0x0ff,
		(dev->d_CID[1] >> 24)&0x0ff,
		(dev->d_CID[1] >> 16)&0x0ff,
		(dev->d_CID[1] >>  8)&0x0ff,
		(dev->d_CID[1]      )&0x0ff,
		(dev->d_CID[2] >> 28)&0x00f,
		(dev->d_CID[2] >> 24)&0x00f, sn);
	printf(
"\tYear of Man.:     %d\n"
"\tMonth of Man.:    %d\n",
		((md>>4)+2000), md&0x0f);
#endif
}
// }}}

uint32_t sdio_send_rca(SDIODRV *dev) {				// CMD3
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READREG)+3;

	sdio_wait_while_busy(dev);
	return dev->d_RCA = (dev->d_dev->sd_data >> 16)&0x0ffff;
}
// }}}

void	sdio_select_card(SDIODRV *dev) {			// CMD7
	// {{{
	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = SDIO_READREGb+7;

	sdio_wait_while_busy(dev);
}
// }}}

uint32_t sdio_send_if_cond(SDIODRV *dev, uint32_t ifcond) { // CMD8
	// {{{
	unsigned	c, r;

	dev->d_dev->sd_data = ifcond;
	dev->d_dev->sd_cmd = SDIO_READREG+8;

	sdio_wait_while_busy(dev);

	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;

	if (SDDEBUG && SDINFO) {
		txstr("CMD8:    SEND_IF_COND\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}

	return r;
}
// }}}

uint32_t sdio_send_op_cond(SDIODRV *dev, uint32_t opcond) { // ACMD41
	// {{{
	unsigned	c, r;

	sdio_send_app_cmd(dev);

	dev->d_dev->sd_data = opcond;
	dev->d_dev->sd_cmd = SDIO_READREG+41;

	sdio_wait_while_busy(dev);
	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;

	dev->d_OCR = r;
	if (SDDEBUG && SDINFO) {
		txstr("CMD8:    SEND_OP_COND : "); txhex(opcond); txstr("\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}

	return r;
}
// }}}

void	sdio_set_bus_width(SDIODRV *dev, uint32_t width) { // CMD6
	// {{{
	sdio_send_app_cmd(dev);

	dev->d_dev->sd_data = width;
	dev->d_dev->sd_cmd = SDIO_READREG+6;

	sdio_wait_while_busy(dev);

	if (SDDEBUG && SDINFO) {
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD6:    SET_BUS_WIDTH\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
}
// }}}

void	sdio_send_app_cmd(SDIODRV *dev) {  // CMD 55
	// {{{
	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READREG)+55;

	sdio_wait_while_busy(dev);

	if (SDDEBUG && SDINFO) {
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD55:   SEND_APP_CMD\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
}
// }}}

void	sdio_dump_ocr(SDIODRV *dev) {
	// {{{
	if (SDINFO) {
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
#endif
	}
}
// }}}

void sdio_read_scr(SDIODRV *dev) {	  // ACMD 51
	// {{{
	uint32_t	phy = dev->d_dev->sd_phy;

	phy &= 0xf0ffffff;
	phy |= (3 << 24);	// 64 bits = 8 bytes = 2^3 bytes
	dev->d_dev->sd_phy = phy;

	sdio_send_app_cmd(dev);

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_MEM|SDIO_READREG)+51;

	sdio_wait_while_busy(dev);

	if (SDDEBUG && SDINFO) {
		// {{{
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("ACMD51:  SEND_SCR\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
	// }}}

	for(int k=0; k<8; k+= sizeof(uint32_t)) {
		unsigned	uv;

		uv = dev->d_dev->sd_fifa;
		if (SDINFO) { txhex(uv); if (k < 4) txstr(":"); }
		dev->d_SCR[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 0] = uv;
	} if (SDINFO) txstr("\n");

	phy &= 0xf0ffffff;
	phy |= SECTOR_512B;
	dev->d_dev->sd_phy = phy;

	if (SDINFO)
		sdio_dump_scr(dev);
}
// }}}

void	sdio_dump_scr(SDIODRV *dev) {
	// {{{
	txstr("  SCR_STRUCTURE: "); txhex((dev->d_SCR[0] >> 4)& 0x0f); txstr("\n");
	txstr("  SD_SPEC      : "); txhex(dev->d_SCR[0]& 0x0f); txstr("\n");
	txstr("  STATAFTERERAS: "); txhex((dev->d_SCR[1]&0x80)?1:0); txstr("\n");
	txstr("  SD_SECURITY  : "); txhex((dev->d_SCR[1]>>4) & 0x07); txstr("\n");
	txstr("  SD_BUS_WIDTHS: "); txhex(dev->d_SCR[1] & 0x0f); txstr("\n");
	txstr("  EX_SECURITY  : "); txhex((dev->d_SCR[2]>>3) & 0x07); txstr("\n");
	txstr("  SD_SPEC3     : "); txhex((dev->d_SCR[2] & 0x80) ? 1:0); txstr("\n");
	txstr("  SD_SPEC4     : "); txhex((dev->d_SCR[2] & 0x04) ? 1:0); txstr("\n");
	txstr("  SD_SPECX     : "); txhex(((dev->d_SCR[3]>>6)&0x03)
					| ((dev->d_SCR[2]&0x03)<<2)); txstr("\n");
	txstr("  CMD_SUPPORT  : "); txhex(dev->d_SCR[3] & 0x0f); txstr("\n");
}
// }}}

void sdio_read_csd(SDIODRV *dev) {	  // CMD 9
	// {{{
	if (SDDEBUG)	txstr("READ-CSD\n");

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = (SDIO_CMD | SDIO_R2 | SDIO_ERR)+9;

	sdio_wait_while_busy(dev);

	if (SDDEBUG && SDINFO) {
		// {{{
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD9:    SEND_CSD\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
	// }}}

	for(int k=0; k<16; k+= sizeof(uint32_t)) {
		unsigned	uv;

		uv = dev->d_dev->sd_fifa;
		if (SDINFO) { txhex(uv); if (k < 12) txstr(":"); }
		dev->d_CSD[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 0] = uv;
	}

	unsigned	C_SIZE, READ_BL_LEN, CSD_STRUCTURE;

	CSD_STRUCTURE = (dev->d_CSD[0]>>6)&3;

	if (SDINFO && SDDEBUG) {
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

	if (SDINFO) {
		txstr("\n");
		txstr("  CSD_STRUCTURE : "); txhex(CSD_STRUCTURE); txstr("\n");
	} if (0 == CSD_STRUCTURE) {
		// {{{
		unsigned	ERASE_BLK_EN, C_SIZE_MULT, BLOCK_LEN, MULT,
				BLOCKNR, FILE_FORMAT_GRP, FILE_FORMAT,
				WRITE_BL_LEN, SECTOR_SIZE;

		READ_BL_LEN = dev->d_CSD[5] & 0x0f;
		BLOCK_LEN = (READ_BL_LEN < 12) ? (1<<READ_BL_LEN) : READ_BL_LEN;
		C_SIZE = ((dev->d_CSD[6]&0x3)<<10)
				|((dev->d_CSD[7]&0x0ff)<<2)
				|((dev->d_CSD[8]>>6)&0x03);
		C_SIZE_MULT = ((dev->d_CSD[9]&3)<<2)|((dev->d_CSD[10]>>7)&1);
		MULT = 1<<(C_SIZE_MULT+2);
		BLOCKNR = MULT * (C_SIZE+1);
		SECTOR_SIZE = ((dev->d_CSD[10]&0x03f)<<2)|((dev->d_CSD[11]>>7)&1);
		WRITE_BL_LEN = ((dev->d_CSD[12]&3)<<2)|((dev->d_CSD[13]>>6)&3);
		FILE_FORMAT_GRP = (dev->d_CSD[14]&0x80)?1:0;
		FILE_FORMAT = dev->d_CSD[14]&3;
		ERASE_BLK_EN = (dev->d_CSD[10] & 0x40)?1:0;

		dev->d_sector_count = BLOCKNR / 512;
		if (ERASE_BLK_EN)
			dev->d_block_size = 512;
		else
			dev->d_block_size = SECTOR_SIZE;

		if (SDINFO) {
			txstr("  TAAC          : "); txhex(dev->d_CSD[1]); txstr("\n");
			txstr("  NSAC          : "); txhex(dev->d_CSD[2]); txstr("\n");
			txstr("  TRAN_SPEED        : "); txhex(dev->d_CSD[3]); txstr("\n");
			txstr("  CCC               : "); txhex((dev->d_CSD[4]<<4)|((dev->d_CSD[10]&0x0f0)>>4)); txstr("\n");
			txstr("  READ_BL_LEN       : "); txhex(READ_BL_LEN); txstr("\n");
			txstr("  READ_BL_PARTIAL   : "); txhex((dev->d_CSD[6] & 0x80) ? 1:0); txstr("\n");
			txstr("  WRITE_BLK_MISALIGN: "); txhex((dev->d_CSD[6] & 0x40) ? 1:0); txstr("\n");
			txstr("  READ_BLK_MISALIGN : "); txhex((dev->d_CSD[6] & 0x20) ? 1:0); txstr("\n");
			txstr("  DSR_IMP           : "); txhex((dev->d_CSD[6] & 0x10) ? 1:0); txstr("\n");
			txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\n");
			txstr("  VDD_R_CURR_MIN    : "); txhex((dev->d_CSD[8]>>3)&0x07); txstr("\n");
			txstr("  VDD_R_CURR_MAX    : "); txhex(dev->d_CSD[8]&0x07); txstr("\n");
			txstr("  VDD_W_CURR_MIN    : "); txhex((dev->d_CSD[9]>>5)&0x07); txstr("\n");
			txstr("  VDD_W_CURR_MAX    : "); txhex((dev->d_CSD[9]>>2)&0x07); txstr("\n");
			txstr("  C_SIZE_MULT       : "); txhex(C_SIZE_MULT); txstr("\n");
			txstr("  ERASE_BLK_EN      : "); txhex((dev->d_CSD[10] & 0x40)?1:0); txstr("\n");
			txstr("  SECTOR_SIZE       : "); txhex(SECTOR_SIZE);
				txstr(" (");
				txdecimal((SECTOR_SIZE+1)*(1<<WRITE_BL_LEN));
				txstr(" bytes)\n");
			txstr("  WP_GRP_SIZE       : "); txhex(dev->d_CSD[11]&0x07f); txstr("\n");
			txstr("  WP_GRP_ENABLE     : "); txhex((dev->d_CSD[12]&0x080)?1:0); txstr("\n");
			txstr("  R2W_FACTOR        : "); txhex((dev->d_CSD[12]>>2)&7); txstr("\n");
			txstr("  WRITE_BL_LEN      : "); txhex(WRITE_BL_LEN);
			if (WRITE_BL_LEN < 9 || WRITE_BL_LEN > 11)
				txstr(" (Reserved)\n");
			else {
				txstr(" (");
				txdecimal(1<<WRITE_BL_LEN); txstr(" bytes)\n");
			}
			txstr("  WRITE_BL_PARTIAL  : "); txhex((dev->d_CSD[13]&0x20)?1:0); txstr("\n");
			txstr("  FILE_FORMAT_GRP   : "); txhex(FILE_FORMAT_GRP); txstr("\n");
			txstr("  COPY              : "); txhex((dev->d_CSD[14]&0x40)?1:0); txstr("\n");
			txstr("  PERM_WRITE_PROTECT: "); txhex((dev->d_CSD[14]&0x20)?1:0); txstr("\n");
			txstr("  TMP_WRITE_PROTECT : "); txhex((dev->d_CSD[14]&0x10)?1:0); txstr("\n");
			txstr("  FILE_FORMAT       : "); txhex(FILE_FORMAT);
			if (0 == FILE_FORMAT_GRP) {
				if (0 == FILE_FORMAT)
					txstr("  (Has parition tbl)\n");
				else if (1 == FILE_FORMAT)
					txstr("  (DOS FAT w/ boot sector, no partition)\n");
				else if (2 == FILE_FORMAT)
					txstr("  (Universal file format)\n");
				else
					txstr("  (Others/unknown)\n");
			} else
				txstr("  (Reserved)\n");
			txstr("  Size  = "); txdecimal(BLOCKNR); txstr(" blocks of ");
			txdecimal(BLOCK_LEN); txstr(" bytes each\n");
		}
		// }}}
	} else if (1 == CSD_STRUCTURE) {
		// {{{
		READ_BL_LEN = 9;
		C_SIZE = ((dev->d_CSD[7] & 0x03f)<<16)
				|((dev->d_CSD[8] & 0x0ff)<<8)
				|(dev->d_CSD[9] & 0x0ff);
		dev->d_sector_count = (C_SIZE+1) * 1024;
		dev->d_block_size = 512;

		if (SDINFO) {
			txstr("  CCC               : "); txhex((dev->d_CSD[4]<<4)|((dev->d_CSD[5]&0x0f0)>>4)); txstr("\n");
			txstr("  DSR_IMP           : "); txhex((dev->d_CSD[6] & 0x10) ? 1:0); txstr("\n");
			txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\n");
			txstr("  COPY              : "); txhex((dev->d_CSD[14]&0x40)?1:0); txstr("\n");
			txstr("  PERM_WRITE_PROTECT: "); txhex((dev->d_CSD[14]&0x20)?1:0); txstr("\n");
			txstr("  TMP_WRITE_PROTECT : "); txhex((dev->d_CSD[14]&0x10)?1:0); txstr("\n");
			txstr("  Size  = "); txdecimal((C_SIZE+1) * 512); txstr(" kB\n");
		}
		// }}}
	} else {
		txstr("ERROR: Unknown CSD type: "); txhex(CSD_STRUCTURE); txstr("\n");
		dev->d_sector_count = 0;
		dev->d_block_size   = 0;
	}
}
// }}}

unsigned sdio_switch(SDIODRV *dev, unsigned swcmd, unsigned *ubuf) {  // CMD 6
	// {{{
	unsigned	c, phy, fail = 0;

	if (SDDEBUG)	txstr("CMD-SWITCH_FUNC\n");

	phy = dev->d_dev->sd_phy;
	phy &= 0xf0ffffff;
	phy |= (6 << 24);	// 512bit response => 64 bytes
	dev->d_dev->sd_phy = phy;

	// bit 31: 0 => check function, 1 => switch function
	// [30:24]	Must == 0
	// [15:12]	funtion group 4 (Power Limit)
	//		4'h0 => Default value
	//		4'hf => No change, keep the same value
	// [11: 8]	funtion group 3 (Drive Strength)
	// [ 7: 4]	funtion group 2 (Command system)
	// [ 3: 0]	funtion group 1 (Access mode)
	//
	// Set to 0x80fffff1 to switch to HS/SDR25 mode (50MHz clock)
	// Requiring 1.8V ...
	// Set to 0x80fffff2 to switch to SDR50  mode (104MHz clock)
	// Set to 0x80fffff3 to switch to SDR104 mode (208MHz clock)
	// Set to 0x80fffff4 to switch to DDR50  mode (50MHz clock)
	dev->d_dev->sd_data = swcmd;
	dev->d_dev->sd_cmd = (SDIO_CMD | SDIO_R1 | SDIO_ERR | SDIO_MEM)+6;

	sdio_wait_while_busy(dev);

	c = dev->d_dev->sd_cmd;
	if ((c & SDIO_RXERR) && (c & SDIO_RXECODE))
		fail = 1;
	if (SDDEBUG && SDINFO) {
		// {{{
		unsigned	r;

		r = dev->d_dev->sd_data;
		txstr("CMD6:    SWITCH_FUNC\n");
		txstr("  Cmd:     "); txhex(c);
		if (fail)
			txstr(" -- FAIL");
		txstr("\n");
		txstr("  Data:    "); txhex(swcmd);
			txstr(" -> "); txhex(r); txstr("\n");
	}
	// }}}

	if (ubuf) {
		for(int k=0; k<512/32; k++)
			ubuf[k] = dev->d_dev->sd_fifa;
	}

	phy &= 0xf0ffffff;
	phy |= SECTOR_512B;	// Return to a 512Byte PHY setting
	dev->d_dev->sd_phy = phy;

	return fail;
}
// }}}

void sdio_dump_r1(const unsigned rv) {
	// {{{
	txstr("SDIO R1 Decode:  "); txhex(rv);
	if (rv & 0x80000000)
		txstr("\n  OUT_OF_RANGE");
	if (rv & 0x40000000)
		txstr("\n  ADDRESS_ERROR");
	if (rv & 0x20000000)
		txstr("\n  BLOCK_LEN_ERROR");
	if (rv & 0x10000000)
		txstr("\n  ERASE_SEQ_ERROR");
	if (rv & 0x08000000)
		txstr("\n  ERASE_PARAM");
	if (rv & 0x04000000)
		txstr("\n  WP_VIOLATION");
	if (rv & 0x02000000)
		txstr("\n  CARD_IS_LOCKED");
	if (rv & 0x01000000)
		txstr("\n  LOCK_UNLOCK_FAILED");
	if (rv & 0x00800000)
		txstr("\n  COM_CRC_ERROR");
	if (rv & 0x00400000)
		txstr("\n  ILLEGAL_COMMAND");
	if (rv & 0x00200000)
		txstr("\n  CARD_ECC_FAILED");
	if (rv & 0x00100000)
		txstr("\n  CC_ERROR (Internal card controller err)");
	if (rv & 0x00080000)
		txstr("\n  ERROR (General or unknown error)");
	if (rv & 0x00010000)
		txstr("\n  CSD_OVERWRITE");
	if (rv & 0x00008000)
		txstr("\n  WP_ERASE_SKIP");
	if (rv & 0x00004000)
		txstr("\n  CARD_ECC_DISABLED");
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
	case 15: txstr("\n  STATE: (reserved for I/O mode)"); break;
	default: txstr("\n  STATE: (reserved)"); break;
	}
	if (rv & 0x00000100)
		txstr("\n  READY_FOR_DATA");
	if (rv & 0x00000040)
		txstr("\n  FX_EVENT");
	if (rv & 0x00000020)
		txstr("\n  APP_CMD");
	if (rv & 0x00000008)
		txstr("\n  AKE_SEQ_ERROR");
	txstr("\n");
}
// }}}

unsigned sdio_get_r1(SDIODRV *dev) {	// CMD13=send_status
	// {{{
	unsigned	vc, vd;

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd  = (SDIO_ERR|SDIO_READREG) + 13;

	sdio_wait_while_busy(dev);

	txhex(vc = dev->d_dev->sd_cmd);
	txstr(" : ");
	txhex(vd = dev->d_dev->sd_data);
	txstr("\n");

	sdio_dump_r1(vd);
	return	vd;
}
// }}}

static void sdio_dump_sector(const unsigned *ubuf) {
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

int	sdio_write_block(SDIODRV *dev, uint32_t sector, uint32_t *buf){// CMD 24
	// {{{
	unsigned	vc, vd;

	GRAB_MUTEX;

	if (9 != ((dev->d_dev->sd_phy >> 24)&0x0f)) {
		uint32_t	phy = dev->d_dev->sd_phy;
		phy &= 0xf0ffffff;
		phy |= SECTOR_512B;
	}

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

	dev->d_dev->sd_data = sector;
	dev->d_dev->sd_cmd = SDIO_WRITEBLK;

	sdio_wait_while_busy(dev);

	vc = dev->d_dev->sd_cmd;
	vd = dev->d_dev->sd_data;

	RELEASE_MUTEX;

	if (SDDEBUG) {
		if (vc & SDIO_ERR) {
			txstr("SDIO ERR: SDIO write response = ");
			txhex(vc);
			txstr("\n");
		}

		if (SDINFO)
			sdio_get_r1(dev);
	} if (vc & SDIO_ERR) {
		dev->d_dev->sd_cmd = SDIO_ERR;
		return -1;
	} return 0;
}
// }}}

int	sdio_read_block(SDIODRV *dev, uint32_t sector, uint32_t *buf){// CMD 17
	// {{{
	unsigned	cmd;

	if (SDDEBUG) {
		txstr("SDIO-READ: ");
		txhex(sector);
		txstr("\n");
	}

	if (dev->d_dev->sd_cmd & SDIO_REMOVED) {
		txstr("SDIO ERR: SD-Card was removed\n");
		return -1;
	}

	GRAB_MUTEX;

	if (9 != ((dev->d_dev->sd_phy >> 24)&0x0f)) {
		uint32_t	phy = dev->d_dev->sd_phy;
		phy &= 0xf0ffffff;
		phy |= SECTOR_512B;
	}

	dev->d_dev->sd_data = sector;
	dev->d_dev->sd_cmd = SDIO_READBLK;

	sdio_wait_while_busy(dev);

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

	if (SDDEBUG && SDINFO)
		sdio_dump_sector(buf);

	cmd = dev->d_dev->sd_cmd;
	RELEASE_MUTEX;

	if (cmd & (SDIO_ERR | SDIO_REMOVED)) {
		txstr("SDIO ERR: Read\n");
		return -1;
	} return 0;
}
// }}}

SDIODRV *sdio_init(SDIO *dev) {
	// {{{
	unsigned	ifcond, op_cond;
	SDIODRV	*dv = (SDIODRV *)malloc(sizeof(SDIODRV));
	dv->d_dev = dev;
	dv->d_RCA = 0;
	dv->d_sector_count = 0;
	dv->d_block_size   = 0;

	// NEW_MUTEX;
	GRAB_MUTEX;

	dv->d_dev->sd_phy = SPEED_SLOW | SECTOR_512B;
	while(SPEED_SLOW != (dv->d_dev->sd_phy & 0x0ff))
		;

	sdio_go_idle(dv);
	ifcond = sdio_send_if_cond(dv,0x01a5);
	if (0x08000 == (dv->d_dev->sd_cmd & 0x038000)) {
		do {
			op_cond = 0;
			op_cond = sdio_send_op_cond(dv, op_cond);
		} while(op_cond & 0x80000000);
	} else {
		if (0xa5 != (ifcond & 0x0ff)) {
			RELEASE_MUTEX;

			txstr("SDIO ERROR: IFCOND returned ");
			txhex(ifcond); txstr("\n");
			free(dv);
			return NULL;
		}

		do {
			op_cond = 0x40ff8000;
			op_cond = sdio_send_op_cond(dv, op_cond);
		} while(0 == (op_cond & 0x80000000));
	} if (SDINFO)
		sdio_dump_ocr(dv);

	sdio_all_send_cid(dv);

	sdio_send_rca(dv);
	sdio_send_cid(dv);

	dv->d_dev->sd_phy = SECTOR_512B | SDIOCK_25MHZ | SDIO_PUSHPULL;
	while(SDIOCK_25MHZ != (dv->d_dev->sd_phy & 0x0ff))
		; // Wait for the clock to change

	sdio_read_csd(dv);

	sdio_select_card(dv);

	// SEND_SCR
	sdio_read_scr(dv);

	// LOCK_UNLOCK ?
	// SET_BUS_WIDTH
	if (dv->d_SCR[1] & 0x04) {
		dv->d_dev->sd_phy |= SDIO_WBEST;
		if (0 != (dv->d_dev->sd_phy & SDIO_WBEST)) {
			// Set a 4-bit bus width via ACMD6
			sdio_set_bus_width(dv, 2);
			dv->d_dev->sd_phy |= SDIO_W4;
		}
	}

	// Do we support HS mode?  If so, let's switch to it
	// {{{
	{
		unsigned phy = dv->d_dev->sd_phy;

		// Shut the clock down and switch to 50MHz.  Then come back
		// and read the clock.  If it doesn't set to 50MHz, then we
		// couldn't set the clock, and so we should abandon our attempt.
		dv->d_dev->sd_phy = SDIOCK_SHUTDN
					| (phy & ~0x0ff) | SDIOCK_50MHZ;

		for(int k=0; k<50; k++)
			if (SDIOCK_50MHZ == (dv->d_dev->sd_phy & 0x0ff))
				break;

		if ((dv->d_dev->sd_phy & 0x0ff) != SDIOCK_50MHZ) {
			// 50MHz is not supported by our PHY

			if (SDDEBUG) {
				txstr("No PHY support for 50MHz: ");
				txhex(dv->d_dev->sd_phy);
				txstr("\n");
			}

			// Return the PHY to its original setting
			dv->d_dev->sd_phy = phy;
		} else if ((dv->d_SCR[1] & 0x0f) > 0) {
			// CMD6 to move to HS mode
			unsigned	ubuf[16];

			// Return the PHY to its original setting
			dv->d_dev->sd_phy = phy;

			// First, query if the mode is available
			sdio_switch(dv, 0x00fffff1, ubuf);

			if (SDINFO) {
				// {{{
				txstr("Function modes supported: ");
				txhex(ubuf[3]);
				txstr("\n");
			}
			// }}}

			// If HS mode is available, switch to it
			// {{{
			if ((0 == (dv->d_dev->sd_cmd & SDIO_ERR))
					&& (0 != (ubuf[3] & 0x020000))) {
				// We don't need to read the response, since
				// we now know it's supported--just send the
				// request.
				sdio_switch(dv, 0x80fffff1, NULL);
				phy = (dv->d_dev->sd_phy & (~0x0ff))
							| SDIOCK_50MHZ;
				dv->d_dev->sd_phy = phy;
				phy = dv->d_dev->sd_phy;
				phy = (phy & 0xffe0ffff) | 0x080000;
				dv->d_dev->sd_phy = phy;
			} else if (SDINFO || SDDEBUG) {
				txstr("HS mode is unavailable\n");
				if (SDDEBUG) {
					txstr("SD-CMD: "); txhex(dv->d_dev->sd_cmd);
					txstr("\n");
					txstr("BUF[3]: "); txhex(ubuf[3]);
					txstr("\n");
				}
			}
			// }}}
		}
	}
	// }}}

	// Select drive strength?
	/* if 1.8V signaling
	if (dv->d_dev->sd_phy & (OPT_1P8V | SDIO_18V)) {
		// CMD19, tuning block to determine sample point
	}
	*/

	RELEASE_MUTEX;

	if (SDDEBUG) {
		printf("Block size:   %d\n", dv->d_block_size);
		printf("Sector count: %d\n", dv->d_sector_count);
	}

	return	dv;
}
// }}}

int	sdio_write(SDIODRV *dev, const unsigned sector,
			const unsigned count, const char *buf) {
	// {{{
	unsigned	st, s;

	if (1 == count || !SDMULTI) {
		for(unsigned k=0; k<count; k++) {
			st = sdio_write_block(dev, sector, (uint32_t *)buf);
			if (0 != st)
				return RES_ERROR;
		} return RES_OK;
	} else {
		unsigned	card_stat, phy;

		GRAB_MUTEX;

		phy = dev->d_dev->sd_phy;
		if (9 != ((phy >> 24)&0x0f)) {
			// Here, we don't care if the clock is shut down as
			// well, since we control the write speed.
			phy &= 0xf0ffffff;
			phy |= SECTOR_512B;
		}

		for(unsigned s=0; s<count; s++) {
#ifdef	INCLUDE_DMA_CONTROLLER
			if (SDEXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))){
				_zip->z_dma.d_len = 512;
				_zip->z_dma.d_rd  = (char *)buf;
				_zip->z_dma.d_wr  = (s&1)
					? (char *)&dev->d_dev->sd_fifb
					: (char *)&dev->d_dev->sd_fifa;
				_zip->z_dma.d_ctrl= DMAREQUEST|DMACLEAR|DMA_SRCWIDE
						|DMA_CONSTDST|DMA_DSTWORD;
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

			if (s == 0) { // Issue the WRITE_MULTIPLE_BLOCK cmd
				// {{{
				// Make sure our device is idle
				while(dev->d_dev->sd_cmd & SDIO_BUSY)
					;

				// Issue a write-multiple command
				if (dev->d_OCR & 0x40000000)
					// High capacity card
					dev->d_dev->sd_data = sector;
				else
					dev->d_dev->sd_data = sector*512;
				dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1
					|SDIO_ERR|SDIO_WRITE | SDIO_MEM) + 25;

				while(dev->d_dev->sd_cmd & SDIO_CMDBUSY)
					;

				// Check if we'll have any errors with this cmd
				if (dev->d_dev->sd_cmd & SDIO_ERR) {
					RELEASE_MUTEX;
					return RES_ERROR;
				}

				card_stat = dev->d_dev->sd_data;
				if (card_stat & SDIO_R1ERR) {
					RELEASE_MUTEX;
					return RES_ERROR;
				}

				// Don't wait.  Go around again and load the
				// next block of data before checking if the
				// write has completed.
				// }}}
			} else { // Send the next block
				// {{{
				// Wait for the previous write to complete
				while(dev->d_dev->sd_cmd & SDIO_BUSY)
					;

				// Then send another block of data
				dev->d_dev->sd_cmd = (SDIO_WRITE | SDIO_MEM)
					+ ((s&1) ? SDIO_FIFO : 0);
				// }}}
			}
		}

		// Wait for the final write to complete
		while(dev->d_dev->sd_cmd & SDIO_BUSY)
			;

		// Send a STOP_TRANSMISSION request
		dev->d_dev->sd_data = 0;
		dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b |SDIO_ERR) + 12;
		while(dev->d_dev->sd_cmd & SDIO_BUSY)
			;

		if (dev->d_dev->sd_cmd & SDIO_ERR) {
			RELEASE_MUTEX;
			return RES_ERROR;
		}

		card_stat = dev->d_dev->sd_data;
		RELEASE_MUTEX;

		if (card_stat & SDIO_R1ERR)
			return RES_ERROR;

		return RES_OK;
	}
}
// }}}

int	sdio_read(SDIODRV *dev, const unsigned sector,
				const unsigned count, char *buf) {
	// {{{
	unsigned	st = 0;

	if (1 == count || !SDMULTI) {
		for(unsigned k=0; k<count; k++) {
			st = sdio_read_block(dev, sector+k,
						(uint32_t *)(&buf[k*512]));
			if (0 != st) {
				return RES_ERROR;
			}
		} return RES_OK;
	} else {
		unsigned	err, card_stat, phy, cmd;

		GRAB_MUTEX;

		phy = dev->d_dev->sd_phy;
		if ((0 == (phy & SDIOCK_SHUTDN)) || (9 != ((phy >> 24)&0x0f))) {
			// Read multiple *requires* the clock be shut down
			// between pages, to make sure the device doesn't try
			// to produce data before we are ready for it.
			phy &= 0xf0ffffff;
			phy |= SECTOR_512B | SDIOCK_SHUTDN;
		}

		err = 0;
		// Issue the read multiple command
		// {{{
		if (dev->d_OCR & 0x40000000)
			// High capacity card
			dev->d_dev->sd_data = sector;
		else
			dev->d_dev->sd_data = sector*512;
		dev->d_dev->sd_cmd  = (SDIO_CMD|SDIO_R1b|SDIO_MEM|SDIO_ERR)+18;
		// }}}

		// Read each sector
		// {{{
		for(unsigned s=0; s<count; s++) {
			// Wait until we have a block to read
			while((cmd = dev->d_dev->sd_cmd) & SDIO_BUSY)
				;

			// Send the next (or last) command
			// {{{
			if (0 != (cmd & SDIO_ERR)) {
				err = 1;
			} if (s +1 < count && !err) {
				// Immediately start the next read request
				dev->d_dev->sd_cmd  = SDIO_MEM + 18
						+ ((s&1) ? 0 : SDIO_FIFO);
			} else {
				// Send a STOP_TRANSMISSION request
				dev->d_dev->sd_data = 0;
				dev->d_dev->sd_cmd  = (SDIO_CMD | SDIO_R1b |SDIO_ERR) + 12;
			}
			// }}}

			// Now copy out the data that we've read
			// {{{
#ifdef	INCLUDE_DMA_CONTROLLER
			if (SDEXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
				_zip->z_dma.d_len = 512;
				_zip->z_dma.d_rd  = (s&1)
					? (char *)&dev->d_dev->sd_fifb
					: (char *)&dev->d_dev->sd_fifa;
				_zip->z_dma.d_wr  = (char *)buf;
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

		// Check the results of the STOP_TRANSMISSION request
		while((cmd = dev->d_dev->sd_cmd) & SDIO_BUSY)
			;

		if (err) {
			// If we had any read failures along the way, return
			// an error status
			RELEASE_MUTEX;
			return RES_ERROR;
		}

		// If the stop transmission command didn't receive a proper
		// response, return an error status
		if (cmd & SDIO_ERR) {
			RELEASE_MUTEX;
			return RES_ERROR;
		}

		// If the card has an error, return an error status
		card_stat = dev->d_dev->sd_data;
		RELEASE_MUTEX;
		if (card_stat & SDIO_R1ERR)
			return RES_ERROR;

		return RES_OK;
	}
}
// }}}

int	sdio_ioctl(SDIODRV *dev, char cmd, char *buf) {
	// {{{
	int		dstat;
	unsigned	vc;

	vc = dev->d_dev->sd_cmd;
	if (vc & SDIO_PRESENTN)
		return RES_ERROR;
	if (vc & SDIO_REMOVED)
		return	RES_NOTRDY;

	switch(cmd) {
	case CTRL_SYNC: {
			sdio_wait_while_busy(dev);
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
