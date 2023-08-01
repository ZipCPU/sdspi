////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	emmcdrv.c
// {{{
// Project:	SDIO SD-Card controller
//
// Purpose:	
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
#include <stdlib.h>
#include <stdint.h>
typedef	uint8_t  BYTE;
typedef	uint16_t WORD;
typedef	uint32_t DWORD, LBA_t, UINT;
#include <diskio.h>
#include "emmcdrv.h"

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

static	const int	EMMCINFO = 1, EMMCDEBUG=1, EXTDMA = 0;
// }}}

typedef	struct	EMMCDRV_S {
	EMMC		*d_dev;
	uint32_t	d_CID[4], d_OCR;
	char		d_SCR[8], d_CSD[16];
	uint16_t	d_RCA;
	uint32_t	d_sector_count, d_block_size;
} EMMCDRV;

static	const	uint32_t
		// Command bit enumerations
		SDIO_RNONE = 0x00000000,
		SDIO_R1    = 0x00000100,
		SDIO_R2    = 0x00000200,
		SDIO_R1b   = 0x00000300,
		SDIO_WRITE = 0x00000400,
		SDIO_MEM   = 0x00000800,
		SDIO_FIFO  = 0x00001000,
		SDIO_DMA   = 0x00002000,
		SDIO_CMDBUSY  = 0x00004000,
		SDIO_CARDBUSY = 0x00100000,
		SDIO_BUSY     = 0x00104800,
		SDIO_ERR      = 0x00008000,
		SDIO_REMOVED  = 0x00040000,
		SDIO_PRESENTN = 0x00080000,
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
		SPEED_SLOW   = SDIOCK_400KHZ,
		SPEED_DEFAULT= SDIOCK_DS,
		SPEED_FAST   = SDIOCK_HS,
		//
		SECTOR_16B   = 0x04000000,
		SECTOR_512B  = 0x09000000,
		//
		SDIO_CMD     = 0x00000040,
		SDIO_READREG  = SDIO_CMD | SDIO_R1 | SDIO_ERR,
		SDIO_READR2  = (SDIO_CMD | SDIO_R2),
		SDIO_WRITEBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_WRITE | SDIO_MEM) + 24,
		SDIO_READBLK  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
					| SDIO_MEM) + 17,
		SDIO_READCID  = (SDIO_CMD | SDIO_R2 | SDIO_ERR) + 2;

static	void	emmc_wait_while_busy(EMMCDRV *dev);
static	void	emmc_go_idle(EMMCDRV *dev);
static	void	emmc_all_send_cid(EMMCDRV *dev);
static	uint32_t emmc_send_rca(EMMCDRV *dev);
static	void	emmc_select_card(EMMCDRV *dev);	// CMD7
static	uint32_t emmc_send_if_cond(EMMCDRV *dev, uint32_t ifcond); // CMD8
static	uint32_t emmc_send_op_cond(EMMCDRV *dev, uint32_t opcond); // ACMD41
static	void	emmc_set_bus_width(EMMCDRV *dev, uint32_t width); // CMD6
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


void	emmc_wait_while_busy(EMMCDRV *dev) {
	// {{{

	// Busy wait implementation
	uint32_t	st;

	st = dev->d_dev->sd_cmd;
	while(st & SDIO_BUSY)
		st = dev->d_dev->sd_cmd;

	// Could also do interrupt
	// Could also do a system call and yield to the scheduler while waiting
}
// }}}

void	emmc_go_idle(EMMCDRV *dev) {				// CMD0
	// {{{
	dev->d_dev->sd_phy = SECTOR_512B | SDIOCK_400KHZ;
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_CMD | SDIO_RNONE | SDIO_ERR;

	emmc_wait_while_busy(dev);
}
// }}}

void	emmc_all_send_cid(EMMCDRV *dev) {	// CMD2
	// {{{
	if (EMMCDEBUG)	txstr("READ-CID\n");

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READR2)+2;

	emmc_wait_while_busy(dev);

	dev->d_CID[0] = dev->d_dev->sd_fifa;
	dev->d_CID[1] = dev->d_dev->sd_fifa;
	dev->d_CID[2] = dev->d_dev->sd_fifa;
	dev->d_CID[3] = dev->d_dev->sd_fifa;

	if (EMMCINFO)
		emmc_dump_cid(dev);
}
// }}}


void	emmc_dump_cid(EMMCDRV *dev) {
	// {{{
	txstr("CID: ");
	txhex(dev->d_CID[0]); txstr(":");
	txhex(dev->d_CID[1]); txstr(":");
	txhex(dev->d_CID[2]); txstr(":");
	txhex(dev->d_CID[3]); txstr("\n");

#ifdef	STDIO_DEBUG
	unsigned sn, md;

	sn = buf[2];
	sn = (sn << 8) | (buf[3] >> 24) & 0x0ff;
	md = (buf[3] >>  8) & 0x0fff;

	printf("CID:\n"
"\tManufacturer ID:  0x%02x\n"
"\tApplication ID:   %c%c\n"
"\tProduct Name:     %5c\n"
"\tProduct Revision: %x.%x\n"
"\tSerial Number:    0x%0x\n",
		(buf[0] >> 24)&0x0ff,
		(buf[0] >> 16)&0x0ff,
		(buf[0] >>  8)&0x0ff,
		(buf[0]      )&0x0ff,
		(buf[1] >> 24)&0x0ff,
		(buf[1] >> 16)&0x0ff,
		(buf[1] >>  8)&0x0ff,
		(buf[1]      )&0x0ff,
		(buf[2] >> 28)&0x00f,
		(buf[2] >> 24)&0x00f, sn);
	printf(
"\tYear of Man.:     %d\n"
"\tMonth of Man.:    %d\n",
		((md>>4)+2000), md&0x0f);
#endif
}
// }}}

uint32_t emmc_send_rca(EMMCDRV *dev) {				// CMD3
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = (SDIO_ERR|SDIO_READREG)+3;

	emmc_wait_while_busy(dev);
	return dev->d_RCA = (dev->d_dev->sd_data >> 16)&0x0ffff;
}
// }}}

void	emmc_select_card(EMMCDRV *dev) {			// CMD7
	// {{{
	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = SDIO_READREG+7;

	emmc_wait_while_busy(dev);
}
// }}}

uint32_t emmc_send_if_cond(EMMCDRV *dev, uint32_t ifcond) { // CMD8
	// {{{
	unsigned	c, r;

	dev->d_dev->sd_data = ifcond;
	dev->d_dev->sd_cmd = SDIO_READREG+8;

	emmc_wait_while_busy(dev);

	c = dev->d_dev->sd_cmd;
	r = dev->d_dev->sd_data;

	if (EMMCDEBUG && EMMCINFO) {
		txstr("CMD8:    SEND_IF_COND\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}

	return r;
}
// }}}

uint32_t emmc_send_op_cond(EMMCDRV *dev, uint32_t opcond) { // CMD1
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

void	emmc_set_bus_width(EMMCDRV *dev, uint32_t width) { // CMD6
	// {{{
	emmc_send_app_cmd(dev);

	dev->d_dev->sd_data = width;
	dev->d_dev->sd_cmd = SDIO_READREG+6;

	emmc_wait_while_busy(dev);

	if (EMMCDEBUG && EMMCINFO) {
		unsigned	c, r;

		c = dev->d_dev->sd_cmd;
		r = dev->d_dev->sd_data;

		txstr("CMD6:    SET_BUS_WIDTH\n");
		txstr("  Cmd:     "); txhex(c); txstr("\n");
		txstr("  Data:    "); txhex(r); txstr("\n");
	}
}
// }}}

void	emmc_send_app_cmd(EMMCDRV *dev) {  // CMD 55
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
		txstr("READ-OCR: OCR = "); txhex(dev->d_OCR); txstr("\r\n");
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

void emmc_read_scr(EMMCDRV *dev) {	  // ACMD 51
	// {{{
	uint32_t	phy = dev->d_dev->sd_phy;

	phy &= 0xf0ffffff;
	phy |= (3 << 24);	// 64 bits = 8 bytes = 2^3 bytes
	dev->d_dev->sd_phy = phy;

	emmc_send_app_cmd(dev);

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_READREG+51;

	emmc_wait_while_busy(dev);

	if (EMMCDEBUG && EMMCINFO) {
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
		if (EMMCINFO) { txhex(uv); if (k < 4) txstr(":"); }
		dev->d_SCR[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 0] = uv;
	} if (EMMCINFO) txstr("\n");

	phy &= 0xf0ffffff;
	phy |= (9 << 24);
	dev->d_dev->sd_phy = phy;


	if (EMMCINFO)
		emmc_dump_scr(dev);
}
// }}}

void	emmc_dump_scr(EMMCDRV *dev) {
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
	}
	// }}}

	for(int k=0; k<16; k+= sizeof(uint32_t)) {
		unsigned	uv;

		uv = dev->d_dev->sd_fifa;
		if (EMMCINFO) { txhex(uv); if (k < 12) txstr(":"); }
		dev->d_CSD[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_CSD[k + 0] = uv;
	}

	unsigned	C_SIZE, READ_BL_LEN, CSD_STRUCTURE;

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

		if (EMMCINFO) {
			txstr("  TAAC          : "); txhex(dev->d_CSD[1]); txstr("\r\n");
			txstr("  NSAC          : "); txhex(dev->d_CSD[2]); txstr("\r\n");
			txstr("  TRAN_SPEED        : "); txhex(dev->d_CSD[3]); txstr("\r\n");
			txstr("  CCC               : "); txhex((dev->d_CSD[4]<<4)|((dev->d_CSD[10]&0x0f0)>>4)); txstr("\r\n");
			txstr("  READ_BL_LEN       : "); txhex(READ_BL_LEN); txstr("\r\n");
			txstr("  READ_BL_PARTIAL   : "); txhex((dev->d_CSD[6] & 0x80) ? 1:0); txstr("\r\n");
			txstr("  WRITE_BLK_MISALIGN: "); txhex((dev->d_CSD[6] & 0x40) ? 1:0); txstr("\r\n");
			txstr("  READ_BLK_MISALIGN : "); txhex((dev->d_CSD[6] & 0x20) ? 1:0); txstr("\r\n");
			txstr("  DSR_IMP           : "); txhex((dev->d_CSD[6] & 0x10) ? 1:0); txstr("\r\n");
			txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\r\n");
			txstr("  VDD_R_CURR_MIN    : "); txhex((dev->d_CSD[8]>>3)&0x07); txstr("\r\n");
			txstr("  VDD_R_CURR_MAX    : "); txhex(dev->d_CSD[8]&0x07); txstr("\r\n");
			txstr("  VDD_W_CURR_MIN    : "); txhex((dev->d_CSD[9]>>5)&0x07); txstr("\r\n");
			txstr("  VDD_W_CURR_MAX    : "); txhex((dev->d_CSD[9]>>2)&0x07); txstr("\r\n");
			txstr("  C_SIZE_MULT       : "); txhex(C_SIZE_MULT); txstr("\r\n");
			txstr("  ERASE_BLK_EN      : "); txhex((dev->d_CSD[10] & 0x40)?1:0); txstr("\r\n");
			txstr("  SECTOR_SIZE       : "); txhex(SECTOR_SIZE);
				txstr(" (");
				txdecimal((SECTOR_SIZE+1)*(1<<WRITE_BL_LEN));
				txstr(" bytes)\r\n");
			txstr("  WP_GRP_SIZE       : "); txhex(dev->d_CSD[11]&0x07f); txstr("\r\n");
			txstr("  WP_GRP_ENABLE     : "); txhex((dev->d_CSD[12]&0x080)?1:0); txstr("\r\n");
			txstr("  R2W_FACTOR        : "); txhex((dev->d_CSD[12]>>2)&7); txstr("\r\n");
			txstr("  WRITE_BL_LEN      : "); txhex(WRITE_BL_LEN);
			if (WRITE_BL_LEN < 9 || WRITE_BL_LEN > 11)
				txstr(" (Reserved)\r\n");
			else {
				txstr(" (");
				txdecimal(1<<WRITE_BL_LEN); txstr(" bytes)\r\n");
			}
			txstr("  WRITE_BL_PARTIAL  : "); txhex((dev->d_CSD[13]&0x20)?1:0); txstr("\r\n");
			txstr("  FILE_FORMAT_GRP   : "); txhex(FILE_FORMAT_GRP); txstr("\r\n");
			txstr("  COPY              : "); txhex((dev->d_CSD[14]&0x40)?1:0); txstr("\r\n");
			txstr("  PERM_WRITE_PROTECT: "); txhex((dev->d_CSD[14]&0x20)?1:0); txstr("\r\n");
			txstr("  TMP_WRITE_PROTECT : "); txhex((dev->d_CSD[14]&0x10)?1:0); txstr("\r\n");
			txstr("  FILE_FORMAT       : "); txhex(FILE_FORMAT);
			if (0 == FILE_FORMAT_GRP) {
				if (0 == FILE_FORMAT)
					txstr("  (Has parition tbl)\r\n");
				else if (1 == FILE_FORMAT)
					txstr("  (DOS FAT w/ boot sector, no partition)\r\n");
				else if (2 == FILE_FORMAT)
					txstr("  (Universal file format)\r\n");
				else
					txstr("  (Others/unknown)\r\n");
			} else
				txstr("  (Reserved)\r\n");
			txstr("  Size  = "); txdecimal(BLOCKNR); txstr(" blocks of ");
			txdecimal(BLOCK_LEN); txstr(" bytes each\r\n");
		}
		// }}}
	} else if (1 == ((dev->d_CSD[15]>>6)&3)) {
		// {{{
		READ_BL_LEN = 9;
		C_SIZE = ((dev->d_CSD[7] & 0x03f)<<16)
				|((dev->d_CSD[8] & 0x0ff)<<8)
				|(dev->d_CSD[9] & 0x0ff);
		dev->d_sector_count = (C_SIZE+1) * 1024;
		dev->d_block_size = 512;

		if (EMMCINFO) {
			txstr("  CCC               : "); txhex((dev->d_CSD[4]<<4)|((dev->d_CSD[10]&0x0f0)>>4)); txstr("\r\n");
			txstr("  DSR_IMP           : "); txhex((dev->d_CSD[6] & 0x10) ? 1:0); txstr("\r\n");
			txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\r\n");
			txstr("  COPY              : "); txhex((dev->d_CSD[14]&0x40)?1:0); txstr("\r\n");
			txstr("  PERM_WRITE_PROTECT: "); txhex((dev->d_CSD[14]&0x20)?1:0); txstr("\r\n");
			txstr("  TMP_WRITE_PROTECT : "); txhex((dev->d_CSD[14]&0x10)?1:0); txstr("\r\n");
			txstr("  Size  = "); txdecimal((C_SIZE+1) * 512); txstr(" kB\r\n");
		}
		// }}}
	} else {
		txstr("ERROR: Unknown CSD type\r\n");
		dev->d_sector_count = 0;
		dev->d_block_size   = 0;
	}
}
// }}}

// Get and Dump R2
// {{{
void emmc_dump_r1(const unsigned rv) {
	if (EMMCDEBUG) {
		txstr("SDIO R1 Decode:  "); txhex(rv);
		if (rv & 0x80000000)
			txstr("\r\n  OUT_OF_RANGE");
		if (rv & 0x40000000)
			txstr("\r\n  ADDRESS_ERROR");
		if (rv & 0x20000000)
			txstr("\r\n  BLOCK_LEN_ERROR");
		if (rv & 0x10000000)
			txstr("\r\n  ERASE_SEQ_ERROR");
		if (rv & 0x08000000)
			txstr("\r\n  ERASE_PARAM");
		if (rv & 0x04000000)
			txstr("\r\n  WP_VIOLATION");
		if (rv & 0x02000000)
			txstr("\r\n  CARD_IS_LOCKED");
		if (rv & 0x01000000)
			txstr("\r\n  LOCK_UNLOCK_FAILED");
		if (rv & 0x00800000)
			txstr("\r\n  COM_CRC_ERROR");
		if (rv & 0x00400000)
			txstr("\r\n  ILLEGAL_COMMAND");
		if (rv & 0x00200000)
			txstr("\r\n  CARD_ECC_FAILED");
		if (rv & 0x00100000)
			txstr("\r\n  CC_ERROR (Internal card controller err)");
		if (rv & 0x00080000)
			txstr("\r\n  ERROR (General or unknown error)");
		if (rv & 0x00010000)
			txstr("\r\n  CSD_OVERWRITE");
		if (rv & 0x00008000)
			txstr("\r\n  WP_ERASE_SKIP");
		if (rv & 0x00004000)
			txstr("\r\n  CARD_ECC_DISABLED");
		if (rv & 0x00002000)
			txstr("\r\n  ERASE_RESET");
		switch((rv >> 9)&0x0f) {
		case 0: txstr("\r\n  STATE: idle");  break;
		case 1: txstr("\r\n  STATE: ready"); break;
		case 2: txstr("\r\n  STATE: ident"); break;
		case 3: txstr("\r\n  STATE: stdby"); break;
		case 4: txstr("\r\n  STATE: tran");  break;
		case 5: txstr("\r\n  STATE: data");  break;
		case 6: txstr("\r\n  STATE: rcv");   break;
		case 7: txstr("\r\n  STATE: prg");   break;
		case 8: txstr("\r\n  STATE: dis");   break;
		case 15: txstr("\r\n  STATE: (reserved for I/O mode)"); break;
		default: txstr("\r\n  STATE: (reserved)"); break;
		}
		if (rv & 0x00000080)
			txstr("\r\n  READY_FOR_DATA");
		if (rv & 0x00000040)
			txstr("\r\n  FX_EVENT");
		if (rv & 0x00000020)
			txstr("\r\n  APP_CMD");
		if (rv & 0x00000008)
			txstr("\r\n  AKE_SEQ_ERROR");
		txstr("\r\n");
		// emmc_dump_r1(uv >> 8);
	}
}

unsigned emmc_get_r1(EMMCDRV *dev) {	// CMD13=send_status
	unsigned	vc, vd;

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd  = SDIO_READREG + 13;

	emmc_wait_while_busy(dev);

	txhex(vc = dev->d_dev->sd_cmd);
	txstr(" : ");
	txhex(vd = dev->d_dev->sd_data);
	txstr("\r\n");

	emmc_dump_r1(vd);
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
	unsigned	vc, vd;

	if (9 != ((dev->d_dev->sd_phy >> 24)&0x0f)) {
		uint32_t	phy = dev->d_dev->sd_phy;
		phy &= 0xf0ffffff;
		phy |= (9 << 24);
	}

#ifndef	INCLUDE_DMA_CONTROLLER_NOT
	if (EXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
		_zip->z_dma.d_len = 512;
		_zip->z_dma.d_rd  = (char *)buf;
		_zip->z_dma.d_wr  = &dev->d_dev->sd_fifa;
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

	emmc_wait_while_busy(dev);

	vc = dev->d_dev->sd_cmd;
	vd = dev->d_dev->sd_data;

	if (EMMCDEBUG) {
		if (vc & SDIO_ERR) {
			txstr("SDIO ERR: SDIO write response = ");
			txhex(vc);
			txstr("\n");
		}

		if (EMMCINFO)
			emmc_get_r1(dev);
	} if (vc & SDIO_ERR) {
		dev->d_dev->sd_cmd = SDIO_ERR;
		return -1;
	} return 0;
}
// }}}

int	emmc_read_block(EMMCDRV *dev, uint32_t sector, uint32_t *buf){// CMD 17
	// {{{
	if (EMMCDEBUG) {
		txstr("SDIO-READ: ");
		txhex(sector);
		txstr("\n");
	}

	if (dev->d_dev->sd_cmd & SDIO_REMOVED) {
		txstr("SDIO ERR: SD-Card was removed\n");
		return -1;
	}

	if (9 != ((dev->d_dev->sd_phy >> 24)&0x0f)) {
		uint32_t	phy = dev->d_dev->sd_phy;
		phy &= 0xf0ffffff;
		phy |= (9 << 24);
	}

	dev->d_dev->sd_data = sector;
	dev->d_dev->sd_cmd = SDIO_READBLK;

	emmc_wait_while_busy(dev);

#ifndef	INCLUDE_DMA_CONTROLLER_NOT
	if (SDUSEDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
		_zip->z_dma.d_len = 512;
		_zip->z_dma.d_rd  = (char *)&sdcard->sd_fifo[0];
		_zip->z_dma.d_wr  = buf;
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

	if (dev->d_dev->sd_cmd & (SDIO_ERR | SDIO_REMOVED)) {
		txstr("SDIO ERR: Read\r\n");
		return -1;
	} return 0;
}
// }}}

EMMCDRV *emmc_init(EMMC *dev) {
	// {{{
	unsigned	ifcond, op_cond;
	EMMCDRV	*dv = (EMMCDRV *)malloc(sizeof(EMMCDRV));
	dv->d_dev = dev;
	dv->d_OCR = 0;
	dv->d_RCA = 0;
	dv->d_sector_count = 0;
	dv->d_block_size   = 0;

	dv->d_dev->sd_phy = SDIOCK_400KHZ | SECTOR_512B;

	emmc_go_idle(dv);

	do {
		op_cond = emmc_send_op_cond(dv,0x40ff8080);
	} while(0 == (op_cond & 0x80000000));

	if (EMMCINFO || ((op_cond & 0x3ffffff) != 0x0ff8080)) {
		emmc_dump_ocr(dv);
	}

	emmc_all_send_cid(dv);		// CMD2

	emmc_send_rca(dv);

	{
		unsigned phy = dv->d_dev->sd_phy;

		// We can move up to 25MHz with no further hassles.
		// Going faster requires coordination with the eMMC chip
		phy &= ~0x0ff;
		phy |= SDIOCK_25MHZ | SDIO_PUSHPULL;
		dv->d_dev->sd_phy = phy;
	}

	emmc_read_csd(dv);		// CMD9, Read card specific data

	// CMD4 (SET_DSR), CMD9 (SEND_CSD), CMD10 (SEND_CID), CMD39(FAST_IO) ?
	// CMD40 (GO_IRQ_STATE)
	// CMD6 SWITCH command set ... when to issue this?

	emmc_select_card(dv);		// Move to transfer state

	// emmc_read_ext_csd(dv);	// Read extended CSD register, 512Bytes

	return	dv;
}
// }}}

int	emmc_write(EMMCDRV *dev, const unsigned sector,
			const unsigned count, const char *buf) {
	// {{{
	unsigned	st;

	for(unsigned k=0; k<count; k++) {
		st = emmc_write_block(dev, sector+k, (uint32_t *)(&buf[k*512]));
		if (0 != st) {
			return RES_ERROR;
		}
	} return RES_OK;
}
// }}}

int	emmc_read(EMMCDRV *dev, const unsigned sector,
				const unsigned count, char *buf) {
	// {{{
	unsigned	st = 0;

	for(unsigned k=0; k<count; k++) {
		st = emmc_read_block(dev, sector+k, (uint32_t *)(&buf[k*512]));
		if (0 != st) {
			return RES_ERROR;
		}
	} return RES_OK;
}
// }}}

int	emmc_ioctl(EMMCDRV *dev, char cmd, char *buf) {
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
			emmc_wait_while_busy(dev);
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
