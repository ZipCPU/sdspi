////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdiodrv.c
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	
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

#define	txstr(A)		puts(A)
#define	txhex(A)		printf("%08x", A)
#define	txdecimal(A)		printf("%d", A)
#else
// extern	void	txstr(const char *);
// extern	void	txhex(unsigned);
// extern	void	txdecimal(int);
#endif

static	const int	SDINFO = 0, SDDEBUG=0;
// }}}

/*
typedef	struct SDIO_S {
	volatile uint32_t	sd_cmd, sd_data, sd_fifa, sd_fifb, sd_phy;
} SDIO;
*/

typedef	struct	SDIODRV_S {
	SDIO		*d_dev;
	uint32_t	d_CID[4], d_OCR;
	char		d_SCR[8], d_CSD[16];
	uint16_t	d_RCA;
	uint32_t	d_sector_count, d_block_size;
} SDIODRV;

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
		SPEED_SLOW   = SDIOCK_100KHZ,
		SPEED_DEFAULT= SDIOCK_DS,
		SPEED_FAST   = SDIOCK_HS,
		//
		SECTOR_16B   = 0x04000000,
		SECTOR_512B  = 0x09000000,
		//
		SDIO_CMD     = 0x00000040,
		SDIO_READREG  = SDIO_CMD | SDIO_R1 | SDIO_ERR,
		SDIO_WRITEBLK = (SDIO_CMD | SDIO_R1 | SDIO_ERR
				| SDIO_WRITE | SDIO_MEM) + 24,
		SDIO_READBLK  = (SDIO_CMD | SDIO_R1 | SDIO_ERR
					| SDIO_MEM) + 17,
		SDIO_READCID  = (SDIO_CMD | SDIO_R2 | SDIO_ERR) + 2;

static	void	sdio_wait_while_busy(SDIODRV *dev);
static	void	sdio_go_idle(SDIODRV *dev);
static	void	sdio_all_send_cid(SDIODRV *dev, uint32_t *buf);
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

	// Busy wait implementation
	uint32_t	st;

	st = dev->d_dev->sd_cmd;
	while(st & SDIO_BUSY)
		st = dev->d_dev->sd_cmd;

	// Could also do interrupt
	// Could also do a system call and yield to the scheduler while waiting
}
// }}}

void	sdio_go_idle(SDIODRV *dev) {				// CMD0
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_CMD | SDIO_RNONE | SDIO_ERR;

	sdio_wait_while_busy(dev);
}
// }}}

void	sdio_all_send_cid(SDIODRV *dev, uint32_t *buf) {	// CMD2
	// {{{
	if (SDDEBUG)	txstr("READ-CID\n");

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_READCID;

	sdio_wait_while_busy(dev);

	if (buf) {
		buf[0] = dev->d_dev->sd_fifa;
		buf[1] = dev->d_dev->sd_fifa;
		buf[2] = dev->d_dev->sd_fifa;
		buf[3] = dev->d_dev->sd_fifa;

		if (SDINFO) {
			txstr("CID: ");
			txhex(buf[0]); txstr(":");
			txhex(buf[1]); txstr(":");
			txhex(buf[2]); txstr(":");
			txhex(buf[3]); txstr(":");

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
	}
}
// }}}

uint32_t sdio_send_rca(SDIODRV *dev) {				// CMD3
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_READREG+3;

	sdio_wait_while_busy(dev);
	return dev->d_RCA = (dev->d_dev->sd_data >> 16)&0x0ffff;
}
// }}}

void	sdio_select_card(SDIODRV *dev) {			// CMD7
	// {{{
	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = SDIO_READREG+7;

	sdio_wait_while_busy(dev);
}
// }}}

uint32_t sdio_send_if_cond(SDIODRV *dev, uint32_t ifcond) { // CMD8
	// {{{
	dev->d_dev->sd_data = ifcond;
	dev->d_dev->sd_cmd = SDIO_READREG+8;

	sdio_wait_while_busy(dev);
	return	dev->d_dev->sd_data;
}
// }}}

uint32_t sdio_send_op_cond(SDIODRV *dev, uint32_t opcond) { // ACMD41
	// {{{
	sdio_send_app_cmd(dev);

	dev->d_dev->sd_data = opcond;
	dev->d_dev->sd_cmd = SDIO_READREG+41;

	sdio_wait_while_busy(dev);
	dev->d_OCR = dev->d_dev->sd_data;
	return	dev->d_OCR;
}
// }}}

void	sdio_set_bus_width(SDIODRV *dev, uint32_t width) { // CMD6
	// {{{
	sdio_send_app_cmd(dev);

	dev->d_dev->sd_data = width;
	dev->d_dev->sd_cmd = SDIO_READREG+6;

	sdio_wait_while_busy(dev);
}
// }}}

void	sdio_send_app_cmd(SDIODRV *dev) {  // CMD 55
	// {{{
	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_READREG+55;

	sdio_wait_while_busy(dev);
}
// }}}

void	sdio_dump_ocr(SDIODRV *dev) {
	// {{{
	if (SDINFO) {
		txstr("READ-OCR: OCR = "); txhex(dev->d_OCR); txstr("\r\n");
		if (dev->d_OCR & 0x80000000)
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
	phy |= (1 << 24);
	dev->d_dev->sd_phy = phy;

	sdio_send_app_cmd(dev);

	dev->d_dev->sd_data = 0;
	dev->d_dev->sd_cmd = SDIO_READREG+51;

	sdio_wait_while_busy(dev);

	for(int k=0; k<8; k+= sizeof(uint32_t)) {
		unsigned	uv;

		uv = dev->d_dev->sd_fifa;
		dev->d_SCR[k + 3] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 2] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 1] = uv & 0x0ff; uv >>= 8;
		dev->d_SCR[k + 0] = uv;
	}

	phy &= 0xf0ffffff;
	phy |= (1 << 24);
	dev->d_dev->sd_phy = phy;


	if (SDINFO)
		sdio_dump_scr(dev);
}

void	sdio_dump_scr(SDIODRV *dev) {
	txstr("  SCR_STRUCTURE: "); txhex((dev->d_SCR[7] >> 4)& 0x0f); txstr("\r\n");
	txstr("  SD_SPEC      : "); txhex(dev->d_SCR[7]& 0x0f); txstr("\r\n");
	txstr("  STATAFTERERAS: "); txhex((dev->d_SCR[6]&0x80)?1:0); txstr("\r\n");
	txstr("  SD_SECURITY  : "); txhex((dev->d_SCR[6]>>4) & 0x07); txstr("\r\n");
	txstr("  SD_BUS_WIDTHS: "); txhex(dev->d_SCR[6] & 0x0f); txstr("\r\n");
	txstr("  EX_SECURITY  : "); txhex((dev->d_SCR[5]>>3) & 0x07); txstr("\r\n");
	txstr("  SD_SPEC3     : "); txhex((dev->d_SCR[5] & 0x80) ? 1:0); txstr("\r\n");
	txstr("  SD_SPEC4     : "); txhex((dev->d_SCR[5] & 0x04) ? 1:0); txstr("\r\n");
	txstr("  SD_SPECX     : "); txhex(((dev->d_SCR[3]>>6)&0x03)
					| ((dev->d_SCR[4]&0x03)<<2));; txstr("\r\n");
	txstr("  CMD_SUPPORT  : "); txhex(dev->d_SCR[4] & 0x0f); txstr("\r\n");
}
// }}}

void sdio_read_csd(SDIODRV *dev) {	  // CMD 9
	// {{{
	if (SDDEBUG)	txstr("READ-CID\n");

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd = (SDIO_CMD | SDIO_R2 | SDIO_ERR)+9;

	sdio_wait_while_busy(dev);

	if (SDINFO)
		txstr("CSD ");

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

	CSD_STRUCTURE = (dev->d_CSD[15]>>6)&3;

	if (SDINFO) {
		txstr("\r\n");
		txstr("  CSD_STRUCTURE : "); txhex(CSD_STRUCTURE); txstr("\r\n");
	} if (0 == CSD_STRUCTURE) {
		// {{{
		unsigned	ERASE_BLK_EN, C_SIZE_MULT, BLOCK_LEN, MULT,
				BLOCKNR, FILE_FORMAT_GRP, FILE_FORMAT,
				WRITE_BL_LEN, SECTOR_SIZE;

		READ_BL_LEN = dev->d_CSD[10] & 0x0f;
		BLOCK_LEN = (READ_BL_LEN < 12) ? (1<<READ_BL_LEN) : READ_BL_LEN;
		C_SIZE = ((dev->d_CSD[9]&0x3)<<10)
				|((dev->d_CSD[8]&0x0ff)<<2)
				|((dev->d_CSD[7]>>6)&0x03);
		C_SIZE_MULT = ((dev->d_CSD[6]&3)<<2)|((dev->d_CSD[5]>>7)&1);
		MULT = 1<<(C_SIZE_MULT+2);
		BLOCKNR = MULT * (C_SIZE+1);
		SECTOR_SIZE = ((dev->d_CSD[5]&0x03f)<<2)|((dev->d_CSD[4]>>7)&1);
		WRITE_BL_LEN = ((dev->d_CSD[3]&3)<<2)|((dev->d_CSD[2]>>6)&3);
		FILE_FORMAT_GRP = (dev->d_CSD[1]&0x80)?1:0;
		FILE_FORMAT = dev->d_CSD[1]&3;
		ERASE_BLK_EN = (dev->d_CSD[5] & 0x40)?1:0;

		dev->d_sector_count = BLOCKNR / 512;
		if (ERASE_BLK_EN)
			dev->d_block_size = 512;
		else
			dev->d_block_size = SECTOR_SIZE;

		if (SDINFO) {
			txstr("  TAAC          : "); txhex(dev->d_CSD[14]); txstr("\r\n");
			txstr("  NSAC          : "); txhex(dev->d_CSD[13]); txstr("\r\n");
			txstr("  TRAN_SPEED        : "); txhex(dev->d_CSD[12]); txstr("\r\n");
			txstr("  CCC               : "); txhex((dev->d_CSD[11]<<4)|((dev->d_CSD[10]&0x0f0)>>4)); txstr("\r\n");
			txstr("  READ_BL_LEN       : "); txhex(READ_BL_LEN); txstr("\r\n");
			txstr("  READ_BL_PARTIAL   : "); txhex((dev->d_CSD[9] & 0x80) ? 1:0); txstr("\r\n");
			txstr("  WRITE_BLK_MISALIGN: "); txhex((dev->d_CSD[9] & 0x40) ? 1:0); txstr("\r\n");
			txstr("  READ_BLK_MISALIGN : "); txhex((dev->d_CSD[9] & 0x20) ? 1:0); txstr("\r\n");
			txstr("  DSR_IMP           : "); txhex((dev->d_CSD[9] & 0x10) ? 1:0); txstr("\r\n");
			txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\r\n");
			txstr("  VDD_R_CURR_MIN    : "); txhex((dev->d_CSD[7]>>3)&0x07); txstr("\r\n");
			txstr("  VDD_R_CURR_MAX    : "); txhex(dev->d_CSD[7]&0x07); txstr("\r\n");
			txstr("  VDD_W_CURR_MIN    : "); txhex((dev->d_CSD[6]>>5)&0x07); txstr("\r\n");
			txstr("  VDD_W_CURR_MAX    : "); txhex((dev->d_CSD[6]>>2)&0x07); txstr("\r\n");
			txstr("  C_SIZE_MULT       : "); txhex(C_SIZE_MULT); txstr("\r\n");
			txstr("  ERASE_BLK_EN      : "); txhex((dev->d_CSD[5] & 0x40)?1:0); txstr("\r\n");
			txstr("  SECTOR_SIZE       : "); txhex(SECTOR_SIZE);
				txstr(" (");
				txdecimal((SECTOR_SIZE+1)*(1<<WRITE_BL_LEN));
				txstr(" bytes)\r\n");
			txstr("  WP_GRP_SIZE       : "); txhex(dev->d_CSD[4]&0x07f); txstr("\r\n");
			txstr("  WP_GRP_ENABLE     : "); txhex((dev->d_CSD[3]&0x080)?1:0); txstr("\r\n");
			txstr("  R2W_FACTOR        : "); txhex((dev->d_CSD[3]>>2)&7); txstr("\r\n");
			txstr("  WRITE_BL_LEN      : "); txhex(WRITE_BL_LEN);
			if (WRITE_BL_LEN < 9 || WRITE_BL_LEN > 11)
				txstr(" (Reserved)\r\n");
			else {
				txstr(" (");
				txdecimal(1<<WRITE_BL_LEN); txstr(" bytes)\r\n");
			}
			txstr("  WRITE_BL_PARTIAL  : "); txhex((dev->d_CSD[2]&0x20)?1:0); txstr("\r\n");
			txstr("  FILE_FORMAT_GRP   : "); txhex(FILE_FORMAT_GRP); txstr("\r\n");
			txstr("  COPY              : "); txhex((dev->d_CSD[1]&0x40)?1:0); txstr("\r\n");
			txstr("  PERM_WRITE_PROTECT: "); txhex((dev->d_CSD[1]&0x20)?1:0); txstr("\r\n");
			txstr("  TMP_WRITE_PROTECT : "); txhex((dev->d_CSD[1]&0x10)?1:0); txstr("\r\n");
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
		C_SIZE = ((dev->d_CSD[8] & 0x03f)<<16)
				|((dev->d_CSD[7] & 0x0ff)<<8)
				|(dev->d_CSD[6] & 0x0ff);
		dev->d_sector_count = (C_SIZE+1) * 1024;
		dev->d_block_size = 512;

		if (SDINFO) {
			txstr("  CCC               : "); txhex((dev->d_CSD[11]<<4)|((dev->d_CSD[10]&0x0f0)>>4)); txstr("\r\n");
			txstr("  DSR_IMP           : "); txhex((dev->d_CSD[9] & 0x10) ? 1:0); txstr("\r\n");
			txstr("  C_SIZE            : "); txhex(C_SIZE); txstr("\r\n");
			txstr("  COPY              : "); txhex((dev->d_CSD[1]&0x40)?1:0); txstr("\r\n");
			txstr("  PERM_WRITE_PROTECT: "); txhex((dev->d_CSD[1]&0x20)?1:0); txstr("\r\n");
			txstr("  TMP_WRITE_PROTECT : "); txhex((dev->d_CSD[1]&0x10)?1:0); txstr("\r\n");
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
void sdio_dump_r1(const unsigned rv) {
	if (SDDEBUG) {
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
		// sdio_dump_r1(uv >> 8);
	}
}

unsigned sdio_get_r1(SDIODRV *dev) {	// CMD13=send_status
	unsigned	vc, vd;

	dev->d_dev->sd_data = dev->d_RCA << 16;
	dev->d_dev->sd_cmd  = SDIO_READREG + 13;

	sdio_wait_while_busy(dev);

	txhex(vc = dev->d_dev->sd_cmd);
	txstr(" : ");
	txhex(vd = dev->d_dev->sd_data);
	txstr("\r\n");

	sdio_dump_r1(vd);
}
// }}}

static void sdio_dump_sector(const unsigned *ubuf) {
	// {{{
	for(int j=0; j<512/4; j++) {
		txhex(ubuf[j]);
		if (7 == (j&7))
			txstr("\r\n");
		else
			txstr(" ");
	} txstr("\r\n");
}
// }}}

int	sdio_write_block(SDIODRV *dev, uint32_t sector, uint32_t *buf){// CMD 24
	// {{{
	unsigned	vc, vd;

	if (9 != ((dev->d_dev->sd_phy >> 24)&0x0f)) {
		uint32_t	phy = dev->d_dev->sd_phy;
		phy &= 0xf0ffffff;
		phy |= (9 << 24);
	}

#ifdef	INCLUDE_DMA_CONTROLLER
	if (SDEXTDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
		_zip->z_dma.d_len = 512/sizeof(char);
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

	sdio_wait_while_busy(dev);

	vc = dev->d_dev->sd_cmd;
	vd = dev->d_dev->sd_data;

	if (SDDEBUG) {
		if (vc & SDIO_ERR) {
			txstr("SDIO ERR: SDIO write response = ");
			txhex(vc);
			txstr("\r\n");
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
	if (SDDEBUG) {
		txstr("SDIO-READ: ");
		txhex(sector);
		txstr("\r\n");
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

	sdio_wait_while_busy(dev);

#ifdef	INCLUDE_DMA_CONTROLLER
	if (SDUSEDMA && (0 == (_zip->z_dma.d_ctrl & DMA_BUSY))) {
		_zip->z_dma.d_len = 512/sizeof(char);
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

	if (SDDEBUG && SDINFO)
		sdio_dump_sector(buf);

	if (dev->d_dev->sd_cmd & (SDIO_ERR | SDIO_REMOVED)) {
		txstr("SDIO ERR: Read\r\n");
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

	dv->d_dev->sd_phy = SPEED_SLOW | SECTOR_512B;

	sdio_go_idle(dv);
	ifcond = sdio_send_if_cond(dv,0x01a5);
	if (0x08000 == (dv->d_dev->sd_cmd & 0x038000)) {
		do {
			op_cond = 0;
			op_cond = sdio_send_op_cond(dv, op_cond);
		} while(op_cond & 0x80000000);
	} else {
		if (0xa5 != ifcond & 0x0ff) {
			txstr("SDIO ERROR: IFCOND returned ");
			txhex(ifcond); txstr("\r\n");
			free(dv);
			return NULL;
		}

		do {
			op_cond = 0x40000000;
			op_cond = sdio_send_op_cond(dv, op_cond);
		} while(op_cond & 0x80000000);
	} if (SDINFO)
		sdio_dump_ocr(dv);

	sdio_all_send_cid(dv, (uint32_t *)&dv->d_CID);

	sdio_send_rca(dv);

	sdio_select_card(dv);

	// SEND_SCR
	sdio_read_scr(dv);

	// LOCK_UNLOCK ?
	// SET_BUS_WIDTH
	if ((dv->d_SCR[6] & 0x04) && (0 == (dv->d_dev->sd_phy & SDIO_WBEST))) {
		dv->d_dev->sd_phy |= SDIO_WBEST;
		if (0 != (dv->d_dev->sd_phy & SDIO_WBEST)) {
			// Set a 4-bit bus width via ACMD6
			sdio_set_bus_width(dv, 2);
			dv->d_dev->sd_phy |= SDIO_W4;
		}
	}

	// Select drive strength?
	// CMD19, tuning block to determine sample point?
	dv->d_dev->sd_phy = SECTOR_512B | SDIOCK_DS;

	return	dv;
}
// }}}

int	sdio_write(SDIODRV *dev, const unsigned sector,
			const unsigned count, const char *buf) {
	// {{{
	unsigned	st;

	for(unsigned k=0; k<count; k++) {
		st = sdio_write_block(dev, sector+k, (uint32_t *)(&buf[k*512]));
		if (0 != st) {
			return RES_ERROR;
		}
	} return RES_OK;
}
// }}}

int	sdio_read(SDIODRV *dev, const unsigned sector,
				const unsigned count, char *buf) {
	// {{{
	unsigned	st = 0;

	for(unsigned k=0; k<count; k++) {
		st = sdio_read_block(dev, sector+k, (uint32_t *)(&buf[k*512]));
		if (0 != st) {
			return RES_ERROR;
		}
	} return RES_OK;
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
	}

	return	RES_PARERR;
}
// }}}
