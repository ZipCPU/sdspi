////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sdcard.h
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	Function header definitions and some constant definitions for
//		working with the SDSPI controller.  Other constant definitions
//	are currently provided by the AutoFPGA script and placed into board.h.
//	You can (currently) find alternate definitions for these in the
//	autotest_tb.cpp file.
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
//
////////////////////////////////////////////////////////////////////////////////
//
// }}}
#ifndef	SDCARD_H
#define	SDCARD_H

#include "board.h"

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

extern	int	sdcard_err;
extern	int	sdcard_ocr;
extern	char	sdcard_csd[16];
extern	char	sdcard_cid[16];

#ifdef	_BOARD_HAS_SDSPI

extern	int	sdcard_read_ocr(void);
extern	int	sdcard_read_scr(unsigned *scr);
extern	int	sdcard_read_csd(char *csd);
extern	int	sdcard_read_cid(char *cid);
extern	int	sdcard_init(void);
extern	int	sdcard_read(int sector, char *buf);
extern	int	sdcard_write(int sector, const char *buf);

#else
// _BOARD_HAS_SDSPI is defined by AutoFPGA when including the SDSPI controller
// into a design, according to the SDSPI script
#error "No _BOARD_HAS_SDSPI defined"
#endif	// _BOARD_HAS_SDSPI
#endif	// SDCARD_H
