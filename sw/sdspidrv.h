////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sw/sdspidrv.h
// {{{
// Project:	SD-Card controller
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
#ifndef	SDSPIDRV_H
#define	SDSPIDRV_H

extern	int	sdcard_err;

struct	SDSPI_S;
struct	SDSPIDRV_S;

extern	struct SDSPIDRV_S *sdspi_init(struct SDSPI_S *);
extern	int	sdspi_read(struct SDSPIDRV_S *, unsigned sector, unsigned count, char *buf);
extern	int	sdspi_write(struct SDSPIDRV_S *, unsigned sector, unsigned count, const char *buf);
extern	int	sdspi_ioctl(struct SDSPIDRV_S *, char cmd, char *buf);

#endif	// SDSPIDRV_H
