////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sw/emmcdrv.h
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
// }}}
#ifndef	EMMCDRV_H
#define	EMMCDRV_H
#include <stdint.h>

typedef	struct EMMC_S {
	volatile uint32_t	sd_cmd, sd_data, sd_fifa, sd_fifb, sd_phy;
// #if (sizeof(void *) <= 4) && !defined(__LITTLE_ENDIAN__)
	volatile uint32_t	sd_unused;
// #endif
	volatile void		*sd_dma_addr;
// #if (sizeof(void *) <= 4) &&  defined(__LITTLE_ENDIAN__)
//	volatile uint32_t	sd_unused;
// #endif
	volatile uint32_t	sd_dma_length;
} EMMC;

struct	EMMCDRV_S;

extern	struct	EMMCDRV_S *emmc_init(EMMC *dev);
extern	int	emmc_write(struct EMMCDRV_S *dev, const unsigned sector, const unsigned count, const char *buf);
extern	int	emmc_read(struct EMMCDRV_S *dev, const unsigned sector, const unsigned count, char *buf);
extern	int	emmc_ioctl(struct EMMCDRV_S *dev, char cmd, char *buf);
#endif
