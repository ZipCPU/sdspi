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
// Copyright (C) 2016-2026, Gisselquist Technology, LLC
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
#if defined(__SIZEOF_POINTER__) && (__SIZEOF_POINTER__ == 8)
	// 64b architectures
	volatile void	*sd_dma_addr;
#else
	// 32b architectures, or default if __SIZEOF__POINTER__ is undefined
  #if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
	volatile void		*sd_dma_addr;
	volatile uint32_t	sd_unused;
  #else
	volatile uint32_t	sd_unused;
	volatile void		*sd_dma_addr;
  #endif
#endif
	volatile uint32_t	sd_dma_length;
} EMMC;

struct	EMMCDRV_S;

extern	struct	EMMCDRV_S *emmc_init(EMMC *dev);
extern	int	emmc_write(struct EMMCDRV_S *dev, const unsigned sector, const unsigned count, const char *buf);
extern	int	emmc_read(struct EMMCDRV_S *dev, const unsigned sector, const unsigned count, char *buf);
extern	int	emmc_ioctl(struct EMMCDRV_S *dev, char cmd, char *buf);
extern	int	emmc_boot(struct EMMC *dev, const unsigned count, char *buf);
extern	int	emmc_altboot(struct EMMC *dev, const unsigned count, char *buf);
#endif
