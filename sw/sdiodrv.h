////////////////////////////////////////////////////////////////////////////////
//
// Filename:	sw/sdiodrv.h
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
#ifndef	SDIODRV_H
#define	SDIODRV_H
#include <stdint.h>

typedef	struct SDIO_S {
	volatile uint32_t	sd_cmd, sd_data, sd_fifa, sd_fifb, sd_phy;

	// The next two 32b words are allocated to a 64b pointer.
	// If pointers are 32b, then we only need to allocate one of them
	// to this purpose.  Which one we allocated will depend upon whether
	// we are big-endian (ZipCPU) or little-endian (AXI, and much of the
	// rest of the world).
#if defined(__SIZEOF_POINTER__) && (__SIZEOF_POINTER__ == 8)
	// 64b architectures
	volatile uint32_t	sd_unused;
#else
	// 32b architectures, or default if not __SIZEOF_POINTER__ is undefined
  #if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
	// Little-Endian
	volatile void		*sd_dma_addr;
	volatile uint32_t	sd_unused;
  #else
	// Big-Endian, the default if __BYTE_ORDER__ is undefined
	volatile uint32_t	sd_unused;
	volatile void		*sd_dma_addr;
  #endif
#endif
	volatile uint32_t	sd_dma_length;
} SDIO;

struct	SDIODRV_S;

extern	struct	SDIODRV_S *sdio_init(SDIO *dev);
extern	int	sdio_write(struct SDIODRV_S *dev, const unsigned sector, const unsigned count, const char *buf);
extern	int	sdio_read(struct SDIODRV_S *dev, const unsigned sector, const unsigned count, char *buf);
extern	int	sdio_ioctl(struct SDIODRV_S *dev, char cmd, char *buf);
#endif
