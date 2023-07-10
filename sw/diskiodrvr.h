////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	diskiodrvr.h
// {{{
// Project:	SD-Card controller
//
// Purpose:	Defines a structure which can be used to identify both the
//		1) number of drives (devices) in a system, and 2) what type
//	of drives and thus which software device driver, needs to be applied
//	to each device.
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
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
#ifndef	DISKIODRVR_H
#define	DISKIODRVR_H

typedef	struct	DISKIODRVR_S {
	void	*(*dio_init)(void *io_addr);
	DISKIODRVR_S * (*dio_init)(void *io_addr);
	int	(*dio_write)(void *dev, const unsigned sector,
				const unsigned count, const char *buf);
	int	(*dio_read)(void *dev, const unsigned sector,
				const unsigned count, char *buf);
	int	(*dio_ioctl)(void *dev, const char cmd, char *buf);
} DISKIODRVR;

DISKIODRVR	SDIODRVR  = { sdio_init, sdio_write, sdio_read, sdio_ioctl  };
DISKIODRVR	SDSPIDRVR = { sdspi_init,sdspi_write,sdspi_read,sdspi_ioctl };
// DISKIODRVR	EMMCDRVR  = { emmc_init, emmc_write, emmc_read, emmc_ioctl };

typedef	struct	FATDRIVE_S {
	void		*fd_addr;
	DISKIODRVR	*fd_driver;
	void		*fd_data;
} FATDRIVE;

// UPDATE ME!
// The following lines need to be updated from one board to the next, so that
// there's one FATDRIVE triplet per drive on the board, and so that MAX_DRIVES
// contains the number of items in the table.
//
#define	MAX_DRIVES	4
FATDRIVE	DRIVES[MAX_DRIVES] = {
#ifdef	_BOARD_HAS_SDSPI
		{ _sdspi, &SDSPIDRVR, NULL },
#else
		{ NULL, NULL, NULL },
#endif
#ifdef	_BOARD_HAS_SDIO
		{ _sdio, &SDIODRVR, NULL },
#else
		{ NULL, NULL, NULL },
#endif
#ifdef	_BOARD_HAS_EMMC
		{ _emmc, &EMMCDRVR, NULL },
#else
		{ NULL, NULL, NULL },
#endif
		{NULL, NULL, NULL }
	};

#endif
