////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdspisim.h
//
// Project:	Wishbone Controlled SD-Card Controller over SPI port
//
// Purpose:	This library simulates the operation of a SPI commanded SD-Card,
//		such as might be found on a XuLA2-LX25 board made by xess.com.
//
//	This simulator is for testing use in a Verilator/C++ environment, where
//	it would be used in place of the actual hardware.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2015-2017, Gisselquist Technology, LLC
//
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
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
#ifndef	SDSPISIM_H
#define	SDSPISIM_H

typedef enum	eRESET_STATES {
	SDSPI_POWERUP_RESET,
	SDSPI_CMD0_IDLE,
	SDSPI_RCVD_CMD8,
	SDSPI_RCVD_ACMD41,
	SDSPI_RESET_COMPLETE,
	SDSPI_IN_OPERATION
} RESET_STATES;

#define	SDSPI_RSPLEN	8
#define	SDSPI_MAXBLKLEN	(1+2048+2)
#define	SDSPI_CSDLEN	(16)
#define	SDSPI_CIDLEN	(16)
class	SDSPISIM {
	FILE		*m_dev;
	unsigned long	m_devblocks;

	int		m_last_sck, m_delay, m_mosi;
	bool		m_busy, m_debug, m_block_address, m_altcmd_flag,
			m_syncd, m_host_supports_high_capacity, m_reading_data,
			m_have_token;

	RESET_STATES	m_reset_state;

	int		m_cmdidx, m_bitpos, m_rspidx, m_rspdly, m_blkdly,
				m_blklen, m_blkidx, m_last_miso, m_powerup_busy,
				m_rxloc;
	char		m_cmdbuf[8], m_dat_out, m_dat_in;
	char		m_rspbuf[SDSPI_RSPLEN];
	char		m_block_buf[SDSPI_MAXBLKLEN];
	char		m_csd[SDSPI_CSDLEN], m_cid[SDSPI_CIDLEN];

public:
	SDSPISIM(const bool debug = false);
	void	load(const char *fname);
	void	debug(const bool dbg) { m_debug = dbg; }
	bool	debug(void) const { return m_debug; }
	int	operator()(const int csn, const int sck, const int dat);
	unsigned cmdcrc(int ln, char *buf) const;
	bool	check_cmdcrc(char *buf) const;
	unsigned blockcrc(int ln, char *buf) const;
	void	add_block_crc(int ln, char *buf) const;
};

#endif
