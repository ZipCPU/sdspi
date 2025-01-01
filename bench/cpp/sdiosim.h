////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/cpp/sdiosim.h
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
// Copyright (C) 2016-2025, Gisselquist Technology, LLC
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
#ifndef	SDIOSIM_H
#define	SDIOSIM_H

#include <stdio.h>
#include <stdint.h>

class	SDIOSIM {
	static	constexpr int	DBUFLN = 512+2+32;
	bool	HCS;

	FILE		*m_fp;
	uint32_t	m_buf[512/sizeof(uint32_t)];
	bool		m_readonly, m_reply_active, m_open_drain, m_ddr,
			m_cmd_started, m_reply_started;
	bool		m_data_started, m_multiblock;
	uint32_t	m_sector, m_data_posn;

	uint32_t	m_last_dat, m_last_cmd, m_lastck, m_app_cmd,
			m_selected, m_RCA, m_width, m_drive, m_data_count,
			m_crctoken;
	char		m_cmd_buf[8], m_cid[16], m_reply_buf[20],
			m_dbuf[DBUFLN], m_csd[16], m_scr[8];
	uint32_t	m_cmd_pos, m_reply_posn, m_reply_count, m_R1,
			m_reply_delay, m_data_delay, m_busy_cycles;
	bool		m_debug;
protected:
	void		init(void);
	unsigned	cmdbit(unsigned);
	unsigned	datp(unsigned);
	unsigned	datn(unsigned);
	void		accept_command(void);
	void		load_reply(int cmd, unsigned arg);
	void		appendcrc(unsigned len_bytes);
	uint8_t		cmdcrc(int ln, char *buf);
	unsigned	blockcrc(unsigned fill, unsigned bit);
	void		CID(void);
	void		CSD(void);
	void		SCR(void);
public:
	// SDIOSIM(const unsigned lglen);
	SDIOSIM(const char *fname);
	void	load(const unsigned addr, const char *fname);
	void	load(const char *fname) { load(0, fname); }
	bool	card_busy(void) const { return m_busy_cycles > 0; };
	unsigned crctoken(void) const { return m_crctoken; };
	void	apply(unsigned sdclk, unsigned ddr,
			unsigned cmd_en, unsigned cmd_data,
			unsigned data_en, unsigned rx_en, unsigned tx_data,
			unsigned &o_sync, unsigned &async_sync,
			unsigned &async_data);
};

#endif
