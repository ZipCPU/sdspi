////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdiosim.cpp
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
// }}}
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
//
// #include "sdiosim.h"

class	SDIOSIM {
	FILE		*m_fp;
	uint32_t	m_buf[512/sizeof(uint32_t)];
	bool		m_readonly, m_reply_active, m_open_drain, m_ddr,
			m_data_started, m_cmd_started;
	uint32_t	m_last_dat, m_last_cmd, m_lastck, m_app_cmd,
			m_selected, m_RCA, m_width, m_drive;
	char		m_cmd_buf[8], m_cid[15], m_reply_buf[20],
			m_dbuf[512+1+32];
	uint32_t	m_cmd_pos, m_reply_posn, m_reply_count, m_R1,
			m_reply_delay, m_sector, m_data_delay, m_data_posn;
protected:
	void		init(void);
	unsigned	cmdbit(unsigned);
	unsigned	datp(unsigned);
	unsigned	datn(unsigned);
	void		accept_command(void);
	void		load_reply(int cmd, unsigned arg);
	uint8_t		cmdcrc(int ln, char *buf);
	uint16_t	blockcrc(uint16_t fill, int bit);
	void		CID(void);
public:
	// SDIOSIM(const unsigned lglen);
	SDIOSIM(const char *fname);
	void	load(const unsigned addr, const char *fname);
	void	load(const char *fname) { load(0, fname); }
	void	apply(unsigned sdclk, unsigned ddr,
			unsigned cmd_en, unsigned cmd_data,
			unsigned data_en, unsigned rx_en, unsigned tx_data,
			unsigned &o_sync, unsigned &async_sync,
			unsigned &async_data);
};


SDIOSIM::SDIOSIM(const char *fname) {
	// {{{
	if (0 == access(fname, R_OK|W_OK)) {
		m_fp = fopen(fname, "rw");
		m_readonly = false;
	} else if (0 == access(fname, R_OK)) {
		m_fp = fopen(fname, "r");
		m_readonly = true;
	} else {
		fprintf(stderr, "ERR: Could not open SDIOSIM backing file, %s\n", fname);
		exit(EXIT_FAILURE);
	}

	init();
}
// }}}

void	SDIOSIM::init(void) {
	// {{{
	m_lastck = 0; m_last_dat = -1; m_last_cmd = -1;
	m_cmd_pos = 0;
	for(unsigned ik=0; ik<8; ik++)
		m_cmd_buf[ik] = 0;
	m_sector = 0;
	m_open_drain = true;
	m_app_cmd = 0;
	m_selected = 1;
	m_RCA = 0;
	m_width = 1;
	m_reply_active = false;
	m_reply_posn   = 0;
	m_reply_count  = 0;
	m_reply_delay  = 0;
	m_drive = 0;
	m_ddr = false;
	m_data_started = false;
	m_cmd_started = false;
}
// }}}

void	SDIOSIM::CID(void) {
	for(unsigned k=0; k<15; k++)
		m_cid[k] = rand();
}

uint8_t	SDIOSIM::cmdcrc(int ln, char *buf) {
	// {{{
	unsigned int fill = 0, taps = 0x12;

	for(int i=0; i<ln; i++) {
		fill ^= buf[i];
		for(int j=0; j<8; j++) {
			if (fill&0x80)
				fill = (fill<<1)^taps;
			else
				fill <<= 1;
		}
	}

	fill &= 0x0fe; fill |= 1;
	return fill;
}
// }}}

void	SDIOSIM::load_reply(int cmd, unsigned arg) {
	// {{{
	m_reply_active = true;
	m_reply_delay  = 1 + (rand() & 0x07);
	m_reply_buf[0] = cmd & 0x03f;
	m_reply_buf[1] = (arg >> 24)&0x0ff;
	m_reply_buf[2] = (arg >> 16)&0x0ff;
	m_reply_buf[3] = (arg >>  8)&0x0ff;
	m_reply_buf[4] = (arg      )&0x0ff;
	m_reply_buf[5] = cmdcrc(5, &m_reply_buf[1]);

	m_reply_posn = 0;
	m_reply_count = 48;
}
// }}}

void	SDIOSIM::accept_command(void) {
	// {{{
	uint32_t	arg;

	arg =  ((m_cmd_buf[1] & 0x0ff) << 24);
	arg |= ((m_cmd_buf[2] & 0x0ff) << 16);
	arg |= ((m_cmd_buf[3] & 0x0ff) <<  8);
	arg |=  (m_cmd_buf[4] & 0x0ff);

	printf("SDIOSIM::CMD%d - %02x:%02x%02x%02x%02x,%02x\n",
		m_cmd_buf[0]&0x03f, m_cmd_buf[0]&0x0ff,
		m_cmd_buf[1]&0x0ff, m_cmd_buf[2]&0x0ff,
		m_cmd_buf[3]&0x0ff, m_cmd_buf[4]&0x0ff,
		m_cmd_buf[5]&0x0ff);

	switch(m_cmd_buf[0]) {
	case 0: // Go idle
		// {{{
		m_open_drain = true;
		m_app_cmd = 0;
		m_selected = 1;
		m_RCA = 0;
		m_drive = 0;
		m_reply_active = 0;
		m_ddr = false;
		break; // Go idle
		// }}}
	case 6:
		m_app_cmd = 0;
		if (m_app_cmd) { // ACMD6
		// {{{
			if (m_selected) {
				m_width=((arg&0x03)==2) ? 4:1;
				load_reply(6, m_R1);
			}
		} else {
			// Not (yet) implemented: SWITCH_FUNCTION
		} break;
		// }}}
	case 55: // APP_CMD
		// {{{
		if (m_selected && (m_RCA == (arg >> 16))) {
			m_app_cmd = 1;
			load_reply(55, m_R1);
		} break;
		// }}}
	case 2: // ALL SEND CID
		// {{{
		m_app_cmd = 0;
		if (m_RCA == 0) {
			m_reply_active = 1;
			m_reply_buf[0] = 0xff;
			m_reply_buf[1] = 2;
			for(unsigned k=0; k<128/8; k++)
				m_reply_buf[k+2] = m_cid[k];
			m_reply_buf[1+128/8] = cmdcrc(128/8, &m_reply_buf[1]);

			m_reply_posn = 4;
			m_reply_count = 136+4;
			m_reply_delay = 0;
		} break;
		// }}}
	case 3: // SEND RELATIVE ADDRESS
		// {{{
		m_app_cmd = 0;
		m_open_drain = false;
		do {
			m_RCA = rand()&0x0ff;
		} while(m_RCA == 0);

		load_reply(3, m_RCA<<16);
		m_selected = 0;
		m_drive = 1;
		break;
		// }}}
	case 7: // SELECT_CARD
		// {{{
		m_app_cmd = 0;
		m_selected = (m_RCA == ((arg>>16)&0x0ffff));
		if (m_selected)
			load_reply(8,m_R1);
		break;
		// }}}
	case 8: // SEND_IF_COND
		// {{{
		m_app_cmd = 0;
		m_reply_active = 1;
		m_reply_buf[0] = ((m_cmd_buf[0] & 0x0ff) << 24) & 0x03f;
		m_reply_buf[1] = 0;
		m_reply_buf[2] = 0;
		m_reply_buf[3] = 0;
		m_reply_buf[4] = 1;
		m_reply_buf[5] = (arg & 0x0ff);
		m_reply_buf[6] = cmdcrc(5, &m_reply_buf[1]);

		m_reply_posn = 4;
		m_reply_count = 48+4;
		break;
		// }}}
	case 11: // VOLTAGE_SWITCH
		// We only support 3.3V (for now)
		m_app_cmd = 0;
		break;
	case 19: // READ_BLOCK
		// {{{
		m_app_cmd = 0;
		if (m_selected) {
			m_drive = 1;
			load_reply(19,m_R1);

			m_sector     = arg;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;
			// m_data_count = 512;

			(void)fseek(m_fp, (long)(m_sector * 512l), SEEK_SET);
			(void)fread(m_dbuf, sizeof(char), 512, m_fp);
		} break;
		// }}}
	case 24: // WRITE_BLOCK
		// {{{
		m_app_cmd = 0;
		if (m_selected) {
			m_drive = 0;
			load_reply(19,m_R1);

			m_sector     = arg;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;
			m_data_started = false;
			// m_data_count = 512;
		} break;
		// }}}
	default:
		break;
	}
}
// }}}

unsigned SDIOSIM::cmdbit(unsigned in) {
	// {{{
	if (m_reply_active) {
		unsigned	r;

		if (m_reply_delay > 0) {
			m_reply_delay--;
			return 1;
		}

		r = m_reply_buf[m_reply_posn>>3];
		r = r >> (7-(m_reply_posn&7));
		r &= 1;

		m_reply_posn++;
		if (m_reply_posn >= m_reply_count)
			m_reply_active = 0;

		return r;
	} else if (m_cmd_pos > 0 || in==0) {
		m_cmd_buf[m_cmd_pos>>3] = (m_cmd_buf[m_cmd_pos>>3] << 1) | in;
		m_cmd_pos++;

		if (m_cmd_pos == 48) {
			m_cmd_pos = 0;
			if (cmdcrc(5, m_cmd_buf) != m_cmd_buf[5]) {
				printf("SDIOSIM::CMD CRC-FAILURE!\n");
			} else if ((m_cmd_buf[0] & 0x0c0) != 0x040) {
				printf("SDIOSIM::CMD FRAME FAILURE: %02x\n",
					m_cmd_buf[0]);
			} else {
				accept_command();
				m_cmd_started = false;
			}
		}
	} return in;
}
// }}}

unsigned SDIOSIM::datp(unsigned in) {
	// {{{
	if (m_drive) {
		unsigned	r;

		if (m_data_delay > 0) {
			m_data_delay--;
			return 0x0ff;
		}

		if (m_width == 8) {
			r = m_dbuf[m_data_posn>>3];
			r = (r & 0x0ff);
			m_data_posn += 8;

		} else if (m_width == 4) {
			r = m_dbuf[m_data_posn>>3];
			r = r >> (4-(m_data_posn&4));
			r = (0x0f0) | (r & 0x0f);
			m_data_posn += 4;

		} else { // if (m_width == 1)
			r = m_dbuf[m_data_posn>>3];
			r = r >> (7-(m_data_posn&7));
			r = (0x0fe) | (r & 1);

			m_data_posn ++;
		}

		if (m_data_posn >= (512*8+m_width*32))
			m_drive = false;

		if (m_open_drain)
			return r & in;
		else
			return r;
	} else if (!m_data_started) {
		if (m_width >= 8) {
			if (0 == in)
				m_data_started = true;
		} else if (m_width >= 4) {
			if (0 == (in & 0x0f))
				m_data_started = true;
		} else if (0 == (in & 0x01))
			m_data_started = true;
	} else if (m_data_posn < 512*8 + 8*16) {
		unsigned	idx = m_data_posn >> 3;
		if (m_width >= 8) {
			m_dbuf[idx] = in & 0x0ff;
			m_data_posn += 8;
		} else if (m_width >= 4) {
			unsigned	tmp;

			tmp = m_dbuf[idx];
			if (m_data_posn & 4)
				m_dbuf[idx] = (in << 4);
			else
				m_dbuf[idx] = (tmp & 0x0f0) | (in & 0x0f);
			m_data_posn += 4;
		} else {
			unsigned	tmp;

			in &= 1;
			tmp = m_dbuf[idx];
			if (0 == (m_data_posn & 7))
				m_dbuf[idx] = (in << 7);
			else
				m_dbuf[idx] = tmp | (in << (7-(m_data_posn&7)));
			m_data_posn += 1;
		}

		if (m_data_posn == 512*8 + (16 * m_width)) {
			bool		crc_fail = false;
			unsigned	fill;

			// Check CRCs
			if (m_width >= 8) {
				for(unsigned w=0; w<8 && !crc_fail; w++) {
					fill = 0;
					for(unsigned k=0; k<m_data_posn; k+=8) {
						unsigned b;
						b = (m_dbuf[k>>3] >> w) & 1;
						fill = blockcrc(fill, b);
					} if (fill != 0)
						crc_fail = true;
				}
			} else if (m_width >= 4) {
				for(unsigned w=0; w<4 && !crc_fail; w++) {
					fill = 0;
					for(unsigned k=0; k<m_data_posn; k+=8) {
						unsigned v = m_dbuf[k>>3];

						v >>= w;
						if (v & 0x010)
							fill = blockcrc(fill,1);
						else
							fill = blockcrc(fill,0);
						fill = blockcrc(fill,(v&1));
					} if (fill != 0)
						crc_fail = true;
				}
			} else { // if (m_width == 1)
				fill = 0;
				for(unsigned k=0; k<m_data_posn; k++) {
					unsigned	b;

					b = (m_dbuf[k>>3] >> (7-(k&7))) & 1;
					fill = blockcrc(fill, b);
				}
				crc_fail = (fill != 0);
			}
		} return in;
	}

	return 0x0ff;
}
// }}}

unsigned SDIOSIM::datn(unsigned in) {
	// {{{
	if (!m_ddr)
		return in;

	if (m_drive) {
		// {{{
		unsigned	r = 0;

		if (m_data_delay > 0) {
			m_data_delay--;
			return 0x0ff;
		}

		if (m_width == 8) {
			r = m_dbuf[m_data_posn>>3];
			r = (r & 0x0ff);
			m_data_posn += 8;

		} else if (m_width == 4) {
			r = m_dbuf[m_data_posn>>3];
			r = r >> (4-(m_data_posn&4));
			r = (0x0f0) | (r & 0x0f);
			m_data_posn += 4;

		} else { // if (m_width == 1)
			r = m_dbuf[m_data_posn>>3];
			r = r >> (7-(m_data_posn&7));
			r = (0x0fe) | (r & 1);

			m_data_posn ++;
		}

		if (!m_ddr && m_data_posn >= (512*8+m_width*16))
			m_drive = false;

		if (m_open_drain)
			return r & in;
		else
			return r;
		// }}}
	} else if (!m_data_started) {
		// {{{
		if (m_width >= 8) {
			if (0 == in)
				m_data_started = true;
		} else if (m_width >= 4) {
			if (0 == (in & 0x0f))
				m_data_started = true;
		} else if (0 == (in & 0x01))
			m_data_started = true;

		return in;
		// }}}
	} else if (m_data_posn < 512*8 + 16*16) { // Receive a block of data
		// {{{
		unsigned	idx = m_data_posn >> 3;
		if (m_width >= 8) {
			m_dbuf[idx] = in & 0x0ff;
			m_data_posn += 8;
		} else if (m_width >= 4) {
			unsigned	tmp;

			tmp = m_dbuf[idx];
			if (m_data_posn & 4)
				m_dbuf[idx] = (in << 4);
			else
				m_dbuf[idx] = (tmp & 0x0f0) | (in & 0x0f);
			m_data_posn += 4;
		} else {
			unsigned	tmp;

			in &= 1;
			tmp = m_dbuf[idx];
			if (0 == (m_data_posn & 7))
				m_dbuf[idx] = (in << 7);
			else
				m_dbuf[idx] = tmp | (in << (7-(m_data_posn&7)));
			m_data_posn += 1;
		}

		if (m_data_posn == 512*8 + (32 * m_width)) { // Check CRC
			// {{{
			bool		crc_fail = false;
			unsigned	fill = 0;

			// Check CRCs
			if (m_width >= 8) {
				for(unsigned w=0; w<8 && !crc_fail; w++) {
					fill = 0;
					for(unsigned k=0; k<m_data_posn; k+=16){
						unsigned b;

						b = (m_dbuf[k>>4] >> w) & 1;
						fill = blockcrc(fill, b);
					} if (fill != 0)
						crc_fail = true;
				}

				for(unsigned w=0; w<8 && !crc_fail; w++) {
					fill = 0;
					for(unsigned k=0; k<m_data_posn; k+=16){
						unsigned b;

						b = (m_dbuf[1+(k>>4)] >> w) & 1;
						fill = blockcrc(fill, b);
					} if (fill != 0)
						crc_fail = true;
				}
			} else if (m_width >= 4) {
				for(unsigned w=0; w<8 && !crc_fail; w++) {
					fill = 0;
					for(unsigned k=0; k<m_data_posn; k+=8) {
						unsigned v = m_dbuf[k>>3] >> 4;

						fill = blockcrc(fill,(v&1));
					} if (fill != 0)
						crc_fail = true;
				}
			} else { // if (m_width == 1)
				fill = 0;
				for(unsigned k=0; k<m_data_posn; k+=2) {
					unsigned b;

					b = (m_dbuf[k>>3] >> (7-(k&7))) & 1;
					fill = blockcrc(fill, b);
				}
				crc_fail = (fill != 0);

				if (!crc_fail) {
					fill = 0;
					for(unsigned k=1; k<m_data_posn; k+=2) {
						unsigned b;

						b = (m_dbuf[k>>3] >> (7-(k&7))) & 1;
						fill = blockcrc(fill, b);
					}
					crc_fail = (fill != 0);
				}
			}

			if (!crc_fail) {
				(void)fseek(m_fp, (long)(m_sector * 512l), SEEK_SET);
				(void)fwrite(m_dbuf, sizeof(char), 512, m_fp);
			}
		}
		// }}}

		return in;
		// }}}
	} return in;
}
// }}}

void	SDIOSIM::apply(unsigned sdclk, unsigned ddr,
			unsigned cmd_en, unsigned cmd_data,
			unsigned data_en, unsigned rx_en, unsigned tx_data,
			unsigned &o_sync, unsigned &async_sync,
				unsigned &async_data) {
	// {{{
	unsigned	cstb = 0, cmd = 0, rstb = 0, rdat = 0, rlsb = 0;

	m_ddr = ddr;
	if (cmd_en)  m_cmd_started = 0;
	if (data_en || !rx_en) m_data_started = 0;

	// Tick the clock
	// {{{
	sdclk &= 0x0ff; sdclk |= (m_lastck&1)<<8; m_lastck = (sdclk&1);
	for(int s=8; s > 0; s--) {
		unsigned	edge, idat, icmd;

		edge = ((sdclk >> (s-1))&3);

		icmd = (s >= 4) ? (cmd_data & 2) : (cmd_data & 1);
		icmd = (icmd != 0) ? 1 : 0;
		if (!cmd_en)
			icmd = 1;

		idat = tx_data >> (((s-1)/2)*8);
		if (!data_en)
			idat = 0x0ff;
		else
			idat &= 0x0ff;

		if (edge == 1) {
			cstb = (cstb << 1) | 1;
			cmd = (cmd << 1) | (cmdbit(icmd) & 1);

			rstb = (rstb << 1) | 1;
			rdat = (rdat << 8) | (datp(idat) & 0x0ff);
			rlsb = (rlsb << 1) | (rdat&1);

		} else if (ddr && edge == 2) {
			rstb = (rstb << 1) | 1;
			rdat = (rdat << 8) | (datn(idat) & 0x0ff);
			rlsb = (rlsb << 1) | (rdat&1);
		}
	}
	// }}}

	// Pack the command return
	// {{{
	if (cstb)
		m_last_cmd = cmd & 1;
	if (!m_cmd_started && (cstb != 0) && (cstb != (cmd & cstb))) {
		if (cmd_en)
			cstb <= 0;
		else {
			if (cmd & 2)
				cstb &= ~2;
			m_cmd_started = 1;
		}
	} if (1 == (cstb & 3)) {
		cstb <<= 1;
		cmd  <<= 1;
	}
	// }}}

	// Pack the return data strobe
	// {{{
	if (data_en || !rx_en) {
		rstb = 0;
		m_data_started == 0;
	} else if ((!m_data_started) && (rstb != (rstb & rlsb))) {
		unsigned	msk = (rstb & ~rlsb) & 0x0f;

		if (msk&0x08) {
			m_data_started = 1;
		} else if (msk &0x04 ) {
			m_data_started = 1;
			rstb <<= 1;
			rdat <<= 8;
		} else if (msk &0x02 ) {
			m_data_started = 1;
			rstb <<= 2;
			rdat <<= 16;
		} else if (msk & 0x01 ) {
			m_data_started = 1;
			rstb <<= 3;
			rdat <<= 24;
		}
	} if (rstb && !(rstb & 0x08)) {
		while(!(rstb & 0x08)) {
			rstb <<= 1;
			rdat <<= 8;
		}
	} if (!(rstb & 0x04) && (rstb & 0x03)) {
		while(!(rstb & 0x04)) {
			rstb = (rstb & 0x08) | ((rstb & 0x03) << 1);
			rdat = (rdat & 0xff000000) | ((rdat & 0x0ffff) << 8);
		}
	} if (!(rstb & 0x02)) {
		rstb = (rstb & 0x0c) | ((rstb & 0x01) << 1);
		rdat = (rdat & 0xffff0000) | ((rdat & 0x0ff) << 8);
	}
	// }}}

	// Synchronous return can only return two clocks of data
	o_sync = (cstb << 30) | (cmd << 28) | (rstb << 24) | (rdat >> 16);

	async_sync = (cstb & 2) | ((rstb == 0x0f) ? 1:0);
	async_data = rdat;
}
// }}}
