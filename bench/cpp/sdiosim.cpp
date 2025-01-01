////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/cpp/sdiosim.cpp
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include "sdiosim.h"

constexpr unsigned	COM_CRC_ERROR	= 0x00800000,
			CRCACK = 1, CRCNAK = 2,
			SECTORSZ = 512;

SDIOSIM::SDIOSIM(const char *fname) : m_debug(false) {
	// {{{
	HCS = true;
	m_crctoken = 0;

	if (0 == access(fname, R_OK|W_OK)) {
		m_fp = fopen(fname, "r+");
		m_readonly = false;
		if (m_debug)
			printf( "SDIOSIM: %s opened for read and write\n", fname);
	} else if (0 == access(fname, R_OK)) {
		m_fp = fopen(fname, "r");
		m_readonly = true;
		printf( "SDIOSIM: WARNING: %s opened read only\n", fname);
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
	m_reply_started = false;
	m_multiblock = false;
	m_busy_cycles = 0;
	m_crctoken = 0;

	CID();
	CSD();
	SCR();
}
// }}}

void	SDIOSIM::CID(void) {
	// {{{
	// for(unsigned k=0; k<15; k++)
	//	m_cid[k] = rand();

	// Copied from a card I'm using
	m_cid[ 0] = 0x02;
	m_cid[ 1] = 0x54;
	m_cid[ 2] = 0x4d;
	m_cid[ 3] = 0x53;

	m_cid[ 4] = 0x41;
	m_cid[ 5] = 0x30;
	m_cid[ 6] = 0x34;
	m_cid[ 7] = 0x47;

	m_cid[ 8] = 0x21;
	m_cid[ 9] = 0x31;
	m_cid[10] = 0x60;
	m_cid[11] = 0x59;

	m_cid[12] = 0x12;
	m_cid[13] = 0x01;
	m_cid[14] = 0x3c;
	m_cid[15] = 0x4d;
}
// }}}

void	SDIOSIM::CSD(void) {
	// {{{
	// Copied from a card I'm using
	m_csd[ 0] = 0x40;
	m_csd[ 1] = 0x0e;
	m_csd[ 2] = 0x00;
	m_csd[ 3] = 0x32;

	m_csd[ 4] = 0x5b;
	m_csd[ 5] = 0x59;
	m_csd[ 6] = 0x00;
	m_csd[ 7] = 0x00;

	m_csd[ 8] = 0x1d;
	m_csd[ 9] = 0x17;
	m_csd[10] = 0x7f;
	m_csd[11] = 0x80;

	m_csd[12] = 0x0a;
	m_csd[13] = 0x40;
	m_csd[14] = 0x00;
	m_csd[15] = 0x8d;
}
// }}}

void	SDIOSIM::SCR(void) {
	// {{{
	for(unsigned k=0; k<8; k++)
		m_scr[k] = rand();
	m_scr[0] = 0x06;	// Bits 63:56
	m_scr[1] = 0x85;	// Bits 55:48
	m_scr[2] = 0x84;	// Bits 47:40
	m_scr[3] = 0x80;	// Bits 39:32
		// SD_SPEC3=1'b1	// Bit 47
		// EX_SECURITY=4'h0	Bits 46:43
		// SD_SPEC4=1'b1	Bit  42
		///////
		// SD_SPECX=4'h2	Bit  41:38
		// 	2'bxx		Bits 37:36
		// CMD_SUPPORT=4'h0	Bits 35:32

	// Bytes 4-7 are reserved for manufacturer usage
	// m_scr[4] = 0x00;	// Bits 31:24
	// m_scr[5] = 0x00;	// Bits 23:16
	// m_scr[6] = 0x00;	// Bits 15: 8
	// m_scr[7] = 0x00;	// Bits  7: 0
}
// }}}

uint8_t	SDIOSIM::cmdcrc(int ln, char *buf) {
	// {{{
		unsigned int	fill = 0;
	constexpr unsigned	TAPS = 0x12;	// was 0x09, now <<= 1

	for(int i=0; i<ln; i++) {
		fill ^= buf[i];
		for(int j=0; j<8; j++) {
			if (fill&0x80)
				fill = (fill<<1)^TAPS;
			else
				fill <<= 1;
		}
	}

	fill &= 0x0fe; fill |= 1;
	return fill;
}
// }}}

unsigned	SDIOSIM::blockcrc(unsigned fill, unsigned bit) {
	// {{{
	constexpr unsigned	TAPS = 0x1021;
	unsigned	b;

	b = (fill >> 15)&1;
	b ^= bit;

	if (b)
		fill = (fill<<1)^TAPS;
	else
		fill <<= 1;

	return fill & 0x0ffff;
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
	m_reply_buf[5] = cmdcrc(5, m_reply_buf);

	if (m_debug) {
		printf("SDIOSIM::REPLY%d%*s-- %02x:%02x%02x%02x%02x,%02x\n",
			m_reply_buf[0] & 0x0ff,
				((m_reply_buf[0]&0x0ff) > 9) ? 1:2, "",
			m_reply_buf[0] & 0x0ff,
			m_reply_buf[1] & 0x0ff, m_reply_buf[2] & 0x0ff,
			m_reply_buf[3] & 0x0ff, m_reply_buf[4] & 0x0ff,
			m_reply_buf[5] & 0x0ff);
	}

	m_reply_posn = 0;
	m_reply_count = 48;
}
// }}}

void	SDIOSIM::appendcrc(unsigned len_bytes) {
	// {{{
	unsigned	fill;

	//unsigned	SDIOSIM::blockcrc(unsigned fill, unsigned bit) {
	if (false && m_debug) {
		// {{{
		printf("SDIOSIM: Preparing block to receive:\n");
		for(unsigned k=0; k<len_bytes; k++) {
			printf("%02x", m_dbuf[k] & 0x0ff);
			if ((k == (len_bytes-1)) || (15 == (k&15)))
				printf("\n");
			else if (7 == (k & 7))
				printf("  ");
			else
				printf(" ");
		}
	}
	// }}}

	if (m_ddr) {
		if (m_width >= 8) {
			// {{{
			for(int k=0; k<32; k++)
				m_dbuf[len_bytes+k] = 0;
			m_dbuf[len_bytes+32] = 0xff;
			for(unsigned w=0; w<16; w++) {
				unsigned lsb = w & 7;
				fill = 0;
				for(unsigned k=(w>=8 ? 1:0);k<len_bytes; k+=2){
					unsigned b;
					b = (m_dbuf[k] >> lsb) & 1;
					fill = blockcrc(fill, b);
				}

				unsigned msk = 1 << (7-lsb);
				for(int k=0; k<16; k++)
					m_dbuf[len_bytes + 2*k + (w>=8 ? 1:0)]
						|= ((fill & (1<<(15-k))) ? msk:0);
			}

			for(unsigned k=len_bytes+34; k>0; k--)
				m_dbuf[k] = m_dbuf[k-2];
			m_dbuf[0] = 0;
			m_dbuf[1] = 0;
			for(unsigned k=len_bytes+14; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			// }}}
		} else if (m_width >= 4) {
			// {{{
			for(int k=0; k<16; k++)
				m_dbuf[len_bytes+k] = 0;
			m_dbuf[len_bytes+16] = 0xff;
			for(unsigned w=0; w<8; w++) {
				fill = 0;
				for(unsigned k=0; k<len_bytes; k++) {
					unsigned b;
					b = (m_dbuf[k] >> w) & 1;
					fill = blockcrc(fill, b);
				}

				unsigned msk = 1 << (7-w);
				for(int k=0; k<16; k++)
					m_dbuf[len_bytes + k] |= ((fill & (1<<(15-k))) ? msk:0);
			}

			for(unsigned k=len_bytes+17; k>0; k--)
				m_dbuf[k] = m_dbuf[k-1];
			m_dbuf[0] = 0;
			for(unsigned k=len_bytes+17; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			// }}}
		} else { // if (m_width == 1)
			// {{{
			fill = 0;
			for(unsigned k=0; k<len_bytes; k++) {
				unsigned b, d;
				d = m_dbuf[k] & 0x0ff;

				for(unsigned w=0; w<8; w+=2) {
					b = (d & 0x80) ? 1:0; d<<= 2;
					fill = blockcrc(fill, b);
				}
			}

			for(unsigned w=0; w<8; w++) {
				unsigned b, p;

				b  = fill >> (15-2*w);
				b &= 1;
				b <<= (7-2*(w&3));

				p = len_bytes + ((w >= 4) ? 1:0);
				m_dbuf[p] |= b;
			}

			fill = 0;
			for(unsigned k=0; k<len_bytes; k++) {
				unsigned b, d;
				d = m_dbuf[k] & 0x0ff;

				for(unsigned w=1; w<8; w+=2) {
					b = (d & 0x40) ? 1:0; d<<= 2;
					fill = blockcrc(fill, b);
				}
			}

			for(unsigned w=0; w<8; w++) {
				unsigned b, p;

				b  = fill >> (14-2*w);
				b &= 1;
				b <<= (6-2*(w&3));

				p = len_bytes + ((w >= 4) ? 1:0);
				m_dbuf[p] |= b;
			}

			m_dbuf[len_bytes+2] = 0x0ff;

			for(unsigned k=len_bytes+3; k> 0; k--) {
				unsigned d;
				d = m_dbuf[k] & 0x0ff;
				d >>= 2;
				d |= (m_dbuf[k-1] << 6);
				m_dbuf[k] = d;
			} m_dbuf[0] = (m_dbuf[0] & 0x0ff) >> 2u;
			for(unsigned k=len_bytes+3; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			// }}}
		}
	} else {
		if (m_width >= 8) {
			// {{{
			for(int k=0; k<16; k++)
				m_dbuf[len_bytes+k] = 0;
			m_dbuf[len_bytes+16] = 0xff;
			for(unsigned w=0; w<8; w++) {
				fill = 0;
				for(unsigned k=0; k<len_bytes; k++) {
					unsigned b;
					b = (m_dbuf[k] >> w) & 1;
					fill = blockcrc(fill, b);
				}

				unsigned msk = 1 << (7-w);
				for(int k=0; k<16; k++)
					m_dbuf[len_bytes + k] |= ((fill & (1<<(15-k))) ? msk:0);
			}

			for(unsigned k=len_bytes+17; k>0; k--)
				m_dbuf[k] = m_dbuf[k-1];
			m_dbuf[0] = 0;
			for(unsigned k=len_bytes+17; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			// }}}
		} else if (m_width >= 4) {
			// {{{
			for(int k=0; k<8; k++)
				m_dbuf[len_bytes+k] = 0;
			m_dbuf[len_bytes+ 8] = 0xff;
			m_dbuf[len_bytes+ 9] = 0xff;
			for(unsigned w=0; w<4; w++) {
				fill = 0;
				for(unsigned k=0; k<len_bytes; k++) {
					unsigned b, d;

					d = (m_dbuf[k] & 0x0ff) >> w;

					b = (d & 0x10) ? 1:0;
					fill = blockcrc(fill, b);

					b = d & 1;
					fill = blockcrc(fill, b);
				}

				for(int k=0; k<8; k++) {
					unsigned d = 0;

					d  = (fill & 0x8000) ? 1 : 0;
					fill <<= 1; d <<= 4;

					d |= (fill & 0x8000) ? 1 : 0;
					fill <<= 1;

					m_dbuf[len_bytes + k] |= (d << w);
				}
			}

			if (m_debug) {
				// {{{
				for(unsigned w=0; w < 4; w++) {
					fill = 0;
					for(unsigned k=0; k<len_bytes+8; k++) {
						unsigned b, d;

						d = (m_dbuf[k] & 0x0ff) >> w;

						b = (d & 0x10) ? 1:0;
						fill = blockcrc(fill, b);

						b = d & 1;
						fill = blockcrc(fill, b);
					}
					// printf("SDIOSIM: Check CRC%d = %04x\n", w, fill); fflush(stdout);
					assert(fill == 0);
				}
			}
			// }}}

			for(unsigned k=len_bytes+9; k> 0; k--) {
				unsigned d;

				d = m_dbuf[k] & 0x0ff;
				d >>= 4;
				d |= (m_dbuf[k-1] & 0x0f)<<4;
				m_dbuf[k] = d;
			} m_dbuf[0] = (m_dbuf[0] & 0x0ff) >> 4;
			for(unsigned k=len_bytes+9; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			// }}}
		} else { // if (m_width == 1)
			// {{{
			fill = 0;
			for(unsigned k=0; k<len_bytes; k++) {
				unsigned b, d;
				d = m_dbuf[k] & 0x0ff;

				for(unsigned w=0; w<8; w++) {
					b = (d & 0x80) ? 1:0; d<<= 1;
					fill = blockcrc(fill, b);
				}
			}

			m_dbuf[len_bytes  ] =  (fill >> 8) & 0x0ff;
			m_dbuf[len_bytes+1] =  (fill & 0x0ff);
			m_dbuf[len_bytes+2] = 0x0ff;

			if (m_debug) {
				// {{{
				fill = 0;
				for(unsigned k=0; k<len_bytes+2; k++) {
					unsigned b, d;
					d = m_dbuf[k] & 0x0ff;

					for(unsigned w=0; w<8; w++) {
						b = (d & 0x80) ? 1:0; d<<= 1;
						fill = blockcrc(fill, b);
					}
				}

				//printf("SDIOSIM: Check CRC = %08x\n", fill); fflush(stdout);
				assert(fill == 0);
			}
			// }}}

			for(unsigned k=len_bytes+3; k> 0; k--) {
				unsigned d;
				d = m_dbuf[k] & 0x0ff;
				d >>= 1;
				if (m_dbuf[k-1]&0x01)
					d |= 0x80;
				m_dbuf[k] = d;
			} m_dbuf[0] = (m_dbuf[0] & 0x0ff) >> 1u;
			for(unsigned k=len_bytes+3; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			// }}}
		}
	}
}
// }}}

void	SDIOSIM::accept_command(void) {
	// {{{
	uint32_t	arg;
	bool		valid_crc;

	arg =  ((m_cmd_buf[1] & 0x0ff) << 24);
	arg |= ((m_cmd_buf[2] & 0x0ff) << 16);
	arg |= ((m_cmd_buf[3] & 0x0ff) <<  8);
	arg |=  (m_cmd_buf[4] & 0x0ff);

	if (m_debug) {
		// {{{
		printf("SDIOSIM::");
		if (m_app_cmd)
			printf("A");
		printf("CMD%d", m_cmd_buf[0] & 0x03f);
		printf("%*s",2 + (m_app_cmd ? 0:1)
				+ (((m_cmd_buf[0] & 0x03f) > 9) ? 0 : 1),"");
		printf("-  %02x:%02x%02x%02x%02x,%02x\n",
			m_cmd_buf[0]&0x0ff,
			m_cmd_buf[1]&0x0ff, m_cmd_buf[2]&0x0ff,
			m_cmd_buf[3]&0x0ff, m_cmd_buf[4]&0x0ff,
			m_cmd_buf[5]&0x0ff);
	}
	// }}}

	// Check the incoming CRC
	// {{{
	{
		unsigned	expected_crc = cmdcrc(5, m_cmd_buf);
		unsigned	rcvd_crc = m_cmd_buf[5] & 0x0ff;

		valid_crc = rcvd_crc == expected_crc;
		if (!valid_crc) {
			printf("SDIOSIM::ERR, CRC mismatch, expecting %02x, received %02x\n", expected_crc, rcvd_crc);
		}
	}
	// }}}

	// printf("SDIOSIM: Switching on %02x\n", m_cmd_buf[0] & 0x03f);

	switch(m_cmd_buf[0] & 0x03f) {
	case 0: // Go idle
		// {{{
		m_open_drain = true;
		m_app_cmd = 0;
		m_selected = 1;
		m_RCA = 0;
		m_drive = 0;
		m_reply_active = 0;
		m_ddr = false;
		m_drive = false;
		m_busy_cycles = 0;
		break; // Go idle
		// }}}
	case 6:
		if (m_app_cmd) { // ACMD6
			// {{{
			if (m_selected) {
				m_width=((arg&0x03)==2) ? 4:1;
				// printf("SDIOSIM: Setting width to %d bits\n", m_width);
				load_reply(6, m_R1);
				m_R1 &= ~(COM_CRC_ERROR);
			} else printf("SDIOSIM::ACMD6, ERR -- Not selected!!\n");
			m_app_cmd = 0;
			// }}}
		} else { // CMD6 - SWITCH_FUNC
			// {{{
			// printf("SDIOSIM::ERR - ACMD6 w/ No APP CMD set\n");
			// Not (yet) implemented: SWITCH_FUNCTION
			// Ignore this command for now.
			// printf("SDIOSIM::CMD-SWITCH\n");
			load_reply(6, m_R1);
			m_R1 &= ~(COM_CRC_ERROR);

			// arg

			// Return the new/updated/(would be) updated SCR reg
			m_drive = 1;
			m_sector     = 0;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;

			for(unsigned k=0; k<64; k++)
				m_dbuf[k] = 0;

			m_dbuf[0] = 0x0ff;	// Bits 504	(MAX Current)
			m_dbuf[1] = 0x0ff;	// Bits 496
			m_dbuf[2] = 0x00;	// Bits 495:488, Support Function grp 6
			m_dbuf[3] = 0x01;	// Bits 487:480
			m_dbuf[4] = 0x00;	// Bits 479:472, Support Fn gp 5
			m_dbuf[5] = 0x01;	// Bits 471:464
			m_dbuf[6] = 0x00;	// Bits 463:456, Support Fn gp 4
			m_dbuf[7] = 0x1f;	// Bits 455:448
			m_dbuf[8] = 0x00;	// Bits 447:440, Support Fn gp 3
			m_dbuf[9] = 0x0f;	// Bits 439:432
			m_dbuf[10]= 0x00;	// Bits 431:424, Support Fn gp 2
			m_dbuf[11]= 0x01;	// Bits 423:416
			m_dbuf[12]= 0x00;	// Bits 415:408, Support Fn gp 1
			m_dbuf[13]= 0x1f;	// Bits 407:400
			m_dbuf[14]= 0x000;	// Bits 399:392, FuncSel, grp6-5
			m_dbuf[15]= 0x000;	// Bits 391:384, FuncSel, grp4-3
			m_dbuf[16]= 0x000;	// Bits 383:376, FuncSel, grp2-1
			m_dbuf[17]= 0x001;	// Bits 375:368,

			appendcrc(64);
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
		if (m_selected) {
			load_reply(7,m_R1);
			m_R1 &= ~(COM_CRC_ERROR);
		}
		break;
		// }}}
	case 8: // SEND_IF_COND
		// {{{
		assert(valid_crc);

		m_app_cmd = 0;

		load_reply(8,0x0100 | (arg & 0x0ff));
		break;
		// }}}
	case 9: // SEND CSD
		// {{{
		m_app_cmd = 0;
		if (m_RCA == (arg >> 16)) {
			m_reply_active = 1;
			m_reply_buf[0] = 9;
			for(unsigned k=0; k<128/8; k++)
				m_reply_buf[k+1] = m_csd[k];
			m_reply_buf[1+128/8] = cmdcrc(1+128/8, &m_reply_buf[0]);

			m_reply_posn = 0;
			m_reply_count = 8+128+8;
			m_reply_delay = 0;
		} else {
			printf("SDIOSIM:Ignoring CMD#9, RCA=%04x, arg=%08x\n",
				m_RCA, arg);

		} break;
		// }}}
	case 10: // SEND CID
		// {{{
		m_app_cmd = 0;
		if (m_RCA == (arg >> 16)) {
			m_reply_active = 1;
			m_reply_buf[0] = 10;
			for(unsigned k=0; k<128/8; k++)
				m_reply_buf[k+1] = m_cid[k];
			m_reply_buf[1+128/8] = cmdcrc(128/8, &m_reply_buf[0]);

			m_reply_posn = 0;
			m_reply_count = 8+128+8;
			m_reply_delay = 0;
		// } else {
			// printf("SDIOSIM:Ignoring CMD#10, RCA=%04x, arg=%08x\n", m_RCA, arg);

		} break;
		// }}}
	case 11: // VOLTAGE_SWITCH
		// {{{
		// We only support 3.3V (for now)
		m_app_cmd = 0;
		break;
		// }}}
	case 12: // STOP_TRANSMISSION
		// {{{
		m_sector = 0;
		m_drive = 0;
		m_data_posn = 8*DBUFLN;
		m_multiblock = false;
		m_data_started = false;
		load_reply(12,m_R1);
		m_R1 &= ~(COM_CRC_ERROR);
		break;
		// }}}
	// case 16: // SET_BLOCKLEN	-- We only support 512B blocks
	case 17: // READ_SINGLE_BLOCK
		// {{{
		m_app_cmd = 0;
		if (m_selected) {
			__attribute__((unused)) size_t	sz;
			m_drive = 1;
			load_reply(17,m_R1);
			m_R1 &= ~(COM_CRC_ERROR);

			m_sector     = arg;
			if (!HCS)
				m_sector >>= 9;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;
			m_multiblock = false;

			(void)fseek(m_fp, (long)(m_sector * SECTORSZ), SEEK_SET);
			sz = fread(m_dbuf, sizeof(char), SECTORSZ, m_fp);
			for(unsigned k=SECTORSZ; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			appendcrc(SECTORSZ);
			assert(m_busy_cycles == 0);
		} break;
		// }}}
	case 18: // READ_MULTIPLE_BLOCK
		// {{{
		m_app_cmd = 0;
		if (m_selected) {
			__attribute__((unused)) size_t	sz;
			m_drive = 1;
			load_reply(18,m_R1);
			m_R1 &= ~(COM_CRC_ERROR);

			m_sector     = arg;
			if (!HCS)
				m_sector >>= 9;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;
			m_multiblock = true;

			(void)fseek(m_fp, (long)(m_sector * SECTORSZ), SEEK_SET);
			sz = fread(m_dbuf, sizeof(char), SECTORSZ, m_fp);
			m_sector++;
			for(unsigned k=SECTORSZ; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			appendcrc(SECTORSZ);
			assert(m_busy_cycles == 0);
		} break;
		// }}}
	// case 19: // SEND_TUNING_BLOCK
	// case 20: // SPEED_CLASS_CONTROL
	// case 23: // SET_BLOCK_COUNT
	case 24: // WRITE_BLOCK
		// {{{
		m_app_cmd = 0;
		if (m_selected) {
			if (m_debug)
				printf("SDIOSIM::WRITE-BLOCK (data_posn was %d)\n", m_data_posn);
			m_drive = 0;
			load_reply(24,m_R1);
			m_R1 &= ~(COM_CRC_ERROR);

			m_sector     = arg;
			if (!HCS)
				m_sector >>= 9;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;
			m_data_started = false;
			m_multiblock = false;

			assert(m_busy_cycles == 0);
			// m_data_count = SECTORSZ;
		} break;
		// }}}
	case 25: // WRITE_MULTIPLE_BLOCK
		// {{{
		m_app_cmd = 0;
		if (m_selected) {
			if (m_debug)
				printf("SDIOSIM::WRITE-MULTI-BLOCK (data_posn was %d)\n", m_data_posn);
			m_drive = 0;
			load_reply(25,m_R1);
			m_R1 &= ~(COM_CRC_ERROR);

			m_sector     = arg;
			if (!HCS)
				m_sector >>= 9;
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;
			m_data_started = false;
			m_multiblock = true;
			assert(m_busy_cycles == 0);
		} break;
		// }}}
	case 41: // ACMD41: SEND_OP_COND
		if (m_app_cmd) { // ACMD41
		// {{{
			if (m_selected) {
				unsigned opcond = 0x00ff8000;

				if (HCS)
					opcond |= 0x40000000;
				if (arg == 0)
					opcond = 0x0ff8000;
				else
					opcond &= arg;

				if (arg != 0 && (!HCS || (opcond & 0x40000000)))
					// Release from busy immediatley
					//   HCS cards never leave this state
					//   if the busy bit isn't set
					opcond |= 0x80000000;
				load_reply(41, opcond);
			} m_app_cmd = 0;
		// } else {
			// The spec does not define any CMD41
		} break;
		// }}}
	case 51: // ACMD51: SEND_SCR
		if (m_app_cmd) {
		// {{{
			if (m_selected) {
				__attribute__((unused)) size_t	sz;
				m_drive = 1;
				load_reply(51,m_R1);
				m_R1 &= ~(COM_CRC_ERROR);

				m_sector     = 0;
				m_data_delay = rand() & 1023;
				m_data_posn  = 0;

				for(unsigned k=0; k<8; k++)
					m_dbuf[k] = m_scr[k];

				appendcrc(64/8);
			} else printf("SDIOSIM::SEND-SCR Card not selected\n");
			m_app_cmd = 0;
		} break;
		// }}}
	case 55: // APP_CMD
		// {{{
		if (m_selected && (m_RCA == (arg >> 16))) {
			m_app_cmd = 1;
			load_reply(55, m_R1 | 0x20);
			m_R1 &= ~(COM_CRC_ERROR);
		} break;
		// }}}
	default:
		printf("SDIOSIM:Unknown command CMD#%d\n",m_cmd_buf[0] & 0x03f);
		break;
	}

	if (55 != (m_cmd_buf[0] & 0x03f)) {
		// printf("SDIOSIM::Reset APP-CMD\n");
		m_app_cmd = 0;
	}
	m_reply_delay += 8;
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
			if (cmdcrc(5, m_cmd_buf) != (0x0ff & m_cmd_buf[5])) {
				printf("SDIOSIM::CMD CRC-FAILURE! ... %02x:%02x%02x%02x%02x,%02x\n",
					m_cmd_buf[0]&0x0ff, m_cmd_buf[1]&0x0ff,
					m_cmd_buf[2]&0x0ff, m_cmd_buf[3]&0x0ff,
					m_cmd_buf[4]&0x0ff, m_cmd_buf[5]&0x0ff);
			} else if ((m_cmd_buf[0] & 0x0c0) != 0x040) {
				printf("SDIOSIM::CMD FRAME FAILURE: %02x\n",
					m_cmd_buf[0] & 0x0ff);
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
	if (m_drive) {	// Read operations
		// {{{
		unsigned	r;

		if (m_data_delay > 0) {
			m_data_delay--;
			return 0x0ff;
		}

		if (m_data_posn >= 8*DBUFLN) {
			r = 0x0ff;
		} else if (m_width == 8) {
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

		if (m_open_drain) {
			return r & in;
		} else {
			return r;
		}
		// }}}
	} else { // Write (to card) operations
		if (m_busy_cycles > 0) { // Busy bits
			// {{{
			m_busy_cycles--;

			if (m_open_drain) {
				return 0xfe & in;
			} else {
				return 0xfe;
			}
			// }}}
		} else if (!m_data_started) { // Look for a start bit
			// {{{
			if (m_width >= 8) {
				if (0 == in)
					m_data_started = true;
			} else if (m_width >= 4) {
				if (0 == (in & 0x0f))
					m_data_started = true;
			} else if (0 == (in & 0x01))
				m_data_started = true;
			// }}}
		} else if (m_data_posn < SECTORSZ*8 + 8*16) { // Receive data
			// {{{
			unsigned	idx = m_data_posn >> 3;

			// if (m_debug) printf("SDIOSIM: RCV %02x\n", in);
			if (m_width >= 8) {
				m_dbuf[idx] = in & 0x0ff;
				m_data_posn += 8;
			} else if (m_width >= 4) {
				unsigned	tmp;

				tmp = m_dbuf[idx];
				if (0 == (m_data_posn & 4))
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

			if (m_data_posn == SECTORSZ*8 + (16 * m_width)) {
				// {{{
				bool		crc_fail = false;
				unsigned	fill;

				// Check CRCs
				if (m_width >= 8) {
					// {{{
					for(unsigned w=0; w<8 && !crc_fail; w++) {
						fill = 0;
						for(unsigned k=0; k<m_data_posn; k+=8) {
							unsigned b;
							b = (m_dbuf[k>>3] >> w) & 1;
							fill = blockcrc(fill, b);
						} if (fill != 0)
							crc_fail = true;
					}
					// }}}
				} else if (m_width >= 4) {
					// {{{
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
					// }}}
				} else { // if (m_width == 1)
					// {{{
					fill = 0;
					for(unsigned k=0; k<m_data_posn; k++) {
						unsigned	b;

						b = (m_dbuf[k>>3] >> (7-(k&7))) & 1;
						fill = blockcrc(fill, b);
					}
					crc_fail = (fill != 0);
					// }}}
				}

				if (!crc_fail) {
					long	offset = m_sector * SECTORSZ;

					m_crctoken |= CRCACK << 16;
					if (0 != fseek(m_fp, offset, SEEK_SET)) {
						perror("FSEEK O/S Err:");
					}
					if (m_debug)
					printf( "SDIOSIM-P: Writing sector %04x @0x%08lx\n", m_sector, offset);
					if (SECTORSZ != fwrite(m_dbuf, sizeof(char), SECTORSZ, m_fp)) {
						perror("FWRITE O/S Err:");
					}
					m_sector++;

					if (false && m_debug)
					for(unsigned k=0; k<SECTORSZ; k++) {
						printf("%02x", m_dbuf[k] & 0x0ff);
						if (15 == (k&15))
							printf("\n");
						else if (7 == (k&7))
							printf("  ");
						else
							printf(" ");
					}

				} else {
					m_crctoken |= CRCNAK << 16;
					printf("SDIOSIM-P: Bad-CRC\n");
					m_R1 |= COM_CRC_ERROR;
					if (m_debug) {
					for(unsigned k=0; k<m_data_posn; k+=8) {
						printf("%02x", m_dbuf[k/8] & 0x0ff);
						if (0 == ((k+8) & 127)) {
							printf("\n");
						} else if (0 == ((k+8) & 63)) {
							printf("  ");
						} else printf(" ");
					}}
				}

				if (m_multiblock) {
					// printf( "SDIOSIM-P: Expecting next block (multi-block write)\n");
					m_data_posn = 0;
					m_data_started = false;
					m_busy_cycles = 32 + (rand() & 1023);
					m_data_started = false;
				}

				// }}}
			}

			return in;
			// }}}
		}
	}

	return 0x0ff;
}
// }}}

unsigned SDIOSIM::datn(unsigned in) {
	// {{{
	if (!m_ddr) {
		if (m_busy_cycles > 0) {
			if (m_open_drain) {
				return 0xfe & in;
			} else {
				return 0xfe;
			}
		} else
			return in;
	}

	if (m_drive) { // Read operations
		// {{{
		unsigned	r = 0;

		if (m_data_delay > 0) {
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

		if (m_open_drain)
			return r & in;
		else
			return r;
		// }}}
	} else { // Write operations
		if (m_busy_cycles > 0) { // Busy bits
			// {{{
			// m_busy_cycles--; Count busy on positive edges only

			if (m_open_drain) {
				return 0xfe & in;
			} else {
				return 0xfe;
			}
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
		} else if (m_data_posn < SECTORSZ*8 + 2*m_width*16) { // Receive a block of data
			// {{{
			unsigned	idx = m_data_posn >> 3;
			if (m_width >= 8) {
				m_dbuf[idx] = in & 0x0ff;
				m_data_posn += 8;
			} else if (m_width >= 4) {
				// {{{
				unsigned	tmp;

				tmp = m_dbuf[idx];
				if (m_data_posn & 4)
					m_dbuf[idx] = (in << 4);
				else
					m_dbuf[idx] = (tmp & 0x0f0) | (in & 0x0f);
				m_data_posn += 4;
				// }}}
			} else {
				// {{{
				unsigned	tmp;

				in &= 1;
				tmp = m_dbuf[idx];
				if (0 == (m_data_posn & 7))
					m_dbuf[idx] = (in << 7);
				else
					m_dbuf[idx] = tmp | (in << (7-(m_data_posn&7)));
				m_data_posn += 1;
				// }}}
			}

			if (m_data_posn == SECTORSZ*8 + (32 * m_width)) {
				// {{{ // Check CRC
				bool		crc_fail = false;
				unsigned	fill = 0;

				// Check CRCs
				if (m_width >= 8) {
					for(unsigned w=0; w<8 && !crc_fail; w++) {
						fill = 0;
						for(unsigned k=0; k<m_data_posn; k+=16){
							unsigned b;

							b = (m_dbuf[k>>3] >> w) & 1;
							fill = blockcrc(fill, b);
						} if (fill != 0)
							crc_fail = true;
					}

					for(unsigned w=0; w<8 && !crc_fail; w++) {
						fill = 0;
						for(unsigned k=0; k<m_data_posn; k+=16){
							unsigned b;

							b = (m_dbuf[1+(k>>3)] >> w) & 1;
							fill = blockcrc(fill, b);
						} if (fill != 0)
							crc_fail = true;
					}
				} else if (m_width >= 4) {
					for(unsigned w=0; w<8 && !crc_fail; w++) {
						fill = 0;
						for(unsigned k=0; k<m_data_posn; k+=8) {
							unsigned v = m_dbuf[k>>3] >> w;

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
					m_crctoken |= CRCACK << 16;
					(void)fseek(m_fp, (long)(m_sector * SECTORSZ), SEEK_SET);
					if (m_debug) printf("SDIOSIM-N: Writing sector %04x\n", m_sector);
					(void)fwrite(m_dbuf, sizeof(char), SECTORSZ, m_fp);
					m_sector++;

				} else {
					m_crctoken |= CRCNAK << 16;
					printf("SDIOSIM-N: Bad-CRC\n");
					m_R1 |= COM_CRC_ERROR;
				}

				if (m_multiblock) {
					// printf( "SDIOSIM-N: Expecting next block (multi-block write)\n");
					m_data_posn = 0;
					m_data_started = false;
					m_busy_cycles = 32 + (rand() & 1023);
					m_data_started = false;
				}
				// }}}
			}
			return in;
			// }}}
		}
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
	if (cmd_en)  { m_cmd_started = 0; m_reply_started = false; };
	if (!data_en && !rx_en) {
		// if (m_drive) printf("M-DRIVE && !RX-EN!\n");
		m_drive = m_drive && m_multiblock;
		if (m_drive && m_data_started && m_data_posn > 0) {
			__attribute__((unused)) size_t	sz;

			if (m_debug)
				printf("SDIOSIM: Multi-block read reload\n");
			// Load the next sector
			m_data_delay = rand() & 1023;
			m_data_posn  = 0;

			sz = fread(m_dbuf, sizeof(char), SECTORSZ, m_fp);
			m_sector++;
			for(unsigned k=SECTORSZ; k<DBUFLN; k++)
				m_dbuf[k] = 0x0ff;
			appendcrc(SECTORSZ);
		} m_data_started = 0;
	}

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
			if (data_en)
				m_crctoken &= ~0x0ff;
			else
				m_crctoken >>= 2;
		} else if (ddr && edge == 2) {
			rstb = (rstb << 1) | 1;
			rdat = (rdat << 8) | (datn(idat) & 0x0ff);
			rlsb = (rlsb << 1) | (rdat&1);
		}
	}
	// }}}

	// Pack the command return
	// {{{
	if (cstb) {
		m_last_cmd = cmd & 1;
		if (!m_reply_started && 0 == (cmd & 1))
			m_reply_started = true;
		if (!m_reply_started)
			cstb = 0;
	} if (!m_cmd_started && (cstb != 0) && (cstb != (cmd & cstb))) {
		if (cmd_en)
			cstb = 0;
		else {
			if (cmd & 2)
				cstb &= ~2;
			m_cmd_started = 1;
		}
	} if (1 == (cstb & 3)) {
		cstb <<= 1;
		cmd  <<= 1;
	} if (cmd_en) {
		cstb = 0;
		m_reply_started = false;
	}
	// }}}

	// Pack the return data strobe
	// {{{
	if (!data_en && !rx_en) {
		rstb = 0;
		m_data_started = false;

	} else if (!m_data_started && rx_en) {
		/*
		if (m_drive && m_data_delay <= 1)
			printf("RSTB & RLSB = %x & %x = %x, MSK = %x\n",
				rstb, rlsb, (rstb & rlsb)&0x0f,
				(rstb & ~rlsb)&0x0f);
		*/
		if (rstb != (rstb & rlsb)) {
			unsigned	msk = (rstb & ~rlsb) & 0x0f;

			if (msk&0x08) {
				rstb &= 0x07;
			} else if (msk &0x04 ) {
				rstb <<= 1;
				rdat <<= 8;
				rstb &= 0x03;
			} else if (msk &0x02 ) {
				rstb <<= 2;
				rdat <<= 16;
				rstb &= 0x01;
			} else if (msk & 0x01 ) {
				rstb <<= 3;
				rdat <<= 24;
				rstb = 0;
			} if (msk)
				m_data_started = 1;
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
	} if (!m_data_started) {
		rstb = 0;
	}
	// }}}

	// Synchronous return can only return two clocks of data
	o_sync = (cstb << 30) | (cmd << 28) | ((rstb & 0x0c) << 22) | (rdat >> 16);

	async_sync = (cstb & 2) | ((rstb == 0x0f) ? 1:0);
	async_data = rdat;
}
// }}}
