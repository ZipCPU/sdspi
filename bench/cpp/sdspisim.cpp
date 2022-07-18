////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	sdspisim.cpp
// {{{
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
// }}}
// Copyright (C) 2015-2022, Gisselquist Technology, LLC
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
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "sdspisim.h"

static	const unsigned
	MICROSECONDS = 80, // Clocks in a microsecond
	MILLISECONDS = MICROSECONDS * 1000,
	tRESET = 4*MILLISECONDS, // Just a wild guess
	LGSECTOR_SIZE = 9,
	SECTOR_SIZE = (1<<LGSECTOR_SIZE);

static	const	unsigned
	CCS = 1; // 0: SDSC card, 1: SDHC or SDXC card

SDSPISIM::SDSPISIM(const bool debug) {
	// {{{
	m_dev = NULL;
	m_last_sck = 1;
	m_block_address = (CCS==1);
	m_host_supports_high_capacity = false;
	m_powerup_busy = -1;
	m_reset_state = SDSPI_POWERUP_RESET;
	//
	CSD();	// assign values to m_csd

	// CID Register
	CID();	// Assigne values to m_cid, our CID register

	// m_write_count = 0;
	// m_ireg = m_oreg = 0;
	// m_sreg = 0x01c;
	// m_creg = 0x001;	// Iinitial creg on delivery

	//
	m_reading_data = false;
	m_have_token = false;
	m_debug = debug;
}
// }}}

void	SDSPISIM::load(const char *fname) {
	// {{{
	m_dev = fopen(fname, "r+b");

	if (m_dev) {
		unsigned long devln;
		fseek(m_dev, 0l, SEEK_END);
		devln = ftell(m_dev);
		fseek(m_dev, 0l, SEEK_SET);

		m_devblocks = devln>>LGSECTOR_SIZE;

		if (m_debug) printf("SDCARD: NBLOCKS = %ld\n", m_devblocks);
	}
}
// }}}

unsigned	SDSPISIM::read_bitfield(int offset, int bits,
				int ln, const uint8_t *bitfield) {
	// {{{
	// unsigned	total = 8*ln;
	// unsigned	index = total-offset;
	return 0;
}
// }}}

unsigned	SDSPISIM::OCR(void) {
	// {{{
	unsigned	ocr = 0x00ff80;

	if (CCS)
		ocr |= 0x400000;
	if (m_powerup_busy)
		ocr |= 0x800000;

	return ocr;
}
// }}}

void	SDSPISIM::CSD(void) {
	// {{{
	static const uint8_t __attribute__((unused)) SYNTHETIC_CSD[] = {
		// Normal SD Card, not high capacity
		0x00,
		0x0f, 0x0f,
		// Can be either 0x32 (25MHz) or 0x5a (50MHz)
		0x32,
		// Could also be 0x7b, if we supported more commands
		0x5b,
		// 9 -> 2^9 or 512 byte blocks (10 -> 1024, 11-> 2048, no othrs)
		0x59,
		//  Partial blocks allowed?
		0x00,
		// C_SIZE, 2'b00, then top 6 bits
		0x00,
		// C_SIZE, 22-bits, mid 8-bits
		0x00,
		// C_SIZE, 22-bits, bottom 8-bits
		0x00,
		0x7f, 0x80,
		0x0a, 0x40, 0 };
	static const uint8_t	__attribute__((unused)) LEXAR_CSD[] = {
		0x40, 0x0e, 0x00, 0x32,
		0xdb, 0x79, 0x00, 0x01,
		0xd7, 0x03, 0x7f, 0x80,
		0x0a, 0x40, 0x00
		};
	static const uint8_t	__attribute__((unused)) SANDISK_CSD[] = {
		0x30, 0x0e, 0x00, 0x32,
		0x5b, 0x59, 0x00, 0x07,
		0x72, 0x5f, 0x7f, 0x80,
		0x0a, 0x40, 0x40
		};
#define	DEFAULT_CSD	LEXAR_CSD
// #define	DEFAULT_CSD	SANDISK_CSD
	assert(sizeof(DEFAULT_CSD) == SDSPI_CSDLEN-1);

	for(int k=0; k<15; k++)
		m_csd[k] = DEFAULT_CSD[k];

	//
	// Adjust the C_SIZE field for the correct number of blocks
	//

	// The following code works on high-capacity cards only
	assert((DEFAULT_CSD[0] & 0x0c0)== 0x40);

	m_csd[7] = (m_devblocks >> 16) & 0x03f;
	m_csd[8] = (m_devblocks >>  8) & 0x0ff;
	m_csd[9] = (m_devblocks      ) & 0x0ff;

	//
	// Now make the CRC match

	m_csd[15] = cmdcrc(15, (char *)m_csd);
}
// }}}

uint8_t	SDSPISIM::CSD(int index) {
	// {{{
	assert(index >= 0);
	assert(index < SDSPI_CSDLEN);
	return m_csd[index];
}
// }}}

void	SDSPISIM::CID(void) {
	// {{{
	// CID Reg
	static const uint8_t __attribute__((unused)) SYNTHETIC_CID[] = {
		0xba, 0xd0, 0xda, 0xdd,
		0x00, 0xde, 0xad, 0xbe,
		0xef, 0x20, 0x16, 0x05,
		0x26, 0x00, 0x00 };
	static const uint8_t __attribute__((unused)) LEXAR_CID[] = {
		0x9c, 0x53, 0x4f, 0x4c,
		0x58, 0x36, 0x34, 0x47,
		0x10, 0x29, 0x80, 0x03,
		0x7b, 0x01, 0x38
	};
	static const uint8_t __attribute__((unused)) SANDISK_CID[] = {
		0x03, 0x53, 0x44, 0x53,
		0x43, 0x32, 0x35, 0x36,
		0x80, 0xef, 0x4a, 0x3b,
		0x4a, 0x01, 0x39
	};
// #define	DEFAULT_CID	SANDISK_CID
#define	DEFAULT_CID	LEXAR_CID
	assert(sizeof(DEFAULT_CID) == SDSPI_CIDLEN-1);

	for(int k=0; k<15; k++)
		m_cid[k] = DEFAULT_CID[k];
	m_cid[15] = cmdcrc(15, (char *)m_cid);
};
// }}}

uint8_t	SDSPISIM::CID(int index) {
	// {{{
	assert(index >= 0);
	assert(index < SDSPI_CIDLEN);
	return m_cid[index];
}
// }}}

int	SDSPISIM::operator()(const int csn, const int sck, const int mosi) {
	// {{{
	// Keep track of a timer to determine when page program and erase
	// cycles complete.

	/*
	if (m_write_count > 0) {
		//
	}
	*/

	m_delay++;
	if (m_powerup_busy>0)
		m_powerup_busy--;

	if (csn) {
		// {{{
		m_delay = 0;
		m_cmdidx= 0;
		m_rspidx= 0;
		m_bitpos= 0;
		m_delay = 0;
		m_busy  = false;
		m_last_sck = sck;
		m_syncd = false;
		m_last_miso = 1;
		m_dat_out = 0x0ff;
		// Reset everything when not selected
		return 0;
		// }}}
	} else if (sck == m_last_sck) {
		// {{{
		m_last_sck = sck;
		return m_last_miso;
		// }}}
	} else if (!m_last_sck) {
		// {{{
		// Register our input on the rising edge
		m_mosi = mosi;
		m_syncd= true;
		m_last_sck = sck;
		return m_last_miso;
		// }}}
	} if (!m_syncd) {
		m_last_sck = sck;
		return m_last_miso;
	}

	// Only change our output on the falling edge

	m_last_sck = sck;
	// if (m_debug) printf("SDSPI: (%3d) [%d,%d,%d] ", m_delay, csn, sck, m_mosi);
	// assert(m_delay > 20);

	m_bitpos++;
	m_dat_in = (m_dat_in<<1)|m_mosi;


	// if (m_debug) printf("(bitpos=%d,dat_in=%02x)\n", m_bitpos&7, m_dat_in&0x0ff);

	if ((m_bitpos&7)==0) {
		// {{{
		// if (m_debug) printf("SDSPI--RX BYTE %02x\n", m_dat_in&0x0ff);
		m_dat_out = 0xff;
		if (m_reading_data) {
			// {{{
			if (m_have_token) {
				// {{{
				m_block_buf[m_rxloc++] = m_dat_in;
				// if (m_debug) printf("SDSPI: WR[%3d] = %02x\n", m_rxloc-1,
				//	m_dat_in&0x0ff);
				if (m_rxloc >= (unsigned)SECTOR_SIZE+2) {
					unsigned crc, rxcrc;

					crc = blockcrc(SECTOR_SIZE, m_block_buf);
					rxcrc = ((m_block_buf[SECTOR_SIZE]&0x0ff)<<8)
						|(m_block_buf[SECTOR_SIZE+1]&0x0ff);

					if (m_debug) printf("LEN = %d\n", m_rxloc);
					if (m_debug) printf("CHECKING CRC: (rx) %04x =? %04x (calc)\n",
						crc, rxcrc);
					m_reading_data = false;
					m_have_token = false;
					if (rxcrc == crc) {
						m_dat_out = 5;
						if (m_dev) {
							fwrite(m_block_buf, 1, SECTOR_SIZE, m_dev);
							fflush(m_dev);
						}
					} else {
						m_dat_out = 0x0b;
						printf("RXCRC Err!  %04x != %04x\n", rxcrc, crc);
						assert(rxcrc == crc);
					}
				}
				// }}}
			} else {
				// {{{
				if ((m_dat_in&0x0ff) == 0x0fe) {
					if (m_debug) printf("SDSPI: TOKEN!!\n");
					m_have_token = true;
					m_rxloc = 0;
				} else if (m_debug)
					printf("SDSPI: waiting on token\n");
				// }}}
			}
			// }}}
		} else if (m_cmdidx < 6) {
			// {{{
			// if (m_debug) printf("SDSPI: CMDIDX = %d\n",m_cmdidx);
			// All commands *must* start with a 01... pair of bits.
			if (m_cmdidx == 0)
				assert((m_dat_in&0xc0)==0x40);

			// Record the command for later processing
			m_cmdbuf[m_cmdidx++] = m_dat_in;
			// }}}
		} else if (m_cmdidx == 6) { // Command processing
			// {{{
			// We're going to start a response from here ...
			m_rspidx = 0;
			m_blkdly = 0;
			m_blkidx = SDSPI_MAXBLKLEN;
			if (m_debug) {
				printf("SDSPI: CMDIDX = %d -- WE HAVE A COMMAND #%2d! [ ", m_cmdidx, m_cmdbuf[0]&0x3f);
				for(int i=0; i<6; i++)
					printf("%02x ", m_cmdbuf[i] & 0xff);
				printf("]\n"); fflush(stdout);
			}

			unsigned	arg;
			arg = ((((((m_cmdbuf[1]<<8)|(m_cmdbuf[2]&0x0ff))<<8)
				|(m_cmdbuf[3]&0x0ff))<<8)
				|(m_cmdbuf[4]&0x0ff));
			arg &= 0x0ffffffff;

			// Check the CRC
			if (!check_cmdcrc(m_cmdbuf)) {
				assert(0 && "BAD CRC");
				m_rspbuf[0] = 0x09;
				m_rspdly = 1;
			} else if (m_altcmd_flag) { // Alternate commands
				// {{{
				switch(m_cmdbuf[0]&0x03f) {
				case 41: // ACMD41 -- SD_SEND_OP_COND
					// and start initialization sequence
					assert((m_reset_state == SDSPI_RCVD_CMD8)||(m_reset_state == SDSPI_RCVD_ACMD41)||(m_reset_state == SDSPI_RESET_COMPLETE));
					if((unsigned)m_powerup_busy>tRESET)
						m_powerup_busy = tRESET;
					assert((arg&0x0bfffffff) == 0);
					m_rspbuf[0] = (m_powerup_busy)?1:0;
					m_rspdly = 2;
					m_host_supports_high_capacity = (m_cmdbuf[1]&0x40)?1:0;
					m_reset_state = (m_powerup_busy)?
						SDSPI_RCVD_ACMD41
						:SDSPI_RESET_COMPLETE;
					break;
				case 51: // ACMD51
					m_block_buf[0] = 0x0fe;
					for(int j=0; j<8; j++)
						m_block_buf[j+1] = m_csd[j];
					m_blklen = 8;
					add_block_crc(m_blklen, m_block_buf);

					m_blkdly = 0;
					m_blkidx = 0;
					m_dat_out = 0;
					break;
				case 13: // ACMD13
				case 22: // ACMD22
				case 23: // ACMD23
				case 42: // ACMD42
				default: // Unimplemented command!
					m_rspbuf[0] = 0x04;
					m_rspdly = 4;
					fprintf(stderr, "SDSPI ERR: Alt command ACMD%d not implemented!\n", m_cmdbuf[0]&0x03f);
					assert(0 && "Not Implemented");
				} m_altcmd_flag = false;
				// }}}
			} else { // Regular command processing
				// {{{
				m_altcmd_flag = false;
				memset(m_rspbuf, 0x0ff, SDSPI_RSPLEN);
				if (m_debug) printf("SDSPI: Received a command 0x%02x (%d)\n",
					m_cmdbuf[0], m_cmdbuf[0]&0x03f);
				switch(m_cmdbuf[0]&0x3f) {
				case  0: // CMD0  -- GO_IDLE_STATE
					m_rspbuf[0] = 0x01;
					m_rspdly = 4;
					m_reset_state = SDSPI_CMD0_IDLE;
					break;
				case  1: // CMD1  -- SEND_OP_COND
					assert((arg&0x0bfffffff) == 0);
					m_rspbuf[0] = 0x02;
					m_rspdly = 4;
					m_host_supports_high_capacity = (m_cmdbuf[1]&0x40)?1:0;
					break;
				case  8: // CMD8  -- SEND_IF_COND
					assert((arg&0x0fffff000) == 0);
					m_rspbuf[0] = 0x00;
					// See p82 for this format
					m_rspbuf[1] = 0;
					m_rspbuf[2] = 0;
					// If we do not accept the voltage range
					m_rspbuf[3] = 0;
					// Now, check if we accept it
					// We only accept 2.7-3.6V in this
					// simulation.
					if ((arg&0x0f00)==0x0100)
						m_rspbuf[3] = 1;
					m_rspbuf[4] = (char)(arg&0x0ff);
					m_rspdly = 4;
					assert((m_reset_state == SDSPI_CMD0_IDLE)||(m_reset_state == SDSPI_RCVD_CMD8));
					m_reset_state = SDSPI_RCVD_CMD8;
					break;
				case  9: // CMD9  -- SEND_CSD
					// Block read, returning start token,
					// 16 bytes, then 2 crc bytes
					assert(m_reset_state == SDSPI_IN_OPERATION);
					m_rspbuf[0] = 0x00;
					memset(m_block_buf, 0x0ff, SDSPI_MAXBLKLEN);
					m_block_buf[0] = 0x0fe;
					for(int j=0; j<16; j++)
						m_block_buf[j+1] = m_csd[j];
					m_blklen = 16;
					add_block_crc(m_blklen, m_block_buf);

					m_blkdly = 60;
					m_blkidx = 0;
					break;
				case 10: // CMD10 -- SEND_CID
					// Block read, returning start token,
					// 16 bytes, then 2 crc bytes
					assert(m_reset_state == SDSPI_IN_OPERATION);
					m_rspbuf[0] = 0x00;
					memset(m_block_buf, 0x0ff, SDSPI_MAXBLKLEN);
					m_block_buf[0] = 0x0fe;
					for(int j=0; j<16; j++)
						m_block_buf[j+1] = m_cid[j];
					m_blklen = 16;
					add_block_crc(m_blklen, m_block_buf);

					m_blkdly = 60;
					m_blkidx = 0;
					break;
				case 13: // CMD13 -- SEND_STATUS
					assert(m_reset_state == SDSPI_IN_OPERATION);
					m_rspbuf[0] = 0x00;
					m_rspbuf[1] = 0x00;
					// if (m_wp_fault) m_rspbuf[1]|=0x20;
					// if (m_err) m_rspbuf[1]|=0x04;
					m_rspdly = 4;
					break;
				case 17: // CMD17 -- READ_SINGLE_BLOCK
					assert(m_reset_state == SDSPI_IN_OPERATION);
					m_rspbuf[0] = 0x00;
					memset(m_block_buf, 0x0ff, SDSPI_MAXBLKLEN);
					if (m_dev) {
						if (m_debug) printf("Reading from block %08x of %08lx\n", arg, m_devblocks);
						if (m_block_address) {
							assert(arg < m_devblocks);
							fseek(m_dev, arg<<LGSECTOR_SIZE, SEEK_SET);
fprintf(stderr, "READ: Seek to sector %d\n", arg);
						} else {
							assert(arg < m_devblocks<<9);
							fseek(m_dev, arg, SEEK_SET);
						}
					} m_block_buf[0] = 0x0fe;
					m_blklen = SECTOR_SIZE; //(1<<m_csd[5]);
					if (m_dev)
						m_blklen = fread(&m_block_buf[1], m_blklen, 1, m_dev);
					else
						memset(&m_block_buf[1], 0, m_blklen);
					m_blklen = (m_blklen != SECTOR_SIZE) ? SECTOR_SIZE : m_blklen;
					add_block_crc(m_blklen, m_block_buf);

					m_blkdly = 60;
					m_blkidx = 0;
					break;
				case 24: // CMD24 -- WRITE_BLOCK
					if (m_dev) {
						if (m_debug) printf("Going to write to block %08x of %08lx\n", arg, m_devblocks);
						if (m_block_address) {
							assert(arg < m_devblocks);
fprintf(stderr, "WRITE: Seek to sector %d\n", arg);
							fseek(m_dev, arg<<LGSECTOR_SIZE, SEEK_SET);
						} else {
							assert(arg < m_devblocks<<9);
							fseek(m_dev, arg, SEEK_SET);
						}
					}
					m_reading_data = true;
					m_have_token = false;
					m_dat_out = 0;
					break;
				case 55: // CMD55 -- APP_CMD
					m_rspbuf[0] = 0x00;
					m_rspdly = 2;
					m_altcmd_flag = true;
					break;
				case 58: {// CMD58 -- READ_OCR, respond R7
					// argument is stuff bits/dont care
					unsigned	ocr = OCR();
					m_rspbuf[0] = 0x00;	// R1, no errs
					m_rspbuf[1] = (ocr >> 24)&0x0ff;
					// See p112, Tbl 5-1 for this format
					// m_rspbuf[1] = ((m_powerup_busy)?0x80:0)
					//		|(CCS?0x40:0);
					m_rspbuf[2] = (ocr >> 16)&0x0ff;
					m_rspbuf[3] = (ocr >>  8)&0x0ff;
					m_rspbuf[4] = (ocr      )&0x0ff;
					// m_rspbuf[2] = 0xff;// 2.7-3.6V supported
					// m_rspbuf[3] = 0x80;
					// m_rspbuf[4] = 0; // No low-voltage supt
					m_rspdly = 4;
					if (m_reset_state == SDSPI_RESET_COMPLETE)
						m_reset_state = SDSPI_IN_OPERATION;
					}
					break;
				case  6: // CMD6  -- SWITCH_FUNC
				case 12: // CMD12 -- STOP_TRANSMISSION (!impl)
				case 16: // CMD16 -- SET_BLOCKLEN
				case 18: // CMD18 -- READ_MULTIPLE_BLOCK
				case 25: // CMD25 -- WRITE_MULTIPLE_BLOCK
				case 27: // CMD27 -- PROGRAM_CSD
				case 32: // CMD32 -- ERASE_WR_BLK_START_ADDR
				case 33: // CMD33 -- ERASE_WR_BLK_END_ADDR
				case 38: // CMD38 -- ERASE
				case 56: // CMD56 -- GEN_CMD
				default: // Unimplemented command
					m_rspbuf[0] = 0x04;
					m_rspdly = 4;
					if (m_debug) printf("SDSPI ERR: Command CMD%d not implemented!\n", m_cmdbuf[0]&0x03f);
					fflush(stdout);
					assert(0 && "Not Implemented");
				}
				// }}}
			} m_cmdidx++;

			// If we are using blocks, add bytes for the start
			// token and the two CRC bytes
			m_blklen += 3;
			// }}}
		} else if (m_rspdly > 0) {
			// {{{
			assert((m_dat_in&0x0ff) == 0x0ff);
			// A delay until a response is given
			if (m_busy)
				m_dat_out = 0;
			m_rspdly--;
			// }}}
		} else if (m_rspidx < SDSPI_RSPLEN) {
			// {{{
			assert((m_dat_in&0x0ff) == 0x0ff);
			m_dat_out = m_rspbuf[m_rspidx++];
			// }}}
		} else if (m_blkdly > 0) {
			// {{{
			assert((m_dat_in&0x0ff) == 0x0ff);
			m_blkdly--;
			// }}}
		} else if (m_blkidx < SDSPI_MAXBLKLEN) {
			// {{{
			assert((m_dat_in&0x0ff) == 0x0ff);
			m_dat_out = m_block_buf[m_blkidx++];
			// }}}
		}
			// else m_dat_out = 0x0ff; // So set already above
		// }}}
	}

	int result = (m_dat_out&0x80)?1:0;
	m_dat_out <<= 1;
	m_delay = 0;
	m_last_miso = result;
	fflush(stdout);
	return result;
}
// }}}

unsigned SDSPISIM::cmdcrc(int len, char *buf) const {
	// {{{
	unsigned int fill = 0, taps = 0x12;

	for(int i=0; i<len; i++) {
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

bool	SDSPISIM::check_cmdcrc(char *buf) const {
	// {{{
	unsigned fill = cmdcrc(5, buf);
	if (m_debug && (fill != (buf[5]&0x0ff)))
		printf("SDSPI: CRC-CHECK ERR: should have a CRC of %02x, not %02x\n",
			fill, buf[5] & 0x0ff);
	return (fill == (buf[5]&0x0ff));
}
// }}}

unsigned SDSPISIM::blockcrc(int len, char *buf) const {
	// {{{
	unsigned int fill = 0, taps = 0x1021;
	bool	dbg = false; // (len == SECTOR_SIZE)&&(m_debug);

	if (dbg) {
		for(int i=0; i<len; i+=16) {
			printf("SDSPISIM::BUF[%3d] ", i);
			for(int k=0; (k<16)&&(k+i<len); k++) {
				printf("%02x ", buf[i+k]&0x0ff);
				if ((k&7) == 7)
					printf(" ");
			}
			printf("\n");
		}
	}
	for(int i=0; i<len; i++) {
		fill ^= ((buf[i]&0x0ff) << 8);
		for(int j=0; j<8; j++) {
			if (fill&0x8000)
				fill = (fill<<1)^taps;
			else
				fill <<= 1;
		}
		// if ((dbg)&&(i < 8))
		//	printf("After byte %d, CRC = %04x\n", i, fill & 0x0ffff);
	}

	fill &= 0x0ffff;
	if (dbg) { printf("BLOCKCRC(%d,...) = %04x\n", len, fill); }
	return fill;
}
// }}}

void	SDSPISIM::add_block_crc(int len, char *buf) const {
	// {{{
	unsigned fill = blockcrc(len, &buf[1]);

	buf[len+1] = (fill >> 8)&0x0ff;
	buf[len+2] = (fill     )&0x0ff;
}
// }}}
