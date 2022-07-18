////////////////////////////////////////////////////////////////////////////////
//
// Filename:	wb_tb.h
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
// Copyright (C) 2016-2022, Gisselquist Technology, LLC
// {{{
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
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
// }}}
#include <stdio.h>

#include <verilated.h>
#include <verilated_vcd_c.h>
#include "testb.h"

const int	BOMBCOUNT = 32,
		LGMEMSIZE = 15;

template <class VA>	class	WB_TB : public TESTB<VA> {
public:
	bool	m_bomb;

	WB_TB(void) {
		m_bomb = false;
		TESTB<VA>::m_core->i_wb_cyc = 0;
		TESTB<VA>::m_core->i_wb_stb = 0;
	}

#define	TICK	this->tick
	/*
	virtual	void	tick(void) {
		printf("WB-TICK\n");
		TESTB<VA>::tick();
	}
	*/

	unsigned wb_read(unsigned a) {
		// {{{
		int		errcount = 0;
		unsigned	result;

		// printf("WB-READM(%08x)\n", a);

		TESTB<VA>::m_core->i_wb_cyc = 1;
		TESTB<VA>::m_core->i_wb_stb = 1;
		TESTB<VA>::m_core->i_wb_we  = 0;
		TESTB<VA>::m_core->i_wb_addr= a;

		if (TESTB<VA>::m_core->o_wb_stall) {
			while((errcount++ < BOMBCOUNT)&&(TESTB<VA>::m_core->o_wb_stall)) {
				TICK();
			}
		} TICK();

		TESTB<VA>::m_core->i_wb_stb = 0;

		while((errcount++ <  BOMBCOUNT)&&(!TESTB<VA>::m_core->o_wb_ack)) {
			TICK();
		}


		result = TESTB<VA>::m_core->o_wb_data;

		// Release the bus
		TESTB<VA>::m_core->i_wb_cyc = 0;
		TESTB<VA>::m_core->i_wb_stb = 0;

		if(errcount >= BOMBCOUNT) {
			printf("WB/SR-BOMB: NO RESPONSE AFTER %d CLOCKS\n", errcount);
			m_bomb = true;
		} else if (!TESTB<VA>::m_core->o_wb_ack) {
			printf("WB/SR-BOMB: NO ACK, NO TIMEOUT\n");
			m_bomb = true;
		}
		TICK();

		assert(!TESTB<VA>::m_core->o_wb_ack);
		assert(!TESTB<VA>::m_core->o_wb_stall);

		return result;
	}
	// }}}

	void	wb_read(unsigned a, int len, unsigned *buf, const int inc=1) {
		// {{{
		int		errcount = 0;
		int		THISBOMBCOUNT = BOMBCOUNT * len;
		int		cnt, rdidx;

		printf("WB-READM(%08x, %d)\n", a, len);
		TESTB<VA>::m_core->i_wb_cyc  = 0;
		TESTB<VA>::m_core->i_wb_stb  = 0;

		while((errcount++ < BOMBCOUNT)&&(TESTB<VA>::m_core->o_wb_stall))
			TICK();

		if (errcount >= BOMBCOUNT) {
			printf("WB-READ(%d): Setting bomb to true (errcount = %d)\n", __LINE__, errcount);
			m_bomb = true;
			return;
		}

		errcount = 0;

		TESTB<VA>::m_core->i_wb_cyc  = 1;
		TESTB<VA>::m_core->i_wb_stb  = 1;
		TESTB<VA>::m_core->i_wb_we   = 0;
		TESTB<VA>::m_core->i_wb_addr = a;

		rdidx =0; cnt = 0;

		do {
			int	s;
			s = (TESTB<VA>::m_core->o_wb_stall==0)?0:1;
			TICK();
			TESTB<VA>::m_core->i_wb_addr += inc&(s^1);
			cnt += (s^1);
			if (TESTB<VA>::m_core->o_wb_ack)
				buf[rdidx++] = TESTB<VA>::m_core->o_wb_data;
		} while((cnt < len)&&(errcount++ < THISBOMBCOUNT));

		TESTB<VA>::m_core->i_wb_stb = 0;

		while((rdidx < len)&&(errcount++ < THISBOMBCOUNT)) {
			TICK();
			if (TESTB<VA>::m_core->o_wb_ack)
				buf[rdidx++] = TESTB<VA>::m_core->o_wb_data;
		}

		// Release the bus
		TESTB<VA>::m_core->i_wb_cyc = 0;

		if(errcount >= THISBOMBCOUNT) {
			printf("WB/PR-BOMB: NO RESPONSE AFTER %d CLOCKS\n", errcount);
			m_bomb = true;
		} else if (!TESTB<VA>::m_core->o_wb_ack) {
			printf("WB/PR-BOMB: NO ACK, NO TIMEOUT\n");
			m_bomb = true;
		}
		TICK();
		assert(!TESTB<VA>::m_core->o_wb_ack);
	}
	// }}}

	void	wb_write(unsigned a, unsigned v) {
		// {{{
		int errcount = 0;

		printf("WB-WRITEM(%08x) <= %08x\n", a, v);
		TESTB<VA>::m_core->i_wb_cyc = 1;
		TESTB<VA>::m_core->i_wb_stb = 1;
		TESTB<VA>::m_core->i_wb_we  = 1;
		TESTB<VA>::m_core->i_wb_addr= a;
		TESTB<VA>::m_core->i_wb_data= v;
		TESTB<VA>::m_core->i_wb_sel = 0x0f;

		if (TESTB<VA>::m_core->o_wb_stall)
			while((errcount++ < BOMBCOUNT)&&(TESTB<VA>::m_core->o_wb_stall)) {
				printf("Stalled, so waiting, errcount=%d\n", errcount);
				TICK();
			}
		TICK();

		TESTB<VA>::m_core->i_wb_stb = 0;

		while((errcount++ <  BOMBCOUNT)&&(!TESTB<VA>::m_core->o_wb_ack))
			TICK();
		TICK();

		// Release the bus?
		TESTB<VA>::m_core->i_wb_cyc = 0;
		TESTB<VA>::m_core->i_wb_stb = 0;

		if(errcount >= BOMBCOUNT) {
			printf("WB/SW-BOMB: NO RESPONSE AFTER %d CLOCKS (LINE=%d)\n", errcount, __LINE__);
			m_bomb = true;
		} TICK();
		assert(!TESTB<VA>::m_core->o_wb_ack);
		assert(!TESTB<VA>::m_core->o_wb_stall);
	}
	// }}}

	void	wb_write(unsigned a, unsigned int ln, unsigned *buf, const int inc=1) {
		// {{{
		unsigned errcount = 0, nacks = 0;

		printf("WB-WRITEM(%08x, %d, ...)\n", a, ln);
		TESTB<VA>::m_core->i_wb_cyc = 1;
		TESTB<VA>::m_core->i_wb_stb = 1;
		TESTB<VA>::m_core->i_wb_we  = 1;
		TESTB<VA>::m_core->i_wb_addr= a;
		TESTB<VA>::m_core->i_wb_sel = 0x0f;
		for(unsigned stbcnt=0; stbcnt<ln; stbcnt++) {
			// m_core->i_wb_addr= a+stbcnt;
			TESTB<VA>::m_core->i_wb_data= buf[stbcnt];
			errcount = 0;

			while((errcount++ < BOMBCOUNT)&&(TESTB<VA>::m_core->o_wb_stall)) {
				TICK();
				if (TESTB<VA>::m_core->o_wb_ack)
					nacks++;
			}
			// Tick, now that we're not stalled.  This is the tick
			// that gets accepted.
			TICK();
			if (TESTB<VA>::m_core->o_wb_ack) nacks++;

			// Now update the address
			TESTB<VA>::m_core->i_wb_addr += inc;
		}

		TESTB<VA>::m_core->i_wb_stb = 0;

		errcount = 0;
		while((nacks < ln)&&(errcount++ < BOMBCOUNT)) {
			TICK();
			if (TESTB<VA>::m_core->o_wb_ack) {
				nacks++;
				errcount = 0;
			}
		}

		// Release the bus
		TESTB<VA>::m_core->i_wb_cyc = 0;
		TESTB<VA>::m_core->i_wb_stb = 0;

		if(errcount >= BOMBCOUNT) {
			printf("WB/PW-BOMB: NO RESPONSE AFTER %d CLOCKS (LINE=%d)\n",errcount,__LINE__);
			m_bomb = true;
		}
		TICK();
		assert(!TESTB<VA>::m_core->o_wb_ack);
		assert(!TESTB<VA>::m_core->o_wb_stall);
	}
	// }}}

	bool	bombed(void) const { return m_bomb; }

	// bool	debug(void) const	{ return m_debug; }
	// bool	debug(bool nxtv)	{ return m_debug = nxtv; }
};

