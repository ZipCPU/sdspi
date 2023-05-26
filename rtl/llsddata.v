////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rtl/llsddata.v
// {{{
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	Low-level access to the bi-directional data lines of the
//		FPGA.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2019-2020, Gisselquist Technology, LLC
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
// with this program.  (It's in the $(ROOT)/doc directory, run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
`default_nettype none
// }}}
module	llsddata #(
		// {{{
		// 16-bit CRC coefficient
		parameter [15:0]	TAPS = 16'h102
		// parameter [12:0]	MAX_BYTES_PER_BLOCK = 512;
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Clock generator inputs
		input	wire		i_pedge, i_nedge,
		// Write/transmit interface
		input	wire		i_stb,		// Write request
		input	wire	[7:0]	i_byte,		// Write data
		output	wire		o_busy,		// Write channel busy
		// Read interface
		input	wire		i_expect_read,
		output	reg		o_stb,
		output	reg	[7:0]	o_byte,
		output	reg		o_err,
		//
		input	wire	[3:0]	i_sd_data,
		output	wire	[3:0]	o_sd_data
		// }}}
	);

	// Local declarations
	// {{{
	wire		write_cmd, read_cmd;
	reg		wr_bit_count;
	reg		wr_busy, wr_write_state;
	reg	[11:0]	wr_sreg;
	reg	[4:0]	wr_crc_bits;
	reg		wr_crc_active;
	reg	[15:0]	wr_crc		[3:0];
	integer		ik;

	////////////////////////////////////////

	reg		rd_read_state;
	reg	[7:0]	rd_sreg;
	reg		rd_byte, rd_crc_check0;
	reg		rd_bits;
	reg	[15:0]	rd_crc		[3:0];
	reg	[15:0]	rd_crc_last [0:1];

	reg	[1:0]	s_stb;
	reg	[15:0]	s_byte;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Write/transmit processing
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	assign	write_cmd = (i_stb && !o_busy);

	initial	wr_write_state = 0;
	always @(posedge i_clk)
	if (i_reset)
	begin
		// {{{
		wr_write_state <= 0;
		wr_busy <= 0;
		wr_bit_count <= 0;
		wr_sreg <= -1;
		wr_crc_bits   <= 0;
		wr_crc_active <= 1'b0;
		// }}}
	end else if (wr_write_state == 0)
	begin
		// {{{
		wr_write_state <= 0;
		wr_busy <= 0;
		wr_bit_count <= 0;
		wr_sreg <= -1;
		wr_crc_bits <= 0;
		wr_crc_active <= 1'b0;

		if (write_cmd)
		begin
			wr_sreg <= { 4'b0, i_byte };
			wr_crc_bits <= 16;
			wr_write_state <= 1;
			wr_busy <= 1;
		end
		// }}}
	end else if (i_pedge)
	begin
		// {{{
		wr_bit_count <= wr_bit_count + 1;
		wr_busy <= wr_crc_active || (!wr_bit_count);
		wr_sreg <= { wr_sreg[7:0], 4'hf };

		if (i_stb && !o_busy)
		begin
			// {{{
`ifdef	FORMAL
			assert(wr_crc_bits == 5'd16);
`endif // FORMAL
			wr_sreg <= { wr_sreg[7:4], i_byte };
			wr_crc_bits <= 5'd16;
			// }}}
		end else // if (wr_byte)
		begin // Host has just dropped the i_stb line
			// {{{
`ifdef	FORMAL
			assert((wr_crc_bytes != 2) || !i_stb && !o_busy);
`endif
			wr_crc_active <= 1'b1;

			// Send the first (next) nibble of the CRC
			wr_sreg[ 8] <= wr_crc[0][15];
			wr_sreg[ 9] <= wr_crc[1][15];
			wr_sreg[10] <= wr_crc[2][15];
			wr_sreg[11] <= wr_crc[3][15];

			wr_sreg[7:0] <= 8'hff;

			if (wr_crc_bits > 0)
				wr_crc_bits <= wr_crc_bits - 1;
			else
				// We're all done, go back to idle
				wr_write_state <= 0;
			// }}}
		end
		// }}}
	end

	always @(posedge i_clk)
	if (i_reset)
	begin
		for(ik=0; ik<4; ik=ik+1)
			wr_crc[ik] <= 0;
	end else if (wr_write_state == 0)
	begin
		for(ik=0; ik<4; ik=ik+1)
			wr_crc[ik] <= 0;
	end else if (i_pedge)
	begin
		if (!wr_crc_active)
		begin // Advance the CRC
			// {{{
			for(ik=0; ik<4; ik=ik+1)
				wr_crc[ik] <= { wr_crc[ik][14:0], 1'b0 }
					+ ((wr_sreg[4+ik] ^ wr_crc[ik][15])
						? TAPS : 0);
			// }}}
		end else begin // Flush the CRC
			for(ik=0; ik<4; ik=ik+1)
				wr_crc[ik] <= { wr_crc[ik][14:0], 1'b1 };
		end
	end

	assign	o_sd_data = wr_sreg[11:8];
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Read processing
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	initial	rd_read_state = 0;
	initial	o_byte = 8'hff;
	initial	s_byte[0] = 0;
	always @(posedge i_clk)
	if (i_reset || wr_write_state)
	begin
		// {{{
		rd_read_state <= 0;
		rd_byte  <= 0;
		rd_bits  <= 0;
		rd_sreg  <= 0;
		o_stb    <= 0;
		o_byte   <= 8'h00;
		o_err    <= 0;
		rd_crc_last[0] <= 0;
		rd_crc_last[1] <= 0;
		rd_crc_check0  <= 0;
		// }}}
	end else if (i_nedge)
	begin
		// {{{
		if (read_cmd)
			rd_read_state <= 1;

		if (!rd_read_state)
		begin
			// First bit doesn't count.  On that bit we'll set
			// rd_read_state, so the second bit (first data bit)
			// will have rd_read_state set and rd_byte == rd_bits==0
			rd_byte <= 0;
			rd_bits <= 0;
		end else begin
			rd_bits <= rd_bits + 1;
			rd_byte <= !rd_bits;
		end

		rd_sreg <= { rd_sreg[3:0], i_sd_data };

/*
		if (rd_byte)
		begin
			// This won't work.  Might've worked nice for SPI,
			// won't work for SDIO where we have a different
			// CRC on each data line.
			rd_crc_last[0] <= rd_crc;
			rd_crc_last[1] <= rd_crc_last[0];
			rd_crc_check0  <= (rd_sreg == rd_crc_last[0][15:8]);
			o_err <= (!rd_crc_check0
				|| (rd_sreg != rd_crc_last[1][7:0]));
		end else
*/
			o_err <= 0;
		// }}}
	end

	// rd_crc
	// {{{
	always @(posedge i_clk)
	if (i_reset || !rd_read_state)
	begin
		// {{{
		for(ik=0; ik<4; ik=ik+1)
			rd_crc[ik] <= 0;
		// }}}
	end else if (i_nedge)
	begin
		for(ik=0; ik<4; ik=ik+1)
			rd_crc[ik] <= { rd_crc[ik][14:0], 1'b0 }
				^ ((rd_crc[ik][15]^i_sd_data[ik]) ? TAPS : 0);
	end
	// }}}

	// s_byte, s_stb, o_stb, o_byte
	// {{{
	initial	s_byte = 0;
	always @(posedge i_clk)
	begin
		if (i_nedge && rd_byte)
		begin
			s_byte <= { s_byte[7:0], rd_sreg[7:0] };
			s_stb  <= { s_stb[0], rd_read_state };

			if (s_stb[1])
				o_byte <= s_byte[15:8];
			o_stb  <= s_stb[1];
		end else
			o_stb <= 1'b0;

		if (i_reset)
			{ o_stb, s_stb } <= 0;
	end
	// }}}

	assign	read_cmd = (!wr_write_state && !wr_busy && i_sd_data == 4'h0 && i_expect_read);
	// }}}

	assign	o_busy = (wr_busy || !i_pedge);
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
	reg	f_past_valid;
	initial	f_past_valid = 1'b0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	////////////////////////////////////////////////////////////////////////
	//
	// Initial state checking
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(!r_write_state);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// CRC update function
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
	if (!o_sd_data)
		assume(!i_sd_data);

	////////////////////////////////////////////////////////////////////////
	//
	// CRC update function
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	function [15:0]	f_updatecrc;
		input [15:0]	i_data;
		integer		icrc;

		f_updatecrc = 0;
		for(icrc=0; icrc< 8; icrc=icrc+1)
		if (i_data[15-icrc] ^ f_updatecrc[15])
			f_updatecrc[15:0] <= { f_updatecrc[14:0], 1'b0 } ^ 16'h1021;
		else
			f_updatecrc[15:0] <= { f_updatecrc[14:0], 1'b0 };
	endfunction
`endif
// }}}
endmodule
