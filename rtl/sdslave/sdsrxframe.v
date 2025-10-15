////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rtl/sdslave/sdsrxframe.v
// {{{
// Project:	SD-Card controller
//
// Purpose:	SDIO Slave receive module.
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
`default_nettype none
// }}}
module	sdsrxframe #(
		parameter [0:0]	OPT_DDR = 1'b1,
		parameter	NUMIO = 4
	) (
		// {{{
		input	wire		i_clk, i_reset,
		input	wire		i_cfg_en,
		input	wire	[3:0]	i_cfg_lgblksz, // 4=16B, 9=512B, 15=32768B
		input	wire		i_cfg_ddr,
		input	wire	[1:0]	i_cfg_width,	// 0=1b, 1=4b, 2=8b
		// i_data[15:8] arrives on the positive edge, followed by
		//   i_data[7:0] on the negative edge (if so enabled)
		input	wire		i_valid,
		input	wire	[15:0]	i_data,
		//
		output	wire		o_valid,
		output	wire	[31:0]	o_data,
		output wire		o_last,
		output	wire		o_good,	// Full block, valid CRCs
					o_err	// CRC failure
		// }}}
	);

	// Local declarations
	// {{{
	localparam	NCRC = 16;
	localparam	[NCRC-1:0]	CRC_POLY = 16'h1021;

	localparam	// WIDTH_1W = 2'b00,
			WIDTH_4W = 2'b01,
			WIDTH_8W = 2'b10;
	integer	ik, jk;
	reg		started, crc_active, last, write, crc_good, crc_fail,
			w_valid_crc, wr_last;
	reg	[19:0]	count;
	reg	[31:0]	sreg;
	reg	[NCRC-1:0]	crc_fill	[2*NUMIO-1:0];
	// }}}

	always @(posedge i_clk)
	if (i_reset || !i_cfg_en)
	begin
		// {{{
		// We'll be here if our card is not selected, or any time we
		// aren't expecting to receive anything.
		started <= 0;
		crc_active <= 0;
		count <= 0;
		last <= 0;
		// }}}
	end else if (i_valid)
	begin
		if (!started)
		begin
			// {{{
			crc_active <= 0;
			last <= 0;
			case(i_cfg_width)
			WIDTH_4W: begin
				started <= i_data[11:8] == 4'h0;
				count <= ((i_cfg_ddr ? 1 : 2) << i_cfg_lgblksz) + 16;
				end
			WIDTH_8W: begin
				started <= (i_data[15:8] == 8'h0);
				if (i_cfg_ddr)
					count <= ((1 << i_cfg_lgblksz) >> 1) + 16;
				else
					count <=  (1 << i_cfg_lgblksz) + 16;
				end
			// WIDTH_1W: follows the default below
			default: begin
				started <= i_data[8] == 1'b0;
				count <= ((i_cfg_ddr ? 4 : 8)<<i_cfg_lgblksz) + 16;
				end
			endcase
			// }}}
		end else begin
			count <= count - 1;
			last <= (count <= 1);
			crc_active <= (count <= 17);

			if (last)
				started <= 0;
		end
	end

	always @(posedge i_clk)
	if (i_reset || !i_cfg_en || !started)
	begin
		sreg <= 0;
		write <= 0;
		wr_last <= 0;
	end else if (i_valid)	// Might even be always true ...
	begin
		write <= 0;
		wr_last <= last;
		if (OPT_DDR && i_cfg_ddr)
		begin
			// {{{
			case(i_cfg_width)
			WIDTH_4W: begin
				if (count[0])
				begin
					sreg <= { sreg[31:12], i_data[11:8],
							sreg[7:4],
								i_data[3:0] };
				end else begin
					sreg <= { sreg[15:0],
						i_data[11:8], 4'h0,
						i_data[ 3:0], 4'h0 };
				end
				write <= (count[1:0] == 2'h1);
				end
			WIDTH_8W: begin
				sreg <= { sreg[15:0], i_data[15:0] };
				write <= count[0] == 1'b1;
				end
			// WIDTH_1W:
			default: begin
				sreg <= { sreg[29:0], i_data[8], i_data[0] };
				write <= (count[3:0] == 4'h1);
				end
			endcase

			if (crc_active || last)
				write <= 0;
			// }}}
		end else begin
			// {{{
			case(i_cfg_width)
			WIDTH_4W: begin
				sreg <= { sreg[27:0], i_data[11:8] };
				write <= (count[2:0] == 3'h1);
				end
			WIDTH_8W: begin
				sreg <= { sreg[23:0], i_data[15:8] };
				write <= (count[1:0] == 2'h1);
				end
			// WIDTH_1W:
			default: begin
				sreg <= { sreg[30:0], i_data[8] };
				// Write after every 32 bits, save that our
				// count is off by 16 to account for the 16
				// CRC bits
				wr_last <= (count <= 20'h11);
				write <= (count[4:0] == 5'h11);
				end
			endcase

			if (crc_active || last)
				write <= 0;
			// }}}
		end
	end else
		write <= 0;

	assign	o_valid = write;
	assign	o_data  = sreg;
	assign	o_last  = wr_last;

	// CRC Fill, for both positive and negative edges
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		for(ik=0; ik<2*NUMIO; ik=ik+1)
			crc_fill[ik] <= 0;
	end else if (!started || last || !i_cfg_en)
	begin
		for(ik=0; ik<2*NUMIO; ik=ik+1)
			crc_fill[ik] <= 0;
	end else if (i_valid)
	begin
		for(ik=0; ik<NUMIO; ik=ik+1)
		begin
			// Positive edge
			crc_fill[NUMIO+ik] <= ADVANCE_CRC(crc_fill[NUMIO+ik],
							i_data[8+ik]);
			// Negative edge
			crc_fill[ik] <= ADVANCE_CRC(crc_fill[ik], i_data[ik]);

			if (!OPT_DDR || !i_cfg_ddr)
				crc_fill[ik] <= 0;

			case(i_cfg_width)
			WIDTH_4W: if (ik >= 4)
				begin
				crc_fill[ik] <= 0;
				crc_fill[NUMIO+ik] <= 0;
				end
			WIDTH_8W: begin end
			// WIDTH_1W:
			default: if (ik >= 1)
				begin
				crc_fill[ik] <= 0;
				crc_fill[NUMIO+ik] <= 0;
				end
			endcase
		end // for(ik ...)
	end
	// }}}

wire	[NCRC-1:0]	fill0p, fill0n;
	assign	fill0n = crc_fill[0];
	assign	fill0p = crc_fill[NUMIO];


	// CRC Checking
	// {{{
	// Can we do most of the zero calculation on the clock prior?  Such
	// as checking all but the last bit?
	//	pre_zero = crc_fill[NCRC-1:0] == 0
	//
	// always @(*)
	// for(zk=0; zk<2*NUMIO; zk=zk+1)
		// crc_zero[zk] = { pre_zero[zk][1],
		//			pre_zero[zk][0] == i_data[zk] };

	always @(*)
	begin
		// Assume a valid CRC until proven wrong
		w_valid_crc = 1;

		// The CRC is invalid if it is non zero
		for(jk=0; jk<NUMIO; jk=jk+1)
		begin
			if (crc_fill[jk] != 0)
				w_valid_crc = 0;
			if (crc_fill[NUMIO+jk] != 0)
				w_valid_crc = 0;
		end

		// It's also invalid if, on the stop bit (clock), we don't
		// have a valid stop indication
		case(i_cfg_width)
		WIDTH_4W: if (i_data[11:8] != 4'hf) w_valid_crc = 0;
		WIDTH_8W: if (i_data[15:8] != 8'hff) w_valid_crc = 0;
		// WIDTH_1W:
		default if (i_data[8] != 1'b1) w_valid_crc = 0;
		endcase
	end
	// }}}

	// Results
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		crc_good <= 0;
		crc_fail <= 0;
	end else if (!started || !last || !i_valid || !i_cfg_en)
	begin
		crc_good <= 0;
		crc_fail <= 0;
	end else begin
		crc_good <=  w_valid_crc;
		crc_fail <= !w_valid_crc;
	end

	assign	o_err  = crc_fail;
	assign	o_good = crc_good;
	// }}}

	function automatic [NCRC-1:0]	ADVANCE_CRC(input[NCRC-1:0] prior,
						input i_crc_data);
	begin
		if (prior[NCRC-1] ^ i_crc_data)
			ADVANCE_CRC = { prior[NCRC-2:0], 1'b0 } ^ CRC_POLY;
		else
			ADVANCE_CRC = { prior[NCRC-2:0], 1'b0 };
	end endfunction
endmodule
