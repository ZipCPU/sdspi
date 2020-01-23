////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	llsdcmd.v
//
// Project:	SD-Card controller, using a shared SPI interface
//
// Purpose:	This is the first component of an SDIO interface, the component
//		that handles all transactions over the CMD wire.
//
// The master requests a command transaction by seting i_cmd, its 32'bit
// parameter i_data, and then raises the i_stb signal.  It then waits for the
// core to become idle.  (i.e., to drop o_busy).  This core then transmits the
// command, while adding the CRC to the end.  Once the command completes, the
// core waits for the other end to initiate a response.  Once sync has been
// detected, the response is then read in.  Once the whole response word has
// arrived, o_cmd will be set with the type of response, o_data with it's
// 32-bit data, and o_stb to indicate that o_cmd, and o_data are valid.
// In addition, upon any CRC failure, the o_err signal will be raised at this
// time as well.  (It is meaningless at any other time, so always guard it with
// o_stb).  Similarly, o_crc captures the contents of the CRC register at the
// end of the packet ... in case anything else might need it.
//
// That works for 48-bit commands and 48-bit responses.
//
// Since some responses can be longer than 48-bits, this core also has a
// byte-wise interface. Once synchronized, every following byte will be captured
// and output on o_byte.  Further, o_byte is qualified by o_bstb.  If o_bstb
// is low, o_byte may take on any value.  If, on the other hand, o_byte is
// now ready, o_bstb will be set high to indicate the presence of new data.
//
// o_bstb will continue to be high every 8-bits hereafter until either the next
// i_stb, or until i_reset becomes active (again)..
//
// When using a clock divider, i_pedge will be true one clock before the rising
// edge of the clock.  It's a signal that can be used to cause things to
// transition that need to do so on the positive edge of the clock.  Similarly,
// i_nedge is to be set one clock prior to the negative clock edge for the same
// purpose, but with respect to the negative edge of the clock.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2019-2020, Gisselquist Technology, LLC
//
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
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype none
//
module	llsdcmd(i_clk, i_reset,
		i_pedge, i_stb, i_cmd, i_data, o_busy,
		i_nedge, o_stb, o_cmd, o_data, o_crc, o_err,
		o_bstb, o_byte,
		i_sd_cmd, o_sd_cmd);	
	parameter [6:0]	CRC_POLYNOMIAL = 7'h09;
	input	wire	i_clk, i_reset, i_pedge;
	//
	input	wire		i_stb;
	input	wire	[7:0]	i_cmd;
	input	wire	[31:0]	i_data;
	output	reg		o_busy;
	//
	input	wire		i_nedge;
	output	reg		o_stb;
	output	reg	[7:0]	o_cmd;
	output	reg	[31:0]	o_data;
	output	wire	[7:0]	o_crc;
	output	reg		o_err;
	//
	output	reg		o_bstb;
	output	reg	[7:0]	o_byte;
	//
	input	wire		i_sd_cmd;
	output	reg		o_sd_cmd;

	//
	//
	//


	reg	[3:0]	r_rxstate;
	integer	ik;

	reg	[39:0]	tx_cmd_sreg;
	reg	[6:0]	r_txcrc;
	reg	[47:0]	tx_busy_sreg, rx_sreg;
	reg		last_busy, tx_busy;
	reg	[5:0]	r_rxpkt_count;

	//
	//
	//
	wire	write_cmd = (i_stb && !o_busy && !i_cmd[7]);
	wire	read_cmd  = (i_nedge && !i_sd_cmd && r_rxstate == 1 && !tx_busy);

	////////////////////////////////////////////////////////////////////////
	//
	// Transmit/command logic
	//

	initial	tx_busy_sreg =  0;
	initial	tx_cmd_sreg  = -1;
	always @(posedge i_clk)
	if (i_reset)
	begin
		tx_busy_sreg <= 0;
		tx_cmd_sreg  <= -1;
	end else if (write_cmd)
	begin
		tx_cmd_sreg <= { i_cmd, i_data };
		tx_busy_sreg <= -1;
	end else if (i_pedge)
	begin
		tx_cmd_sreg  <= { tx_cmd_sreg[38:0], 1'b1 };
		if (tx_busy_sreg[39:38] == 2'b10)
			tx_cmd_sreg[39:33] <= r_txcrc;
		tx_busy_sreg <= tx_busy_sreg << 1;
	end

	always @(*)
	begin
		tx_busy = tx_busy_sreg[47];
		o_busy = tx_busy || !i_pedge;
	end

	always @(*)
		o_sd_cmd = tx_cmd_sreg[39];

	always @(posedge i_clk)
	if (write_cmd)
		r_txcrc <= 0; // (i_cmd[6]) ? 8'h12 : 8'h0;
	else if (i_pedge)
	begin
		if (r_txcrc[6] ^ tx_cmd_sreg[38])
			r_txcrc <= { r_txcrc[5:0], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			r_txcrc <= { r_txcrc[5:0], 1'b0 };
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Receiver/response logic
	//
	initial	last_busy = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		last_busy <= 1'b0;
	else if (i_pedge)
		last_busy <= tx_busy;

	initial	r_rxstate = 0;
	always @(posedge i_clk)
	if (i_reset || (write_cmd))
		r_rxstate <= 4'h0;
	else if (i_pedge && !tx_busy && last_busy)
		r_rxstate <= 4'h1;
	else if (i_nedge && r_rxstate == 4'h1 && !i_sd_cmd)
		r_rxstate <= 4'h2;
	else if (i_nedge && r_rxstate > 4'h1)
	begin
		if (r_rxstate <= 4'h8)
			r_rxstate <= r_rxstate + 4'h1;
		else
			r_rxstate <= 4'h2;
	end

	initial	o_bstb = 0;
	always @(posedge i_clk)
	if (i_reset || write_cmd)
		o_bstb <= 0;
	else
		o_bstb <= i_nedge && (r_rxstate == 4'h8);

	initial	o_byte = -1;
	always @(posedge i_clk)
	if (i_reset || (write_cmd))
		o_byte <= -1;
	else if (i_nedge && r_rxstate == 4'h8)
		o_byte <= { rx_sreg[6:0], i_sd_cmd };

	initial	rx_sreg = -1;
	always @(posedge i_clk)
	if (i_reset || (write_cmd))
		rx_sreg <= -1;
	else if (i_nedge && read_cmd)
		rx_sreg <= { {(47){1'b1}}, 1'b0 };
	else if (i_nedge && r_rxstate > 1)
		rx_sreg <= { rx_sreg[46:0], i_sd_cmd };

	always @(posedge i_clk)
	if (i_nedge && r_rxpkt_count == 6'd47)
		{ o_cmd, o_data, o_crc } = { rx_sreg[46:0], i_sd_cmd };

	initial	r_rxpkt_count = 0;
	always @(posedge i_clk)
	if (i_reset || write_cmd || tx_busy)
		r_rxpkt_count <= 0;
	else if (read_cmd)
		r_rxpkt_count <= 1;
	else if (i_nedge && (r_rxpkt_count != 0) && (r_rxpkt_count < 6'h3f))
		r_rxpkt_count <= r_rxpkt_count + 1;

	reg	[6:0]	r_rxcrc;
	reg		r_err;

	always @(posedge i_clk)
	if (read_cmd)
	begin
		r_rxcrc <= 0;
		r_err <= 0;
	end else if (i_nedge)
	begin
		if (r_rxpkt_count < 6'd40)
		begin
			r_err <= 0;
			if (i_sd_cmd ^ r_rxcrc[6])
				r_rxcrc <= { r_rxcrc[5:0], 1'b0 } ^ CRC_POLYNOMIAL;
			else
				r_rxcrc <= { r_rxcrc[5:0], 1'b0 };
		end else begin
			r_rxcrc <= r_rxcrc << 1;
			r_rxcrc[0] <= 1'b1;
			r_err  <= r_err || (r_rxcrc[6] != i_sd_cmd);
		end
	end

	initial	o_stb = 1'b0;
	always @(posedge i_clk)
	if (i_reset || write_cmd)
		o_stb <= 1'b0;
	else if (i_nedge && r_rxpkt_count == 6'd47)
		o_stb <= 1'b1;
	else
		o_stb <= 1'b0;

	initial	o_err = 1'b0;
	always @(posedge i_clk)
	if (i_reset || write_cmd)
		o_err <= 1'b0;
	else if (i_nedge && r_rxpkt_count == 6'd47)
		o_err <= r_err || (r_rxcrc[6] != i_sd_cmd);
	else
		o_err <= 1'b0;

`ifdef	FORMAL
always @(*)
begin
	cover(i_nedge && r_rxpkt_count == 6'd8);
	cover(i_nedge && r_rxpkt_count == 6'd16);
	cover(i_nedge && r_rxpkt_count == 6'd24);
	cover(i_nedge && r_rxpkt_count == 6'd48);
end

	reg	f_past_valid;

	reg	[47:0]	f_txcmd, f_txcmdnow;
	reg	[5:0]	f_txseq;
	reg	[7:0]	f_txcrc;

	initial	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid <= 1;

	////////////////////////////////////////////////////////////////////////
	//
	// Interface assumptions
	//

	always @(posedge i_clk)
	if (!$past(i_pedge))
		assume(i_pedge);
	else if (!$past(i_pedge,2))
		assume(i_pedge);

	always @(*)
		assume(i_pedge == i_nedge);

	always @(posedge i_clk)
	if(f_past_valid && !$past(i_reset) && $past(i_stb && o_busy))
	begin
		assume(i_stb);
		assume($stable(i_cmd));
		assume($stable(i_data));
	end
		

	////////////////////////////////////////////////////////////////////////
	//
	// Initial property checks
	//
	////////////////////////////////////////////////////////////////////////
	//
	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset))
	begin
		assert(tx_busy_sreg == 0);
		assert(&tx_cmd_sreg);
		//
		assert(last_busy == 1'b0);
		assert(r_rxstate == 4'h0);
		//
		assert(o_bstb == 1'b0);
		assert(o_byte == 8'hff);
		assert(&rx_sreg);
		assert(r_rxpkt_count == 0);
		//
		assert(!o_stb);
		assert(!o_err);
		//
		//
		assert(&f_txseq);
		assert(&f_rxcmd);
		assert(&f_rxseq);
		assert(&f_rxbyte_count);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Transmitter checks
	//
	always @(*)
		f_txcrc = f_gencrc({ i_cmd, i_data });

	always @(*)
	if (!o_sd_cmd)
		assume(!i_sd_cmd);

	always @(posedge i_clk)
	if (!o_busy && i_stb && !i_cmd[7])
		f_txcmd <= { i_cmd, i_data, f_txcrc };

	initial	f_txseq = 6'h3f;
	always @(posedge i_clk)
	if (i_reset)
		f_txseq <= 6'h3f;
	else if (write_cmd)
		f_txseq <= 0;
	else if (i_pedge && f_txseq != 6'h3f)
		f_txseq <= f_txseq + 1;

	always @(*)
		f_txcmdnow = f_txcmd << f_txseq;

	reg	[47:0]	f_checktxmsg;
	reg	[7:0]	f_checktxcrc;
	always @(*)
	begin
		f_checktxmsg = f_txcmd[47:0] >> (6'd47-f_txseq);
		f_checktxcrc = f_gencrc(f_checktxmsg);
	end

	always @(*)
	if (f_txseq < 6'd48)
	begin
		assert(o_sd_cmd == f_txcmdnow[47]);
		assert(f_txcmd[7:0] == f_gencrc({ f_txcmd[47:8] }));
		assert(o_busy);
		assert((~tx_busy_sreg[31:0] & ~tx_cmd_sreg[31:0])==0);
		if ((f_txseq > 3)&&(f_txseq < 6'd40))
		begin
			assert(r_txcrc == f_checktxcrc[7:1]);
			assert(tx_busy_sreg == ({(48){1'b1}} << f_txseq));
			if (f_txseq == 6'd8)
				assert(tx_cmd_sreg[39:0] == { f_txcmd[39:8], 8'hff});
			if (f_txseq == 6'd16)
				assert(tx_cmd_sreg[39:0] == { f_txcmd[31:8], 16'hffff});
			if (f_txseq == 6'd24)
				assert(tx_cmd_sreg[39:0] == { f_txcmd[23:8], 24'hffffff});
			if (f_txseq == 6'd32)
				assert(tx_cmd_sreg[39:0] == { f_txcmd[15:8], 32'hffffffff});
		end
	end else begin
		assert(o_sd_cmd);
		assert(tx_busy_sreg == 0);
		assert(&tx_cmd_sreg);
	end

	////////////////////////////////////////////////////////////////////////
	//
	// Receiver checks
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	(* anyseq *)	reg	[45:0]	f_rxnextcmd;
	reg	[47:0]	f_rxcmd, f_rxcmdnow;
	reg	[5:0]	f_rxseq;
	reg	[3:0]	f_rxbyte_count;
	reg		f_rxcmd_err;
	reg	[7:0]	f_rxbyte;
	reg		f_rxbit;

	initial	f_rxcmd = -1;
	initial	f_rxseq = 6'h3f;
	always @(posedge i_clk)
	if (i_reset || write_cmd || tx_busy)
	begin
		f_rxcmd <= -1;
		f_rxseq <= 6'h3f;
	end else if (read_cmd)
	begin
		f_rxcmd <= { 2'b00, f_rxnextcmd };
		f_rxcmd_err <= (f_gencrc({ 2'b00, f_rxnextcmd[45:8] })
						!= f_rxnextcmd[7:0] );
		f_rxseq <= 1;
	end else if (i_nedge)
	begin
		// f_rxcmd <= f_rxcmd << 1;
		if (f_rxseq != 6'h3f)
			f_rxseq <= f_rxseq + 1;
	end

	always @(*)
	if (f_rxseq != 6'h3f)
	begin
		assert(f_rxcmd[47:46] == 2'b00);
		assert(f_rxcmd_err == (f_gencrc(f_rxcmd[47:8]) != f_rxcmd[7:0]));
	end

	//
	// The generic byte interface
	//
	initial	f_rxbyte_count = 4'hf;	
	always @(posedge i_clk)
	if (i_reset || write_cmd)
		f_rxbyte_count <= 4'hf;
	else if (read_cmd)
		f_rxbyte_count <= 0;
	else if (i_nedge && f_rxbyte_count != 4'hf)
	begin
		f_rxbyte_count <= f_rxbyte_count + 1;
		if (f_rxbyte_count == 4'h8)
			f_rxbyte_count <= 4'h1;
	end

	always @(posedge i_clk)
		assert(o_bstb == (f_rxbyte_count == 4'h7 && $past(i_nedge)));

	always @(posedge i_clk)
	if (i_nedge)
		f_rxbyte <= { f_rxbyte[6:0], i_sd_cmd };

	always @(posedge i_clk)
	if (o_bstb)
		assert(f_rxbyte == o_byte);

	always @(posedge i_clk)
	if ($past(i_nedge) && (f_rxseq == 6'h8
				|| f_rxseq == 6'h10
				|| f_rxseq == 6'h18
				|| f_rxseq == 6'h20
				|| f_rxseq == 6'h28))
		assert(o_bstb);

	always @(*)
		f_rxbit = { 16'h0, f_rxcmd } >> (6'd47-f_rxseq);

	always @(*)
	if (f_rxseq < 6'd48)
		assume(i_sd_cmd == f_rxbit);

	always @(posedge i_clk)
	if (o_stb)
	begin
		assert(o_bstb);
		assert(o_cmd  == f_rxcmd[47:40]);
		assert(o_data == f_rxcmd[39: 8]);
		assert(o_crc  == f_rxcmd[ 7: 0]);
		assert(o_err  == f_rxcmd_err);
	end else
		assert(o_err == 1'b0);

	always @(posedge i_clk)
	if (!f_past_valid || $past(i_reset) || $past(!i_nedge))
		assert(!o_stb);

	always @(*)
	if (tx_busy && i_pedge)
	begin
		assert(r_rxstate <= 1);
		assert(f_rxbyte_count == 4'hf);
		assert(r_rxpkt_count == 0);
	end else if (!i_pedge)
		assert(o_busy);

	always @(*)
	if (tx_busy)
		assert(r_rxstate == 0);

	reg	[47:0]	f_checkrxmsg;
	reg	[7:0]	f_checkrxcrc;
	always @(*)
	begin
		f_checkrxmsg = f_rxcmd[47:0] >> (6'd48-f_rxseq);
		f_checkrxcrc = f_gencrc(f_checkrxmsg);
		f_checkrxmsg = f_checkrxmsg | ({(48){1'b1}} << f_rxseq);
		if (r_rxpkt_count == 0)
			assert(f_rxseq > 6'd48);
		else
			assert(r_rxpkt_count == f_rxseq);
		if ((f_rxseq > 6'd3)&&(f_rxseq < 6'd40))
		begin
			assert(r_rxcrc == f_checkrxcrc[7:1]);
			assert(rx_sreg == f_checkrxmsg);
		end
	end

	always @(*)
		assert(r_rxstate <= 4'ha);

	reg	[2:0]	f_rxcheckstate;
	always @(*)
		f_rxcheckstate = r_rxstate - 1;

	always @(*)
	if (r_rxstate <= 1)
		assert(f_rxseq >= 6'd48);
	else if (r_rxpkt_count < 6'd48)
		assert(f_rxcheckstate == r_rxpkt_count[2:0]);

	////////////////////////////////////////////////////////////////////////
	//
	// Cover properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
		cover(f_past_valid && !$past(i_reset) && $fell(o_busy));

	always @(posedge i_clk)
		cover(o_bstb);

	always @(posedge i_clk)
		cover(o_stb && !o_err);

	always @(posedge i_clk)
		cover(o_stb && o_err);

	////////////////////////////////////////////////////////////////////////
	//
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	function [7:0]	f_gencrc;
		input	[39:0]	i_cmdword;
		integer	icrc;

		f_gencrc = 0;
		for(icrc=0; icrc<40; icrc=icrc+1)
		if (i_cmdword[39-icrc] ^ f_gencrc[7])
			f_gencrc[7:1] = { f_gencrc[6:1], 1'b0 } ^ CRC_POLYNOMIAL;
		else
			f_gencrc[7:1] = { f_gencrc[6:1], 1'b0 };
		f_gencrc = { f_gencrc[7:1], 1'b1 };
	endfunction
// 29
//
// 60 -> e5
// 50 -> ff
// 48 -> 53
// 44 -> b3
// 42 -> 61
// 41 -> c5

`ifdef	BROKEN

	always @(*)
	if (!f_past_valid)
	begin
		assert(f_gencrc({ 2'b01, 6'h00, 32'h000 }) == 8'b1001_0101);
		assert(f_gencrc({ 2'b01, 6'h11, 32'h000 }) == 8'b0101_0101);
		assert(f_gencrc({ 2'b00, 6'h11, 32'h0900}) == 8'b0110_0111);
	end
`endif

	////////////////////////////////////////////////////////////////////////
	//
	// Careless assumptions
	//
	////////////////////////////////////////////////////////////////////////
	//
	//

	// We could make the following assumption ...
	//  but it's not really necessary
	//	always @(*)
	//	if (write_cmd)
	//		assume(i_cmd[6]);

`endif
endmodule
