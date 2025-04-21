################################################################################
##
## Filename:	bench/verilog/Makefile
## {{{
## Project:	SD-Card controller
##
## Purpose:	
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2016-2025, Gisselquist Technology, LLC
## {{{
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of the GNU General Public License as published
## by the Free Software Foundation, either version 3 of the License, or (at
## your option) any later version.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
## target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
## }}}
## License:	GPL, v3, as defined and found on www.gnu.org,
## {{{
##		http://www.gnu.org/licenses/gpl.html
##
################################################################################
##
## }}}
RTL := ../../rtl
CPU := $(HOME)/work/rnd/zipcpu/trunk

tb_sdckgen: $(RTL)/sdckgen.v tb_sdckgen.v
	iverilog -g2012 $^ -o $@

tb_txframe: $(RTL)/sdckgen.v $(RTL)/sdtxframe.v tb_txframe.v
	iverilog -g2012 $^ -o $@

obj_dir/Vtb_cpu.h:
	verilator -Wall -Wno-TIMESCALEMOD -y $(RTL) -y . -y $(CPU)/rtl -y $(CPU)/rtl/core -y $(CPU)/rtl/peripherals -y $(CPU)/rtl/zipdma -y $(CPU)/rtl/ex -y $(CPU)/sim/rtl -cc tb_cpu.v

.PHONY: test_ckgen
test_ckgen: tb_sdckgen
	./tb_sdckgen

.PHONY: test_txframe
test_txframe: tb_txframe
	./tb_txframe

.PHONY: test
test: test_txframe test_ckgen
	perl sim_run.pl all
