################################################################################
##
## Filename: 	Makefile
##
## Project:	SD-Card controller, using a shared SPI interface
##
## Purpose:	To build the Verilator library necessary for simulating (via
##		verilator) this code.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
##
## Copyright (C) 2016, Gisselquist Technology, LLC
##
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of  the GNU General Public License as published
## by the Free Software Foundation, either version 3 of the License, or (at
## your option) any later version.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with this program.  (It's in the $(ROOT)/doc directory, run make with no
## target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
##
## License:	GPL, v3, as defined and found on www.gnu.org,
##		http://www.gnu.org/licenses/gpl.html
##
##
################################################################################
##
##

all: rtl export.v
VERILATOR:= verilator
# VOB := $(HOME)/src/verilog/vopro
VOB := cat
VOBJ:= obj_dir
VFLAGS := -MMD -Wall -trace -cc

.PHONY: rtl
rtl: $(VOBJ)/Vsdspi.h
$(VOBJ)/Vsdspi.h: sdspi.v llsdspi.v
	$(VERILATOR) $(VFLAGS) sdspi.v

export.v: sdspi.v llsdspi.v
	$(VOB) $^ > $@
