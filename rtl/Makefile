################################################################################
##
## Filename: 	Makefile
## {{{
## Project:	SD-Card controller, using a shared SPI interface
##
## Purpose:	To build the Verilator library necessary for simulating (via
##		verilator) this code.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2016-2022, Gisselquist Technology, LLC
## {{{
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
## }}}
## License:	GPL, v3, as defined and found on www.gnu.org,
## {{{
##		http://www.gnu.org/licenses/gpl.html
##
##
################################################################################
##
## }}}
.PHONY: all
all:
VERILATOR:= verilator
# VOB := $(HOME)/src/verilog/vopro
VOB := cat
VOBJ:= obj_dir
VFLAGS := -MMD -Wall -trace -cc
SUBMAKE:= $(MAKE) --no-print-directory -C
all: rtl export.v $(VOBJ)/Vsdspi__ALL.so

.PHONY: rtl
## {{{
rtl: $(VOBJ)/Vsdspi.h
$(VOBJ)/Vsdspi.h: sdspi.v llsdspi.v
	$(VERILATOR) $(VFLAGS) sdspi.v
## }}}

$(VOBJ)/Vsdspi__ALL.so: $(VOBJ)/Vsdspi.h
	$(SUBMAKE) $(VOBJ) -f Vsdspi.mk

#
# Create a single file to encapsulate this whole core
export.v: sdspi.v spicmd.v spitxdata.v spirxdata.v llsdspi.v
	$(VOB) $^ > $@

.PHONY: clean
## {{{
clean:
	rm -rf export.v $(VOBJ)
## }}}
