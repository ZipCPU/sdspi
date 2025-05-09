################################################################################
##
## Filename:	rtl/Makefile
## {{{
## Project:	SD-Card controller
##
## Purpose:	To build the Verilator library necessary for simulating (via
##		verilator) this code.
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
.PHONY: all
all:
VERILATOR:= verilator
# VOB := $(HOME)/src/verilog/vopro	## Verilog scrambler
VOB := cat
VOBJ:= obj_dir
VFLAGS := -MMD -Wall -trace -cc
SUBMAKE:= $(MAKE) --no-print-directory -C
SDSPI_FILES := sdspi.v spicmd.v spitxdata.v spirxdata.v llsdspi.v
SDDMA_FILES := sddma_mm2s.v sddma_rxgears.v sddma_txgears.v sddma_s2mm.v sddma.v sdfifo.v sdskid.v
SDIO_FILES := sdio.v sdwb.v sdckgen.v sdcmd.v sdrxframe.v sdtxframe.v $(SDDMA_FILES)
all: rtl export $(VOBJ)/Vsdspi__ALL.so usage

.DELETE_ON_ERROR:

.PHONY: rtl
## {{{
rtl: $(VOBJ)/Vsdspi.h $(VOBJ)/Vsdio.h $(VOBJ)/Vsdaxil.h
$(VOBJ)/Vsdspi.h: sdspi.v llsdspi.v
	$(VERILATOR) $(VFLAGS) sdspi.v
$(VOBJ)/Vsdio.h: sdio.v sdwb.v sdcmd.v sdckgen.v sdrxframe.v sdtxframe.v
	$(VERILATOR) $(VFLAGS) sdio.v
$(VOBJ)/Vsdaxil.h: sdio.v sdaxil.v sdskid.v sdcmd.v sdckgen.v sdrxframe.v sdtxframe.v
	$(VERILATOR) -DSDIO_AXI --prefix Vsdaxi $(VFLAGS) sdio.v
## }}}

$(VOBJ)/Vsdspi__ALL.so: $(VOBJ)/Vsdspi.h
	$(SUBMAKE) $(VOBJ) -f Vsdspi.mk

#
# Create a single file to encapsulate the SDSPI core
.PHONY: export
export: exportspi.v exportsd.v
exportspi.v: $(SDSPI_FILES)
	$(VOB) $^ > $@

#
# Create a single file to encapsulate the SDSPI core
exportsd.v: $(SDIO_FILES)
	$(VOB) $^ > $@

.PHONY: usage
usage: usage.txt
usage.txt: usage.pl $(SDSPI_FILES) $(SDIO_FILES)
	perl usage.pl

.PHONY: clean
## {{{
clean:
	rm -rf exportspi.v exportsd.v $(VOBJ)
## }}}

## Including dependencies
## {{{
DEPS := $(wildcard $(VOBJ)/*.d)

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(DEPS),)
include $(DEPS)
endif
endif
## }}}

