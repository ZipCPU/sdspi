################################################################################
##
## Filename: 	Makefile
## {{{
## Project:	SD-Card controller, using a shared SPI interface
##
## Purpose:	
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2019-2022, Gisselquist Technology, LLC
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
################################################################################
##
## }}}
.PHONY: all
all:	sdspi llsdspi spicmd spitxdata spirxdata

RTL := ../../rtl
SDSPI := sdspi
LLSDSPI := llsdspi
SDSPITX := spitxdata
SDSPIRX := spirxdata
SPICMD  := spicmd

.PHONY: $(LLSDSPI)
## {{{
$(LLSDSPI): $(LLSDSPI)_prf/PASS  $(LLSDSPI)_cvr/PASS $(LLSDSPI)_prfc/PASS
$(LLSDSPI): $(LLSDSPI)_prfa/PASS $(LLSDSPI)_cvra/PASS
$(LLSDSPI)_prf/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby prf
$(LLSDSPI)_prfa/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby prfa
$(LLSDSPI)_prfc/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby prfc
$(LLSDSPI)_cvr/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby cvr
$(LLSDSPI)_cvra/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby cvra
## }}}

.PHONY: $(SDSPI)
## {{{
SDSPIDEP := $(LLSDSPI)_prf/PASS $(SPICMD)_prf/PASS	\
	$(SDSPITX)_prf/PASS $(SDSPIRX)_prf/PASS
$(SDSPI): $(SDSPI)_prf/PASS $(SDSPIDEP) #  $(SDSPI)_cvr/PASS
$(SDSPI)_prf/PASS: $(RTL)/$(SDSPI).v $(SDSPI).sby $(SDSPIDEP)
	sby -f $(SDSPI).sby prf
#
# sdspi_cvr/PASS is a hard target to reach, since the SPI port will take
# a minimum of 32 clocks per transaction byte.  Hence, a 6-byte command
# followed by a 16-byte word will be a minimum of 704 clocks before any
# internal delays are factored in.
$(SDSPI)_cvr/PASS: $(RTL)/$(SDSPI).v $(SDSPI).sby $(SDSPIDEP)
	sby -f $(SDSPI).sby cvr
## }}}

.PHONY: $(SPICMD)
## {{{
$(SPICMD): $(SPICMD)_prf/PASS $(SPICMD)_cvr/PASS
$(SPICMD)_prf/PASS: $(RTL)/$(SPICMD).v $(SPICMD).sby
	sby -f $(SPICMD).sby prf
$(SPICMD)_cvr/PASS: $(RTL)/$(SPICMD).v $(SPICMD).sby
	sby -f $(SPICMD).sby cvr
## }}}

.PHONY: $(SDSPITX)
## {{{
$(SDSPITX): $(SDSPITX)_prf/PASS $(SDSPITX)_cvr/PASS
$(SDSPITX)_prf/PASS: $(RTL)/$(SDSPITX).v $(SDSPITX).sby
	sby -f $(SDSPITX).sby prf
$(SDSPITX)_cvr/PASS: $(RTL)/$(SDSPITX).v $(SDSPITX).sby
	sby -f $(SDSPITX).sby cvr
## }}}

.PHONY: $(SDSPIRX)
## {{{
$(SDSPIRX): $(SDSPIRX)_prf/PASS $(SDSPIRX)_cvr/PASS
$(SDSPIRX)_prf/PASS: $(RTL)/$(SDSPIRX).v $(SDSPIRX).sby
	sby -f $(SDSPIRX).sby prf
$(SDSPIRX)_cvr/PASS: $(RTL)/$(SDSPIRX).v $(SDSPIRX).sby
	sby -f $(SDSPIRX).sby cvr
## }}}

.PHONY: clean
## {{{
clean:
	rm -rf $(LLSDSPI)_*/
	rm -rf $(SPICMD)_*/
	rm -rf $(SDSPIRX)_*/
	rm -rf $(SDSPITX)_*/
	rm -rf $(SDSPI)_*/
## }}}
