################################################################################
##
## Filename: 	Makefile
##
## Project:	SD-Card controller, using a shared SPI interface
##
## Purpose:	
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
##
## Copyright (C) 2019, Gisselquist Technology, LLC
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
.PHONY: all
all:	llsdspi

RTL := ../../rtl
LLSDSPI := llsdspi

.PHONY: $(LLSDSPI)
$(LLSDSPI): $(LLSDSPI)_prf/PASS  $(LLSDSPI)_cvr/PASS
$(LLSDSPI): $(LLSDSPI)_prfa/PASS $(LLSDSPI)_cvra/PASS
$(LLSDSPI)_prf/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby prf
$(LLSDSPI)_prfa/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby prfa
$(LLSDSPI)_cvr/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby cvr
$(LLSDSPI)_cvra/PASS: $(RTL)/$(LLSDSPI).v $(LLSDSPI).sby
	sby -f $(LLSDSPI).sby cvra

.PHONY: clean
clean:
	rm -rf $(LLSDSPI)_*/
