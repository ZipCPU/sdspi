################################################################################
##
## Filename: 	Makefile
## {{{
## Project:	SD-Card controller, using a shared SPI interface
##
## Purpose:	Coordinate building the specification for this core, the
##		Verilator Verilog check, and any bench software.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2016-2023, Gisselquist Technology, LLC
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
all:	verilated test
## Some definitions
## {{{
BENCH := `find bench -name Makefile` `find bench -name "*.cpp"` `find bench -name "*.h"`
RTL   := `find rtl -name "*.v"` `find rtl -name Makefile`
NOTES := # `find . -name "*.txt"` `find . -name "*.html"`
SW=
#SW    := `find sw -name "*.cpp"` `find sw -name "*.h"`	\
#	`find sw -name "*.sh"` `find sw -name "*.py"`	\
#	`find sw -name "*.pl"` `find sw -name Makefile`
DEVSW=
YYMMDD:=`date +%Y%m%d`
SUBMAKE := $(MAKE) --no-print-directory -C
## }}}

.PHONY: archive
## {{{
archive:
	tar --transform s,^,$(YYMMDD)-sdspi/, -chjf $(YYMMDD)-sdspi.tjz $(BENCH) $(SW) $(RTL) $(NOTES)
## }}}

.PHONY: verilated rtl
## {{{
rtl: verilated
verilated:
	$(SUBMAKE) rtl
## }}}

.PHONY: doc
## {{{
# The documents target does not get, nor should it be, made automatically.
# This is because the project is intended to be shipped with the documents
# automatically built, and I don't necessarily expect all those who download
# this "core" to have LaTeX distribution necessary to rebuild the specification
# and GPL LaTeX documents into their PDF results.
doc:
	$(SUBMAKE) doc
## }}}

.PHONY: formal
## {{{
formal:
	$(SUBMAKE) bench/formal
## }}}

.PHONY: bench
## {{{
bench: rtl
	$(SUBMAKE) bench/cpp
## }}}

.PHONY: test
## {{{
test: formal rtl
	$(SUBMAKE) bench/cpp test
## }}}
#.PHONY: sw
# sw:
#	cd sw ; $(MAKE) --no-print-directory

.PHONY: clean
## {{{
clean:
	$(SUBMAKE) rtl clean
	$(SUBMAKE) doc clean
	$(SUBMAKE) bench/formal clean
	$(SUBMAKE) bench/cpp    clean
## }}}
