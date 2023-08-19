################################################################################
##
## Filename: 	Makefile
## {{{
## Project:	SD-Card controller, using a shared SPI interface
##
## Purpose:	To coordinate the build of documentation PDFs from their
##		LaTeX sources.
##
##	Targets include:
##		all		Builds all documents
##		gpl-3.0.pdf	Builds the GPL license these files are released
##					under.
##		spec.pdf	Builds the specification for the SDSPI
##					controller.
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
##
################################################################################
##
## }}}
all:	gpl-3.0.pdf sdspi.pdf sdio.pdf
DSRC := src

.PHONY: gpl
gpl: gpl-3.0.pdf
## {{{
gpl-3.0.pdf: $(DSRC)/gpl-3.0.tex
	latex $(DSRC)/gpl-3.0.tex
	latex $(DSRC)/gpl-3.0.tex
	dvips -q -z -t letter -P pdf -o gpl-3.0.ps gpl-3.0.dvi
	ps2pdf -dAutoRotatePages=/All gpl-3.0.ps gpl-3.0.pdf
	rm gpl-3.0.dvi gpl-3.0.log gpl-3.0.aux gpl-3.0.ps
## }}}

.PHONY: spec
spec: sdspi sdio

.PHONY: sdspi
## {{{
sdspi: sdspi.pdf
sdspi.pdf: $(DSRC)/sdspi.tex $(DSRC)/gqtekspec.cls $(DSRC)/GT.eps
	cd $(DSRC)/; latex sdspi.tex
	cd $(DSRC)/; latex sdspi.tex
	cd $(DSRC)/; dvips -q -z -t letter -P pdf -o ../sdspi.ps sdspi.dvi
	ps2pdf -dAutoRotatePages=/All sdspi.ps sdspi.pdf
	-grep -i warning $(DSRC)/sdspi.log
	@rm -f $(DSRC)/sdspi.dvi $(DSRC)/sdspi.log
	@rm -f $(DSRC)/sdspi.aux $(DSRC)/sdspi.toc
	@rm -f $(DSRC)/sdspi.lot $(DSRC)/sdspi.lof
	@rm -f $(DSRC)/sdspi.out sdspi.ps
## }}}

.PHONY: sdio
## {{{
sdio: sdio.pdf
sdio.pdf: $(DSRC)/sdio.tex $(DSRC)/gqtekspec.cls $(DSRC)/GT.eps
	cd $(DSRC)/; latex sdio.tex
	cd $(DSRC)/; latex sdio.tex
	cd $(DSRC)/; dvips -q -z -t letter -P pdf -o ../sdio.ps sdio.dvi
	ps2pdf -dAutoRotatePages=/All sdio.ps sdio.pdf
	-grep -i warning $(DSRC)/sdio.log
	@rm -f $(DSRC)/sdio.dvi $(DSRC)/sdio.log
	@rm -f $(DSRC)/sdio.aux $(DSRC)/sdio.toc
	@rm -f $(DSRC)/sdio.lot $(DSRC)/sdio.lof
	@rm -f $(DSRC)/sdio.out sdio.ps
## }}}

.PHONY: clean
## {{{
clean:
	rm -f $(DSRC)/sdspi.dvi $(DSRC)/sdspi.log
	rm -f $(DSRC)/sdspi.aux $(DSRC)/sdspi.toc
	rm -f $(DSRC)/sdspi.lot $(DSRC)/sdspi.lof
	rm -f $(DSRC)/sdspi.out sdspi.ps sdspi.pdf
	#
	rm -f $(DSRC)/sdio.dvi $(DSRC)/sdio.log
	rm -f $(DSRC)/sdio.aux $(DSRC)/sdio.toc
	rm -f $(DSRC)/sdio.lot $(DSRC)/sdio.lof
	rm -f $(DSRC)/sdio.out sdio.ps sdio.pdf
	rm -f gpl-3.0.pdf
## }}}
