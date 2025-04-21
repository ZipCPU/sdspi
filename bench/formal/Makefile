################################################################################
##
## Filename: 	Makefile
## {{{
## Project:	SD-Card controller(s)
##
## Purpose:	
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2019-2025, Gisselquist Technology, LLC
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
all:	sdspi llsdspi spicmd spitxdata spirxdata sdcmd sdwb sdaxil sdckgen sdtxframe sdrxframe report

RTL := ../../rtl
## SDSPI components
SDSPI := sdspi
LLSDSPI := llsdspi
SDSPITX := spitxdata
SDSPIRX := spirxdata
SPICMD  := spicmd
## SDIO components
SDCMD := sdcmd
SDWB  := sdwb
SDAXIL:= sdaxil
SDCK  := sdckgen
SDTX  := sdtxframe
SDRX  := sdrxframe

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
	$(SDSPITX)_prf/PASS $(SDSPIRX)_prf/PASS		\
	$(SDSPITX)_prfle/PASS $(SDSPIRX)_prfle/PASS
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
$(SDSPITX)_prfle/PASS: $(RTL)/$(SDSPITX).v $(SDSPITX).sby
	sby -f $(SDSPITX).sby prfle
$(SDSPITX)_cvr/PASS: $(RTL)/$(SDSPITX).v $(SDSPITX).sby
	sby -f $(SDSPITX).sby cvr
## }}}

.PHONY: $(SDSPIRX)
## {{{
$(SDSPIRX): $(SDSPIRX)_prf/PASS $(SDSPIRX)_cvr/PASS
$(SDSPIRX)_prf/PASS: $(RTL)/$(SDSPIRX).v $(SDSPIRX).sby
	sby -f $(SDSPIRX).sby prf
$(SDSPIRX)_prfle/PASS: $(RTL)/$(SDSPIRX).v $(SDSPIRX).sby
	sby -f $(SDSPIRX).sby prfle
$(SDSPIRX)_cvr/PASS: $(RTL)/$(SDSPIRX).v $(SDSPIRX).sby
	sby -f $(SDSPIRX).sby cvr
## }}}

.PHONY: $(SDCMD)
## {{{
$(SDCMD): $(SDCMD)_prf/PASS $(SDCMD)_prfds/PASS $(SDCMD)_cvr/PASS
$(SDCMD)_prf/PASS: $(RTL)/$(SDCMD).v $(SDCMD).sby
	sby -f $(SDCMD).sby prf
$(SDCMD)_prfds/PASS: $(RTL)/$(SDCMD).v $(SDCMD).sby
	sby -f $(SDCMD).sby prfds
$(SDCMD)_cvr/PASS: $(RTL)/$(SDCMD).v $(SDCMD).sby
	sby -f $(SDCMD).sby cvr
## }}}

.PHONY: $(SDWB)
## {{{
$(SDWB): $(SDWB)_prf/PASS $(SDWB)_cvr/PASS
$(SDWB): $(SDWB)_prfds/PASS $(SDWB)_prfdr/PASS $(SDWB)_prfdma/PASS
$(SDWB)_prf/PASS: $(SDWB).sby $(RTL)/$(SDWB).v fwb_slave.v fwb_register.v
	sby -f $(SDWB).sby prf
$(SDWB)_prfdma/PASS: $(SDWB).sby $(RTL)/$(SDWB).v fwb_slave.v fwb_register.v
	sby -f $(SDWB).sby prfdma
$(SDWB)_prfds/PASS: $(SDWB).sby $(RTL)/$(SDWB).v fwb_slave.v fwb_register.v
	sby -f $(SDWB).sby prfds
$(SDWB)_prfdr/PASS: $(SDWB).sby $(RTL)/$(SDWB).v fwb_slave.v fwb_register.v
	sby -f $(SDWB).sby prfdr
$(SDWB)_cvr/PASS: $(SDWB).sby $(RTL)/$(SDWB).v fwb_slave.v fwb_register.v
	sby -f $(SDWB).sby cvr
## }}}

.PHONY: $(SDAXIL)
## {{{
$(SDAXIL): $(SDAXIL)_prf/PASS $(SDAXIL)_cvr/PASS
$(SDAXIL): $(SDAXIL)_prfds/PASS $(SDAXIL)_prfdr/PASS $(SDAXIL)_prfdma/PASS
$(SDAXIL)_prf/PASS: $(SDAXIL).sby $(RTL)/$(SDAXIL).v faxil_slave.v faxil_register.v
	sby -f $(SDAXIL).sby prf
$(SDAXIL)_prfdma/PASS: $(SDAXIL).sby $(RTL)/$(SDAXIL).v faxil_slave.v faxil_register.v
	sby -f $(SDAXIL).sby prfdma
$(SDAXIL)_prfds/PASS: $(SDAXIL).sby $(RTL)/$(SDAXIL).v faxil_slave.v faxil_register.v
	sby -f $(SDAXIL).sby prfds
$(SDAXIL)_prfdr/PASS: $(SDAXIL).sby $(RTL)/$(SDAXIL).v faxil_slave.v faxil_register.v
	sby -f $(SDAXIL).sby prfdr
$(SDAXIL)_cvr/PASS: $(SDAXIL).sby $(RTL)/$(SDAXIL).v faxil_slave.v faxil_register.v
	sby -f $(SDAXIL).sby cvr
## }}}

.PHONY: $(SDCK)
## {{{
$(SDCK): $(SDCK)_prf/PASS $(SDCK)_prfd/PASS $(SDCK)_prf8/PASS
$(SDCK): $(SDCK)_cvr/PASS $(SDCK)_cvrd/PASS $(SDCK)_cvr8/PASS
$(SDCK)_prf/PASS: $(SDCK).sby fclk.v $(RTL)/$(SDCK).v
	sby -f $(SDCK).sby prf
$(SDCK)_prfd/PASS: $(SDCK).sby fclk.v $(RTL)/$(SDCK).v
	sby -f $(SDCK).sby prfd
$(SDCK)_prf8/PASS: $(SDCK).sby fclk.v $(RTL)/$(SDCK).v
	sby -f $(SDCK).sby prf8
$(SDCK)_cvr/PASS: $(SDCK).sby fclk.v $(RTL)/$(SDCK).v
	sby -f $(SDCK).sby cvr
$(SDCK)_cvrd/PASS: $(SDCK).sby fclk.v $(RTL)/$(SDCK).v
	sby -f $(SDCK).sby cvrd
$(SDCK)_cvr8/PASS: $(SDCK).sby fclk.v $(RTL)/$(SDCK).v
	sby -f $(SDCK).sby cvr8
## }}}

.PHONY: $(SDTX) txframe
## {{{
txframe: $(SDTX)
$(SDTX): $(SDTX)_prf/PASS  $(SDTX)_cvr/PASS
$(SDTX): $(SDTX)_prfs/PASS $(SDTX)_cvrs/PASS
$(SDTX): $(SDTX)_prf4/PASS $(SDTX)_prfs4/PASS
$(SDTX)_prf/PASS: $(RTL)/$(SDTX).v fclk.v $(SDTX).sby
	sby -f $(SDTX).sby prf
$(SDTX)_prf4/PASS: $(RTL)/$(SDTX).v fclk.v $(SDTX).sby
	sby -f $(SDTX).sby prf4
$(SDTX)_prfs/PASS: $(RTL)/$(SDTX).v fclk.v $(SDTX).sby
	sby -f $(SDTX).sby prfs
$(SDTX)_prfs4/PASS: $(RTL)/$(SDTX).v fclk.v $(SDTX).sby
	sby -f $(SDTX).sby prfs4
$(SDTX)_cvr/PASS: $(RTL)/$(SDTX).v fclk.v $(SDTX).sby
	sby -f $(SDTX).sby cvr
$(SDTX)_cvrs/PASS: $(RTL)/$(SDTX).v fclk.v $(SDTX).sby
	sby -f $(SDTX).sby cvrs
## }}}

.PHONY: $(SDRX) rxframe
## {{{
rxframe: $(SDRX)
$(SDRX): $(SDRX)_prf/PASS  $(SDRX)_prfs/PASS $(SDRX)_cvr/PASS
$(SDRX): $(SDRX)_prf4/PASS $(SDRX)_prfs4/PASS
$(SDRX)_prf/PASS: $(RTL)/$(SDRX).v $(SDRX).sby
	sby -f $(SDRX).sby prf
$(SDRX)_prf4/PASS: $(RTL)/$(SDRX).v $(SDRX).sby
	sby -f $(SDRX).sby prf4
$(SDRX)_prfs/PASS: $(RTL)/$(SDRX).v $(SDRX).sby
	sby -f $(SDRX).sby prfs
$(SDRX)_prfs4/PASS: $(RTL)/$(SDRX).v $(SDRX).sby
	sby -f $(SDRX).sby prfs4
$(SDRX)_cvr/PASS: $(RTL)/$(SDRX).v $(SDRX).sby
	sby -f $(SDRX).sby cvr
## }}}


.PHONY: report
## {{{
report:
	+perl genreport.pl > report.html
## }}}

.PHONY: clean
## {{{
clean:
	rm -rf $(LLSDSPI)_*/
	rm -rf $(SPICMD)_*/
	rm -rf $(SDSPIRX)_*/
	rm -rf $(SDSPITX)_*/
	rm -rf $(SDSPI)_*/
	rm -rf $(SDCK)_*/
	rm -rf $(SDCMD)_*/
	rm -rf $(SDRX)_*/
	rm -rf $(SDTX)_*/
	rm -rf $(SDWB)_*/
## }}}
