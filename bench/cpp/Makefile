################################################################################
##
## Filename:	bench/cpp/Makefile
## {{{
## Project:	SD-Card controller
##
## Purpose:	Nothing currently.  Eventually, if I create a proper bench
##		test, rather than just running this on real hardware as part of
##	another project (XuLA2-LX25 SoC), this Makefile will coordinate the
##	building of that bench test software.
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
CXX:= g++
OBJDIR := obj-pc
RTLD   := ../../rtl
ifeq ($(VERILATOR_ROOT),)
VERILATOR := verilator
VERILATOR_ROOT ?= $(shell bash -c '$(VERILATOR) -V|grep VERILATOR_ROOT | head -1 | sed -e " s/^.*=\s*//"')
else
VERILATOR := $(VERILATOR_ROOT)/bin/verilator
endif
export $(VERILATOR)
VROOT := $(VERILATOR_ROOT)
VINCD := $(VERILATOR_ROOT)/include
INCS   := -I $(VINCD) -I $(VINCD)/vlstd -I $(RTLD)/obj_dir
CFLAGS  := -O3
PROGRAMS := tb_sdspi
all: $(PROGRAMS)

SOURCES := tb_sdspi.cpp sdspisim.cpp
HEADERS := sdspisim.h wb_tb.h testb.h
OBJECTS := $(addprefix $(OBJDIR)/,$(subst .cpp,.o,$(SOURCES)))
VLSRCS  := verilated.cpp verilated_vcd_c.cpp verilated_threads.cpp
VLOBJS  := $(addprefix $(OBJDIR)/,$(subst .cpp,.o,$(VLSRCS)))
VLIBS   := $(RTLD)/obj_dir/Vsdspi__ALL.a

## Build patterns: .cpp to .o
## {{{
$(OBJDIR)/%.o: %.cpp
	$(mk-objdir)
	$(CXX) $(CFLAGS) $(INCS) -c $< -o $@

$(OBJDIR)/%.o: $(VROOT)/include/%.cpp
	$(mk-objdir)
	$(CXX) $(CFLAGS) $(INCS) -c $< -o $@
## }}}

## tb_sdspi
## {{{
tb_sdspi: $(OBJECTS) $(VLIBS) $(VLOBJS)
	$(CXX) $(CFLAGS) $(OBJECTS) $(VLIBS) $(VLOBJS) -lpthread -o $@
## }}}

sdcard.img:
	dd if=/dev/zero of=sdcard.img bs=512 count=131072
	mkfs.fat -F 16 sdcard.img

## test
## {{{
test: tb_sdspi sdcard.img
	./tb_sdspi
## }}}

define	mk-objdir
	@bash -c "if [ ! -e $(OBJDIR) ]; then mkdir -p $(OBJDIR)/; fi"
endef

tags: $(SOURCES) $(HEADERS)
	@ctags $(SOURCES)

.PHONY: clean
## {{{
clean:
	rm -rf $(OBJDIR)/ $(PROGRAMS)
	rm -f *.vcd
## }}}

## Dependency handling
## {{{
define	build-depends
	@echo "Building dependencies"
	$(mk-objdir)
	@$(CXX) $(CPPFLAGS) $(INCS) -MM zipcpu_tb.cpp $(VLIB) $(SOURCES) > $(OBJDIR)/xdepends.txt
	@sed -e 's/^.*.o: /$(OBJDIR)\/&/' < $(OBJDIR)/xdepends.txt > $(OBJDIR)/depeends.txt
	@rm $(OBJDIR)/xdepends.txt
endef

.PHONY: depends
depends: tags $(OBJDIR)/
	$(build-depends)

-include $(OBJDIR)/depends.txt
## }}}
