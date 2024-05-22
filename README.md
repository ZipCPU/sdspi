# SD-Card controller

This repository contains two Verilog hardware RTL controllers for handling
SD cards from an FPGA.  The [first and older controller](rtl/sdspi.v) handles
SD cards via their (optional) SPI interface.  The [second and newer
controller](rtl/sdio.v) works using the SDIO interface.  This [second
controller](rtl/sdio.v) has also been demonstrated to handle eMMC cards as well.

## SPI-based controller

[The SDSPI controller](rtl/sdspi.v) exports an SD card controller interface
from internal to an FPGA to the rest of the FPGA core, while taking care of the
lower level details internal to the interface.  Unlike the [SDIO
controller](rtl/sdio.v) in this respository, this controller focuses on the SPI
interface of the SD Card.  While this is a slower interface, the SPI interface
is necessary to access the card when using a [XuLA2
board](http://www.xess.com/shop/product/xula2-lx25/) (for which it was
originally written), or in general any time the full 9--bit, bi--directional
interface to the SD card has not been implemented.  Further, for those who are
die--hard Verilog authors, this core is written in Verilog as opposed to the
[XESS provided demonstration SD Card controller found on
GitHub](https://github.com/xesscorp/VHDL\_Lib/SDCard.vhd), which was written
in VHDL.  For those who are not such die--hard Verilog authors, this controller
provides a lower level interface to the card than these other controllers. 
Whereas the [XESS controller](https://github.com/xesscorp/VHDL\_Lib/SDCard.vhd)
will automatically start up the card and interact with it, this controller
requires external software to be used when interacting with the card.  This
makes this SDSPI controller both more versatile, in the face of potential
changes to the card interface, but also less turn-key.

While this core was written for the purpose of being used with the
[ZipCPU](https://github.com/ZipCPU/zipcpu), as enhanced by the Wishbone DMA
controller used by the ZipCPU, nothing in this core prevents it from being
used with any other architecture that supports the 32-bit Wishbone interface
of this core.

This core has been written as a wishbone slave, not a master.  Using the core
together with a separate master, such as a CPU or a DMA controller, only makes
sense.  This design choice, however, also restricts the core from being able to
use the multiple block write or multiple block read commands, restricting it to 
single block read and write commands alone.

*Status*: The SDSPI IP is **silicon proven**.  It is no longer under active
  development.  It has been used successfully in several FPGA projects.  The
  components of this IP have formal proofs, which they are known to pass.  A
  Verilator C++ model also exists which can fairly faithfully represent an SD
  card's SPI interface.  A software library also exists which can act as a
  back end when using the [FATFS library](http://elm-chan.org/fsw/ff/00index_e.html).

For more information, please consult the [SDSPI user guide](doc/sdspi.pdf).

## SDIO

This repository also contains a [second and newer SD card
controller](rtl/sdio.v), designed to exploit both the full SDIO protocol and
the 8b EMMC protocol--either with or without data strobes.  This controller
has been tested against both SDIO and eMMC chips, with the differences between
the two types of chips handled by software.

The interface to this controller is roughly the same as that of the [SDSPI
controller](rtl/sdspi.v), although there are enough significant differences
to warrant a [separate user guide](doc/sdio.pdf).

The controller is designed to support IO modes all the way up to the HS400
mode used by eMMC.  HS400 is an eMMC DDR mode based off of a 200MHz IO clock,
using a data strobe pin on return.  Also supported are an SDR mode using a
200MHz clock, DDR and SDR modes using a 100MHz clock, as well as both DDR and
SDR support for integer divisions of the 100MHz clock, starting with a 50MHz
clock and going all the way down to 100kHz.  This is all based upon a nominal
100MHz system clock, together with a 400MHz clock for SERDES support.  For
designs without 8:1 and 1:8 SERDES IO components, 100MHz and slower clocks are
still supported, depending upon whether or not DDR I/O components are available.
Both open-drain and push-pull IOs are supported, and the front end can switch
between the two as necessary based upon options within a PHY configuration
register.  No support is planned for any of the UHS-II protocols.

Both Wishbone and AXI-Lite interfaces are supported.

*Status*: The SDIO controller has now been **silicon proven**.  It is currently
  working successfully in [its first FPGA
  project](https://github.com/ZipCPU/eth10g), where it is being used to control
  both an SD card as well as an eMMC chip.  Many of the components of this IP
  have formal proofs, which they are known to pass.

  Notably missing among the component proofs is a proof of the [front
  end](rtl/sdfrontend.v).  The [front end](rtl/sdfrontend.v)'s verification
  depends upon integrated simulation testing.

  Both [Verilog](bench/verilog/mdl_sdio.v) and [C++](bench/cpp/sdiosim.cpp)
  models have been built which can be used to test this controller in
  simulation, although only the Verilog [SDIO](bench/verilog/mdl_sdio.v) and
  [eMMC](bench/verilog/mdl_emmc.v) models have been tested to date.

For more information, please consult the [SDIO user guide](doc/sdio.pdf).

### Roadmap and TODO items

Although the initial RTL has both been fully drafted and successfully hardware
tested, this project is far from finished.  Several key steps remain before
this controller will be a completed product:

- **Multi-block**: Multiple block commands have been demonstrated in
  simulation for the SDIO model.  While eMMC multiblock support exists, it
  hasn't yet been tested, and will likely fail.

  Multiblock commands form the basis for the DMA's operation.

- **OPT\_DMA**: An optional DMA is now available, and passing tests in silicon.

  Only the Wishbone version of the DMA controller exists at present.  Although
  some components exist in my [wb2axip
  repository](https://github.com/ZipCPU/wb2axip) which could support an AXI
  DMA, these components have neither been integrated nor tested as part of this
  design.

  Since the [eMMC model](bench/verilog/mdl_emmc.v) doesn't yet support the
  multi-block commands required by the DMA, DMA simulation testing has been
  limited to the [SDIO model](bench/verilog/mdl_sdio.v).

- **STREAM DMA**: At customer request, hooks now exist for an (optional)
  stream DMA interface.  This interface will accept an AXI stream input,
  and/or an AXI stream output.  Data present on the AXI stream input may
  then be written directly to the device.  Reads from the device may also
  produce data at the output stream.  At present, the stream inputs must
  be a minimum of 32bits, and a power of two in width.  The stream outputs
  must either be 32bits or the full bus width.  No provision exist for stream
  data less than a word in size.

  This interface will need to be tested together with the pending hardware
  tests for the DMA.

- **C++ Model**: An early [Verilator C++ model of an SDIO
  component](bench/cpp/sdiosim.cpp) has been drafted.  It needs to be finished
  and tested.  No C++ eMMC model exists at present, nor is any data strobe
  support is planned.

- **SW Testing**: [Control software](sw/) has been written, and has been
  used to demonstrate both [SDIO](sw/sdiodrv.c) and [EMMC](sw/emmcdrvr.c)
  performance.  This software is designed to work with the [FATFS
  library](http://elm-chan.org/fsw/ff/00index_e.html).

  An integrated test bench exists for testing this software from a
  [ZipCPU](https://zipcpu.com/about/zipcpu.html), it just hasn't (yet) been
  tested.  Because this integrated test bench will require integration with an
  external project (i.e. the [ZipCPU](https://zipcpu.com/about/zipcpu.html),
  further work is on hold pending a decision on how to integrate multiple
  dissimilar design components together for this purpose.

- **AXI Support**: A version exists in the dev branch that supports an AXI-Lite
  interface.

- **eMMC Boot mode**: No plan exists to support eMMC boot mode (at present).
  This decision will likely be revisited in the future.

  Boot mode may require support for eMMC CRC tokens, which aren't (yet)
  supported.  These are 8'bit return values, indicating whether or not a
  page has been read or written and passes its CRC check.

  Some (untested) support exists for boot mode in the eMMC model.

# Logic usage

Current logic usage is being tracked for iCE40 and Xilinx 6-LUT devices
in the [usage.txt](rtl/usage.txt) file in the [RTL/](rtl/) directory.

# Commercial Applications

Should you find the GPLv3 license insufficient for your needs, other licenses
may be purchased from Gisselquist Technology, LLC.
