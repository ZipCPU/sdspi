# SD-Card controller

This repository contains two Verilog RTL for controlling SD cards from an FPGA.
The [first and older controller](rtl/sdspi.v) handles SD cards via their
(optional) SPI interface.  The [second and newer controller](rtl/sdio.v) works
using the SDIO interface.  [It](rtl/sdio.v) should be able to handle both SDIO
and eMMC cards as a result.

## SPI-based controller

[The SDSPI controller](rtl/sdspi.v) exports an SD card controller interface
from internal to an FPGA to the rest of the FPGA core, while taking care of the
lower level details internal to the interface.  Unlike the [SDIO
controller](rtl/sdio.v) inthis respository, this controller focuses on the SPI
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

*Status*: The SDSPI IP is silicon proven.  It is no longer under active
  development.  It has been used successfully in several FPGA projects.  The
  components of this IP have formal proofs, which they are known to pass.  A
  Verilator C++ model also exists which can fairly faithfully represent an SD
  card's SPI interface.  A software library also exists which can act as a
  back end when using the FATFS library.

For more information, please consult the [specification document](doc/spec.pdf).

## SDIO

This repository also contains a [second and newer SD card
controller](rtl/sdio.v), designed to exploit the full SDIO protocol.  This
controller should work with both SDIO and eMMC chips, with the differences
between the two protocols handled by software.

The interface to this controller is roughly the same as that of the [SDSPI
controller](rtl/sdspi.v), although there are enough significant differences
to warrant a new user guide.

The controller is designed to support IO modes all the way up to the HS400
mode used by eMMC.  HS400 is a DDR mode based off of a 200MHz IO clock, using
a data strobe pin on return.  Also supported are an SDR mode using a 200MHz
clock, DDR and SDR modes using a 100MHz clock, as well as both DDR and SDR
support for integer divisions of the 100MHz clock, starting with a 50MHz clock
and going all the way down to 100kHz.  This is all based upon a nominal 100MHz
system clock, together with a 400MHz clock for SERDES support.  For designs
without 8:1 and 1:8 SERDES IO components, 100MHz and slower clocks are still
supported, depending upon whether or not DDR I/O components are available.
Both open-drain and push-pull IOs are supported, and the front end can switch
between the two as necessary based upon options within a PHY configuration
register.

*Status*: The SDIO controller is not yet silicon proven.  It is currently
  being tested in [its first FPGA project](https://github.com/ZipCPU/eth10g),
  where it will be used to control both an SD card as well as an eMMC card.
  Many of the components of this IP have formal proofs, which they are known
  to pass.  Notably missing among the component proofs is a proof of the data
  receiver.  A [Verilog model](bench/verilog/mdl_sdio.v) also exists which can
  be used to test this controller in simulation.

### Roadmap and TODO items

Although the RTL is now fully drafted, this project is far from finished.
Several key steps remain before it will be a viable product:

- **User Guide**: The user interface still needs to be documented.  Although
  it is similar to the SDSPI interface, it is different enough to require a
  new user guide.

- **SDIO Data Receiver**: While the [SDIO receiver](rtl/sdrxframe.v) works in
  a [simulated test bench environment](bench/verilog/tb_sdio.v), it has not
  yet been formally verified.  A proper formal proof of this component is
  still needed.

- **Formal Contracts**: Not all formal proofs properly test the "contract",
  that data provided on the incoming interface will be properly transferred
  to the downstream interface.

- **Test bench status**: While [test script(s) exist](bench/testscript), the
  [primary test driver](bench/verilog/sim_sdio.pl) does not properly return
  the status of any test back to its environment at present.

- **C++ Model**: An early Verilator C++ model has been drafted.  It needs to
  be finished, tested and posted.  No data strobe support is planned for this
  model at present.

- **Multi-block**: While simulation tests have demonstrated CMD17,
  `READ_SINGLE_BLOCK`, and CMD24, `WRITE_BLOCK`, the multiple block commands
  have not yet been tested.  These include CMD18, `READ_MULTIPLE_BLOCK`, and
  CMD25, `WRITE_MULTIPLE_BLOCK`.

- **R1b**: Busy bit (R1b) functionality may have been forgotten.

- **SW**: Control software is still being written.

- **CRC Tokens**: eMMC CRC tokens are not (yet) supported.

- **DW > 32bits**: The controller currently only works with a Wishbone bus
  width of 32-bits.  An extension is planned to extend the bus to arbitrary
  widths for greater throughput.  The goal of this extension would be to allow
  an external DMA to drive data through this design at the full speed of the
  bus.

- **OPT_DMA**: An optional DMA extension is planned, to allow data blocks to
  be transferred to memory at the full speed of the internal bus without
  CPU intervention.

- **eMMC Boot mode**: No plan exists to support eMMC boot mode at present.

# Commercial Applications

Should you find the GPLv3 license insufficient for your needs, other licenses
can be purchased from Gisselquist Technology, LLC.
