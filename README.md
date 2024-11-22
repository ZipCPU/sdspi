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
originally written), or in general any time the full 7--bit, bi--directional
interface to the SD card has not been implemented.  Further, for those who are
die--hard Verilog authors, this core is written in Verilog as opposed to the
[XESS provided demonstration SD Card controller found on
GitHub](https://github.com/xesscorp/VHDL\_Lib/SDCard.vhd), which was written
in VHDL.  For those who are not such die--hard Verilog authors, this controller
provides a lower level interface to the card than other controllers. 
Whereas the [XESS controller](https://github.com/xesscorp/VHDL\_Lib/SDCard.vhd)
will automatically start up the card and interact with it, this controller
requires [external software](sw/sdspidrv.c) to be used when interacting with
the card.  This makes this SDSPI controller both more versatile, in the face
of potential changes to the card interface, but also less turn-key.

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

### Roadmap and TODO items

*Status*: The SDSPI IP is **silicon proven**.  It is no longer under active
  development.  It has been used successfully in several FPGA projects.  The
  components of this IP have formal proofs, which they are known to pass.  A
  Verilator [C++ model](bench/cpp/sdspisim.cpp) also exists which can fairly
  faithfully represent an SD card's SPI interface.  A [software
  library](sw/sdspidrv.c) also exists which can act as a back end when using
  the [FATFS library](http://elm-chan.org/fsw/ff/00index_e.html).

- **AXI Support**: The [SDSPI controller](doc/sdspi.pdf) has no support for
  an AXI environment.  The RTL modifications required to provide AXI-Lite
  support to this controller would be minor.  Testbench modifications would
  be more significant.

- **All-Verilog Test bench**: The [SDSPI controller](doc/sdspi.pdf) has a
  [C++ model](bench/cpp/sdspisim.cpp) only for simulation based testing.
  There is no all Verilog test bench at present, nor do I have any plans to
  develop one.

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
  both an SD card as well as an eMMC chip.  It is also now working successfully
  in a [second project](https://github.com/ZipCPU/videozip/).  Many of the
  components of this IP have formal proofs, which they are known to pass.

  Notably missing among the formal component proofs is a proof of the [front
  end](rtl/sdfrontend.v).  The [front end](rtl/sdfrontend.v)'s verification
  depends upon integrated simulation testing.

  Both [Verilog](bench/verilog/mdl_sdio.v) and [C++](bench/cpp/sdiosim.cpp)
  models have been built which can be used to test this controller in
  simulation.  Unlike the [Verilog SDIO](bench/verilog/mdl_sdio.v) model, the
  [C++ SDIO](bench/cpp/sdiosim.cpp) supports a file-backed memory, allowing
  full software testing with filesystem(s) present.  All three simulation
  components have been now been tested successfully: the Verilog
  [SDIO](bench/verilog/mdl_sdio.v) and [eMMC](bench/verilog/mdl_emmc.v)
  models, as well as the [SDIO C++](bench/cpp/sdiosim.cpp) model.

For more information, please consult the [SDIO user guide](doc/sdio.pdf).

### Roadmap and TODO items

Although the initial RTL has both been fully drafted and successfully hardware
tested, this project is far from finished.  Several key steps remain before
this controller will be a completed product:

- **Multi-block**: Multiple block commands have been demonstrated in
  simulation when using the Verilog [SDIO model](bench/verilog/mdl_sdio.v).
  Multiblock simulation support is still lacking in the Verilog [eMMC
  model](bench/verilog/mdl_emmc.v).

  Multiblock commands form the basis for the DMA's operation.

- **`OPT_DMA`**: An optional DMA is now available, and passing tests in silicon.

  Only the Wishbone version of the DMA controller exists at present.

  Although components exist in my [wb2axip
  repository](https://github.com/ZipCPU/wb2axip) which could support an AXI
  DMA, these components have neither been integrated nor tested as part of this
  design.  Other user's have successfully connected external AXI
  [MM2S](https://github.com/ZipCPU/wb2axip/blob/master/rtl/aximm2s.v) and
  [S2MM](https://github.com/ZipCPU/wb2axip/blob/master/rtl/axis2mm.v)
  components to the AXI stream interface, and have thus demonstrated successful
  DMA support in AXI environments.

- **STREAM DMA**: At customer request, hooks now exist for an (optional)
  stream DMA interface.  This interface will accept an AXI stream input,
  and/or an AXI stream output.  Data present on the AXI stream input may
  then be written directly to the device.  Reads from the device may also
  produce data at the output stream.

  This interface is now supported and [tested via
  simulation](bench/testscript/sdstream.v).  No known issues exist.  It does
  have some software quirks:

  - When using the stream interface, the DMA address should be set to -1.  This
    selects the stream interface as either source or destination.  (The actual
    controller command will indicate the direction of the transfer.)
  - Any memory source ([MM2S](https://github.com/ZipCPU/wb2axip/blob/master/rtl/aximm2s.v))
    should be configured for the full transfer length--potentially many blocks.
  - There is no TLAST stream input (slave).
  - When the external (SD or eMMC) device is the data source ([S2MM](https://github.com/ZipCPU/wb2axip/blob/master/rtl/axis2mm.v)),
    the TLAST signal will be set at the end of each 512B block.  This may
    require the external DMA to be configured to transfer data one block at a
    time, or perhaps to ignore the TLAST signal.
  - Transfer errors (failing CRCs, non-responsive cards, etc.) may cause the streams to be unsynchronized.  To fix, the design may be given a soft reset (if necessary), and the external [MM2S](https://github.com/ZipCPU/wb2axip/blob/master/rtl/aximm2s.v)/[S2MM](https://github.com/ZipCPU/wb2axip/blob/master/rtl/axis2mm.v) DMAs may also need to be given similar resets.

- **C++ Model**: A [Verilator C++ model of an SDIO
  component](bench/cpp/sdiosim.cpp) is now a part of the repository.  This
  model has demonstrated tremendous utility when doing software testing.

  No C++ eMMC model exists at present.

- **SW Testing**: [Control software](sw/) has been written, and has been
  used to demonstrate both [SDIO](sw/sdiodrv.c) and [EMMC](sw/emmcdrvr.c)
  performance.  This software is designed to work with the [FATFS
  library](http://elm-chan.org/fsw/ff/00index_e.html).

  Software testing is currently taking place as part of the integrated test
  benches associated with separate repositories, such as the
  [VideoZip](https://github.com/ZipCPU/videozip) repository that contains both
  this component and the [ZipCPU](https://zipcpu.com/about/zipcpu.html).

- **AXI Support**: This design has also been demonstrated in AXI environments.
  The control interface has an AXI-Lite port which can be used to interact
  with the IP.  A flag exists to swap endianness, so that the design will be
  properly little endian when using this interface.  At present, however,
  there is no integrated AXI DMA master capability--only AXI stream ports.
  (Integrated AXI DMA master support is planned, just not funded at present.)

  When coupled with external AXI
  [MM2S](https://github.com/ZipCPU/wb2axip/blob/master/rtl/aximm2s.v) and
  [S2MM](https://github.com/ZipCPU/wb2axip/blob/master/rtl/axis2mm.v)
  components, the IP may sufficiently provide for the needs of most AXI
  environments.

- **CRC Tokens**: CRC token's are 5b response values, indicating whether
  or not a page has transferred successfully.
  The [frontend](rtl/sdfrontend.v) can successfully recognize those CRC tokens
  following block write transfers.

  Failure to receive a CRC token when one is expected will (now) generate an
  error condition, as will receiving a negative CRC acknowledgment.

- **eMMC Boot mode**: No plan exists to support eMMC boot mode (at present).
  This decision will likely be revisited in the future.

  Some (untested, preliminary) support exists for boot mode in the Verilog
  [eMMC model](bench/verilog/mdl_emmc.v).

- **eMMC Collision Detection**: [Collision detection remains an ongoing issue
  with eMMC support](https://github.com/ZipCPU/sdspi/issues/13).  This issue
  is limited to the `GO_IRQ_STATE` command, and specifically to the case where
  both controller and device attempt to leave the IRQ state at the same time.
  Without collision support, the message to leave the IRQ state may be
  corrupted on return.  This should be detectable via a bad CRC on the command
  line.

# Logic usage

Current logic usage is being tracked for iCE40 and Xilinx 6-LUT devices
in the [usage.txt](rtl/usage.txt) file in the [RTL/](rtl/) directory.

# Commercial Applications

Should you find the GPLv3 license insufficient for your needs, other licenses
may be purchased from Gisselquist Technology, LLC.
