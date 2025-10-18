# SD-Card controller

This repository contains three Verilog hardware RTL controllers for handling
SD cards from an FPGA.  The [first and older controller](rtl/sdspi/sdspi.v)
handles SD cards via their (optional) SPI interface.  The [second and newer
controller](rtl/sdio/sdio.v) works using the SDIO interface.  This [second
controller](rtl/sdio/sdio.v) has also been demonstrated to handle eMMC cards
as well.  A [third controller](rtl/sdslave/sdslave.v) has also been added to
the mix.  This [third controller](rtl/sdslave/sdslave.v) is an SDIO *slave*
controller.  It has been designed to enable an FPGA to act like an SD card.

## SPI-based controller

[The SDSPI controller](rtl/sdspi/sdspi.v) exports an SD card controller
interface from internal to an FPGA to the rest of the FPGA core, while taking
care of the lower level details internal to the interface.  Unlike the [SDIO
controller](rtl/sdio/sdio.v) in this respository, this controller focuses on
the SPI interface of the SD Card.  While this is a slower interface, the SPI
interface is necessary to access the card when using a [XuLA2
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
the card.  This makes this [SDSPI controller](rtl/sdspi/sdspi.v) both more
versatile, in the face of potential changes to the card interface, but also
less turn-key.

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
controller](rtl/sdio/sdio.v), designed to exploit both the full SDIO protocol
and the 8b EMMC protocol--either with or without data strobes.  This controller
has been tested against both SDIO and eMMC chips, with the differences between
the two types of chips handled by software.

The interface to this controller is roughly the same as that of the [SDSPI
controller](rtl/sdspi/sdspi.v), although there are enough significant
differences to warrant a [separate user guide](doc/sdio.pdf).

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

Both Wishbone and AXI interfaces are supported.

*Status*: The SDIO controller has now been **silicon proven**.  It is currently
  working successfully in [its first FPGA
  project](https://github.com/ZipCPU/eth10g), where it is being used to control
  both an SD card as well as an eMMC chip.  It is also now working successfully
  in a [second project](https://github.com/ZipCPU/videozip/).  Many of the
  components of this IP have formal proofs, which they are known to pass.

  Notably missing among the formal component proofs is a proof of the [front
  end](rtl/sdio/sdfrontend.v).  The [front end](rtl/sdio/sdfrontend.v)'s
  verification depends upon integrated simulation testing.

  Both [Verilog](bench/verilog/mdl_sdio.v) and [C++](bench/cpp/sdiosim.cpp)
  models have been built which can be used to test this controller in
  simulation.  Unlike the [Verilog SDIO](bench/verilog/mdl_sdio.v) model, the
  [C++ SDIO](bench/cpp/sdiosim.cpp) supports a file-backed memory, allowing
  full software testing with filesystem(s) present.  All three simulation
  components have been now been tested successfully: the Verilog
  [SDIO](bench/verilog/mdl_sdio.v) and [eMMC](bench/verilog/mdl_emmc.v)
  models, as well as the [SDIO C++](bench/cpp/sdiosim.cpp) model.

Features include:

- **Multi-block**: Multiple block commands have been demonstrated in
  simulation when using both the Verilog [SDIO model](bench/verilog/mdl_sdio.v)
  and [eMMC model](bench/verilog/mdl_emmc.v)s.

  Multiblock commands form the basis for the DMA's operation.

- **`OPT_DMA`**: An optional DMA is now available, and passing tests in silicon.

  Both Wishbone and AXI versions of the DMA controller exist and pass all
  simulation based testing.

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
  - Transfer errors (failing CRCs, non-responsive cards, etc.) may cause the streams to lose synchronization.  To fix, the design may be given a soft reset (if necessary), and the external [MM2S](https://github.com/ZipCPU/wb2axip/blob/master/rtl/aximm2s.v)/[S2MM](https://github.com/ZipCPU/wb2axip/blob/master/rtl/axis2mm.v) DMAs may also need to be given similar resets.

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
  The control interface also has an (optional) AXI-Lite port which can be used
  to interact with the IP.  A flag exists to swap endianness, so that the
  design will be properly little endian when using this interface.  The AXI
  version of the design includes an integrated AXI DMA.

- **CRC Tokens**: CRC token's are 5b response values, indicating whether
  or not a page has transferred successfully.
  The [frontend](rtl/sdio/sdfrontend.v) can successfully recognize those CRC tokens
  following block write transfers.

  Failure to receive a CRC token when one is expected will (now) generate an
  error condition, as will receiving a negative CRC acknowledgment.

For more information, please consult the [SDIO user guide](doc/sdio.pdf).

### Roadmap and TODO items

Now that the RTL has been fully drafted and successfully tested in hardware,
it's moving from its development to application phase.  Further improvements
could still be made, as listed below:

- **C++ Model**: The design is missing a C++ model for testing the eMMC
  interface.

- **eMMC HS400 Data Strobe**: Although support exists for the data strobe, it
  has not been tested in hardware.  There are reasons to believe the data strobe
  front end implementation might not work in hardware, and specifically reasons
  to believe it won't close timing.  As a result, this feature is awaiting
  hardware which will support testing with the data strobe.

- **eMMC Boot mode**: No plan exists to support eMMC boot mode (at present).
  This decision may be revisited in the future, as I would love to support eMMC
  boot mode.

  Some (untested, preliminary) support exists for boot mode in the Verilog
  [eMMC model](bench/verilog/mdl_emmc.v).

- **eMMC Collision Detection**: [Collision detection remains an ongoing issue
  with eMMC support](https://github.com/ZipCPU/sdspi/issues/13).  This issue
  is limited to the `GO_IRQ_STATE` command, and specifically to the case where
  both controller and device attempt to leave the IRQ state at the same time.
  Without collision support, the message to leave the IRQ state may be
  corrupted on return.  This should be detectable via a bad CRC on the command
  line.

## SDSlave controller

The [Slave controller](rtl/sdslave/sdslave.v) is new.  This controller is
designed to let an external "host" interact with an FPGA via the SDIO protocol.
As an example, I am intending to test an RPi CM4 interface with an FPGA via this
controller and protocol.  The RPi should be able to read/write anything in the
FPGA Wishbone (or eventually AXI) address space via this approach.  At present,
via simulation only, all commands to/from memory only apply to blocks of
512Bytes, so there's some work left to be done.

Possible uses include:

- Controlling a memory, external to the FPGA, through the SDIO interface.

  Possibilities include flash (read-only), SDRAM, eMMC, SDIO, SATA, you name it.
  This might make it possible for an external CPU to read data at nearly the
  same time the FPGA is recording or processing it in real time.

- Video overlay control, whereby the host generates an overlay for the FPGA to
  place on an active video stream.

- Soft-core CPU software loading or shared memory handling

- Bridging an interface--such as 10Gb network, available to the FPGA, to the
  host.  (Given that the fastest this interface *might* go is only 50MB/s, it
  doesn't really make sense to try to keep up with a 10Gb or 125MB/s link over
  SDIO, but it is still an option.  Perhaps the HS400 mode might make more sense
  here?)

At any rate, development is ongoing.

Key features of this design include:

- **Hands-Off** - No run-time configuration is required from the local system.
  All commands come from the external/remote system.

- **SDR/DDR, 1b, 4b, or 8b** - Basic stuff here.  SDIO doesn't have or require
  8b interfaces, so the only use for the 8b interface would be for eMMC control.
  This design should be able to (eventually) handle the eMMC, but that's a
  future thing if ever.

- **Low Logic** - I wasn't expecting this one, but the SDSlave controller has
  less than half the logic requirement of the SDIO controller.  We'll see if
  this continues to be the case as more commands are implemented.

### Roadmap and TODO items

- **All Mandatory Commands** - At present, this design only supports *some* of
  the mandatory SD commands, not all of them.  As such, it's not likely to work
  with any generic SDIO host (yet).  Commands supported include: CMD0, CMD2,
  CMD3, CMD7, CMD8, CMD9, CMD10, CMD12, CMD13, CMD15, CMD17, CMD18, CMD19,
  CMD24, CMD25, CMD55, ACMD6, ACMD13, ACMD41, and ACMD51.  Not all of these
  commands have been tested or simulated.

  Commands still needing support include: CMD4, CMD11, CMD16, CMD23, CMD27,
  CMD32, CMD33, CMD38, CMD42, CMD56, ACMD22, ACMD23, and ACMD42.

  The SCR register is only partially implemented, and writes to it are not (yet)
  supported.

  While reading the CSD register is supported, the values read may (or may not)
  yet reflect the state of anything within.

- **Error Handling** - The current simulation is primarily a "happy path"
  simulation environment.  While errors may still take place, I've only ever
  canceled the simulation following any error rather than making sure the
  errors are properly handled, detected, identified, and recovered from.
  For example, any request to read from or write to an address that yields a
  bus error should properly end any transactions.  That has not been
  demonstrated yet, and I would (currently) doubt that the error handling would
  be clean.

- **Variable block sizes** - At some point in time, I may need or wish to
  support block sizes of other than 512Bytes.  The current architecture,
  however, will restrict all block sizes to powers of two and ... I'm so far
  okay with that.

- **Proper Front-end** - The design still needs a proper front end given to it.
  The one currently found in the [test bench](bench/verilog/tb_wb.v) really
  needs to be rewritten.

- **Verified Timing** - I'm pretty sure that I'm violating protocol timing in
  a couple of places.  These are just little things, like guaranteeing exactly
  two clocks between receive packet and the ACK/NAK token start bit, or not
  starting a transmit packet until the CMD reply completes, etc.

- **User Guide** - This needs to be written.  For now, the major SDIO commands
  are implemented, and so the SDIO spec can serve (somewhat) as a user guide
  for working with this IP.  It's just that ... the SDIO spec will tell you
  nothing about how to set it up, so this still needs to be done.

- **Hardware Testing** - My plan is to connect this to an RPi CM4 module, and
  see how things go.  I'll let you know.

- **eMMC FSM** - The FSM module may be rewritten to support an eMMC slave
  controller.  The other slave modules have been built to work in an eMMC
  environment (8b, w/ or w/o DS), so it should only require a rewrite of the FSM
  to support eMMC devices instead of SD devices.

Bottom line?  This is still a work in progress, and it will probably remain
so until hardware testing completes.

# Logic usage

Current logic usage is being tracked for iCE40 and Xilinx 6-LUT devices
in the [usage.txt](rtl/usage.txt) file in the [RTL/](rtl/) directory.

# Commercial Applications

Should you find the GPLv3 license insufficient for your needs, other licenses
may be purchased from Gisselquist Technology, LLC.
