# SD-Card controller

This repository contains two IPs for handling SD cards.  The first and older
IP handles SD cards via their (optional) SPI interface.  The second and newer
IP works using the SDIO interface.  It should be able to handle both SDIO
and eMMC cards as a result.

## SPI-based controller

[This Verilog core](rtl/sdspi.v) exports an SD card controller interface from
internal to an FPGA to the rest of the FPGA core, while taking care of the
lower level details internal to the interface.  Unlike the [SDIO
controller](rtl/sdio.v) inthis respository, this controller focuses on the SPI
interface of the SD Card.  While this is a slower interface, the SPI interface
is necessary to access the card when using a [XuLA2 board](http://www.xess.com/shop/product/xula2-lx25/), or
in general any time the full 9--bit, bi--directional interface to the SD card
has not been implemented.
Further, for those who are die--hard Verilog authors, this core is written in
Verilog as opposed to the [XESS provided demonstration SD Card controller
found on GitHub](https://github.com/xesscorp/VHDL\_Lib/SDCard.vhd), which was
written
in VHDL.  For those who are not such die--hard Verilog authors, this controller
provides a lower level interface to the card than these other controllers. 
Whereas the XESS controller will automatically start up the card and interact
with it, this controller requires external software to be used when interacting
with the card.  This makes this SDSPI controller both more versatile, in the
face of potential changes to the card interface, but also less turn-key.

While this core was written for the purpose of being used with the [ZipCPU](https://github.com/ZipCPU/zipcpu),
as enhanced by the Wishbone DMA controller used by the ZipCPU, nothing in this
core prevents it from being used with any other architecture that supports
the 32-bit Wishbone interface of this core.

This core has been written as a wishbone slave, not a master.  Using the core
together with a separate master, such as a CPU or a DMA controller, only makes
sense.  This design choice, however, also restricts the core from being able to
use the multiple block write or multiple block read commands, restricting us to 
single block read and write commands alone.

For more information, please consult the specification document.

## SDIO

This particular branch contains the beginnings of an SDIO protocol
implementation.  The implementation itself is roughly complete, although
it (now) needs a lot of testing and verification.

# Commercial Applications

Should you find the GPLv3 license insufficient for your needs, other licenses
can be purchased from Gisselquist Technology, LLC.
