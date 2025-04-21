# RTL directory

This directory contains the Verilog components for both the [SDSPI](sdspi.v)
controller as well as the [SDIO](sdio.v) controller.

## SDSPI

- [SDSPI](sdspi.v) is the top level of the SDSPI controller.  It contains
  all of the Wishbone processing logic, as well as references to all of the
  components within.

  - [LLSDSPI](llsdspi.v) is the low level SPI driver.  It turns "higher-level"
    commands into actual transactions taking place across the wires, and
    returns bytes read from the MISO line for processing by the
    [SDRXDATA](sdrxdata.v) module.

  - [SDCMD](spicmd.v) controls both the issuing of commands over the SPI
    interface, as well as any return responses.

  - [SPIRXDATA](spirxdata.v) captures data returned from the MISO interface,
    and formats this data for writing to a local FIFO maintained at the
    [SDSPI](sdspi.v) level.  Note that [SPIRXDATA](spirxdata.v) is used for
    *data* from the device, not command replies.

  - [SPITXDATA](spitxdata.v) transforms data from the local FIFO to data
    blocks to be sent via the [low-level interface](llsdspi.v).

This particular design is somewhat optimized for low area.

## SDIO

- [SDIO-TOP](sdio_top.v) is a top level, existing as a wrapper for both the
  [front end PHY](sdfrontend.v) as well as the [host controller](sdio.v).
  My anticipation is that I will typically break these components up in my
  designs, so that the [host controller](sdio.v) will exist within the main
  body of any design, whereas the [front end PHY](sdfrontend.v) will only
  exist within the top level--since it contains _vendor dependent_ IO macro
  components which may not be simulable within an open source environment.

  - [SDFRONTEND](sdfrontend.v) is the front end to this IP.  It contains three
    separate implementations: 1) one for direct IO support, going up to 50MHz
    SDR or 25MHz DDR, 2) one that can go a touch faster if DDR IO elements
    are available, and 3) a third which, using 8:1 and 1:8 IO SERDES components,
    will support IOs all the way up to 200MHz DDR with a return data strobe.
    Actual IO buffers for the various SDIO signal wires are also instantiated
    at this level.

    - [XSDDDR](xsdddr.v), contains the low-level DDR IO controllers.  It's
      been designed specifically for Xilinx components, but also supports a
      non-Xilinx simulation model.

    - [XSDSERDES8X](xsdserdes8x.v), contains the setup necessary to instantiate
      both low-level 8:1 and 1:8 SERDES IO controllers.  This has also been
      designed for Xilinx components, and no non-Xilinx simulation model
      (currently) exists for this component.

  - [SDIO](sdio.v) is the top level of the host controller.  It contains all
    of the logic of this IP, save the PHY front end itself.

    - [SDWB](sdwb.v) is the Wishbone command and control module.

    - [SDAXIL](sdaxil.v) is an alternative command and control module, for use when using the AXI-Lite slave interface.

      - [SDSKID](sdskid.v) is a basic skidbuffer, required by AXI-Lite interface to maintain solid throughput.

    - [SDCKGEN](sdckgen.v) is the clock divider driving IO actions throughout.

    - [SDCMD](sdcmd.v) controls interaction via the command pin.

    - [SDRXFRAME](sdrxframe.v) transforms data returned by the IO controller 
      into commands to write into the data FIFOs within [SDWB](sdwb.v).  Data
      may be returned by either the asynchronous interface (if the data strobe
      is in use) at up to 32-bits per clock cycle, or at slower rates of up
      to 16-bits per clock cycle.

    - [SDTXFRAME](sdtxframe.v) transforms a 32-bit AXI stream into an outgoing
      transmit sequence.  Three data widths are supported: 1b, 4b, and 8b.
      (SDIO tops out at 4b, whereas eMMC can use an 8b interface.)  Three
      clock types are supported: 1) one data word per clock, for supporting
      100MHz/SDR or 50MHz/DDR clock frequencies and below, 2) two data words
      per clock, for supporting 200MHz/SDR or 100MHz/DDR, and 3) four data
      words per clock, for 200MHz DDR.

    - [SDDMA](sddma.v) is top level of the DMA memory handling module.  It depends heavily on the command/control interface to activate.  This module primarily converts memory to an AXI stream to be fed to the command/control module, and again AXI stream to memory.  An option exists for skipping memory and writing directly from an AXI stream, or likewise reading directly from one.

      - [SDDMA_MM2S](sddma_mm2s.v) reads data from memory, to feed the gearboxes and then write to the SD card.  This forms the beginning of the S2SD (write) path.

      - [SDDMA_RXGEARS](sddma_rxgears.v) massages incoming data to the size of a full bus word, in preparation for (bus-word sized) the FIFO.

      - [SDFIFO](sdfifo.v), a basic synchronous data FIFO.

      - [SDDMA_TXGEARS](sddma_txgears.v) takes data from the FIFO and massages it to either the size of a full bus word for writing to memory, or to 32b for writing to the SD controller.

      - [SDDMA_S2MM](sddma_s2mm.v) writes data from SD card, having gone through the gearboxes, finally to memory.  This forms the conclusion of the SD2S (read) path.


The [SDIO controller](sdio_top.v) has been optimized for speed, rather than
either area or lowpower.  The [DMA component](sddma.v), in particular, may
double the size of the core while achieving even greater throughput.  As with
many of my IP's, there is an `OPT_LOWPOWER` parameter which can be used to
adjust this optimization for lowpower at the (potential) expense of area.

## Logic Usage

A [perl script](usage.pl) is also available to measure the logic usage of these
two IP components via Yosys.  Measurement results are kept [here](usage.txt).
As of this writing, the [SDSPI](sdspi.v) controller requires only 548 Xilinx
6-LUTs, whereas the [SDIO](sdio.v) controller requires 1438 Xilinx 6-LUTs
without the DMA, and 2708 6-LUTs with the DMA.

