# BOOT Design document

This is a preliminary design document.  It primarily consists of notes from
the designer, for the purpose of guiding and directing the build of the eMMC
boot capability.  Once these details are added into the user guide, this
document will become obsolete, with the user guide taking precedence.

## Parameters

- `OPT_BOOTEN=1` - Enables BOOT functionality of all types.  Setting
  `OPT_BOOTEN=0` will remove any boot functionality from the design.

- `OPT_AUTOBOOT=1` - Automatically starts boot on release from reset.  User
   release from hardware reset requires user intervention
   to automatically boot.

- `BOOT_BLOCKS` -(Must be `>0` if `OPT_AUTOBOOT=1`) Number of 512 Byte blocks
   to transfer on any autoboot command

- `BOOT_ADDR` -Address to transfer boot blocks to when autobooting

- `BOOT_TOKEN=1` - Set to expect a boot token on autoboot startup,
   acknowledging start of boot sequence from device.

- `BOOT_CLK` - Sets the clock setting for autoboot.

- `BOOT_MODE[1:0]` - Sets # of bits for autoboot mode

- `BOOT_MODE[2]` sets DDR mode during autoboot.

- `BOOT_MODE[3]` sets DS (HS400) mode during autoboot.  (eMMC standard says
  HS400 not supported during BOOT operation.)

- Note: BOOT *always* uses the DMA
- Note: AutoBOOT always enables the DMA interrupt.
  Interrupts are not maskable in the controller.
- Note: BOOT is always in PUSH/PULL mode for both CMD and DATA.

## Entering BOOT

There are three ways to enter boot mode.

1. If `OPT_AUTOBOOT=1`, the design will enter boot mode automatically following
   a reset.
2. The controller will also enter boot mode following a write of
   `32'ha300|CRC_TOKEN` to the command register.
3. The controller will enter the alternate boot mode following a write
   of `32'ha300|CRC_TOKEN` to the command register, provided the argument
   register is set to `32'hffff_fffa`.


## Commands:

Writing the following to the command register will ...

- `CMD=32'h8040` (i.e. CMD0), with `ARG=32'hf0f0_f0f0`

  Starts a hard reset without using the reset command.

- `CMD_BOOT_EN = 32'ha300 (| CRC_TOKEN)`

  Decoded: Clear error, enable DMA, expect an R1B reply to a non-command.

  Starts a boot sequence, holding CMD wire low.  If `CRC_TOKEN` bit is set,
  then we'll also expect a token from the device before the boot starts.

- `CMD_BOOT_EN = 32'ha040 (| CRC_TOKEN)`, with `ARG = 32'hffff_fffa`.  If
  `CRC_TOKEN` bit is set, then we'll also expect a token from the device
   before the boot starts.

  Decoded: Clear error, enable DMA, send a CMD0, expect no responses.  Key
  detail is in the command ARG.  Any CMD0 w/ this ARG will cause a BOOT.

  Sends a CMD0 to start a boot sequence in alternate boot mode.

## Registers:

- CMD, as discussed above
- DMA address
- DMA blocks
- PHY

## Signals:

- Frontend: `i_expect_token`.  If set, expect a token once `i_data_en=0`.
- SD Bus processor:
  - `o_boot_tok`: Sent to enable `i_expect_token`.  Transmit may also set this.
  - `i_boot_ack`, `i_boot_nak` (Only valid following `i_expect_token`, and only
    valid until the first token returned.  May be raised on any token, only
    relevant following boot.)
  - `w_boot_active`
  - `w_boot_end`

