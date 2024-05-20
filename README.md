# Overview

This is a single board computer based on Motorola 68000. It featues:
* 512kiB to 8MiB DRAM (SIMM 30 pins x2)
* basic memory protection / paging
* 8bits expansion bus with 4 slots
* PS/2 keyboard ans mouse support
* serial link (TTL)
* boot with serial bootstrap or user flash

Some pictures [here](/doc).

The block diagram below depicts the board architecture:
![board architecture](/doc/arch.png)


## Board glue logic

Most board glue logic is implemented with an Altera EPM7128S CPLD. This part is obsolete but modern replacement exists with Actel Microsemi Microchip ATF1508AS (never tested though). The CPLD implements:
* DRAM address multiplexing
* DRAM refresh
* basic memory management: base address translation and inbound check with power of two alignment constraints
* 8-bits expansion bus access, for 8-bits wide or 16-bits wide memory cycle from the 68000
* first stage boot procedure, by copying during reset the 512 first bytes from the flash memory (see MFP chip description below) at address 000000h in DRAM. 

Expansion bus also uses a few discrete logic (74245 buffers and 74138 address decoder).

An additional 74148 encodes interrupt sources to CPU IPL0/1/2.

### interrupt management

The autovectored interrupt scheme is used (VPA# assertion). To save pins on the CPLD the FC0 and FC1 are not available to identify an interrupt acknowledge cycle, so any read cycle at FFFFFFh with FC2 set to 1 is considered as such.

### 8-bits expansion bus

This bus provides access to 5 peripherals:
* 4 available through dedicated connectors
* 1 embeded to the board: the MFP

It provides for each peripheral up to 128kiB aperture into CPU memory space.

The table below lists the signals on this bus as seen for each slot.

| signal  | type              | shared  | description          |
|:-------:|:-----------------:|:-------:|:--------------------:|
| RST#    | input             | yes     | system reset         |
| CE#     | input             | no      | access enable        |
| ACK#    | open drain output | yes     | access acknowledge   |
| WE#     | input             | yes     | write enable         |
| ALE#    | input             | yes     | address latch enable |
| IRQ#    | open drain output | option  | interrupt request    |
| A0      | input             | yes     | address bit 0        |
| AD[7:0] | bidir             | yes     | address / data       |

#### single access cycles

This is how translates a byte access from the 68000 (UDS# or LDS# alone)

![Write cycle - single word](/doc/bus_write_single.png)
![Read cycle - single word](/doc/bus_read_single.png)

#### multiple access cycles

This is how translates a 16-bits access from the 68000 (UDS# and LDS#)

![Write cycle - multiple words](/doc/bus_write_multiple.png)

## MFP (multifunction peripheral)

This is a software defined chip base on a PIC18F27K42 microcontroller. It provides:
* embedded flash with 2 software images for the 68000, selection by detecting pressed key at reset. 
* IOs/timer with dedicated low-priority interrupt: two PS/2 port, one serial link, 1 kHz RTC timer
* 250 Hz system timer with dedicated highest-priority interrupt
* hard/soft reset
* debug register (POST or so)

The MFP has only two memory locations accessible through the expansion bus: one to set the actual register to access, the other to read or write data from/to the register.

# Memory mapping

| memory range      | access | size   | memory area
|:-----------------:|:------:|:------:|:-----
| 000000h - 7FFFFFh | U/S    | 8MiB   | DRAM. memory address translation/mask/check for user mode only |
| 800000h - 8FFFFFh | S	     | 7MiB   | DO NOT USE. mirrors F00000h - FFFFFFh                          |
| F00000h - F1FFFFh | S      | 128kiB | MFP registers. address bits 16:1 are ignored                   |
| F20000h - F3FFFFh | S      | 128kiB | expansion bus connector J9                                     |
| F40000h - F5FFFFh | S      | 128kiB | expansion bus connector J11                                    |
| F60000h - F7FFFFh | S      | 128kiB | DO NOT USE. reserved for expansion bus hardware implementation |
| F80000h - F9FFFFh | U*/S   | 128kiB | expansion bus connector J10. user access to be removed         |
| FA0000h - FBFFFFh | S      | 128kiB | DO NOT USE. unmapped, might be used for another slop           |
| FC0000h - FDFFFFh | S      | 128kiB | expansion bus connector J8                                     |
| FE0000h - FFFFFFh | S      | 128kiB | restricted to PMMU register configuration                      |
