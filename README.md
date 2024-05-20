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

Most board glue logic is implemented with an Altera EPM7128S CPLD. This part is obsolete but modern replacement exists with ~~Actel~~ ~~Microsemi~~ Microchip ATF1508AS (never tested though). The CPLD implements:
* DRAM address multiplexing
* DRAM refresh
* basic memory management: base address translation and inbound check with power of two alignment constraints
* 8-bits expansion bus access, for 8-bits wide or 16-bits wide memory cycle from the 68000
* first stage boot procedure, by copying during reset the 512 first bytes from the flash memory (see MFP chip description below) at address 000000h in DRAM. 

Expansion bus also uses a few discrete logic (74245 buffers and 74138 address decoder).

An additional 74148 encodes interrupt sources to CPU IPL0/1/2.

### memory management

The CPLD provides very basic memory management capabilities, for user mode accesses only (FC2 set to 0). It relies on two value set with four registers. These registers are write only, 16-bits, but only the upper bit 15:8 are used.

| register    | address | description             |
|:-----------:|:-------:|:------------------------|
| MMU_BASE_LO | FE0000h | base address bits 19:12 |
| MMU_BASE_HI | FE0002h | base address bits 22:20 |
| MMU_MASK_LO | FE0004h | address mask bits 19:12 |
| MMU_MASK_HI | FE0006h | address mask bits 21:20 |

Any user address is and'ed with MMU_MASK then or'ed with MMU_BASE. It results in a memory area allocated to a user program with a power of two size, with a base address aligned to its own size. Any user access that targets an address above the allocated size ends with a bus error.

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

| register    | address | description             |
|:-----------:|:-------:|:------------------------|
| MFP_DATA    | F00000h | MFP register data |
| MFP_ADDRESS | F00001h | MFP register index |

### Register set

| register    | index   | description             |
|:-----------:|:-------:|:------------------------|
| FLASH_DATA  | 0       | boot flash data - address auto increment |
| TICKS       | 3       | 4-us ticks counter, 0-249 |
| UART_STS    | 4       | UART status<br>bit 7: TX ready<br>bit 1: RX FIFO overflow<br>bit 0: RX FIFO underflow |
| UART_DATA   | 5       | UART data in/out |
| POST        | 7       | POST code (debug) |
| PS2P0_STS   | 8       | PS/2 port 0 status<br>bit 0: data available |
| PS2P0_DATA  | 9       | PS/2 port 0 data |
| PS2P1_STS   | 10      | PS/2 port 1 status<br>bit 0: data available |
| PS2P1_DATA  | 11      | PS/2 port 1 data |
| SYSCFG      | 12      | system tick configuration<br>bit 0: enable timer interrupt<br>bit 5: set by bootstap loader to prevent flash to RAM boot transfer<br>bit 6: enable self immediate interrupt<br>bit 7: soft reset |
| SYSSTS      | 13      |  system tick status<br>bit 0: timer interrupt pending<br>bit 6: self immediate interrupt pending |
| IRQCFG      | 14      | interrupt controller configuration<br>bit 0: enable RTC interrupt<br>bit 1: enable PS/2 keyboard interrupt<br>bit 2: enable PS/2 mouse interrupt<br>bit 3: enable UART RX interrupt |
| IRQSTS      | 15      | interrupt controller status<br>bit 0: RTC interrupt pending<br>bit 1: PS/2 keyboard interrupt pending<br>bit 2: PS/2 mouse interrupt pending<br>bit 3: UART RX interrupt pending |

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
