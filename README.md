winxp-mbr
=========

This repository provides a disassembly of the Windows XP Master Boot Record.
The whole code has been disassembled by ndisasm, and then recreated in nasm
syntax. Exhaustive comments have been added in order to help with anybody who
might want to look a bit more into the boot process of this OS.

Only the MBR (e.g. the first sector of the boot disk) has been analyzed so far.
Analysis and disassembly of the active partition's first sector (which contains
the next stage of the bootloader, and is loaded by the MBR code) will appear
here sometime in the future.

