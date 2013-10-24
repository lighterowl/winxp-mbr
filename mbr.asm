bits 16
cpu 186

; a structure representing a partition table entry. useful when specifying
; offsets for read instructions.
struc ptable_entry
  .boot_ind:    resb 1
  .start_head:  resb 1
  .start_sect:  resb 1
  .start_cyl:   resb 1
  .system_id:   resb 1
  .end_head:    resb 1
  .end_sect:    resb 1
  .end_cyl:     resb 1
  .rel_sect:    resd 1 ; starting sector of the partition
  .total_sect:  resd 1
endstruc

; only the last 6 bits of start_sect are actually the starting sector.
PTABLE_START_SECT_MASK equ 0x3f
; while the other two are the upper two bits of the starting cylinder.
PTABLE_START_CYL_MASK equ 0xc0

IMAGE_LOAD_ADDRESS     equ 0x7c00
MAIN_BOOT_CODE_ADDRESS equ 0x0600
SECTOR_SIZE            equ 512
INIT_SECTION_BYTES     equ (main_boot_code - init)
FAT32_CHS_SYSTEM_ID    equ 0xb
FAT32_LBA_SYSTEM_ID    equ 0xc

; although the image is loaded at 0x7c00, it's more convenient to specify the
; origin as 0x600, since that's where jumps and reading take place.
org MAIN_BOOT_CODE_ADDRESS

; this is the entry point of the code. after BIOS detects the boot device, it
; loads its first 512 bytes ("sector") into the physical address 0x7c00 and
; executes a jump to that address.
init:
  ; this sets up a stack at 0x7c00. the memory located at 0x500-0x7c00 is
  ; guaranteed to be free for use by programs, so the stack can safely grow in
  ; this area.
  xor ax,ax
  mov ss,ax
  mov sp,IMAGE_LOAD_ADDRESS
  
  ; enable interrupts. this is weird, since code executing this early usually
  ; keeps them disabled.
  sti
  
  ; initialize segment registers to zero.
  push  ax
  pop   es
  push  ax
  pop   ds
  
  ; DF <- 0. with this setting, instructions with the REP prefix increment the
  ; counter.
  cld
  
  ; set the source and destination registers for copying the main boot code.
  ; when this code is executed, the whole "init" code lies at 0x7c00. the "main"
  ; boot code is relocated to 0x61b. the bytes already executed are omitted, and
  ; what's copied is the rest of the sector, after the "init" bytes.
  mov si,(IMAGE_LOAD_ADDRESS + INIT_SECTION_BYTES)
  mov di,(MAIN_BOOT_CODE_ADDRESS + INIT_SECTION_BYTES)
  
  ; push the new code location to the stack.
  push ax
  push di
  
  ; set the number of bytes to be copied, omitting the code already executed and
  ; start copying.
  mov cx,(SECTOR_SIZE - INIT_SECTION_BYTES)
  rep movsb
  
  ; "return" to the new location of the code. since this is a far return in real
  ; mode, it pops two words from the stack : first one, containing the offset
  ; (0x61b), and the second one with the segment (0). (CS:IP) is thus set to
  ; (0:0x61b) and the execution continues there.
  retf

main_boot_code:
  mov bp, partition_table ; for scanning the partition table
  mov cl, 4 ; there are four partition table entries

.check_boot_ind:
  ; look for a partition with the boot indicator.
  cmp [bp+ptable_entry.boot_ind], ch
  jl  found_active_partition ; less than zero (ch is zero at this point)
  jnz invalid_part_table ; nonzero and non-negative - invalid.
  
  add  bp,16 ; move to the next entry
  loop .check_boot_ind ; scan all the four entries (until cx is zero).
  
  int 0x18 ; if no active partition is found.
  ; INT 18h is the "diskless boot hook", which should start the BASIC
  ; interpreter that's stored in the ROM. almost no today's BIOSes have it, so
  ; calling this interrupt usually results in a "No boot device available"
  ; message.

found_active_partition:
  mov si,bp ; the pointer to the first active partition is saved in bp

; check if there's another active partition in the table. if there is more than
; one active partition, the table is considered invalid.
.scan_extra_active_partition:
  add si,16
  dec cx ; next entry
  jz  part_table_ok ; finished scanning the table - can proceed with booting.
  cmp [si],ch ; check the boot indicator
  je .scan_extra_active_partition ; if zero (not active), keep scanning until we
  ; reach the end. if not zero, fall through to the code below

invalid_part_table:
; a jump here is executed if the partition table is detected to be invalid, for
; example if two active partitions are found.
  
  ; load the starting address of the message into si. the low byte of the
  ; address is stored in the image itself.
  mov al,[invalid_partition_table_msg_low_addr]

print_msg:
  mov ah,0x7
  mov si,ax

.load_char_and_print:
  lodsb ; al <- [ds:si]

.inf_loop:
  cmp al, 0x0 ; the strings are 0-terminated.
  jz .inf_loop ; if we hit the end, go into an infinite loop.
  
  ; else, print the character to the screen.
  mov bx, 0x7 ; bl <- foreground color
  mov ah, 0xe
  int 0x10 ; Int 10/AH=0Eh : Teletype Output. al contains the char to print.
  jmp .load_char_and_print

part_table_ok:
  mov  [bp+0x10], cl ; number of tries for part_table_ok
  call read_first_sector
  jnc .check_sector_contents ; first sector read successfully.

.read_retry_loop:
  inc byte [bp+0x10] ; number of tries for part_table_ok
  
  ; check the system ID of the partition that we found to be active.
  ; interestingly, this is done only if the read fails. also, if the partition
  ; is an NTFS partition and the first read failed, booting will fail, since the
  ; loader only checks for FAT32 IDs.
  cmp byte [bp+ptable_entry.system_id],FAT32_CHS_SYSTEM_ID
  je  .try_next_read
  cmp byte [bp+ptable_entry.system_id],FAT32_LBA_SYSTEM_ID
  je  .try_next_read
  
  ; print "Error loading operating system" and hang.
  mov al,[error_loading_os_msg_low_addr]
  jnz print_msg
  
.try_next_read:
  ; the following instructions modify the starting sector/cylinder locations of
  ; the active partition's data, as stored in memory. it also adds 6 sectors to
  ; its starting location. this is probably to support loading the OS from
  ; different sectors than the 1st one of the partition, though I haven't ever
  ; seen this used.
  add byte [bp+ptable_entry.start_sect],0x6
  add word [bp+ptable_entry.rel_sect],byte +0x6
  adc word [bp+ptable_entry.rel_sect+2],byte +0x0
  
  ; try re-reading the sector with the new relative location set.
  call read_first_sector
  jnc .check_sector_contents ; read_first_sector sets CF on error.
  
  ; if the call failed, just print "Error loading operating system"
  mov al,[error_loading_os_msg_low_addr]
  jmp print_msg

.check_sector_contents:
  ; the last two bytes of the partition's first sector should be the same as the
  ; MBR signature (0x55, 0xaa).
  cmp word [0x7dfe],0xaa55
  je .jump_to_first_part_sector
  
  ; check the retry counter, and - if it's not zero yet - try reading the 1st
  ; sector at a different offset (check comments around .try_next_read).
  cmp byte [bp+0x10],0x0
  je .read_retry_loop
  
  ; if the number of tries is zero, print "Missing operating system".
  mov al,[missing_os_msg_low_addr]
  jmp print_msg

.jump_to_first_part_sector:
  ; this executes a jump to the code that has been loaded from the boot
  ; partition's first sector. it uses the same technique as the MBR init code.
  mov di,sp
  push ds ; 0
  push di ; 0x7c00
  mov si,bp
  
  ; the "jump" is executed to (0000:7c00) - the same location as the initial MBR
  ; code, but now it's replaced with the contents of the partition's 1st sector.
  retf

read_first_sector:
  mov di, 0x5 ; irrelevant for the interrupt : used later as the number of
  ; retries while reading sectors from the disk.
  mov dl, [bp+ptable_entry.boot_ind] ; boot indicator (80h) as drive number. bit
  ; 7 is set, so the call will return information about hard disks.
  mov ah, 0x8
  int 0x13 ; Int 13/AH=08h : Get Drive Parameters
  jc .try_read_or_reset ; CF set on error
  
  mov al, cl
  and al, 0x3f ; bits 5-0 of cl contain the number of sectors per track.
  cbw ; sign extend
  mov bl, dh ; dh is the maximum head number.
  mov bh, ah ; ah is zero on return from this call.
  inc bx ; the value returned is always -1.
  mul bx ; multiply sectors per track by maximum head number.
  ; (dx:ax) now contains the number of sectors in a head.
  
  mov  dx, cx ; cx still contains the value returned by int 13h. the cylinder
  ; count's low bits are stored in dh, and its two high bits as high bits in dl.
  xchg dl, dh ; swap the bytes of dx (in order to get the cylinder count)
  mov  cl, 6
  shr  dh, cl ; shift out the last 6 bits (sectors per track) from dh.
  inc  dx ; dx now contains the whole cylinder count. like sect/track, the
  ; value returned from the interrupt is -1.
  mul  dx ; multiply the whole cylinder count by sectors per head's low word,
  ; thus getting the disk's last sector into (dx:ax).
  
  ; the starting sector of the partition is stored as a dword in the partition
  ; table. compare the whole calculated value of disk's sectors against the
  ; partition's starting sector.
  cmp [bp+ptable_entry.rel_sect+2], dx
  ja  .read_first_sector_by_ah_42
  jb  .try_read_or_reset
  cmp [bp+ptable_entry.rel_sect], ax
  jnb .read_first_sector_by_ah_42

.try_read_or_reset:
  mov ax, 0x201 ; al <- number of sectors (1), ah <- 2 (call number)
  mov bx, IMAGE_LOAD_ADDRESS ; (es:bx) <- data buffer.
  mov cx, [bp+ptable_entry.start_sect] ; (ch,cl) <- (cyls, sects)
  mov dx, [bp+ptable_entry.boot_ind] ; 80h as the drive number again.
  int 0x13 ; Int 13/AH=02h : Read Sectors Into Memory
  jnc global_ret ; if read correctly, just return.

  dec di ; di is the number of retries
  jz  global_ret ; if reached zero here, return with CF set.
  
  xor ah,ah
  mov dl, [bp+ptable_entry.boot_ind] ; 80h again.
  int 0x13 ; Int 13/AH=00h : Reset Disk System
  jmp .try_read_or_reset

.read_first_sector_by_ah_42:
  mov dl,[bp+ptable_entry.boot_ind] ; drive number - 80h again.
  pushaw
  mov bx,0x55aa
  mov ah,0x41
  int 0x13 ; Int 13/AH=41h/BX=55AAh : Int 13 Extensions Installation Check
  jc .return_error ; extensions not supported
  
  cmp bx,0xaa55
  jne .return_error ; extensions not installed
  
  test cl,0x1 ; "API support bitmap" - bit 0 means the support for removable
  ; drive extensions
  jz .return_error ; not supported
  
  popaw
  
.read_by_int_42h:
  pushaw
  push 0 ; as below, 4th word.
  push 0 ; as below, 3rd word.
  push word [bp+ptable_entry.rel_sect+2] ; as below, 2nd word
  push word [bp+ptable_entry.rel_sect] ; absolute sector number 1st word
  push 0 ; destination buffer segment
  push 0x7c00 ; destination buffer offset
  push 1 ; number of blocks to transfer (word 1)
  push 0x10 ; size of packet (byte 10h) & reserved (0)
  mov ah,0x42
  mov si,sp ; (ds:si) is the address of the parameter block (AKA "disk address
  ; packet")
  int 0x13 ; Int 13/AH=42h - Extended Read
  
  popaw
  popaw
  jnc global_ret ; int 13h sets CF on error.
  
  dec di ; di still holds the number of tries (initially 5).
  jz global_ret ; number of tries exceeded.
  
  xor ah,ah
  mov dl,[bp+0x0] ; disk number - 80h again.
  int 0x13 ; Int 13/AH=00h : Reset Disk System
  
  jmp .read_by_int_42h

; a jump here is executed if a critical error has occurred and CF=1 must be
; returned.
.return_error:
  popaw
  stc

global_ret:
  ret

INVALID_PARTITION_TABLE_MSG_LOW_ADDR_C equ ((($-$$) + IMAGE_LOAD_ADDRESS) & 0xff)
invalid_partition_table_msg db "Invalid partition table",0

ERROR_LOADING_OS_MSG_LOW_ADDR_C equ ((($-$$) + IMAGE_LOAD_ADDRESS) & 0xff)
error_loading_os_msg db "Error loading operating system",0

MISSING_OS_MSG_LOW_ADDR_C equ ((($-$$) + IMAGE_LOAD_ADDRESS) & 0xff)
missing_os_msg db "Missing operating system",0

; some zeros for internationalized strings. the low addresses of the strings
; start at offset 0x1b5 (437) in my image, and the area between the last string
; and the first address is filled with zeros.
times (437-($-$$)) db 0

; low bytes of the strings' addresses. this allows their locations to be
; dynamic, which is crucial when preparing non-English strings, since they might
; be longer or shorter, thus changing the original addresses.
invalid_partition_table_msg_low_addr db INVALID_PARTITION_TABLE_MSG_LOW_ADDR_C
error_loading_os_msg_low_addr db ERROR_LOADING_OS_MSG_LOW_ADDR_C
missing_os_msg_low_addr db MISSING_OS_MSG_LOW_ADDR_C

; there's some additional data in here. I have absolutely no idea what it is for
; since it's never referenced from the code. I put zeros in this place. remember
; that the partition table starts at offset 0x1be = 446.
times (446-($-$$)) db 0

; the partition table. we're going to put empty entires in there.
partition_table:
%rep 4
  istruc ptable_entry
    at ptable_entry.boot_ind,    db 0
    at ptable_entry.start_head,  db 0
    at ptable_entry.start_sect,  db 0
    at ptable_entry.start_cyl,   db 0
    at ptable_entry.system_id,   db 0
    at ptable_entry.end_head,    db 0
    at ptable_entry.end_sect,    db 0
    at ptable_entry.end_cyl,     db 0
    at ptable_entry.rel_sect,    dd 0
    at ptable_entry.total_sect,  dd 0
  iend
%endrep

; MBR signature
db 0x55, 0xaa
