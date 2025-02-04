/* 4k page size, 512k total flash*/
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  BOOTLOADER            : ORIGIN = 0x00000000, LENGTH = 128K /* bootloader */
  FLASH                 : ORIGIN = 0x00020000, LENGTH = 384K /* application */
  RAM                   : ORIGIN = 0x20000000, LENGTH = 128K
}
