/* 4k page size, 512k total flash*/
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH            : ORIGIN = 0x00000000, LENGTH = 128K /* bootloader */
  APPLICATION      : ORIGIN = 0x00020000, LENGTH = 384K /* application */
  RAM              : ORIGIN = 0x20000000, LENGTH = 128K
}

__application_start = ORIGIN(APPLICATION);
__application_end = ORIGIN(APPLICATION) + LENGTH(APPLICATION);