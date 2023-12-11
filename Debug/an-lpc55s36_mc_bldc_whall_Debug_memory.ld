/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2023 NXP
 * Generated linker script file for LPC55S36
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.8.0 [Build 1165] [2023-07-26] on Dec 11, 2023, 2:20:25 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x3d800 /* 246K bytes (alias Flash) */  
  QSPI_FLASH (rx) : ORIGIN = 0x80000000, LENGTH = 0x4000000 /* 64M bytes (alias Flash2) */  
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x1c000 /* 112K bytes (alias RAM) */  
  SRAMX (rwx) : ORIGIN = 0x4000000, LENGTH = 0x4000 /* 16K bytes (alias RAM2) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x3d800 ; /* 246K bytes */  
  __top_Flash = 0x0 + 0x3d800 ; /* 246K bytes */  
  __base_QSPI_FLASH = 0x80000000  ; /* QSPI_FLASH */  
  __base_Flash2 = 0x80000000 ; /* Flash2 */  
  __top_QSPI_FLASH = 0x80000000 + 0x4000000 ; /* 64M bytes */  
  __top_Flash2 = 0x80000000 + 0x4000000 ; /* 64M bytes */  
  __base_SRAM = 0x20000000  ; /* SRAM */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM = 0x20000000 + 0x1c000 ; /* 112K bytes */  
  __top_RAM = 0x20000000 + 0x1c000 ; /* 112K bytes */  
  __base_SRAMX = 0x4000000  ; /* SRAMX */  
  __base_RAM2 = 0x4000000 ; /* RAM2 */  
  __top_SRAMX = 0x4000000 + 0x4000 ; /* 16K bytes */  
  __top_RAM2 = 0x4000000 + 0x4000 ; /* 16K bytes */  
