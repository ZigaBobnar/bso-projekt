
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
 
 /* spaniakos <spaniakos@gmail.com>
  Added __ARDUINO_X86__ support
*/

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

  /*** USER DEFINES:  ***/  
  #define FreeRTOS
  #define RF24_SPI_TRANSACTIONS
  //#define FAILURE_HANDLING
  //#define SERIAL_DEBUG
  //#define MINIMAL
  //#define SPI_UART  // Requires library from https://github.com/TMRh20/Sketches/tree/master/SPI_UART
  //#define SOFTSPI   // Requires library from https://github.com/greiman/DigitalIO
  
  /**********************/
  #define rf24_max(a,b) (a>b?a:b)
  #define rf24_min(a,b) (a<b?a:b)

  #if defined SPI_HAS_TRANSACTION && !defined SPI_UART && !defined SOFTSPI
    #define RF24_SPI_TRANSACTIONS
  #endif
 
 



  #include <stdio.h>

  #define _BV(bit) \
	(1 << (bit)) 

  #define PSTR(x) (x)
  #define printf_P printf
  #define strlen_P strlen
  #define PROGMEM
  #define pgm_read_word(p) (*(p))
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
  #define PRIPSTR "%s"

  #ifdef SERIAL_DEBUG
        #define IF_SERIAL_DEBUG(x) ({x;})
  #else
        #define IF_SERIAL_DEBUG(x)
  #endif

  #include "esp/spi.h"

#endif // __RF24_CONFIG_H__

