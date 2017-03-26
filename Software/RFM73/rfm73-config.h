//
// Hi-Tech Lite include file for DB038 board with 16F887:
// using an (external) RFM70 transciever
//
// Loosely based on the example application provided by HopeRF
//
//
// $Id: rfm73-config.h,v 1.1 2013/06/12 06:45:52 Staples Exp $
//
// (c) Wouter van Ooijen - wouter@voti.nl
//
// Copying and distribution of this file, with or without modification,
// are permitted in any medium without royalty provided the copyright
// notice and this notice are preserved.  This file is offered as-is,
// without any warranty.
//

#ifndef _DB038_RFM70_H_
#define _DB038_RFM70_H_

#include "peripheral.h"

#define RFM73_CSN( x )   if(x) { \
                           GPIO_WriteHigh(PORT_NSS, PIN_NSS); \
                         } else { \
                           GPIO_WriteLow(PORT_NSS, PIN_NSS); \
                         }

#define RFM73_CE( x )    if(x) { \
                           GPIO_WriteHigh(PORT_SPI_IRQ_CE, PIN_CE); \
                         } else { \
                           GPIO_WriteLow(PORT_SPI_IRQ_CE, PIN_CE); \
                         }

#define RFM73_WAIT_MS( x ) waitMs( x )

#include "rfm73.h"

#endif