//
// Hi-Tech Lite include file for DB038 board with 16F887:
// using an (external) RFM70 transciever
//
// Loosely based on the example application provided by HopeRF
//
//
// $Id: rfm70-config.h,v 1.1 2011/12/24 13:47:23 Wouter Exp $
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

/*
#define RFM70_SCK( x )   PIN_COPY( PORTC, 0, (x) ) 
#define RFM70_MOSI( x )  PIN_COPY( PORTC, 1, (x) )
#define RFM70_MISO       ( RC2 )
*/
#define RFM70_CSN( x )   if(x) {GPIO_WriteHigh(port_NSS, pin_NSS);} else {GPIO_WriteLow(port_NSS, pin_NSS);}
#define RFM70_CE( x )    if(x) {GPIO_WriteHigh(port_SPI_IRQ_CE, pin_CE);} else {GPIO_WriteLow(port_SPI_IRQ_CE, pin_CE);}

/*
#define RFM70_PIN_DIRECTION { \
   TRISC0 = 0; \
   TRISC1 = 0; \
   TRISC2 = 1; \
   TRISC3 = 0; \
   TRISC4 = 0; \
}
*/

//#define RFM70_WAIT_US( x ) WAIT_US( x )
#define RFM70_WAIT_MS( x ) waitMs( x )

#include "rfm70.h"

#endif