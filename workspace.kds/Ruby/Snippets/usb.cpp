/** 
    @file usb.cpp
    @brief Simple USB Stack for Kinetis

    @version  V4.12.1.150
    @date     13 Nov 2016

   \verbatim
    Kinetis USB Code

    Copyright (C) 2008-16  Peter O'Donoghue

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
    \endverbatim

\verbatim
Change History
+=================================================================================
| 20 Oct 2016 | Created
+=================================================================================
\endverbatim
 */
/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include <string.h>
#include <stdio.h>
#include "derivative.h"
#include "usb.h"

namespace USBDM {

/** BDTs organised by endpoint, odd/even, tx/rx */
EndpointBdtEntry endPointBdts[Usb0::NUMBER_OF_ENDPOINTS] __attribute__ ((aligned (512)));

#ifdef MS_COMPATIBLE_ID_FEATURE

const MS_CompatibleIdFeatureDescriptor msCompatibleIdFeatureDescriptor = {
      /* lLength;             */  nativeToLe32((uint32_t)sizeof(MS_CompatibleIdFeatureDescriptor)),
      /* wVersion;            */  nativeToLe16(0x0100),
      /* wIndex;              */  nativeToLe16(0x0004),
      /* bnumSections;        */  1,
      /*---------------------- Section 1 -----------------------------*/
      /* bReserved1[7];       */  {0},
      /* bInterfaceNum;       */  0,
      /* bReserved2;          */  1,
      /* bCompatibleId[8];    */  "WINUSB\0",
      /* bSubCompatibleId[8]; */  {0},
      /* bReserved3[6];       */  {0}
};

const MS_PropertiesFeatureDescriptor msPropertiesFeatureDescriptor = {
      /* uint32_t lLength;         */ nativeToLe32((uint32_t)sizeof(MS_PropertiesFeatureDescriptor)),
      /* uint16_t wVersion;        */ nativeToLe16(0x0100),
      /* uint16_t wIndex;          */ nativeToLe16(5),
      /* uint16_t bnumSections;    */ nativeToLe16(2),
      /*---------------------- Section 1 -----------------------------*/
      /* uint32_t lPropertySize0;  */ nativeToLe32(
            sizeof(msPropertiesFeatureDescriptor.lPropertySize0)+
            sizeof(msPropertiesFeatureDescriptor.ldataType0)+
            sizeof(msPropertiesFeatureDescriptor.wNameLength0)+
            sizeof(msPropertiesFeatureDescriptor.bName0)+
            sizeof(msPropertiesFeatureDescriptor.wPropertyLength0)+
            sizeof(msPropertiesFeatureDescriptor.bData0)
      ),
      /* uint32_t ldataType0;       */ nativeToLe32(1U), // 1 = Unicode string
      /* uint16_t wNameLength0;     */ nativeToLe16(sizeof(msPropertiesFeatureDescriptor.bName0)),
      /* char16_t bName0[42];       */ MS_DEVICE_INTERFACE_GUIDs,
      /* uint32_t wPropertyLength0; */ nativeToLe32(sizeof(msPropertiesFeatureDescriptor.bData0)),
      /* char16_t bData0[78];       */ MS_DEVICE_GUID,
      /*---------------------- Section 2 -----------------------------*/
      /* uint32_t lPropertySize1;   */ nativeToLe32(
            sizeof(msPropertiesFeatureDescriptor.lPropertySize1)+
            sizeof(msPropertiesFeatureDescriptor.ldataType1)+
            sizeof(msPropertiesFeatureDescriptor.wNameLength1)+
            sizeof(msPropertiesFeatureDescriptor.bName1)+
            sizeof(msPropertiesFeatureDescriptor.wPropertyLength1)+
            sizeof(msPropertiesFeatureDescriptor.bData1)
      ),
      /* uint32_t ldataType1;       */ nativeToLe32(2U), // 2 = Unicode string with environment variables
      /* uint16_t wNameLength1;     */ nativeToLe16(sizeof(msPropertiesFeatureDescriptor.bName1)),
      /* uint8_t  bName1[];         */ MS_ICONS,
      /* uint32_t wPropertyLength1; */ nativeToLe32(sizeof(msPropertiesFeatureDescriptor.bData1)),
      /* uint8_t  bData1[];         */ MS_ICON_PATH,
};
#endif

/**
 * Get name of USB token
 *
 * @param  token USB token
 *
 * @return Pointer to static string
 */
const char *UsbBase::getTokenName(unsigned token) {
   static const char *names[] = {
         "Unknown #0",
         "OUTToken",   //  (0x1) - Out token
         "ACKToken",   //  (0x2) - Acknowledge
         "DATA0Token", //  (0x3) - Data 0
         "Unknown #4",
         "SOFToken",   //  (0x5) - Start of Frame token
         "NYETToken",  //  (0x6) - No Response Yet
         "DATA2Token", //  (0x7) - Data 2
         "Unknown #8",
         "INToken",    //  (0x9) - In token
         "NAKToken",   //  (0xA) - Negative Acknowledge
         "DATA1Token", //  (0xB) - Data 1
         "PREToken",   //  (0xC) - Preamble
         "SETUPToken", //  (0xD) - Setup token
         "STALLToken", //  (0xE) - Stall
         "MDATAToken", //  (0xF) - M data
   };
   const char *rc = "Unknown";
   if (token<(sizeof(names)/sizeof(names[0]))) {
      rc = names[token];
   }
   return rc;
}

/**
 * Get name of USB request
 *
 * @param  reqType Request type
 *
 * @return Pointer to static string
 */
const char *UsbBase::getRequestName(uint8_t reqType){
   static const char *names[] = {
         "GET_STATUS",              /* 0x00 */
         "CLEAR_FEATURE",           /* 0x01 */
         "Unknown #2",
         "SET_FEATURE",             /* 0x03 */
         "Unknown #4",
         "SET_ADDRESS",             /* 0x05 */
         "GET_DESCRIPTOR",          /* 0x06 */
         "SET_DESCRIPTOR",          /* 0x07 */
         "GET_CONFIGURATION",       /* 0x08 */
         "SET_CONFIGURATION",       /* 0x09 */
         "GET_INTERFACE",           /* 0x0a */
         "SET_INTERFACE",           /* 0x0b */
         "SYNCH_FRAME",             /* 0x0c */
         "Unknown #D",
         "Unknown #E",
         "Unknown #F",
   };
   const char *rc = "Unknown";
   if (reqType<(sizeof(names)/sizeof(names[0]))) {
      rc = names[reqType];
   }
   return rc;
}

/**
 * Report contents of BDT
 *
 * @param name    Descriptive name to use
 * @param bdt     BDT to report
 */
void UsbBase::reportBdt(const char *name, BdtEntry *bdt) {
   (void)name;
   (void)bdt;
   if (bdt->u.setup.own) {
      PRINTF("%s addr=0x%08lX, bc=%d, %s, %s, %s\n",
            name,
            bdt->addr, bdt->bc,
            bdt->u.setup.data0_1?"DATA1":"DATA0",
                  bdt->u.setup.bdt_stall?"STALL":"OK",
                        "USB"
      );
   }
   else {
      PRINTF("%s addr=0x%08lX, bc=%d, %s, %s\n",
            name,
            bdt->addr, bdt->bc,
            getTokenName(bdt->u.result.tok_pid),
            "PROC"
      );
   }
}

/**
 * Report contents of LineCodingStructure to stdout
 *
 * @param lineCodingStructure
 */
void UsbBase::reportLineCoding(const LineCodingStructure *lineCodingStructure) {
   (void)lineCodingStructure;
   PRINTF("rate   = %ld bps\n", lineCodingStructure->dwDTERate);
   PRINTF("format = %d\n", lineCodingStructure->bCharFormat);
   PRINTF("parity = %d\n", lineCodingStructure->bParityType);
   PRINTF("bits   = %d\n", lineCodingStructure->bDataBits);
}

/**
 * Format SETUP packet as string
 *
 * @param p SETUP packet
 *
 * @return Pointer to static buffer
 */
const char *UsbBase::reportSetupPacket(SetupPacket *p) {
   static char buff[100];
   snprintf(buff, sizeof(buff), "[0x%02X,%s(0x%02X),%d,%d,%d]",
         p->bmRequestType,
         getRequestName(p->bRequest),
         p->bRequest,
         (int)(p->wValue),
         (int)(p->wIndex),
         (int)(p->wLength)
   );
   return buff;
}

/**
 * Report line state value to stdout
 *
 * @param value
 */
void UsbBase::reportLineState(uint8_t value) {
   (void)value;
   PRINTF("Line state: RTS=%d, DTR=%d\n", (value&(1<<1))?1:0, (value&(1<<0))?1:0);
}

/**
 *  Creates a valid string descriptor in UTF-16-LE from a limited UTF-8 string
 *
 *  @param to       Where to place descriptor
 *  @param from     Zero terminated UTF-8 C string
 *  @param maxSize  Size of destination
 *
 *  @note Only handles UTF-8 characters that fit in a single UTF-16 value.
 */
void UsbBase::utf8ToStringDescriptor(uint8_t *to, const uint8_t *from, unsigned maxSize) {
   uint8_t *size = to; // 1st byte is where to place descriptor size

   *to++ = 2;         // 1st byte = descriptor size (2 bytes so far)
   *to++ = DT_STRING; // 2nd byte = descriptor type, DT_STRING;

   while (*from != '\0') {
      // Buffer for converted character
      uint16_t utf16Char=0;

      // Update size
      *size  += 2;
      if (*from < 0x80) {
         // 1-byte UTF-8
         utf16Char = *from++;
      }
      else if ((*from &0xE0) == 0xC0){
         // 2-byte UTF-8
         utf16Char  = (0x1F&*from++)<<6;
         utf16Char += (0x3F&*from++);
      }
      else if ((*from &0xF0) == 0xE0){
         // 3-byte UTF-8
         utf16Char  = (0x0F&*from++)<<12;
         utf16Char += (0x3F&*from++)<<6;
         utf16Char += (0x3F&*from++);
      }
      // Write UTF-16LE value
      *to++ = (char)utf16Char;
      *to++ = (char)(utf16Char>>8);
      if (*size>=maxSize) {
         // Truncate value
         break;
      }
   }
}

} // End namespace USBDM
