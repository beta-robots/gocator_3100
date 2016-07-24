/** 
 * @file    GoSerial.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SERIAL_X_H
#define GO_SERIAL_X_H

#include <GoSdk/Outputs/GoSerial.h>
#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>
kBeginHeader()

typedef struct GoSerialClass
{   
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoSerialProtocol protocol; 
    kArrayList protocolOptions; 

    kArrayList measurementOptions; 
    kArrayList measurementSources; 

    GoSelcomConfig selcom;
    GoAsciiConfig ascii;
} GoSerialClass; 

kDeclareClass(Go, GoSerial, kObject)

#define GoSerial_Cast_(CONTEXT)    kCastClass_(GoSerial, CONTEXT)

GoFx(kStatus) GoSerial_Construct(GoSerial* Serial, kObject sensor, kAlloc allocator); 

GoFx(kStatus) GoSerial_Init(GoSerial Serial, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSerial_VRelease(GoSerial Serial); 

GoFx(kStatus) GoSerial_Read(GoSerial Serial, kXml xml, kXmlItem item);
GoFx(kStatus) GoSerial_Write(GoSerial Serial, kXml xml, kXmlItem item); 

/** 
 * Gets the list of source measurement options.
 *
 * @public                  @memberof GoSerial
 * @param   serial          GoSerial object.
 * @return                  An array list of source measurement options.
 */
GoFx(kArrayList) GoSerial_OptionList(GoSerial serial);

kEndHeader()

#endif