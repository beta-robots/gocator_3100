/** 
 * @file    GoOutput.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_OUTPUT_X_H
#define GO_SDK_OUTPUT_X_H

#include <GoSdk/Outputs/GoOutput.h>
#include <kApi/Data/kXml.h>
#include <kApi/Data/kString.h>
kBeginHeader()

#define GO_DIGITAL_OUTPUT_COUNT                   (2)

typedef struct GoOutputClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoEthernet ethernet; 
    GoSerial serial; 
    GoDigital digital[GO_DIGITAL_OUTPUT_COUNT]; 
    GoAnalog analog; 
} GoOutputClass; 

kDeclareClass(Go, GoOutput, kObject)

GoFx(kStatus) GoOutput_Construct(GoOutput* output, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoOutput_Init(GoOutput output, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoOutput_VRelease(GoOutput output);

GoFx(kStatus) GoOutput_Read(GoOutput output, kXml xml, kXmlItem item);
GoFx(kStatus) GoOutput_Write(GoOutput output, kXml xml, kXmlItem item); 

kEndHeader()

#endif