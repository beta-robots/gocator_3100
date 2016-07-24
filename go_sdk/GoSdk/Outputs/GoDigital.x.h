/** 
 * @file    GoDigital.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_DIGITAL_X_H
#define GO_DIGITAL_X_H

#include <GoSdk/Outputs/GoDigital.h>
#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>

kBeginHeader()

typedef struct GoDigitalClass
{   
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    //Configuration elements
    kArrayList decisionOptions; 
    kArrayList decisionSources; 

    GoDigitalPass passMode; 
    kBool scheduleEnabled; 
    GoDigitalSignal signalType; 
    k64s delay; 
    GoOutputDelayDomain delayDomain;
    GoDigitalEvent event;

    GoElement32u pulseWidth;
} GoDigitalClass; 

kDeclareClass(Go, GoDigital, kObject)

#define GoDigital_Cast_(CONTEXT)    kCastClass_(GoDigital, CONTEXT)

GoFx(kStatus) GoDigital_Construct(GoDigital* Digital, k32u id, kObject sensor, kAlloc allocator); 

GoFx(kStatus) GoDigital_Init(GoDigital Digital, kType type, k32u id, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoDigital_VRelease(GoDigital Digital); 

GoFx(kStatus) GoDigital_Read(GoDigital Digital, kXml xml, kXmlItem item);
GoFx(kStatus) GoDigital_Write(GoDigital Digital, kXml xml, kXmlItem item);

/** 
 * Gets the list of source options for the specified output type.
 *
 * @public                  @memberof GoDigital
 * @param   digital         GoDigital object.
 * @return                  An array list of source options.
 */
GoFx(kArrayList) GoDigital_OptionList(GoDigital digital);

kEndHeader()

#endif