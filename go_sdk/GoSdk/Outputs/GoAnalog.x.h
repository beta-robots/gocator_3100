/** 
 * @file    GoAnalog.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ANALOG_X_H
#define GO_ANALOG_X_H

#include <GoSdk/Outputs/GoAnalog.h>
#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>

kBeginHeader()

typedef struct GoAnalogClass
{   
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    kArrayList measurementOptions; 
    kArrayList measurementSource; //while only one source is expected, an array list is used such that the no selection scenario can be covered

    GoElement64f currentMin; 
    GoElement64f currentMax; 
    GoElement64f currentInvalid; 

    kBool scheduleEnabled; 
    k64s delay; 
    GoOutputDelayDomain delayDomain;
    GoAnalogEvent event;

    k64f dataScaleMin; 
    k64f dataScaleMax; 
} GoAnalogClass; 

kDeclareClass(Go, GoAnalog, kObject)

#define GoAnalog_Cast_(CONTEXT)    kCastClass_(GoAnalog, CONTEXT)

GoFx(kStatus) GoAnalog_Construct(GoAnalog* Analog, kObject sensor, kAlloc allocator); 

GoFx(kStatus) GoAnalog_Init(GoAnalog Analog, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoAnalog_VRelease(GoAnalog Analog); 

GoFx(kStatus) GoAnalog_Read(GoAnalog Analog, kXml xml, kXmlItem item);
GoFx(kStatus) GoAnalog_Write(GoAnalog Analog, kXml xml, kXmlItem item);

/** 
 * Gets the list of source options for the specified output type.
 *
 * @public              @memberof GoAnalog
 * @param   analog      GoAnalog object.
 * @return              An array list of source options.
 */
GoFx(kArrayList) GoAnalog_OptionList(GoAnalog analog);

kEndHeader()

#endif