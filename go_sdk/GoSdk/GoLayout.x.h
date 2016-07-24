/** 
 * @file    GoLayout.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_LAYOUT_X_H
#define GO_LAYOUT_X_H

#include <kApi/Data/kXml.h>

kBeginHeader()

typedef struct GoLayoutClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    GoTransformedDataRegion transformedDataRegion;
    GoOrientation orientation;
    kBool multiplexBuddyEnabled;
    kBool multiplexSingleEnabled;
    k64f multiplexSingleDelay;
    k64f multiplexSingleExposureDuration; //read only
    k64f multiplexSinglePeriod;
    k64f multiplexSinglePeriodMin;

    k32u xSpacingCount;
    k32u ySpacingCount;
} GoLayoutClass;

kDeclareClass(Go, GoLayout, kObject)

#define GoLayout_Cast_(CONTEXT)    kCastClass_(GoLayout, CONTEXT)

GoFx(kStatus) GoLayout_Construct(GoLayout* layout, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoLayout_Init(GoLayout layout, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoLayout_VRelease(GoLayout layout);

GoFx(kStatus) GoLayout_Read(GoLayout layout, kXml xml, kXmlItem item);
GoFx(kStatus) GoLayout_Write(GoLayout layout, kXml xml, kXmlItem item); 

kEndHeader()

#endif
