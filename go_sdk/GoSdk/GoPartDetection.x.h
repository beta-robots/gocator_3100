/** 
 * @file    GoPartDetection.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PART_DETECTION_X_H
#define GO_PART_DETECTION_X_H

#include <kApi/Data/kXml.h>

kBeginHeader()

typedef struct GoPartEdgeFiltering
{
    kBool enabled;
    kBool used;
    kBool preserveInterior;
    GoElement64f elementWidth;
    GoElement64f elementLength;
} GoPartEdgeFiltering;

typedef struct GoPartDetectionClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    kBool enabled;
    kBool enabledSystemValue;
    kBool enabledUsed;

    GoPartHeightThresholdDirection thresholdDirection;
    GoElement64f threshold;

    GoElement64f gapWidth;
    kBool gapWidthUsed;

    GoElement64f gapLength;
    kBool gapLengthUsed;

    GoElement64f paddingWidth;
    kBool paddingWidthUsed;

    GoElement64f paddingLength;
    kBool paddingLengthUsed;

    GoElement64f minLength;
    kBool minLengthUsed;

    GoElement64f maxLength;
    kBool maxLengthUsed;

    GoElement64f minArea;
    kBool minAreaUsed;

    GoPartFrameOfReference frameOfReference;
    kBool frameOfReferenceUsed;

    GoPartEdgeFiltering edgeFiltering;
} GoPartDetectionClass;

kDeclareClass(Go, GoPartDetection, kObject)

#define GoPartDetection_Cast_(CONTEXT)    kCastClass_(GoPartDetection, CONTEXT)

GoFx(kStatus) GoPartDetection_Construct(GoPartDetection* part, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoPartDetection_Init(GoPartDetection partDetection, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoPartDetection_VRelease(GoPartDetection partDetection);

GoFx(kStatus) GoPartDetection_Read(GoPartDetection partDetection, kXml xml, kXmlItem item);
GoFx(kStatus) GoPartDetection_Write(GoPartDetection detection, kXml xml, kXmlItem item); 

kEndHeader()

#endif
