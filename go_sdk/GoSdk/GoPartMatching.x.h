/** 
 * @file    GoPartMatching.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PART_MATCHING_X_H
#define GO_PART_MATCHING_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoSurfaceTools.h>

kBeginHeader()

typedef struct GoPartMatchEdge
{
    kText256 modelName;
    k64f qualityDecisionMin; //range
} GoPartMatchEdge;

typedef struct GoPartMatchBox
{
    k64f zAngle;

    k64f widthMin;
    k64f widthMax;
    k64f lengthMin;
    k64f lengthMax;
} GoPartMatchBox;

typedef struct GoPartMatchEllipse
{
    k64f zAngle;

    k64f majorMin;
    k64f majorMax;
    k64f minorMin;
    k64f minorMax;
} GoPartMatchEllipse;

typedef struct GoPartMatchingClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    kBool enabled;
    kBool enabledUsed;

    GoPartMatchAlgorithm algorithm;

    GoPartMatchEdge edge;
    GoPartMatchBox boundingBox;
    GoPartMatchEllipse ellipse;
} GoPartMatchingClass;

kDeclareClass(Go, GoPartMatching, kObject)

#define GoPartMatching_Cast_(CONTEXT)    kCastClass_(GoPartMatching, CONTEXT)

GoFx(kStatus) GoPartMatching_Construct(GoPartMatching* part, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoPartMatching_Init(GoPartMatching partDetection, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoPartMatching_VRelease(GoPartMatching partDetection);

GoFx(kStatus) GoPartMatching_Read(GoPartMatching partDetection, kXml xml, kXmlItem item);
GoFx(kStatus) GoPartMatching_Write(GoPartMatching matching, kXml xml, kXmlItem item); 

kEndHeader()

#endif
