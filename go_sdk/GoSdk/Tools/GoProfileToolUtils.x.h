/** 
 * @file    GoProfileToolUtils.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOL_UTILS_X_H
#define GO_PROFILE_TOOL_UTILS_X_H

#include <kApi/Data/kXml.h>
kBeginHeader()

typedef struct GoProfileRegionClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f z;
    k64f width;
    k64f height;
} GoProfileRegionClass; 

kDeclareClass(Go, GoProfileRegion, kObject)

#define GoProfileRegion_Cast_(CONTEXT)       kCastClass_(GoProfileRegion, CONTEXT)

GoFx(kStatus) GoProfileRegion_Construct(GoProfileRegion* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileRegion_Init(GoProfileRegion region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileRegion_VRelease(GoProfileRegion region);

GoFx(kStatus) GoProfileRegion_Read(GoProfileRegion region, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileRegion_Write(GoProfileRegion region, kXml xml, kXmlItem item); 

typedef struct GoProfileFeatureClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoProfileFeatureType type;
    GoProfileRegion region;
} GoProfileFeatureClass; 

kDeclareClass(Go, GoProfileFeature, kObject)

#define GoProfileFeature_Cast_(CONTEXT)        kCastClass_(GoProfileFeature, CONTEXT)

GoFx(kStatus) GoProfileFeature_Construct(GoProfileFeature* feature, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileFeature_Init(GoProfileFeature feature, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileFeature_VRelease(GoProfileFeature feature);

GoFx(kStatus) GoProfileFeature_Read(GoProfileFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileFeature_Write(GoProfileFeature feature, kXml xml, kXmlItem item); 


typedef struct GoProfileLineRegionClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k32u regionCount;
    GoProfileRegion regions[2];
} GoProfileLineRegionClass; 

kDeclareClass(Go, GoProfileLineRegion, kObject)

#define GoProfileLineRegion_Cast_(CONTEXT)        kCastClass_(GoProfileLineRegion, CONTEXT)

GoFx(kStatus) GoProfileLineRegion_Construct(GoProfileLineRegion* lineRegion, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileLineRegion_Init(GoProfileLineRegion lineRegion, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileLineRegion_VRelease(GoProfileLineRegion lineRegion);

GoFx(kStatus) GoProfileLineRegion_Read(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLineRegion_Write(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item); 


typedef struct GoProfileEdgeClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoProfileEdgeType edgeType;
    GoProfileRegion region;
    k64f maxVoidWidth;
    k64f minDepth;
    k64f surfaceWidth;
    k64f surfaceOffset;
    k64f nominalRadius;
    k64f edgeAngle;
} GoProfileEdgeClass; 

kDeclareClass(Go, GoProfileEdge, kObject)

#define GoProfileEdge_Cast_(CONTEXT)        kCastClass_(GoProfileEdge, CONTEXT)

GoFx(kStatus) GoProfileEdge_Construct(GoProfileEdge* profileEdge, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileEdge_Init(GoProfileEdge profileEdge, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileEdge_VRelease(GoProfileEdge profileEdge);

GoFx(kStatus) GoProfileEdge_Read(GoProfileEdge edge, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileEdge_Write(GoProfileEdge edge, kXml xml, kXmlItem item); 

kEndHeader()

#endif
