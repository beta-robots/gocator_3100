/** 
 * @file    GoSurfaceToolUtils.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACE_TOOL_UTILS_X_H
#define GO_SURFACE_TOOL_UTILS_X_H

#include <kApi/Data/kXml.h>
kBeginHeader()

typedef struct GoSurfaceRegion2dClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f y;
    k64f width;
    k64f length;
} GoSurfaceRegion2dClass; 

kDeclareClass(Go, GoSurfaceRegion2d, kObject)

#define GoSurfaceRegion2d_Cast_(CONTEXT)    kCastClass_(GoSurfaceRegion2d, CONTEXT)

GoFx(kStatus) GoSurfaceRegion2d_Construct(GoSurfaceRegion2d* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceRegion2d_Init(GoSurfaceRegion2d region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceRegion2d_VRelease(GoSurfaceRegion2d region);

GoFx(kStatus) GoSurfaceRegion2d_Read(GoSurfaceRegion2d region, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRegion2d_Write(GoSurfaceRegion2d region, kXml xml, kXmlItem item); 


typedef struct GoRegion3dClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f y;
    k64f z;
    k64f width;
    k64f length;
    k64f height;
} GoRegion3dClass; 

kDeclareClass(Go, GoRegion3d, kObject)

#define GoRegion3d_Cast_(CONTEXT)    kCastClass_(GoRegion3d, CONTEXT)

GoFx(kStatus) GoRegion3d_Construct(GoRegion3d* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoRegion3d_Init(GoRegion3d region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoRegion3d_VRelease(GoRegion3d region);

GoFx(kStatus) GoRegion3d_Read(GoRegion3d region, kXml xml, kXmlItem item);
GoFx(kStatus) GoRegion3d_Write(GoRegion3d region, kXml xml, kXmlItem item); 


typedef struct GoCylinderRegionClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f y;
    k64f z;
    k64f radius;
    k64f height;
} GoCylinderRegionClass; 

kDeclareClass(Go, GoCylinderRegion, kObject)

#define GoCylinderRegion_Cast_(CONTEXT)    kCastClass_(GoCylinderRegion, CONTEXT)

GoFx(kStatus) GoCylinderRegion_Construct(GoCylinderRegion* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoCylinderRegion_Init(GoCylinderRegion region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoCylinderRegion_VRelease(GoCylinderRegion region);

GoFx(kStatus) GoCylinderRegion_Read(GoCylinderRegion region, kXml xml, kXmlItem item);
GoFx(kStatus) GoCylinderRegion_Write(GoCylinderRegion region, kXml xml, kXmlItem item); 



typedef struct GoSurfaceFeatureClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoSurfaceFeatureType type;
    kBool regionEnabled;
    GoRegion3d region;
} GoSurfaceFeatureClass; 

kDeclareClass(Go, GoSurfaceFeature, kObject)

#define GoSurfaceFeature_Cast_(CONTEXT)    kCastClass_(GoSurfaceFeature, CONTEXT)

GoFx(kStatus) GoSurfaceFeature_Construct(GoSurfaceFeature* feature, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceFeature_Init(GoSurfaceFeature feature, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceFeature_VRelease(GoSurfaceFeature feature);

GoFx(kStatus) GoSurfaceFeature_Read(GoSurfaceFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceFeature_Write(GoSurfaceFeature feature, kXml xml, kXmlItem item); 

kEndHeader()

#endif
