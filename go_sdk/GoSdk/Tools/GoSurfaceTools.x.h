/** 
 * @file    GoSurfaceTools.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACE_TOOLS_X_H
#define GO_SURFACE_TOOLS_X_H

#include <kApi/Data/kXml.h>
kBeginHeader()

typedef struct GoSurfaceToolClass
{
    GoToolClass base; 

    kArrayList sourceOptions;
    GoDataSource source;

    kArrayList xAnchorOptions;
    k32s xAnchor;
    kArrayList yAnchorOptions;
    k32s yAnchor;
    kArrayList zAnchorOptions;
    k32s zAnchor;
} GoSurfaceToolClass; 

kDeclareClass(Go, GoSurfaceTool, GoTool)

#define GoSurfaceTool_Cast_(CONTEXT)    kCastClass_(GoSurfaceTool, CONTEXT)

GoFx(kStatus) GoSurfaceTool_Init(GoSurfaceTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceTool_VRelease(GoSurfaceTool tool);
GoFx(kStatus) GoSurfaceTool_Read(GoSurfaceTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceTool_Write(GoSurfaceTool tool, kXml xml, kXmlItem item);


typedef struct GoSurfaceBoxClass
{
    GoSurfaceToolClass base; 
    kBool zRotationEnabled;
    kBool regionEnabled;
    GoRegion3d region;
} GoSurfaceBoxClass; 

kDeclareClass(Go, GoSurfaceBox, GoSurfaceTool)

#define GoSurfaceBox_Cast_(CONTEXT)    kCastClass_(GoSurfaceBox, CONTEXT)

GoFx(kStatus) GoSurfaceBox_Construct(GoSurfaceBox* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceBox_VInit(GoSurfaceBox tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceBox_VRelease(GoSurfaceBox tool);
GoFx(kStatus) GoSurfaceBox_VRead(GoSurfaceBox tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceBox_VWrite(GoSurfaceBox tool, kXml xml, kXmlItem item); 

typedef struct GoSurfaceCountersunkHoleClass
{
    GoSurfaceToolClass base; 

    k64f nominalBevelAngle;
    k64f bevelAngleTolerance;
    k64f nominalOuterRadius;
    k64f outerRadiusTolerance;
    k64f nominalInnerRadius;
    k64f innerRadiusTolerance;
    k64f bevelRadiusOffset;

    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS];

    kBool autoTiltEnabled;
    k64f tiltXAngle;
    k64f tiltYAngle;

    kBool curveFitEnabled;
    k64f curveOrientation;
} GoSurfaceCountersunkHoleClass; 

kDeclareClass(Go, GoSurfaceCountersunkHole, GoSurfaceTool)

#define GoSurfaceCountersunkHole_Cast_(CONTEXT)    kCastClass_(GoSurfaceCountersunkHole, CONTEXT)

GoFx(kStatus) GoSurfaceCountersunkHole_Construct(GoSurfaceCountersunkHole* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceCountersunkHole_VInit(GoSurfaceCountersunkHole tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceCountersunkHole_VRelease(GoSurfaceCountersunkHole tool);
GoFx(kStatus) GoSurfaceCountersunkHole_VRead(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceCountersunkHole_VWrite(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item); 


typedef struct GoSurfaceEllipseClass
{
    GoSurfaceToolClass base; 
    kBool regionEnabled;
    GoRegion3d region;
} GoSurfaceEllipseClass; 

kDeclareClass(Go, GoSurfaceEllipse, GoSurfaceTool)

#define GoSurfaceEllipse_Cast_(CONTEXT)    kCastClass_(GoSurfaceEllipse, CONTEXT)

GoFx(kStatus) GoSurfaceEllipse_Construct(GoSurfaceEllipse* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceEllipse_VInit(GoSurfaceEllipse tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceEllipse_VRelease(GoSurfaceEllipse tool);
GoFx(kStatus) GoSurfaceEllipse_VRead(GoSurfaceEllipse tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEllipse_VWrite(GoSurfaceEllipse tool, kXml xml, kXmlItem item); 


typedef struct GoSurfaceHoleClass
{
    GoSurfaceToolClass base; 

    k64f nominalRadius;
    k64f radiusTolerance;
    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_HOLE_MAX_REF_REGIONS];
    k64f tiltXAngle;
    k64f tiltYAngle;
    kBool autoTiltEnabled;
} GoSurfaceHoleClass; 

kDeclareClass(Go, GoSurfaceHole, GoSurfaceTool)

#define GoSurfaceHole_Cast_(CONTEXT)    kCastClass_(GoSurfaceHole, CONTEXT)

GoFx(kStatus) GoSurfaceHole_Construct(GoSurfaceHole* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceHole_VInit(GoSurfaceHole tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceHole_VRelease(GoSurfaceHole tool);
GoFx(kStatus) GoSurfaceHole_VRead(GoSurfaceHole tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceHole_VWrite(GoSurfaceHole tool, kXml xml, kXmlItem item); 


typedef struct GoSurfaceOpeningClass
{
    GoSurfaceToolClass base; 

    GoSurfaceOpeningType type;
    k64f nominalWidth;
    k64f nominalLength;
    k64f nominalAngle;
    k64f nominalRadius;
    k64f widthTolerance;
    k64f lengthTolerance;
    k64f angleTolerance;
    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_OPENING_MAX_REF_REGIONS];
    kBool autoTiltEnabled;
    k64f tiltXAngle;
    k64f tiltYAngle;
} GoSurfaceOpeningClass; 

kDeclareClass(Go, GoSurfaceOpening, GoSurfaceTool)

#define GoSurfaceOpening_Cast_(CONTEXT)    kCastClass_(GoSurfaceOpening, CONTEXT)

GoFx(kStatus) GoSurfaceOpening_Construct(GoSurfaceOpening* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceOpening_VInit(GoSurfaceOpening tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceOpening_VRelease(GoSurfaceOpening tool);
GoFx(kStatus) GoSurfaceOpening_VRead(GoSurfaceOpening tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceOpening_VWrite(GoSurfaceOpening tool, kXml xml, kXmlItem item); 


typedef struct GoSurfacePlaneClass
{
    GoSurfaceToolClass base; 

    kBool regionsEnabled;
    kSize regionCount;
    GoRegion3d regions[GO_SURFACE_PLANE_MAX_REGIONS];
} GoSurfacePlaneClass; 

kDeclareClass(Go, GoSurfacePlane, GoSurfaceTool)

#define GoSurfacePlane_Cast_(CONTEXT)    kCastClass_(GoSurfacePlane, CONTEXT)

GoFx(kStatus) GoSurfacePlane_Construct(GoSurfacePlane* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfacePlane_VInit(GoSurfacePlane tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfacePlane_VRelease(GoSurfacePlane tool);
GoFx(kStatus) GoSurfacePlane_VRead(GoSurfacePlane tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfacePlane_VWrite(GoSurfacePlane tool, kXml xml, kXmlItem item); 


typedef struct GoSurfacePositionClass
{
    GoSurfaceToolClass base; 
    GoSurfaceFeature feature;
} GoSurfacePositionClass; 

kDeclareClass(Go, GoSurfacePosition, GoSurfaceTool)

#define GoSurfacePosition_Cast_(CONTEXT)    kCastClass_(GoSurfacePosition, CONTEXT)

GoFx(kStatus) GoSurfacePosition_Construct(GoSurfacePosition* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfacePosition_VInit(GoSurfacePosition tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfacePosition_VRelease(GoSurfacePosition tool);
GoFx(kStatus) GoSurfacePosition_VRead(GoSurfacePosition tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfacePosition_VWrite(GoSurfacePosition tool, kXml xml, kXmlItem item); 


typedef struct GoSurfaceStudClass
{
    GoSurfaceToolClass base; 
    k64f studRadius;
    k64f studHeight;
    k64f baseHeight;
    k64f tipHeight;

    kBool regionEnabled;
    GoRegion3d region;

    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_STUD_MAX_REF_REGIONS];
    kBool autoTiltEnabled;
    k64f tiltXAngle;
    k64f tiltYAngle;
} GoSurfaceStudClass; 

kDeclareClass(Go, GoSurfaceStud, GoSurfaceTool)

#define GoSurfaceStud_Cast_(CONTEXT)    kCastClass_(GoSurfaceStud, CONTEXT)

GoFx(kStatus) GoSurfaceStud_Construct(GoSurfaceStud* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceStud_VInit(GoSurfaceStud tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceStud_VRelease(GoSurfaceStud tool);
GoFx(kStatus) GoSurfaceStud_VRead(GoSurfaceStud tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceStud_VWrite(GoSurfaceStud tool, kXml xml, kXmlItem item); 


typedef struct GoSurfaceVolumeClass
{
    GoSurfaceToolClass base; 

    kBool regionEnabled;
    GoRegion3d region;
} GoSurfaceVolumeClass; 

kDeclareClass(Go, GoSurfaceVolume, GoSurfaceTool)

#define GoSurfaceVolume_Cast_(CONTEXT)    kCastClass_(GoSurfaceVolume, CONTEXT)

GoFx(kStatus) GoSurfaceVolume_Construct(GoSurfaceVolume* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceVolume_VInit(GoSurfaceVolume tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceVolume_VRelease(GoSurfaceVolume tool);
GoFx(kStatus) GoSurfaceVolume_VRead(GoSurfaceVolume tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceVolume_VWrite(GoSurfaceVolume tool, kXml xml, kXmlItem item); 


kEndHeader()

#endif
