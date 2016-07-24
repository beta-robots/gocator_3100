/** 
 * @file    GoProfileTools.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOLS_X_H
#define GO_PROFILE_TOOLS_X_H

#include <kApi/Data/kXml.h>
kBeginHeader()

typedef struct GoProfileToolClass
{
    GoToolClass base; 

    kArrayList sourceOptions;
    GoDataSource source;

    kArrayList xAnchorOptions;   //of k32u
    k32s xAnchor;
    kArrayList zAnchorOptions;   //of k32u
    k32s zAnchor;
} GoProfileToolClass; 

kDeclareClass(Go, GoProfileTool, GoTool)

#define GoProfileTool_Cast_(CONTEXT)    kCastClass_(GoProfileTool, CONTEXT)

GoFx(kStatus) GoProfileTool_Init(GoProfileTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileTool_VRelease(GoProfileTool tool);
GoFx(kStatus) GoProfileTool_Read(GoProfileTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileTool_Write(GoProfileTool tool, kXml xml, kXmlItem item);


typedef struct GoProfileAreaClass
{
    GoProfileToolClass base; 

    GoProfileAreaType type;
    kBool typeUsed;

    GoProfileBaseline baseline;
    kBool baselineUsed;

    GoProfileRegion region;
    GoProfileLineRegion line;
} GoProfileAreaClass; 

kDeclareClass(Go, GoProfileArea, GoProfileTool)

#define GoProfileArea_Cast_(CONTEXT)    kCastClass_(GoProfileArea, CONTEXT)


GoFx(kStatus) GoProfileArea_Construct(GoProfileArea* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileArea_VInit(GoProfileArea tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileArea_VRelease(GoProfileArea tool);
GoFx(kStatus) GoProfileArea_VRead(GoProfileArea tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileArea_VWrite(GoProfileArea tool, kXml xml, kXmlItem item); 


typedef struct GoProfileBoxClass
{
    GoProfileToolClass base; 
    kBool regionEnabled;
    GoProfileRegion region;
} GoProfileBoxClass; 

kDeclareClass(Go, GoProfileBox, GoProfileTool)

#define GoProfileBox_Cast_(CONTEXT)    kCastClass_(GoProfileBox, CONTEXT)

GoFx(kStatus) GoProfileBox_Construct(GoProfileBox* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileBox_VInit(GoProfileBox tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileBox_VRelease(GoProfileBox tool);
GoFx(kStatus) GoProfileBox_VRead(GoProfileBox tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileBox_VWrite(GoProfileBox tool, kXml xml, kXmlItem item); 


typedef struct GoProfileCircleClass
{
    GoProfileToolClass base; 
    GoProfileRegion region;
} GoProfileCircleClass; 

kDeclareClass(Go, GoProfileCircle, GoProfileTool)

#define GoProfileCircle_Cast_(CONTEXT)    kCastClass_(GoProfileCircle, CONTEXT)

GoFx(kStatus) GoProfileCircle_Construct(GoProfileCircle* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileCircle_VInit(GoProfileCircle tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileCircle_VRelease(GoProfileCircle tool);
GoFx(kStatus) GoProfileCircle_VRead(GoProfileCircle tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileCircle_VWrite(GoProfileCircle tool, kXml xml, kXmlItem item); 


typedef struct GoProfileDimClass
{
    GoProfileToolClass base; 
    GoProfileFeature refFeature;
    GoProfileFeature feature;
} GoProfileDimClass; 

kDeclareClass(Go, GoProfileDim, GoProfileTool)

#define GoProfileDim_Cast_(CONTEXT)    kCastClass_(GoProfileDim, CONTEXT)

GoFx(kStatus) GoProfileDim_Construct(GoProfileDim* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileDim_VInit(GoProfileDim tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileDim_VRelease(GoProfileDim tool);
GoFx(kStatus) GoProfileDim_VRead(GoProfileDim tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileDim_VWrite(GoProfileDim tool, kXml xml, kXmlItem item); 


typedef struct GoProfileGrooveClass
{
    GoProfileToolClass base; 
    GoProfileGrooveShape shape;
    k64f minWidth;
    k64f maxWidth;
    k64f minDepth;
    GoProfileRegion region;
} GoProfileGrooveClass; 

kDeclareClass(Go, GoProfileGroove, GoProfileTool)

#define GoProfileGroove_Cast_(CONTEXT)    kCastClass_(GoProfileGroove, CONTEXT)

GoFx(kStatus) GoProfileGroove_Construct(GoProfileGroove* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileGroove_VInit(GoProfileGroove tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileGroove_VRelease(GoProfileGroove tool);
GoFx(kStatus) GoProfileGroove_VRead(GoProfileGroove tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGroove_VWrite(GoProfileGroove tool, kXml xml, kXmlItem item); 


typedef struct GoProfileIntersectClass
{
    GoProfileToolClass base; 
    GoProfileBaseline refLineType;
    GoProfileLineRegion refLineRegion;
    GoProfileLineRegion lineRegion;
} GoProfileIntersectClass; 

kDeclareClass(Go, GoProfileIntersect, GoProfileTool)

#define GoProfileIntersect_Cast_(CONTEXT)    kCastClass_(GoProfileIntersect, CONTEXT)

GoFx(kStatus) GoProfileIntersect_Construct(GoProfileIntersect* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileIntersect_VInit(GoProfileIntersect tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileIntersect_VRelease(GoProfileIntersect tool);
GoFx(kStatus) GoProfileIntersect_VRead(GoProfileIntersect tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileIntersect_VWrite(GoProfileIntersect tool, kXml xml, kXmlItem item); 


typedef struct GoProfileLineClass
{
    GoProfileToolClass base; 
    GoProfileRegion region;
} GoProfileLineClass; 

kDeclareClass(Go, GoProfileLine, GoProfileTool)

#define GoProfileLine_Cast_(CONTEXT)    kCastClass_(GoProfileLine, CONTEXT)

GoFx(kStatus) GoProfileLine_Construct(GoProfileLine* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileLine_VInit(GoProfileLine tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileLine_VRelease(GoProfileLine tool);
GoFx(kStatus) GoProfileLine_VRead(GoProfileLine tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLine_VWrite(GoProfileLine tool, kXml xml, kXmlItem item); 


typedef struct GoProfilePanelClass
{
    GoProfileToolClass base; 
    GoProfilePanelSide refEdgeSide;
    k64f maxGapWidth;
    GoProfileEdge leftEdge;
    GoProfileEdge rightEdge;
} GoProfilePanelClass; 

kDeclareClass(Go, GoProfilePanel, GoProfileTool)

#define GoProfilePanel_Cast_(CONTEXT)    kCastClass_(GoProfilePanel, CONTEXT)

GoFx(kStatus) GoProfilePanel_Construct(GoProfilePanel* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfilePanel_VInit(GoProfilePanel tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfilePanel_VRelease(GoProfilePanel tool);
GoFx(kStatus) GoProfilePanel_VRead(GoProfilePanel tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanel_VWrite(GoProfilePanel tool, kXml xml, kXmlItem item); 


typedef struct GoProfilePositionClass
{
    GoProfileToolClass base; 
    GoProfileFeature feature;
} GoProfilePositionClass; 

kDeclareClass(Go, GoProfilePosition, GoProfileTool)

#define GoProfilePosition_Cast_(CONTEXT)    kCastClass_(GoProfilePosition, CONTEXT)

GoFx(kStatus) GoProfilePosition_Construct(GoProfilePosition* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfilePosition_VInit(GoProfilePosition tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfilePosition_VRelease(GoProfilePosition tool);
GoFx(kStatus) GoProfilePosition_VRead(GoProfilePosition tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePosition_VWrite(GoProfilePosition tool, kXml xml, kXmlItem item); 


typedef struct GoProfileStripClass
{
    GoProfileToolClass base; 
    GoProfileStripBaseType baseType;
    GoProfileStripEdgeType leftEdge;
    GoProfileStripEdgeType rightEdge;
    kBool tiltEnabled;
    k64f supportWidth;
    k64f transitionWidth;
    k64f minWidth;
    k64f minHeight;
    k64f maxVoidWidth;
    GoProfileRegion region;
} GoProfileStripClass; 

kDeclareClass(Go, GoProfileStrip, GoProfileTool)

#define GoProfileStrip_Cast_(CONTEXT)    kCastClass_(GoProfileStrip, CONTEXT)

GoFx(kStatus) GoProfileStrip_Construct(GoProfileStrip* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileStrip_VInit(GoProfileStrip tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileStrip_VRelease(GoProfileStrip tool);
GoFx(kStatus) GoProfileStrip_VRead(GoProfileStrip tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileStrip_VWrite(GoProfileStrip tool, kXml xml, kXmlItem item); 

kEndHeader()

#endif
