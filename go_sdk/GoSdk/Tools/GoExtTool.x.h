/** 
 * @file    GoExtTool.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_TOOL_X_H
#define GO_EXT_TOOL_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoMeasurement.h>
kBeginHeader()

typedef struct GoExtToolAnchorCfg
{
    kBool used;
    kArrayList options;
    k32s anchor;
} GoExtToolAnchorCfg;

typedef struct GoExtToolClass
{
    GoToolClass base; 
    
    kText128 toolType;
    
    kArrayList sourceOptions;
    GoDataSource source;

    GoExtToolAnchorCfg anchoring[3];    // 3 for X, Y, and Z anchoring

    kArrayList parameters; //of type GoExtParam
} GoExtToolClass; 

kDeclareClass(Go, GoExtTool, GoTool)

#define GoExtTool_Cast_(CONTEXT)    kCastClass_(GoExtTool, CONTEXT)

GoFx(kStatus) GoExtTool_Construct(GoExtTool* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtTool_VInit(GoExtTool tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtTool_VRelease(GoExtTool tool);
GoFx(kStatus) GoExtTool_VRead(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_VWrite(GoExtTool tool, kXml xml, kXmlItem item); 

GoFx(kXmlItem) GoExtTool_ConfigurationXml(GoExtTool tool);

GoFx(kStatus) GoExtTool_Read32sContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_Read64fContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_ReadBoolContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_ReadStringContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_ReadProfileRegionContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_ReadSurfaceRegion2dContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_ReadRegion3dContainer(GoExtTool tool, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtTool_Write32sContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_Write64fContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_WriteBoolContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_WriteStringContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_WriteProfileRegionContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_WriteSurfaceRegion2dContainer(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_WriteRegion3dContainer(GoExtTool tool, kXml xml, kXmlItem item);

kEndHeader()

#endif
