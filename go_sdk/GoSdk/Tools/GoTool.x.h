/** 
 * @file    GoTool.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOL_X_H
#define GO_TOOL_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoMeasurement.h>
kBeginHeader()

typedef struct GoToolVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(GoTool tool, kType type, kObject sensor, kAlloc allocator); 
    kStatus (kCall* VWrite)(GoTool tool, kXml xml, kXmlItem item); 
    kStatus (kCall* VRead)(GoTool tool, kXml xml, kXmlItem item);
} GoToolVTable; 

typedef struct GoToolClass
{
    kObjectClass base; 
    kObject sensor; 

    kSize id;   //represents position in the Tools element
    kXml xml;
    kXmlItem xmlItem;

    kText128 name;
    kArrayList measurements;
    kArrayList measurementOptions;
    kArrayList nodesToMerge;
    GoToolType typeId;
} GoToolClass; 

kDeclareVirtualClass(Go, GoTool, kObject)

#define GoTool_Cast_(CONTEXT)                   kCastClass_(GoTool, CONTEXT)
#define GoTool_Class_(MODE)                     (kCastClass_(GoTool, MODE))
#define GoTool_VTable_(MODE)                    (kCast(GoToolVTable*, kType_VTable_(kObject_Type_(MODE))))

GoFx(kStatus) GoTool_Construct(GoTool* tool, kType type, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoTool_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTool_VRelease(GoTool tool);
GoFx(kStatus) GoTool_VRead(GoTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoTool_VWrite(GoTool tool, kXml xml, kXmlItem item); 
GoFx(kStatus) GoTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTool_Read(GoTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoTool_Write(GoTool tool, kXml xml, kXmlItem item); 
GoFx(kObject) GoTool_Sensor(GoTool tool);
GoFx(kStatus) GoTool_SetId(GoTool tool, kSize id);

/** 
 * Adds the given measurement to the tool set.
 *
 * @public                  @memberof GoTool
 * @param    tool           GoTool object.
 * @param    type           The type of the measurement.
 * @param    isFilterable   Indicates whether the measurement can be toggled and filtered;
 * @param    measurement    A pointer to hold a reference to the created measurement (can be null).
 * @return   Operation status.            
 */
GoFx(kStatus) GoTool_AddMeasurement(GoTool tool, kType type, kBool isFilterable, GoMeasurement* measurement);


/** 
 * Removes a measurement at a given index.
 *
 * @public               @memberof GoTool
 * @param    tool        GoTool object.
 * @param    index       The index of the measurement to remove.
 * @return   Operation status.            
 */
GoFx(kStatus) GoTool_RemoveMeasurement(GoTool tool, kSize index);

/** 
 * Removes all measurements for the given tool.
 *
 * @public               @memberof GoTool
 * @param    tool        GoTool object.
 * @return   Operation status.            
 */
GoFx(kStatus) GoTool_ClearMeasurements(GoTool tool);


kEndHeader()

#endif
