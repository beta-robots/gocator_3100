/** 
 * @file    GoTool.h
 * @brief   Declares the base GoTool class.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOL_H
#define GO_TOOL_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoMeasurements.h>
#include <kApi/Data/kXml.h>
kBeginHeader()

/**
 * @class   GoTool
 * @extends kObject
 * @ingroup GoSdk-Tools
 * @brief   Represents the base tool class.
 */
typedef kObject GoTool; 

/** 
 * Returns the measurement count.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @return              The measurement count.            
 */
GoFx(kSize) GoTool_MeasurementCount(GoTool tool);

/** 
 * Retrieves the measurement at the given index.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    index      The index of the measurement.
 * @return              The measurement at the given index or kNULL if an invalid index is provided.            
 */
GoFx(GoMeasurement) GoTool_MeasurementAt(GoTool tool, kSize index);

/** 
 * Sets the name of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    name       The name to be set for the tool.
 * @return              Operation status.            
 */
GoFx(kStatus) GoTool_SetName(GoTool tool, const kChar* name);

/** 
 * Retrieves the name of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    name       Receives the name of the tool.
 * @param    capacity   The maximum capacity of the name array.
 * @return              Operation status.            
 */
GoFx(kStatus) GoTool_Name(GoTool tool, kChar* name, kSize capacity);

/** 
 * Retrieves the tool type enumeration value of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @return              The tool type enumeration value.            
 */
GoFx(GoToolType) GoTool_Type(GoTool tool);

/** 
 * Retrieves the first found instance of a measurement for a given enumeration type.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    type       A GoMeasurementType representing the measurement type to find in the given tool.
 * @return              A measurement object if one is found, otherwise kNULL.
 */
GoFx(GoMeasurement) GoTool_FindMeasurementByType(GoTool tool, GoMeasurementType type);

kEndHeader()
#include <GoSdk/Tools/GoTool.x.h>

#endif
