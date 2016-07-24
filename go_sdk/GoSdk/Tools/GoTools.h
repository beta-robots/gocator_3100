/** 
 * @file    GoTools.h
 * @brief   Declares the GoTool classes. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOLS_H
#define GO_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/Tools/GoExtTool.h>
#include <GoSdk/Tools/GoProfileTools.h>
#include <GoSdk/Tools/GoRangeTools.h>
#include <GoSdk/Tools/GoSurfaceTools.h>

kBeginHeader()

typedef kObject GoToolOption;

/**
 * @class   GoTools
 * @extends kObject
 * @ingroup GoSdk-Tools
 * @brief   Represents a collection of tools.
 */
typedef kObject GoTools;

/** 
 * Returns the current number of tools.
 *
 * @public               @memberof GoTools
 * @version              Introduced in firmware 4.0.10.27
 * @param    tools       GoTools object.
 * @return               The current tool count.                
 */
GoFx(kSize) GoTools_ToolCount(GoTools tools);

/** 
 * Returns a tool handle at the given index.
 *
 * @public               @memberof GoTools
 * @version              Introduced in firmware 4.0.10.27
 * @param    tools       GoTools object.
 * @param    index       The index with which to retrieve a tool.
 * @return               A handle to a tool at the given index or kNULL if the index is invalid.
 */
GoFx(GoTool) GoTools_ToolAt(GoTools tools, kSize index);

/** 
 * Adds a tool and returns a handle to it. NOTE: This function will always succeed, 
 * but the tool handle will be null if passed in and an error occurs.
 *
 * @public               @memberof GoTools
 * @version              Introduced in firmware 4.0.10.27
 * @param    tools       GoTools object.
 * @param    type        The tool type enumerator value representing the type of tool to add.
 * @param    tool        A pointer to the newly added tool.
 * @return               Operation status.
 */
GoFx(kStatus) GoTools_AddTool(GoTools tools, GoToolType type, GoTool* tool);

/** 
 * Removes a tool at the given index.
 *
 * @public               @memberof GoTools
 * @version              Introduced in firmware 4.0.10.27
 * @param    tools       GoTools object.
 * @param    index       The index with which to remove a tool.
 * @return               Operation status.
 */
GoFx(kStatus) GoTools_RemoveTool(GoTools tools, kSize index);

/** 
 * Removes all tools in the given GoTools instance.
 *
 * @public               @memberof GoTools
 * @version              Introduced in firmware 4.0.10.27
 * @param    tools       GoTools object.
 * @return               Operation status.
 */
GoFx(kStatus) GoTools_ClearTools(GoTools tools);

/** 
 * Returns an enabled measurement handle if the specified ID is valid.
 *
 * @public               @memberof GoTools
 * @version              Introduced in firmware 4.0.10.27
 * @param    tools       GoTools object.
 * @param    id          The measurement ID to search for.
 * @return               A GoMeasurement object if an enabled measurement with the ID is found, otherwise kNULL.
 */
GoFx(GoMeasurement) GoTools_FindMeasurementById(GoTool tools, k32u id); 

/** 
 * Automatically update a given measurement to use a valid ID within the set of tools contained in the GoTools object.
 *
 * @public                  @memberof GoTools
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tools          GoTools object.
 * @param    measurement    GoMeasurement object to update.
 * @return                  Operation status.
 */
GoFx(kStatus) GoTools_AssignMeasurementId(GoTools tools, GoMeasurement measurement);


/**
 * @class   GoScript
 * @extends GoTool
 * @ingroup GoSdk-Tools
 * @brief   Represents a script tool.
 */
typedef GoTool GoScript; 

/** 
 * Gets the code for the script.
 *
 * @public               @memberof GoScript
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoScript object.
 * @param    code        Receives a null-terminated string containing the script code. 
 * @return               Operation status.                
 */
GoFx(kStatus) GoScript_Code(GoScript tool, kChar** code);

/** 
 * Sets the code for the script.
 *
 * @public               @memberof GoScript
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoScript object.
 * @param    code        The code to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoScript_SetCode(GoScript tool, kChar* code);

/** 
 * Adds a script output.
 *
 * @public               @memberof GoScript
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoScript object.
 * @param    id          An ID(must not already be used by other measurements) of the script output to add.
 * @return               Operation status.
 */
GoFx(kStatus) GoScript_AddOutput(GoScript tool, k32u id);

/** 
 * Removes a script output with the specific ID.
 *
 * @public               @memberof GoScript
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoScript object.
 * @param    id          An ID of the script output to remove.
 * @return               Operation status.
 */
GoFx(kStatus) GoScript_RemoveOutput(GoScript tool, k32u id);

/** 
 * Returns the count of script tool outputs.
 *
 * @public               @memberof GoScript
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoScript object.
 * @return               Script output count.
 */
GoFx(kSize) GoScript_OutputCount(GoScript tool);

/** 
 * Returns a handle to a script output at the given index.
 *
 * @public               @memberof GoScript
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoScript object.
 * @param    index       The index with which to return a corresponding script output.
 * @return               A GoScriptOutput object or kNULL if the index is invalid.
 */
GoFx(GoScriptOutput) GoScript_OutputAt(GoScript tool, kSize index);

kEndHeader()
#include <GoSdk/Tools/GoTools.x.h>

#endif
