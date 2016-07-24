///@cond private

/** 
 * @file    GoExtTool.h
 * @brief   Declares the base GoExtTool class.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_TOOL_H
#define GO_EXT_TOOL_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <kApi/Data/kXml.h>
kBeginHeader()

/**
 * @class   GoExtTool
 * @extends GoTool
 * @ingroup GoSdk-Tools
 * @brief   Represents an extensible tool.
 */
typedef GoTool GoExtTool; 

/** 
 * Returns the measurement count.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              The measurement count.            
 */
GoFx(kSize) GoExtTool_MeasurementCount(GoExtTool tool);

/** 
 * Retrieves the measurement at the given index.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @param    index      The index of the measurement.
 * @return              The measurement at the given index or kNULL if an invalid index is provided.            
 */
GoFx(GoMeasurement) GoExtTool_MeasurementAt(GoExtTool tool, kSize index);

/** 
* Sets the name of the tool.
*
* @public              @memberof GoExtTool
* @param    tool       GoExtTool object.
* @param    name       The name to be set for the tool.
* @return              Operation status.            
*/
GoFx(kStatus) GoExtTool_SetDisplayName(GoExtTool tool, const kChar* name);

/** 
 * Retrieves the display name of the tool.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              The tool display name.
 */
GoFx(const kChar*) GoExtTool_DisplayName(GoExtTool tool);

/** 
 * Retrieves the name of the tool.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @param    type       Receives the type of the custom tool.
 * @param    capacity   The maximum capacity of the name array.
 * @return              Operation status.            
 */
GoFx(kStatus) GoExtTool_Type(GoExtTool tool, kChar* type, kSize capacity);

/** 
 * Sets the data source. 
 *
 * @public               @memberof GoExtTool
 * @param    tool        GoExtTool object.
 * @param    source      GoDataSource object.
 * @return               Operation status.  
 * @see                  GoExtTool_IntensitySupportEnabled
 */
GoFx(kStatus) GoExtTool_SetSource(GoExtTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public               @memberof GoExtTool
 * @param    tool        GoExtTool object.
 * @return   The data source.            
 */
GoFx(GoDataSource) GoExtTool_Source(GoExtTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              The current tool data source option list count.            
 */
GoFx(kSize) GoExtTool_SourceOptionCount(GoExtTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The tool data source option at the given index, or k32U_MAX if an invalid index is given.            
 */
GoFx(k32u) GoExtTool_SourceOptionAt(GoExtTool tool, kSize index);

/** 
 * Returns a boolean value representing whether the tool supports anchoring.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              kTRUE if supported, kFALSE if not.
 */
GoFx(kBool) GoExtTool_XAnchorSupportEnabled(GoExtTool tool);

/** 
 * Gets the X-anchoring option list count.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoExtTool_XAnchorOptionCount(GoExtTool tool);

/** 
 * Gets the X-anchoring option at the given index.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The X-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoExtTool_XAnchorOptionAt(GoExtTool tool, kSize index);

/** 
 * Gets the current X-anchoring source.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @return              The X-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoExtTool_XAnchor(GoExtTool tool);

/** 
 * Sets the X-anchoring source.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @param    id         The measurement ID of a valid X-anchoring source.
 * @return              Operation status.
 * @see                 GoExtTool_AnchorSupportEnabled
 */
GoFx(kStatus) GoExtTool_SetXAnchor(GoExtTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 * @see                 GoExtTool_AnchorSupportEnabled
 */
GoFx(kBool) GoExtTool_XAnchorEnabled(GoExtTool tool);

/// @cond (Gocator_3x00)

/** 
 * Gets the Y-anchoring option list count.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @return              The Y-anchoring option list count.
 */
GoFx(kSize) GoExtTool_YAnchorOptionCount(GoExtTool tool);

/** 
 * Gets the Y-anchoring option at the given index.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The Y-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoExtTool_YAnchorOptionAt(GoExtTool tool, kSize index);

/** 
 * Gets the current Y-anchoring source.
 *
 * @public              @memberof GoExtTool
 *
 * @param    tool       GoExtTool object.
 * @return              The Y-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoExtTool_YAnchor(GoExtTool tool);

/** 
 * Sets the Y-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @param    id         The measurement ID of a valid Y-anchoring source.
 * @return              Operation status.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kStatus) GoExtTool_SetYAnchor(GoExtTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Y-anchoring source has been set for Y-anchoring.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kBool) GoExtTool_YAnchorEnabled(GoExtTool tool);

/// @endcond 

/// @cond (Gocator_2x00 || Gocator_3x00)

/** 
 * Gets the Z-anchoring option list count.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoExtTool_ZAnchorOptionCount(GoExtTool tool);

/** 
 * Gets the Z-anchoring option at the given index.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoExtTool_ZAnchorOptionAt(GoExtTool tool, kSize index);

/** 
 * Gets the current Z-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              The Z-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoExtTool_ZAnchor(GoExtTool tool);

/** 
 * Sets the Z-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @param    id         The measurement ID of a valid Z-anchoring source.
 * @return              Operation status.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kStatus) GoExtTool_SetZAnchor(GoExtTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
 *
 * @public              @memberof GoExtTool
 * @param    tool       GoExtTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kBool) GoExtTool_ZAnchorEnabled(GoExtTool tool);

/// @endcond

/** 
 * Retrieves the first found instance of a measurement for a given enumeration type.
 *
 * @public              @memberof GoExtTo
 * @param    tool       GoExtTool object.
 * @param    name       The name of the measurement to retrieve.
 * @return              A measurement object if one is found, otherwise kNULL.
 */
GoFx(GoMeasurement) GoExtTool_FindMeasurementByName(GoExtTool tool, const kChar* name);

GoFx(kSize) GoExtTool_ParameterCount(GoExtTool tool);
GoFx(GoExtParam) GoExtTool_ParameterAt(GoExtTool tool, kSize index);
GoFx(GoExtParam) GoExtTool_FindParameterById(GoExtTool tool, const kChar* label);

kEndHeader()
#include <GoSdk/Tools/GoExtTool.x.h>

#endif

///@endcond
