/// @cond (Gocator_2x00 || Gocator_3x00)
/** 
 * @file    GoSurfaceTools.h
 * @brief   Declares all surface tools and their related classes. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_SURFACE_TOOLS_H
#define GO_SURFACE_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <GoSdk/GoUtils.h>

kBeginHeader()

/**
 * @class   GoSurfaceTool
 * @extends GoTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base surface tool.
 */
typedef GoTool GoSurfaceTool; 

/** 
 * Sets the data source. 
 *
 * @public               @memberof GoSurfaceTool
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoSurfaceTool object.
 * @param    source      GoDataSource object.
 * @return               Operation status.            
 */
GoFx(kStatus) GoSurfaceTool_SetSource(GoSurfaceTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public               @memberof GoSurfaceTool
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoSurfaceTool object.
 * @return   The data source.            
 */
GoFx(GoDataSource) GoSurfaceTool_Source(GoSurfaceTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The current tool data source option list count.            
 */
GoFx(kSize) GoSurfaceTool_SourceOptionCount(GoSurfaceTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The tool data source option at the given index, or k32U_MAX if an invalid index is given.            
 */

GoFx(k32u) GoSurfaceTool_SourceOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the X-anchoring option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoSurfaceTool_XAnchorOptionCount(GoSurfaceTool tool);

/** 
 * Gets the X-anchoring option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The X-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoSurfaceTool_XAnchorOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the current X-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The X-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoSurfaceTool_XAnchor(GoSurfaceTool tool);

/** 
 * Sets the X-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    id         The measurement ID of a valid X-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceTool_SetXAnchor(GoSurfaceTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceTool_XAnchorEnabled(GoSurfaceTool tool);

/** 
 * Gets the Y-anchoring option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The Y-anchoring option list count.
 */
GoFx(kSize) GoSurfaceTool_YAnchorOptionCount(GoSurfaceTool tool);

/** 
 * Gets the Y-anchoring option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The Y-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoSurfaceTool_YAnchorOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the current Y-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The Y-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoSurfaceTool_YAnchor(GoSurfaceTool tool);

/** 
 * Sets the Y-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    id         The measurement ID of a valid Y-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceTool_SetYAnchor(GoSurfaceTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Y-anchoring source has been set for Y-anchoring.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceTool_YAnchorEnabled(GoSurfaceTool tool);

/** 
 * Gets the Z-anchoring option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoSurfaceTool_ZAnchorOptionCount(GoSurfaceTool tool);

/** 
 * Gets the Z-anchoring option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoSurfaceTool_ZAnchorOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the current Z-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The Z-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoSurfaceTool_ZAnchor(GoSurfaceTool tool);

/** 
 * Sets the Z-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    id         The measurement ID of a valid Z-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceTool_SetZAnchor(GoSurfaceTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
 *
 * @public              @memberof GoSurfaceTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceTool_ZAnchorEnabled(GoSurfaceTool tool);


/**
 * @class   GoSurfaceBox
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface bounding box tool.
 */
typedef GoSurfaceTool GoSurfaceBox; 

/** 
 * Returns the enabled state of Z-rotation.
 *
 * @public                  @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceBox_ZRotationEnabled(GoSurfaceBox tool);

/** 
 * Enables or disables Z-rotation.
 *
 * @public                  @memberof GoSurfaceBox
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @param    enable         kTRUE to enable Z-rotation, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceBox_EnableZRotation(GoSurfaceBox tool, kBool enable);

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceBox
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceBox_RegionEnabled(GoSurfaceBox tool);

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoSurfaceBox
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceBox_EnableRegion(GoSurfaceBox tool, kBool enable);

/** 
 * Gets the surface bounding box region.
 *
 * @public                  @memberof GoSurfaceBox
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceBox_Region(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox X measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox X measurement.
 */
GoFx(GoSurfaceBoxX) GoSurfaceBox_XMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Y measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Y measurement.
 */
GoFx(GoSurfaceBoxY) GoSurfaceBox_YMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Z measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Z measurement.
 */
GoFx(GoSurfaceBoxZ) GoSurfaceBox_ZMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Width measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Width measurement.
 */
GoFx(GoSurfaceBoxWidth) GoSurfaceBox_WidthMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Length measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Length measurement.
 */
GoFx(GoSurfaceBoxLength) GoSurfaceBox_LengthMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Height measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Height measurement.
 */
GoFx(GoSurfaceBoxHeight) GoSurfaceBox_HeightMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Z Angle measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Z Angle measurement.
 */
GoFx(GoSurfaceBoxZAngle) GoSurfaceBox_ZAngleMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox global X measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox global X measurement.
 */
GoFx(GoSurfaceBoxGlobalX) GoSurfaceBox_GlobalXMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox global Y measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox global Y measurement.
 */
GoFx(GoSurfaceBoxGlobalY) GoSurfaceBox_GlobalYMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox global Z Angle measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox global Z angle measurement.
 */
GoFx(GoSurfaceBoxGlobalZAngle) GoSurfaceBox_GlobalZAngleMeasurement(GoSurfaceBox tool);


/**
* @class   GoSurfaceCountersunkHole
* @extends GoSurfaceTool
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a Surface Counter Sunk Hole tool.
*/
typedef GoSurfaceTool GoSurfaceCountersunkHole; 

/**
 * Sets the nominal bevel angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalBevelAngle(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the nominal bevel angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal bevel angle.
 */
GoFx(k64f) GoSurfaceCountersunkHole_NominalBevelAngle(GoSurfaceCountersunkHole tool);

/**
 * Sets the bevel angle tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelAngleTolerance(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the bevel angle tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The bevel angle tolerance.
 */
GoFx(k64f) GoSurfaceCountersunkHole_BevelAngleTolerance(GoSurfaceCountersunkHole tool);

/**
 * Sets the nominal outer radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalOuterRadius(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the nominal outer radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal outer radius.
 */
GoFx(k64f) GoSurfaceCountersunkHole_NominalOuterRadius(GoSurfaceCountersunkHole tool);

/**
 * Sets the outer radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetOuterRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the outer radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The outer radius tolerance.
 */
GoFx(k64f) GoSurfaceCountersunkHole_OuterRadiusTolerance(GoSurfaceCountersunkHole tool);

/**
 * Sets the nominal inner radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalInnerRadius(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the nominal inner radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal inner radius.
 */
GoFx(k64f) GoSurfaceCountersunkHole_NominalInnerRadius(GoSurfaceCountersunkHole tool);

/**
 * Sets the inner radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetInnerRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the inner radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The inner radius tolerance.
 */
GoFx(k64f) GoSurfaceCountersunkHole_InnerRadiusTolerance(GoSurfaceCountersunkHole tool);

/**
 * Sets the bevel radius offset.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelRadiusOffset(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the bevel radius offset.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The bevel radius offset.
 */
GoFx(k64f) GoSurfaceCountersunkHole_BevelRadiusOffset(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables partial counter sunk hole detection.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable partial detection and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnablePartialDetection(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of partial detection.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if partial detection is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_PartialDetectionEnabled(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables the tool region.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable the tool region and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableRegion(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of the tool region.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if the tool region is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_RegionEnabled(GoSurfaceCountersunkHole tool);

/**
 * Returns the tool region.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The tool region.
 */
GoFx(GoRegion3d) GoSurfaceCountersunkHole_Region(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables reference regions.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable reference regions and kFALSE to disable them.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableRefRegions(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of the tool reference regions.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if refrence regions are enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_RefRegionsEnabled(GoSurfaceCountersunkHole tool);

/**
 * Sets the reference region count.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    count   The number of references regions to use when enabled.
 * @return           Reference region count.
 * @see              GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS

 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetRefRegionCount(GoSurfaceCountersunkHole tool, kSize count);

/**
 * Returns the reference region count.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Reference region count.
 */
GoFx(kSize) GoSurfaceCountersunkHole_RefRegionCount(GoSurfaceCountersunkHole tool);

/**
 * Returns the reference region at the given index.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    index   The index of the reference region to retrieve.
 * @return           A reference region.
 */
GoFx(GoSurfaceRegion2d) GoSurfaceCountersunkHole_RefRegionAt(GoSurfaceCountersunkHole tool, kSize index);

/**
 * Enables or disables automatic tilt.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable auto tilt and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableAutoTilt(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of auto tilt.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if auto tilt is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_AutoTiltEnabled(GoSurfaceCountersunkHole tool);

/**
 * Sets the tilt X angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltXAngle(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the tilt X angle value.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Tilt X angle value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_TiltXAngle(GoSurfaceCountersunkHole tool);

/**
 * Sets the tilt Y angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltYAngle(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the tilt Y angle value.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Tilt Y angle value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_TiltYAngle(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables curve fitting.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable curve fitting and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableCurveFit(GoSurfaceCountersunkHole tool, kBool enable);


/**
 * Returns the state of auto tilt.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if curve fitting is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_CurveFitEnabled(GoSurfaceCountersunkHole tool);

/**
 * Sets the curve orientation angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetCurveOrientation(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the curve orientation angle value.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Curve orientation angle value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_CurveOrientation(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool X position measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole X position measurement.
 */
GoFx(GoSurfaceCountersunkHoleX) GoSurfaceCountersunkHole_XMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Y position measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Y position measurement.
 */
GoFx(GoSurfaceCountersunkHoleY) GoSurfaceCountersunkHole_YMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Z position measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Z position measurement.
 */
GoFx(GoSurfaceCountersunkHoleZ) GoSurfaceCountersunkHole_ZMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Outer Radius measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Outer Radius measurement.
 */
GoFx(GoSurfaceCountersunkHoleOuterRadius) GoSurfaceCountersunkHole_OuterRadiusMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Depth measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Depth measurement.
 */
GoFx(GoSurfaceCountersunkHoleDepth) GoSurfaceCountersunkHole_DepthMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Bevel Radius measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Bevel Radius measurement.
 */
GoFx(GoSurfaceCountersunkHoleBevelRadius) GoSurfaceCountersunkHole_BevelRadiusMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Bevel Angle measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Bevel Angle measurement.
 */
GoFx(GoSurfaceCountersunkHoleBevelAngle) GoSurfaceCountersunkHole_BevelAngleMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool X Angle measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole X angle measurement.
 */
GoFx(GoSurfaceCountersunkHoleXAngle) GoSurfaceCountersunkHole_XAngleMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Y Angle measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Y angle measurement.
 */
GoFx(GoSurfaceCountersunkHoleYAngle) GoSurfaceCountersunkHole_YAngleMeasurement(GoSurfaceCountersunkHole tool);


/**
 * @class   GoSurfaceEllipse
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface ellipse tool.
 */
typedef GoSurfaceTool GoSurfaceEllipse; 

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceEllipse object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceEllipse_EnableRegion(GoSurfaceEllipse tool, kBool enable);

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceEllipse object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceEllipse_RegionEnabled(GoSurfaceEllipse tool);

/** 
 * Gets the tool region.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceEllipse object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceEllipse_Region(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Major measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Major measurement.
 */
GoFx(GoSurfaceEllipseMajor) GoSurfaceEllipse_MajorMeasurement(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Minor measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Minor measurement.
 */
GoFx(GoSurfaceEllipseMinor) GoSurfaceEllipse_MinorMeasurement(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Ratio measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Ratio measurement.
 */
GoFx(GoSurfaceEllipseRatio) GoSurfaceEllipse_RatioMeasurement(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Z Angle measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Z Angle measurement.
 */
GoFx(GoSurfaceEllipseZAngle) GoSurfaceEllipse_ZAngleMeasurement(GoSurfaceEllipse tool);


/**
 * @class   GoSurfaceHole
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface hole tool.
 */
typedef GoSurfaceTool GoSurfaceHole; 

/** 
 * Gets the current nominal radius value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  The nominal radius value.
 */
GoFx(k64f) GoSurfaceHole_NominalRadius(GoSurfaceHole tool);

/** 
 * Sets the nominal radius value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    nominalRadius  Nominal radius value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetNominalRadius(GoSurfaceHole tool, k64f nominalRadius);

/** 
 * Gets the current radius tolerance value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  The radius tolerance value.
 */
GoFx(k64f) GoSurfaceHole_RadiusTolerance(GoSurfaceHole tool);

/** 
 * Sets the radius tolerance value.
 *
 * @public                      @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool               GoSurfaceHole object.
 * @param    radiusTolerance    The radius tolerance value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetRadiusTolerance(GoSurfaceHole tool, k64f radiusTolerance);

/** 
 * Gets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_PartialDetectionEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable partial detection and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnablePartialDetection(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_RegionEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableRegion(GoSurfaceHole tool, kBool enable);

/** 
 * Returns the tool's region object.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceHole_Region(GoSurfaceHole tool);

/** 
 * Gets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_RefRegionsEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable reference regions and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableRefRegions(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the reference region count.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  The reference region count.
 */
GoFx(kSize) GoSurfaceHole_RefRegionCount(GoSurfaceHole tool);

/** 
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    count          The reference region count.
 * @return                  Operation status.
 * @see                     GO_SURFACE_HOLE_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceHole_SetRefRegionCount(GoSurfaceHole tool, kSize count);

/** 
 * Gets a reference region object at the given index.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    index          The index with which to retrieve a reference region.
 * @return                  A GoSurfaceRegion2d object.
 * @see                     GoSurfaceHole_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceHole_RefRegionAt(GoSurfaceHole tool, kSize index);

/** 
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_AutoTiltEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable it and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableAutoTilt(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  Tilt X-angle value.
 */
GoFx(k64f) GoSurfaceHole_TiltXAngle(GoSurfaceHole tool);

/** 
 * Sets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    value          The tilt X-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetTiltXAngle(GoSurfaceHole tool, k64f value);

/** 
 * Gets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  Tilt Y-angle value.
 */
GoFx(k64f) GoSurfaceHole_TiltYAngle(GoSurfaceHole tool);

/** 
 * Sets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    value          The tilt Y-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetTiltYAngle(GoSurfaceHole tool, k64f value);

/**
 * Returns a GoSurfaceHole X measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHole X measurement.
 */
GoFx(GoSurfaceHoleX) GoSurfaceHole_XMeasurement(GoSurfaceHole tool);

/**
 * Returns a GoSurfaceHole Y measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHole Y measurement.
 */
GoFx(GoSurfaceHoleY) GoSurfaceHole_YMeasurement(GoSurfaceHole tool);

/**
 * Returns a GoSurfaceHole Z measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHole Z measurement.
 */
GoFx(GoSurfaceHoleZ) GoSurfaceHole_ZMeasurement(GoSurfaceHole tool);

/**
 * Returns a GoSurfaceHole Radius measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHole Radius measurement.
 */
GoFx(GoSurfaceHoleRadius) GoSurfaceHole_RadiusMeasurement(GoSurfaceHole tool);


/**
 * @class   GoSurfaceOpening
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface opening tool.
 */
typedef GoSurfaceTool GoSurfaceOpening; 

GoFx(kStatus) GoSurfaceOpening_SetType(GoSurfaceOpening tool, GoSurfaceOpeningType type);

/** 
 * Gets the surface opening type.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The surface opening type.
 */
GoFx(GoSurfaceOpeningType) GoSurfaceOpening_Type(GoSurfaceOpening tool);

/** 
 * Gets the nominal width.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal width (in mm).
 */
GoFx(k64f) GoSurfaceOpening_NominalWidth(GoSurfaceOpening tool);

/** 
 * Sets the nominal width.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal width to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalWidth(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the nominal length.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal length (in mm).
 */
GoFx(k64f) GoSurfaceOpening_NominalLength(GoSurfaceOpening tool);

/** 
 * Sets the nominal length.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal length to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalLength(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the nominal angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal angle (in degrees).
 */
GoFx(k64f) GoSurfaceOpening_NominalAngle(GoSurfaceOpening tool);

/** 
 * Sets the nominal angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal angle to set (in degrees).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalAngle(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the nominal radius.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal radius (in mm).
 */
GoFx(k64f) GoSurfaceOpening_NominalRadius(GoSurfaceOpening tool);

/** 
 * Sets the nominal radius.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal radius to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalRadius(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the width tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The width tolerance (in mm).
 */
GoFx(k64f) GoSurfaceOpening_WidthTolerance(GoSurfaceOpening tool);

/** 
 * Sets the width tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The width tolerance to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetWidthTolerance(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the length tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The length tolerance (in mm).
 */
GoFx(k64f) GoSurfaceOpening_LengthTolerance(GoSurfaceOpening tool);

/** 
 * Sets the length tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The length tolerance to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetLengthTolerance(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the angle tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The angle tolerance (in degrees).
 */
GoFx(k64f) GoSurfaceOpening_AngleTolerance(GoSurfaceOpening tool);

/** 
 * Sets the angle tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The angle tolerance to set (in degrees).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetAngleTolerance(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if partial detection is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_PartialDetectionEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable partial detection and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnablePartialDetection(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if the tool region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_RegionEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableRegion(GoSurfaceOpening tool, kBool enable);

/** 
 * Returns the region object for the tool.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceOpening_Region(GoSurfaceOpening tool);

/** 
 * Gets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if reference regions are enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_RefRegionsEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable reference regions and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableRefRegions(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the reference region count.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The count of reference regions.
 */
GoFx(kSize) GoSurfaceOpening_RefRegionCount(GoSurfaceOpening tool);

/** 
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    count          The reference region count to set.
 * @return                  Operation status.
 * @see                     GO_SURFACE_OPENING_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceOpening_SetRefRegionCount(GoSurfaceOpening tool, kSize count);

/** 
 * Gets the reference region object at the specified index.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    index          The index with which to retrieve a reference region.
 * @return                  A GoSurfaceRegion2d object or kNULL if the index is invalid.
 * @see                     GoSurfaceOpening_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceOpening_RefRegionAt(GoSurfaceOpening tool, kSize index);

/** 
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if auto-tilt is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_AutoTiltEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable auto-tilt and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableAutoTilt(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the tilt X-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The tilt X-angle.
 */
GoFx(k64f) GoSurfaceOpening_TiltXAngle(GoSurfaceOpening tool);

/** 
 * Sets the tilt X-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The tilt X-angle to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetTiltXAngle(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the tilt Y-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The tilt Y-angle.
 */
GoFx(k64f) GoSurfaceOpening_TiltYAngle(GoSurfaceOpening tool);

/** 
 * Sets the tilt Y-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The tilt Y-angle to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetTiltYAngle(GoSurfaceOpening tool, k64f value);

/**
 * Returns a GoSurfaceOpening X measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening X measurement.
 */
GoFx(GoSurfaceOpeningX) GoSurfaceOpening_XMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Y measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening Y measurement.
 */
GoFx(GoSurfaceOpeningY) GoSurfaceOpening_YMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Z measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening Z measurement.
 */
GoFx(GoSurfaceOpeningZ) GoSurfaceOpening_ZMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Width measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening Width measurement.
 */
GoFx(GoSurfaceOpeningWidth) GoSurfaceOpening_WidthMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Length measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening Length measurement.
 */
GoFx(GoSurfaceOpeningLength) GoSurfaceOpening_LengthMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Angle measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening Angle measurement.
 */
GoFx(GoSurfaceOpeningAngle) GoSurfaceOpening_AngleMeasurement(GoSurfaceOpening tool);


/**
 * @class   GoSurfacePlane
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface plane tool.
 */
typedef GoSurfaceTool GoSurfacePlane; 

/** 
 * Gets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfacePlane
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @return                  kTRUE if the tool region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfacePlane_RegionsEnabled(GoSurfacePlane tool);

/** 
 * Sets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfacePlane
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @param    enable         kTRUE to enable regions and kFALSE to disable them.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfacePlane_EnableRegions(GoSurfacePlane tool, kBool enable);

/** 
 * Gets the tool's region count.
 *
 * @public                  @memberof GoSurfacePlane
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @return                  The number of regions in the tool.
 */
GoFx(kSize) GoSurfacePlane_RegionCount(GoSurfacePlane tool);

/** 
 * Sets the tool region count.
 *
 * @public                  @memberof GoSurfacePlane
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @param    count          The region count to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfacePlane_SetRegionCount(GoSurfacePlane tool, kSize count);

/** 
 * Gets a region at the specified index.
 *
 * @public                  @memberof GoSurfacePlane
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @param    index          The index with which to return a tool region.
 * @return                  A GoRegion3d object or kNULL if the index is in invalid.
 * @see                     GoSurfacePlane_RegionCount
 */
GoFx(GoRegion3d) GoSurfacePlane_RegionAt(GoSurfacePlane tool, kSize index);

/**
 * Returns a GoSurfacePlane X Angle measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlane X Angle measurement.
 */
GoFx(GoSurfacePlaneXAngle) GoSurfacePlane_XAngleMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Y Angle measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlane Y Angle measurement.
 */
GoFx(GoSurfacePlaneYAngle) GoSurfacePlane_YAngleMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Z Offset measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlane Z Offset measurement.
 */
GoFx(GoSurfacePlaneZOffset) GoSurfacePlane_ZOffsetMeasurement(GoSurfacePlane tool);


/**
 * @class   GoSurfacePosition
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface position tool.
 */
typedef GoSurfaceTool GoSurfacePosition; 

GoFx(GoSurfaceFeature) GoSurfacePosition_Feature(GoSurfacePosition tool);

/**
 * Returns a GoSurfacePosition X measurement object.
 *
 * @public           @memberof GoSurfacePosition
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePosition object.
 * @return           A GoSurfacePosition X measurement.
 */
GoFx(GoSurfacePositionX) GoSurfacePosition_XMeasurement(GoSurfacePosition tool);

/**
 * Returns a GoSurfacePosition Y measurement object.
 *
 * @public           @memberof GoSurfacePosition
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePosition object.
 * @return           A GoSurfacePosition Y measurement.
 */
GoFx(GoSurfacePositionY) GoSurfacePosition_YMeasurement(GoSurfacePosition tool);

/**
 * Returns a GoSurfacePosition Z measurement object.
 *
 * @public           @memberof GoSurfacePosition
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePosition object.
 * @return           A GoSurfacePosition Z measurement.
 */
GoFx(GoSurfacePositionZ) GoSurfacePosition_ZMeasurement(GoSurfacePosition tool);


/**
 * @class   GoSurfaceStud
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface stud tool.
 */
typedef GoSurfaceTool GoSurfaceStud; 

/** 
 * Returns the stud radius value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud radius value.
 */
GoFx(k64f) GoSurfaceStud_StudRadius(GoSurfaceStud tool);

/** 
 * Sets the stud radius value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The stud radius value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetStudRadius(GoSurfaceStud tool, k64f value);

/** 
 * Returns the stud height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud height value.
 */
GoFx(k64f) GoSurfaceStud_StudHeight(GoSurfaceStud tool);

/** 
 * Sets the stud height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The stud height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetStudHeight(GoSurfaceStud tool, k64f value);

/** 
 * Returns the stud base height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud base height value.
 */
GoFx(k64f) GoSurfaceStud_BaseHeight(GoSurfaceStud tool);

/** 
 * Sets the base height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The base height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetBaseHeight(GoSurfaceStud tool, k64f value);

/** 
 * Returns the stud tip height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud tip height value.
 */
GoFx(k64f) GoSurfaceStud_TipHeight(GoSurfaceStud tool);

/** 
 * Sets the tip height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The tip height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetTipHeight(GoSurfaceStud tool, k64f value);

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceStud_RegionEnabled(GoSurfaceStud tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    enable         kTRUE to enable the region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_EnableRegion(GoSurfaceStud tool, kBool enable);

/** 
 * Returns the tool region object.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceStud_Region(GoSurfaceStud tool);

/** 
 * Gets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return   enable         kTRUE if the reference regions are enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceStud_RefRegionsEnabled(GoSurfaceStud tool);

/** 
 * Sets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    enable         kTRUE to enable the region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_EnableRefRegions(GoSurfaceStud tool, kBool enable);

/** 
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    count          The number of reference regions to use.
 * @return                  Operation status.
 * @see                     GO_SURFACE_STUD_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceStud_SetRefRegionCount(GoSurfaceStud tool, kSize count);

/** 
 * Returns the reference region count.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The reference region count.
 */
GoFx(kSize) GoSurfaceStud_RefRegionCount(GoSurfaceStud tool);

/** 
 * Returns the reference region object at the given index.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    index          The index with which to return a reference region.
 * @return                  A GoSurfaceRegion2d object.
 * @see                     GoSurfaceStud_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceStud_RefRegionAt(GoSurfaceStud tool, kSize index);

/** 
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceStud_AutoTiltEnabled(GoSurfaceStud tool);

/** 
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    enable         kTRUE to enable the region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_EnableAutoTilt(GoSurfaceStud tool, kBool enable);

/** 
 * Gets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The tilt X-angle value.
 */
GoFx(k64f) GoSurfaceStud_TiltXAngle(GoSurfaceStud tool);

/** 
 * Sets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The tilt X-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetTiltXAngle(GoSurfaceStud tool, k64f value);

/** 
 * Returns the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The tilt Y-angle value.
 */
GoFx(k64f) GoSurfaceStud_TiltYAngle(GoSurfaceStud tool);

/** 
 * Sets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The tilt Y-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetTiltYAngle(GoSurfaceStud tool, k64f value);

/**
 * Returns a GoSurfaceStud Base X measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Base X measurement.
 */
GoFx(GoSurfaceStudBaseX) GoSurfaceStud_BaseXMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Base Y measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Base Y measurement.
 */
GoFx(GoSurfaceStudBaseY) GoSurfaceStud_BaseYMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Base Z measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Base Z measurement.
 */
GoFx(GoSurfaceStudBaseZ) GoSurfaceStud_BaseZMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Tip X measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Tip X measurement.
 */
GoFx(GoSurfaceStudTipX) GoSurfaceStud_TipXMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Tip Y measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Tip Y measurement.
 */
GoFx(GoSurfaceStudTipY) GoSurfaceStud_TipYMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Tip Z measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Tip Z measurement.
 */
GoFx(GoSurfaceStudTipZ) GoSurfaceStud_TipZMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Radius measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Radius measurement.
 */
GoFx(GoSurfaceStudRadius) GoSurfaceStud_RadiusMeasurement(GoSurfaceStud tool);


/**
 * @class   GoSurfaceVolume
 * @extends GoSurfaceTool
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface volume tool.
 */
typedef GoSurfaceTool GoSurfaceVolume; 

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceVolume
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceVolume object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceVolume_RegionEnabled(GoSurfaceVolume tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceVolume
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceVolume object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 */
GoFx(kStatus) GoSurfaceVolume_EnableRegion(GoSurfaceVolume tool, kBool enable);

/** 
 * Returns the tool region object.
 *
 * @public                  @memberof GoSurfaceVolume
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceVolume object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceVolume_Region(GoSurfaceVolume tool);

/**
 * Returns a GoSurfaceVolume Volume measurement object.
 *
 * @public           @memberof GoSurfaceVolume
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceVolume object.
 * @return           A GoSurfaceVolume Volume measurement.
 */
GoFx(GoSurfaceVolumeVolume) GoSurfaceVolume_VolumeMeasurement(GoSurfaceVolume tool);

/**
 * Returns a GoSurfaceVolume Area measurement object.
 *
 * @public           @memberof GoSurfaceVolume
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceVolume object.
 * @return           A GoSurfaceVolume Area measurement.
 */
GoFx(GoSurfaceVolumeArea) GoSurfaceVolume_AreaMeasurement(GoSurfaceVolume tool);

/**
 * Returns a GoSurfaceVolume Thickness measurement object.
 *
 * @public           @memberof GoSurfaceVolume
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceVolume object.
 * @return           A GoSurfaceVolume Thickness measurement.
 */
GoFx(GoSurfaceVolumeThickness) GoSurfaceVolume_ThicknessMeasurement(GoSurfaceVolume tool);


kEndHeader()
#include <GoSdk/Tools/GoSurfaceTools.x.h>

#endif

/// @endcond
