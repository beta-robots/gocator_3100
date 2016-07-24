/// @cond (Gocator_1x00 || Gocator_2x00)

/** 
 * @file    GoProfileTools.h
 * @brief   Declares all profile tools and their related classes. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOLS_H
#define GO_PROFILE_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdk/GoUtils.h>

kBeginHeader()

/**
 * @class   GoProfileTool
 * @extends GoTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a base profile tool.
 */
typedef GoTool GoProfileTool; 

/** 
 * Sets the data source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    source     GoDataSource object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoProfileTool_SetSource(GoProfileTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The current profile tool data source.            
 */
GoFx(GoDataSource) GoProfileTool_Source(GoProfileTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The current profile tool data source option list count.            
 */
GoFx(kSize) GoProfileTool_SourceOptionCount(GoProfileTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The profile tool data source option at the given index, or k32U_MAX if an invalid index is given.            
 */
GoFx(GoDataSource) GoProfileTool_SourceOptionAt(GoProfileTool tool, kSize index);

/** 
 * Gets the X-anchoring option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoProfileTool_XAnchorOptionCount(GoProfileTool tool);

/** 
 * Gets the X-anchoring option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The X-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoProfileTool_XAnchorOptionAt(GoProfileTool tool, kSize index);

/** 
 * Gets the current X-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The X-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoProfileTool_XAnchor(GoProfileTool tool);

/** 
 * Sets the X-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    id         The measurement ID of a valid X-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoProfileTool_SetXAnchor(GoProfileTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoProfileTool_XAnchorEnabled(GoProfileTool tool);

/** 
 * Gets the Z-anchoring option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoProfileTool_ZAnchorOptionCount(GoProfileTool tool);

/** 
 * Gets the Z-anchoring option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoProfileTool_ZAnchorOptionAt(GoProfileTool tool, kSize index);

/** 
 * Gets the current Z-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The Z-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoProfileTool_ZAnchor(GoProfileTool tool);

/** 
 * Sets the Z-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    id         The measurement ID of a valid Z-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoProfileTool_SetZAnchor(GoProfileTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoProfileTool_ZAnchorEnabled(GoProfileTool tool);


/**
 * @class   GoProfileArea
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile area tool.
 */
typedef GoProfileTool GoProfileArea; 

/** 
 * Gets the profile area baseline.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The profile area baseline.            
 */
GoFx(GoProfileBaseline) GoProfileArea_Baseline(GoProfileArea tool);

/** 
 * Sets the profile area type.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @param    type        The baseline type to set.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileArea_SetBaseline(GoProfileArea tool, GoProfileBaseline type);

/** 
 * Returns a boolean representing whether the profile area baseline is used.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               kTRUE if the baseline is used; kFALSE otherwise.
 */
GoFx(kBool) GoProfileArea_BaselineUsed(GoProfileArea tool);

/** 
 * Gets the reference profile line.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The reference profile line.            
 */
GoFx(GoProfileLineRegion) GoProfileArea_LineRegion(GoProfileArea tool);

/** 
 * Gets the profile area type.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The profile area type.            
 */
GoFx(GoProfileAreaType) GoProfileArea_Type(GoProfileArea tool);

/** 
 * Gets the boolean representing whether the area type is used.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               kTRUE if profile area type is used; kFALSE otherwise.
 */
GoFx(kBool) GoProfileArea_TypeUsed(GoProfileArea tool);

/** 
 * Sets the profile area type.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @param    type        GoProfileAreaType object.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileArea_SetType(GoProfileArea tool, GoProfileAreaType type);

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileArea_Region(GoProfileArea tool);

/**
 * Returns a GoProfileArea Area measurement object.
 *
 * @public           @memberof GoProfileArea
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileArea object.
 * @return           A GoProfileAreaArea measurement.
 */
GoFx(GoProfileAreaArea) GoProfileArea_AreaMeasurement(GoProfileArea tool);

/**
 * Returns a GoProfileArea Centroid X measurement object.
 *
 * @public           @memberof GoProfileArea
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileArea object.
 * @return           A GoProfileAreaCentroidX measurement.
 */
GoFx(GoProfileAreaCentroidX) GoProfileArea_CentroidXMeasurement(GoProfileArea tool);

/**
 * Returns a GoProfileArea Centroid Z measurement object.
 *
 * @public           @memberof GoProfileArea
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileArea object.
 * @return           A GoProfileAreaCentroidZ measurement.
 */
GoFx(GoProfileAreaCentroidZ) GoProfileArea_CentroidZMeasurement(GoProfileArea tool);


/**
 * @class   GoProfileBox
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile bounding box tool.
 */
typedef GoProfileTool GoProfileBox; 

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool           GoProfileBox object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoProfileBox_RegionEnabled(GoProfileBox tool);

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool           GoProfileBox object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBox_EnableRegion(GoProfileBox tool, kBool enable);

/** 
 * Gets the profile bounding box region.
 *
 * @public                  @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool           GoProfileBox object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoProfileRegion) GoProfileBox_Region(GoProfileBox tool);

/**
 * Returns a GoProfileBox X measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox X measurement.
 */
GoFx(GoProfileBoxX) GoProfileBox_XMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox Z measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox Z measurement.
 */
GoFx(GoProfileBoxZ) GoProfileBox_ZMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox Width measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox Width measurement.
 */
GoFx(GoProfileBoxWidth) GoProfileBox_WidthMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox Height measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox Height measurement.
 */
GoFx(GoProfileBoxHeight) GoProfileBox_HeightMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox global X measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox global X measurement.
 */
GoFx(GoProfileBoxGlobalX) GoProfileBox_GlobalXMeasurement(GoProfileBox tool);


/**
 * @class   GoProfileCircle
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile circle tool.
 */
typedef GoProfileTool GoProfileCircle; 

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileCircle_Region(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle X measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleX measurement.
 */
GoFx(GoProfileCircleX) GoProfileCircle_XMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Z measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleZ measurement.
 */
GoFx(GoProfileCircleZ) GoProfileCircle_ZMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Radius measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircle Radius measurement.
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_RadiusMeasurement(GoProfileCircle tool);


/**
 * @class   GoProfileDim
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile dimension tool.
 */
typedef GoProfileTool GoProfileDim; 


/** 
 * Gets the reference profile feature.
 *
 * @public               @memberof GoProfileDim
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileDim object.
 * @return               The reference profile feature object.
 */
GoFx(GoProfileFeature) GoProfileDim_RefFeature(GoProfileDim tool);

/** 
 * Gets the non-reference profile feature.
 *
 * @public               @memberof GoProfileDim
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileDim object.
 * @return               The profile feature object.
 */
GoFx(GoProfileFeature) GoProfileDim_Feature(GoProfileDim tool);

/**
 * Returns a GoProfileDim Width measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimWidth measurement.
 */
GoFx(GoProfileDimWidth) GoProfileDim_WidthMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Height measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimHeight measurement.
 */
GoFx(GoProfileDimHeight) GoProfileDim_HeightMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Distance measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimDistance measurement.
 */
GoFx(GoProfileDimDistance) GoProfileDim_DistanceMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Center X measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimCenterX measurement.
 */
GoFx(GoProfileDimCenterX) GoProfileDim_CenterXMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Center Z measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimCenterZ measurement.
 */
GoFx(GoProfileDimCenterZ) GoProfileDim_CenterZMeasurement(GoProfileDim tool);


/**
 * @class   GoProfileGroove
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile groove tool.
 */
typedef GoProfileTool GoProfileGroove; 

/** 
 * Adds an additional profile groove tool measurement.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @param    type           The measurement type to add. It must be a valid profile groove tool type.
 * @param    measurement    A reference to the new GoMeasurement handle. Can be kNULL if you do not wish to do anything immediate with the new measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGroove_AddMeasurement(GoProfileGroove tool, GoMeasurementType type, GoMeasurement* measurement);

/** 
 * Removes a measurement from the tool at the given index.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @param    index          The index with which to remove a measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGroove_RemoveMeasurement(GoProfileGroove tool, kSize index);

/** 
 * Returns the measurement count for the given tool.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @return                  Tool measurement count.
 */
GoFx(kSize) GoProfileGroove_MeasurementCount(GoProfileGroove tool);

/** 
 * Returns a measurement object at the given index.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @param    index          The index with which to return a measurement object.
 * @return                  A profile groove tool measurement or kNULL if the index is invalid.
 */
GoFx(GoMeasurement) GoProfileGroove_MeasurementAt(GoProfileGroove tool, kSize index);

/** 
 * Gets the current groove determination shape.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The profile groove shape.            
 */
GoFx(GoProfileGrooveShape) GoProfileGroove_Shape(GoProfileGroove tool);

/** 
 * Sets the groove determination shape.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    shape       The intended profile groove shape.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetShape(GoProfileGroove tool, GoProfileGrooveShape shape);

/** 
 * Gets the groove depth minimum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The groove depth minimum value.            
 */
GoFx(k64f) GoProfileGroove_MinDepth(GoProfileGroove tool);

/** 
 * Sets the groove depth minimum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    depth       The minimum groove depth value to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetMinDepth(GoProfileGroove tool, k64f depth);

/** 
 * Gets the groove width maximum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The groove width maximum value.            
 */
GoFx(k64f) GoProfileGroove_MaxWidth(GoProfileGroove tool);

/** 
 * Sets the groove width maximum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    width       The maximum groove width value to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetMaxWidth(GoProfileGroove tool, k64f width);

/** 
 * Gets the groove width minimum value.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The groove width minimum.            
 */
GoFx(k64f) GoProfileGroove_MinWidth(GoProfileGroove tool);

/** 
 * Sets the groove width minimum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    width       The minimum groove width value to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetMinWidth(GoProfileGroove tool, k64f width);

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileGroove_Region(GoProfileGroove tool);


/**
 * @class   GoProfileIntersect
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile intersect tool.
 */
typedef GoProfileTool GoProfileIntersect; 


/** 
 * Gets the reference profile line type.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return   The profile line type.            
 */
GoFx(GoProfileBaseline) GoProfileIntersect_RefLineType(GoProfileIntersect tool);

/** 
 * Sets the reference line type.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @param    type        The line type to set.
 * @return               
 */
GoFx(kStatus) GoProfileIntersect_SetRefLineType(GoProfileIntersect tool, GoProfileBaseline type);

/** 
 * Gets the reference profile line.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return               The reference profile line.
 */
GoFx(GoProfileLineRegion) GoProfileIntersect_RefLine(GoProfileIntersect tool);

/** 
 * Gets the non-reference profile line.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return               The non-reference profile line.            
 */
GoFx(GoProfileLineRegion) GoProfileIntersect_Line(GoProfileIntersect tool);

/**
 * Returns a GoProfileIntersect X measurement object.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return               A GoProfileIntersect X measurement.
 */
GoFx(GoProfileIntersectX) GoProfileIntersect_XMeasurement(GoProfileIntersect tool);

/**
 * Returns a GoProfileIntersect Z measurement object.
 *
 * @public              @memberof GoProfileIntersect
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileIntersect object.
 * @return              A GoProfileIntersect Z measurement.
 */
GoFx(GoProfileIntersectZ) GoProfileIntersect_ZMeasurement(GoProfileIntersect tool);

/**
 * Returns a GoProfileIntersect Angle measurement object.
 *
 * @public              @memberof GoProfileIntersect
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileIntersect object.
 * @return              A GoProfileIntersect Angle measurement.
 */
GoFx(GoProfileIntersectAngle) GoProfileIntersect_AngleMeasurement(GoProfileIntersect tool);


/**
 * @class   GoProfileLine
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile line tool.
 */
typedef GoProfileTool GoProfileLine; 

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileDev
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileDev object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileLine_Region(GoProfileLine tool);

/**
 * Returns a GoProfileLine Standard Deviation measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Standard Deviation measurement.
 */
GoFx(GoProfileLineStdDev) GoProfileLine_StdDevMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Maximum Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Maximum Error measurement.
 */
GoFx(GoProfileLineMaxError) GoProfileLine_MaxErrorMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Minimum Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Minimum Error measurement.
 */
GoFx(GoProfileLineMinError) GoProfileLine_MinErrorMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Percentile measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Percentile measurement.
 */
GoFx(GoProfileLinePercentile) GoProfileLine_PercentileMeasurement(GoProfileLine tool);


/**
 * @class   GoProfilePanel
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile panel tool.
 */
typedef GoProfileTool GoProfilePanel; 

/** 
 * Gets the maximum gap width.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The maximum gap width.            
 */
GoFx(k64f) GoProfilePanel_MaxGapWidth(GoProfilePanel tool);

/** 
 * Sets the maximum gap width.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @param    width       The maximum gap width value to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfilePanel_SetMaxGapWidth(GoProfilePanel tool, k64f width);

/** 
 * Gets the reference edge side.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The reference edge side.            
 */
GoFx(GoProfilePanelSide) GoProfilePanel_RefEdgeSide(GoProfilePanel tool);

/** 
 * Sets the reference edge side.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @param    side        The reference edge side.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfilePanel_SetRefEdgeSide(GoProfilePanel tool, GoProfilePanelSide side);

/** 
 * Gets the left profile edge.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The left profile edge.            
 */
GoFx(GoProfileEdge) GoProfilePanel_LeftEdge(GoProfilePanel tool);

/** 
 * Gets the right profile edge.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The right profile edge.            
 */
GoFx(GoProfileEdge) GoProfilePanel_RightEdge(GoProfilePanel tool);

/**
 * Returns a GoProfilePanel Gap measurement object.
 *
 * @public           @memberof GoProfilePanel
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfilePanel object.
 * @return           A GoProfilePanel Gap measurement.
 */
GoFx(GoProfilePanelGap) GoProfilePanel_GapMeasurement(GoProfilePanel tool);

/**
 * Returns a GoProfilePanel Flush measurement object.
 *
 * @public           @memberof GoProfilePanel
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfilePanel object.
 * @return           A GoProfilePanel Flush measurement.
 */
GoFx(GoProfilePanelFlush) GoProfilePanel_FlushMeasurement(GoProfilePanel tool);


/**
 * @class   GoProfilePosition
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile position tool.
 */
typedef GoProfileTool GoProfilePosition; 

/** 
 * Gets the profile feature.
 *
 * @public               @memberof GoProfilePosition
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePos object.
 * @return               The profile feature.            
 */
GoFx(GoProfileFeature) GoProfilePosition_Feature(GoProfilePosition tool);

/**
 * Returns a GoProfilePosition X measurement object.
 *
 * @public              @memberof GoProfilePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfilePosition object.
 * @return              A GoProfilePosition X measurement.
 */
GoFx(GoProfilePositionX) GoProfilePosition_XMeasurement(GoProfilePosition tool);

/**
 * Returns a GoProfilePosition Z measurement object.
 *
 * @public              @memberof GoProfilePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfilePosition object.
 * @return              A GoProfilePosition Z measurement.
 */
GoFx(GoProfilePositionZ) GoProfilePosition_ZMeasurement(GoProfilePosition tool);


/**
 * @class   GoProfileStrip
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile strip tool.
 */
typedef GoProfileTool GoProfileStrip; 

/** 
 * Sets the strip base type.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    type           The strip base type.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetBaseType(GoProfileStrip tool, GoProfileStripBaseType type);

/** 
 * Gets the strip base type.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The strip base type.
 */
GoFx(GoProfileStripBaseType) GoProfileStrip_BaseType(GoProfileStrip tool);

/** 
 * Gets the left edge value.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The left edge value.
 */
GoFx(k8u) GoProfileStrip_LeftEdge(GoProfileStrip tool);

/** 
 * Sets the left edge.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    leftEdge       Left edge value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetLeftEdge(GoProfileStrip tool, k8u leftEdge);

/** 
 * Gets the right edge value.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The right edge value.
 */
GoFx(k8u) GoProfileStrip_RightEdge(GoProfileStrip tool);

/** 
 * Sets the right edge.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    rightEdge      Right edge value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetRightEdge(GoProfileStrip tool, k8u rightEdge);

/** 
 * Gets the tilt enabled state.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  kTRUE if tilt is enabled, kFALSE otherwise.
 */
GoFx(kBool) GoProfileStrip_TiltEnabled(GoProfileStrip tool);

/** 
 * Enables or disables tilt.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    enable         kTRUE to enable tilt, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_EnableTilt(GoProfileStrip tool, kBool enable);

/** 
 * Gets the transition width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The transition width (in mm).
 */
GoFx(k64f) GoProfileStrip_TransitionWidth(GoProfileStrip tool);

/** 
 * Sets the transition width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The transition width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetTransitionWidth(GoProfileStrip tool, k64f value);

/** 
 * Gets the minimum width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The minimum width (in mm).
 */
GoFx(k64f) GoProfileStrip_MinWidth(GoProfileStrip tool);

/** 
 * Sets the minimum width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The minimum width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetMinWidth(GoProfileStrip tool, k64f value);

/** 
 * Gets the minimum height.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The minimum height (in mm).
 */
GoFx(k64f) GoProfileStrip_MinHeight(GoProfileStrip tool);

/** 
 * Sets the transition height.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The minimum height (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetMinHeight(GoProfileStrip tool, k64f value);

/** 
 * Gets the maximum void width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The maximum void width (in mm).
 */
GoFx(k64f) GoProfileStrip_MaxVoidWidth(GoProfileStrip tool);

/** 
 * Sets the maximum void width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The maximum void width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetMaxVoidWidth(GoProfileStrip tool, k64f value);

/** 
 * Gets the profile strip tool region.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  A GoProfileRegion object.
 */
GoFx(GoProfileRegion) GoProfileStrip_Region(GoProfileStrip tool);

/** 
 * Adds an additional profile strip tool measurement.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    type           The measurement type to add. It must be a valid profile strip tool type.
 * @param    measurement    A reference to the new GoMeasurement handle. Can be kNULL if you do not wish to do anything immediate with the new measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_AddMeasurement(GoProfileStrip tool, GoMeasurementType type, GoMeasurement* measurement);

/** 
 * Removes a measurement from the tool at the given index.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    index          The index with which to remove a measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_RemoveMeasurement(GoProfileStrip tool, kSize index);

/** 
 * Returns the measurement count for the given tool.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  Tool measurement count.
 */
GoFx(kSize) GoProfileStrip_MeasurementCount(GoProfileStrip tool);

/** 
 * Returns a measurement object at the given index.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    index          The index with which to return a measurement object.
 * @return                  A profile strip tool measurement or kNULL if the index is invalid.
 */
GoFx(GoMeasurement) GoProfileStrip_MeasurementAt(GoProfileStrip tool, kSize index);

kEndHeader()
#include <GoSdk/Tools/GoProfileTools.x.h>

#endif
/// @endcond
