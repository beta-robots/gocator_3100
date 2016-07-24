/// @cond Gocator_1x00
/** 
 * @file    GoRangeTools.h
 * @brief   Declares all range tools and their related classes.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_RANGE_TOOLS_H
#define GO_RANGE_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/Tools/GoTool.h>
kBeginHeader()

/**
 * @class   GoRangeTool
 * @extends GoTool
 * @ingroup GoSdk-RangeTools
 * @brief   Represents a base range tool.
 */
typedef GoTool GoRangeTool; 

/** 
 * Sets the data source.
 *
 * @public              @memberof GoRangeTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangeTool object.
 * @param    source     GoDataSource object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRangeTool_SetSource(GoRangeTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public              @memberof GoRangeTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangeTool object.
 * @return              The data source.            
 */
GoFx(GoDataSource) GoRangeTool_Source(GoRangeTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public             @memberof GoRangeTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool      GoRangeTool object.
 * @return             The data source option list count.            
 */
GoFx(kSize) GoRangeTool_SourceOptionCount(GoRangeTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public             @memberof GoRangeTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool      GoRangeTool object.
 * @param    index     The index with which to retrieve a data source option.
 * @return             The data source option at the given index or k32U_MAX if an invalid index is given.            
 */
GoFx(GoDataSource) GoRangeTool_SourceOptionAt(GoRangeTool tool, kSize index);

/**
 * @class   GoRangePosition
 * @extends GoRangeTool
 * @ingroup GoSdk-RangeTools
 * @brief   Represents a range position tool.
 */
typedef GoRangeTool GoRangePosition; 

/**
 * Returns a range position tool Z measurement object.
 *
 * @public              @memberof GoRangePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangePosition object.
 * @return              A GoRangePositionZ measurement.
 */
GoFx(GoRangePositionZ) GoRangePosition_ZMeasurement(GoRangePosition tool);

/**
 * @class   GoRangeThickness
 * @extends GoRangeTool
 * @ingroup GoSdk-RangeTools
 * @brief   Represents a range thickness tool.
 */
typedef GoRangeTool GoRangeThickness; 

/**
 * Returns a boolean value representing whether or not the measurement is set to return an absolute value.
 *
 * @public              @memberof GoRangePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangePosition object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoRangeThickness_AbsoluteEnabled(GoRangeThickness tool);

/**
 * Enable or disable absolute value measurement output.
 *
 * @public              @memberof GoRangePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangePosition object.
 * @param    enable     kTRUE to enable absolute value and kFALSE to disable it.
 * @return              Operation status.
 */
GoFx(kStatus) GoRangeThickness_EnableAbsolute(GoRangeThickness tool, kBool enable);

/**
 * Returns a range thickness tool Thickness measurement object.
 *
 * @public               @memberof GoRangeThickness
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoRangeThickness object.
 * @return               A GoRangeThicknessThickness measurement.
 */
GoFx(GoRangeThickness) GoRangeThickness_ThicknessMeasurement(GoRangeThickness tool);

kEndHeader()

#include <GoSdk/Tools/GoRangeTools.x.h>

#endif
/// @endcond
