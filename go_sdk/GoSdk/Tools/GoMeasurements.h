/** 
 * @file    GoMeasurements.h
 * @brief   Declares the GoMeasurement classes. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_MEASUREMENTS_H
#define GO_SDK_MEASUREMENTS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoExtParam.h>

kBeginHeader()

/// @cond Gocator_1x00

/**
 * @class   GoRangePositionZ
 * @extends GoMeasurement
 * @ingroup GoSdk
 * @brief   Represents a position Z measurement of a Range Position tool.
 */
typedef GoMeasurement GoRangePositionZ;

/**
 * @class   GoRangeThicknessThickness
 * @extends GoMeasurement
 * @ingroup GoSdk
 * @brief   Represents a the thickness measurement of a Range Thickness tool.
 */
typedef GoMeasurement GoRangeThicknessThickness;

/// @endcond


/// @cond (Gocator_1x00 || Gocator_2x00)

/**
 * @class   GoProfileAreaArea
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an area measurement for a Profile Area tool.
 */
typedef GoMeasurement GoProfileAreaArea;

/**
 * @class   GoProfileAreaCentroidX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a centroid X measurement for a Profile Area Tool.
 */
typedef GoMeasurement GoProfileAreaCentroidX;

/**
 * @class   GoProfileAreaCentroidZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a centroid Z measurement for a Profile Area Tool.
 */
typedef GoMeasurement GoProfileAreaCentroidZ;

/**
 * @class   GoProfileBoxX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxX;

/**
 * @class   GoProfileBoxZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxZ;

/**
 * @class   GoProfileBoxWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxWidth;

/**
 * @class   GoProfileBoxHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a height measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxHeight;

/**
 * @class   GoProfileBoxGlobalX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a global X measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxGlobalX;


/**
 * @class   GoProfileCircleX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X value measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleX;

/**
 * @class   GoProfileCircleZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z value measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleZ;


/**
 * @class   GoProfileCircleRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a radius value measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleRadius;


/**
 * @class   GoProfileDimWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimWidth;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoProfileDimWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimWidth object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoProfileDimWidth_AbsoluteEnabled(GoProfileDimWidth measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoProfileDimWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimWidth object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileDimWidth_EnableAbsolute(GoProfileDimWidth measurement, kBool absolute);

/**
 * @class   GoProfileDimHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a height value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimHeight;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoProfileDimHeight 
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimHeight object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoProfileDimHeight_AbsoluteEnabled(GoProfileDimHeight measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoProfileDimHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimHeight object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileDimHeight_EnableAbsolute(GoProfileDimHeight measurement, kBool absolute);

/**
 * @class   GoProfileDimDistance
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a distance value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimDistance;

/**
 * @class   GoProfileDimCenterX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a center X value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimCenterX;

/**
 * @class   GoProfileDimCenterZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a center Z value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimCenterZ;

/**
 * @class   GoProfileIntersectX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an intersect X measurement for a Profile Intersect Tool.
 */
typedef GoMeasurement GoProfileIntersectX;

/**
 * @class   GoProfileIntersectZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an intersect Z measurement for a Profile Intersect Tool.
 */
typedef GoMeasurement GoProfileIntersectZ;

/**
 * @class   GoProfileIntersectAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an intersect angle measurement for a Profile Intersect Tool.
 */
typedef GoMeasurement GoProfileIntersectAngle;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoProfileIntersectAngle
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileIntersectAngle object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoProfileIntersectAngle_AbsoluteEnabled(GoProfileIntersectAngle measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoProfileIntersectAngle
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileIntersectAngle object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileIntersectAngle_EnableAbsolute(GoProfileIntersectAngle measurement, kBool absolute);

/**
 * @class   GoProfileGrooveX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X value measurement for a Profile Groove Tool.
 */
typedef GoMeasurement GoProfileGrooveX;

/** 
 * Gets the current groove selection type.
 *
 * @public                  @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveX object.
 * @return                  The profile groove location.
 */
GoFx(GoProfileGrooveLocation) GoProfileGrooveX_Location(GoProfileGrooveX measurement);

/** 
 * Sets the groove location.
 *
 * @public                  @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveX  object.
 * @param    value          The groove location value to be set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileGrooveX_SetLocation(GoProfileGrooveX measurement, GoProfileGrooveLocation value);


/** 
 * Gets the current groove selection type.
 *
 * @public                  @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveX object.
 * @return                  The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveX_SelectType(GoProfileGrooveX measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveX object.
 * @param    selectType          The profile groove type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveX_SetSelectType(GoProfileGrooveX measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                      @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveX object.
 * @return                      The current groove index.            
 */
GoFx(k32u) GoProfileGrooveX_SelectIndex(GoProfileGrooveX measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                      @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveX object.
 * @param    selectN            The selected groove index.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileGrooveX_SetSelectIndex(GoProfileGrooveX measurement, k32u selectN);



/**
 * @class   GoProfileGrooveZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z value measurement for a Profile Groove tool.
 */
typedef GoMeasurement GoProfileGrooveZ;

/** 
 * Gets the current groove location type.
 *
 * @public                      @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveZ object.
 * @return                      The profile groove location type.            
 */
GoFx(GoProfileGrooveLocation) GoProfileGrooveZ_Location(GoProfileGrooveZ measurement);

/** 
 * Sets the groove location type.
 *
 * @public                       @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveZ object.
 * @param    location            The profile groove location type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveZ_SetLocation(GoProfileGrooveZ measurement, GoProfileGrooveLocation location);

/** 
 * Gets the current groove selection type.
 *
 * @public                      @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveZ object.
 * @return                      The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveZ_SelectType(GoProfileGrooveZ measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveZ object.
 * @param    selectType          The profile groove selection type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveZ_SetSelectType(GoProfileGrooveZ measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                  @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveZ object.
 * @return                  The current groove index.            
 */
GoFx(k32u) GoProfileGrooveZ_SelectIndex(GoProfileGrooveZ measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                  @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveZ object.
 * @param    selectN        The selected groove index.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileGrooveZ_SetSelectIndex(GoProfileGrooveZ measurement, k32u selectN);


/**
 * @class   GoProfileGrooveWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width measurement for a Profile Groove Tool.
 */
typedef GoMeasurement GoProfileGrooveWidth;

/** 
 * Gets the current groove selection type.
 *
 * @public                      @memberof GoProfileGrooveWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveWidth object.
 * @return                      The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveWidth_SelectType(GoProfileGrooveWidth measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveWidth object.
 * @param    selectType          The profile groove type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveWidth_SetSelectType(GoProfileGrooveWidth measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                      @memberof GoProfileGrooveWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveWidth object.
 * @return                      The current groove index.            
 */
GoFx(k32u) GoProfileGrooveWidth_SelectIndex(GoProfileGrooveWidth measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                      @memberof GoProfileGrooveWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveWidth object.
 * @param    selectN            The selected groove index.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileGrooveWidth_SetSelectIndex(GoProfileGrooveWidth measurement, k32u selectN);


/**
 * @class   GoProfileGrooveDepth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a depth measurement for a Profile Groove Tool.
 */
typedef GoMeasurement GoProfileGrooveDepth;

/** 
 * Gets the current groove selection type.
 *
 * @public                      @memberof GoProfileGrooveDepth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveDepth object.
 * @return                      The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveDepth_SelectType(GoProfileGrooveDepth measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveDepth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveDepth object.
 * @param    selectType          The profile groove type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveDepth_SetSelectType(GoProfileGrooveDepth measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                      @memberof GoProfileGrooveDepth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveDepth object.
 * @return                      The current groove index.            
 */
GoFx(k32u) GoProfileGrooveDepth_SelectIndex(GoProfileGrooveDepth measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                      @memberof GoProfileGrooveDepth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveDepth object.
 * @param    selectN            The selected groove index.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileGrooveDepth_SetSelectIndex(GoProfileGrooveDepth measurement, k32u selectN);


/**
 * @class   GoProfileLineStdDev
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a standard deviation measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLineStdDev;

/**
 * @class   GoProfileLineMinError
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a minimum error measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLineMinError;

/**
 * @class   GoProfileLineMaxError
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a maximum error measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLineMaxError;

/**
 * @class   GoProfileLinePercentile
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a percentile measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLinePercentile;

/** 
 * Gets the percent threshold.
 *
 * @public                      @memberof GoProfileLinePercentile 
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileLinePercentile object.
 * @return                      The percent threshold.
 */
GoFx(k64f) GoProfileLinePercentile_Percent(GoProfileLinePercentile measurement);

/** 
 * Sets the percent threshold.
 *
 * @public                      @memberof GoProfileLinePercentile 
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileLinePercentile object.
 * @param    percent            The percent threshold to set.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileLinePercentile_SetPercent(GoProfileLinePercentile measurement, k64f percent);

/**
 * @class   GoProfilePanelGap
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a gap measurement for a Profile Panel Tool.
 */
typedef GoMeasurement GoProfilePanelGap;

/** 
 * Gets the gap axis.
 *
 * @public                      @memberof GoProfilePanelGap
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelGap object.
 * @return                      The gap axis.    
 */
GoFx(GoProfileGapAxis) GoProfilePanelGap_Axis(GoProfilePanelGap measurement);

/** 
 * Sets the gap axis.
 *
 * @public                      @memberof GoProfilePanelGap
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelGap object.
 * @param    axis               The gap axis value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfilePanelGap_SetAxis(GoProfilePanelGap measurement, GoProfileGapAxis axis);

/**
 * @class   GoProfilePanelFlush
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a flush measurement for a Profile Panel Tool.
 */
typedef GoMeasurement GoProfilePanelFlush;

/** 
 * Gets absolute value state.
 *
 * @public                      @memberof GoProfilePanelFlush
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelFlush object.
 * @return                      kTRUE if absolute value is enabled and kFALSE otherwise.    
 */
GoFx(kBool) GoProfilePanelFlush_AbsoluteEnabled(GoProfilePanelFlush measurement);

/** 
 * Enables or disables the absolute value state.
 *
 * @public                      @memberof GoProfilePanelFlush
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelFlush object.
 * @param    absolute           kTRUE to enable and kFALSE to disable.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfilePanelFlush_EnableAbsolute(GoProfilePanelFlush measurement, kBool absolute);

/**
 * @class   GoProfilePositionX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Position tool.
 */
typedef GoMeasurement GoProfilePositionX;

/**
 * @class   GoProfilePositionZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an Z measurement for a Profile Position tool.
 */
typedef GoMeasurement GoProfilePositionZ;


/**
 * @class   GoProfileStripX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripX;

/** 
 * Gets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @return                      The profile groove location.
 */
GoFx(GoProfileGrooveLocation) GoProfileStripX_Location(GoProfileStripX measurement);

/** 
 * Sets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @param    location           The groove location value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripX_SetLocation(GoProfileStripX measurement, GoProfileGrooveLocation location);

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripX_SelectType(GoProfileStripX measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @param    selectType         The select type to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripX_SetSelectType(GoProfileStripX measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripX_SelectIndex(GoProfileStripX measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripX_SetSelectIndex(GoProfileStripX measurement, k32u selectIndex);


/**
 * @class   GoProfileStripZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripZ;

/** 
 * Gets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @return                      The profile groove location.
 */
GoFx(GoProfileGrooveLocation) GoProfileStripZ_Location(GoProfileStripZ measurement);

/** 
 * Sets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @param    location           The profile groove location to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripZ_SetLocation(GoProfileStripZ measurement, GoProfileGrooveLocation location);

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripZ_SelectType(GoProfileStripZ measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @param    selectType         The select type to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripZ_SetSelectType(GoProfileStripZ measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripZ_SelectIndex(GoProfileStripZ measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripZ_SetSelectIndex(GoProfileStripZ measurement, k32u selectIndex);


/**
 * @class   GoProfileStripWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripWidth;

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripWidth_SelectType(GoProfileStripWidth measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @param    selectType         The select type to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripWidth_SetSelectType(GoProfileStripWidth measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripWidth_SelectIndex(GoProfileStripWidth measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripWidth_SetSelectIndex(GoProfileStripWidth measurement, k32u selectIndex);


/**
 * @class   GoProfileStripHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an height measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripHeight;

/** 
 * Gets the groove location setting.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @return                      The groove location setting.
 */
GoFx(GoProfileGrooveLocation) GoProfileStripHeight_Location(GoProfileStripHeight measurement);

/** 
 * Sets the location.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @param    location           The location value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripHeight_SetLocation(GoProfileStripHeight measurement, GoProfileGrooveLocation location);

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripHeight_SelectType(GoProfileStripHeight measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @param    selectType         The select type value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripHeight_SetSelectType(GoProfileStripHeight measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripHeight_SelectIndex(GoProfileStripHeight measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripHeight_SetSelectIndex(GoProfileStripHeight measurement, k32u selectIndex);

/// @endcond


/// @cond (Gocator_2x00 || Gocator_3x00)

/**
 * @class   GoSurfaceBoxX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxX;

/**
 * @class   GoSurfaceBoxY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxY;

/**
 * @class   GoSurfaceBoxZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxZ;


/**
 * @class   GoSurfaceBoxZAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z-angle measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxZAngle;

/**
 * @class   GoSurfaceBoxGlobalX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a global X measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxGlobalX;

/**
 * @class   GoSurfaceBoxGlobalY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a global Y measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxGlobalY;

/**
 * @class   GoSurfaceBoxGlobalZAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a global Z angle measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxGlobalZAngle;

/**
 * @class   GoSurfaceBoxLength
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a length measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxLength;

/**
 * @class   GoSurfaceBoxWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a width measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxWidth;

/**
 * @class   GoSurfaceBoxHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a height measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxHeight;

/**
 * @class   GoSurfaceEllipseMajor
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a major value measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseMajor;

/**
 * @class   GoSurfaceEllipseMinor
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a minor value measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseMinor;

/**
 * @class   GoSurfaceEllipseRatio
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a ratio measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseRatio;

/**
 * @class   GoSurfaceEllipseZAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z-angle measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseZAngle;

/**
 * @class   GoSurfaceHoleX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleX;

/**
 * @class   GoSurfaceHoleY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleY;

/**
 * @class   GoSurfaceHoleZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleZ;

/**
 * @class   GoSurfaceHoleRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radius measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleRadius;


/**
 * @class   GoSurfaceOpeningX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningX;

/**
 * @class   GoSurfaceOpeningY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningY;

/**
 * @class   GoSurfaceOpeningZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningZ;

/**
 * @class   GoSurfaceOpeningWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a width measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningWidth;

/**
 * @class   GoSurfaceOpeningLength
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a length measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningLength;

/**
 * @class   GoSurfaceOpeningAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an angle measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningAngle;


/**
 * @class   GoSurfacePlaneXAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X-angle measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneXAngle;

/**
 * @class   GoSurfacePlaneYAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y-angle measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneYAngle;

/**
 * @class   GoSurfacePlaneZOffset
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z-offset measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneZOffset;


/**
 * @class   GoSurfacePositionX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Position Tool.
 */
typedef GoMeasurement GoSurfacePositionX;

/**
 * @class   GoSurfacePositionY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Position Tool.
 */
typedef GoMeasurement GoSurfacePositionY;

/**
 * @class   GoSurfacePositionZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Position Tool.
 */
typedef GoMeasurement GoSurfacePositionZ;


/**
 * @class   GoSurfaceStudBaseX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base X measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudBaseX;

/**
 * @class   GoSurfaceStudBaseY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base Y measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudBaseY;

/**
 * @class   GoSurfaceStudBaseZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base Z measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudBaseZ;

/**
 * @class   GoSurfaceStudTipX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a tip X measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudTipX;

/**
 * @class   GoSurfaceStudTipY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a tip Y measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudTipY;

/**
 * @class   GoSurfaceStudTipZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a tip Z measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudTipZ;

/**
 * @class   GoSurfaceStudRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radius measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudRadius;

/** 
 * Gets the radius offset.
 *
 * @public                      @memberof GoSurfaceStudRadius
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceStudRadius object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceStudRadius_RadiusOffset(GoSurfaceStudRadius measurement);

/** 
 * Sets the radius offset.
 *
 * @public                      @memberof GoSurfaceStudRadius
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceStudRadius object.
 * @param    value              The offset value to set..
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceStudRadius_SetRadiusOffset(GoSurfaceStudRadius measurement, k64f value);

/**
 * @class   GoSurfaceVolumeVolume
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a volume measurement for a Surface Volume Tool.
 */
typedef GoMeasurement GoSurfaceVolumeVolume;

/**
 * @class   GoSurfaceVolumeArea
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an area measurement for a Surface Volume Tool.
 */
typedef GoMeasurement GoSurfaceVolumeArea;

/**
 * @class   GoSurfaceVolumeThickness
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a thickness measurement for a Surface Volume Tool.
 */
typedef GoMeasurement GoSurfaceVolumeThickness;

/** 
 * Gets the location.
 *
 * @public                      @memberof GoSurfaceVolumeThickness
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceVolumeThickness object.
 * @return                      The surface location.
 */
GoFx(GoSurfaceLocation) GoSurfaceVolumeThickness_Location(GoSurfaceVolumeThickness measurement);

/** 
 * Sets the location.
 *
 * @public                      @memberof GoSurfaceVolumeThickness
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceVolumeThickness object.
 * @param    location           The surface location.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceVolumeThickness_SetLocation(GoSurfaceVolumeThickness measurement, GoSurfaceLocation location);

/**
 * @class   GoSurfaceCountersunkHoleX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleX;

/**
 * @class   GoSurfaceCountersunkHoleY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleY;

/**
 * @class   GoSurfaceCountersunkHoleZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleZ;

/**
 * @class   GoSurfaceCountersunkHoleOuterRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an Outer Radius position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleOuterRadius;

/**
 * @class   GoSurfaceCountersunkHoleDepth
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Depth position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleDepth;

/**
 * @class   GoSurfaceCountersunkHoleBevelRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an Bevel Radius position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleBevelRadius;

/**
 * @class   GoSurfaceCountersunkHoleBevelAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Bevel Angle measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleBevelAngle;

/**
 * @class   GoSurfaceCountersunkHoleXAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X Angle position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleXAngle;

/**
 * @class   GoSurfaceCountersunkHoleYAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y Angle position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleYAngle;

/// @endcond


/**
 * @class   GoScriptOutput
 * @extends GoMeasurement
 * @ingroup GoSdk-Tools
 * @brief   Represents a script output for a Script Tool.
 */
typedef GoMeasurement GoScriptOutput;

kEndHeader()
#include <GoSdk/Tools/GoMeasurements.x.h>

#endif