/// @cond (Gocator_2x00 || Gocator_3x00)

/**
 * @file    GoPartDetection.h
 * @brief   Declares the GoPartDetection class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PART_DETECTION_H
#define GO_PART_DETECTION_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoPartDetection
 * @extends kObject
 * @ingroup GoSdk-Surface
 * @brief   Represents the part detection parameters of the surface mode configuration.
 */
typedef kObject GoPartDetection; 

/** 
 * Enables part detection.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   enable          kTRUE to enable, or kFALSE to disable.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_EnablePartDetection(GoPartDetection detection, kBool enable);

/** 
 * Gets the current state of part detection.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoPartDetection_PartDetectionEnabled(GoPartDetection detection);

/** 
 * Returns the state of whether or not the user specified part detection value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_EnablePartDetectionUsed(GoPartDetection detection);

/** 
 * Gets the current part detection system state.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoPartDetection_PartDetectionEnabledSystemValue(GoPartDetection detection);


/** 
 * Gets the part detection minimum threshold value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection minimum threshold value.            
 */
GoFx(k64f) GoPartDetection_ThresholdLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection maximum threshold value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection maximum threshold value.            
 */
GoFx(k64f) GoPartDetection_ThresholdLimitMax(GoPartDetection detection);

/** 
 * Sets the part detection height threshold.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   height          The part detection height threshold value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetThreshold(GoPartDetection detection, k64f height);

/** 
 * Gets the part detection threshold value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection threshold value.            
 */
GoFx(k64f) GoPartDetection_Threshold(GoPartDetection detection);

/** 
 * Sets the part detection height threshold direction.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   direction       The part detection height threshold direction value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetThresholdDirection(GoPartDetection detection, GoPartHeightThresholdDirection direction);

/** 
 * Gets the part detection threshold direction.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection threshold direction.
 */
GoFx(GoPartHeightThresholdDirection) GoPartDetection_ThresholdDirection(GoPartDetection detection);

/** 
 * Sets the part detection frame of reference.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   frameOfRef      The part detection frame of reference value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetFrameOfReference(GoPartDetection detection, GoPartFrameOfReference frameOfRef);

/** 
 * Gets the part detection frame of reference.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection frame of reference.
 */
GoFx(GoPartFrameOfReference) GoPartDetection_FrameOfReference(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified frame of reference value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_FrameOfReferenceUsed(GoPartDetection detection);

/** 
 * Gets the part detection gap width minimum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection gap width minimum value.
 */
GoFx(k64f) GoPartDetection_GapWidthLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection gap width maximum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection gap width maximum value.
 */
GoFx(k64f) GoPartDetection_GapWidthLimitMax(GoPartDetection detection);

/** 
 * Sets the part detection gap width.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   gap             The part detection gap width value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetGapWidth(GoPartDetection detection, k64f gap);

/** 
 * Gets the part detection gap width value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection gap width value.
 */
GoFx(k64f) GoPartDetection_GapWidth(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified gap width value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_GapWidthUsed(GoPartDetection detection);

/** 
 * Gets the part detection gap width minimum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection gap width minimum value.
 */
GoFx(k64f) GoPartDetection_GapLengthLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection gap length maximum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection gap length maximum value.
 */
GoFx(k64f) GoPartDetection_GapLengthLimitMax(GoPartDetection detection);

/** 
 * Sets the part detection gap length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   gap             The part detection gap length value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetGapLength(GoPartDetection detection, k64f gap);

/** 
 * Gets the part detection gap length value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection gap length value.
 */
GoFx(k64f) GoPartDetection_GapLength(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified gap length value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_GapLengthUsed(GoPartDetection detection);

/** 
 * Gets the part detection padding width minimum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection padding width minimum value.
 */
GoFx(k64f) GoPartDetection_PaddingWidthLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection padding width maximum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection padding width maximum value.
 */
GoFx(k64f) GoPartDetection_PaddingWidthLimitMax(GoPartDetection detection);

/** 
 * Sets the part detection padding width.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   padding         The part detection padding width value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetPaddingWidth(GoPartDetection detection, k64f padding);

/** 
 * Gets the part detection padding width value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection padding width value.
 */
GoFx(k64f) GoPartDetection_PaddingWidth(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified padding width value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_PaddingWidthUsed(GoPartDetection detection);

/** 
 * Gets the part detection padding length minimum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection padding length minimum value.
 */
GoFx(k64f) GoPartDetection_PaddingLengthLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection padding length maximum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection padding length maximum value.
 */
GoFx(k64f) GoPartDetection_PaddingLengthLimitMax(GoPartDetection detection);

/** 
 * Sets the part detection padding length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   padding         The part detection padding length value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetPaddingLength(GoPartDetection detection, k64f padding);

/** 
 * Gets the part detection padding length value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection padding length value.
 */
GoFx(k64f) GoPartDetection_PaddingLength(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified padding length value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_PaddingLengthUsed(GoPartDetection detection);


/**
 * Gets the part detection minimum length range minimum value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection minimum length range minimum value.
 */
GoFx(k64f) GoPartDetection_MinLengthLimitMin(GoPartDetection detection);

/**
 * Gets the part detection minimum length range maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection minimum length range maximum value.
 */
GoFx(k64f) GoPartDetection_MinLengthLimitMax(GoPartDetection detection);

/**
 * Sets the part detection minimum length.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @param   length          The part detection minimum length value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoPartDetection_SetMinLength(GoPartDetection detection, k64f length);

/**
 * Gets the part detection minimum length value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection minimum length value.
 */
GoFx(k64f) GoPartDetection_MinLength(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified minimum length value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_MinLengthUsed(GoPartDetection detection);


/** 
 * Gets the part detection maximum length range minimum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection maximum length range minimum value.
 */
GoFx(k64f) GoPartDetection_MaxLengthLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection maximum length range maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection maximum length range maximum value.
 */
GoFx(k64f) GoPartDetection_MaxLengthLimitMax(GoPartDetection detection);

/** 
 * Sets the part detection maximum length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   length          The part detection maximum length value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetMaxLength(GoPartDetection detection, k64f length);

/** 
 * Gets the part detection max length value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection max length value.
 */
GoFx(k64f) GoPartDetection_MaxLength(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified maximum length value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_MaxLengthUsed(GoPartDetection detection);

/** 
 * Sets the part detection minimum area.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @param   area            The part detection minimum area value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartDetection_SetMinArea(GoPartDetection detection, k64f area);

/** 
 * Gets the part detection minimum area value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection minimum area value.
 */
GoFx(k64f) GoPartDetection_MinArea(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified minimum area value is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_MinAreaUsed(GoPartDetection detection);

/** 
 * Gets the part detection minimum area range maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection minimum area range maximum value.
 */
GoFx(k64f) GoPartDetection_MinAreaLimitMin(GoPartDetection detection);

/** 
 * Gets the part detection maximum area range maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   detection       GoPartDetection object.
 * @return                  Part detection maximum area range maximum value.
 */
GoFx(k64f) GoPartDetection_MinAreaLimitMax(GoPartDetection detection);

/**
 * Returns the state of whether or not the user specified edge filtering configuration is used.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_EdgeFilterUsed(GoPartDetection detection);

/**
 * Enables or disables edge filtering.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @param   enable          kTRUE to enable edge filtering and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoPartDetection_EnableEdgeFilter(GoPartDetection detection, kBool enable);

/**
 * Returns the state of whether or not edge filtering is enabled.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if enabled and kFALSE if not.
 */
GoFx(kBool) GoPartDetection_EdgeFilterEnabled(GoPartDetection detection);

/**
 * Gets the part detection edge filtering width minimum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection edge filtering width minimum value.
 */
GoFx(k64f) GoPartDetection_EdgeFilterWidthLimitMin(GoPartDetection detection);

/**
 * Gets the part detection edge filtering width maximum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection edge filtering width maximum value.
 */
GoFx(k64f) GoPartDetection_EdgeFilterWidthLimitMax(GoPartDetection detection);

/**
 * Sets the part detection edge filtering width.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @param   value           The part detection edge filtering width value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoPartDetection_SetEdgeFilterWidth(GoPartDetection detection, k64f value);

/**
 * Gets the part detection edge filtering width value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection edge filtering width value.
 */
GoFx(k64f) GoPartDetection_EdgeFilterWidth(GoPartDetection detection);

/**
 * Gets the part detection edge filtering length minimum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection edge filtering length minimum value.
 */
GoFx(k64f) GoPartDetection_EdgeFilterLengthLimitMin(GoPartDetection detection);

/**
 * Gets the part detection edge filtering length maximum limit value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection edge filtering length maximum value.
 */
GoFx(k64f) GoPartDetection_EdgeFilterLengthLimitMax(GoPartDetection detection);

/**
 * Sets the part detection edge filtering Length.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @param   value           The part detection edge filtering length value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoPartDetection_SetEdgeFilterLength(GoPartDetection detection, k64f value);

/**
 * Gets the part detection edge filtering length value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  Part detection edge filtering length value.
 */
GoFx(k64f) GoPartDetection_EdgeFilterLength(GoPartDetection detection);

/**
 * Enables or disables the part detection edge filtering anterior preservation.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @param   enable          The part detection edge anterior preservation value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoPartDetection_EnableEdgeFilterAnteriorPreservation(GoPartDetection detection, kBool enable);

/**
 * Gets the part detection edge filtering anterior preservation value.
 *
 * @public                  @memberof GoPartDetection
 * @version             Introduced in firmware 4.2.4.7
 * @param   detection       GoPartDetection object.
 * @return                  kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoPartDetection_EdgeFilterAnteriorPreservationEnabled(GoPartDetection detection);


kEndHeader()
#include <GoSdk/GoPartDetection.x.h>

#endif

/// @endcond
