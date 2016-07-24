/** 
 * @file    GoSetup.h
 * @brief   Declares the GoSetup class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SETUP_H
#define GO_SDK_SETUP_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoLayout.h>
#include <GoSdk/GoMaterial.h>
#include <GoSdk/GoProfileGeneration.h>
#include <GoSdk/GoSurfaceGeneration.h>
#include <GoSdk/GoPartDetection.h>
#include <GoSdk/GoPartMatching.h>

kBeginHeader()

/**
 * @class   GoSetup
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a device configuration.
 */
typedef kObject GoSetup; 

/** 
 * Sets the default operation mode of the sensor system.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   mode        Operation mode.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetScanMode(GoSetup setup, GoMode mode);

/** 
 * Gets the default operation mode of the sensor system.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Operation mode.
 */
GoFx(GoMode) GoSetup_ScanMode(GoSetup setup);

/** 
 * Gets the scan mode option at the specified index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   index       The index with which to retrieve a mode option.
 * @return              Operation mode.
 * @see                 GoSetup_ScanModeOptionCount
 */
GoFx(GoMode) GoSetup_ScanModeOptionAt(GoSetup setup, kSize index);

/** 
 * Gets the scan mode option count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Scan mode option count.
 */
GoFx(kSize) GoSetup_ScanModeOptionCount(GoSetup setup);

/// @cond Gocator_2x00

/** 
 * Gets the user specified Uniform Spacing enabled state.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 * @see                 GoSetup_UniformSpacingAvailable
 */
GoFx(kBool) GoSetup_UniformSpacingEnabled(GoSetup setup);

/** 
 * Sets the user specified Uniform Spacing enabled state.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      kTRUE to enable it and kFALSE to disable it.
 * @return              Operation status.
 * @see                 GoSetup_UniformSpacingAvailable
 */
GoFx(kStatus) GoSetup_EnableUniformSpacing(GoSetup setup, kBool enable);

/** 
 * Gets a boolean representing whether or not the user specified Uniform Spacing setting is being used at the moment.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if the user specified Uniform Spacing setting is used and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_UniformSpacingAvailable(GoSetup setup);

/** 
 * Gets the Uniform Spacing enabled system value.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 * @see                 GoSetup_UniformSpacingAvailable
 */
GoFx(kBool) GoSetup_UniformSpacingEnabledSystemValue(GoSetup setup);

/// @endcond

/** 
 * Sets the state of the external input triggered encoder Z-pulse feature.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   enable      kTRUE to enable it and kFALSE to disable it.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSetup_EnableExternalInputZPulse(GoSetup setup, kBool enable);

/** 
 * Gets the state of the external input triggered encoder Z-pulse feature.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_ExternalInputZPulseEnabled(GoSetup setup);

/** 
 * Gets the occlusion reduction enabled state.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_OcclusionReductionEnabled(GoSetup setup);

/** 
 * Sets the occlusion reduction enabled state.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      kTRUE to enable it and kFALSE to disable it.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableOcclusionReduction(GoSetup setup, kBool enable);


//class - trigger

/** 
 * Sets the system trigger units.
 * Ignored if GoSetup_TriggerSource is time or encoder
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   unit      The system trigger unit.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetTriggerUnit(GoSetup setup, GoTriggerUnits unit);

/** 
 * Gets the system trigger units.
 * Ignored if GoSetup_TriggerSource is time or encoder
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The system domain.
 */
GoFx(GoTriggerUnits) GoSetup_TriggerUnit(GoSetup setup);

/** 
 * Sets the trigger source for profile triggering.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   source      Profile trigger source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetTriggerSource(GoSetup setup, GoTrigger source);

/** 
 * Gets the trigger source for profile triggering.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Profile trigger source.
 */
GoFx(GoTrigger) GoSetup_TriggerSource(GoSetup setup);

/** 
 * Sets the trigger delay. Depending on GoDomain, units are uS or mm.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   delay       Trigger delay (uS or mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetTriggerDelay(GoSetup setup, k64f delay);

/** 
 * Gets the trigger delay. Depending on GoDomain, units are uS or mm.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Trigger delay (uS or mm).
 */
GoFx(k64f) GoSetup_TriggerDelay(GoSetup setup);

/** 
 * Reports the minimum trigger delay, based on current settings.
 *        Depending on GoDomain, units are uS or mm.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Minimum trigger delay (uS or mm).
 */
GoFx(k64f) GoSetup_TriggerDelayLimitMin(GoSetup setup);

/** 
 * Reports the maximum trigger delay, based on current settings.
 *        Depending on GoDomain, units are uS or mm.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Maximum trigger delay (uS or mm).
 */
GoFx(k64f) GoSetup_TriggerDelayLimitMax(GoSetup setup);

/** 
 * Sets the trigger gate feature.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      Enables trigger gate operation.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableTriggerGate(GoSetup setup, kBool enable);

/** 
 * Reports whether the trigger gate feature is currently enabled.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled, or kFALSE if disabled.
 */
GoFx(kBool) GoSetup_TriggerGateEnabled(GoSetup setup);

/** 
 * Gets the system value representing whether or not the user specified trigger gate enabled setting is being used at the moment.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if the user specified setting is used and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_TriggerGateEnabledUsed(GoSetup setup);

/** 
 * Reports the trigger gate enabled system value.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled, or kFALSE if disabled.
 */
GoFx(kBool) GoSetup_TriggerGateEnabledSystemValue(GoSetup setup);

/** 
 * Enables or disables operation at full frame rate (ignoring frame rate setting).
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      Enables full frame rate operation.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableMaxFrameRate(GoSetup setup, kBool enable);

/** 
 * Reports whether or not system is configured to operate at full frame rate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              True if system operates at full frame rate; false otherwise.
 */
GoFx(kBool) GoSetup_MaxFrameRateEnabled(GoSetup setup);

/** 
 * Sets the current frame rate for time-based triggering. The maximum frame 
 * rate option must be disabled to use the value set in this function.
 * 
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   frameRate   Frame Rate value (Hz).
 * @return              Operation status.
 * @see                 GoSetup_EnableMaxFrameRate, GoSetup_MaxFrameRateEnabled
 */
GoFx(kStatus) GoSetup_SetFrameRate(GoSetup setup, k64f frameRate);

/** 
 * Reports the currently configured frame rate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Maximum frame rate (Hz).
 */
GoFx(k64f) GoSetup_FrameRate(GoSetup setup);

/** 
 * Constraint for the minimum valid value of the Frame Rate setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Minimum valid Frame Rate setting (Hz).
 */
GoFx(k64f) GoSetup_FrameRateLimitMin(GoSetup setup);

/** 
 * Constraint for the maximum valid value of the Frame Rate setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Maximum valid Frame Rate setting (Hz).
 */
GoFx(k64f) GoSetup_FrameRateLimitMax(GoSetup setup);

/** 
 * Sets the encoder trigger mode.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   mode        Encoder trigger mode.
 * @return              Profile trigger source.
 */
GoFx(kStatus) GoSetup_SetEncoderTriggerMode(GoSetup setup, GoEncoderTriggerMode mode);

/** 
 * Gets the encoder trigger mode.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Encoder trigger mode.
 */
GoFx(GoEncoderTriggerMode) GoSetup_EncoderTriggerMode(GoSetup setup);

/** 
 * Sets the current encoder period for encoder-based triggering.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   period      Encoder period (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetEncoderSpacing(GoSetup setup, k64f period);

/** 
 * Gets the current encoder period for encoder-based triggering.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Encoder period (mm).
 */
GoFx(k64f) GoSetup_EncoderSpacing(GoSetup setup);

/** 
 * Constraint for the minimum valid value of the Encoder Period setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Minimum valid Encoder Period setting (mm).
 */
GoFx(k64f) GoSetup_EncoderSpacingLimitMin(GoSetup setup);

/** 
 * Constraint for the maximum valid value of the Encoder Period setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Maximum valid Encoder Period setting (mm).
 */
GoFx(k64f) GoSetup_EncoderSpacingLimitMax(GoSetup setup);

/** 
 * Enables profile intensity collection.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      kTRUE to enable, or kFALSE to disable.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableIntensity(GoSetup setup, kBool enable);

/** 
 * Reports whether the profile intensity collection is enabled.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled, or kFALSE if disabled.
 */
GoFx(kBool) GoSetup_IntensityEnabled(GoSetup setup);

//class - alignment

/** 
 * Gets the input trigger enabled state.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_InputTriggerEnabled(GoSetup setup);

/** 
 * Sets the input trigger enabled state.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      kTRUE to enable it and kFALSE to disable it.
 */
GoFx(kStatus) GoSetup_EnableInputTrigger(GoSetup setup, kBool enable);

/** 
 * Sets the type used for alignment.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   type        Alignment type (stationary or moving).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetAlignmentType(GoSetup setup, GoAlignmentType type);

/** 
 * Gets the type used for alignment calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Alignment type (stationary or moving).
 */
GoFx(GoAlignmentType) GoSetup_AlignmentType(GoSetup setup);

/** 
 * Gets the alignment type option at the given index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   index       The index with which to retrieve an alignment type option.
 * @return              Alignment type option.
 * @see                 GoSetup_AlignmentTypeOptionCount
 */
GoFx(GoAlignmentType) GoSetup_AlignmentTypeOptionAt(GoSetup setup, kSize index);

/** 
 * Gets the alignment type option count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Alignment type option count.
 */
GoFx(kSize) GoSetup_AlignmentTypeOptionCount(GoSetup setup);

/** 
 * Enables encoder calibration after alignment.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enabled     kTRUE to enable encoder calibration after alignment, kFALSE to disable it.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableAlignmentEncoderCalibrate(GoSetup setup, kBool enabled);

/** 
 * Gets the value of the post alignment encoder calibration setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              kTRUE if post alignment encoder calibration is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSetup_AlignmentEncoderCalibrateEnabled(GoSetup setup);

/** 
 * Sets the target type used for stationary alignment calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   target      Alignment target type.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetAlignmentStationaryTarget(GoSetup setup, GoAlignmentTarget target);

/** 
 * Gets the target type used for stationary alignment calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Alignment target type.
 */
GoFx(GoAlignmentTarget) GoSetup_AlignmentStationaryTarget(GoSetup setup);

/** 
 * Gets the stationary alignment target option count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The alignment target count.
 */
GoFx(kSize) GoSetup_AlignmentStationaryTargetOptionCount(GoSetup setup);

/** 
 * Gets the stationary alignment target option at the given index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   index       The index with which to retrieve an alignment target.
 * @return              The alignment target option.
 * @see                 GoSetup_AlignmentStationaryTargetOptionCount
 */
GoFx(GoAlignmentTarget) GoSetup_AlignmentStationaryTargetOptionAt(GoSetup setup, kSize index);

/// @cond (Gocator_1x00 || Gocator_2x00)

/** 
 * Sets the target type used for moving alignment calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   target      Alignment target type.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetAlignmentMovingTarget(GoSetup setup, GoAlignmentTarget target);

/** 
 * Gets the target type used for moving alignment calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Alignment target type.
 */
GoFx(GoAlignmentTarget) GoSetup_AlignmentMovingTarget(GoSetup setup);

/** 
 * Gets the moving alignment target option count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The alignment target count.
 */
GoFx(kSize) GoSetup_AlignmentMovingTargetOptionCount(GoSetup setup);

/**
* Gets the moving alignment target option at the given index.
*
* @public              @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
* @param   setup       GoSetup object.
* @param   index       The index with which to retrieve an alignment target.
* @return              The alignment target option.
* @see                 GoSetup_AlignmentMovingTargetOptionCount
*/
GoFx(GoAlignmentTarget) GoSetup_AlignmentMovingTargetOptionAt(GoSetup setup, kSize index);

/** 
 * Sets the diameter of the disk used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   diameter    Disk diameter (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetDiskDiameter(GoSetup setup, k64f diameter);

/** 
 * Gets the diameter of the disk used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Disk diameter (mm).
 */
GoFx(k64f) GoSetup_DiskDiameter(GoSetup setup);

/** 
 * Sets the height of the disk used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   height      Disk height (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetDiskHeight(GoSetup setup, k64f height);

/** 
 * Gets the height of the disk used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Disk height (mm).
 */
GoFx(k64f) GoSetup_DiskHeight(GoSetup setup);

/// @endcond


/** 
 * Sets the width of the bar used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   width       Bar width (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetBarWidth(GoSetup setup, k64f width);

/** 
 * Gets the width of the bar used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Bar width (mm).
 */
GoFx(k64f) GoSetup_BarWidth(GoSetup setup);

/** 
 * Sets the height of the bar used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   height      Bar height (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetBarHeight(GoSetup setup, k64f height);

/** 
 * Gets the height of the bar used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Bar height (mm).
 */
GoFx(k64f) GoSetup_BarHeight(GoSetup setup);

/** 
 * Sets the number of holes that are defined on the calibration bar.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   count       Hole count.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetBarHoleCount(GoSetup setup, kSize count);

/** 
 * Gets the number of holes that are defined on the calibration bar.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Hole count.
 */
GoFx(kSize) GoSetup_BarHoleCount(GoSetup setup);

/** 
 * Sets the distance between holes that are defined on the calibration bar.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   distance    Hole distance (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetBarHoleDistance(GoSetup setup, k64f distance);

/** 
 * Gets the distance between holes that are defined on the calibration bar.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Hole distance (mm).
 */
GoFx(k64f) GoSetup_BarHoleDistance(GoSetup setup);

/** 
 * Sets the diameter of holes that are defined on the calibration bar.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   diameter    Hole diameter (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetBarHoleDiameter(GoSetup setup, k64f diameter);

/** 
 * Gets the diameter of holes that are defined on the calibration bar.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Hole diameter (mm).
 */
GoFx(k64f) GoSetup_BarHoleDiameter(GoSetup setup);

/** 
 * Sets the height of the plate used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   height      Plate height (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetPlateHeight(GoSetup setup, k64f height);

/** 
 * Gets the height of the plate used for travel calibration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Plate height (mm).
 */
GoFx(k64f) GoSetup_PlateHeight(GoSetup setup);

/** 
 * Sets the number of holes that are defined on the calibration plate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   count       Hole count.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetPlateHoleCount(GoSetup setup, kSize count);

/** 
 * Gets the number of holes that are defined on the calibration plate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Hole count.
 */
GoFx(kSize) GoSetup_PlateHoleCount(GoSetup setup);

/** 
 * Sets the diameter of the reference hole defined on the calibration plate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   diameter    Hole diameter (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetPlateRefHoleDiameter(GoSetup setup, k64f diameter);

/** 
 * Gets the diameter of the reference hole defined on the calibration plate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Hole diameter (mm).
 */
GoFx(k64f) GoSetup_PlateRefHoleDiameter(GoSetup setup);


/** 
 * Sets the diameter of the secondary hole defined on the calibration plate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   diameter    Hole diameter (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetPlateSecHoleDiameter(GoSetup setup, k64f diameter);

/** 
 * Gets the diameter of the secondary hole defined on the calibration plate.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Hole diameter (mm).
 */
GoFx(k64f) GoSetup_PlateSecHoleDiameter(GoSetup setup);


//class - filters

/** 
 * Sets the status of x-direction smoothing.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enabled     Enable or disable x-smoothing
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableXSmoothing(GoSetup setup, kBool enabled);

/** 
 * Gets the status of x-direction smoothing.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              True if x-smoothing is enabled.
 */
GoFx(kBool) GoSetup_XSmoothingEnabled(GoSetup setup);

/** 
 * Sets the x-direction smoothing window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   window      The x-smoothing window (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetXSmoothingWindow(GoSetup setup, k64f window);

/** 
 * Gets the x-direction smoothing window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The x-smoothing window (mm).
 */
GoFx(k64f) GoSetup_XSmoothingWindow(GoSetup setup);

/** 
 * Gets the x-direction smoothing window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The x-smoothing window min (mm).
 */
GoFx(k64f) GoSetup_XSmoothingWindowLimitMin(GoSetup setup);

/** 
 * Gets the x-direction smoothing window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The x-smoothing window max (mm).
 */
GoFx(k64f) GoSetup_XSmoothingWindowLimitMax(GoSetup setup);

/** 
 * Sets the status of y-direction smoothing.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      Enable or disable y-smoothing
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableYSmoothing(GoSetup setup, kBool enable);

/** 
 * Gets the status of y-direction smoothing.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              True if y-smoothing is enabled.
 */
GoFx(kBool) GoSetup_YSmoothingEnabled(GoSetup setup);

/** 
 * Sets the y-direction smoothing window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   window      The y-smoothing window (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetYSmoothingWindow(GoSetup setup, k64f window);

/** 
 * Gets the y-direction smoothing window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The y-smoothing window (mm).
 */
GoFx(k64f) GoSetup_YSmoothingWindow(GoSetup setup);

/** 
 * Gets the y-direction smoothing window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The y-smoothing window min (mm).
 */
GoFx(k64f) GoSetup_YSmoothingWindowLimitMin(GoSetup setup);

/** 
 * Gets the y-direction smoothing window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The y-smoothing window max (mm).
 */
GoFx(k64f) GoSetup_YSmoothingWindowLimitMax(GoSetup setup);


/** 
 * Sets the status of x-direction median.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   enabled     Enable or disable x-median
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableXMedian(GoSetup setup, kBool enabled);

/** 
 * Gets the status of x-direction median.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              True if x-median is enabled.
 */
GoFx(kBool) GoSetup_XMedianEnabled(GoSetup setup);

/** 
 * Sets the x-direction median window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   window      The x-median window (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetXMedianWindow(GoSetup setup, k64f window);

/** 
 * Gets the x-direction median window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The x-median window (mm).
 */
GoFx(k64f) GoSetup_XMedianWindow(GoSetup setup);

/** 
 * Gets the x-direction median window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The x-median window min (mm).
 */
GoFx(k64f) GoSetup_XMedianWindowLimitMin(GoSetup setup);

/** 
 * Gets the x-direction median window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The x-median window max (mm).
 */
GoFx(k64f) GoSetup_XMedianWindowLimitMax(GoSetup setup);

/** 
 * Sets the status of y-direction median.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   enable      Enable or disable y-median
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableYMedian(GoSetup setup, kBool enable);

/** 
 * Gets the status of y-direction median.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              True if y-median is enabled.
 */
GoFx(kBool) GoSetup_YMedianEnabled(GoSetup setup);

/** 
 * Sets the y-direction median window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   window      The y-median window (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetYMedianWindow(GoSetup setup, k64f window);

/** 
 * Gets the y-direction median window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The y-median window (mm).
 */
GoFx(k64f) GoSetup_YMedianWindow(GoSetup setup);

/** 
 * Gets the y-direction median window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The y-median window min (mm).
 */
GoFx(k64f) GoSetup_YMedianWindowLimitMin(GoSetup setup);

/** 
 * Gets the y-direction median window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The y-median window max (mm).
 */
GoFx(k64f) GoSetup_YMedianWindowLimitMax(GoSetup setup);

/** 
 * Sets the status of x-direction decimation.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   enabled     Enable or disable x-decimation
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableXDecimation(GoSetup setup, kBool enabled);

/** 
 * Gets the status of x-direction decimation.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              True if x-decimation is enabled.
 */
GoFx(kBool) GoSetup_XDecimationEnabled(GoSetup setup);

/** 
 * Sets the x-direction decimation window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   window      The x-decimation window (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetXDecimationWindow(GoSetup setup, k64f window);

/** 
 * Gets the x-direction decimation window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The x-decimation window (mm).
 */
GoFx(k64f) GoSetup_XDecimationWindow(GoSetup setup);

/** 
 * Gets the x-direction decimation window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The x-decimation window min (mm).
 */
GoFx(k64f) GoSetup_XDecimationWindowLimitMin(GoSetup setup);

/** 
 * Gets the x-direction decimation window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The x-decimation window max (mm).
 */
GoFx(k64f) GoSetup_XDecimationWindowLimitMax(GoSetup setup);

/** 
 * Sets the status of y-direction decimation.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   enable      Enable or disable y-decimation
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableYDecimation(GoSetup setup, kBool enable);

/** 
 * Gets the status of y-direction decimation.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              True if y-decimation is enabled.
 */
GoFx(kBool) GoSetup_YDecimationEnabled(GoSetup setup);

/** 
 * Sets the y-direction decimation window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   window      The y-decimation window (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetYDecimationWindow(GoSetup setup, k64f window);

/** 
 * Gets the y-direction decimation window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The y-decimation window (mm).
 */
GoFx(k64f) GoSetup_YDecimationWindow(GoSetup setup);

/** 
 * Gets the y-direction decimation window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The y-decimation window min (mm).
 */
GoFx(k64f) GoSetup_YDecimationWindowLimitMin(GoSetup setup);

/** 
 * Gets the y-direction decimation window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @return              The y-decimation window max (mm).
 */
GoFx(k64f) GoSetup_YDecimationWindowLimitMax(GoSetup setup);


/** 
 * Sets the status of x-direction gap-filling.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      Enable or disable x-gap-filling
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableXGapFilling(GoSetup setup, kBool enable);

/** 
 * Gets the status of x-direction gap-filling.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              True if x-gap-filling is enabled.
 */
GoFx(kBool) GoSetup_XGapFillingEnabled(GoSetup setup);

/** 
 * Sets the status of x-direction gap-filling.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   window      Enable or disable x-gap-filling
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetXGapFillingWindow(GoSetup setup, k64f window);

/** 
 * Gets the x-direction gap-filling window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The x-gap-filling window (mm).
 */
GoFx(k64f) GoSetup_XGapFillingWindow(GoSetup setup);

/** 
 * Gets the x-direction gap-filling window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The x-gap-filling window min (mm).
 */
GoFx(k64f) GoSetup_XGapFillingWindowLimitMin(GoSetup setup);

/** 
 * Gets the x-direction gap-filling window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The x-gap-filling window max (mm).
 */
GoFx(k64f) GoSetup_XGapFillingWindowLimitMax(GoSetup setup);

/** 
 * Sets the status of y-direction gap-filling.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   enable      Enable or disable y-gap-filling
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_EnableYGapFilling(GoSetup setup, kBool enable);

/** 
 * Gets the status of y-direction gap-filling.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              True if y-gap-filling is enabled.
 */
GoFx(kBool) GoSetup_YGapFillingEnabled(GoSetup setup);

/** 
 * Sets the y-direction gap-filling window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   window      The y-gap-filling window (mm).
 * @return                
 */
GoFx(kStatus) GoSetup_SetYGapFillingWindow(GoSetup setup, k64f window);

/** 
 * Gets the y-direction gap-filling window.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The y-gap-filling window (mm).
 */
GoFx(k64f) GoSetup_YGapFillingWindow(GoSetup setup);

/** 
 * Gets the y-direction gap-filling window minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The y-gap-filling window min (mm).
 */
GoFx(k64f) GoSetup_YGapFillingWindowLimitMin(GoSetup setup);

/** 
 * Gets the y-direction gap-filling window maximum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              The y-gap-filling window max (mm).
 */
GoFx(k64f) GoSetup_YGapFillingWindowLimitMax(GoSetup setup);

/** 
 * Gets the maximum valid value for the Exposure setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid Exposure value (microseconds).
 */
GoFx(k64f) GoSetup_ExposureLimitMax(GoSetup setup, GoRole role);

/** 
 * Gets the minimum valid value for the Exposure setting.
 * 
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid Exposure value (microseconds).
 */
GoFx(k64f) GoSetup_ExposureLimitMin(GoSetup setup, GoRole role);
  
/** 
 * Sets the exposure value.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   exposure    Intended exposure value.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetExposure(GoSetup setup, GoRole role, k64f exposure);

/** 
 * Gets the exposure value.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Exposure value (microseconds). 
 */
GoFx(k64f) GoSetup_Exposure(GoSetup setup, GoRole role);

/** 
 * Gets the exposure mode.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Exposure mode. 
 */
GoFx(GoExposureMode) GoSetup_ExposureMode(GoSetup setup, GoRole role);

/** 
 * Sets the exposure mode.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   mode        The exposure mode to use.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetExposureMode(GoSetup setup, GoRole role, GoExposureMode mode);

/** 
 * Gets the exposure mode option at the given index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @param   index       The index with which to retrieve an exposure mode option.
 * @return              Exposure mode option.
 * @see                 GoSetup_ExposureModeOptionCount
 */
GoFx(GoExposureMode) GoSetup_ExposureModeOptionAt(GoSetup setup, GoRole role, kSize index);

/** 
 * Gets the exposure mode option count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Exposure mode option count.
 */
GoFx(kSize) GoSetup_ExposureModeOptionCount(GoSetup setup, GoRole role);

/** 
 * Adds an exposure step
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   exposure    Expsosure value (microseconds).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_AddExposureStep(GoSetup setup, GoRole role, k64f exposure);

/** 
 * Removes all exposure steps.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_ClearExposureSteps(GoSetup setup, GoRole role);

/** 
 * Get the exposure step value specified by index
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @param   index       The index of the exposure step to get.
 * @return              The exposure step value (microseconds).
 * @see                 GoSetup_ExposureStepCount
 */
GoFx(k64f) GoSetup_ExposureStepAt(GoSetup setup, GoRole role, kSize index);

/** 
 * Get the number of exposure steps defined
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              The count of exposure steps.
 */
GoFx(kSize) GoSetup_ExposureStepCount(GoSetup setup, GoRole role);

/** 
 * Gets the maximum value for the Dynamic Exposure setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum Dynamic Exposure value (microseconds).
 */
GoFx(k64f) GoSetup_DynamicExposureMax(GoSetup setup, GoRole role);

/** 
 * Sets the maximum  value for the Dynamic Exposure setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   exposure    Maximum Dynamic Exposure value (microseconds).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetDynamicExposureMax(GoSetup setup, GoRole role, k64f exposure);

/** 
 * Gets the minimum value for the Dynamic Exposure setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum Dynamic Exposure value (microseconds).
 */
GoFx(k64f) GoSetup_DynamicExposureMin(GoSetup setup, GoRole role);

/** 
 * Sets the minimum  value for the Dynamic Exposure setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   exposure    Minumum Dynamic Exposure value (microseconds).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetDynamicExposureMin(GoSetup setup, GoRole role, k64f exposure);

/** 
 * Gets the minimum valid value for the ActiveAreaHeight setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum valid ActiveAreaHeight value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaHeightLimitMin(GoSetup setup, GoRole role);
  
/** 
 * Gets the maximum valid value for the ActiveAreaHeight setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid active area height value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaHeightLimitMax(GoSetup setup, GoRole role);

/** 
 * Gets the active area height.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area height (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaHeight(GoSetup setup, GoRole role);

/** 
 * Sets the active area height.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   height      Active area height (mm).
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetActiveAreaHeight(GoSetup setup, GoRole role, k64f height);
  
/** 
 * Gets the active area Length.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area Length (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaLength(GoSetup setup, GoRole role);

/** 
 * Gets the minimum valid value for the ActiveAreaLength setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum valid ActiveAreaLength value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaLengthLimitMin(GoSetup setup, GoRole role);
  
/** 
 * Gets the maximum valid value for the ActiveAreaLength setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid ActiveAreaLength value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaLengthLimitMax(GoSetup setup, GoRole role);
   
/** 
 * Sets the active area Length.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   Length      Active area Length (mm).
 * @return              Operation status.
 */
GoFx(k64f) GoSetup_SetActiveAreaLength(GoSetup setup, GoRole role, k64f Length);
  
/** 
 * Gets the active area Length.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area Length (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaLength(GoSetup setup, GoRole role);
 

/** 
 * Gets the minimum valid value for the ActiveAreaWidth setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum valid ActiveAreaWidth value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaWidthLimitMin(GoSetup setup, GoRole role);

/** 
 * Gets the maximum valid value for the ActiveAreaWidth setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid ActiveAreaWidth value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaWidthLimitMax(GoSetup setup, GoRole role);

/** 
 * Sets the active area width.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   width       Active area width (mm).
 * @return              Operation status.
 */
GoFx(k64f) GoSetup_SetActiveAreaWidth(GoSetup setup, GoRole role, k64f width);

/** 
 * Gets the active area width.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area width (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaWidth(GoSetup setup, GoRole role);

/** 
 * Gets the minimum valid value for the ActiveAreaX setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum valid ActiveAreaX value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaXLimitMin(GoSetup setup, GoRole role);
   
/** 
 * Gets the maximum valid value for the ActiveAreaX setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid ActiveAreaX value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaXLimitMax(GoSetup setup, GoRole role);
    
/** 
 * Sets the active area x origin.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   x           Active area x origin (mm).
 * @return              Operation status.
 */
GoFx(k64f) GoSetup_SetActiveAreaX(GoSetup setup, GoRole role, k64f x);
   
/** 
 * Gets the active area x origin.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area x origin (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaX(GoSetup setup, GoRole role);

/** 
 * Gets the minimum valid value for the ActiveAreaY setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum valid ActiveAreaY value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaYLimitMin(GoSetup setup, GoRole role);
  
/** 
 * Gets the maximum valid value for the ActiveAreaY setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid ActiveAreaY value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaYLimitMax(GoSetup setup, GoRole role);
   
/** 
 * Sets the active area Y.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   Y           Active area Y (mm).
 * @return              Operation status.
 */
GoFx(k64f) GoSetup_SetActiveAreaY(GoSetup setup, GoRole role, k64f Y);
  
/** 
 * Gets the active area Y.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area Y (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaY(GoSetup setup, GoRole role);

/** 
 * Gets the minimum valid value for the ActiveAreaZ setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Minimum valid ActiveAreaZ value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaZLimitMin(GoSetup setup, GoRole role);
   
/** 
 * Gets the maximum valid value for the ActiveAreaZ setting.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Maximum valid ActiveAreaZ value (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaZLimitMax(GoSetup setup, GoRole role);
    
/** 
 * Sets the active area z origin.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   z           Active area z origin (mm).
 * @return              Operation status.
 */
GoFx(k64f) GoSetup_SetActiveAreaZ(GoSetup setup, GoRole role, k64f z);
   
/** 
 * Gets the active area z origin.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Active area z origin (mm).
 */
GoFx(k64f) GoSetup_ActiveAreaZ(GoSetup setup, GoRole role);//

/** 
 * Gets the transformed data region X value.
 *
 * @public             @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
 * @param   setup      GoSetup object.
 * @param   role       The device whose value to retrieve.
 * @return             The transformed data region X value.
 */
GoFx(k64f) GoSetup_TransformedDataRegionX(GoSetup setup, GoRole role);

/** 
 * Gets the transformed data region Y value.
 *
 * @public             @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
 * @param   setup      GoSetup object.
 * @param   role       The device whose value to retrieve.
 * @return             The transformed data region Y value.
 */
GoFx(k64f) GoSetup_TransformedDataRegionY(GoSetup setup, GoRole role);

/** 
 * Gets the transformed data region Z value.
 *
 * @public             @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
 * @param   setup      GoSetup object.
 * @param   role       The device whose value to retrieve.
 * @return             The transformed data region Z value.
 */
GoFx(k64f) GoSetup_TransformedDataRegionZ(GoSetup setup, GoRole role);

/** 
 * Gets the transformed data region width value.
 *
 * @public             @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
 * @param   setup      GoSetup object.
 * @param   role       The device whose value to retrieve.
 * @return             The transformed data region width value.
 */
GoFx(k64f) GoSetup_TransformedDataRegionWidth(GoSetup setup, GoRole role);

/** 
 * Gets the transformed data region length value.
 *
 * @public             @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
 * @param   setup      GoSetup object.
 * @param   role       The device whose value to retrieve.
 * @return             The transformed data region length value.
 */
GoFx(k64f) GoSetup_TransformedDataRegionLength(GoSetup setup, GoRole role);

/** 
 * Gets the transformed data region height value.
 *
 * @public             @memberof GoSetup
 * @version            Introduced in firmware 4.0.10.27
 * @param   setup      GoSetup object.
 * @param   role       The device whose value to retrieve.
 * @return             The transformed data region height value.
 */
GoFx(k64f) GoSetup_TransformedDataRegionHeight(GoSetup setup, GoRole role);

/** 
 * Gets the count of valid x-resolution options.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Count of x-resolution options.
 */
GoFx(kSize) GoSetup_XSubsamplingOptionCount(GoSetup setup, GoRole role);
 
/** 
 * Gets the x-resolution option at the specified index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @param   index       Index of the desired resolution option.
 * @return              X resolution option.
 * @see                 GoSetup_XSubsamplingOptionCount
 */
GoFx(k32u) GoSetup_XSubsamplingOptionAt(GoSetup setup, GoRole role, kSize index);
  
/** 
 * Sets the current x-resolution divider.
 *
 * @public                  @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup           GoSetup object.
 * @param   role            Determines which sensor to apply changes to. 
 * @param   xSubsampling    X subsampling divider (e.g. 1 - full res, 2 - half res). 
 * @return                  Operation status.
 */
GoFx(kStatus) GoSetup_SetXSubsampling(GoSetup setup, GoRole role, k32u xSubsampling);

/** 
 * Gets the current x-resolution divider.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              X resolution divider (e.g. 1 - full res, 2 - half res). 
 */
GoFx(k32u) GoSetup_XSubsampling(GoSetup setup, GoRole role);

/** 
 * Gets the count of valid z-resolution options.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Count of z-resolution options.
 */
GoFx(kSize) GoSetup_ZSubsamplingOptionCount(GoSetup setup, GoRole role);
 
/** 
 * Gets the z-resolution option at the specified index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @param   index       Index of the desired resolution option.
 * @return              Z resolution option.
 * see                  GoSetup_ZSubsamplingOptionCount
 */
GoFx(k32u) GoSetup_ZSubsamplingOptionAt(GoSetup setup, GoRole role, kSize index);
  
/** 
 * Sets the current z-resolution divider.
 *
 * @public                  @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup           GoSetup object.
 * @param   role            Determines which sensor to apply changes to. 
 * @param   zSubsampling    Z subsampling divider (e.g. 1 - full res, 2 - half res). 
 * @return                  Operation status.
 */
GoFx(kStatus) GoSetup_SetZSubsampling(GoSetup setup, GoRole role, k32u zSubsampling);

/** 
 * Gets the current z-resolution divider.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Z resolution divider (e.g. 1 - full res, 2 - half res). 
 */
GoFx(k32u) GoSetup_ZSubsampling(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest x origin. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest x origin (pixels). 
 */
GoFx(k32u) GoSetup_FrontCameraX(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest y origin. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest y origin (pixels). 
 */
GoFx(k32u) GoSetup_FrontCameraY(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest width. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest width (pixels). 
 */
GoFx(k32u) GoSetup_FrontCameraWidth(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest height. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest height (pixels). 
 */
GoFx(k32u) GoSetup_FrontCameraHeight(GoSetup setup, GoRole role);

/** 
 * Returns a boolean representing whether the back camera element is used.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              kTRUE if used, or kFALSE if not used.
 */
GoFx(k32u) GoSetup_BackCameraUsed(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest x origin. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest x origin (pixels). 
 */
GoFx(k32u) GoSetup_BackCameraX(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest y origin. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest y origin (pixels). 
 */
GoFx(k32u) GoSetup_BackCameraY(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest width. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest width (pixels). 
 */
GoFx(k32u) GoSetup_BackCameraWidth(GoSetup setup, GoRole role);

/** 
 * Gets the camera region-of-interest height. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Region of interest height (pixels). 
 */
GoFx(k32u) GoSetup_BackCameraHeight(GoSetup setup, GoRole role);

/** 
 * Enables tracking. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   enable      kTRUE to enable, or kFALSE to disable.
 * @return              Operation status. 
 */
GoFx(kStatus) GoSetup_EnableTracking(GoSetup setup, GoRole role, kBool enable);

/** 
 * Determines if tracking is enabled. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              kTRUE if enabled, or kFALSE if disabled.
 */
GoFx(kBool) GoSetup_TrackingEnabled(GoSetup setup, GoRole role);

/** 
 * Returns a boolean value representing whether the Tracking Enabled field is used.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              kTRUE if used, or kFALSE if not used.
 */
GoFx(kBool) GoSetup_TrackingUsed(GoSetup setup, GoRole role);

/** 
 * Sets the tracking window height.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   height      Tracking window height (mm).
 * @return              Operation status. 
 */
GoFx(kStatus) GoSetup_SetTrackingAreaHeight(GoSetup setup, GoRole role, k64f height);

/** 
 * Gets the tracking window height. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Tracking window height (mm). 
 */
GoFx(k64f) GoSetup_TrackingAreaHeight(GoSetup setup, GoRole role);

/** 
 * Gets the tracking window height minimum limit. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Tracking window height min(mm). 
 */
GoFx(k64f) GoSetup_TrackingAreaHeightLimitMin(GoSetup setup, GoRole role);

/** 
 * Gets the tracking window height maximum limit. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Tracking window height max(mm). 
 */
GoFx(k64f) GoSetup_TrackingAreaHeightLimitMax(GoSetup setup, GoRole role);

/** 
 * Sets the tracking window search threshold. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   threshold   Tracking window search threshold (%)
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetTrackingSearchThreshold(GoSetup setup, GoRole role, k64f threshold);

/** 
 * Gets the tracking window search threshold. 
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Tracking window search threshold (%). 
 */
GoFx(k64f) GoSetup_TrackingSearchThreshold(GoSetup setup, GoRole role);

/** 
 * Returns the state of whether the user specified spacing interval is used.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              kTRUE if the user value is used and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_SpacingIntervalUsed(GoSetup setup, GoRole role);

/** 
 * Gets the spacing interval system value.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Spacing interval.
 */
GoFx(k64f) GoSetup_SpacingIntervalSystemValue(GoSetup setup, GoRole role);

/** 
 * Gets the spacing interval.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Spacing interval.
 */
GoFx(k64f) GoSetup_SpacingInterval(GoSetup setup, GoRole role);

/// @cond (Gocator_3x00)
/** 
 * Sets the spacing interval.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   value       The spacing interval.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetSpacingInterval(GoSetup setup, GoRole role, k64f value);
/// @endcond

/** 
 * Gets the spacing interval value limit minimum.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Spacing interval value limit minimum.
 */
GoFx(k64f) GoSetup_SpacingIntervalLimitMin(GoSetup setup, GoRole role);

/** 
 * Gets the spacing interval value limit maximum
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Spacing interval value limit maximum.
 */
GoFx(k64f) GoSetup_SpacingIntervalLimitMax(GoSetup setup, GoRole role);

/** 
 * Gets the spacing interval type.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              The spacing interval type.
 */
GoFx(GoSpacingIntervalType) GoSetup_SpacingIntervalType(GoSetup setup, GoRole role);

/** 
 * Sets the spacing interval type.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   type        The spacing interval type.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetSpacingIntervalType(GoSetup setup, GoRole role, GoSpacingIntervalType type);

/** 
 * Gets the system value representing whether or not the user specified spacing interval type setting is being used at the moment.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        The device role from which to retrieve the setting.
 * @return              kTRUE if the user specified spacing interval type setting is used and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_SpacingIntervalTypeUsed(GoSetup setup, GoRole role);

/** 
 * Gets the X spacing count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              X spacing count.
 */
GoFx(k32u) GoSetup_XSpacingCount(GoSetup setup, GoRole role);

/** 
 * Gets the Y spacing count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Y spacing count.
 */
GoFx(k32u) GoSetup_YSpacingCount(GoSetup setup, GoRole role);

/** 
 * Gets the intensity step index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              The intensity step index.
 */
GoFx(kSize) GoSetup_IntensityStepIndex(GoSetup setup, GoRole role);

/** 
 * Sets the intensity step index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   index       The exposure step index to use for intensity acquisition.
 * @return              Operation status.
 * @see                 GoSetup_SetExposureMode, GoSetup_ExposureStepCount
 */
GoFx(kStatus) GoSetup_SetIntensityStepIndex(GoSetup setup, GoRole role, kSize index);

/** 
 * Gets the pattern sequence type option count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              Pattern sequence type option count.
 */
GoFx(kSize) GoSetup_PatternSequenceTypeOptionCount(GoSetup setup, GoRole role);

/** 
 * Gets the pattern sequence type option at the given index.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @param   index       The index with which to retrieve a sequence type option.
 * @return              A sequence type option value.
 * @see                 GoSetup_PatternSequenceOptionCount
 */
GoFx(GoPatternSequenceType) GoSetup_PatternSequenceTypeOptionAt(GoSetup setup, GoRole role, kSize index);

/** 
 * Gets the pattern sequence type.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              The pattern sequence type.
 */
GoFx(GoPatternSequenceType) GoSetup_PatternSequenceType(GoSetup setup, GoRole role);

/** 
 * Sets the pattern sequence type.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @param   role        Determines which sensor to apply changes to. 
 * @param   type        The pattern sequence type to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoSetup_SetPatternSequenceType(GoSetup setup, GoRole role, GoPatternSequenceType type);

/** 
 * Returns a boolean value representing whether the pattern sequence type is used.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoSetup_PatternSequenceTypeUsed(GoSetup setup, GoRole role);

/** 
 * Gets the current pattern sequence count.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @param   role        Determines which device to retrieve the value from.
 * @return              The current pattern sequence count.
 */
GoFx(kSize) GoSetup_PatternSequenceCount(GoSetup setup, GoRole role);


/** 
 * Gets the layout configuration module.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Layout configuration module.            
 */
GoFx(GoLayout) GoSetup_Layout(GoSetup setup);

/**
 * Gets the profile generation module, used for profile generation configuration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @return              Profile generation configuration module.
 */
GoFx(GoProfileGeneration) GoSetup_ProfileGeneration(GoSetup setup);

/** 
 * Gets the surface generation module, used for surface generation configuration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Surface generation configuration module.            
 */
GoFx(GoSurfaceGeneration) GoSetup_SurfaceGeneration(GoSetup setup);

/** 
 * Gets the part detection module, used for part detection configuration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   setup       GoSetup object.
 * @return              Part detection configuration module.            
 */
GoFx(GoPartDetection) GoSetup_PartDetection(GoSetup setup);

/** 
 * Gets the part matching module, used for part matching configuration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.2.4.7
 * @param   setup       GoSetup object.
 * @return              Part matching configuration module.            
 */
GoFx(GoPartMatching) GoSetup_PartMatching(GoSetup setup);

/** 
 * Gets the material acquisition module, used for material acquisition configuration.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.1.3.106
 * @param   setup       GoSetup object.
 * @param   role        The device role whose material settings object to return.
 * @return              Material acquisition configuration module.
 */
GoFx(GoMaterial) GoSetup_Material(GoSetup setup, GoRole role);

kEndHeader()
#include <GoSdk/GoSetup.x.h>

#endif
