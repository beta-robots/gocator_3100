/** 
 * @file    GoMeasurement.h
 * @brief   Declares the GoMeasurement class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_MEASUREMENT_H
#define GO_MEASUREMENT_H

#include <GoSdk/GoSdkDef.h>

kBeginHeader()

/**
 * @class   GoMeasurement
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents the base class for a tool measurement or script output.
 */
typedef kObject GoMeasurement; 


/** 
 * Returns whether or not the given measurement has a valid ID assigned to it.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  kTRUE if there is an ID; kFALSE otherwise.            
 */
GoFx(kBool) GoMeasurement_HasId(GoMeasurement measurement);

/** 
 * Clears the assigned ID for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_ClearId(GoMeasurement measurement);

/** 
 * Sets an ID number for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    id             The ID value to set for the measurement.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetId(GoMeasurement measurement, k32u id);

/** 
 * Gets the ID for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  The ID value if there is one assigned.  Otherwise, -1 is returned.            
 */
GoFx(k32s) GoMeasurement_Id(GoMeasurement measurement);

/** 
 * Gets the name for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  A character array pointer for the measurement name.
 */
GoFx(const kChar*) GoMeasurement_Name(GoMeasurement measurement);

/** 
 * Sets the name for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    name           The name to assign to the measurement.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetName(GoMeasurement measurement, const kChar* name);

/** 
 * Returns the source tool of the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  A pointer to the source tool for the measurement.
 */
GoFx(kObject) GoMeasurement_SourceTool(GoMeasurement measurement);

/** 
 * Enables the given measurement for output.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    enable         Set to kTRUE to enable the measurement, kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_Enable(GoMeasurement measurement, kBool enable);

/** 
 * Returns a boolean value representing whether the given measurement is enabled.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  kTRUE if enabled; kFALSE otherwise.            
 */
GoFx(kBool) GoMeasurement_Enabled(GoMeasurement measurement);

/** 
 * Sets the minimum decision value for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    min            The minimum decision value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetDecisionMin(GoMeasurement measurement, k64f min);

/** 
 * Gets the minimum decision value for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  The minimum decision value.            
 */
GoFx(k64f) GoMeasurement_DecisionMin(GoMeasurement measurement);

/** 
 * Sets the maximum decision value for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    max            The maximum decision value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetDecisionMax(GoMeasurement measurement, k64f max);

/** 
 * Gets the maximum decision value for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  The maximum decision value.            
 */
GoFx(k64f) GoMeasurement_DecisionMax(GoMeasurement measurement);

/** 
 * Sets measurement value hold for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    enable         kTRUE to enable measurement value hold, kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_EnableHold(GoMeasurement measurement, kBool enable);

/** 
 * Returns a boolean value representing the current state of measurement value hold.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  kTRUE if enabled and kFALSE if disabled.            
 */
GoFx(kBool) GoMeasurement_HoldEnabled(GoMeasurement measurement);

/** 
 * Sets measurement value smoothing for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    enable         kTRUE to enable measurement value smoothing, kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_EnableSmoothing(GoMeasurement measurement, kBool enable);

/** 
 * Returns a boolean value representing the current state of measurement value smoothing.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  kTRUE if enabled and kFALSE if disabled.            
 */
GoFx(kBool) GoMeasurement_SmoothingEnabled(GoMeasurement measurement);

/** 
 * Sets the measurement value smoothing window for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    value          The intended size of the smoothing window.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetSmoothingWindow(GoMeasurement measurement, k64s value);

/** 
 * Returns the current measurement value smoothing window size.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  Smoothing window size.
 */
GoFx(k64s) GoMeasurement_SmoothingWindow(GoMeasurement measurement);

/** 
 * Sets the measurement value scaling for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    value          The intended measurement value scaling factor.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetScale(GoMeasurement measurement, k64f value);

/** 
 * Returns the current measurement value scaling factor.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  The measurement value scaling factor.
 */
GoFx(k64f) GoMeasurement_Scale(GoMeasurement measurement);

/** 
 * Sets the measurement value offset for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @param    value          The measurement offset to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMeasurement_SetOffset(GoMeasurement measurement, k64f value);

/** 
 * Returns the measurement value offset.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  Measurement value offset.
 */
GoFx(k64f) GoMeasurement_Offset(GoMeasurement measurement);

/** 
 * Gets the measurement type for the given measurement.
 *
 * @public                  @memberof GoMeasurement
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoMeasurement object.
 * @return                  The measurement value type enumerator value.            
 */
GoFx(GoMeasurementType) GoMeasurement_Type(GoMeasurement measurement);

kEndHeader()
#include <GoSdk/Tools/GoMeasurement.x.h>

#endif
