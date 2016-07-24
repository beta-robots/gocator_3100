/** 
 * @file    GoAnalog.h
 * @brief   Declares the GoAnalog class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_ANALOG_H
#define GO_SDK_ANALOG_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoAnalog
 * @extends kObject
 * @ingroup GoSdk-Analog
 * @brief   Represents Analog output settings.
 */
typedef kObject GoAnalog;


/** 
 * Gets the event which triggers this output to fire
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              The output event.            
 */
GoFx(GoAnalogEvent) GoAnalog_Event(GoAnalog analog); 

/** 
 * Set the event which triggers this output to fire
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   event       The selected output event.
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetEvent(GoAnalog analog, GoAnalogEvent event); 

/** 
 * Gets the number of measurement value source options.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Count of source options.
 */
GoFx(kSize) GoAnalog_OptionCount(GoAnalog analog); 

/** 
 * Gets the measurement value source option at the specified index.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   index       Source option index. 
 * @return              Source option.            
 */
GoFx(k32u) GoAnalog_OptionAt(GoAnalog analog, kSize index); 

/** 
 * Selects a source type and source identifier for output.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   sourceId    Output source identifier. 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetSource(GoAnalog analog, k32u sourceId); 

/** 
 * Gets the selected source identifier.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Selected source identifier.            
 */
GoFx(k32u) GoAnalog_Source(GoAnalog analog); 

/** 
 * Clears the currently selected source identifier.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_ClearSource(GoAnalog analog); 

/** 
 * Gets the minimum valid value for CurrentMin, CurrentMax and CurrentInvalid settings.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Minimum valid current value (mA).             
 */
GoFx(k64f) GoAnalog_CurrentLimitMin(GoAnalog analog); 

/** 
 * Gets the maximum valid value for CurrentMin, CurrentMax and CurrentInvalid settings.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum valid current value (mA).             
 */
GoFx(k64f) GoAnalog_CurrentLimitMax(GoAnalog analog); 
 
/** 
 * Sets the minimum current output level. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   min         Minimum current output level (mA). 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetCurrentMin(GoAnalog analog, k64f min); 

/** 
 * Gets the minimum current output level. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Minimum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentMin(GoAnalog analog); 

/** 
 * Gets the minimum allowable current output level to be set for the minimum current. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentMinLimitMin(GoAnalog analog);

/** 
 * Gets the maximum allowable current output level to be set for the minimum current. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentMinLimitMax(GoAnalog analog);

/** 
 * Sets the maximum current output level. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   max         Maximum current output level (mA). 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetCurrentMax(GoAnalog analog, k64f max); 

/** 
 * Gets the maximum current output level. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentMax(GoAnalog analog); 

/** 
 * Gets the minimum allowable current output level to be set for the maximum current. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentMaxLimitMin(GoAnalog analog);

/** 
 * Gets the maximum allowable current output level to be set for the maximum current. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentMaxLimitMax(GoAnalog analog);


/** 
 * Enables the current output level associated with an invalid measurement.  
 * When this is disabled, the output value will be held constant on an invalid measurement
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   enable      kTRUE to enable, or kFALSE to disable.
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_EnableCurrentInvalid(GoAnalog analog, kBool enable); 

/** 
 * Gets the status of the invalid current enabled option
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Whether invalid current is enabled for this output.            
 */ 
GoFx(kBool) GoAnalog_CurrentInvalidEnabled(GoAnalog analog);  

/** 
 * Sets the current output level associated with an invalid measurement.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   invalid     Invalid output current (mA). 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetCurrentInvalid(GoAnalog analog, k64f invalid); 

/** 
 * Gets the current output level associated with an invalid measurement.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Invalid current level (mA).             
 */
GoFx(k64f) GoAnalog_CurrentInvalid(GoAnalog analog);  

/** 
 * Gets the maximum allowable current output level to be set for the invalid value current. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentInvalidLimitMax(GoAnalog analog);

/** 
 * Gets the minimum allowable current output level to be set for the invalid value current. 
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum current output (mA).             
 */
GoFx(k64f) GoAnalog_CurrentInvalidLimitMin(GoAnalog analog);

/** 
 * Sets the measurement value associated with the minimum output current value.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   min         Minimum measurement value (units are measurement dependent). 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetDataScaleMin(GoAnalog analog, k64f min); 

/** 
 * Gets the measurement value associated with the minimum output current value.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Minimum measurement value (units are measurement dependent).            
 */
GoFx(k64f) GoAnalog_DataScaleMin(GoAnalog analog);  

/** 
 * Sets the measurement value associated with the maximum output current value.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   max         Maximum measurement value (units are measurement dependent). 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetDataScaleMax(GoAnalog analog, k64f max);

/** 
 * Gets the measurement value associated with the maximum output current value.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              Maximum measurement value (units are measurement dependent).             
 */
GoFx(k64f) GoAnalog_DataScaleMax(GoAnalog analog); 

/** 
 * Sets the delay from exposure until output is triggered, in units based on GoDomain
 * mm when GoDomain is distance, uS when GoDomain is time.
 *
 * The delay is ignored when GoDomain is Immediate or when output is Software triggered.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   delay       The delay (uS or mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_SetDelay(GoAnalog analog, k64s delay); 

/** 
 * Gets the output delay.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              The output delay (uS or mm).             
 */
GoFx(k64s) GoAnalog_Delay(GoAnalog analog);

/** 
 * Sets the output delay domain.
 *
 * @public                  @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog          GoAnalog object.
 * @param   delayDomain     Output delay domain.
 * @return                  The output delay domain (time or encoder value).
 */
GoFx(kStatus) GoAnalog_SetDelayDomain(GoAnalog analog, GoOutputDelayDomain delayDomain);

/** 
 * Gets the output delay domain.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              The output delay domain.
 */
GoFx(GoOutputDelayDomain) GoAnalog_DelayDomain(GoAnalog analog);

/** 
 * Enables or disables the scheduler for this output.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @param   enabled     kTRUE to enable, kFALSE to disable 
 * @return              Operation status.            
 */
GoFx(kStatus) GoAnalog_EnableSchedule(GoAnalog analog, kBool enabled );

/** 
 * Gets the enabled state of the scheduler for this output.
 *
 * @public              @memberof GoAnalog
 * @version             Introduced in firmware 4.0.10.27
 * @param   analog      GoAnalog object.
 * @return              kTRUE if the scheduler is enabled; kFALSE otherwise.            
 */
GoFx(kBool) GoAnalog_ScheduleEnabled(GoAnalog analog); 

kEndHeader()
#include <GoSdk/Outputs/GoAnalog.x.h>

#endif