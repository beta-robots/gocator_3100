/** 
 * @file    GoDigital.h
 * @brief   Declares the GoDigital class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DIGITAL_H
#define GO_SDK_DIGITAL_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()


/**
 * @class   GoDigital
 * @extends kObject
 * @ingroup GoSdk-Digital
 * @brief   Represents Digital output settings.
 */
typedef kObject GoDigital;

/** 
 * Sets the event which triggers this output to fire.
 *
 * @public              @memberof GoDigital
 * @version             Introduced in firmware 4.0.10.27
 * @param   digital     GoDigital object.
 * @param   event       The output event.
 * @return              Operation status.            
 */
GoFx(kStatus) GoDigital_SetEvent(GoDigital digital, GoDigitalEvent event);

/** 
 * Gets the event which triggers this output to fire
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  The output event.            
 */
GoFx(GoDigitalEvent) GoDigital_Event(GoDigital digital);

/** 
 * Gets the number of available decision source options.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Count of decision source options.            
 */
GoFx(kSize) GoDigital_OptionCount(GoDigital digital); 

/** 
 * Gets the decision source option at the specified index.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   index           Source option index. 
 * @return                  Source option.            
 */
GoFx(k32u) GoDigital_OptionAt(GoDigital digital, kSize index); 

/** 
 * Gets the number of decision sources that are currently selected for determining pass/fail state.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Count of selected sources.            
 */
GoFx(kSize) GoDigital_SourceCount(GoDigital digital); 

/** 
 * Gets the identifier of the selected output source at the specified index.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   index           Selected source index.
 * @return                  Source identifier.            
 */
GoFx(k32u) GoDigital_SourceAt(GoDigital digital, kSize index); 

/** 
 * Selects the specified decision source for use in determining pass/fail status.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   sourceId        Index of the source to be added/selected.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_AddSource(GoDigital digital, k32u sourceId); 

/** 
 * Removes (deselects) the decision source at the specified index.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   index           Index of the source to be removed.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_RemoveSource(GoDigital digital, kSize index); 

/** 
 * Removes all selected decision sources.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_ClearSources(GoDigital digital); 

/** 
 * Sets the pass/fail mode for the digital output.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   pass            Pass/fail mode. 
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_SetPassMode(GoDigital digital, GoDigitalPass pass);

/** 
 * Gets the pass/fail mode for the digital output.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Current pass/fail mode.            
 */
GoFx(GoDigitalPass) GoDigital_PassMode(GoDigital digital);

/** 
 * Gets the minimum valid value for the Pulse Width setting.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Minimum valid pulse width value (microseconds).             
 */
GoFx(k32u) GoDigital_PulseWidthLimitMin(GoDigital digital); 

/** 
 * Gets the maximum valid value for the Pulse Width setting.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Maximum valid pulse width value (microseconds).             
 */
GoFx(k32u) GoDigital_PulseWidthLimitMax(GoDigital digital); 

/** 
 * Sets the width of digital output pulses. 
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   width           Pulse width (microseconds). 
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_SetPulseWidth(GoDigital digital, k32u width); 

/** 
 * Gets the width of digital output pulses. 
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  Pulse width (microseconds).             
 */
GoFx(k32u) GoDigital_PulseWidth(GoDigital digital); 

/** 
 * Sets the signal type of output. 
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   signal          The signal type. 
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_SetSignalType(GoDigital digital, GoDigitalSignal signal); 

/** 
 * Gets the signal type of output.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  The signal type.             
 */
GoFx(GoDigitalSignal) GoDigital_SignalType(GoDigital digital);

/** 
 * Sets the delay from exposure until output is triggered, in units based on GoDomain.
 * mm units when GoDomain is distance, uS units when GoDomain is time. 
 * Ignored when GoDomain is Immediate or when Pass Mode is Software.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   delay           k64s object.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_SetDelay(GoDigital digital, k64s delay); 

/** 
 * Gets the output delay.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  The output delay (us or mm).             
 */
GoFx(k64s) GoDigital_Delay(GoDigital digital);

/** 
 * Sets the output delay domain.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   delayDomain     Output delay domain.
 * @return                  The output delay domain (time or encoder value).
 */

GoFx(kStatus) GoDigital_SetDelayDomain(GoDigital digital, GoOutputDelayDomain delayDomain);

/** 
 * Gets the output delay domain.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  The output delay domain.
 */
GoFx(GoOutputDelayDomain) GoDigital_DelayDomain(GoDigital digital);

/** 
 * Enables or disables the scheduler for this output.
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @param   enabled         kTRUE to enable, kFALSE to disable 
 * @return                  Operation status.            
 */
GoFx(kStatus) GoDigital_EnableSchedule(GoDigital digital, kBool enabled ); 

/** 
 * Gets the enabled state of the scheduler for this output
 *
 * @public                  @memberof GoDigital
 * @version                 Introduced in firmware 4.0.10.27
 * @param   digital         GoDigital object.
 * @return                  kTRUE if the scheduler is enabled.            
 */
GoFx(kBool) GoDigital_ScheduleEnabled(GoDigital digital); 


kEndHeader()
#include <GoSdk/Outputs/GoDigital.x.h>

#endif
