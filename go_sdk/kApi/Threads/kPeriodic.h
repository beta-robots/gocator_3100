/** 
 * @file    kPeriodic.h
 * @brief   Declares the kPeriodic class. 
 *
 * @internal
 * Copyright (C) 2010-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PERIODIC_H
#define K_API_PERIODIC_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kPeriodic
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Provides a periodic function call. 
 */
//typedef kObject kPeriodic;           --forward-declared in kApiDef.x.h 

/** Defines the signature of a callback function to receive timer notifications. */
typedef kStatus(kCall *kPeriodicElapsedFx) (kPointer context, kPeriodic timer); 

/** 
 * Constructs a kPeriodic object. 
 *
 * @public              @memberof kPeriodic
 * @param   timer       Receives a handle to the constructed object. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kPeriodic_Construct(kPeriodic* timer, kAlloc allocator); 

/** 
 * Starts callbacks at the specified period.
 * 
 * It is safe to call this function from within a timer callback.  It is valid to call Start multiple
 * times without first calling Stop, in order to change the timer period or callback function. 
 * 
 * Each subsequent timer period after the first callback is measured relative to the end 
 * of the previous timer callback. 
 *
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @param   period      Callback period, in microseconds. 
 * @param   onElapsed   Callback function.
 * @param   context     User context handle supplied to the callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kPeriodic_Start(kPeriodic timer, k64u period, kPeriodicElapsedFx onElapsed, kPointer context); 

/** 
 * Stops timer callbacks.
 * 
 * It is guaranteed that callbacks are stopped when this function returns. Deadlock can occur 
 * if a timer callback is in progress and is blocked indefinitely. It is safe to call this function 
 * from within a timer callback. 
 *
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @return              Operation status. 
 */
kFx(kStatus) kPeriodic_Stop(kPeriodic timer); 

/** 
 * Reports whether periodic timer callbacks are currently enabled. 
 * 
 * Deadlock can occur if a timer callback is in progress and is blocked indefinitely when this
 * function is called. It is safe to call this function from within a timer callback. 
 * 
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @return              Operation status. 
 */
kFx(kBool) kPeriodic_Enabled(kPeriodic timer); 

kEndHeader()

#include <kApi/Threads/kPeriodic.x.h>

#endif
