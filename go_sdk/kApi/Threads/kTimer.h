/** 
 * @file    kTimer.h
 * @brief   Declares the kTimer class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TIMER_H
#define K_API_TIMER_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kTimer
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents an interval timer. 
 */
//typedef kObject kTimer;     --forward-declared in kApiDef.x.h 

/** 
 * Provides the current time in microseconds. 
 *
 * This function provides a time value in microseconds; successive calls to this function can be 
 * used to measure a time interval. 
 *
 * @public              @memberof kTimer
 * @return              Current time, in microseconds. 
 */
kFx(k64u) kTimer_Now();

/** 
 * Constructs a timer object.
 *
 * @public              @memberof kTimer
 * @param   timer       Destination for the constructed object handle.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kTimer_Construct(kTimer* timer, kAlloc allocator); 

/** 
 * Starts the timer.
 *
 * If the timer will be used to count up, rather than count down, totalTime can be zero.
 *
 * @public              @memberof kTimer
 * @param   timer       Timer object. 
 * @param   totalTime   Total time to count down, in microseconds. 
 * @return              Operation status. 
 */
kFx(kStatus) kTimer_Start(kTimer timer, k64u totalTime); 

/** 
 * Stops the timer.
 *
 * kTimer_Elapsed can be used to report the time between start and stop. 
 * 
 * The use of kTimer_Stop is strictly optional; it is not necessary to call kTimer_Stop for each 
 * invocation of kTimer_Start. 
 *
 * @public              @memberof kTimer
 * @param   timer       Timer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kTimer_Stop(kTimer timer); 

/** 
 * Reports whether a timer has been started.
 *
 * @public              @memberof kTimer
 * @param   timer       Timer object. 
 * @return              kTRUE if the timer was started.
 */
kFx(kBool) kTimer_IsStarted(kTimer timer);

/** 
 * Reports whether a count-down timer has expired.
 *
 * @public              @memberof kTimer
 * @param   timer       Timer object. 
 * @return              kTRUE if the timer has expired. 
 */
kFx(kBool) kTimer_IsExpired(kTimer timer);

/** 
 * Reports the duration, in microseconds, for which the timer has been running.
 *
 * @public              @memberof kTimer
 * @param   timer       Timer object. 
 * @return              Elapsed time, in microseconds. 
 */
kFx(k64u) kTimer_Elapsed(kTimer timer); 

/** 
 * Reports the remaining time, in microseconds, for a countdown timer.
 *
 * @public              @memberof kTimer
 * @param   timer       Timer object. 
 * @return              Remaining time, in microseconds. 
 */
kFx(k64u) kTimer_Remaining(kTimer timer); 

kEndHeader()

#include <kApi/Threads/kTimer.x.h>

#endif
