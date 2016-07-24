/** 
 * @file    kThread.h
 * @brief   Declares the kThread class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_THREAD_H
#define K_API_THREAD_H

#include <kApi/kApiDef.h>

kBeginHeader()

/** Thread entry-point signature; used by kThread_Start.  */
typedef kStatus(kCall* kThreadFx)(kPointer context); 

/**
 * @class   kThread
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a thread.
 */
//typedef kObject kThread;        --forward-declared in kApiDef.x.h 

/** 
 * Causes the current thread to yield control for approximately the specified duration.
 * 
 * The duration specified in this function is approximate. Sleep duration is governed by the underlying 
 * operating system, and may be affected by kernel timer resolution or thread scheduling policies. In general, 
 * if CPU utilization is low (quiet system), then sleep duration is most often +/- 1 kernel timer tick. At the 
 * time of this writing, most desktop operating systems have a 10 ms kernel timer resolution, while most 
 * embedded systems have a 1-10 ms kernel timer resolution. 
 * 
 * If sleep duration must be constrained to be <em>at least</em> the specified duration, consider using 
 * kThread_SleepAtLeast. 
 *
 * @public              @memberof kThread
 * @param   duration    Approximate time to sleep, in microseconds
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Sleep(k64u duration);   

/**
* Causes the current thread to yield control for at least the specified duration.
*
* This function sleeps until <em>at least</em> the specified duration has elapsed. Consider using this function 
* when a sleep operation must provide a guaranteed minimum hold time. 
* 
* @public              @memberof kThread
* @param   duration    Minimum time to sleep, in microseconds
* @return              Operation status.
*/
kFx(kStatus) kThread_SleepAtLeast(k64u duration);

/** 
 * Constructs a kThread object.
 *
 * @public              @memberof kThread
 * @param   thread      Destination for the constructed object handle. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Construct(kThread* thread, kAlloc allocator);

/** 
 * Begins executing a thread using the specified callback function.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @param   function    The thread entry function. 
 * @param   context     An argument to be passed to the thread entry function.
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context); 

/** 
 * Blocks until the thread exits, or until a timeout occurs.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @param   timeout     Timeout in microseconds, or kINFINITE to wait indefinitely.
 * @param   exitCode    Receives the return value from the thread function.
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode); 

kEndHeader()

#include <kApi/Threads/kThread.x.h>

#endif
