/** 
 * @file    kSemaphore.h
 * @brief   Declares the kSemaphore class. 
 *
 * @internal
 * Copyright (C) 2010-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SEMAPHORE_H
#define K_API_SEMAPHORE_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kSemaphore
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a semaphore.
 */
//typedef kObject kSemaphore;      --forward-declared in kApiDef.x.h 

/** 
 * Constructs a semaphore object.
 *
 * @public                  @memberof kSemaphore
 * @param   semaphore       Destination for the constructed object handle.
 * @param   initialCount    Initial value of the semaphore.
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSemaphore_Construct(kSemaphore* semaphore, kSize initialCount, kAlloc allocator); 

/** 
 * Increments the semaphore.
 *
 * @public                  @memberof kSemaphore
 * @param   semaphore       Semaphore object.
 * @return                  Operation status. 
 */
kFx(kStatus) kSemaphore_Post(kSemaphore semaphore); 

/** 
 * Waits until the semaphore can be decremented or the timeout interval has elapsed.
 *
 * kERROR_TIMEOUT is returned if the timeout elapses before the semaphore can be decremented. 
 *
 * @public                  @memberof kSemaphore
 * @param   semaphore       Semaphore object.
 * @param   timeout         Timeout in microseconds, or kINFINITE to wait indefinitely.
 * @return                  Operation status. 
 */
kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout); 

kEndHeader()

#include <kApi/Threads/kSemaphore.x.h>

#endif
