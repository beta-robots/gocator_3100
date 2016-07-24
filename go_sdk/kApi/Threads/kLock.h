/** 
 * @file    kLock.h
 * @brief   Declares the kLock class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LOCK_H
#define K_API_LOCK_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kLock
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a recursive mutual exclusion lock.
 */

/** 
 * Constructs a lock object.
 *
 * @public              @memberof kLock
 * @param   lock        Destination for the constructed object handle.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_Construct(kLock* lock, kAlloc allocator);

/** 
 * Blocks until exclusive ownership of the lock is acquired.
 *
 * kLock ownership is recursive - the lock is relinquished when the number of calls 
 * to kLock_Exit equals the number of calls to kLock_Enter. 
 *
 * @public              @memberof kLock
 * @param   lock        Lock object. 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_Enter(kLock lock); 

/** 
 * Relinquishes ownership of the lock.
 *
 * kLock ownership is recursive - the lock is relinquished when the number of calls 
 * to kLock_Exit equals the number of calls to kLock_Enter. 
 *
 * @public              @memberof kLock
 * @param   lock        Lock object. 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_Exit(kLock lock); 

kEndHeader()

#include <kApi/Threads/kLock.x.h>

#endif
