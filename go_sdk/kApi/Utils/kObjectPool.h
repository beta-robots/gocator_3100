/** 
 * @file    kObjectPool.h
 * @brief   Declares the kObjectPool interface. 
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_OBJECT_POOL_H
#define K_API_OBJECT_POOL_H

#include <kApi/kApiDef.h>  

kBeginHeader()

/**
 * @interface kObjectPool
 * @ingroup   kApi
 * @brief     Supports reclaiming objects upon destruction.
 * @see       kObject_SetPool, kObject_Destroy, kObject_Dispose
 */
//typedef kObject kObjectPool;   --forward-declared in kApiDef.x.h

/** 
 * Receives an object just prior to its destruction.
 *
 * @public          @memberof kObjectPool
 * @param   pool    Pool object. 
 * @param   object  Object to be reclaimed. 
 * @return          Operation status.
 * @see             kObject_SetPool, kObject_Destroy, kObject_Dispose
 */
kFx(kStatus) kObjectPool_Reclaim(kObjectPool pool, kObject object);

kEndHeader()

#include <kApi/Utils/kObjectPool.x.h>

#endif
