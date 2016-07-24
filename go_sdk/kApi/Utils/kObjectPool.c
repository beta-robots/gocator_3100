/** 
 * @file    kObjectPool.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kObjectPool.h>

kBeginInterface(k, kObjectPool, kNull) 
    //interface methods
    kAddVMethod(kObjectPool, kObjectPool, VReclaim)
kEndInterface() 

kFx(kStatus) kObjectPool_Reclaim(kObjectPool pool, kObject object)
{
    kAssertType(pool, kObjectPool); 
    
    return kObjectPool_Reclaim_(pool, object); 
}

kFx(kStatus) kObjectPool_VReclaim(kObjectPool pool, kObject object)
{
    return kERROR_UNIMPLEMENTED; 
}

