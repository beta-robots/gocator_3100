/** 
 * @file    kObjectPool.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_OBJECT_POOL_X_H
#define K_API_OBJECT_POOL_X_H

kBeginHeader()

typedef struct kObjectPoolVTable
{   
    kStatus (kCall* VReclaim)(kObjectPool pool, kObject object);
} kObjectPoolVTable;

kDeclareInterface(k, kObjectPool, kNull) 

kFx(kStatus) kObjectPool_VReclaim(kObjectPool pool, kObject object);

#define kObjectPool_VTable_(P)              kCastIVTable_(kObjectPool, P)

#define kObjectPool_Reclaim_(P, OBJ)        (kObjectPool_VTable_(P)->VReclaim(P, OBJ))

kEndHeader()

#endif
