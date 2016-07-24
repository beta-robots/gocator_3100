/** 
 * @file    kUserAlloc.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_USER_ALLOC_X_H
#define K_API_USER_ALLOC_X_H

kBeginHeader()

typedef struct kUserAllocClass
{
    kAllocClass base; 
    kApiMemAllocFx allocFx;         //User allocation function.
    kApiMemFreeFx freeFx;           //User free function. 
    kPointer provider;              //User context pointer. 
} kUserAllocClass;

kDeclareClass(k, kUserAlloc, kAlloc)

kFx(kStatus) kUserAlloc_Init(kUserAlloc object, kType type, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc alloc); 
kFx(kStatus) kUserAlloc_VRelease(kUserAlloc object); 

kFx(kStatus) kUserAlloc_VGet(kUserAlloc object, kSize size, void* mem); 
kFx(kStatus) kUserAlloc_VFree(kUserAlloc object, void* mem); 

#define kUserAlloc_Cast_(USER_ALLOC)              kCastClass_(kUserAlloc, USER_ALLOC)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_HEAP          kTypeOf(kUserAlloc)                

#endif

kEndHeader()

#endif
