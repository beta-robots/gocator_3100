/** 
 * @file    kLock.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LOCK_X_H
#define K_API_LOCK_X_H

kBeginHeader()

kDeclareClass(k, kLock, kObject)

#if defined(K_PLATFORM)

#if defined(K_WINDOWS)

#   define kLockPlatformFields()            \
        CRITICAL_SECTION handle;    

#elif defined(K_DSP_BIOS)

#   define kLockPlatformFields()            \
        LCK_Handle handle;

#elif defined(K_VX_KERNEL)

#   define kLockPlatformFields()            \
        SEM_ID id;

#else

#   define kLockPlatformFields()            \
        pthread_mutex_t handle;        

#endif

typedef struct kLockClass
{
    kObjectClass base; 
    kLockPlatformFields()
} kLockClass;

kFx(kStatus) kLock_Init(kLock lock, kType type, kAlloc allocator); 
kFx(kStatus) kLock_VRelease(kLock lock); 

#define kLock_Cast_(LOCK)                       (kCastClass_(kLock, LOCK))

#endif

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_LOCK                  kTypeOf(kLock)
#   define kLock_Destroy               kObject_Destroy

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kLock_Construct5(L)             kLock_Construct((L), kNULL)

#endif

kEndHeader()

#endif
