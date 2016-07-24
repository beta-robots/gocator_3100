/** 
 * @file    kSemaphore.x.h
 *
 * @internal
 * Copyright (C) 2010-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SEMAPHORE_X_H
#define K_API_SEMAPHORE_X_H

kBeginHeader()

kDeclareClass(k, kSemaphore, kObject)

#if defined(K_PLATFORM)

#if defined(K_WINDOWS)

#   define kSEMAPHORE_MAX_VALUE             (0x7FFFFFFF)

#   define kSemaphorePlatformFields()       \
        HANDLE handle; 

#elif defined(K_DSP_BIOS)

#   define kSemaphorePlatformFields()       \
        SEM_Handle handle;

#   define kSemaphore_Post_(S)              \
        (SEM_post(kSemaphore_(S)->handle), kOK)

kFx(kStatus) kSemaphore_ConstructAttach(kSemaphore* semaphore, SEM_Handle handle, kAlloc allocator); 
kFx(kStatus) kSemaphore_InitAttach(kSemaphore semaphore, kType type, SEM_Handle handle, kAlloc alloc); 
kFx(kStatus) kSemaphore_SetHandle(kSemaphore semaphore, SEM_Handle handle); 
kFx(SEM_Handle) kSemaphore_Handle(kSemaphore semaphore); 

#elif defined(K_VX_KERNEL)

#   define kSemaphorePlatformFields()       \
        SEM_ID id;


#elif defined(K_POSIX)

#include <kApi/Threads/kLock.h>

#   define kSemaphorePlatformFields()       \
        kSSize count;                       \
        pthread_cond_t canDecrement;        \
        kLock lock;

#endif

typedef struct kSemaphoreClass
{
    kObjectClass base; 
    kSemaphorePlatformFields()
} kSemaphoreClass; 

kFx(kStatus) kSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator); 
kFx(kStatus) kSemaphore_VRelease(kSemaphore semaphore); 

#define kSemaphore_(S)                   (kCast_(kSemaphoreClass*, S))
#define kSemaphore_Cast_(S)              (kCastClass_(kSemaphore, S))

#endif

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_SEMAPHORE             kTypeOf(kSemaphore)
#   define kSemaphore_Destroy          kObject_Destroy

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kSemaphore_Construct5(S, I)      kSemaphore_Construct((S), (I), kNULL)

#endif

kEndHeader()

#endif
