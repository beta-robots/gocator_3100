/** 
 * @file    kLock.c
 * @brief   Declares the kLock class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kLock.h>

kBeginClass(k, kLock, kObject)
    kAddVMethod(kLock, kObject, VRelease)
kEndClass()

kFx(kStatus) kLock_Construct(kLock* lock, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kLock); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, lock)); 

    if (!kSuccess(status = kLock_Init(*lock, type, alloc)))
    {
        kAlloc_FreeRef(alloc, lock); 
    }

    return status; 
} 

#if defined(K_WINDOWS)

kFx(kStatus) kLock_Init(kLock lock, kType type, kAlloc allocator)
{
    kLockClass* obj = lock; 

    kCheck(kObject_Init_(lock, type, allocator)); 

    InitializeCriticalSection(&obj->handle);
    
    return kOK; 
}

kFx(kStatus) kLock_VRelease(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 
    
    DeleteCriticalSection(&obj->handle); 

    kCheck(kObject_VRelease_(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    EnterCriticalSection(&obj->handle);

    return kOK;
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    LeaveCriticalSection(&obj->handle);

    return kOK;
}

#elif defined(K_DSP_BIOS)

kFx(kStatus) kLock_Init(kLock lock, kType type, kAlloc allocator)
{
    kLockClass* obj = lock; 
    LCK_Attrs attributes = { 0 };       // produces default attributes, according to DSP BIOS docs         

    kCheck(kObject_Init_(lock, type, allocator)); 

    if (kIsNull(obj->handle = LCK_create(&attributes)))
    {
        kObject_VRelease(lock); 
        return kERROR_OS; 
    }
    
    return kOK; 
}

kFx(kStatus) kLock_VRelease(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 
    
    if (!kIsNull(obj->handle))
    {
        LCK_delete(obj->handle); 
    }

    kCheck(kObject_VRelease_(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    return LCK_pend(obj->handle, SYS_FOREVER) ? kOK : kERROR_OS; 
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    LCK_post(obj->handle); 

    return kOK;
}

#elif defined(K_VX_KERNEL)

kFx(kStatus) kLock_Init(kLock lock, kType type, kAlloc allocator)
{
    kLockClass* obj = lock; 

    kCheck(kObject_Init_(lock, type, allocator)); 
    
    kInitFields_(kLock, lock); 
    
    obj->id = semMCreate(SEM_Q_PRIORITY); 

    if (obj->id == SEM_ID_NULL)
    {
        kObject_VRelease(lock); 
        return kERROR; 
    }
    
    return kOK; 
}

kFx(kStatus) kLock_VRelease(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 
    
    semDelete(obj->id);

    kCheck(kObject_VRelease_(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    return (semTake(obj->id, K_OS_INFINITE) == OK) ? kOK : kERROR_OS;      
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    return (semGive(obj->id) == OK) ? kOK : kERROR_OS; 
}

#elif defined(K_POSIX)

kFx(kStatus) kLock_Init(kLock lock, kType type, kAlloc allocator)
{
    kLockClass* obj = lock; 
    pthread_mutexattr_t mutexattr; 
    kBool attrInit = kFALSE; 
    kBool mutexInit = kFALSE; 
    kStatus status; 
    
    kCheck(kObject_Init_(lock, type, allocator)); 

    kTry
    {
        kTest(pthread_mutexattr_init(&mutexattr) == 0); 
        attrInit = kTRUE; 

        kTest(pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_RECURSIVE) == 0);

        kTest(pthread_mutex_init(&obj->handle, &mutexattr) == 0);
        mutexInit = kTRUE; 
    }
    kCatchEx(&status)
    {
        if (mutexInit) pthread_mutex_destroy(&obj->handle);
        
        kObject_VRelease(lock); 

        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        if (attrInit) pthread_mutexattr_destroy(&mutexattr); 

        kEndFinallyEx();
    }

    return kOK; 
}

kFx(kStatus) kLock_VRelease(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 
    
    kCheck(pthread_mutex_destroy(&obj->handle) == 0);

    kCheck(kObject_VRelease_(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    kCheck(pthread_mutex_lock(&obj->handle) == 0);

    return kOK;
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kLockClass* obj = kLock_Cast_(lock); 

    kCheck(pthread_mutex_unlock(&obj->handle) == 0);

    return kOK;
}

#endif
