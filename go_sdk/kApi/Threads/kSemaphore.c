/** 
 * @file    kSemaphore.c
 *
 * @internal
 * Copyright (C) 2010-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kSemaphore.h>

kBeginClass(k, kSemaphore, kObject)
    kAddVMethod(kSemaphore, kObject, VRelease)
kEndClass()

kFx(kStatus) kSemaphore_Construct(kSemaphore* semaphore, kSize initialCount, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kSemaphore); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, semaphore)); 

    if (!kSuccess(status = kSemaphore_Init(*semaphore, type, initialCount, alloc)))
    {
        kAlloc_FreeRef(alloc, semaphore); 
    }

    return status; 
} 

#if defined(K_WINDOWS)

kFx(kStatus) kSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kSemaphoreClass* obj = semaphore; 
    
    kCheck(kObject_Init(semaphore, type, allocator)); 

    if (kIsNull(obj->handle = CreateSemaphore(kNULL, (k32u)initialCount, kSEMAPHORE_MAX_VALUE, kNULL)))
    {
        kObject_VRelease(semaphore); 
        return kERROR; 
    }

    return kOK; 
}

kFx(kStatus) kSemaphore_VRelease(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    
    kCheck(CloseHandle(obj->handle));
    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 

    kCheck(ReleaseSemaphore(obj->handle, 1, kNULL)); 

    return kOK;
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    DWORD osTimeout = (DWORD) kTimeToKernelTime(timeout); 

    switch (WaitForSingleObject(obj->handle, osTimeout))
    {
        case WAIT_OBJECT_0:     return kOK; 
        case WAIT_TIMEOUT:      return kERROR_TIMEOUT; 
        default:                return kERROR; 
    }           
}

#elif defined(K_DSP_BIOS)

kFx(kStatus) kSemaphore_ConstructAttach(kSemaphore* semaphore, SEM_Handle handle, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kSemaphore); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, semaphore)); 

    if (!kSuccess(status = kSemaphore_InitAttach(*semaphore, type, handle, alloc)))
    {
        kAlloc_FreeRef(alloc, semaphore); 
    }

    return status; 
} 
 
kFx(kStatus) kSemaphore_InitAttach(kSemaphore semaphore, kType type, SEM_Handle handle, kAlloc alloc)
{
    kSemaphoreClass* obj = semaphore; 

    kCheck(kObject_Init(semaphore, type, alloc)); 

    obj->handle = handle; 
   
    return kOK; 
}

kFx(kStatus) kSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kSemaphoreClass* obj = semaphore; 
    SEM_Attrs attributes = SEM_ATTRS;       // DSP BIOS defaults

    kCheck(kObject_Init(semaphore, type, allocator)); 

    obj->handle = kNULL; 
   
    if (kIsNull(obj->handle = SEM_create((k32u)initialCount, &attributes)))
    {
        kObject_VRelease(semaphore); 
        return kERROR_OS; 
    }

    return kOK; 
}

kFx(kStatus) kSemaphore_VRelease(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    
    if (!kIsNull(obj->handle))
    {
        SEM_delete(obj->handle);
    }

    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 

    SEM_post(obj->handle); 

    return kOK;
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    k32u osTimeout = (k32u) kTimeToKernelTime(timeout); 

    return SEM_pend(obj->handle, osTimeout) ? kOK : kERROR_TIMEOUT; 
}

kFx(kStatus) kSemaphore_SetHandle(kSemaphore semaphore, SEM_Handle handle)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 

    obj->handle = handle; 

    return kOK; 
}

kFx(SEM_Handle) kSemaphore_Handle(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 

    return obj->handle; 
}

#elif defined(K_VX_KERNEL)

kFx(kStatus) kSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kSemaphoreClass* obj = semaphore; 
    
    kCheck(kObject_Init(semaphore, type, allocator)); 
    
    kInitFields_(kSemaphore, semaphore); 

    obj->id = semCCreate(SEM_Q_PRIORITY, (int)initialCount); 

    if (obj->id == SEM_ID_NULL)
    {
        kObject_VRelease(semaphore); 
        return kERROR; 
    }

    return kOK; 
}

kFx(kStatus) kSemaphore_VRelease(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    
    semDelete(obj->id); 
    
    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 

    return (semGive(obj->id) == OK) ? kOK : kERROR_OS; 
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    _Vx_ticks_t osTimeout = (_Vx_ticks_t) kTimeToKernelTime(timeout); 

    return (semTake(obj->id, osTimeout) == OK) ? kOK : kERROR_TIMEOUT;            
}

#elif defined(K_POSIX)

kFx(kStatus) kSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kSemaphoreClass* obj = semaphore; 

    kCheck(kObject_Init_(semaphore, type, allocator));     
        
    if (pthread_cond_init(&obj->canDecrement, kNULL) != 0)
    {
        kObject_VRelease_(semaphore); 
        return kERROR; 
    }
    
    if (!kSuccess(kLock_Construct(&obj->lock, allocator)))
    { 
        pthread_cond_destroy(&obj->canDecrement);
        kObject_VRelease_(semaphore); 
        return kERROR; 
    }
    
    obj->count = (kSSize)initialCount; 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_VRelease(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    
    kCheck(pthread_cond_destroy(&obj->canDecrement) == 0);
    kCheck(kObject_Destroy(obj->lock)); 
    
    kCheck(kObject_VRelease_(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 

    kCheck(kLock_Enter(obj->lock)); 
    {
        obj->count++;
    }
    kCheck(kLock_Exit(obj->lock)); 
    
    kCheck(pthread_cond_signal(&obj->canDecrement) == 0); 

    return kOK;
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kSemaphoreClass* obj = kSemaphore_Cast_(semaphore); 
    kLockClass* lockObj = obj->lock; 
    struct timespec tm; 
    int result = 0; 

    kCheck(kFormatTimeout(timeout, &tm)); 

    kCheck(kLock_Enter(obj->lock)); 
    {
        while ((obj->count < 1) && (result == 0))
        {
            if (timeout == kINFINITE)
            {
                result = pthread_cond_wait(&obj->canDecrement, &lockObj->handle);
            }
            else
            {
                result = pthread_cond_timedwait(&obj->canDecrement, &lockObj->handle, &tm);
            }
            
            if ((result != 0) && (result == EINTR))
            {
                result = 0; 
            }
        }
        
        if (result == 0)
        {
            obj->count--; 
        }
    }
    kCheck(kLock_Exit(obj->lock)); 

    if (result == 0)
    {
        return kOK; 
    }
    else if (result == ETIMEDOUT)
    {
        return kERROR_TIMEOUT; 
    }
    else
    {
        return kERROR; 
    }
}

#endif
