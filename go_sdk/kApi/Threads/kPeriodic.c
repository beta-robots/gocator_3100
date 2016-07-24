/** 
 * @file    kPeriodic.c
 *
 * @internal
 * Copyright (C) 2010-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Threads/kPeriodic.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Threads/kSemaphore.h>

kBeginClass(k, kPeriodic, kObject)
    kAddVMethod(kPeriodic, kObject, VRelease)
kEndClass()

kFx(kStatus) kPeriodic_Construct(kPeriodic* timer, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kPeriodic), timer)); 

    if (!kSuccess(status = kPeriodic_Init(*timer, kTypeOf(kPeriodic),  0, kNULL, 0, alloc)))
    {
        kAlloc_FreeRef(alloc, timer); 
    }

    return status; 
} 

kFx(kStatus) kPeriodic_ConstructEx(kPeriodic* timer, kSize stackSize, const kChar* name, k32s priority, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kPeriodic), timer)); 

    if (!kSuccess(status = kPeriodic_Init(*timer, kTypeOf(kPeriodic), stackSize, name, priority, alloc)))
    {
        kAlloc_FreeRef(alloc, timer); 
    }

    return status; 
} 

kFx(kStatus) kPeriodic_Init(kPeriodic timer, kType type, kSize stackSize, const kChar* name, k32s priority, kAlloc allocator)
{
    kPeriodicClass* obj = timer; 
    kStatus status; 

    kCheck(kObject_Init(timer, type, allocator)); 
    
    obj->lock = obj->thread = obj->stopwatch = kNULL; obj->signal = kNULL; 
    obj->quit = obj->enabled = kFALSE; 
    obj->period = 0; 
    obj->onElapsed = kNULL; 
    obj->onElapsedContext = kNULL; 

    kTry
    {
        kTest(kLock_Construct(&obj->lock, allocator)); 
        kTest(kTimer_Construct(&obj->stopwatch, allocator)); 
        kTest(kSemaphore_Construct(&obj->signal, 0, allocator)); 
        
        kTest(kThread_Construct(&obj->thread, allocator)); 
        kTest(kThread_StartEx(obj->thread, kPeriodic_ThreadEntry, obj, stackSize, name, priority)); 
    }
    kCatch(&status)
    {
        kPeriodic_VRelease(timer); 
        kEndCatch(status); 
    }    
    
    return kOK; 
}

kFx(kStatus) kPeriodic_VRelease(kPeriodic timer)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer); 

    if (obj->thread)
    {
        obj->quit = kTRUE; 
        kCheck(kSemaphore_Post(obj->signal)); 
        kCheck(kThread_Join(obj->thread, kINFINITE, kNULL)); 
    }

    kCheck(kDestroyRef(&obj->thread)); 
    kCheck(kDestroyRef(&obj->stopwatch)); 
    kCheck(kDestroyRef(&obj->lock)); 
    kCheck(kDestroyRef(&obj->signal)); 

    kCheck(kObject_VRelease(timer)); 

    return kOK; 
}

kFx(kStatus) kPeriodic_Start(kPeriodic timer, k64u period, kPeriodicElapsedFx onElapsed, kPointer context)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer); 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        obj->enabled = kTRUE; 
        obj->period = period; 
        obj->onElapsed = onElapsed; 
        obj->onElapsedContext = context; 

        kTest(kTimer_Start(obj->stopwatch, obj->period)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally();
    }

    kCheck(kSemaphore_Post(obj->signal)); 

    return kOK; 
}

kFx(kStatus) kPeriodic_Stop(kPeriodic timer)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer); 

    kCheck(kLock_Enter(obj->lock)); 
    {
        obj->enabled = kFALSE; 
        obj->onElapsed = kNULL; 
        obj->onElapsedContext = kNULL; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return kOK; 
}

kFx(kBool) kPeriodic_Enabled(kPeriodic timer)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer); 
    kBool enabled = kFALSE; 

    kCheck(kLock_Enter(obj->lock)); 
    {
        enabled = obj->enabled; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return enabled; 
}

kFx(kStatus) kPeriodic_ThreadEntry(kPeriodic timer)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer); 
    kStatus waitResult = kOK;  
    kStatus onElapsedResult = kOK; 

    do
    {
        k64u timeout = kINFINITE; 

        kCheck(kLock_Enter(obj->lock)); 
        {
            if (obj->enabled)
            {       
                if (kTimer_IsExpired(obj->stopwatch))
                {
                    onElapsedResult = obj->onElapsed(obj->onElapsedContext, timer);
                    kTimer_Start(obj->stopwatch, obj->period); 
                }

                if (obj->enabled)
                {
                    timeout = kTimer_Remaining(obj->stopwatch); 
                }
            }
        }
        kCheck(kLock_Exit(obj->lock)); 

        if ((waitResult = kSemaphore_Wait(obj->signal, timeout)) != kERROR_TIMEOUT)
        {
            kCheck(waitResult); 
        }
    }
    while (!obj->quit && kSuccess(onElapsedResult)); 

    return kOK; 
}

kFx(kPeriodicElapsedFx) kPeriodic_Handler(kPeriodic timer)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer);

    return obj->onElapsed;
}

kFx(kPointer) kPeriodic_HandlerContext(kPeriodic timer)
{
    kPeriodicClass* obj = kPeriodic_Cast_(timer);

    return obj->onElapsedContext; 
}

