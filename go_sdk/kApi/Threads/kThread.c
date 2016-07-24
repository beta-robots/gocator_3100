/** 
 * @file    kThread.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kThread.h>
#include <kApi/Threads/kSemaphore.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Data/kMath.h>
#include <kApi/Utils/kUtils.h>

kBeginClass(k, kThread, kObject)
    kAddVMethod(kThread, kObject, VRelease)
kEndClass()

kFx(kStatus) kThread_Construct(kThread *thread, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kThread), thread)); 

    if (!kSuccess(status = kThread_Init(*thread, kTypeOf(kThread), alloc)))
    {
        kAlloc_FreeRef(alloc, thread); 
    }

    return status; 
} 

kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context)
{
    return kThread_StartEx(thread, function, context, 0, kNULL, 0); 
}

kFx(kStatus) kThread_SleepAtLeast(k64u duration)
{
    k64u startTime = kTimer_Now(); 
    k64u endTime = startTime + duration; 
    k64u currentTime; 

    while ((currentTime = kTimer_Now()) < endTime)
    {
        kCheck(kThread_Sleep(endTime - currentTime)); 
    }

    return kOK; 
}

kFx(kThreadFx) kThread_Handler(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread);

    return obj->function; 
}

kFx(kPointer) kThead_HandlerContext(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread);

    return obj->context;
}

#if defined(K_WINDOWS)

kFx(k32s) kThread_OsPriority(k32s priority)
{
    if      (priority <= -3)  return THREAD_PRIORITY_IDLE; 
    else if (priority == -2)  return THREAD_PRIORITY_LOWEST; 
    else if (priority == -1)  return THREAD_PRIORITY_BELOW_NORMAL; 
    else if (priority ==  0)  return THREAD_PRIORITY_NORMAL; 
    else if (priority ==  1)  return THREAD_PRIORITY_ABOVE_NORMAL; 
    else if (priority ==  2)  return THREAD_PRIORITY_HIGHEST; 
    else                      return THREAD_PRIORITY_TIME_CRITICAL; 
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    DWORD osDuration = (DWORD) kTimeToKernelTime(duration); 
    
    Sleep(osDuration); 
    
    return kOK; 
}

kFx(kStatus) kThread_Init(kThread thread, kType type, kAlloc allocator)
{
    kThreadClass* obj = thread; 
    
    kCheck(kObject_Init(thread, type, allocator)); 

    kInitFields_(kThread, thread); 
    
    return kOK; 
}

kFx(kStatus) kThread_VRelease(kThread thread)
{
    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

//name argument not supported at this time
kFx(kStatus) kThread_StartEx(kThread thread, kThreadFx function, kPointer context, 
                             kSize stackSize, const kChar* name, k32s priority)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    kStatus exception; 

    kCheckState(kIsNull(obj->function)); 

    obj->function = function; 
    obj->context = context; 

    kTry
    {
        if (kIsNull(obj->handle = (HANDLE)_beginthreadex(0, (unsigned)stackSize, kThread_EntryPoint, thread, CREATE_SUSPENDED, &obj->id)))
        {
            kThrow(kERROR_OS); 
        }

        if (!SetThreadPriority(obj->handle, kThread_OsPriority(priority)))
        {
            kThrow(kERROR_OS); 
        }

        if (ResumeThread(obj->handle) == k32U_MAX)
        {
            kThrow(kERROR_OS); 
        }
    }
    kCatch(&exception)
    {
        if (!kIsNull(obj->handle))
        {
            CloseHandle(obj->handle); 
        }

        obj->handle = kNULL; 
        obj->context = kNULL; 
        obj->id = 0; 
        obj->function = kNULL; 

        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    DWORD osTimeout = (DWORD) kTimeToKernelTime(timeout); 
    DWORD waitResult; 
    DWORD threadExit;

    if (obj->function)
    {
        waitResult = WaitForSingleObject(obj->handle, osTimeout); 
        
        if (waitResult == WAIT_TIMEOUT)
        {
            return kERROR_TIMEOUT; 
        }
        else if (waitResult != WAIT_OBJECT_0)
        {
            return kERROR_OS; 
        }       
        
        if (!GetExitCodeThread(obj->handle, &threadExit))
        {
            return kERROR; 
        }

        if (!CloseHandle(obj->handle))
        {
            return kERROR;
        }
        
        obj->function = kNULL; 
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = threadExit; 
        }
    }
    
    return kOK;
}

unsigned int __stdcall kThread_EntryPoint(void *arg)
{
    kThreadClass* obj = kThread_Cast_(arg); 
    
    return (unsigned int) obj->function(obj->context);
}

kFx(kThreadId) kThread_Id(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    return obj->id;     
}

kFx(kThreadId) kThread_CurrentId()
{
    return (kThreadId) GetCurrentThreadId(); 
}

kFx(kBool) kThread_CompareId(kThreadId a, kThreadId b)
{
    return a == b; 
}

#elif defined (K_DSP_BIOS)

kFx(k32s) kThread_OsPriority(k32s priority)
{
    TSK_Attrs defaults = TSK_ATTRS;     /* DSP BIOS defaults configurable via tconf */
    k32s osPriority = defaults.priority + priority; 

    return kMath_Clamp_(osPriority, kTHREAD_MIN_OS_PRIORITY, kTHREAD_MAX_OS_PRIORITY); 
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    k64u osDuration = kTimeToKernelTime(duration); 
    
    if (osDuration > 0)
    {
        TSK_sleep(osDuration);
    }
    else
    {
        TSK_yield(); 
    }
    
    return kOK; 
}

kFx(kStatus) kThread_Init(kThread thread, kType type, kAlloc allocator)
{
    kThreadClass* obj = thread; 
    kStatus exception; 
    
    kCheck(kObject_Init(thread, type, allocator)); 

    kInitFields_(kThread, thread); 
    
    obj->exitCode = kOK; 

    kTry
    {   
        kTest(kStrCopy(obj->name, kCountOf(obj->name), "kThread instance")); 
        kTest(kSemaphore_Construct(&obj->joinSem, 0, allocator)); 
    }
    kCatch(&exception)
    {
        kThread_VRelease(thread); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kThread_VRelease(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 
    
    kCheck(kObject_Destroy(obj->joinSem)); 

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_StartEx(kThread thread, kThreadFx function, kPointer context, 
                             kSize stackSize, const kChar* name, k32s priority)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    TSK_Attrs attributes = TSK_ATTRS;     /* initialized with DSP BIOS defaults */
    kStatus exception; 

    kCheckState(kIsNull(obj->function)); 

    obj->function = function; 
    obj->context = context; 

    kTry
    {
        if (!kIsNull(name))
        {
            kTest(kStrCopy(obj->name, kCountOf(obj->name), name)); 
        }

        //override defaults
        attributes.priority = kThread_OsPriority(priority); 
        attributes.stacksize = (stackSize > 0) ? stackSize : attributes.stacksize; 
        attributes.name = obj->name; 
        attributes.initstackflag = kTRUE; 
    
        if (kIsNull(obj->handle = TSK_create((Fxn)kThread_EntryPoint, &attributes, thread)))
        {
            kThrow(kERROR_OS); 
        }
    }
    kCatch(&exception)
    {
        obj->function = kNULL; 
        obj->context = kNULL; 
        obj->handle = kNULL; 

        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kThreadClass* obj = kThread_Cast_(thread); 

    if (obj->function)
    {
        kCheck(kSemaphore_Wait(obj->joinSem, timeout)); 

        TSK_delete(obj->handle); 
        
        obj->function = kNULL; 
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = obj->exitCode; 
        }
    }
    
    return kOK;
}

void kThread_EntryPoint(void* arg)
{
    kThreadClass* obj = kThread_Cast_(arg); 
   
    //required by DSP BIOS NDK, in order to use sockets
    if (fdOpenSession(TSK_self()) != 1)
    {
        obj->exitCode = kERROR_OS; 
        kAssert(kFALSE);  
        return; 
    }

    obj->exitCode = obj->function(obj->context);

    fdCloseSession(TSK_self());
 
    kSemaphore_Post(obj->joinSem); 

    //check whether stack overflow ever occurred on this thread
    TSK_checkstacks(TSK_self(), TSK_self());

    return; 
}

kFx(kThreadId) kThread_Id(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    return obj->handle; 
}

kFx(kThreadId) kThread_CurrentId()
{
    return TSK_self(); 
}

kFx(kBool) kThread_CompareId(kThreadId a, kThreadId b)
{
    return a == b; 
}

#elif defined(K_VX_KERNEL)

kFx(k32s) kThread_OsPriority(k32s priority)
{
	k32s defaultPriority = kTHREAD_DEFAULT_PRIORITY; 	  
	k32s osPriority = defaultPriority - priority;   //higher logical priorities are represented by lower priority numbers  

    return kMath_Clamp_(osPriority, kTHREAD_MIN_OS_PRIORITY, kTHREAD_MAX_OS_PRIORITY); 
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
	_Vx_ticks_t osDuration = (_Vx_ticks_t) kTimeToKernelTime(duration); 
    
	if (taskDelay(osDuration) != OK)
	{
	    return kERROR_OS; 
	}
	
    return kOK; 
}

kFx(kStatus) kThread_Init(kThread thread, kType type, kAlloc allocator)
{
    kThreadClass* obj = thread; 
    kStatus exception; 
            
    kCheck(kObject_Init(thread, type, allocator)); 

    kInitFields_(kThread, thread); 
            
    obj->exitCode = kOK; 
    obj->id = TASK_ID_ERROR; 

    kTry
    {   
        kTest(kSemaphore_Construct(&obj->joinSem, 0, allocator)); 
    }
    kCatch(&exception)
    {
        kThread_VRelease(thread); 
        kEndCatch(exception); 
    }

    return kOK; 
}        
        
kFx(kStatus) kThread_VRelease(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 
    
    kCheck(kObject_Destroy(obj->joinSem)); 

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_StartEx(kThread thread, kThreadFx function, kPointer context, 
                             kSize stackSize, const kChar* name, k32s priority)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    k32s osPriority = kThread_OsPriority(priority); 
    kSize osStackSize = (stackSize == 0) ? kTHREAD_DEFAULT_STACK_SIZE : stackSize; 
    kStatus exception; 

    kCheckState(kIsNull(obj->function)); 

    obj->function = function; 
    obj->context = context; 

    kTry
    {    
        obj->id = taskSpawn((kChar*)name, osPriority, VX_FP_TASK, osStackSize, kThread_EntryPoint, (_Vx_usr_arg_t)thread, 0, 0, 0, 0, 0, 0, 0, 0, 0); 

        if (obj->id == TASK_ID_ERROR)
        {
            kThrow(kERROR_OS);   
        }
    }
    kCatch(&exception)
    {
        obj->function = kNULL; 
        obj->context = kNULL; 
        obj->id = TASK_ID_ERROR; 

        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kThreadClass* obj = kThread_Cast_(thread);

    if (obj->function)
    {
        kCheck(kSemaphore_Wait(obj->joinSem, timeout)); 
                
        obj->function = kNULL;         
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = obj->exitCode; 
        }
    }
    
    return kOK;   
}        
       
int kThread_EntryPoint(_Vx_usr_arg_t arg0)
{
    kThreadClass* obj = kThread_Cast_((kPointer)arg0); 
       
    obj->exitCode = obj->function(obj->context);
    
    kSemaphore_Post(obj->joinSem); 
    
    return kOK;  
}

kFx(kThreadId) kThread_Id(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    return obj->id;     
}

kFx(kThreadId) kThread_CurrentId()
{
    return taskIdSelf(); 
}

kFx(kBool) kThread_CompareId(kThreadId a, kThreadId b)
{
    return a == b; 
}

#elif defined(K_POSIX)

kFx(kStatus) kThread_Sleep(k64u duration)
{
    useconds_t durationMs = (useconds_t) ((duration + 999)/1000); 
    
    usleep(durationMs*1000); 
    
    return kOK; 
}

kFx(kStatus) kThread_Init(kThread thread, kType type, kAlloc alloc)
{
    kThreadClass* obj = thread; 
    kStatus exception; 
    
    kCheck(kObject_Init(thread, type, alloc)); 

    obj->function = kNULL; 
    obj->context = kNULL; 
    obj->hasJoined = kNULL; 

    kZero_(obj->handle); 

    kTry
    {
        kTest(kSemaphore_Construct(&obj->hasJoined, 0, alloc)); 
    }
    kCatch(&exception)
    {
        kThread_VRelease(thread); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kThread_VRelease(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 

    kCheck(kObject_Destroy(obj->hasJoined)); 

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

//stackSize, name, and priority arguments not supported at this time
kFx(kStatus) kThread_StartEx(kThread thread, kThreadFx function, kPointer context, 
                             kSize stackSize, const kChar* name, k32s priority)
{
    kThreadClass* obj = kThread_Cast_(thread); 

    kCheckState(kIsNull(obj->function)); 

    obj->function = function; 
    obj->context = context; 

    if (pthread_create(&obj->handle, kNULL, kThread_EntryPoint, thread) != 0)
    {
        obj->function = kNULL; 
        return kERROR; 
    }    

    return kOK; 
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    void* threadExit; 
    
    if (obj->function)
    {
        if (timeout != kINFINITE)
        {
            kCheck(kSemaphore_Wait(obj->hasJoined, timeout)); 
        }

        if (pthread_join(obj->handle, (void**)&threadExit) != 0)
        {
            return kERROR; 
        }

        obj->function = kNULL; 
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = (kStatus) (kSSize) threadExit; 
        }
    }
    
    return kOK; 
}

void* kThread_EntryPoint(void* arg)
{
    kThreadClass* obj = kThread_Cast_(arg); 
    kStatus result; 

    result = obj->function(obj->context);

    //signal thread completion
    kSemaphore_Post(obj->hasJoined); 
        
    return (void*) (kSSize) result;  
}

kFx(kThreadId) kThread_Id(kThread thread)
{
    kThreadClass* obj = kThread_Cast_(thread); 
    return obj->handle;      
}

kFx(kThreadId) kThread_CurrentId()
{
    return pthread_self(); 
}

kFx(kBool) kThread_CompareId(kThreadId a, kThreadId b)
{
    return (pthread_equal(a, b) != 0); 
}

#endif

//K_COMPAT_5
kFx(kStatus) kThread_Construct5(kThread* thread, kThreadFx function, kPointer context, k32s priority)
{
    kThread output = kNULL; 
    kStatus status = kOK;  

    kCheck(kThread_Construct(&output, kNULL)); 

    if (!kSuccess(status = kThread_StartEx(output, function, context, 0, kNULL, priority)))
    {
        kObject_Destroy(output); 
        return status; 
    }

    *thread = output; 

    return kOK; 
}
