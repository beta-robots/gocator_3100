/** 
 * @file    kThread.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_THREAD_X_H
#define K_API_THREAD_X_H

kBeginHeader()

kDeclareClass(k, kThread, kObject)

#if defined(K_PLATFORM) 

#if defined(K_WINDOWS)
    
#   define kThreadPlatformFields()                                  \
        HANDLE handle;      /* thread handle */                     \
        DWORD id;           /* unique thread id from os */

    unsigned int __stdcall kThread_EntryPoint(void* arg);

#elif defined(K_DSP_BIOS)

#   define kTHREAD_MIN_OS_PRIORITY     (1)          /* minimum valid priority (0 reserved for idle task) */
#   define kTHREAD_MAX_OS_PRIORITY     (15)         /* maximum valid priority */

#   define kThreadPlatformFields()                                      \
        TSK_Handle handle;          /* thread handle */                 \
        kText64 name;               /* descriptive name */              \
        kSemaphore joinSem;         /* implements join behaviour */     \
        volatile kStatus exitCode;  /* result of thread execution */

    void kThread_EntryPoint(void* arg);

#elif defined(K_VX_KERNEL)

#   define kTHREAD_MIN_OS_PRIORITY     (0)          /* minimum valid priority value (highest relative priority) */
#   define kTHREAD_MAX_OS_PRIORITY     (255)        /* maximum valid priority value (lowest relative priority) */
#   define kTHREAD_DEFAULT_PRIORITY    (128)		/* default priority assigned to threads created with this class */
#   define kTHREAD_DEFAULT_STACK_SIZE  (0x8000)     /* default stack size assigned to threads created with this class */

#   define kThreadPlatformFields()                                      \
        TASK_ID id;                 /* thread identifier */             \
        kSemaphore joinSem;         /* implements join behaviour */     \
        volatile kStatus exitCode;  /* result of thread execution */

    int kThread_EntryPoint(_Vx_usr_arg_t arg0);
            
#elif defined(K_POSIX)

#   define kThreadPlatformFields()                                      \
        kSemaphore hasJoined;       /* supports join wait argument */   \
        pthread_t handle;           /* thead handle */

    void* kThread_EntryPoint(void* arg);

#endif

typedef struct kThreadClass
{
    kObjectClass base;    
    kThreadFx function;          // entry-point function
    kPointer context;            // entry-point context
    kThreadPlatformFields()
} kThreadClass;

kFx(kStatus) kThread_Init(kThread thread, kType type, kAlloc alloc); 
kFx(kStatus) kThread_VRelease(kThread thread); 

/** 
 * Gets a unique identifier that represents the currently executing thread.
 * 
 * @public              @memberof kThread
 * @return              Unique thread identifier. 
 */
kFx(kThreadId) kThread_CurrentId(); 

/** 
 * Gets a unique identifier representing the thread.
 * 
 * This field is only valid after the thread has been successfully started, and 
 * before the thread has been successfully joined.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @return              Unique thread identifier. 
 */
kFx(kThreadId) kThread_Id(kThread thread); 

/** 
 * Compares two thread identifiers.
 *
 * @public          @memberof kThread
 * @param   a       First thread identifier.
 * @param   b       Second thread identifier.
 * @return          kTRUE if the threads are identical; kFALSE otherwise. 
 */
kFx(kBool) kThread_CompareId(kThreadId a, kThreadId b); 

#endif

/** 
 * Begins executing a thread using the specified callback function and options. 
 *
 * Note: The thread options provided via this function are not supported on every platform.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @param   function    The thread entry function. 
 * @param   context     An argument to be passed to the thread entry function.
 * @param   stackSize   Requested stack size, in bytes (0 for default). 
 * @param   name        Descriptive name for the thread (kNULL for default).
 * @param   priority    Thread priority, relative to 0 (default).
 * @return              Operation status. 
 */
kFx(kStatus) kThread_StartEx(kThread thread, kThreadFx function, kPointer context, 
                             kSize stackSize, const kChar* name, k32s priority); 


kFx(kThreadFx) kThread_Handler(kThread thread);
kFx(kPointer)  kThead_HandlerContext(kThread thread);


#define kThread_Cast_(T)              (kCastClass_(kThread, T))


/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_THREAD                 kTypeOf(kThread)
#   define kThread_Destroy              kObject_Destroy

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kTHREAD_PRIORITY_NORMAL      (0)  
#   define kThread_PriorityMin()        (k32S_MIN)
#   define kThread_PriorityMax()        (k32S_MAX)
    kFx(kStatus) kThread_Construct5(kThread* thread, kThreadFx function, kPointer context, k32s priority); 
#   define kThread_Join5(TR, TI)        kThread_Join(TR, TI, kNULL)

#endif




kEndHeader()

#endif
