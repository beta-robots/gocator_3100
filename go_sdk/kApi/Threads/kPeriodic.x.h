/** 
 * @file    kPeriodic.x.h
 *
 * @internal
 * Copyright (C) 2010-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PERIODIC_X_H
#define K_API_PERIODIC_X_H

kBeginHeader()

typedef struct kPeriodicClass
{
    kObjectClass base; 
    kLock lock; 
    kThread thread; 
    kTimer stopwatch; 
    kSemaphore signal; 
    kBool quit; 
    kBool enabled; 
    k64u period; 
    kPeriodicElapsedFx onElapsed; 
    kPointer onElapsedContext;
} kPeriodicClass;

kDeclareClass(k, kPeriodic, kObject)

kFx(kStatus) kPeriodic_Init(kPeriodic timer, kType type, kSize stackSize, const kChar* name, k32s priority, kAlloc allocator);
kFx(kStatus) kPeriodic_VRelease(kPeriodic timer); 

/** 
 * Constructs a kPeriodic object with additional options.
 *
 * @public              @memberof kPeriodic
 * @param   timer       Receives a handle to the constructed object. 
 * @param   stackSize   Requested stack size, in bytes (0 for default). 
 * @param   name        Descriptive name for the thread (kNULL for default).
 * @param   priority    Thread priority, relative to 0 (default).
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kPeriodic_ConstructEx(kPeriodic* timer, kSize stackSize, const kChar* name, k32s priority, kAlloc allocator); 

kFx(kStatus) kPeriodic_ThreadEntry(kPeriodic timer); 

kFx(kPeriodicElapsedFx) kPeriodic_Handler(kPeriodic timer);
kFx(kPointer) kPeriodic_HandlerContext(kPeriodic timer);

#define kPeriodic_Cast_(P)              (kCastClass_(kPeriodic, P))

kEndHeader()

#endif
