/** 
 * @file    kTimer.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TIMER_X_H
#define K_API_TIMER_X_H

kBeginHeader()

#define kTIMER_MIN_YEARS                (3)                       ///< minimum years that should be safely representable
#define kTIMER_SECONDS_PER_YEAR         (365*24*60*60)            ///< approximate seconds per year

typedef struct kTimerStatic
{
    k32u placeholder;       //unused
} kTimerStatic; 

typedef struct kTimerVTable
{
    kObjectVTable base; 
} kTimerVTable; 

typedef struct kTimerClass
{
    kObjectClass base; 
    kBool isStopped;            //< is the timer currently running?
    k64u startTime;             //< time when the timer is started 
    k64u expiryTime;            //< time when the timer expires 
    k64u stopTime;              //< time when the timer is stopped 
} kTimerClass;

kDeclareFullClass(k, kTimer, kObject)

kFx(kStatus) kTimer_InitStatic(); 
kFx(kStatus) kTimer_ReleaseStatic(); 

kFx(kStatus) kTimer_Init(kTimer timer, kType type, kAlloc allocator); 

kFx(kStatus) kTimer_ConfigureDefaults(); 
kFx(k64u) kTimer_DefaultTickQuery(); 

#define kTimer_Cast_(T)            (kCastClass_(kTimer, T))

#define kTimer_Ticks_()           (kApiLib_TimerQueryHandler_()())                     
#define kTimer_FromTicks_(T)      ((T)*kApiLib_TimerMultiplier_()/kApiLib_TimerDivider_())                     
#define kTimer_Now_()             (kTimer_FromTicks_(kTimer_Ticks_()))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_TIMER                  kTypeOf(kTimer)
#   define kTimer_Destroy               kObject_Destroy
#   define kTimer_ElapsedTime           kTimer_Elapsed
#   define kTimer_RemainingTime         kTimer_Remaining
#   define kTimer_IsRunning             kTimer_IsStarted

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kTimer_Construct5(T)         kTimer_Construct(T, kNULL)
#   define kUpTime()                    (kTimer_Now()/1000)

#endif

kEndHeader()

#endif
