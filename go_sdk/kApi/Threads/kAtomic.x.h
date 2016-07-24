/** 
 * @file    kAtomic.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ATOMIC_X_H
#define K_API_ATOMIC_X_H

kBeginHeader()

/**
 * @internal 
 * @class       kAtomic
 * @ingroup     kApi-Threads
 * @brief       Implements shared support functions for atomic variable classes.
 */
typedef struct kAtomicStatic
{
    k32u placeholder;       //unused
} kAtomicStatic; 

kDeclareStaticClass(k, kAtomic)

kFx(kStatus) kAtomic_InitStatic();
kFx(kStatus) kAtomic_ReleaseStatic();

/*
 * kAtomic32s
 */

kFx(k32s) kAtomic32s_IncrementImpl(kAtomic32s* atomic); 
kFx(k32s) kAtomic32s_DecrementImpl(kAtomic32s* atomic); 
kFx(k32s) kAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value); 
kFx(kBool) kAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value); 
kFx(k32s) kAtomic32s_GetImpl(kAtomic32s* atomic); 

#define kxAtomic32s_Init_(ATM, VAL)                          (*(ATM) = (VAL))
#define kxAtomic32s_Increment_(ATM)                          kApiLib_AtomicHandlers_()->increment32s(ATM) 
#define kxAtomic32s_Decrement_(ATM)                          kApiLib_AtomicHandlers_()->decrement32s(ATM)
#define kxAtomic32s_Exchange_(ATM, VAL)                      kApiLib_AtomicHandlers_()->exchange32s(ATM, VAL)
#define kxAtomic32s_CompareExchange_(ATM, OLD, VAL)          kApiLib_AtomicHandlers_()->compareExchange32s(ATM, OLD, VAL)
#define kxAtomic32s_Get_(ATM)                                kApiLib_AtomicHandlers_()->get32s(ATM)

/*
 * kAtomicPointer
 */

kFx(kPointer) kAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value); 
kFx(kBool) kAtomicPointer_CompareExchangeImpl(kAtomicPointer* atomic, kPointer oldValue, kPointer value); 
kFx(kPointer) kAtomicPointer_GetImpl(kAtomicPointer* atomic); 

#define kxAtomicPointer_Init_(ATM, VAL)                      (*(ATM) = (VAL))
#define kxAtomicPointer_Exchange_(ATM, VAL)                  kApiLib_AtomicHandlers_()->exchangePointer(ATM, VAL)
#define kxAtomicPointer_CompareExchange_(ATM, OLD, VAL)      kApiLib_AtomicHandlers_()->compareExchangePointer(ATM, OLD, VAL)
#define kxAtomicPointer_Get_(ATM)                            kApiLib_AtomicHandlers_()->getPointer(ATM)


//deprecated
#define kAtomic_Increment32s                    kAtomic32s_Increment
#define kAtomic_Decrement32s                    kAtomic32s_Decrement
#define kAtomic_Exchange32s                     kAtomic32s_Exchange
#define kAtomic_CompareExchange32s              kAtomic32s_CompareExchange
#define kAtomic_Get32s                          kAtomic32s_Get
#define kAtomic_Init32s_                        kxAtomic32s_Init_
#define kAtomic_Increment32s_                   kxAtomic32s_Increment_
#define kAtomic_Decrement32s_                   kxAtomic32s_Decrement_
#define kAtomic_Exchange32s_                    kxAtomic32s_Exchange_
#define kAtomic_CompareExchange32s_             kxAtomic32s_CompareExchange_
#define kAtomic_Get32s_                         kxAtomic32s_Get_
#define kAtomic_ExchangePointer                 kAtomicPointer_Exchange
#define kAtomic_CompareExchangePointer          kAtomicPointer_CompareExchange
#define kAtomic_GetPointer                      kAtomicPointer_Get
#define kAtomic_InitPointer_                    kxAtomicPointer_Init_
#define kAtomic_ExchangePointer_                kxAtomicPointer_Exchange_
#define kAtomic_CompareExchangePointer_         kxAtomicPointer_CompareExchange_
#define kAtomic_GetPointer_                     kxAtomicPointer_Get_


kEndHeader()

#endif
