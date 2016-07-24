/** 
 * @file    kAtomic.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kAtomic.h>

kBeginStaticClass(k, kAtomic)
kEndStaticClass()

kFx(kStatus) kAtomic_InitStatic()
{
    kApiAtomicFx handlers = { kNULL }; 

    //respect handlers that have already been installed
    if (kApiLib_HasAtomicHandlers_())
    {
        handlers = *kApiLib_AtomicHandlers_(); 
    }
        
    //ensure a default handler is provided for all methods
    if (kIsNull(handlers.increment32s))                 handlers.increment32s = kAtomic32s_IncrementImpl; 
    if (kIsNull(handlers.decrement32s))                 handlers.decrement32s = kAtomic32s_DecrementImpl; 
    if (kIsNull(handlers.exchange32s))                  handlers.exchange32s = kAtomic32s_ExchangeImpl; 
    if (kIsNull(handlers.compareExchange32s))           handlers.compareExchange32s= kAtomic32s_CompareExchangeImpl; 
    if (kIsNull(handlers.get32s))                       handlers.get32s = kAtomic32s_GetImpl; 
    if (kIsNull(handlers.exchangePointer))              handlers.exchangePointer = kAtomicPointer_ExchangeImpl; 
    if (kIsNull(handlers.compareExchangePointer))       handlers.compareExchangePointer = kAtomicPointer_CompareExchangeImpl; 
    if (kIsNull(handlers.getPointer))                   handlers.getPointer = kAtomicPointer_GetImpl; 

    kCheck(kApiLib_SetAtomicHandlers(&handlers)); 

    return kOK;
}

kFx(kStatus) kAtomic_ReleaseStatic()
{
    return kOK; 
}

kFx(k32s) kAtomic32s_Increment(kAtomic32s* atomic)
{
   return kApiLib_AtomicHandlers_()->increment32s(atomic); 
}

kFx(k32s) kAtomic32s_Decrement(kAtomic32s* atomic)
{
   return kApiLib_AtomicHandlers_()->decrement32s(atomic); 
}

kFx(k32s) kAtomic32s_Exchange(kAtomic32s* atomic, k32s value)
{
   return kApiLib_AtomicHandlers_()->exchange32s(atomic, value); 
}

kFx(kBool) kAtomic32s_CompareExchange(kAtomic32s* atomic, k32s oldValue, k32s value)
{
   return kApiLib_AtomicHandlers_()->compareExchange32s(atomic, oldValue, value); 
}

kFx(k32s) kAtomic32s_Get(kAtomic32s* atomic)
{
   return kApiLib_AtomicHandlers_()->get32s(atomic); 
}

kFx(kPointer) kAtomicPointer_Exchange(kAtomicPointer* atomic, kPointer value)
{
   return kApiLib_AtomicHandlers_()->exchangePointer(atomic, value); 
}

kFx(kBool) kAtomicPointer_CompareExchange(kAtomicPointer* atomic, kPointer oldValue, kPointer value)
{
   return kApiLib_AtomicHandlers_()->compareExchangePointer(atomic, oldValue, value); 
}

kFx(kPointer) kAtomicPointer_Get(kAtomicPointer* atomic)
{
   return kApiLib_AtomicHandlers_()->getPointer(atomic); 
}

#if defined(K_WINDOWS)

kFx(k32s) kAtomic32s_IncrementImpl(kAtomic32s* atomic)
{
    return InterlockedIncrement((volatile LONG*)atomic); 
}

kFx(k32s) kAtomic32s_DecrementImpl(kAtomic32s* atomic)
{
    return InterlockedDecrement((volatile LONG*)atomic);
}

kFx(k32s) kAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value)
{
    return InterlockedExchange((volatile LONG*)atomic, value);
}

kFx(kBool) kAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value)
{
    return (InterlockedCompareExchange((volatile LONG*)atomic, value, oldValue) == oldValue); 
}

kFx(k32s) kAtomic32s_GetImpl(kAtomic32s* atomic)
{
    k32s value; 

    do
    {
        value = *atomic; 
    }
    while (!kAtomic32s_CompareExchange_(atomic, value, value)); 

    return value; 
}

kFx(kPointer) kAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value)
{
    return InterlockedExchangePointer(atomic, value);
}

kFx(kBool) kAtomicPointer_CompareExchangeImpl(void* volatile* atomic, kPointer oldValue, kPointer value)
{
    return (InterlockedCompareExchangePointer(atomic, value, oldValue) == oldValue); 
}

kFx(kPointer) kAtomicPointer_GetImpl(kAtomicPointer* atomic)
{
    kPointer value; 

    do
    {
        value = *atomic; 
    }
    while (!kAtomicPointer_CompareExchange_(atomic, value, value)); 

    return value; 
}

#elif defined(K_DARWIN)

kFx(k32s) kAtomic32s_IncrementImpl(kAtomic32s* atomic)
{
    return OSAtomicIncrement32Barrier(atomic);
}

kFx(k32s) kAtomic32s_DecrementImpl(kAtomic32s* atomic)
{
    return OSAtomicDecrement32Barrier(atomic);
}

kFx(k32s) kAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value)
{
    k32s oldValue; 
    
    do
    {
        oldValue = *atomic;
    }
    while (!OSAtomicCompareAndSwapIntBarrier(oldValue, value, atomic));

    return oldValue; 
}

kFx(kBool) kAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value)
{
    return OSAtomicCompareAndSwapIntBarrier(oldValue, value, atomic); 
}

kFx(k32s) kAtomic32s_GetImpl(kAtomic32s* atomic)
{
    k32s value; 

    do
    {
        value = *atomic; 
    }
    while (!kAtomic32s_CompareExchange_(atomic, value, value)); 

    return value; 
}

kFx(kPointer) kAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value)
{
    kPointer oldValue; 
    
    do
    {
        oldValue = *atomic;
    }
    while (!OSAtomicCompareAndSwapPtrBarrier(oldValue, value, atomic));

    return oldValue; 
}

kFx(kBool) kAtomicPointer_CompareExchangeImpl(kAtomicPointer* atomic, kPointer oldValue, kPointer value)
{
    return OSAtomicCompareAndSwapPtrBarrier(oldValue, value, atomic); 
}

kFx(kPointer) kAtomicPointer_GetImpl(kAtomicPointer* atomic)
{
    kPointer value; 

    do
    {
        value = *atomic; 
    }
    while (!kAtomicPointer_CompareExchange_(atomic, value, value)); 

    return value; 
}

#elif defined (K_VX_KERNEL)

//VxWorks 6.9, even when compiled with GCC, doesn't support the GCC built-in atomics on all architectures (e.g. ARM)  

kFx(k32s) kAtomic32s_IncrementImpl(kAtomic32s* atomic) 
{
    return vxAtomic32Inc(atomic) + 1; 
}

kFx(k32s) kAtomic32s_DecrementImpl(kAtomic32s* atomic)
{
    return vxAtomic32Dec(atomic) - 1; 
}

kFx(k32s) kAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value) 
{
    return vxAtomic32Set(atomic, value);
}

kFx(kBool) kAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value)
{
    return vxAtomic32Cas(atomic, oldValue, value);
}

kFx(k32s) kAtomic32s_GetImpl(kAtomic32s* atomic)
{
    return vxAtomic32Get(atomic);
}

kFx(kPointer) kAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value)
{
    return (kPointer)vxAtomicSet(atomic, (kSize)value);
}

kFx(kBool) kAtomicPointer_CompareExchangeImpl(kAtomicPointer* atomic, kPointer oldValue, kPointer value)
{
    return vxAtomicCas(atomic, (kSize)oldValue, (kSize)value);
}

kFx(kPointer) kAtomicPointer_GetImpl(kAtomicPointer* atomic)
{
    return (kPointer)vxAtomicGet(atomic);
}

#elif defined (K_GCC)

kFx(k32s) kAtomic32s_IncrementImpl(kAtomic32s* atomic) 
{
    return __sync_add_and_fetch(atomic, 1); 
}

kFx(k32s) kAtomic32s_DecrementImpl(kAtomic32s* atomic)
{
    return __sync_sub_and_fetch(atomic, 1); 
}

kFx(k32s) kAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value) 
{
    k32s oldValue; 
   
    do 
    {
        oldValue = *atomic;
    }
    while (!__sync_bool_compare_and_swap(atomic, oldValue, value));

    return oldValue; 
}

kFx(kBool) kAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value)
{
    return __sync_bool_compare_and_swap(atomic, oldValue, value);
}

kFx(k32s) kAtomic32s_GetImpl(kAtomic32s* atomic)
{
    k32s value; 

    do
    {
        value = *atomic; 
    }
    while (!kAtomic32s_CompareExchange_(atomic, value, value)); 

    return value; 
}

kFx(kPointer) kAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value)
{
    kPointer oldValue; 
   
    do 
    {
        oldValue = *atomic;
    }
    while (!__sync_bool_compare_and_swap(atomic, oldValue, value));

    return oldValue; 
}

kFx(kBool) kAtomicPointer_CompareExchangeImpl(kAtomicPointer* atomic, kPointer oldValue, kPointer value)
{
    return __sync_bool_compare_and_swap(atomic, oldValue, value);
}

kFx(kPointer) kAtomicPointer_GetImpl(kAtomicPointer* atomic)
{
    kPointer value; 

    do
    {
        value = *atomic; 
    }
    while (!kAtomicPointer_CompareExchange_(atomic, value, value)); 

    return value; 
}

#else

kFx(k32s) kAtomic32s_IncrementImpl(kAtomic32s* atomic)
{
    kAssert(kFALSE); 
    return 0; 
}

kFx(k32s) kAtomic32s_DecrementImpl(kAtomic32s* atomic)
{
    kAssert(kFALSE); 
    return 0; 
}

kFx(k32s) kAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value)
{
    kAssert(kFALSE); 
    return 0; 
}

kFx(kBool) kAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value)
{
    kAssert(kFALSE); 
    return kFALSE; 
}

kFx(k32s) kAtomic32s_GetImpl(kAtomic32s* atomic)
{
    kAssert(kFALSE); 
    return 0; 
}

kFx(kPointer) kAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value)
{
    kAssert(kFALSE); 
    return kNULL; 
}

kFx(kBool) kAtomicPointer_CompareExchangeImpl(kAtomicPointer* atomic, kPointer oldValue, kPointer value)
{
    kAssert(kFALSE); 
    return kFALSE; 
}

kFx(kPointer) kAtomicPointer_GetImpl(kAtomicPointer* atomic)
{
    kAssert(kFALSE); 
    return kNULL; 
}

#endif










