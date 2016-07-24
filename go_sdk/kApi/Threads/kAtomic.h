/** 
 * @file    kAtomic.h
 * @brief   Declares atomic operation classes. 
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_ATOMIC_H
#define K_API_ATOMIC_H

kBeginHeader()


/**
* @struct  kAtomic32s
* @extends kValue
* @ingroup kApi-Threads
* @brief   Represents a 32-bit, atomically-accessed, signed integer.
* @see     kAtomic, k32s
*/
//typedef kxAtomic32s kAtomic32s;           // --forward-declared in kApiDef.x.h 

/** 
 * Increments an atomic variable. 
 * 
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomic32s
 * @param   atomic      Pointer to  atomic variable.
 * @return              New atomic value.
 */
kFx(k32s) kAtomic32s_Increment(kAtomic32s* atomic); 

/** 
 * Decrements an atomic variable. 
 * 
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomic32s
 * @param   atomic      Pointer to atomic variable.
 * @return              New atomic value.
 */
kFx(k32s) kAtomic32s_Decrement(kAtomic32s* atomic); 

/** 
 * Exchanges the value of an atomic variable.
 * 
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomic32s
 * @param   atomic      Pointer to atomic variable.
 * @param   value       New atomic value.
 * @return              Previous atomic value.
 */
kFx(k32s) kAtomic32s_Exchange(kAtomic32s* atomic, k32s value); 

/** 
 * Conditionally exchanges the value of an atomic variable.
 * 
 * If the atomic value is equal to the oldValue argument, then it is replaced by the value argument. 
 *
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomic32s
 * @param   atomic      Pointer to atomic variable.
 * @param   oldValue    Previous atomic value.
 * @param   value       New atomic value.
 * @return              kTRUE if the exchange succeeded. 
 */
kFx(kBool) kAtomic32s_CompareExchange(kAtomic32s* atomic, k32s oldValue, k32s value); 

/** 
 * Gets the current value of an atomic variable. 
 * 
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomic32s
 * @param   atomic      Pointer to atomic variable.
 * @return              Atomic value.
 */
kFx(k32s) kAtomic32s_Get(kAtomic32s* atomic); 


/** @relates kAtomic32s @{ */

#define kAtomic32s_Init_(ATM, VAL)                          kxAtomic32s_Init_(ATM, VAL)                         ///< Initializes a variable of type kAtomic32s.
#define kAtomic32s_Increment_(ATM)                          kxAtomic32s_Increment_(ATM)                         ///< Macro version of kAtomic32s_Increment. 
#define kAtomic32s_Decrement_(ATM)                          kxAtomic32s_Decrement_(ATM)                         ///< Macro version of kAtomic32s_Decrement.
#define kAtomic32s_Exchange_(ATM, VAL)                      kxAtomic32s_Exchange_(ATM, VAL)                     ///< Macro version of kAtomic32s_Exchange.
#define kAtomic32s_CompareExchange_(ATM, OLD, VAL)          kxAtomic32s_CompareExchange_(ATM, OLD, VAL)         ///< Macro version of kAtomic32s_CompareExchange.
#define kAtomic32s_Get_(ATM)                                kxAtomic32s_Get_(ATM)                               ///< Macro version of kAtomic32s_Get.

/** @} */


/**
* @struct  kAtomicPointer
* @extends kValue
* @ingroup kApi-Threads
* @brief   Represents an atomically-accessed pointer.
* @see     kAtomic, kPointer
*/
//typedef kxAtomicPointer kAtomicPointer;       // --forward-declared in kApiDef.x.h 


/** 
 * Exchanges the value of an atomic variable.
 * 
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomicPointer
 * @param   atomic      Pointer to atomic variable.
 * @param   value       New atomic value.
 * @return              Previous atomic value.
 */
kFx(kPointer) kAtomicPointer_Exchange(kAtomicPointer* atomic, kPointer value); 

/** 
 * Conditionally exchanges the value of an atomic variable.
 * 
 * If the atomic value is equal to the oldValue argument, then it is replaced by the value argument. 
 *
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomicPointer
 * @param   atomic      Pointer to atomic variable.
 * @param   oldValue    Previous atomic value.
 * @param   value       New atomic value.
 * @return              Previous atomic value.
 */
kFx(kBool) kAtomicPointer_CompareExchange(kAtomicPointer* atomic, kPointer oldValue, kPointer value); 

/** 
 * Gets the current value of an atomic variable. 
 * 
 * This method is thread-safe. Implements a full memory barrier. 
 *
 * @public              @memberof kAtomicPointer
 * @param   atomic      Pointer to atomic variable.
 * @return              Atomic value.
 */
kFx(kPointer) kAtomicPointer_Get(kAtomicPointer* atomic); 

/** @relates kAtomicPointer @{ */

#define kAtomicPointer_Init_(ATM, VAL)                      kxAtomicPointer_Init_(ATM, VAL)                     ///< Initializes a variable of type kAtomicPointer.
#define kAtomicPointer_Exchange_(ATM, VAL)                  kxAtomicPointer_Exchange_(ATM, VAL)                 ///< Macro version of kAtomicPointer_Exchange.
#define kAtomicPointer_CompareExchange_(ATM, OLD, VAL)      kxAtomicPointer_CompareExchange_(ATM, OLD, VAL)     ///< Macro version of kAtomicPointer_CompareExchange.
#define kAtomicPointer_Get_(ATM)                            kxAtomicPointer_Get_(ATM)                           ///< Macro version of kAtomicPointer_Get.

/** @} */

kEndHeader()

#include <kApi/Threads/kAtomic.x.h>

#endif
