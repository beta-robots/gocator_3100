/** 
 * @file    kApiLib.h
 * @brief   Zen library management functions. 
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_LIB_H
#define K_API_LIB_H

kBeginHeader()

/** Function signature for memory allocation handler. */
typedef kStatus (kCall* kApiMemAllocFx)(kPointer provider, kSize size, void* mem); 

/** Function signature for memory free handler. */
typedef kStatus (kCall* kApiMemFreeFx)(kPointer provider, void* mem); 

/** Function signature for debug assertion handler. */
typedef kStatus(kCall* kApiAssertFx)(const kChar* file, k32u line);

/** Function signature for debug log handler. */
typedef kStatus(kCall* kApiLogfFx)(const kChar* format, va_list args);

/** Function signature for a lock (exclusive access) operation. */
typedef k32u(kCall* kApiLockFx)(kPointer provider);

/** Function signature for an unlock (exclusive access) operation. */
typedef k32u(kCall* kApiUnlockFx)(kPointer provider);

/**
* @class   kApiLib
* @ingroup kApi
* @brief   Collection of library management functions.
*/

/** 
 * Constructs the Zen API type assembly (kApiLib). 
 *
 * This function initializes the Zen API assembly and returns a handle that represents the assembly. 
 * When the assembly is no longer needed, pass the assembly handle to the kObject_Destroy function. 
 * 
 * This function should be called prior to calling most other Zen API functions. The only exceptions to 
 * this rule are kApiLib functions that configure global handlers (e.g. kApiLib_SetMemAllocHandlers); these functions
 * should typically be called prior to kApiLib_Construct.
 * 
 * Multiple nested calls to this function will return the same assembly instance. To ensure final clean up, the 
 * kObject_Destroy function should be invoked a corresponding number of times on the assembly handle. 
 *
 * @public              @memberof kApiLib
 * @param   assembly    Receives a handle to the kApiLib type assembly.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_Construct(kAssembly* assembly);

/**
 * Sets a handler function for debug logging operations.
 *
 * By default, logging operations are ignored (log information discarded). Call this function to override 
 * the default behaviour.
 * 
 * This function is not thread-safe and should be called before calling kApiLib_Construct.
 *
 * @public              @memberof kApiLib
 * @param   function    Log callback function.
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_SetLogfHandler(kApiLogfFx function);

/**
 * Sets a handler function for debug assertions.
 *
 * By default, kApi uses C standard library functions for debug assertions.
 * Call this function to override the default behaviour.
 *
 * Note, assertion features also require K_DEBUG or K_ASSERT to be defined at
 * compile time.
 *
 * This function is not thread-safe and should be called before calling kApiLib_Construct.
 *
 * @public              @memberof kApiLib
 * @param   function    Assert callback function.
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_SetAssertHandler(kApiAssertFx function);

/** 
 * Sets handler functions for memory alloc/free operations. 
 *
 * By default, kApiLib uses C standard library functions for memory allocation. Call this function 
 * to override the default behaviour. 
 * 
 * This function is not thread-safe and should be called before calling kApiLib_Construct. 
 * 
 * @public              @memberof kApiLib
 * @param   allocFx     Memory allocation callback function.
 * @param   freeFx      Memory deallocation callback function.
 * @param   provider    Context pointer (provided to callback functions). 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetMemAllocHandlers(kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider); 

/**
 * Reports whether memory leaks occurred during use of this library.
 *
 * In debug builds, Zen tracks heap allocations. Call this function after destroying the kApiLib 
 * assembly to determine whether leaks occurred during operation.
 * 
 * To receive a detailed account of the detected leaks, ensure that a log handler is registered using the 
 * kApiLib_SetLogfHandler function. 
 *
 * This function is not thread-safe.
 *
 * @public              @memberof kApiLib
 * @return              kTRUE if leaks were detected; kFALSE otherwise.
 */
kFx(kBool) kApiLib_LeaksDetected();

/**
 * Enables or disables initialization of network services by this library.
 *
 * On Windows, by default, the WSAStartup function is used to initialize the winsock interface. Use the 
 * kApiLib_EnableNetworkInitialization function (with kFALSE argument) to suppress any calls to WSAStartup/WSACleanup.
 *
 * This function is not thread-safe and should be called before calling kApiLib_Construct.
 * 
 * @public              @memberof kApiLib
 * @param   enable      kTRUE for automatic network initialization; kFALSE otherwise.
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_EnableNetworkInitialization(kBool enable);

/**
 * Enables or disables initialization of symbol table resources by this library.
 *
 * On some platforms (e.g., Windows) symbol table services must be explicitly initialized before use. In kApiLib debug 
 * builds, this is performed automatically by default. Use the kApiLib_EnableSymbolInitialization function (with kFALSE argument) 
 * to disable automatic initialization.
 *
 * This function is not thread-safe and should be called before calling kApiLib_Construct.
 * 
 * @public              @memberof kApiLib
 * @param   enable      kTRUE for automatic symbol initialization; kFALSE otherwise.
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_EnableSymbolInitialization(kBool enable);

/**
 * Sets handler functions for symbol table lock/unlock operations.
 *
 * In debug builds, kApiLib may access the debug symbols associated with this process. On some platforms
 * (e.g., Windows) the symbol table is a global resource that is not automatically thread-safe. This function
 * can be used to provide callback functions to mediate access to the symbol table.
 *
 * This function is not thread-safe and should be called before calling kApiLib_Construct.
 *
 * @public              @memberof kApiLib
 * @param   lockFx      Lock callback function
 * @param   unlockFx    Unlock callback function.
 * @param   provider    Context pointer (provided to callback functions).
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_SetSymbolLockHandlers(kApiLockFx lockFx, kApiUnlockFx unlockFx, kPointer provider);

kEndHeader()

#include <kApi/kApiLib.x.h>

#endif
