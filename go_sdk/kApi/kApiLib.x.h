/** 
 * @file    kApiLib.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_LIB_X_H
#define K_API_API_LIB_X_H

kBeginHeader()

kDeclareAssembly(k, kApiLib)

/** Function signature for memory set handler. */
typedef kStatus (kCall* kApiMemSetFx)(void* dest, kByte fill, kSize size); 

/** Function signature for memory copy/move handler.  */
typedef kStatus (kCall* kApiMemCopyFx)(void* dest, const void* src, kSize size); 

/** Function signature for application allocator construction handler.  */
typedef kStatus (kCall* kApiAllocConstructFx)(kAlloc* appAlloc, kAlloc systemAlloc); 

/** Function signature for application allocator destroy handler.  */
typedef kStatus (kCall* kApiAllocDestroyFx)(kAlloc appAlloc); 

/** Function signature for debug trace handler. */
typedef kStatus (kCall* kApiTraceFx)(const kChar* tag, const kChar* file, k32u line); 

/** Function signature for timer tick query handler; returns ticks. */
typedef k64u (kCall* kApiTimerQueryFx)(); 

/** Function signature for random number generator. */
typedef k32u (kCall* kApiRandomFx)(); 

/**
 * @struct  kApiFileFx
 * @ingroup kApi  
 * @brief   Collection of callbacks for file operations.
 * @see     kApiLib_SetFileHandlers
 */
typedef struct kApiFileFx
{
    kStatus (kCall* open)(kFile file, const kChar* path, kFileMode mode); 
    kStatus (kCall* close)(kFile file); 
    kStatus (kCall* read)(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead); 
    kStatus (kCall* write)(kFile file, const kByte* buffer, kSize count); 
    kStatus (kCall* flush)(kFile file); 
    k64u (kCall* seek)(kFile file, k64s offset, kSeekOrigin origin); 
    k64u (kCall* size)(const kChar* path);
    kBool (kCall* exists)(const kChar* path);
    kStatus (kCall* copy)(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context);
    kStatus (kCall* move)(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context);
    kStatus (kCall* del)(const kChar* path);
    kStatus (kCall* tempName)(kChar* name, kSize capacity);
} kApiFileFx; 

/**
 * @struct  kApiDirectoryFx
 * @ingroup kApi  
 * @brief   Collection of callbacks for directory and path operations.
 * @see     kApiLib_SetDirectoryHandlers
 */
typedef struct kApiDirectoryFx
{
    kStatus (kCall* create)(const kChar* path);
    kBool (kCall* exists)(const kChar* path);
    kStatus (kCall* move)(const kChar* source, const kChar* destination);
    kStatus (kCall* del)(const kChar* path); 
    kStatus (kCall* enumerate)(const kChar* directory, kBool includeFiles, kBool includeDirectories, kArrayList entries); 
    kStatus (kCall* setCurrent)(const kChar* directory); 
    kStatus (kCall* current)(kChar* directory, kSize capacity); 
    kStatus (kCall* appDirectory)(kChar* path, kSize capacity);
    kStatus (kCall* tempDirectory)(kChar* path, kSize capacity);
    kStatus (kCall* appConfigDirectory)(const kChar* appName, kChar* path, kSize capacity);
    kStatus (kCall* appDataDirectory)(const kChar* appName, kChar* path, kSize capacity);
    kStatus (kCall* pluginDirectory)(kChar* directory, kSize capacity);
    kStatus (kCall* toVirtual)(const kChar* path, kChar* vpath, kSize capacity);
    kStatus (kCall* fromVirtual)(const kChar* vpath, kChar* path, kSize capacity);
} kApiDirectoryFx; 

/**
 * @struct  kApiAtomicFx
 * @ingroup kApi  
 * @brief   Collection of callbacks for atomic variable operations.
 * @see     kApiLib_SetAtomicHandlers
 */
typedef struct kApiAtomicFx
{
    k32s (kCall* increment32s)(kAtomic32s* atomic); 
    k32s (kCall* decrement32s)(kAtomic32s* atomic); 
    k32s (kCall* exchange32s)(kAtomic32s* atomic, k32s value); 
    kBool (kCall* compareExchange32s)(kAtomic32s* atomic, k32s oldValue, k32s value); 
    k32s (kCall* get32s)(kAtomic32s* atomic); 
    kPointer (kCall* exchangePointer)(kAtomicPointer* atomic, kPointer value); 
    kBool (kCall* compareExchangePointer)(kAtomicPointer* atomic, kPointer oldValue, kPointer value); 
    kPointer (kCall* getPointer)(kAtomicPointer* atomic); 
} kApiAtomicFx; 

/* 
 * Library configuration variables. Use the functions and macros provided in this module
 * to access/modify these variables (do not manipulate directly).
 */

extern kDx(kApiMemAllocFx) kApiLib_memAllocFx;                  ///< User-provided callback function for memory allocation.
extern kDx(kApiMemFreeFx) kApiLib_memFreeFx;                    ///< User-provided callback function for memory deallocation.
extern kDx(kPointer) kApiLib_memAllocProvider;                  ///< User-provided context pointer for memory allocation/deallocation.
extern kDx(kApiMemSetFx) kApiLib_memSetFx;                      ///< User-provided callback function for memory initialization.
extern kDx(kApiMemCopyFx) kApiLib_memCopyFx;                    ///< User-provided callback function for memory copy.
extern kDx(kApiMemCopyFx) kApiLib_memMoveFx;                    ///< User-provided callback function for memory move.
extern kDx(kApiAllocConstructFx) kApiLib_appAllocConstructFx;   ///< User-provided callback function for application allocator construction.
extern kDx(kApiAllocDestroyFx) kApiLib_appAllocDestroyFx;       ///< User-provided callback function for application allocator destruction.

extern kDx(kApiLogfFx) kApiLib_logfFx;                     ///< User-provided callback function for formatted logging operations. 
extern kDx(kApiTraceFx) kApiLib_traceFx;                   ///< User-provided callback function for trace operations.
extern kDx(kApiAssertFx) kApiLib_assertFx;                 ///< User-provided callback function for assertions.

extern kDx(kBool) kApiLib_leakLoggingEnabled;              ///< User-configurable option to enable memory leak logging.
extern kDx(kBool) kApiLib_leaksDetected;                   ///< Leak status, available after kApi assembly is destroyed. 
extern kDx(kBool) kApiLib_checkTraceEnabled;               ///< User-configurable option to enable check-trace feature.

extern kDx(k64u) kApiLib_timerMultiplier;                  ///< User-provided constant for high-resolution timer multiplier (us = mult*ticks/div).
extern kDx(k64u) kApiLib_timerDivider;                     ///< User-provided constant for high-resolution timer divider (us = mult*ticks/div).
extern kDx(kApiTimerQueryFx) kApiLib_timerQueryFx;         ///< User-provided callback function for high-resolution timer queries.

extern kDx(k64u) kApiLib_kernelTimerMultiplier;            ///< User-provided constant for kernel clock multiplier (us = mult*ticks/div).
extern kDx(k64u) kApiLib_kernelTimerDivider;               ///< User-provided constant for kernel clock divider (us = mult*ticks/div).

extern kDx(kApiRandomFx) kApiLib_randomFx;                 ///< User-provided callback function for random number generation.

extern kDx(kBool) kApiLib_hasFileFx;                       ///< Flag to indicate whether file callbacks have been provided.
extern kDx(kApiFileFx) kApiLib_fileFx;                     ///< User-provided callback functions for file operations.

extern kDx(kBool) kApiLib_hasDirectoryFx;                  ///< Flag to indicate whether directory callbacks have been provided.
extern kDx(kApiDirectoryFx) kApiLib_directoryFx;           ///< User-provided callback functions for directory operations.

extern kDx(kBool) kApiLib_hasAtomicFx;                     ///< Flag to indicate whether atomic callbacks have been provided.
extern kDx(kApiAtomicFx) kApiLib_atomicFx;                 ///< User-provided callback functions for atomic operations.

extern kDx(kChar) kApiLib_nativeSeparator;                 ///< Native path separator for underlying system.

extern kDx(kBool) kApiLib_networkInitializationEnabled;    ///< Should kApiLib initialize the network interface library?

extern kDx(kApiLockFx) kApiLib_symbolLockFx;               ///< User-provided callback function for symbol table lock. 
extern kDx(kApiUnlockFx) kApiLib_symbolUnlockFx;           ///< User-provided callback function for symbol table unlock. 
extern kDx(kPointer) kApiLib_symbolLockProvider;           ///< User-provided context pointer for symbol table lock/unlock. 
extern kDx(kBool) kApiLib_symbolInitializationEnabled;     ///< Should kApiLib initialize the back trace library?

/** 
 * Sets a handler function for memory copy operations. 
 *
 * By default, kApi uses C standard library functions for memory copy operations. 
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Memory copy callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetMemCopyHandler(kApiMemCopyFx function); 

/** 
 * Sets a handler function for memory move operations. 
 *
 * By default, kApi uses C standard library functions for memory move operations. 
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Memory move callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetMemMoveHandler(kApiMemCopyFx function); 

/** 
 * Sets a handler to construct the application allocator object. 
 *
 * This function is not thread-safe.
 *
 * @param   function    Allocation construction function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetAppAllocConstructHandler(kApiAllocConstructFx function); 

/** 
 * Sets a handler to destroy the application allocator object. 
 *
 * This function is not thread-safe.
 *
 * @param   function    Allocation destruction function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetAppAllocDestroyHandler(kApiAllocDestroyFx function); 

/** 
 * (Deprecated) Sets a handler function for trace operations. 
 *
 * By default, trace operations are ignored (trace information discarded). 
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Trace callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetTraceHandler(kApiTraceFx function); 

/** 
 * (Deprecated) Enables or disables leak logging.
 *
 * By default, memory leaks are sent to the kApi log handler. This function can be used
 * to disable leak logging.
 * 
 * Note, leak logging also require K_DEBUG to be defined at compile time.
 * 
 * This function is not thread-safe.
 *
 * @param   enabled     Specifies whether leak-logging is enabled.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_EnableLeakLogging(kBool enabled); 

/** 
 * Sets a listener function that will be notified on debug heap allocations.
 *
 * The 'args' parameter passed to the listener function will be a pointer to 
 * a kDebugAllocation structure. This feature can be used while debugging to break 
 * on a particular allocation index, without modifying code inside the kApi library.
 * This technique can be helpful in tracking down heap leaks and corruptions. 
 * 
 * This function should be called after constructing the kApi assembly.
 * 
 * Note, debug allocation notification requires K_DEBUG to be defined at compile 
 * time.
 *
 * @param   function    Debug memory allocation listener function.
 * @param   receiver    Context argument to be passed to listener. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetDebugAllocListener(kCallbackFx function, kPointer receiver); 

/** 
 * (Deprecated) Enables an option to trace kCheck/kTest failures. 
 *
 * By default, kCheck/Test failures do not have any side-effects. 
 * Call this function enable an option that will generate trace calls 
 * for any kCheck/kTest failure.
 * 
 * Note, check-tracing also requires K_DEBUG and K_CHECK_TRACE to be defined at 
 * compile time.
 * 
 * This function is not thread-safe.
 *
 * @param   enabled     Specifies whether check-tracing is enabled.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_EnableCheckTrace(kBool enabled); 

/** 
 * Sets the scale for high-precision timer calculations.
 *
 * By default, kApi uses platform-specific library functions for high-resolution
 * timers. Use this function in conjunction with kApiLib_SetTimerQueryHandler to override 
 * the default behaviour.
 * 
 * Timer calculations use the following relationship: time = ticks * multiplier / divider.
 * Multiplier and divider values should be selected to produce a time value in microseconds, 
 * and without causing 64-bit integer overflow. Ticks are determined by querying the callback 
 * provided to kApiLib_SetTimerQueryHandler.  
 * 
 * This function is not thread-safe.
 *
 * @param   multiplier  Tick multiplier for timer calculations.
 * @param   divider     Tick divider for timer calculations.
 * @return              Operation status. 
 * @see                 kApiLib_SetTimerQueryHandler
 */
kFx(kStatus) kApiLib_SetTimerScale(k64u multiplier, k64u divider); 

/** 
 * Sets a handler function for timer queries. 
 *
 * By default, kApi uses platform-specific library functions for high-resolution
 * timers. Use this function in conjunction with kApiLib_SetTimerScale to override 
 * the default behaviour.
 * 
 * This function is not thread-safe.
 *
 * @param   function    Memory allocation callback function.
 * @return              Operation status. 
 * @see                 kApiLib_SetTimerScale
 */
kFx(kStatus) kApiLib_SetTimerQueryHandler(kApiTimerQueryFx function); 

/** 
 * Sets the scale for kernel time calculations.
 *
 * Time calculations use the following relationship: time = ticks * multiplier / divider.
 * Multiplier and divider values should be selected to produce a time value in microseconds, 
 * and without causing 64-bit integer overflow. 
 * 
 * This function is not thread-safe.
 *
 * @param   multiplier  Tick multiplier for kernel timer calculations.
 * @param   divider     Tick divider for kernel timer calculations.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetKernelTimerScale(k64u multiplier, k64u divider); 

/** 
 * Sets a handler function for generating random numbers.
 *
 * By default, kApi uses C standard library functions for random number generation.
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Random number generation callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetRandomHandler(kApiRandomFx function); 

/** 
 * Sets handler functions for file operations.
 *
 * By default, kApi uses platform-specific functions for file operations.
 * However, some embedded platforms do not provide a standard file interface. On these systems, 
 * this function can be used to provide an interface to a custom/ad-hoc file system.
 * 
 * This function is not thread-safe.
 *
 * @param   functions   File callback functions. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetFileHandlers(kApiFileFx* functions); 

/** 
 * Sets handler functions for directory operations.
 *
 * By default, kApi uses platform-specific functions for directory operations.
 * However, some embedded platforms do not provide a standard directory interface. 
 * On these systems, this function can be used to provide an interface to a 
 * custom/ad-hoc directory system.
 * 
 * This function is not thread-safe.
 *
 * @param   functions   Directory callback functions. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetDirectoryHandlers(kApiDirectoryFx* functions); 

/** 
 * Sets handler functions for atomic variable operations.
 *
 * By default, kApi uses platform-specific functions for atomic variable operations.
 * However, some platforms do not provide standard utility functions for working with 
 * atomic variables. On these systems, this function can be used to implement support
 * for atomic operations.
 * 
 * This function is not thread-safe.
 *
 * @param   functions   Atomic callback functions. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetAtomicHandlers(kApiAtomicFx* functions); 

/** 
 * Sets native path separator used by underlying operating system.
 *
 * This function is not thread-safe.
 *
 * @param   separator   Native separator. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetNativeSeparator(kChar separator); 

/** 
 * Raises an assertion (or aborts) in the event of an object cast failure.
 *
 * This utility function is used by kApi type-checking macros; it is not intended to be 
 * called directly. 
 *
 * @param   file    Source file name in which cast failure occurred.
 * @param   line    Line number at which cast failure occurred.
 * @return          Returns kNull. 
 */
kFx(kPointer) kApiLib_CastFailHandler(const kChar* file, k32s line);

#define kApiLib_MemAllocHandler_()                 (kApiLib_memAllocFx)                     ///< Gets user-provided callback function for system memory allocation.
#define kApiLib_MemFreeHandler_()                  (kApiLib_memFreeFx)                      ///< Gets user-provided callback function for system memory deallocation.
#define kApiLib_MemAllocProvider_()                (kApiLib_memAllocProvider)               ///< Gets user-provided context pointer for memory allocation/deallocation.
#define kApiLib_MemSetHandler_()                   (kApiLib_memSetFx)                       ///< Gets user-provided callback function for memory initialization.
#define kApiLib_MemCopyHandler_()                  (kApiLib_memCopyFx)                      ///< Gets user-provided callback function for memory copy.
#define kApiLib_MemMoveHandler_()                  (kApiLib_memMoveFx)                      ///< Gets user-provided callback function for memory move.
#define kApiLib_AppAllocConstructHandler_()        (kApiLib_appAllocConstructFx)            ///< Gets user-provided callback function to construct application allocator.
#define kApiLib_AppAllocDestroyHandler_()          (kApiLib_appAllocDestroyFx)              ///< Gets user-provided callback function to destroy application allocator.

#define kApiLib_LogfHandler_()                     (kApiLib_logfFx)                         ///< Gets user-provided callback function for formatted logging operations. 
#define kApiLib_TraceHandler_()                    (kApiLib_traceFx)                        ///< Gets user-provided callback function for trace operations.
#define kApiLib_AssertHandler_()                   (kApiLib_assertFx)                       ///< Gets user-provided callback function for assertions.

#define kApiLib_CheckTraceEnabled_()               (kApiLib_checkTraceEnabled)              ///< Gets user-configurable option to enable check-trace feature.
#define kApiLib_LeakLoggingEnabled_()              (kApiLib_leakLoggingEnabled)             ///< Gets user-configurable option to enable leak-logging feature.
#define kApiLib_LeaksDetected_()                   (kApiLib_leaksDetected)                  ///< Reports whether leaks occurred during kApi library tear-down.
#define kApiLib_SetLeaksDetected_()                (kApiLib_leaksDetected = kTRUE)          ///< Used within the kApi library to set the leaks-detected flag.

#define kApiLib_TimerMultiplier_()                 (kApiLib_timerMultiplier)                ///< Gets user-provided constant for high-resolution timer multiplier.
#define kApiLib_TimerDivider_()                    (kApiLib_timerDivider)                   ///< Gets user-provided constant for high-resolution timer divider.
#define kApiLib_TimerQueryHandler_()               (kApiLib_timerQueryFx)                   ///< Gets user-provided callback function for timer queries.

#define kApiLib_KernelTimerMultiplier_()           (kApiLib_kernelTimerMultiplier)          ///< Gets user-provided constant for kernel timer multiplier.
#define kApiLib_KernelTimerDivider_()              (kApiLib_kernelTimerDivider)             ///< Gets user-provided constant for kernel timer divider.

#define kApiLib_RandomHandler_()                   (kApiLib_randomFx)                       ///< Gets user-provided callback function for random number generation.

#define kApiLib_HasFileHandlers_()                 (kApiLib_hasFileFx)                      ///< Gets flag to indicate whether file callbacks have been provided.
#define kApiLib_FileHandlers_()                    (&kApiLib_fileFx)                        ///< Gets user-provided callback functions for file operations.

#define kApiLib_HasDirectoryHandlers_()            (kApiLib_hasDirectoryFx)                 ///< Gets flag to indicate whether directory callbacks have been provided.
#define kApiLib_DirectoryHandlers_()               (&kApiLib_directoryFx)                   ///< Gets user-provided callback functions for directory operations.

#define kApiLib_HasAtomicHandlers_()               (kApiLib_hasAtomicFx)                    ///< Gets flag to indicate whether atomic callbacks have been provided.
#define kApiLib_AtomicHandlers_()                  (&kApiLib_atomicFx)                      ///< Gets user-provided callback functions for atomic operations.

#define kApiLib_NativeSeparator_()                 (kApiLib_nativeSeparator)                ///< Gets native separator used by underlying operating system.

#define kApiLib_NetworkInitializationEnabled_()    (kApiLib_networkInitializationEnabled)   ///< Reports whether network stack should be initialized by this library.

#define kApiLib_SymbolLockHandler_()               (kApiLib_symbolLockFx)                   ///< Gets user-provided callback function for symbol table lock operation.
#define kApiLib_SymbolUnlockHandler_()             (kApiLib_symbolUnlockFx)                 ///< Gets user-provided callback function for symbol table unlock operation.
#define kApiLib_SymbolLockProvider_()              (kApiLib_symbolLockProvider)             ///< Gets user-provided context pointer for symbol table lock/unlock.

#define kApiLib_SymbolInitializationEnabled_()     (kApiLib_symbolInitializationEnabled)    ///< Reports whether symbol services should be be initialized by this library.

kEndHeader()

#endif

