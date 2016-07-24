/** 
 * @file    kApiLib.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiLib.h>
#include <kApi/kApi.h>
#include <kApi/Utils/kUtils.h>

/* 
 * Library configuration variables. See comments in kApiLib.x.h.
 */

kDx(kApiMemAllocFx) kApiLib_memAllocFx = kDefaultMemAlloc; 
kDx(kApiMemFreeFx) kApiLib_memFreeFx = kDefaultMemFree; 
kDx(kPointer) kApiLib_memAllocProvider = kNULL; 
kDx(kApiMemSetFx) kApiLib_memSetFx = kNULL; 
kDx(kApiMemCopyFx) kApiLib_memCopyFx = kNULL; 
kDx(kApiMemCopyFx) kApiLib_memMoveFx = kNULL; 
kDx(kApiAllocConstructFx) kApiLib_appAllocConstructFx = kAlloc_DefaultConstructAppAlloc;
kDx(kApiAllocDestroyFx) kApiLib_appAllocDestroyFx = kAlloc_DefaultDestroyAppAlloc;

kDx(kApiLogfFx) kApiLib_logfFx = kNULL; 
kDx(kApiTraceFx) kApiLib_traceFx = kNULL; 
kDx(kApiAssertFx) kApiLib_assertFx = kNULL; 

kDx(kBool) kApiLib_leakLoggingEnabled = kTRUE; 
kDx(kBool) kApiLib_leaksDetected = kFALSE; 
kDx(kBool) kApiLib_checkTraceEnabled = kFALSE; 

kDx(k64u) kApiLib_timerMultiplier = 1; 
kDx(k64u) kApiLib_timerDivider = 1; 
kDx(kApiTimerQueryFx) kApiLib_timerQueryFx = kNULL; 

kDx(k64u) kApiLib_kernelTimerMultiplier = 1000;        /* by default, assume kernel time unit is 1 ms */
kDx(k64u) kApiLib_kernelTimerDivider = 1; 

kDx(kApiRandomFx) kApiLib_randomFx = kNULL; 

kDx(kBool) kApiLib_hasFileFx = kFALSE;        
kDx(kApiFileFx) kApiLib_fileFx = { kNULL };                

kDx(kBool) kApiLib_hasDirectoryFx = kFALSE;        
kDx(kApiDirectoryFx) kApiLib_directoryFx = { kNULL };      

kDx(kBool) kApiLib_hasAtomicFx = kFALSE;        
kDx(kApiAtomicFx) kApiLib_atomicFx = { kNULL };     

kDx(kChar) kApiLib_nativeSeparator = 0;             

kDx(kBool) kApiLib_networkInitializationEnabled = kTRUE; 

kDx(kApiLockFx) kApiLib_symbolLockFx = kNULL; 
kDx(kApiUnlockFx) kApiLib_symbolUnlockFx = kNULL; 
kDx(kPointer) kApiLib_symbolLockProvider = kNULL;
kDx(kBool) kApiLib_symbolInitializationEnabled = kTRUE; 

/* 
 * Assembly types.
 */

kBeginAssembly(k, kApiLib, kAPI_VERSION)

    //Interfaces
    kAddType(kCollection)
    kAddType(kObjectPool)

    //Values
    kAddType(k16u)
    kAddType(k16s)
    kAddType(k32f)
    kAddType(k32u)
    kAddType(k32s)
    kAddType(k64f)
    kAddType(k64u)
    kAddType(k64s)
    kAddType(k8u)
    kAddType(k8s)
    kAddType(kArgb)
    kAddType(kBool)
    kAddType(kByte)
    kAddType(kCallback)   
    kAddType(kCallbackFx)
    kAddType(kCfa)
    kAddType(kChar)
    kAddType(kComparison)
    kAddType(kDat5SerializerTypeInfo)
    kAddType(kDat6SerializerTypeInfo)
    kAddType(kDebugAllocation)
    kAddType(kFileMode)
    kAddType(kHttpStatus)
    kAddType(kIpAddress)
    kAddType(kIpEndPoint)
    kAddType(kIpEntry)
    kAddType(kIpVersion)
    kAddType(kPixelFormat)
    kAddType(kPoint16s)
    kAddType(kPoint32s)
    kAddType(kPoint32f)
    kAddType(kPoint64f)
    kAddType(kPoint3d16s)
    kAddType(kPoint3d32s)
    kAddType(kPoint3d32f)
    kAddType(kPoint3d64f)
    kAddType(kPointer)
    kAddType(kRect16s)
    kAddType(kRect32s)
    kAddType(kRect32f)
    kAddType(kRect64f)
    kAddType(kRect3d64f)
    kAddType(kRgb)
    kAddType(kRotatedRect32s)
    kAddType(kRotatedRect32f)
    kAddType(kSeekOrigin)
    kAddType(kSerializerWriteSection)
    kAddType(kSize)
    kAddType(kSSize)
    kAddType(kSocketType)
    kAddType(kStatus)
    kAddType(kText16)
    kAddType(kText32)
    kAddType(kText64)
    kAddType(kText128)
    kAddType(kText256)
    kAddType(kValue)
    kAddType(kVersion)
    kAddType(kVoid)
    kAddType(kXmlAttrBlock)
    kAddType(kXmlItemBlock)
    kAddBytes() //K_COMPAT_5

    //Classes
    kAddType(kAlloc)
    kAddType(kArray1)
    kAddType(kArray2)
    kAddType(kArray3)
    kAddType(kArrayList)
    kAddType(kAssembly)
    kAddType(kAtomic)
    kAddType(kBackTrace)
    kAddType(kBox)
    kAddType(kDat6Serializer)
    kAddType(kDat5Serializer)
    kAddType(kDebugAlloc)
    kAddType(kDirectory)
    kAddType(kDynamicLib)
    kAddType(kEvent)
    kAddType(kFile)
    kAddType(kHttpServer)
    kAddType(kHttpServerChannel)
    kAddType(kHttpServerRequest)
    kAddType(kHttpServerResponse)
    kAddType(kImage)
    kAddType(kList)
    kAddType(kLock)
    kAddType(kMap)
    kAddType(kMath)
    kAddType(kMemory)
    kAddType(kMsgQueue)
    kAddType(kNetwork)
    kAddType(kObject)
    kAddType(kPath)
    kAddType(kPeriodic)
    kAddType(kPlugin)
    kAddType(kPoolAlloc)
    kAddType(kQueue)
    kAddType(kSemaphore)
    kAddType(kSerializer)
    kAddType(kSocket)
    kAddType(kStream)
    kAddType(kString)
    kAddType(kTcpClient)
    kAddType(kTcpServer)
    kAddType(kThread)
    kAddType(kTimer)
    kAddType(kType)
    kAddType(kUdpClient)
    kAddType(kUserAlloc)
    kAddType(kUtils)
    kAddType(kXml)

    //Initialization order
    kAddPriority(kAtomic)
    kAddPriority(kUtils)
    kAddPriority(kAlloc)
    kAddPriority(kAssembly)

kEndAssembly()

/* 
 * Library management functions. 
 */

kFx(kStatus) kApiLib_SetMemAllocHandlers(kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider)
{
    kApiLib_memAllocFx = allocFx; 
    kApiLib_memFreeFx = freeFx; 
    kApiLib_memAllocProvider = provider; 

    return kOK;
}

kFx(kStatus) kApiLib_SetMemSetHandler(kApiMemSetFx function)
{
    kApiLib_memSetFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetMemCopyHandler(kApiMemCopyFx function)
{
    kApiLib_memCopyFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetMemMoveHandler(kApiMemCopyFx function)
{
    kApiLib_memMoveFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetAppAllocConstructHandler(kApiAllocConstructFx function)
{
    kApiLib_appAllocConstructFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetAppAllocDestroyHandler(kApiAllocDestroyFx function)
{
    kApiLib_appAllocDestroyFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetLogfHandler(kApiLogfFx function)
{
    kApiLib_logfFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetTraceHandler(kApiTraceFx function)
{
    kApiLib_traceFx = function; 

    return kOK; 
} 

kFx(kStatus) kApiLib_EnableCheckTrace(kBool enabled)
{
    kApiLib_checkTraceEnabled = enabled; 

    return kOK; 
}

kFx(kPointer) kApiLib_CastFailHandler(const kChar* file, k32s line)
{
    if (kApiLib_AssertHandler_())
    {
        kApiLib_AssertHandler_()(file, line); 
    }
    else
    {
       abort(); 
    }

    return kNULL; 
}

kFx(kStatus) kApiLib_EnableLeakLogging(kBool enabled)
{
    kApiLib_leakLoggingEnabled = enabled; 

    return kOK; 
}

kFx(kBool) kApiLib_LeaksDetected()
{
    return kApiLib_leaksDetected; 
}

kFx(kStatus) kApiLib_SetDebugAllocListener(kCallbackFx function, kPointer receiver)
{
    kAlloc alloc = kAlloc_App(); 

    if (kObject_Is(alloc, kTypeOf(kDebugAlloc)))
    {
        kCheck(kDebugAlloc_SetAllocListener(alloc, function, receiver)); 
    }

    return kOK; 
}

kFx(kStatus) kApiLib_SetAssertHandler(kApiAssertFx function)
{
    kApiLib_assertFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetTimerScale(k64u multiplier, k64u divider)
{
    kApiLib_timerMultiplier = multiplier; 
    kApiLib_timerDivider = divider; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetTimerQueryHandler(kApiTimerQueryFx function)
{
    kApiLib_timerQueryFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetKernelTimerScale(k64u multiplier, k64u divider)
{
    kApiLib_kernelTimerMultiplier = multiplier; 
    kApiLib_kernelTimerDivider = divider; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetRandomHandler(kApiRandomFx function)
{
    kApiLib_randomFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetFileHandlers(kApiFileFx* functions)
{
    if (!kApiLib_hasFileFx)
    {
        kZero_(kApiLib_fileFx); 
        kApiLib_hasFileFx = kTRUE; 
    }

    return kOverrideFunctions(&kApiLib_fileFx, sizeof(kApiLib_fileFx), functions); 
}

kFx(kStatus) kApiLib_SetDirectoryHandlers(kApiDirectoryFx* functions)
{
    if (!kApiLib_hasDirectoryFx)
    {
        kZero_(kApiLib_directoryFx); 
        kApiLib_hasDirectoryFx = kTRUE; 
    }

    return kOverrideFunctions(&kApiLib_directoryFx, sizeof(kApiLib_directoryFx), functions); 
}

kFx(kStatus) kApiLib_SetAtomicHandlers(kApiAtomicFx* functions)
{
    kApiLib_atomicFx = *functions; 
    kApiLib_hasAtomicFx = kTRUE; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetNativeSeparator(kChar separator)
{
    kApiLib_nativeSeparator = separator; 

    return kOK; 
}

kFx(kStatus) kApiLib_EnableNetworkInitialization(kBool enable)
{
    kApiLib_networkInitializationEnabled = enable; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetSymbolLockHandlers(kApiLockFx lockFx, kApiUnlockFx unlockFx, kPointer provider)
{
    kApiLib_symbolLockFx = lockFx; 
    kApiLib_symbolUnlockFx = unlockFx; 
    kApiLib_symbolLockProvider = provider; 

    return kOK; 
}

kFx(kStatus) kApiLib_EnableSymbolInitialization(kBool enable)
{
    kApiLib_symbolInitializationEnabled = enable; 

    return kOK; 
}

