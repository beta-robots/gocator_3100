/** 
 * @file    kBackTrace.c
 *
 * @internal
 * Copyright (C) 2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kBackTrace.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Io/kPath.h>

kBeginFullClass(k, kBackTrace, kObject)
    kAddVMethod(kBackTrace, kObject, VInitClone)
    kAddVMethod(kBackTrace, kObject, VEquals)
    kAddVMethod(kBackTrace, kObject, VHashCode)
kEndFullClass()

kFx(kStatus) kBackTrace_Construct(kBackTrace* trace, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kBackTrace), trace)); 

    if (!kSuccess(status = kBackTrace_Init(*trace, kTypeOf(kBackTrace), alloc)))
    {
        kAlloc_FreeRef(alloc, trace); 
    }

    return status; 
}

kFx(kStatus) kBackTrace_Init(kBackTrace trace, kType type, kAlloc allocator)
{
    kBackTraceClass* obj = trace; 

    kCheck(kObject_Init_(trace, type, allocator)); 

    obj->depth = 0;     
    obj->skip = 0; 

    return kOK; 
}

kFx(kStatus) kBackTrace_VInitClone(kBackTrace trace, kBackTrace source, kAlloc allocator)
{
    kBackTraceClass* obj = trace; 
    kBackTraceClass* srcObj = kBackTrace_Cast_(source); 

    kCheck(kObject_Init_(trace, kTypeOf(kBackTrace), allocator)); 

    obj->depth = srcObj->depth; 
    obj->skip = srcObj->skip; 

    kMemCopy(&obj->functions[0], &srcObj->functions[0], srcObj->depth*sizeof(kPointer)); 

    return kOK; 
}

kFx(kBool) kBackTrace_VEquals(kBackTrace trace, kObject other)
{
    kAssertType(trace, kBackTrace); 

    if (kObject_Type_(other) == kTypeOf(kBackTrace))
    {
        kBackTraceClass* obj = trace; 
        kBackTraceClass* otherObj = kBackTrace_Cast_(other); 

        if ((obj->depth - obj->skip) == (otherObj->depth - otherObj->skip))
        {
            return kMemEquals(&obj->functions[obj->skip], &otherObj->functions[obj->skip], (obj->depth - obj->skip)*sizeof(kPointer)); 
        }
    }
  
    return kFALSE; 
}

kFx(kSize) kBackTrace_VHashCode(kBackTrace trace)
{
    kBackTraceClass* obj = kBackTrace_Cast_(trace); 
    kSize sizeBits = K_POINTER_SIZE*8; 
    kSize hashCode = 0; 
    kSize i; 
  
    for (i = obj->skip; i < obj->depth; ++i)
    {
        kSize shift = i & (sizeBits - 1); 
        kSize value = (kSize) obj->functions[i]; 

        if (shift > 0)
        {           
            value = (value << shift) | (value >> (sizeBits - shift)); 
        }

        hashCode = hashCode ^ value;  
    }

    return hashCode; 
}

kFx(kSize) kBackTrace_Depth(kBackTrace trace)
{
    kBackTraceClass* obj = kBackTrace_Cast_(trace); 
  
    return obj->depth - obj->skip; 
}

#if defined(K_DEBUG) && defined(K_WINDOWS) && _MSC_VER >= 1500

#include <DbgHelp.h>

kFx(kStatus) kBackTrace_InitStatic()
{
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 
    kPointer symbolService = GetCurrentProcess();

    if (kApiLib_SymbolInitializationEnabled_())
    {
        if (!SymInitializeW(symbolService, kNULL, kTRUE))
        {
            kLogf("kBackTrace: SymInitialize failed."); 
        }
        else
        {
            sobj->service = symbolService; 
        }
    }
    else
    {
        sobj->service = symbolService; 
    }

    kCheck(kLock_Construct(&sobj->lock, kNULL)); 

    kCheck(kAssembly_AddLoadHandler(kBackTrace_OnAssemblyLoad, kNULL)); 

    return kOK; 
}

kFx(kStatus) kBackTrace_ReleaseStatic()
{
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 

    kAssembly_RemoveLoadHandler(kBackTrace_OnAssemblyLoad, kNULL); 

    if (kApiLib_SymbolInitializationEnabled_() && !kIsNull(sobj->service))
    {
        SymCleanup(sobj->service); 
    }

    kCheck(kObject_Destroy(sobj->lock)); 

    return kOK; 
}

kFx(void) kBackTrace_Lock()
{
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 

    if (!kIsNull(kApiLib_SymbolLockHandler_()))
    {
        kApiLib_SymbolLockHandler_()(kApiLib_SymbolLockProvider_()); 
    }
    else if (!kIsNull(sobj->lock))
    {
        kLock_Enter(sobj->lock); 
    }
}

kFx(void) kBackTrace_Unlock()
{
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 

    if (!kIsNull(kApiLib_SymbolUnlockHandler_()))
    {
        kApiLib_SymbolUnlockHandler_()(kApiLib_SymbolLockProvider_()); 
    }
    else if (!kIsNull(sobj->lock))
    {
        kLock_Exit(sobj->lock); 
    }
}

kFx(kStatus) kBackTrace_OnAssemblyLoad(kBackTrace trace, kAssembly assembly, kPointer unused)
{   
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 

    kBackTrace_Lock(); 
    {
        if (kApiLib_SymbolInitializationEnabled_() && !kIsNull(sobj->service))
        {
            SymRefreshModuleList(sobj->service); 
        }
    }
    kBackTrace_Unlock(); 
   
    return kOK; 
}

kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip) 
{
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 
    kBackTraceClass* obj = kBackTrace_Cast_(trace); 
    kSize totalSkip = skip + 1;   //always remove kBackTrace_Capture from trace

    if (!kIsNull(sobj->service))
    {
        kBackTrace_Lock(); 
        {
            obj->depth = CaptureStackBackTrace(0, (ULONG)kCountOf(obj->functions), obj->functions, kNULL); 
        }
        kBackTrace_Unlock(); 
    }

    obj->skip = kMin_(totalSkip, obj->depth); 

    return kOK; 
}

kFx(kStatus) kBackTrace_Describe(kBackTrace trace, kArrayList* lines, kAlloc allocator)
{
    kBackTraceStatic* sobj = kStaticOf(kBackTrace); 
    kBackTraceClass* obj = kBackTrace_Cast_(trace); 
    SYMBOL_INFOW* symbolInfo = kNULL; 
    IMAGEHLP_LINEW64* lineInfo = kNULL; 
    kArrayList output = kNULL; 

    kTry
    {
        const kSize NAME_CAPACITY = 256; 
        kSize symbolInfoLength = sizeof(SYMBOL_INFO) + (NAME_CAPACITY-1)*sizeof(WCHAR);    
        kChar temp1[kPATH_MAX]; 
        kChar temp2[kPATH_MAX]; 
        DWORD displacement; 
        BOOL symStatus; 
        kSize i; 

        kTest(kArrayList_Construct(&output, kTypeOf(kString), obj->depth-obj->skip, allocator)); 

        if (!kIsNull(sobj->service))
        {
            kTest(kObject_GetMem_(trace, symbolInfoLength, &symbolInfo)); 
            symbolInfo->MaxNameLen = (ULONG) NAME_CAPACITY; 
            symbolInfo->SizeOfStruct = sizeof(SYMBOL_INFO); 

            kTest(kObject_GetMem_(trace, sizeof(IMAGEHLP_LINEW64), &lineInfo));            
            lineInfo->SizeOfStruct = sizeof(IMAGEHLP_LINEW64);

            for (i = obj->skip; i < obj->depth; ++i)
            {
                DWORD64 function = (DWORD64)(obj->functions[i]);
                kString line = kNULL; 

                kTest(kString_Construct(&line, kNULL, allocator)); 
                kTest(kArrayList_Add(output, &line)); 

                kBackTrace_Lock(); 
                {
                    symStatus = SymFromAddrW(sobj->service, function, NULL, symbolInfo); 
                }
                kBackTrace_Unlock(); 

                if (symStatus)
                {
                    if (WideCharToMultiByte(CP_UTF8, 0, symbolInfo->Name, -1, temp1, (int)kCountOf(temp1), kNULL, kNULL) != 0)
                    {
                        kTest(kString_Setf(line, "%s", temp1)); 

                        kBackTrace_Lock(); 
                        {
                            symStatus = SymGetLineFromAddrW64(sobj->service, function, &displacement, lineInfo); 
                        }
                        kBackTrace_Unlock(); 

                        if (symStatus)
                        {
                            kTest(kPath_NativeWideToNormalizedWin(lineInfo->FileName, temp1, kCountOf(temp1))); 
                            kTest(kPath_FileName(temp1, temp2, kCountOf(temp2))); 

                            kTest(kString_Addf(line, " [%s, line %u]", temp2, lineInfo->LineNumber)); 
                        }
                    }
                }

                if (kString_Length_(line) == 0)
                {
                    kString_Set(line, "[Unknown]"); 
                }
            }
        }

        *lines = output; 
        output = kNULL; 
    }
    kFinally
    {
        kObject_Dispose(output); 
        kObject_FreeMem_(trace, symbolInfo); 
        kObject_FreeMem_(trace, lineInfo); 

        kEndFinally(); 
    }

    return kOK;
}

#elif defined(K_DEBUG) && defined(K_GCC) && !defined(K_VX_KERNEL)

#include <execinfo.h>

kFx(kStatus) kBackTrace_InitStatic()
{
    return kOK; 
}

kFx(kStatus) kBackTrace_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip) 
{
    kBackTraceClass* obj = kBackTrace_Cast_(trace); 
    kSize totalSkip = skip + 1;   //always remove kBackTrace_Capture from trace
   
    obj->depth = (kSize) backtrace(&obj->functions[0], kCountOf(obj->functions)); 
    obj->skip = kMin_(totalSkip, obj->depth); 
        
    return kOK; 
}

kFx(kStatus) kBackTrace_Describe(kBackTrace trace, kArrayList* lines, kAlloc allocator)
{
    kBackTraceClass* obj = kBackTrace_Cast_(trace); 
    kChar** symbols = kNULL; 
    kArrayList output = kNULL; 
    kString line = kNULL; 
    kSize i;

    kTry
    {
        kTest(kArrayList_Construct(&output, kTypeOf(kString), obj->depth - obj->skip, allocator)); 

        if (!kIsNull(symbols = backtrace_symbols(obj->functions, (int)obj->depth)))
        {
            for (i = obj->skip; i < obj->depth; ++i)
            {
                kTest(kString_Construct(&line, symbols[i], allocator)); 
                kTest(kArrayList_Add(output, &line)); 

                if (kString_Length_(line) == 0)
                {
                    kString_Set(line, "[Unknown]"); 
                }
            }

            *lines = output; 
            output = kNULL; 
        }
    }
    kFinally
    {
        kObject_Dispose(output); 
        free(symbols); 

        kEndFinally(); 
    }

    return kOK;
}

#else 

kFx(kStatus) kBackTrace_InitStatic()
{
    return kOK; 
}

kFx(kStatus) kBackTrace_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip)
{
    return kOK; 
}

kFx(kStatus) kBackTrace_Describe(kBackTrace trace, kArrayList* lines, kAlloc allocator)
{
    return kArrayList_Construct(lines, kTypeOf(kString), 0, allocator); 
}

#endif
