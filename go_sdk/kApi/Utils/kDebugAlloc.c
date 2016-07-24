/** 
 * @file    kDebugAlloc.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kDebugAlloc.h>
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kString.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Utils/kBackTrace.h>

kBeginValue(k, kDebugAllocation, kValue)
    kAddField(kDebugAllocation, kPointer, data)
    kAddField(kDebugAllocation, kSize, size)
    kAddField(kDebugAllocation, k64u, index)
kEndValue()

kBeginClass(k, kDebugAlloc, kAlloc)
    kAddVMethod(kDebugAlloc, kObject, VRelease)
    kAddVMethod(kDebugAlloc, kAlloc, VGet)
    kAddVMethod(kDebugAlloc, kAlloc, VFree)
kEndClass()

kFx(kStatus) kDebugAlloc_Construct(kDebugAlloc* object, const kChar* name, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kDebugAlloc), object)); 

    if (!kSuccess(status = kDebugAlloc_Init(*object, kTypeOf(kDebugAlloc), name, alloc)))
    {
        kAlloc_FreeRef(alloc, object); 
    }

    return status; 
} 

kFx(kStatus) kDebugAlloc_Init(kDebugAlloc object, kType type, const kChar* name, kAlloc alloc)
{
    kDebugAllocClass* obj = object; 
    kStatus exception = kOK; 

    kCheck(kAlloc_Init(object, type, alloc)); 

    kInitFields_(kDebugAlloc, object); 

    kTry
    {
        kTest(kStrCopy(obj->name, kCountOf(obj->name), name)); 

        kTest(kLock_Construct(&obj->lock, alloc));         
        kTest(kMap_Construct(&obj->history, kTypeOf(kPointer), kTypeOf(kDebugAllocation), 16, alloc)); 

        kTest(kMap_Construct(&obj->candidateObjectLeaks, kTypeOf(k64u), kTypeOf(kTypeName), 0, alloc)); 

        kTest(kBackTrace_Construct(&obj->backTrace, alloc)); 
        kTest(kMap_Construct(&obj->backTraceDescriptions, kTypeOf(kBackTrace), kTypeOf(kArrayList), 0, alloc)); 

        kTest(kArray1_Construct(&obj->padPattern, kTypeOf(kByte), K_DEBUG_ALLOC_PAD_SIZE, alloc)); 
        kTest(kDebugAlloc_InitPadPattern(object)); 
    }
    kCatch(&exception)
    {
        kDebugAlloc_VRelease(object); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kDebugAlloc_VRelease(kDebugAlloc object)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object);  

    kCheck(kObject_Destroy(obj->lock)); 
    kCheck(kObject_Destroy(obj->history)); 
    kCheck(kObject_Destroy(obj->candidateObjectLeaks)); 
    kCheck(kObject_Destroy(obj->backTrace)); 
    kCheck(kObject_Dispose(obj->backTraceDescriptions)); 
    kCheck(kObject_Destroy(obj->padPattern)); 

    kCheck(kAlloc_VRelease(object)); 
    
    return kOK; 
}

kFx(kStatus) kDebugAlloc_InitPadPattern(kDebugAlloc object)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kByte patternBuffer[] = { 19, 223, 61, 239, 149, 181, 37, 101 };        //miscellaneous prime numbers
    kByte* padData = kArray1_Data(obj->padPattern); 
    kSize padSize = kArray1_Count(obj->padPattern); 
    kSize i; 

    for (i = 0; i < padSize; ++i)
    {
        padData[i] = patternBuffer[i % kCountOf(patternBuffer)]; 
    }

    return kOK; 
}

kFx(kStatus) kDebugAlloc_Clear(kDebugAlloc object)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kSize padSize = kArray1_Count(obj->padPattern); 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        kMapItem it = kMap_First(obj->history); 

        while (!kIsNull(it))
        {
            const kDebugAllocation* allocation = kMap_Value(obj->history, it); 
            kByte* allocatedMem = allocation->data - padSize; 
            
            kTest(kObject_FreeMem(object, allocatedMem)); 
            obj->allocated -= allocation->size; 

            it = kMap_Next(obj->history, it); 
        }

        kTest(kMap_Clear(obj->history)); 

        obj->allocated = 0; 
        obj->counter = 0; 
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDebugAlloc_VGet(kDebugAlloc object, kSize size, void* mem)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kByte* allocatedMem = kNULL; 
    kSize padSize = kArray1_Count_(obj->padPattern); 
    const kByte* padData = kArray1_Data_(obj->padPattern); 
    kBackTrace backTrace = kNULL; 
    kArrayList backTraceDescription = kNULL; 
    kArrayList traceInfo = kNULL; 
    kDebugAllocation allocation; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {   
        kTest(kBackTrace_Capture(obj->backTrace, 1)); 

        if (!kSuccess(kMap_Find(obj->backTraceDescriptions, &obj->backTrace, &traceInfo)))
        {
            kTest(kObject_Clone(&backTrace, obj->backTrace, kObject_Alloc_(object))); 
            kTest(kBackTrace_Describe(backTrace, &backTraceDescription, kObject_Alloc_(object))); 

            kTest(kMap_Add(obj->backTraceDescriptions, &backTrace, &backTraceDescription)); 

            traceInfo = backTraceDescription; 
            backTrace = backTraceDescription = kNULL; 
        }

        kTest(kObject_GetMem(object, size + 2*padSize, &allocatedMem)); 

        allocation.data = allocatedMem + padSize; 
        allocation.size = size; 
        allocation.index = obj->counter;          
        allocation.trace = traceInfo; 

        kItemCopy_(allocatedMem, padData, padSize); 
        kItemCopy_(allocatedMem + padSize + size, padData, padSize); 

        kTest(kMap_Add(obj->history, &allocation.data, &allocation)); 
        
        allocatedMem = kNULL; 
        *(void**)mem = allocation.data;  

        obj->allocated += size; 
        obj->counter++; 

        if (!kIsNull(obj->allocListener.function))
        {
            obj->allocListener.function(obj->allocListener.receiver, object, &allocation); 
        }
    }
    kFinally
    {
        kObject_FreeMem(object, allocatedMem);

        kObject_Destroy(backTrace); 
        kObject_Dispose(backTraceDescription); 

        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDebugAlloc_VFree(kDebugAlloc object, void* mem)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kSize padSize = kArray1_Count_(obj->padPattern); 
    const kByte* padData = kArray1_Data_(obj->padPattern); 

    if (!kIsNull(mem))
    {
        kCheck(kLock_Enter(obj->lock)); 

        kTry
        {    
            kMapItem item = kNULL; 
            kDebugAllocation* allocation = kNULL; 
            kByte* allocatedMem = kNULL; 
            kSize size = 0; 

            //If the assertion below is encountered, then the caller has attempted to free
            //an address that is not currently an outstanding memory allocation (often, double-free).
            if(!kSuccess(kMap_FindItem(obj->history, &mem, &item)))
            {
                kAssert(kFALSE); 
                kThrow(kERROR_HEAP); 
            }

            allocation = kMap_Value(obj->history, item);  
            allocatedMem = allocation->data - padSize; 
            size = allocation->size; 

            //If the assertions below are encountered, then heap memory corruption has occurred. 
            //(Memory immediately before or immediately after the user allocation has been modified.)
            if (!kMemEquals(allocatedMem, padData, padSize))
            {
                kAssert(kFALSE); 
                kThrow(kERROR_HEAP); 
            }
            else if (!kMemEquals(allocatedMem + padSize + size, padData, padSize))
            {
                kAssert(kFALSE); 
                kThrow(kERROR_HEAP); 
            }

            kTest(kMap_RemoveItem(obj->history, item));   
            kTest(kObject_FreeMem(object, allocatedMem)); 

            obj->allocated -= size; 
        }
        kFinally
        {
            kLock_Exit(obj->lock); 
            kEndFinally(); 
        }
    }

    return kOK; 
}

kFx(k64u) kDebugAlloc_Checkpoint(kDebugAlloc object)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    k64u counter = 0; 

    kCheck(kLock_Enter(obj->lock)); 
    {
        counter = obj->counter; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return counter; 
}

kFx(kStatus) kDebugAlloc_Allocations(kDebugAlloc object, k64u since, kArrayList* history, kAlloc alloc)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kArrayList historyOut = kNULL; 
    kMapItem it = kNULL; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        kTest(kArrayList_Construct(&historyOut, kTypeOf(kDebugAllocation), 0, alloc)); 

        it = kMap_First(obj->history); 

        while (!kIsNull(it))
        {
            const kDebugAllocation* allocation = kMap_Value(obj->history, it); 
            
            if (allocation->index >= since)
            {
                kTest(kArrayList_Add(historyOut, allocation)); 
            }

            it = kMap_Next(obj->history, it); 
        }

        *history = historyOut; 
        historyOut = kNULL; 
    }
    kFinally
    {
        kCheck(kObject_Destroy(historyOut)); 
        kCheck(kLock_Exit(obj->lock)); 

        kEndFinally(); 
    }    

    return kOK; 
}

kFx(kStatus) kDebugAlloc_DetectLeakedObjects(kDebugAlloc object, k64u since)
{
    kArrayList assemblies = kNULL; 
    kSize i; 

    kTry
    {
        kTest(kArrayList_Construct(&assemblies, kTypeOf(kAssembly), 0, kAlloc_System())); 
        kTest(kAssembly_Enumerate(assemblies)); 

        for (i = 0; i < kArrayList_Count(assemblies); ++i)
        {
            kAssembly assembly = kArrayList_As_(assemblies, i, kObject); 

            kTest(kDebugAlloc_DetectLeakedAssemblyObjects(object, 0, assembly)); 
        }
    }
    kFinally
    {
        kDisposeRef(&assemblies); 

        kEndFinally(); 
    }

    return kOK; 
}


kFx(kStatus) kDebugAlloc_DetectLeakedAssemblyObjects(kDebugAlloc object, k64u since, kAssembly assembly)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kSize typeCount = kAssembly_TypeCount(assembly); 
    kMapItem it = kNULL; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        it = kMap_First(obj->history); 

        while (!kIsNull(it))
        {
            const kDebugAllocation* allocation = kMap_Value(obj->history, it); 
            
            if (allocation->index >= since)
            {
                kObject candidateObject = (kObject) allocation->data; 

                if ((allocation->size >= sizeof(kObjectClass)) && !kIsNull(kObject_Type_(candidateObject)) && kObject_VerifyTag_(candidateObject))
                {
                    kType candidateType = kObject_Type_(candidateObject); 
                    kSize i; 

                    //determine whether the candidate object header has a type field that matches a known 
                    //type in the given assembly (a hash set of types in the assembly would speed this up)
                    for (i = 0; i < typeCount; ++i)
                    {
                        kType type = kAssembly_TypeAt(assembly, i); 

                        if ((type == candidateType) && (allocation->size == kType_InnerSize_(type)))
                        {
                            const kChar* typeName = kType_Name(type); 

                            kTest(kMap_Replace(obj->candidateObjectLeaks, &allocation->index, typeName)); 
                        }
                    }
                }
            }

            it = kMap_Next(obj->history, it); 
        }       
    }
    kFinally
    {
        kCheck(kLock_Exit(obj->lock)); 

        kEndFinally(); 
    }    

    return kOK; 
}

kFx(kStatus) kDebugAlloc_LogAllocations(kDebugAlloc object, k64u since)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kArrayList allocations = kNULL; 
    kSize i = 0; 
    kSize j = 0; 

    kTry
    {
        kTest(kDebugAlloc_Allocations(object, since, &allocations, kObject_Alloc(object))); 

        for (i = 0; i < kArrayList_Count(allocations); ++i)
        {
            const kDebugAllocation* allocation = kArrayList_At(allocations, i); 
            const kSize DUMP_COUNT = 32; 
            const kSize DUMP_HEADER = 7; 
            const kSize DUMP_ITEM = 3; 
            kSize dumpCount = kMin_(DUMP_COUNT, allocation->size); 
            kText256 dumpMessage; 
            kTypeName typeName; 

            kLogf("%s leak - index: %llu, address: 0x%llX, size: %llu.", obj->name, (k64u)allocation->index, (k64u)(kSize)allocation->data, (k64u)allocation->size); 
        
            //dump header (must match header size constant above)
            kStrPrintf(dumpMessage, kCountOf(dumpMessage), "  Data:"); 

            //dump bytes (must match item size constant above)
            for (j = 0; j < dumpCount; ++j)
            {
                kStrPrintf(&dumpMessage[DUMP_HEADER + j*DUMP_ITEM], DUMP_ITEM+1, " %02X", allocation->data[j]); 
            }

            kLogf("%s ...", dumpMessage); 

            //print object type, if known
            if (kSuccess(kMap_Find(obj->candidateObjectLeaks, &allocation->index, &typeName)))
            {
                kLogf("  Type: %s", typeName); 
            }

            //print stack trace, if known
            if (!kIsNull(allocation->trace) && (kArrayList_Count(allocation->trace) > 0))
            {
                kLogf("  Trace:");  

                for (j = 0; j < kArrayList_Count_(allocation->trace); ++j)
                {
                    kString line = kArrayList_As_(allocation->trace, j, kString); 
                    
                    kLogf("    %s", kString_Chars_(line));  
                }
            }
  
            kLogf(""); 
        }       
    }
    kFinally
    {
        kCheck(kObject_Destroy(allocations)); 

        kEndFinally(); 
    }    

    return kOK; 
}

kFx(kStatus) kDebugAlloc_SetAllocListener(kDebugAlloc object, kCallbackFx function, kPointer receiver)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
   
    kCheck(kLock_Enter(obj->lock)); 
    {
        obj->allocListener.function = function; 
        obj->allocListener.receiver = receiver; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return kOK; 
}

kFx(kSize) kDebugAlloc_Allocated(kDebugAlloc object)
{
    kDebugAllocClass* obj = kDebugAlloc_Cast_(object); 
    kSize allocated = 0; 

    kCheck(kLock_Enter(obj->lock)); 
    {
        allocated = obj->allocated; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return allocated; 
}
