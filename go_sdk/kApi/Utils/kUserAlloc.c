/** 
 * @file    kUserAlloc.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/kApiLib.h>

kBeginClass(k, kUserAlloc, kAlloc)
    kAddVMethod(kUserAlloc, kObject, VRelease)
    kAddVMethod(kUserAlloc, kAlloc, VGet)
    kAddVMethod(kUserAlloc, kAlloc, VFree)
kEndClass()

kFx(kStatus) kUserAlloc_Construct(kUserAlloc* object, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc allocator)
{
    kUserAlloc output = kNULL; 
    kType type = kTypeOf(kUserAlloc); 
    kStatus status; 
    
    if (kIsNull(allocator))
    {
        //this type can be used before the type system is fully initialized
        kCheck(kSysMemAlloc(sizeof(kUserAllocClass), &output)); 
    }
    else
    {
        kCheck(kAlloc_GetObject(allocator, kTypeOf(kUserAlloc), &output)); 
    }

    if (!kSuccess(status = kUserAlloc_Init(output, type, allocFx, freeFx, provider, allocator)))
    {
        kSysMemFree(output); 
        return status; 
    }

    *object = output;

    return kOK; 
} 

kFx(kStatus) kUserAlloc_Init(kUserAlloc object, kType type, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc alloc)
{
    kUserAllocClass* obj = object; 

    kCheck(kAlloc_Init(object, type, alloc)); 

    obj->allocFx = allocFx; 
    obj->freeFx = freeFx; 
    obj->provider = provider; 

    return kOK; 
}

kFx(kStatus) kUserAlloc_VRelease(kUserAlloc object)
{
    kCheck(kAlloc_VRelease(object)); 
    
    return kOK; 
}

kFx(kStatus) kUserAlloc_VGet(kUserAlloc object, kSize size, void* mem)
{
    kUserAllocClass* obj = kUserAlloc_Cast_(object); 

    return obj->allocFx(obj->provider, size, mem); 
}

kFx(kStatus) kUserAlloc_VFree(kUserAlloc object, void* mem)
{
    kUserAllocClass* obj = kUserAlloc_Cast_(object); 

    return obj->freeFx(obj->provider, mem); 
}
