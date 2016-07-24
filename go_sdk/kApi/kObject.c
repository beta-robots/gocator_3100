/** 
 * @file    kObject.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kObject.h>
#include <kApi/Utils/kObjectPool.h>

kBeginVirtualClass(k, kObject, kNull)

    kAddFlags(kObject, kTYPE_FLAGS_ABSTRACT)

    //serialization versions
    kAddVersion(kObject, "kdat5", "5.0.0.0", "22-0", Serialize, Deserialize)
    kAddVersion(kObject, "kdat6", "5.7.1.0", "kObject-0", Serialize, Deserialize)

    //virtual methods
    kAddVMethod(kObject, kObject, VRelease)
    kAddVMethod(kObject, kObject, VDisposeItems)
    kAddVMethod(kObject, kObject, VInitClone)
    kAddVMethod(kObject, kObject, VHashCode)
    kAddVMethod(kObject, kObject, VEquals)
    kAddVMethod(kObject, kObject, VSize)

kEndVirtualClass()

kFx(kStatus) kObject_Clone(kObject* object, kObject source, kAlloc allocator)
{
    if (kIsNull(source))
    {
        *object = kNULL; 
        return kOK; 
    }
    else 
    {
        kObjectClass* srcObj = kObject_Cast_(source); 
        kAlloc alloc = kAlloc_Fallback_(allocator);
        kType type = srcObj->type; 
        kStatus status; 

        kCheckArgs(kType_Overrides_(type, kObject, VInitClone)); 

        kCheck(kAlloc_GetObject_(alloc, type, object)); 

        if (!kSuccess(status = kObject_VTable_(source)->VInitClone(*object, source, alloc)))
        {
            kAlloc_FreeRef(alloc, object); 
        }

        return status; 
    }
}

kFx(kStatus) kObject_Destroy(kObject object)
{
    if (!kIsNull(object))
    {
        kObjectClass* obj = kObject_Cast_(object); 

        if (kAtomic32s_Decrement_(&obj->refCount) == 0)
        {
            if (!kIsNull(obj->pool))
            {
                obj->refCount = 1; 
                
                kCheck(kObjectPool_Reclaim_(obj->pool, object));  
            }
            else
            {
                kAlloc alloc = obj->alloc; 

                kCheck(kObject_Release_(object)); 

                if (!kIsNull(alloc))
                {
                    kCheck(kAlloc_Free_(alloc, obj)); 
                }
                else
                {
                    kCheck(kSysMemFree(obj)); 
                }
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kObject_Dispose(kObject object)
{
    if (!kIsNull(object))
    {
        kObjectClass* obj = kObject_Cast_(object); 

        if (kAtomic32s_Decrement_(&obj->refCount) == 0)
        {
            if (!kIsNull(obj->pool))
            {
                obj->refCount = 1; 
                
                kCheck(kObjectPool_Reclaim_(obj->pool, object));  
            }
            else
            {
                kAlloc alloc = obj->alloc; 

                kCheck(kObject_DisposeItems_(object)); 
                kCheck(kObject_Release_(object)); 

                if (!kIsNull(alloc))
                {
                    kCheck(kAlloc_Free_(alloc, obj)); 
                }
                else
                {
                    kCheck(kSysMemFree(obj)); 
                }
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kObject_Init(kObject object, kType type, kAlloc allocator)
{
    return kObject_Init_(object, type, allocator); 
}

kFx(kStatus) kObject_Release(kObject object)
{  
    kAssertType(object, kObject); 

    return kObject_Release_(object); 
}

kFx(kStatus) kObject_VRelease(kObject object)
{
    return kObject_VRelease_(object); 
}

kFx(kStatus) kObject_DisposeItems(kObject object)
{
    kAssertType(object, kObject); 

    return kObject_DisposeItems_(object); 
}

kFx(kStatus) kObject_VDisposeItems(kObject object)
{
    kAssertType(object, kObject); 

    return kObject_VDisposeItems_(object); 
}

kFx(kStatus) kObject_InitClone(kObject object, kObject source, kAlloc allocator)
{
    kCheckArgs(kType_Overrides_(kObject_Type_(source), kObject, VInitClone)); 

    return kObject_VTable_(object)->VInitClone(object, source, allocator); 
}

kFx(kStatus) kObject_VInitClone(kObject object, kObject source, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kObject_Share(kObject object)
{
    kAssertType(object, kObject); 

    return kObject_Share_(object); 
}

kFx(kStatus) kObject_SetPool(kObject object, kObjectPool pool)
{
    kAssertType(object, kObject); 

    return kObject_SetPool_(object, pool); 
}

kFx(kSize) kObject_HashCode(kObject object)
{
    kAssertType(object, kObject); 

    return kObject_HashCode_(object); 
}

kFx(kSize) kObject_VHashCode(kObject object)
{
    kAssertType(object, kObject); 

    return kHashPointer_(object); 
}

kFx(kBool) kObject_Equals(kObject object, kObject other)
{
    kAssertType(object, kObject); 
    kAssertType(other, kObject); 

    return kObject_Equals_(object, other); 
}

kFx(kBool) kObject_VEquals(kObject object, kObject other)
{
    kAssertType(object, kObject); 
    kAssertType(other, kObject); 

    return (object == other); 
}

kFx(kSize) kObject_Size(kObject object)
{
    kAssertType(object, kObject); 

    return kObject_Size_(object); 
}

kFx(kSize) kObject_VSize(kObject object)
{
    kAssertType(object, kObject); 

    return kType_InnerSize_(kObject_Type_(object)); 
}

kFx(kBool) kObject_IsShared(kObject object)
{
    kObjectClass* obj = kObject_Cast_(object); 
    k32s refCount = kAtomic32s_Get_(&obj->refCount);

    return (refCount > 1); 
}

kFx(kStatus) kObject_Serialize(kObject object, kSerializer serializer)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kObject_Deserialize(kObject object, kSerializer serializer, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kObject_GetMem(kObject object, kSize size, void* mem)
{
    kAssertType(object, kObject); 

    return kObject_GetMem_(object, size, mem); 
}

kFx(kStatus) kObject_GetMemZero(kObject object, kSize size, void* mem)
{
    kAssertType(object, kObject); 

    return kObject_GetMemZero_(object, size, mem); 
}

kFx(kStatus) kObject_FreeMem(kObject object, void* mem)
{
    kAssertType(object, kObject); 

    return kObject_FreeMem_(object, mem); 
}

kFx(kStatus) kObject_FreeMemRef(kObject object, void* mem)
{
    kAssertType(object, kObject); 

    return kObject_FreeMemRef_(object, mem); 
}

kFx(kType) kObject_Type(kObject object)
{   
    kAssertType(object, kObject); 

    return kObject_Type_(object);
}

kFx(kBool) kObject_Is(kObject object, kType type)
{
    return kObject_Is_(object, type); 
}

kFx(kAlloc) kObject_Alloc(kObject object)
{
    kAssertType(object, kObject); 

    return kObject_Alloc_(object); 
}
