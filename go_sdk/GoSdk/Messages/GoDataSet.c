/** 
 * @file    GoDataSet.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoDataSet.h>

kBeginClass(Go, GoDataSet, kObject)

    //serialization versions
    kAddVersion(GoDataSet, "kdat6", "4.0.0.0", "GoDataSet-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(GoDataSet, kObject, VRelease)
    kAddVMethod(GoDataSet, kObject, VInitClone)
    kAddVMethod(GoDataSet, kObject, VSize)

kEndClass()

GoFx(kStatus) GoDataSet_Construct(GoDataSet* set, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(GoDataSet), set)); 

    if (!kSuccess(status = GoDataSet_Init(*set, kTypeOf(GoDataSet), alloc)))
    {
        kAlloc_FreeRef(alloc, set); 
    }

    return status; 
} 

GoFx(kStatus) GoDataSet_Init(GoDataSet set, kType type, kAlloc alloc)
{
    kCheck(kObject_Init_(set, type, alloc)); 

    kZeroDerivedFields_(GoDataSet, set); 

    return kOK; 
}

GoFx(kStatus) GoDataSet_VInitClone(GoDataSet set, GoDataSet source, kAlloc alloc)
{
    GoDataSetClass* obj = set; 
    GoDataSetClass* srcObj = source; 
    kStatus status; 

    kCheck(GoDataSet_Init(set, kObject_Type_(source), alloc)); 

    kTry
    {
        obj->senderId = srcObj->senderId; 

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc)); 
    }
    kCatch(&status)
    {
        GoDataSet_VRelease(set); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoDataSet_VRelease(GoDataSet set)
{
    GoDataSetClass* obj = set; 

    kCheck(kObject_Dispose(obj->content)); 

    kCheck(kObject_VRelease(set));     

    return kOK; 
}

GoFx(kStatus) GoDataSet_Allocate(GoDataSet set, kSize minimumCapacity)
{
    GoDataSetClass* obj = set; 

    kCheck(kDisposeRef(&obj->content)); 
    kCheck(kArrayList_Construct(&obj->content, kTypeOf(kObject), minimumCapacity, kObject_Alloc_(set))); 

    return kOK; 
}

GoFx(kStatus) GoDataSet_Add(GoDataSet set, kObject item)
{
    GoDataSetClass* obj = set; 
    
    if (kIsNull(obj->content))
    {
        kCheck(GoDataSet_Allocate(set, 1)); 
    }

    kCheck(kArrayList_Add(obj->content, &item)); 

    return kOK; 
}

GoFx(kSize) GoDataSet_VSize(GoDataSet set)
{
    GoDataSetClass* obj = set; 
    return sizeof(GoDataSetClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoDataSet_WriteDat6V0(GoDataSet set, kSerializer serializer)
{
    GoDataSetClass* obj = set; 

    kCheck(kSerializer_BeginWrite_(serializer, kTypeOf(k16u), kFALSE)); 
    kCheck(kSerializer_Write32u_(serializer, obj->senderId));
    kCheck(kSerializer_EndWrite_(serializer)); 

    kCheck(kSerializer_WriteObject_(serializer, obj->content)); 

    return kOK; 
}

GoFx(kStatus) GoDataSet_ReadDat6V0(GoDataSet set, kSerializer serializer, kAlloc alloc)
{
    GoDataSetClass* obj = set; 
    kStatus status;
    
    kCheck(GoDataSet_Init(set, kTypeOf(GoDataSet), alloc)); 

    kTry
    {
        kTest(kSerializer_BeginRead_(serializer, kTypeOf(k16u), kFALSE)); 
        kTest(kSerializer_Read32u_(serializer, &obj->senderId));
        kTest(kSerializer_EndRead_(serializer)); 
            
        kTest(kSerializer_ReadObject_(serializer, &obj->content, alloc)); 
    }
    kCatch(&status)
    {
        GoDataSet_VRelease(set); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(k32u) GoDataSet_SenderId(GoDataSet set)
{
    GoDataSetClass* obj = set; 
    return obj->senderId; 
}

GoFx(kSize) GoDataSet_Count(GoDataSet set)
{
    GoDataSetClass* obj = set; 
    return kArrayList_Count_(obj->content); 
}

GoFx(kObject) GoDataSet_At(GoDataSet set, kSize index)
{
    GoDataSetClass* obj = set; 

    kAssert(index < kArrayList_Count(obj->content));

    return kArrayList_As_(obj->content, index, kObject); 
}
