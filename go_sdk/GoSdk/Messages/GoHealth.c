/** 
 * @file    GoHealthMsg.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoHealth.h>

/* 
 * GoIndicator
 */

kBeginValue(Go, GoIndicator, kValue)
    kAddField(GoIndicator, k32u, id)
    kAddField(GoIndicator, k32u, instance)
    kAddField(GoIndicator, k64s, value)
kEndValue()


/* 
 * GoHealthMsg
 */

kBeginClass(Go, GoHealthMsg, kObject) 
    
    //serialization versions
    kAddVersion(GoHealthMsg, "godat", "6.0.0.0", "0", WriteV0, ReadV0)
    kAddVersion(GoHealthMsg, "kdat6", "4.0.0.0", "GoHealthMsg-0", WriteV0, ReadV0)

    //virtual methods
    kAddVMethod(GoHealthMsg, kObject, VInitClone)
    kAddVMethod(GoHealthMsg, kObject, VRelease)
    kAddVMethod(GoHealthMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoHealthMsg_Construct(GoHealthMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoHealthMsg), msg)); 

    if (!kSuccess(status = GoHealthMsg_Init(*msg, kTypeOf(GoHealthMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoHealthMsg_Init(GoHealthMsg msg, kType type, kAlloc alloc)
{
    GoHealthMsgClass* obj = msg; 

    kCheck(kObject_Init_(msg, type, alloc)); 
    kInitFields_(GoHealthMsg, msg);

    return kOK; 
}

GoFx(kStatus) GoHealthMsg_VInitClone(GoHealthMsg msg, GoHealthMsg source, kAlloc alloc)
{
    GoHealthMsgClass* obj = msg; 
    GoHealthMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoHealthMsg_Init(msg, kObject_Type_(source), alloc));     

    kTry
    {
        obj->source = srcObj->source;
        
        kTest(kObject_Clone(&obj->indicators, srcObj->indicators, alloc)); 
    }
    kCatch(&status)
    {
        GoHealthMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoHealthMsg_Allocate(GoHealthMsg msg, kSize count)
{
    GoHealthMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->indicators)); 
    kCheck(kArray1_Construct(&obj->indicators, kTypeOf(GoIndicator), count, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kStatus) GoHealthMsg_VRelease(GoHealthMsg msg)
{
    GoHealthMsgClass* obj = msg; 
    
    kCheck(kObject_Destroy(obj->indicators)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoHealthMsg_VSize(GoHealthMsg msg)
{
    GoHealthMsgClass* obj = msg; 

    return sizeof(GoHealthMsgClass) + (kIsNull(obj->indicators) ? 0 : kObject_Size_(obj->indicators)); 
}

GoFx(kStatus) GoHealthMsg_WriteV0(GoHealthMsg msg, kSerializer serializer)
{
    GoHealthMsgClass* obj = msg; 
    k32u count = (k32u) kArray1_Count_(obj->indicators); 
    k32u i; 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source));  
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 

    for (i = 0; i < count; ++i)
    {
        const GoIndicator* indicator = kArray1_At_(obj->indicators, i); 

        kCheck(kSerializer_Write32u_(serializer, indicator->id)); 
        kCheck(kSerializer_Write32u_(serializer, indicator->instance)); 
        kCheck(kSerializer_Write64s_(serializer, indicator->value)); 
    }

    return kOK; 
}

GoFx(kStatus) GoHealthMsg_ReadV0(GoHealthMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoHealthMsgClass* obj = msg; 
    kStatus status;
    k8u source = 0; 
    k8u reserved; 
    k32u count = 0; 
    k32u i; 

    kCheck(GoHealthMsg_Init(msg, kTypeOf(GoHealthMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
            
        obj->source = (GoDataSource)source; 

        kTest(kArray1_Construct(&obj->indicators, kTypeOf(GoIndicator), count, alloc)); 

        for (i = 0; i < count; ++i)
        {
            const GoIndicator* indicator = kArray1_At_(obj->indicators, i); 

            kTest(kSerializer_Read32u_(serializer, &indicator->id)); 
            kTest(kSerializer_Read32u_(serializer, &indicator->instance)); 
            kTest(kSerializer_Read64s_(serializer, &indicator->value)); 
        } 
    }
    kCatch(&status)
    {
        GoHealthMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoHealthMsg_Source(GoHealthMsg msg)
{
    return GoHealthMsg_(msg)->source; 
}

GoFx(kSize) GoHealthMsg_Count(GoHealthMsg msg)
{
    return kArray1_Count_(GoHealthMsg_Content_(msg)); 
}

GoFx(GoIndicator*) GoHealthMsg_At(GoHealthMsg msg, kSize index)
{
    kAssert(index < GoHealthMsg_Count(msg));

    return (GoIndicator*)kArray1_At(GoHealthMsg_Content_(msg), index); 
}

GoFx(GoIndicator*) GoHealthMsg_Find(GoHealthMsg msg, k32u id, k32u instance)
{
    GoIndicator* indicator;
    kSize i;

    for (i = 0; i < GoHealthMsg_Count(msg); i++)
    {
        indicator = GoHealthMsg_At(msg, i);

        if (indicator->id == id && indicator->instance == instance)
        {
            return indicator;
        }
    }

    return kNULL;
}
