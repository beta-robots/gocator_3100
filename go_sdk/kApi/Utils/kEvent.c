/** 
 * @file    kEvent.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kEvent.h>
#include <kApi/Data/kList.h>

kBeginClass(k, kEvent, kObject)
    kAddVMethod(kEvent, kObject, VRelease)
    kAddVMethod(kEvent, kObject, VInitClone)
kEndClass()

kFx(kStatus) kEvent_Construct(kEvent* evnt, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kEvent), evnt)); 

    if (!kSuccess(status = kEvent_Init(*evnt, kTypeOf(kEvent), alloc)))
    {
        kAlloc_FreeRef(alloc, evnt); 
    }

    return status; 
}

kFx(kStatus) kEvent_Init(kEvent evnt, kType type, kAlloc allocator)
{
    kEventClass* obj = evnt; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(evnt, type, allocator)); 

    kCheck(kInitFields_(kEvent, evnt)); 

    kTry
    {
        kTest(kList_Construct(&obj->listeners, kTypeOf(kCallback), 0, allocator)); 
    }
    kCatch(&status); 
    {
        kEvent_VRelease(evnt); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kEvent_VInitClone(kEvent evnt, kEvent source, kAlloc allocator)
{
    kEventClass* obj = evnt; 
    kEventClass* srcObj = source; 
    kStatus status = kOK; 

    kCheck(kEvent_Init(evnt, kObject_Type_(source), allocator)); 

    kTry
    {
        kTest(kList_Assign(obj->listeners, srcObj->listeners)); 
    }
    kCatch(&status); 
    {
        kEvent_VRelease(evnt); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kEvent_VRelease(kEvent evnt)
{
    kEventClass* obj = kEvent_Cast_(evnt); 

    kCheck(kObject_Destroy(obj->listeners)); 

    kCheck(kObject_VRelease_(evnt)); 

    return kOK; 
}

kFx(kStatus) kEvent_Add(kEvent evnt, kCallbackFx function, kPointer receiver)
{
    kEventClass* obj = kEvent_Cast_(evnt); 
    kCallback entry; 

    entry.function = function; 
    entry.receiver = receiver; 

    kCheck(kList_Add(obj->listeners, &entry, kNULL)); 

    return kOK; 
}

kFx(kStatus) kEvent_Remove(kEvent evnt, kCallbackFx function, kPointer receiver)
{
    kEventClass* obj = kEvent_Cast_(evnt); 
    kListItem it = kList_First(obj->listeners); 

    while (!kIsNull(it))
    {
        kCallback* entry = kList_At(obj->listeners, it); 
        
        if ((entry->function == function) && (entry->receiver == receiver))
        {
            //adjust notification iterator, if necessary
            if (it == obj->notifyIt)
            {
                obj->notifyIt = kList_Next(obj->listeners, obj->notifyIt); 
            }

            return kList_Remove(obj->listeners, it); 
        }

        it = kList_Next(obj->listeners, it); 
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kEvent_Clear(kEvent evnt)
{
    kEventClass* obj = kEvent_Cast_(evnt); 

    kCheck(kList_Clear(obj->listeners)); 
    obj->notifyIt = kNULL; 

    return kOK; 
}

kFx(kStatus) kEvent_Notify(kEvent evnt, kPointer sender, void* args)
{
    kEventClass* obj = kEvent_Cast_(evnt); 
    kStatus status = kOK; 
    kStatus callStatus; 

    //break notification cycles
    if (!kIsNull(obj->notifyIt))
    {
        return kOK; 
    }

    //share the event, to prevent self-destruction during notification callbacks
    kCheck(kObject_Share_(evnt)); 

    obj->notifyIt = kList_First(obj->listeners); 

    while (!kIsNull(obj->notifyIt))
    {
        kCallback entry = kList_As_(obj->listeners, obj->notifyIt, kCallback); 

        obj->notifyIt = kList_Next(obj->listeners, obj->notifyIt); 

        callStatus = entry.function(entry.receiver, sender, args); 
        status = kSuccess(status) ? callStatus : status; 
    }

    obj->notifyIt = kNULL; 
    kObject_Destroy(evnt); 

    return status; 
}

kFx(kSize) kEvent_Count(kEvent evnt)
{
    kEventClass* obj = kEvent_Cast_(evnt); 

    return kList_Count_(obj->listeners); 
}

kFx(kList) kEvent_Listeners(kEvent evnt)
{
    kEventClass* obj = kEvent_Cast_(evnt); 

    return obj->listeners; 
}

