/** 
 * @file    kQueue.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kQueue.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kQueue, kObject) 
    
    //serialization versions
    kAddVersion(kQueue, "kdat5", "5.0.0.0", "42-1", WriteDat5V1, ReadDat5V1)
    kAddVersion(kQueue, "kdat6", "5.7.1.0", "kQueue-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kQueue, kObject, VRelease)
    kAddVMethod(kQueue, kObject, VDisposeItems)
    kAddVMethod(kQueue, kObject, VInitClone)
    kAddVMethod(kQueue, kObject, VSize)

    //collection interface 
    kAddInterface(kQueue, kCollection)
    kAddIVMethod(kQueue, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kQueue, kCollection, VItemType, ItemType)
    kAddIVMethod(kQueue, kCollection, VCount, Count)
    kAddIVMethod(kQueue, kCollection, VHasNext, HasNext)
    kAddIVMethod(kQueue, kCollection, VNext, Next)

kEndClass() 

kFx(kStatus) kQueue_Construct(kQueue* queue, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kQueue), queue)); 

    if (!kSuccess(status = kQueue_Init(*queue, kTypeOf(kQueue), itemType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, queue); 
    }

    return status; 
} 

kFx(kStatus) kQueue_Init(kQueue queue, kType classType, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kQueueClass* obj = queue; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(queue, classType, allocator)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->count = 0; 
    obj->head = 0; 
    obj->capacity = 0; 
    obj->divisor = 1; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kQueue_Reserve(queue, initialCapacity)); 
    }
    kCatch(&status)
    {
        kQueue_VRelease(queue); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kQueue_VInitClone(kQueue queue, kQueue source, kAlloc allocator)
{
    kQueueClass* obj = queue; 
    kQueueClass* srcObj = source; 
    kSize headCount = kQueue_HeadCount_(source); 
    kSize tailCount = kQueue_TailCount_(source); 
    kStatus status = kOK; 

    kCheck(kObject_Init_(queue, kObject_Type_(source), allocator));     
    
    obj->itemType = srcObj->itemType; 
    obj->itemSize = srcObj->itemSize; 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->count = 0; 
    obj->head = 0; 
    obj->capacity = 0; 
    obj->divisor = 1; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kQueue_Resize(queue, srcObj->count)); 

        kTest(kCloneContent(obj->itemType, kQueue_At_(queue, 0), kQueue_At_(source, 0), headCount, allocator)); 
        kTest(kCloneContent(obj->itemType, kQueue_At_(queue, headCount), kQueue_At_(source, headCount), tailCount, allocator)); 
    }
    kCatch(&status)
    {
        kQueue_VDisposeItems(queue); 
        kQueue_VRelease(queue); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kQueue_VRelease(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    kCheck(kObject_FreeMem_(queue, obj->items)); 

    kCheck(kObject_VRelease_(queue)); 

    return kOK; 
}

kFx(kStatus) kQueue_VDisposeItems(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize headCount = kQueue_HeadCount_(queue); 
    kSize tailCount = kQueue_TailCount_(queue); 

    kCheck(kDisposeContent(obj->itemType, kQueue_At_(queue, 0), headCount)); 
    kCheck(kDisposeContent(obj->itemType, kQueue_At_(queue, headCount), tailCount)); 

    return kOK; 
}

kFx(kSize) kQueue_VSize(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kQueue_DataSize_(queue); 
    kSize size = sizeof(kQueueClass) + dataSize; 
    kSize headCount = kQueue_HeadCount_(queue); 
    kSize tailCount = kQueue_TailCount_(queue); 

    size += kSizeContent(obj->itemType, kQueue_At_(queue, 0), headCount); 
    size += kSizeContent(obj->itemType, kQueue_At_(queue, headCount), tailCount); 

    return size; 
}

kFx(kStatus) kQueue_WriteDat5V1(kQueue queue, kSerializer serializer)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize headCount = kQueue_HeadCount_(queue); 
    kSize tailCount = kQueue_TailCount_(queue); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize_(serializer, obj->count)); 
    kCheck(kSerializer_WriteSize_(serializer, obj->head)); 
    kCheck(kSerializer_WriteSize_(serializer, obj->capacity)); 
    kCheck(kSerializer_Write32s_(serializer, kTRUE));  //isDynamic

    kCheck(kSerializer_WriteSize_(serializer, headCount)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_At_(queue, 0), headCount)); 

    kCheck(kSerializer_WriteSize_(serializer, tailCount)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_At_(queue, headCount), tailCount)); 

    return kOK; 
}

kFx(kStatus) kQueue_ReadDat5V1(kQueue queue, kSerializer serializer, kAlloc allocator)
{
    kSize count = 0, head = 0, headCount = 0, tailCount = 0, capacity = 0; 
    kTypeVersion itemVersion;
    k32s isDynamic; 
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &count));
    kCheck(kSerializer_ReadSize_(serializer, &head));
    kCheck(kSerializer_ReadSize_(serializer, &capacity));      
    kCheck(kSerializer_Read32s_(serializer, &isDynamic));

    kCheck(kSerializer_ReadSize_(serializer, &headCount));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kQueue_Init(queue, kTypeOf(kQueue), itemType, count, allocator)); 

    kTry
    {
        kTest(kQueue_Resize(queue, count)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, kQueue_At_(queue, 0), headCount)); 

        kTest(kSerializer_ReadSize_(serializer, &tailCount));
        kTest(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, kQueue_At_(queue, headCount), tailCount)); 
    }
    kCatch(&status)
    {
        kQueue_VDisposeItems(queue); 
        kQueue_VRelease(queue); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kQueue_WriteDat6V0(kQueue queue, kSerializer serializer)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize headCount = kQueue_HeadCount_(queue); 
    kSize tailCount = kQueue_TailCount_(queue); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->count)); 

    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_At_(queue, 0), headCount)); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_At_(queue, headCount), tailCount)); 

    return kOK; 
}

kFx(kStatus) kQueue_ReadDat6V0(kQueue queue, kSerializer serializer, kAlloc allocator)
{
    kQueueClass *obj = queue; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize count = 0; 
    kStatus status; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &count)); 

    kCheck(kQueue_Init(queue, kTypeOf(kQueue), itemType, count, allocator)); 

    kTry
    {
        kTest(kQueue_Resize(queue, count)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, count)); 
    }
    kCatch(&status)
    {
        kQueue_VDisposeItems(queue); 
        kQueue_VRelease(queue); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kQueue_Allocate(kQueue queue, kType itemType, kSize initialCapacity)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->count = 0; 
    obj->head = 0; 
    obj->capacity = 0; 
    obj->divisor = 1; 

    kCheck(kQueue_Reserve(queue, initialCapacity)); 

    return kOK; 
}

kFx(kStatus) kQueue_Assign(kQueue queue, kQueue source)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kQueueClass* srcObj = kQueue_Cast_(source); 
    kSize headCount = kQueue_HeadCount_(source); 
    kSize tailCount = kQueue_TailCount_(source); 

    kCheck(kQueue_Allocate(queue, srcObj->itemType, srcObj->count)); 

    kCheck(kQueue_Resize(queue, srcObj->count)); 

    kCheck(kCopyContent(obj->itemType, kQueue_At_(queue, 0), kQueue_At_(source, 0), headCount)); 
    kCheck(kCopyContent(obj->itemType, kQueue_At_(queue, headCount), kQueue_At_(source, headCount), tailCount)); 

    return kOK;   
}

kFx(kStatus) kQueue_Clear(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    obj->count = 0; 
    obj->head = 0; 

    return kOK; 
}

kFx(kStatus) kQueue_Purge(kQueue queue)
{
    kCheck(kObject_DisposeItems_(queue)); 
    kCheck(kQueue_Clear(queue)); 

    return kOK; 
}

kFx(kStatus) kQueue_Zero(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize headCount = kQueue_HeadCount_(queue); 
    kSize tailCount = kQueue_TailCount_(queue); 

    kCheck(kZeroContent(obj->itemType, kQueue_At_(queue, 0), headCount));
    kCheck(kZeroContent(obj->itemType, kQueue_At_(queue, headCount), tailCount));

    return kOK; 
}

kFx(kStatus) kQueue_Reserve(kQueue queue, kSize minimumCapacity)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    if (obj->capacity < minimumCapacity)
    {
        kSize grownCapacity = kMax_(minimumCapacity, kQUEUE_GROWTH_FACTOR*obj->capacity);
        kSize newCapacity = kMax_(grownCapacity, kQUEUE_MIN_CAPACITY); 
        kSize newSize = kMax_(obj->allocSize, newCapacity*obj->itemSize); 

        if (newSize > obj->allocSize)
        {
            kByte* oldItems = obj->items; 
            kByte* newItems = kNULL; 

            kCheck(kObject_GetMem_(queue, newSize, &newItems)); 

            if (obj->count > 0)
            {
                kSize headCount = kQueue_HeadCount_(queue); 
                kSize tailCount = kQueue_TailCount_(queue); 

                kMemCopy(kItemAt_(newItems, 0, obj->itemSize), kQueue_At_(queue, 0), headCount*obj->itemSize); 
                kMemCopy(kItemAt_(newItems, headCount, obj->itemSize), kQueue_At_(queue, headCount), tailCount*obj->itemSize); 
            }

            obj->items = newItems; 
            obj->allocSize = newSize; 
            obj->head = 0; 

            kCheck(kObject_FreeMem_(queue, oldItems)); 
        }

        obj->capacity = newCapacity;
        obj->divisor = kMax_(1, obj->capacity); 
    }
    
    return kOK; 
}

kFx(kStatus) kQueue_Resize(kQueue queue, kSize count)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize i; 

    if (count > obj->capacity)
    {
        kCheck(kQueue_Reserve(queue, count)); 
    }

    if (kType_IsReference_(obj->itemType) && (count > obj->count))
    {
        for (i = obj->count; i < count; ++i)
        {
            kItemZero_(kQueue_At_(queue, i), obj->itemSize); 
        }
    }

    obj->count = count; 
    
    return kOK; 
}

kFx(kStatus) kQueue_Add(kQueue queue, const void* item)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    if (obj->count == obj->capacity)
    {
        kCheck(kQueue_Reserve(queue, obj->capacity + 1)); 
    }

    kItemImport_(kQueue_At_(queue, obj->count), item, obj->itemType); 

    obj->count++; 

    return kOK; 
}

kFx(kStatus) kQueue_SetItem(kQueue queue, kSize index, const void* item)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    kCheckArgs(index < obj->count); 

    kItemImport_(kQueue_At_(queue, index), item, obj->itemType); 

    return kOK; 
}

kFx(kStatus) kQueue_Item(kQueue queue, kSize index, void* item)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    kCheckArgs(index < obj->count); 

    kItemCopy_(item, kQueue_At_(queue, index), obj->itemSize); 

    return kOK; 
}

kFx(kStatus) kQueue_Remove(kQueue queue, void* item)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    kCheckArgs(obj->count > 0); 

    if (!kIsNull(item))
    {
        kItemCopy_(item, kQueue_At_(queue, 0), obj->itemSize); 
    }

    obj->count--; 
    obj->head = (obj->head + 1) % obj->capacity; 

    return kOK; 
}

kFx(kStatus) kQueue_AddCount(kQueue queue, kSize count)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    if ((obj->count + count) > obj->capacity)
    {
        kCheck(kQueue_Reserve(queue, obj->count + count)); 
    }

    obj->count += count; 

    return kOK; 
}

kFx(kStatus) kQueue_RemoveCount(kQueue queue, kSize count)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    if (count > obj->count)
    {
        return kERROR_STATE; 
    }

    obj->count -= count; 
    obj->head = (obj->head + count) % obj->capacity; 

    return kOK; 
}

kFx(kIterator) kQueue_GetIterator(kQueue queue)
{
    return (kSize) 0; 
}

kFx(kBool) kQueue_HasNext(kQueue queue, kIterator iterator)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize it = (kSize) iterator; 
    kSize end = (kSize) obj->count; 
    
    return (it != end); 
}

kFx(void*) kQueue_Next(kQueue queue, kIterator* iterator)
{
    kQueueClass* obj = kQueue_Cast_(queue); 
    kSize it = (kSize) iterator; 
    void* next = kQueue_At_(obj, it); 
       
    *iterator = (kPointer)(kSize)(it + 1); 

    return next; 
}

kFx(void*) kQueue_At(kQueue queue, kSize index)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    return kQueue_At_(obj, index); 
}

kFx(kSize) kQueue_ItemSize(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    return kQueue_ItemSize_(obj); 
}

kFx(kType) kQueue_ItemType(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    return kQueue_ItemType_(obj); 
}

kFx(kSize) kQueue_Count(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    return kQueue_Count_(obj); 
}

kFx(kSize) kQueue_Capacity(kQueue queue)
{
    kQueueClass* obj = kQueue_Cast_(queue); 

    return kQueue_Capacity_(obj); 
}
