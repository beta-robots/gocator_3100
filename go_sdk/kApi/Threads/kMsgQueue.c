/** 
 * @file    kMsgQueue.c
 *
 * @internal
 * Copyright (C) 2011-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Threads/kMsgQueue.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kSemaphore.h>

kBeginClass(k, kMsgQueue, kObject)
    kAddVMethod(kMsgQueue, kObject, VRelease)
    kAddVMethod(kMsgQueue, kObject, VDisposeItems)
    kAddVMethod(kMsgQueue, kObject, VSize)
kEndClass()

kFx(kStatus) kMsgQueue_Construct(kMsgQueue* queue, kType itemType, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kMsgQueue), queue)); 

    if (!kSuccess(status = kMsgQueue_Init(*queue, kTypeOf(kMsgQueue), itemType, alloc)))
    {
        kAlloc_FreeRef(alloc, queue); 
    }

    return status; 
} 

kFx(kStatus) kMsgQueue_Init(kMsgQueue queue, kType type, kType itemType, kAlloc allocator)
{
    kMsgQueueClass* obj = queue; 
    kStatus status; 

    kCheck(kObject_Init(queue, type, allocator));     

    kInitFields_(kMsgQueue, queue); 

    obj->entryDivisor = 1; 
    obj->maxCount = kSIZE_MAX; 
    obj->maxSize = kSIZE_MAX; 

    kTry
    {
        kTest(kMsgQueue_Layout(queue, itemType)); 
        kTest(kSemaphore_Construct(&obj->canRemove, 0, allocator)); 
        kTest(kLock_Construct(&obj->lock, allocator)); 
    }
    kCatch(&status)
    {
        kMsgQueue_VRelease(queue); 
        kEndCatch(status); 
    }    
    
    return kOK; 
}

kFx(kStatus) kMsgQueue_Layout(kMsgQueue queue, kType itemType)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kStructField sizeField; 
    kStructField* fields[2];     

    sizeField.type = kTypeOf(kSize); 
    sizeField.offset = offsetof(kMsgQueueEntryStruct, size); 
    sizeField.count = 1; 
    fields[0] = &sizeField; 

    obj->itemField.type = itemType; 
    obj->itemField.offset = kSIZE_NULL; 
    obj->itemField.count = 1; 
    fields[1] = &obj->itemField; 

    kCheck(kType_LayoutStruct(fields, kCountOf(fields), &obj->entrySize)); 
 
    return kOK; 
}

kFx(kStatus) kMsgQueue_VRelease(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    kCheck(kObject_FreeMem_(queue, obj->entries)); 

    kCheck(kObject_Destroy(obj->canRemove)); 
    kCheck(kObject_Destroy(obj->lock));   

    kCheck(kObject_VRelease_(queue)); 

    return kOK; 
}

kFx(kStatus) kMsgQueue_VDisposeItems(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize i; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        if (kType_IsReference_(obj->itemField.type))
        {
            for (i = 0; i < obj->count; ++i)
            {
                kMsgQueueEntry entry = kMsgQueue_EntryAt_(queue, i); 
                kTest(kDisposeRef(kMsgQueueEntry_Item_(queue, entry))); 
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kSize) kMsgQueue_VSize(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize size = 0; 

    kCheck(kLock_Enter(obj->lock));
    {
        size += sizeof(kMsgQueueClass) + obj->entryCapacity*obj->entrySize + obj->size; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return size; 
}

kFx(kStatus) kMsgQueue_SetDropHandler(kMsgQueue queue, kMsgQueueDropFx onDrop, kPointer receiver)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    obj->onDrop.function = (kCallbackFx) onDrop; 
    obj->onDrop.receiver = receiver; 

    return kOK; 
}

kFx(kStatus) kMsgQueue_SetMaxSize(kMsgQueue queue, kSize size)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        obj->maxSize = size; 

        kTest(kMsgQueue_Prune(queue)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kMsgQueue_SetMaxCount(kMsgQueue queue, kSize count)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        obj->maxCount = count; 

        kTest(kMsgQueue_Prune(queue)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kBool) kMsgQueue_ExceedsCapacity(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    return (obj->count > obj->maxCount) || (obj->size > obj->maxSize); 
}

kFx(kStatus) kMsgQueue_Add(kMsgQueue queue, void* item)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kMsgQueueEntryStruct* entryObj = kNULL; 
    kSize itemSize = 0; 
    kBool shouldPost = kFALSE; 
    
    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        if (kMsgQueue_ItemIsRef_(queue))
        {
            kObject objectItem = kAs_(item, kObject); 
            itemSize = kIsNull(objectItem) ? 0 : kObject_Size(objectItem); 
        }
        else
        {
            itemSize = obj->itemField.fieldSize; 
        }

        if (obj->count == obj->entryCapacity)
        {
            kTest(kMsgQueue_Reserve(queue, obj->entryCapacity + 1)); 
        }
        
        entryObj = kMsgQueue_EntryAt_(queue, obj->count); 
        entryObj->size = itemSize; 
        kItemImport_(kMsgQueueEntry_Item_(queue, entryObj), item, obj->itemField.type); 

        obj->size += itemSize; 
        obj->count++;        
        shouldPost = kTRUE; 

        if (kMsgQueue_ExceedsCapacity(queue))
        {
            kMsgQueue_Prune(queue); 
        }
        
        if (obj->pruneCount > 0)
        {       
            shouldPost = kFALSE; 
            obj->pruneCount--; 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock);      

        if (shouldPost)
        {
            kCheck(kSemaphore_Post(obj->canRemove)); 
        }

        kEndFinally(); 
    }
   
    return kOK; 
}

kFx(kStatus) kMsgQueue_Remove(kMsgQueue queue, void* item, k64u timeout)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kBool shouldContinue = kTRUE;     

    while (shouldContinue)
    {
        kCheck(kSemaphore_Wait(obj->canRemove, timeout)); 

        kCheck(kLock_Enter(obj->lock)); 
        {
            if (obj->pruneCount > 0)
            {
                obj->pruneCount--; 
            }
            else
            {
                kMsgQueueEntryStruct* entryObj = kMsgQueue_EntryAt_(queue, 0); 
                kSize itemSize = entryObj->size; 

                shouldContinue = kFALSE; 

                if (item)
                {
                    kItemCopy_(item, kMsgQueueEntry_Item_(queue, entryObj), obj->itemField.fieldSize); 
                }

                obj->size -= itemSize; 
                obj->head = (obj->head + 1) % obj->entryDivisor; 
                obj->count--; 
            }
        }
        kCheck(kLock_Exit(obj->lock)); 
    }  

    return kOK; 
} 

kFx(kStatus) kMsgQueue_Clear(kMsgQueue queue)
{
    while (kSuccess(kMsgQueue_Remove(queue, kNULL, 0))); 

    return kOK; 
}

kFx(kStatus) kMsgQueue_Purge(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    if (kType_IsReference_(obj->itemField.type))
    {
        kObject item = kNULL; 

        while (kSuccess(kMsgQueue_Remove(queue, &item, 0)))
        {
            kCheck(kObject_Dispose(item)); 
        }
    }
    else
    {
        return kMsgQueue_Clear(queue); 
    }

    return kOK; 
}

kFx(kStatus) kMsgQueue_Reserve(kMsgQueue queue, kSize count)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize mimumCapacity = count + 1;    //reserved one extra; allows add before prune
   
    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        if (obj->entryCapacity < mimumCapacity)
        {
            kSize newCapacity = 0; 
            void* newItems = kNULL;  

            newCapacity = kMax_(mimumCapacity, kMSG_QUEUE_GROWTH_FACTOR*obj->entryCapacity);
            newCapacity = kMax_(newCapacity, kMSG_QUEUE_MIN_CAPACITY); 

            kTest(kObject_GetMem(queue, newCapacity*obj->entrySize, &newItems)); 

            if (obj->count > 0)
            {
                kSize headCount = kMsgQueue_HeadCount_(queue); 
                kSize tailCount = kMsgQueue_TailCount_(queue); 
                kSize entrySize = obj->entrySize; 

                kMemCopy(kItemAt_(newItems, 0, entrySize), kMsgQueue_EntryAt_(queue, 0), headCount*entrySize); 
                kMemCopy(kItemAt_(newItems, headCount, entrySize), kMsgQueue_EntryAt_(queue, headCount), tailCount*entrySize); 
            }

            kObject_FreeMem(queue, obj->entries); 

            obj->entries = newItems; 
            obj->entryCapacity = newCapacity; 
            obj->entryDivisor = kMax_(1, obj->entryCapacity); 
            obj->head = 0; 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }
    
    return kOK; 
}

kFx(kStatus) kMsgQueue_Prune(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kMsgQueueDropArgs args; 

    while (kMsgQueue_ExceedsCapacity(queue))
    {
        kMsgQueueEntryStruct* entryObj = kMsgQueue_EntryAt_(queue, 0); 
        void* entryItem = kMsgQueueEntry_Item_(queue, entryObj); 

        if (obj->onDrop.function)
        {
            args.item = entryItem; 
            obj->onDrop.function(obj->onDrop.receiver, queue, &args); 
        }
        else if (kMsgQueue_ItemIsRef_(queue))
        {
            kDisposeRef(entryItem); 
        }

        obj->size -= entryObj->size; 
        obj->head = (obj->head + 1) % obj->entryCapacity; 
        obj->count--; 
        obj->pruneCount++; 
        obj->dropCount++; 
    }

    return kOK; 
}

kFx(kSize) kMsgQueue_Count(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize count = 0; 

    kCheck(kLock_Enter(obj->lock)); 
    {
        count = obj->count; 
    }
    kCheck(kLock_Exit(obj->lock)); 
    
    return count; 
}

kFx(kSize) kMsgQueue_MaxSize(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize size = 0; 

    kCheck(kLock_Enter(obj->lock)); 
    {
        size = obj->maxSize; 
    }
    kCheck(kLock_Exit(obj->lock)); 
    
    return size; 
}

kFx(kSize) kMsgQueue_MaxCount(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize count = 0; 

    kCheck(kLock_Enter(obj->lock)); 
    {
        count = obj->maxCount; 
    }
    kCheck(kLock_Exit(obj->lock)); 
    
    return count; 
}

kFx(kType) kMsgQueue_ItemType(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    
    return obj->itemField.type;
}

kFx(kSize) kMsgQueue_ItemSize(kQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue);

    return obj->itemField.typeSize;
}

kFx(kSize) kMsgQueue_DataSize(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize size = 0; 

    kLock_Enter(obj->lock); 
    {
        size = obj->size; 
    }
    kLock_Exit(obj->lock); 
    
    return size; 
}

kFx(k64u) kMsgQueue_DropCount(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    k64u dropCount = 0; 

    kLock_Enter(obj->lock); 
    {
        dropCount = obj->dropCount; 
    }
    kLock_Exit(obj->lock); 
    
    return dropCount; 
}

//Deprecated
kFx(kStatus) kMsgQueue_SetCapacity(kMsgQueue queue, kMsgQueueCapacityType capacityType, kSize capacity)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        obj->maxCount = kSIZE_MAX; 
        obj->maxSize = kSIZE_MAX; 

        if (capacityType == kMSG_QUEUE_CAPACITY_TYPE_COUNT)
        {
            obj->maxCount = capacity; 
        }
        else if (capacityType == kMSG_QUEUE_CAPACITY_TYPE_MEMORY)
        {
            obj->maxSize = capacity; 
        }

        kTest(kMsgQueue_Prune(queue)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);        
        kEndFinally(); 
    }

    return kOK; 
}

//Deprecated
kFx(kSize) kMsgQueue_Capacity(kMsgQueue queue)
{
    kMsgQueueClass* obj = kMsgQueue_Cast_(queue); 
    kSize capacity = kSIZE_MAX;  

    kLock_Enter(obj->lock); 
    {
        capacity = kMin_(capacity, obj->maxCount); 
        capacity = kMin_(capacity, obj->maxSize); 
    }
    kLock_Exit(obj->lock); 
    
    return capacity; 
}
