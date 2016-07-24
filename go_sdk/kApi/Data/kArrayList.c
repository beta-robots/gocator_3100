/** 
 * @file    kArrayList.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kArrayList, kObject) 
    
    //serialization versions
    kAddVersion(kArrayList, "kdat5", "5.0.0.0", "27-1", WriteDat5V1, ReadDat5V1)
    kAddVersion(kArrayList, "kdat6", "5.7.1.0", "kArrayList-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kArrayList, kObject, VRelease)
    kAddVMethod(kArrayList, kObject, VDisposeItems)
    kAddVMethod(kArrayList, kObject, VInitClone)
    kAddVMethod(kArrayList, kObject, VSize)

    //collection interface 
    kAddInterface(kArrayList, kCollection)
    kAddIVMethod(kArrayList, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArrayList, kCollection, VItemType, ItemType)
    kAddIVMethod(kArrayList, kCollection, VCount, Count)
    kAddIVMethod(kArrayList, kCollection, VHasNext, HasNext)
    kAddIVMethod(kArrayList, kCollection, VNext, Next)

kEndClass() 

kFx(kStatus) kArrayList_Construct(kArrayList* list, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kArrayList), list)); 

    if (!kSuccess(status = kArrayList_Init(*list, kTypeOf(kArrayList), itemType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, list); 
    }

    return status; 
} 

kFx(kStatus) kArrayList_Init(kArrayList list, kType classType, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kArrayListClass* obj = list; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(list, classType, allocator)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArrayList_Reserve(list, initialCapacity)); 
    }
    kCatch(&status)
    {
        kArrayList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArrayList_VInitClone(kArrayList list, kArrayList source, kAlloc allocator)
{
    kArrayListClass* obj = list; 
    kArrayListClass* srcObj = source; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(list, kObject_Type_(source), allocator));     
    
    obj->itemType = srcObj->itemType; 
    obj->itemSize = srcObj->itemSize; 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArrayList_Resize(list, srcObj->count)); 
        kTest(kCloneContent(obj->itemType, obj->items, srcObj->items, obj->count, allocator)); 
    }
    kCatch(&status)
    {
        kArrayList_VDisposeItems(list); 
        kArrayList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArrayList_VRelease(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMem_(list, obj->items)); 
    }

    kCheck(kObject_VRelease_(list)); 

    return kOK; 
}

kFx(kStatus) kArrayList_VDisposeItems(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheck(kDisposeContent(obj->itemType, obj->items, obj->count)); 

    return kOK; 
}

kFx(kSize) kArrayList_VSize(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArrayList_DataSize_(list); 
    kSize size = sizeof(kArrayListClass) + dataSize; 

    size += kSizeContent(obj->itemType, obj->items, obj->count); 

    return size; 
}

kFx(kStatus) kArrayList_WriteDat5V1(kArrayList list, kSerializer serializer)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize_(serializer, obj->capacity)); 
    kCheck(kSerializer_Write32s_(serializer, kTRUE));  //isDynamic

    kCheck(kSerializer_WriteSize_(serializer, obj->count)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->count)); 

    return kOK; 
}

kFx(kStatus) kArrayList_ReadDat5V1(kArrayList list, kSerializer serializer, kAlloc allocator)
{
    kArrayListClass *obj = list; 
    kSize capacity = 0; 
    kSize count = 0; 
    kTypeVersion itemVersion;
    k32s isDynamic; 
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &capacity));      
    kCheck(kSerializer_Read32s_(serializer, &isDynamic));

    kCheck(kSerializer_ReadSize_(serializer, &count));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kArrayList_Init(list, kTypeOf(kArrayList), itemType, count, allocator)); 

    kTry
    {
        kTest(kArrayList_Resize(list, count)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, count)); 
    }
    kCatch(&status)
    {
        kArrayList_VDisposeItems(list); 
        kArrayList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArrayList_WriteDat6V0(kArrayList list, kSerializer serializer)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->count)); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->count)); 

    return kOK; 
}

kFx(kStatus) kArrayList_ReadDat6V0(kArrayList list, kSerializer serializer, kAlloc allocator)
{
    kArrayListClass *obj = list; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize count = 0; 
    kStatus status; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &count)); 

    kCheck(kArrayList_Init(list, kTypeOf(kArrayList), itemType, count, allocator)); 

    kTry
    {
        kTest(kArrayList_Resize(list, count)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, count)); 
    }
    kCatch(&status)
    {
        kArrayList_VDisposeItems(list); 
        kArrayList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArrayList_Allocate(kArrayList list, kType itemType, kSize initialCapacity)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->count = 0; 
    obj->capacity = 0; 

    kCheck(kArrayList_Reserve(list, initialCapacity)); 

    return kOK; 
}

kFx(kStatus) kArrayList_Attach(kArrayList list, void* items, kType itemType, kSize capacity)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMemRef_(list, &obj->items)); 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->count = capacity; 
    obj->capacity = capacity; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) kArrayList_Import(kArrayList list, const void* items, kType itemType, kSize count)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheck(kArrayList_Allocate(list, itemType, count)); 
    
    kCheck(kMemCopy(obj->items, items, count*obj->itemSize));     
    obj->count = count; 

    return kOK; 
}

kFx(kStatus) kArrayList_Append(kArrayList list, const void* items, kSize count)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheck(kArrayList_Reserve(list, obj->count + count)); 

    kCheck(kMemCopy(kArrayList_At_(list, obj->count), items, count*obj->itemSize)); 
    obj->count += count; 

    return kOK; 
}

kFx(kStatus) kArrayList_Assign(kArrayList list, kArrayList source)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 
    kArrayListClass* srcObj = kArrayList_Cast_(source); 

    kCheck(kArrayList_Allocate(list, srcObj->itemType, srcObj->count)); 

    kCheck(kArrayList_Resize(list, srcObj->count)); 

    kCheck(kCopyContent(obj->itemType, obj->items, srcObj->items, obj->count)); 

    return kOK;   
}

kFx(kStatus) kArrayList_Clear(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    obj->count = 0; 

    return kOK; 
}

kFx(kStatus) kArrayList_Purge(kArrayList list)
{    
    kCheck(kObject_DisposeItems_(list)); 
    kCheck(kArrayList_Clear_(list)); 

    return kOK; 
}

kFx(kStatus) kArrayList_Zero(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheck(kZeroContent(obj->itemType, obj->items, obj->count));

    return kOK; 
}

kFx(kStatus) kArrayList_Reserve(kArrayList list, kSize minimumCapacity)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (obj->capacity < minimumCapacity)
    {
        kSize grownCapacity = kMax_(minimumCapacity, kARRAY_LIST_GROWTH_FACTOR*obj->capacity);
        kSize newCapacity = kMax_(grownCapacity, kARRAY_LIST_MIN_CAPACITY); 
        kSize newSize = kMax_(obj->allocSize, newCapacity*obj->itemSize); 

        if (newSize > obj->allocSize)
        {
            void* oldItems = obj->items; 
            void* newItems = kNULL; 

            kCheck(kObject_GetMem_(list, newSize, &newItems)); 

            if (obj->count > 0)
            {
                kMemCopy(newItems, oldItems, obj->count*obj->itemSize); 
            }

            obj->items = newItems; 
            obj->allocSize = newSize; 

            kCheck(kObject_FreeMem_(list, oldItems)); 
        }

        obj->capacity = newCapacity;
    }
    
    return kOK; 
}

kFx(kStatus) kArrayList_Resize(kArrayList list, kSize count)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (count > obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, count)); 
    }

    if (kType_IsReference_(obj->itemType) && (count > obj->count))
    {
        kMemSet(kArrayList_At_(list, obj->count), 0, obj->itemSize*(count - obj->count)); 
    }

    obj->count = count; 
    
    return kOK; 
}

kFx(kStatus) kArrayList_AddCount(kArrayList list, kSize count)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if ((obj->count + count) > obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, obj->count + count)); 
    }

    obj->count += count; 

    return kOK; 
}

kFx(kStatus) kArrayList_RemoveCount(kArrayList list, kSize count)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (count > obj->count)
    {
        return kERROR_STATE; 
    }

    obj->count -= count; 

    return kOK; 
}


kFx(kStatus) kArrayList_Add(kArrayList list, const void* item)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    if (obj->count == obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, obj->capacity + 1)); 
    }

    kItemImport_(kArrayList_At_(list, obj->count), item, obj->itemType); 

    obj->count++; 

    return kOK; 
}

kFx(kStatus) kArrayList_SetItem(kArrayList list, kSize index, const void* item)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheckArgs(index < obj->count); 

    kItemImport_(kArrayList_At_(list, index), item, obj->itemType); 

    return kOK; 
}

kFx(kStatus) kArrayList_Item(kArrayList list, kSize index, void* item)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheckArgs(index < obj->count); 

    kItemCopy_(item, kArrayList_At_(list, index), obj->itemSize); 

    return kOK; 
}

kFx(kStatus) kArrayList_Insert(kArrayList list, kSize before, const void* item)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheckArgs(before <= obj->count); 

    if (obj->count == obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, obj->capacity + 1)); 
    }

    if (before != obj->count)
    {
        kMemMove(kArrayList_At_(list, before + 1), kArrayList_At_(list, before), obj->itemSize*(obj->count - before)); 
    }

    kItemImport_(kArrayList_At_(list, before), item, obj->itemType); 

    obj->count++; 

    return kOK; 
}

kFx(kStatus) kArrayList_Remove(kArrayList list, kSize index, void* item)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    kCheckArgs(index < obj->count); 

    if (item)
    {
        kItemCopy_(item, kArrayList_At_(list, index), obj->itemSize); 
    }

    if (index != (obj->count-1))
    {
        kMemMove(kArrayList_At_(list, index), kArrayList_At_(list, index + 1), obj->itemSize*(obj->count - index - 1)); 
    }

    obj->count--; 

    return kOK; 
}

kFx(kIterator) kArrayList_GetIterator(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return obj->items; 
}

kFx(kBool) kArrayList_HasNext(kArrayList list, kIterator iterator)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 
    void* end = (kByte*)obj->items + obj->count*obj->itemSize;

    return (iterator != end); 
}

kFx(void*) kArrayList_Next(kArrayList list, kIterator* iterator)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}

kFx(void*) kArrayList_Data(kArrayList list)
{
    kAssertType(list, kArrayList);

    return kArrayList_Data_(list); 
}

kFx(kSize) kArrayList_DataSize(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return kArrayList_DataSize_(obj); 
}

kFx(void*) kArrayList_At(kArrayList list, kSize index)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return kArrayList_At_(obj, index); 
}

kFx(kSize) kArrayList_ItemSize(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return kArrayList_ItemSize_(obj); 
}

kFx(kType) kArrayList_ItemType(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return kArrayList_ItemType_(obj); 
}

kFx(kSize) kArrayList_Count(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return kArrayList_Count_(obj); 
}

kFx(kSize) kArrayList_Capacity(kArrayList list)
{
    kArrayListClass* obj = kArrayList_Cast_(list); 

    return kArrayList_Capacity_(obj); 
}
