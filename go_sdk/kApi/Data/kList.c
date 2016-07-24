/** 
 * @file    kList.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kList.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kList, kObject) 

    //serialization versions
    kAddVersion(kList, "kdat6", "6.0.0.0", "kList-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kList, kObject, VRelease)
    kAddVMethod(kList, kObject, VDisposeItems)
    kAddVMethod(kList, kObject, VInitClone)
    kAddVMethod(kList, kObject, VSize)

    //kCollection interface
    kAddInterface(kList, kCollection); 
    kAddIVMethod(kList, kCollection, VGetIterator, CollectionGetIterator)
    kAddIVMethod(kList, kCollection, VItemType, CollectionItemType)
    kAddIVMethod(kList, kCollection, VCount, Count)
    kAddIVMethod(kList, kCollection, VHasNext, CollectionHasNext)
    kAddIVMethod(kList, kCollection, VNext, CollectionNext)

kEndClass() 

kFx(kStatus) kList_Construct(kList* list, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kList), list)); 

    if (!kSuccess(status = kList_Init(*list, kTypeOf(kList), itemType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, list); 
    }

    return status; 
} 

kFx(kStatus) kList_Init(kList list, kType classType, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kStatus status = kOK; 

    kCheck(kObject_Init_(list, classType, allocator)); 

    kInitFields_(kList, list);  
    
    kTry
    {
        kTest(kList_Layout(list, itemType)); 
        kTest(kList_Reserve(list, initialCapacity));       
    }
    kCatch(&status)
    {
        kList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kList_VInitClone(kList list, kList source, kAlloc allocator)
{ 
    kListClass* srcObj = source;
    kStatus status = kOK; 
    kObject contentClone = kNULL; 

    kCheck(kList_Init(list, kObject_Type_(source), srcObj->contentField.type, srcObj->count, allocator)); 

    kTry
    {
        kListItem item = kList_First(source); 

        while (!kIsNull(item))
        {
            const void* content = kListItem_Content_(source, item); 
            
            if (kList_ItemIsRef_(source))
            {
                kTest(kObject_Clone(&contentClone, *(kObject*)content, allocator)); 
                content = &contentClone; 
            }

            kTest(kList_Add(list, content, kNULL)); 
            contentClone = kNULL; 

            item = kList_Next(source, item); 
        }
    }
    kCatch(&status)
    {
        kObject_Dispose(contentClone); 
        kList_VDisposeItems(list); 
        kList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kList_VRelease(kList list)
{
    kCheck(kList_Deallocate(list)); 

    kCheck(kObject_VRelease_(list)); 

    return kOK; 
}

kFx(kStatus) kList_VDisposeItems(kList list)
{
    kListClass* obj = kList_Cast_(list);   

    if (!kIsNull(obj->contentField.type) && kList_ItemIsRef_(list))
    {
        kListItem item = kList_First(list); 

        while (!kIsNull(item))
        {
            kCheck(kObject_Dispose(kList_As_(list, item, kObject))); 

            item = kList_Next(list, item); 
        }
    }

    return kOK; 
}

kFx(kStatus) kList_Allocate(kList list, kType itemType, kSize initialCapacity)
{
    kListClass* obj = kList_Cast_(list);   
    
    kCheck(kList_Deallocate(list)); 

    obj->free = kNULL; 
    obj->first = kNULL; 
    obj->last = kNULL; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->contentField.type = kNULL; 

    kCheck(kList_Layout(list, itemType)); 
    kCheck(kList_Reserve(list, initialCapacity));   

    return kOK; 
}

kFx(kStatus) kList_Assign(kList list, kList source)
{
    kListClass* srcObj = kList_Cast_(source); 
    kListItem item  = kNULL; 

    kCheck(kList_Allocate(list, srcObj->contentField.type, srcObj->count)); 

    item = kList_First(source); 

    while (!kIsNull(item))
    {
        kCheck(kList_Add(list, kListItem_Content_(source, item), kNULL)); 
        
        item = kList_Next(source, item); 
    }

    return kOK; 
}

kFx(kStatus) kList_Layout(kList list, kType itemType)
{
    kListClass* obj = kList_Cast_(list);   
    kStructField nextField, previousField; 
    kStructField* fields[3];  

    nextField.type = kTypeOf(kPointer); 
    nextField.offset = offsetof(kListItemStruct, next); 
    nextField.count = 1; 
    fields[0] = &nextField; 

    previousField.type = kTypeOf(kPointer); 
    previousField.offset = offsetof(kListItemStruct, previous); 
    previousField.count = 1; 
    fields[1] = &previousField; 

    obj->contentField.type = kIsNull(itemType) ? kTypeOf(kVoid) : itemType;
    obj->contentField.offset = kSIZE_NULL; 
    obj->contentField.count = 1; 
    fields[2] = &obj->contentField; 

    kCheck(kType_LayoutStruct(fields, kCountOf(fields), &obj->itemSize)); 
              
    return kOK; 
}

kFx(kStatus) kList_Deallocate(kList list)
{
    kListClass* obj = kList_Cast_(list);   

    while (!kIsNull(obj->blocks))
    {
        kListItemBlock* next = obj->blocks->next; 

        kCheck(kObject_FreeMem(list, obj->blocks)); 

        obj->blocks = next; 
    }

    return kOK; 
}

kFx(kSize) kList_VSize(kList list)
{
    kListClass* obj = kList_Cast_(list);   
    kSize size = sizeof(kListClass);
    kListItemBlock* blockObj = obj->blocks; 

    while (!kIsNull(blockObj))
    {
        size += (kByte*)blockObj->items - (kByte*)blockObj; 
        size += obj->itemSize*blockObj->itemCount;

        blockObj = blockObj->next; 
    }

    if (kList_ItemIsRef_(list))
    {
        kListItem item = kList_First(list); 
        
        while (!kIsNull(item))
        {
            kObject content = kList_As_(list, item, kObject); 

            if (!kIsNull(content))
            {
                size += kObject_Size_(content); 
            }

            item = kList_Next(list, item); 
        }
    }

    return size; 
}

kFx(kStatus) kList_WriteDat6V0(kList list, kSerializer serializer)
{
    kListClass* obj = kList_Cast_(list);   
    kListItem item = kList_First(list); 
    kTypeVersion contentVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->contentField.type, &contentVersion)); 
    kCheck(kSerializer_WriteSize_(serializer, obj->count)); 

    while (!kIsNull(item))
    {
        kCheck(kSerializer_WriteItems_(serializer, obj->contentField.type, contentVersion, kListItem_Content_(list, item), 1)); 

        item = kList_Next(list, item); 
    }

    return kOK; 
}

kFx(kStatus) kList_ReadDat6V0(kList list, kSerializer serializer, kAlloc allocator)
{
    kListClass *obj = list; 
    kListItemStruct* itemObj = kNULL; 
    kType contentType; 
    kTypeVersion contentVersion; 
    kSize count = 0;  
    kStatus status = kOK; 
    kSize i; 

    kCheck(kSerializer_ReadType_(serializer, &contentType, &contentVersion)); 
    kCheck(kSerializer_ReadSize_(serializer, &count)); 

    kCheck(kList_Init(list, kTypeOf(kList), contentType, count, allocator)); 

    kTry
    {         
        for (i = 0; i < count; ++i)
        {   
            itemObj = obj->free; 
            obj->free = itemObj->next; 

            itemObj->next = kNULL; 
            itemObj->previous = obj->last; 

            if (!kIsNull(itemObj->previous))
            {
                itemObj->previous->next = itemObj; 
            }

            obj->last = itemObj; 
                
            if (kIsNull(obj->first))
            {
                obj->first = itemObj; 
            }

            kItemZero_((void*)kListItem_Content_(list, itemObj), obj->contentField.fieldSize); 

            kTest(kSerializer_ReadItems_(serializer, obj->contentField.type, contentVersion, (void*)kListItem_Content_(list, itemObj), 1)); 
        }

        obj->count = count; 
    }
    kCatch(&status)
    {
        kList_VDisposeItems(list); 
        kList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kList_Reserve(kList list, kSize capacity)
{
    kListClass* obj = kList_Cast_(list);   

    if (capacity > obj->capacity)
    {
        kSize itemCount = kMax_(capacity - obj->capacity, kLIST_MIN_ITEMS_PER_BLOCK); 
        kSize itemOffset = kAlign_(sizeof(kListItemBlock), kALIGN_ANY); 
        kSize blockSize = kAlign_(itemOffset + itemCount*obj->itemSize, kALIGN_ANY); 
        kListItemBlock* block = kNULL; 
        kSize i; 

        kCheck(kObject_GetMem_(list, blockSize, &block)); 

        block->itemCount = itemCount; 
        block->items = (void*)((kByte*)block + itemOffset); 

        block->next = obj->blocks; 
        obj->blocks = block; 

        for (i = 0; i < itemCount; ++i)
        {
            kListItemStruct* item = (kListItemStruct*) ((kByte*)(&block->items[0]) + i*obj->itemSize); 

            item->next = obj->free; 
            obj->free = item; 
        }

        obj->capacity += itemCount; 
    }

    return kOK; 
}

kFx(kIterator) kList_CollectionGetIterator(kList list)
{
    return kList_First(list); 
}

kFx(kType) kList_CollectionItemType(kList list)
{
    return kList_ItemType(list); 
}

kFx(kBool) kList_CollectionHasNext(kList list, kIterator iterator)
{
    return !kIsNull(iterator); 
}

kFx(void*) kList_CollectionNext(kList list, kIterator* iterator)
{
    void* content = kListItem_Content_(list, *iterator); 

    *iterator = kList_Next(list, *iterator); 

    return content; 
}

kFx(kType) kList_ItemType(kList list)
{
    kListClass* obj = kList_Cast_(list);   

    return obj->contentField.type; 
}

kFx(kSize) kList_Count(kList list)
{
    kListClass* obj = kList_Cast_(list);   

    return obj->count; 
}

kFx(kSize) kList_Capacity(kList list)
{
    kListClass* obj = kList_Cast_(list);   

    return obj->capacity; 
}

kFx(kStatus) kList_Add(kList list, const void* itemContent, kListItem* item)
{
    kListClass* obj = kList_Cast_(list);   
    kListItemStruct* itemObj = kNULL; 
    void* contentSlot = kNULL; 

    if (obj->count == obj->capacity)
    {
        kCheck(kList_Reserve(list, obj->capacity+1)); 
    }

    itemObj = obj->free; 
    obj->free = itemObj->next; 

    itemObj->next = kNULL; 
    itemObj->previous = obj->last; 

    if (!kIsNull(itemObj->previous))
    {
        itemObj->previous->next = itemObj; 
    }

    obj->last = itemObj; 
                
    if (kIsNull(obj->first))
    {
        obj->first = itemObj; 
    }

    obj->count++; 

    contentSlot = (void*)kListItem_Content_(list, itemObj); 

    if (!kIsNull(itemContent))
    {
        kItemImport_(contentSlot, itemContent, obj->contentField.type); 
    }
    
    if (!kIsNull(item))
    {
        *item = itemObj; 
    }

    return kOK; 
}

kFx(kStatus) kList_Insert(kList list, kListItem before, const void* itemContent, kListItem* item)
{
    kListClass* obj = kList_Cast_(list);   
    kListItemStruct* itemObj = kNULL; 
    kListItemStruct* beforeObj = before; 
    void* contentSlot = kNULL; 

    if (kIsNull(before))
    {
        return kList_Add(list, itemContent, item); 
    }

    if (obj->count == obj->capacity)
    {
        kCheck(kList_Reserve(list, obj->capacity+1)); 
    }

    itemObj = obj->free; 
    obj->free = itemObj->next; 

    itemObj->next = beforeObj; 
    itemObj->previous = beforeObj->previous; 

    if (!kIsNull(itemObj->previous))
    {
        itemObj->previous->next = itemObj; 
    }

    beforeObj->previous = itemObj; 

    if (obj->first == beforeObj)
    {
        obj->first = itemObj; 
    }

    obj->count++; 

    contentSlot = (void*)kListItem_Content_(list, itemObj); 

    if (!kIsNull(itemContent))
    {
        kItemImport_(contentSlot, itemContent, obj->contentField.type); 
    }

    if (!kIsNull(item))
    {
        *item = itemObj; 
    }
    
    return kOK; 
}

kFx(kStatus) kList_Remove(kList list, kListItem item)
{
    kListClass* obj = kList_Cast_(list);   
    kListItemStruct* itemObj = item; 

    if (!kIsNull(itemObj->previous))
    {
        itemObj->previous->next = itemObj->next; 
    }
    
    if (!kIsNull(itemObj->next))
    {
        itemObj->next->previous = itemObj->previous; 
    }

    if (obj->first == itemObj)
    {
        obj->first = itemObj->next; 
    }

    if (obj->last == itemObj)
    {
        obj->last = itemObj->previous; 
    }

    itemObj->next = obj->free; 
    itemObj->previous = kNULL;
    obj->free = itemObj; 

    obj->count--; 
    
    return kOK; 
}

kFx(kStatus) kList_Clear(kList list)
{
    kListClass* obj = kList_Cast_(list);   

    if (obj->count > 0)
    {
        obj->last->next = obj->free; 
        obj->free = obj->first; 

        obj->first = kNULL; 
        obj->last = kNULL; 

        obj->count = 0; 
    }

    return kOK; 
}

kFx(kStatus) kList_Purge(kList list)
{    
    kCheck(kObject_DisposeItems_(list)); 
    kCheck(kList_Clear(list)); 

    return kOK; 
}

kFx(kStatus) kList_SetItem(kList list, kListItem item, const void* content)
{
    kListClass* obj = kList_Cast_(list);   
    void* contentSlot = kListItem_Content_(list, item); 

    kItemImport_(contentSlot, content, obj->contentField.type); 

    return kOK; 
}

kFx(kStatus) kList_Item(kList list, kListItem item, void* content) 
{
    kListClass* obj = kList_Cast_(list);   
    void* contentSlot = kListItem_Content_(list, item); 

    kItemCopy_(content, contentSlot, obj->contentField.fieldSize); 

    return kOK; 
}

kFx(kListItem) kList_First(kList list)
{
    kListClass* obj = kList_Cast_(list);   
    return obj->first; 
}

kFx(kListItem) kList_Last(kList list)
{
    kListClass* obj = kList_Cast_(list);   
    return obj->last; 
}

kFx(kListItem) kList_Next(kList list, kListItem item)
{
    kAssertType(list, kList);

    return kList_Next_(list, item); 
}

kFx(kListItem) kList_Previous(kList list, kListItem item)
{
    kAssertType(list, kList);

    return kList_Previous_(list, item); 
}

kFx(kListItem) kList_FindIndex(kList list, kSize index)
{
    kListClass* obj = kList_Cast_(list);   

    if (index > obj->count)
    {
        return kNULL; 
    }
    else
    {
        kListItem it = kList_First(list); 
        kSize i; 

        for (i = 0; i < index; ++i)
        {
            it = kList_Next(list, it); 
        }

        return it; 
    }
}

kFx(void*) kList_At(kList list, kListItem item)
{
    kAssertType(list, kList);    
    
    return kListItem_Content_(list, item); 
}


kFx(void*) kList_AtIndex(kList list, kSize index)
{
    kListItem item = kList_FindIndex(list, index); 

    return kIsNull(item) ? kNULL : kList_At_(list, item); 
}
