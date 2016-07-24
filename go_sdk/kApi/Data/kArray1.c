/** 
 * @file    kArray1.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kArray1, kObject) 
    
    //serialization versions
    kAddVersion(kArray1, "kdat5", "5.0.0.0", "24-1", WriteDat5V1, ReadDat5V1)
    kAddVersion(kArray1, "kdat6", "5.7.1.0", "kArray1-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kArray1, kObject, VRelease)
    kAddVMethod(kArray1, kObject, VDisposeItems)
    kAddVMethod(kArray1, kObject, VInitClone)
    kAddVMethod(kArray1, kObject, VSize)

    //collection interface 
    kAddInterface(kArray1, kCollection)
    kAddIVMethod(kArray1, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArray1, kCollection, VItemType, ItemType)
    kAddIVMethod(kArray1, kCollection, VCount, Count)
    kAddIVMethod(kArray1, kCollection, VHasNext, HasNext)
    kAddIVMethod(kArray1, kCollection, VNext, Next)

kEndClass() 

kFx(kStatus) kArray1_Construct(kArray1* array, kType itemType, kSize length, kObject allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kArray1), array)); 

    if (!kSuccess(status = kArray1_Init(*array, kTypeOf(kArray1), itemType, length, alloc)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
} 

kFx(kStatus) kArray1_Init(kArray1 array, kType classType, kType itemType, kSize length, kAlloc allocator)
{
    kArray1Class* obj = array; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(array, classType, allocator)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL;
    obj->length = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArray1_Realloc(array, length)); 
    }
    kCatch(&status)
    {
        kArray1_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray1_VInitClone(kArray1 array, kArray1 source, kAlloc allocator)
{
    kArray1Class* obj = array; 
    kArray1Class* srcObj = kArray1_Cast_(source); 
    kStatus status = kOK; 

    kCheck(kObject_Init_(array, kObject_Type_(source), allocator));     

    obj->itemType = srcObj->itemType; 
    obj->itemSize = srcObj->itemSize; 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->length = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArray1_Realloc(array, srcObj->length)); 

        kTest(kCloneContent(obj->itemType, obj->items, srcObj->items, obj->length, allocator)); 
    }
    kCatch(&status)
    {
        kArray1_VDisposeItems(array); 
        kArray1_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray1_VRelease(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMem_(array, obj->items)); 
    }

    kCheck(kObject_VRelease_(array)); 

    return kOK; 
}

kFx(kStatus) kArray1_VDisposeItems(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    kCheck(kDisposeContent(obj->itemType, obj->items, obj->length)); 

    return kOK; 
}

kFx(kSize) kArray1_VSize(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArray1_DataSize_(array); 
    kSize size = sizeof(kArray1Class) + dataSize; 

    size += kSizeContent(obj->itemType, obj->items, obj->length); 

    return size; 
}

kFx(kStatus) kArray1_WriteDat5V1(kArray1 array, kSerializer serializer)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize_(serializer, obj->length)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems_(serializer, obj->itemType, itemVersion, obj->items, obj->length)); 

    return kOK; 
}

kFx(kStatus) kArray1_ReadDat5V1(kArray1 array, kSerializer serializer, kAlloc allocator)
{
    kArray1Class* obj = array;   
    kSize length = 0; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &length));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kArray1_Init(array, kTypeOf(kArray1), itemType, length, allocator)); 
  
    if (!kSuccess(status = kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, length)))
    {
        kArray1_VDisposeItems(array); 
        kArray1_VRelease(array); 
        return status;
    }

    return kOK; 
}

kFx(kStatus) kArray1_WriteDat6V0(kArray1 array, kSerializer serializer)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->length)); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->length)); 

    return kOK; 
}

kFx(kStatus) kArray1_ReadDat6V0(kArray1 array, kSerializer serializer, kAlloc allocator)
{
    kArray1Class* obj = array; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kStatus status; 
    kSize length = 0; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &length)); 

    kCheck(kArray1_Init(array, kTypeOf(kArray1), itemType, length, allocator)); 

    kTry
    {
        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, length)); 
    }
    kCatch(&status)
    {
        kArray1_VDisposeItems(array); 
        kArray1_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray1_Realloc(kArray1 array, kSize length)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    kSize newSize = kMax_(obj->allocSize, length*obj->itemSize); 

    if (newSize > obj->allocSize)
    {
        void* oldItems = obj->items; 
        void* newItems = kNULL; 

        kCheck(kObject_GetMem_(array, newSize, &newItems));         
        
        obj->items = newItems; 
        obj->allocSize = newSize; 

        kCheck(kObject_FreeMem_(array, oldItems)); 
    }
        
    if (kType_IsReference_(obj->itemType))
    {
        kMemSet(obj->items, 0, obj->itemSize*length);  
    }

    obj->length = length; 
    
    return kOK; 
}

kFx(kStatus) kArray1_Allocate(kArray1 array, kType itemType, kSize length)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->length = 0; 

    kCheck(kArray1_Realloc(array, length)); 

    return kOK; 
}

kFx(kStatus) kArray1_Attach(kArray1 array, void* items, kType itemType, kSize length)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMemRef_(array, &obj->items)); 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->length = length; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) kArray1_Assign(kArray1 array, kArray1 source)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    kArray1Class* srcObj = kArray1_Cast_(source); 

    kCheck(kArray1_Allocate(array, srcObj->itemType, srcObj->length)); 
    kCheck(kCopyContent(obj->itemType, obj->items, srcObj->items, obj->length)); 

    return kOK;   
}

kFx(kStatus) kArray1_Zero(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    kCheck(kZeroContent(obj->itemType, obj->items, obj->length));

    return kOK; 
}

kFx(kStatus) kArray1_SetItem(kArray1 array, kSize index, const void* item)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    kCheckArgs(index < obj->length);    

    kItemImport_(kArray1_At_(array, index), item, obj->itemType); 

    return kOK; 
}

kFx(kStatus) kArray1_Item(kArray1 array, kSize index, void* item)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    kCheckArgs(index < obj->length); 

    kItemCopy_(item, kArray1_At_(array, index), obj->itemSize); 

    return kOK; 
}

kFx(kIterator) kArray1_GetIterator(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    return obj->items; 
}

kFx(kBool) kArray1_HasNext(kArray1 array, kIterator iterator)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    void* end = (kByte*)obj->items + obj->length*obj->itemSize;

    return (iterator != end); 
}

kFx(void*) kArray1_Next(kArray1 array, kIterator* iterator)
{
    kArray1Class* obj = kArray1_Cast_(array);  
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}

kFx(void*) kArray1_Data(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_Data_(obj); 
}

kFx(kSize) kArray1_DataSize(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_DataSize_(obj); 
}

kFx(void*) kArray1_At(kArray1 array, kSize index)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_At_(obj, index); 
}

kFx(kType) kArray1_ItemType(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_ItemType_(obj); 
}

kFx(kSize) kArray1_ItemSize(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_ItemSize_(obj); 
}

kFx(kSize) kArray1_Length(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_Length_(obj); 
}
 
kFx(kSize) kArray1_Count(kArray1 array)
{
    kArray1Class* obj = kArray1_Cast_(array);  

    return kArray1_Count_(obj); 
}
