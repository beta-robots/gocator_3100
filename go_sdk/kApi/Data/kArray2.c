/** 
 * @file    kArray2.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArray2.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kArray2, kObject) 
    
    //serialization versions
    kAddVersion(kArray2, "kdat5", "5.0.0.0", "25-1", WriteDat5V1, ReadDat5V1)
    kAddVersion(kArray2, "kdat6", "5.7.1.0", "kArray2-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kArray2, kObject, VRelease)
    kAddVMethod(kArray2, kObject, VDisposeItems)
    kAddVMethod(kArray2, kObject, VInitClone)
    kAddVMethod(kArray2, kObject, VSize)

    //collection interface 
    kAddInterface(kArray2, kCollection)
    kAddIVMethod(kArray2, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArray2, kCollection, VItemType, ItemType)
    kAddIVMethod(kArray2, kCollection, VCount, Count)
    kAddIVMethod(kArray2, kCollection, VHasNext, HasNext)
    kAddIVMethod(kArray2, kCollection, VNext, Next)

kEndClass() 

kFx(kStatus) kArray2_Construct(kArray2* array, kType itemType, kSize length0, kSize length1, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kArray2), array)); 

    if (!kSuccess(status = kArray2_Init(*array, kTypeOf(kArray2), itemType, length0, length1, alloc)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
} 

kFx(kStatus) kArray2_Init(kArray2 array, kType classType, kType itemType, kSize length0, kSize length1, kAlloc allocator)
{
    kArray2Class* obj = array; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(array, classType, allocator)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL;
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArray2_Realloc(array, length0, length1)); 
    }
    kCatch(&status)
    {
        kArray2_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray2_VInitClone(kArray2 array, kArray2 source, kAlloc allocator)
{
    kArray2Class* obj = array; 
    kArray2Class* srcObj = kArray2_Cast_(source); 
    kStatus status = kOK; 
    kSize count = kArray2_Count_(source); 

    kCheck(kObject_Init_(array, kObject_Type_(source), allocator));     

    obj->itemType = srcObj->itemType; 
    obj->itemSize = srcObj->itemSize; 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArray2_Realloc(array, srcObj->length[0], srcObj->length[1])); 

        kTest(kCloneContent(obj->itemType, obj->items, srcObj->items, count, allocator)); 
    }
    kCatch(&status)
    {
        kArray2_VDisposeItems(array); 
        kArray2_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray2_VRelease(kArray2 array)
{
    kArray2Class* obj = kArray2_Cast_(array); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMem_(array, obj->items)); 
    }

    kCheck(kObject_VRelease_(array)); 

    return kOK; 
}

kFx(kStatus) kArray2_VDisposeItems(kArray2 array)
{
    kArray2Class* obj = kArray2_Cast_(array); 

    kCheck(kDisposeContent(obj->itemType, obj->items, kArray2_Count_(array))); 

    return kOK; 
}

kFx(kSize) kArray2_VSize(kArray2 array)
{
    kArray2Class* obj = kArray2_Cast_(array); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArray2_DataSize_(array); 
    kSize size = sizeof(kArray2Class) + dataSize; 

    size += kSizeContent(obj->itemType, obj->items, kArray2_Count_(array)); 

    return size; 
}

kFx(kStatus) kArray2_WriteDat5V1(kArray2 array, kSerializer serializer)
{
    kArray2Class* obj = kArray2_Cast_(array); 
    kTypeVersion itemVersion; 
    kSize count = kArray2_Count_(array); 

    kCheck(kSerializer_WriteSize_(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize_(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteSize_(serializer, count)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems_(serializer, obj->itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) kArray2_ReadDat5V1(kArray2 array, kSerializer serializer, kAlloc allocator)
{
    kArray2Class* obj = array; 
    kSize length0 = 0, length1 = 0, count = 0; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &length0)); 
    kCheck(kSerializer_ReadSize_(serializer, &length1)); 

    kCheck(kSerializer_ReadSize_(serializer, &count));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kArray2_Init(array, kTypeOf(kArray2), itemType, length0, length1, allocator)); 
  
    if (!kSuccess(status = kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, count)))
    {
        kArray2_VDisposeItems(array); 
        kArray2_VRelease(array); 
        return status;
    }

    return kOK; 
}

kFx(kStatus) kArray2_WriteDat6V0(kArray2 array, kSerializer serializer)
{
    kArray2Class *obj = kArray2_Cast_(array); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize_(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, kArray2_Count_(array))); 

    return kOK; 
}

kFx(kStatus) kArray2_ReadDat6V0(kArray2 array, kSerializer serializer, kAlloc allocator)
{
    kArray2Class *obj = array; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length0 = 0, length1 = 0; 
    kStatus status; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &length0)); 
    kCheck(kSerializer_ReadSize_(serializer, &length1)); 

    kCheck(kArray2_Init(array, kTypeOf(kArray2), itemType, length0, length1, allocator)); 

    kTry
    {
        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, kArray2_Count_(array))); 
    }
    kCatch(&status)
    {
        kArray2_VDisposeItems(array); 
        kArray2_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray2_Realloc(kArray2 array, kSize length0, kSize length1)
{
    kArray2Class *obj = kArray2_Cast_(array); 
    kSize newSize = kMax_(obj->allocSize, length0*length1*obj->itemSize); 

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
        kMemSet(obj->items, 0, length0*length1*obj->itemSize);  
    }

    obj->length[0] = length0; 
    obj->length[1] = length1; 
    
    return kOK; 
}


kFx(kStatus) kArray2_Allocate(kArray2 array, kType itemType, kSize length0, kSize length1)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->length[0] = 0; 
    obj->length[1] = 0; 

    kCheck(kArray2_Realloc(array, length0, length1)); 

    return kOK; 
}

kFx(kStatus) kArray2_Attach(kArray2 array, void* items, kType itemType, kSize length0, kSize length1)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMemRef_(array, &obj->items)); 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->length[0] = length0; 
    obj->length[1] = length1; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) kArray2_Assign(kArray2 array, kArray2 source)
{
    kArray2Class* obj = kArray2_Cast_(array); 
    kArray2Class* srcObj = kArray2_Cast_(source); 

    kCheck(kArray2_Allocate(array, srcObj->itemType, srcObj->length[0], srcObj->length[1])); 
    kCheck(kCopyContent(obj->itemType, obj->items, srcObj->items, kArray2_Count_(array))); 

    return kOK;   
}

kFx(kStatus) kArray2_Zero(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    kCheck(kZeroContent(obj->itemType, obj->items, kArray2_Count_(array)));

    return kOK; 
}

kFx(kStatus) kArray2_SetItem(kArray2 array, kSize index0, kSize index1, const void* item)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1])); 

    kItemImport_(kArray2_At_(array, index0, index1), item, obj->itemType); 

    return kOK; 
}

kFx(kStatus) kArray2_Item(kArray2 array, kSize index0, kSize index1, void* item)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1])); 

    kItemCopy_(item, kArray2_At_(array, index0, index1), obj->itemSize); 

    return kOK; 
}

kFx(kIterator) kArray2_GetIterator(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 
    return obj->items; 
}

kFx(kBool) kArray2_HasNext(kArray2 array, kIterator iterator)
{
    kArray2Class *obj = kArray2_Cast_(array); 
    void* end = (kByte*)obj->items + kArray2_DataSize_(array); 

    return (iterator != end); 
}

kFx(void*) kArray2_Next(kArray2 array, kIterator* iterator)
{
    kArray2Class *obj = kArray2_Cast_(array); 
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}

kFx(void*) kArray2_Data(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    return kArray2_Data_(obj); 
}

kFx(kSize) kArray2_DataSize(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    return kArray2_DataSize_(obj); 
}

kFx(void*) kArray2_At(kArray2 array, kSize index0, kSize index1)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    return kArray2_At_(obj, index0, index1); 
}

kFx(kType) kArray2_ItemType(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    return kArray2_ItemType_(obj); 
}

kFx(kSize) kArray2_ItemSize(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    return kArray2_ItemSize_(obj); 
}

kFx(kSize) kArray2_Length(kArray2 array, kSize dimension)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    kAssert(dimension < 2); 

    return kArray2_Length_(obj, dimension);  
}
 
kFx(kSize) kArray2_Count(kArray2 array)
{
    kArray2Class *obj = kArray2_Cast_(array); 

    return kArray2_Count_(obj); 
}
