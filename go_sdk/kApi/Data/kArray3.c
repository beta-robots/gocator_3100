/** 
 * @file    kArray3.c
 *
 * @internal
 * Copyright (C) 2006-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArray3.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kArray3, kObject) 
    
    //serialization versions
    kAddVersion(kArray3, "kdat5", "5.0.0.0", "26-0", WriteDat5V0, ReadDat5V0)
    kAddVersion(kArray3, "kdat6", "5.7.1.0", "kArray3-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kArray3, kObject, VRelease)
    kAddVMethod(kArray3, kObject, VDisposeItems)
    kAddVMethod(kArray3, kObject, VInitClone)
    kAddVMethod(kArray3, kObject, VSize)

    //collection interface 
    kAddInterface(kArray3, kCollection)
    kAddIVMethod(kArray3, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArray3, kCollection, VItemType, ItemType)
    kAddIVMethod(kArray3, kCollection, VCount, Count)
    kAddIVMethod(kArray3, kCollection, VHasNext, HasNext)
    kAddIVMethod(kArray3, kCollection, VNext, Next)

kEndClass() 

kFx(kStatus) kArray3_Construct(kArray3* array, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kArray3), array)); 

    if (!kSuccess(status = kArray3_Init(*array, kTypeOf(kArray3), itemType, length0, length1, length2, alloc)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
} 

kFx(kStatus) kArray3_Init(kArray3 array, kType classType, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator)
{
    kArray3Class* obj = array; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(array, classType, allocator)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL;
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->length[2] = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArray3_Realloc(array, length0, length1, length2)); 
    }
    kCatch(&status)
    {
        kArray3_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray3_VInitClone(kArray3 array, kArray3 source, kAlloc allocator)
{
    kArray3Class* obj = array; 
    kArray3Class* srcObj = source; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(array, kObject_Type_(source), allocator));     

    obj->itemType = srcObj->itemType; 
    obj->itemSize = srcObj->itemSize; 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->length[2] = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArray3_Realloc(array, srcObj->length[0], srcObj->length[1],  srcObj->length[2])); 
        kTest(kCloneContent(obj->itemType, obj->items, srcObj->items, kArray3_Count_(source), allocator));  
    }
    kCatch(&status)
    {
        kArray3_VDisposeItems(array); 
        kArray3_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray3_VRelease(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMem_(array, obj->items)); 
    }

    kCheck(kObject_VRelease_(array)); 

    return kOK; 
}

kFx(kStatus) kArray3_VDisposeItems(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    kCheck(kDisposeContent(obj->itemType, obj->items, kArray3_Count_(array))); 

    return kOK; 
}

kFx(kSize) kArray3_VSize(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArray3_DataSize_(array); 
    kSize size = sizeof(kArray3Class) + dataSize; 

    size += kSizeContent(obj->itemType, obj->items, kArray3_Count_(array)); 

    return size; 
}

kFx(kStatus) kArray3_WriteDat5V0(kArray3 array, kSerializer serializer)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    kTypeVersion itemVersion; 
    kSize count = kArray3_Count_(array); 

    kCheck(kSerializer_WriteSize_(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize_(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteSize_(serializer, obj->length[2])); 
    kCheck(kSerializer_WriteSize_(serializer, count)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems_(serializer, obj->itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) kArray3_ReadDat5V0(kArray3 array, kSerializer serializer, kAlloc allocator)
{
    kArray3Class *obj = array; 
    kSize length0 = 0, length1 = 0, length2 = 0, count = 0; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &length0)); 
    kCheck(kSerializer_ReadSize_(serializer, &length1)); 
    kCheck(kSerializer_ReadSize_(serializer, &length2)); 

    kCheck(kSerializer_Read32u_(serializer, &count));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kArray3_Init(array, kTypeOf(kArray3), itemType, length0, length1, length2, allocator)); 

    if (!kSuccess(status = kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, count)))
    {
        kArray3_VDisposeItems(array); 
        kArray3_VRelease(array);
        return status;
    }

    return kOK; 
}

kFx(kStatus) kArray3_WriteDat6V0(kArray3 array, kSerializer serializer)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize_(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteSize_(serializer, obj->length[2])); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, kArray3_Count_(array))); 

    return kOK; 
}

kFx(kStatus) kArray3_ReadDat6V0(kArray3 array, kSerializer serializer, kAlloc allocator)
{
    kArray3Class *obj = array; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length0 = 0, length1 = 0, length2 = 0; 
    kStatus status; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &length0)); 
    kCheck(kSerializer_ReadSize_(serializer, &length1)); 
    kCheck(kSerializer_ReadSize_(serializer, &length2)); 

    kCheck(kArray3_Init(array, kTypeOf(kArray3), itemType, length0, length1, length2, allocator)); 

    kTry
    {
        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->items, kArray3_Count_(array))); 
    }
    kCatch(&status)
    {
        kArray3_VDisposeItems(array); 
        kArray3_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kArray3_Realloc(kArray3 array, kSize length0, kSize length1, kSize length2)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    kSize newSize = kMax_(obj->allocSize, length0*length1*length2*obj->itemSize); 

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
        kMemSet(obj->items, 0, length0*length1*length2*obj->itemSize);  
    }

    obj->length[0] = length0; 
    obj->length[1] = length1; 
    obj->length[2] = length2; 
    
    return kOK; 
}


kFx(kStatus) kArray3_Allocate(kArray3 array, kType itemType, kSize length0, kSize length1, kSize length2)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->length[2] = 0; 

    kCheck(kArray3_Realloc(array, length0, length1, length2)); 

    return kOK; 
}

kFx(kStatus) kArray3_Attach(kArray3 array, void* items, kType itemType, kSize length0, kSize length1, kSize length2)
{
    kArray3Class* obj = kArray3_Cast_(array); 

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
    obj->length[2] = length2; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) kArray3_Assign(kArray3 array, kArray3 source)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    kArray3Class* srcObj = kArray3_Cast_(source); 

    kCheck(kArray3_Allocate(array, srcObj->itemType, srcObj->length[0], srcObj->length[1], srcObj->length[2])); 
    kCheck(kCopyContent(obj->itemType, obj->items, srcObj->items, kArray3_Count_(array))); 

    return kOK;   
}

kFx(kStatus) kArray3_Zero(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    kCheck(kZeroContent(obj->itemType, obj->items, kArray3_Count_(array)));

    return kOK; 
}

kFx(kStatus) kArray3_SetItem(kArray3 array, kSize index0, kSize index1, kSize index2, const void* item)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1]) && (index2 < obj->length[2])); 

    kItemImport_(kArray3_At_(array, index0, index1, index2), item, obj->itemType); 

    return kOK; 
}

kFx(kStatus) kArray3_Item(kArray3 array, kSize index0, kSize index1, kSize index2, void* item)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1]) && (index2 < obj->length[2])); 

    kItemCopy_(item, kArray3_At_(array, index0, index1, index2), obj->itemSize); 

    return kOK; 
}

kFx(kIterator) kArray3_GetIterator(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return obj->items; 
}

kFx(kBool) kArray3_HasNext(kArray3 array, kIterator iterator)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    void* end = (kByte*)obj->items + kArray3_DataSize_(array); 

    return (iterator != end); 
}

kFx(void*) kArray3_Next(kArray3 array, kIterator* iterator)
{
    kArray3Class* obj = kArray3_Cast_(array); 
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}

kFx(void*) kArray3_Data(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return kArray3_Data_(obj); 
}

kFx(kSize) kArray3_DataSize(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return kArray3_DataSize_(obj); 
}

kFx(void*) kArray3_At(kArray3 array, kSize index0, kSize index1, kSize index2)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return kArray3_At_(obj, index0, index1, index2); 
}

kFx(kType) kArray3_ItemType(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return kArray3_ItemType_(obj); 
}

kFx(kSize) kArray3_ItemSize(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return kArray3_ItemSize_(obj); 
}

kFx(kSize) kArray3_Length(kArray3 array, kSize dimension)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    kAssert(dimension < 3); 

    return kArray3_Length_(obj, dimension);  
}
 
kFx(kSize) kArray3_Count(kArray3 array)
{
    kArray3Class* obj = kArray3_Cast_(array); 

    return kArray3_Count_(obj); 
}
