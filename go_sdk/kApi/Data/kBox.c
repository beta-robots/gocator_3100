/** 
 * @file    kBox.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kBox.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kBox, kObject) 
    
    //serialization versions
    kAddVersion(kBox, "kdat5", "5.0.0.0", "29-1", WriteDat5V1, ReadDat5V1)
    kAddVersion(kBox, "kdat6", "5.7.1.0", "kBox-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kBox, kObject, VRelease)
    kAddVMethod(kBox, kObject, VInitClone)
    kAddVMethod(kBox, kObject, VSize)

kEndClass() 

kFx(kStatus) kBox_Construct(kBox* box, kType itemType, kObject allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kBox), box)); 

    if (!kSuccess(status = kBox_Init(*box, kTypeOf(kBox), itemType, alloc)))
    {
        kAlloc_FreeRef(alloc, box); 
    }

    return status; 
} 

kFx(kStatus) kBox_Init(kBox box, kType classType, kType itemType, kAlloc allocator)
{
    kBoxClass* obj = box; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(box, classType, allocator)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 
    obj->allocSize = 0; 
    obj->item = kNULL;

    kTry
    {
        kTest(kBox_Realloc(box)); 
    }
    kCatch(&status)
    {
        kBox_VRelease(box); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kBox_VInitClone(kBox box, kBox source, kAlloc allocator)
{
    kBoxClass* obj = box; 
    kBoxClass* srcObj = source; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(box, kObject_Type_(source), allocator));     

    obj->itemType = srcObj->itemType; 
    obj->itemSize = srcObj->itemSize; 
    obj->allocSize = 0; 
    obj->item = kNULL; 

    kTry
    {
        kTest(kBox_Realloc(box)); 

        kItemCopy_(obj->item, srcObj->item, obj->itemSize); 
    }
    kCatch(&status)
    {
        kBox_VRelease(box); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kBox_VRelease(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 

    kCheck(kObject_FreeMem_(box, obj->item)); 
    
    kCheck(kObject_VRelease_(box)); 

    return kOK; 
}

kFx(kSize) kBox_VSize(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 
    kSize size = sizeof(kBoxClass) + obj->allocSize; 

    return size; 
}

kFx(kStatus) kBox_WriteDat5V1(kBox box, kSerializer serializer)
{
    kBoxClass* obj = kBox_Cast_(box); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize_(serializer, 1)); 
    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems_(serializer, obj->itemType, itemVersion, obj->item, 1)); 

    return kOK; 
}

kFx(kStatus) kBox_ReadDat5V1(kBox box, kSerializer serializer, kAlloc allocator)
{
    kBoxClass *obj = box; 
    kSize length; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &length));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kBox_Init(box, kTypeOf(kBox), itemType, allocator)); 

    if (!kSuccess(status = kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->item, 1)))
    {
        kBox_VRelease(box); 
        return status;
    }

    return kOK; 
}

kFx(kStatus) kBox_WriteDat6V0(kBox box, kSerializer serializer)
{
    kBoxClass* obj = kBox_Cast_(box); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->item, 1)); 

    return kOK; 
}

kFx(kStatus) kBox_ReadDat6V0(kBox box, kSerializer serializer, kAlloc allocator)
{
    kBoxClass *obj = box; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kStatus status; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));

    kCheck(kBox_Init(box, kTypeOf(kBox), itemType, allocator)); 

    kTry
    {
        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->item, 1)); 
    }
    kCatch(&status)
    {
        kBox_VRelease(box); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kBox_Realloc(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 
    kSize newSize = kMax_(obj->allocSize, obj->itemSize); 

    if (newSize > obj->allocSize)
    {
        void* oldItem = obj->item; 
        void* newItem = kNULL; 

        kCheck(kObject_GetMem_(box, newSize, &newItem));         
        
        obj->item = newItem; 
        obj->allocSize = newSize; 

        kCheck(kObject_FreeMem_(box, oldItem)); 
    }
    
    return kOK; 
}

kFx(kStatus) kBox_Allocate(kBox box, kType itemType)
{
    kBoxClass* obj = kBox_Cast_(box); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size_(obj->itemType); 

    kCheck(kBox_Realloc(box)); 

    return kOK; 
}

kFx(kStatus) kBox_Assign(kBox box, kBox source)
{
    kBoxClass* obj = kBox_Cast_(box); 
    kBoxClass* srcObj = kBox_Cast_(source); 

    kCheck(kBox_Allocate(box, srcObj->itemType)); 

    kItemCopy_(obj->item, srcObj->item, obj->itemSize); 

    return kOK;   
}

kFx(kStatus) kBox_Zero(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 

    kItemZero_(obj->item, obj->itemSize); 

    return kOK; 
}

kFx(kStatus) kBox_SetItem(kBox box, const void* item)
{
    kBoxClass* obj = kBox_Cast_(box); 

    kItemImport_(obj->item, item, obj->itemType); 

    return kOK; 
}

kFx(kStatus) kBox_Item(kBox box, void* item)
{
    kBoxClass* obj = kBox_Cast_(box); 

    kItemCopy_(item, obj->item, obj->itemSize); 

    return kOK; 
}

kFx(void*) kBox_Data(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 

    return kBox_Data_(obj); 
}

kFx(kType) kBox_ItemType(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 

    return kBox_ItemType_(obj); 
}

kFx(kSize) kBox_ItemSize(kBox box)
{
    kBoxClass* obj = kBox_Cast_(box); 

    return kBox_ItemSize_(obj); 
}
