/** 
 * @file    kMap.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kMap.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClass(k, kMap, kObject) 

    //serialization versions
    kAddVersion(kMap, "kdat6", "5.7.1.0", "kMap-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kMap, kObject, VRelease)
    kAddVMethod(kMap, kObject, VDisposeItems)
    kAddVMethod(kMap, kObject, VInitClone)
    kAddVMethod(kMap, kObject, VSize)

kEndClass() 

kFx(kStatus) kMap_Construct(kMap* map, kType keyType, kType valueType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kMap), map)); 

    if (!kSuccess(status = kMap_Init(*map, kTypeOf(kMap), keyType, valueType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, map); 
    }

    return status; 
} 

kFx(kStatus) kMap_Init(kMap map, kType classType, kType keyType, kType valueType, kSize initialCapacity, kAlloc allocator)
{
    kMapClass* obj = map; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(map, classType, allocator)); 

    obj->blocks = kNULL; 
    obj->free = kNULL; 
    obj->buckets = kNULL; 
    obj->bucketCount = 0; 
    obj->count = 0; 
    obj->capacity = 0; 
    kZero_(obj->keyField); 
    kZero_(obj->valueField); 
    obj->collectionItem = kNULL; 
    obj->hashFx = kNULL; 
    obj->equalsFx = kNULL; 
    
    kTry
    {
        kTest(kMap_Layout(map, keyType, valueType)); 
        kTest(kMap_Reserve(map, initialCapacity));       
    }
    kCatch(&status)
    {
        kMap_VRelease(map); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kMap_VInitClone(kMap map, kMap source, kAlloc allocator)
{ 
    kMapClass* obj = map; 
    kMapClass* srcObj = source; 
    kStatus status = kOK; 
    kObject keyClone = kNULL; 
    kObject valueClone = kNULL; 

    kCheck(kMap_Init(map, kObject_Type_(source), srcObj->keyField.type, srcObj->valueField.type, srcObj->count, allocator)); 

    obj->hashFx = srcObj->hashFx; 
    obj->equalsFx = srcObj->equalsFx; 

    kTry
    {
        kMapItem item = kMap_First(source); 

        while (!kIsNull(item))
        {
            const void* key = kMapItem_Key_(source, item); 
            void* value = kMapItem_Value_(source, item); 
            
            if (kMap_KeyIsRef_(source))
            {
                kTest(kObject_Clone(&keyClone, *(kObject*)key, allocator)); 
                key = &keyClone; 
            }
            if (kMap_ValueIsRef_(source))
            {
                kTest(kObject_Clone(&valueClone, *(kObject*)value, allocator)); 
                value = &valueClone; 
            }

            kTest(kMap_Add(map, key, value)); 
            keyClone = valueClone = kNULL; 

            item = kMap_Next(source, item); 
        }
    }
    kCatch(&status)
    {
        kObject_Dispose(keyClone); 
        kObject_Dispose(valueClone); 
        kMap_VDisposeItems(map); 
        kMap_VRelease(map); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kMap_VRelease(kMap map)
{
    kCheck(kMap_Deallocate(map)); 

    kCheck(kObject_VRelease_(map)); 

    return kOK; 
}

kFx(kStatus) kMap_VDisposeItems(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   

    if (!kIsNull(obj->keyField.type) && !kIsNull(obj->valueField.type))
    {
        kBool keyIsRef = kType_IsReference_(obj->keyField.type); 
        kBool valIsRef = kType_IsReference_(obj->valueField.type); 

        if (keyIsRef || valIsRef)
        {
            kMapItem item = kMap_First(map); 

            while (!kIsNull(item))
            {
                if (keyIsRef)   kCheck(kObject_Dispose(kMap_KeyAs_(map, item, kObject))); 
                if (valIsRef)   kCheck(kObject_Dispose(kMap_ValueAs_(map, item, kObject))); 

                item = kMap_Next(map, item); 
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kMap_Allocate(kMap map, kType keyType, kType valueType, kSize initialCapacity)
{
    kMapClass* obj = kMap_Cast_(map);   
    kHashFx hashFx = obj->hashFx; 
    kEqualsFx equalsFx = obj->equalsFx; 

    kCheck(kMap_Deallocate(map)); 

    obj->free = kNULL; 
    obj->bucketCount = 0; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->keyField.type = kNULL; 
    obj->valueField.type = kNULL; 
    obj->collectionItem = kNULL; 
    obj->hashFx = kNULL; 
    obj->equalsFx = kNULL; 

    kCheck(kMap_Layout(map, keyType, valueType)); 
    kCheck(kMap_Reserve(map, initialCapacity));   

    if (obj->keyField.type == keyType)
    {
        obj->hashFx = hashFx; 
        obj->equalsFx = equalsFx; 
    }

    return kOK; 
}

kFx(kStatus) kMap_Assign(kMap map, kMap source)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapClass* srcObj = kMap_Cast_(source); 
    kMapItem item  = kNULL; 

    kCheck(kMap_Allocate(map, srcObj->keyField.type, srcObj->valueField.type, srcObj->count)); 

    obj->hashFx = srcObj->hashFx; 
    obj->equalsFx = srcObj->equalsFx; 

    item = kMap_First(source); 

    while (!kIsNull(item))
    {
        kCheck(kMap_Add(map, kMapItem_Key_(source, item), kMapItem_Value_(source, item))); 
        
        item = kMap_Next(source, item); 
    }

    return kOK; 
}

kFx(kStatus) kMap_Layout(kMap map, kType keyType, kType valueType)
{
    kMapClass* obj = kMap_Cast_(map);   
    kStructField nextField, hashCodeField; 
    kStructField* fields[4];  

    nextField.type = kTypeOf(kPointer); 
    nextField.offset = offsetof(kMapItemStruct, next); 
    nextField.count = 1; 
    fields[0] = &nextField; 

    hashCodeField.type = kTypeOf(kSize); 
    hashCodeField.offset = offsetof(kMapItemStruct, hashCode); 
    hashCodeField.count = 1; 
    fields[1] = &hashCodeField; 

    obj->keyField.type = kIsNull(keyType) ? kTypeOf(kVoid) : keyType;
    obj->keyField.offset = kSIZE_NULL; 
    obj->keyField.count = 1; 
    fields[2] = &obj->keyField; 

    obj->valueField.type = kIsNull(valueType) ? kTypeOf(kVoid) : valueType;
    obj->valueField.offset = kSIZE_NULL; 
    obj->valueField.count = 1; 
    fields[3] = &obj->valueField;

    kCheck(kType_LayoutStruct(fields, kCountOf(fields), &obj->itemSize)); 
              
    return kOK; 
}

kFx(kStatus) kMap_Deallocate(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   

    while (!kIsNull(obj->blocks))
    {
        kMapItemBlock* next = obj->blocks->next; 

        kCheck(kObject_FreeMem(map, obj->blocks)); 

        obj->blocks = next; 
    }

    kCheck(kObject_FreeMemRef(map, &obj->buckets)); 

    return kOK; 
}

kFx(kSize) kMap_VSize(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   
    kSize size = sizeof(kMapClass);
    kMapItemBlock* blockObj = obj->blocks; 
    kBool keyIsRef = kType_IsReference_(obj->keyField.type); 
    kBool valIsRef = kType_IsReference_(obj->valueField.type); 

    while (!kIsNull(blockObj))
    {
        size += (kByte*)blockObj->items - (kByte*)blockObj; 
        size += obj->itemSize*blockObj->itemCount;

        blockObj = blockObj->next; 
    }

    size += sizeof(kPointer) * obj->bucketCount; 

    if (keyIsRef || valIsRef)
    {
        kMapItem item = kMap_First(map); 
        
        while (!kIsNull(item))
        {
            if (keyIsRef)   
            {
                kObject key = kMap_KeyAs_(map, item, kObject); 
                
                if (!kIsNull(key))
                {
                    size += kObject_Size_(key); 
                }
            }

            if (valIsRef)   
            {
                kObject value = kMap_ValueAs_(map, item, kObject); 
                
                if (!kIsNull(value))
                {
                    size += kObject_Size_(value); 
                }
            }

            item = kMap_Next(map, item); 
        }
    }

    return size; 
}

kFx(kStatus) kMap_WriteDat6V0(kMap map, kSerializer serializer)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItem item = kMap_First(map); 
    kTypeVersion keyVersion, valueVersion; 

    kCheck(kSerializer_WriteType_(serializer, obj->keyField.type, &keyVersion)); 
    kCheck(kSerializer_WriteType_(serializer, obj->valueField.type, &valueVersion)); 
    kCheck(kSerializer_WriteSize_(serializer, obj->count)); 

    while (!kIsNull(item))
    {
        kCheck(kSerializer_WriteItems_(serializer, obj->keyField.type, keyVersion, kMapItem_Key_(map, item), 1)); 
        kCheck(kSerializer_WriteItems_(serializer, obj->valueField.type, valueVersion, kMapItem_Value_(map, item), 1)); 

        item = kMap_Next(map, item); 
    }

    return kOK; 
}

kFx(kStatus) kMap_ReadDat6V0(kMap map, kSerializer serializer, kAlloc allocator)
{
    kMapClass *obj = map; 
    kMapItemStruct* itemObj = kNULL; 
    kType keyType, valueType; 
    kTypeVersion keyVersion, valueVersion; 
    kSize count = 0;  
    kStatus status = kOK; 
    kSize i; 

    kCheck(kSerializer_ReadType_(serializer, &keyType, &keyVersion)); 
    kCheck(kSerializer_ReadType_(serializer, &valueType, &valueVersion)); 
    kCheck(kSerializer_ReadSize_(serializer, &count)); 

    kCheck(kMap_Init(map, kTypeOf(kMap), keyType, valueType, count, allocator)); 

    kTry
    {         
        for (i = 0; i < count; ++i)
        {   
            itemObj = obj->free; 
            obj->free = itemObj->next; 
            itemObj->next = obj->buckets[0]; 
            obj->buckets[0] = itemObj; 

            itemObj->hashCode = 0; 
            kItemZero_((void*)kMapItem_Key_(map, itemObj), obj->keyField.fieldSize); 
            kItemZero_(kMapItem_Value_(map, itemObj), obj->valueField.fieldSize); 

            kTest(kSerializer_ReadItems_(serializer, obj->keyField.type, keyVersion, (void*)kMapItem_Key_(map, itemObj), 1)); 
            kTest(kSerializer_ReadItems_(serializer, obj->valueField.type, valueVersion, kMapItem_Value_(map, itemObj), 1)); 
        }

        obj->count = count; 

        kTest(kMap_Rehash(map)); 
    }
    kCatch(&status)
    {
        kMap_VDisposeItems(map); 
        kMap_VRelease(map); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kMap_Reserve(kMap map, kSize capacity)
{
    kMapClass* obj = kMap_Cast_(map);   
    
    if (capacity > obj->capacity)
    {
        kCheck(kMap_ReserveItems(map, capacity)); 
        kCheck(kMap_ReserveBuckets(map));  
    }

    return kOK; 
}

kFx(kStatus) kMap_ReserveItems(kMap map, kSize capacity)
{
    kMapClass* obj = kMap_Cast_(map);   
    kSize itemCount = kMax_(capacity - obj->capacity, kMAP_MIN_ITEMS_PER_BLOCK); 
    kSize itemOffset = kAlign_(sizeof(kMapItemBlock), kALIGN_ANY); 
    kSize blockSize = kAlign_(itemOffset + itemCount*obj->itemSize, kALIGN_ANY); 
    kMapItemBlock* block = kNULL; 
    kSize i; 
      
    kCheck(kObject_GetMem_(map, blockSize, &block)); 

    block->itemCount = itemCount; 
    block->items = (void*)((kByte*)block + itemOffset); 

    block->next = obj->blocks; 
    obj->blocks = block; 

    for (i = 0; i < itemCount; ++i)
    {
        kMapItemStruct* item = (kMapItemStruct*) ((kByte*)(&block->items[0]) + i*obj->itemSize); 

        item->next = obj->free; 
        obj->free = item; 
    }

    obj->capacity += itemCount; 

    return kOK; 
}

kFx(kStatus) kMap_ReserveBuckets(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   
    kSize minBucketCount = kMAP_CAPACITY_TO_BUCKETS(obj->capacity); 
    kSize bucketCount = 1; 
    kMapItemStruct** buckets = kNULL; 
    kMapItemStruct** tempBuckets = kNULL; 
    kSize i; 

    kCheckArgs(minBucketCount < (1U << 31)); 

    while (bucketCount < minBucketCount)
    {
        bucketCount *= 2; 
    }

    kCheck(kObject_GetMemZero_(map, bucketCount*sizeof(kPointer), &buckets)); 

    for (i = 0; i < obj->bucketCount; ++i)
    {
        kMapItemStruct* itemObj = obj->buckets[i]; 

        while (!kIsNull(itemObj))
        {
            kMapItemStruct* nextObj = itemObj->next; 
            kSize newBucketIndex = kMap_HashToIndex_(itemObj->hashCode, bucketCount); 
            
            itemObj->next = buckets[newBucketIndex]; 
            buckets[newBucketIndex] = itemObj; 
            
            itemObj = nextObj; 
        }  
    }

    tempBuckets = obj->buckets; 
    obj->buckets = buckets; 
    obj->bucketCount = bucketCount; 

    kCheck(kObject_FreeMem(map, tempBuckets)); 

    return kOK; 
}

kFx(kStatus) kMap_Rehash(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItemStruct* items = kNULL; 
    kMapItemStruct* itemObj = kNULL; 
    kSize i; 

    for (i = 0; i < obj->bucketCount; ++i)
    {
        itemObj = obj->buckets[i]; 

        while (!kIsNull(itemObj))
        {
            kMapItemStruct* nextObj = itemObj->next; 

            itemObj->hashCode = kMap_HashKey_(map, kMapItem_Key_(map, itemObj)); 
            itemObj->next = items; 
            items = itemObj; 
            
            itemObj = nextObj; 
        }  

        obj->buckets[i] = kNULL; 
    }

    itemObj = items; 

    while (!kIsNull(itemObj))
    {
        kMapItemStruct* nextObj = itemObj->next; 
        kSize bucketIndex = kMap_BucketIndex_(map, itemObj->hashCode); 

        itemObj->next = obj->buckets[bucketIndex]; 
        obj->buckets[bucketIndex] = itemObj; 

        itemObj = nextObj; 
    }  

    return kOK; 
}

kFx(kStatus) kMap_SetEqualsFx(kMap map, kEqualsFx function)
{
    kMapClass* obj = kMap_Cast_(map);   

    obj->equalsFx = function; 

    return kOK; 
}

kFx(kSize) kMap_SetHashFx(kMap map, kHashFx function)
{
    kMapClass* obj = kMap_Cast_(map);   
   
    obj->hashFx = function; 

    kCheck(kMap_Rehash(map)); 

    return kOK; 
}

kFx(kType) kMap_KeyType(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   

    return obj->keyField.type; 
}

kFx(kType) kMap_ValueType(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   

    return obj->valueField.type; 
}

kFx(kSize) kMap_Count(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   

    return obj->count; 
}

kFx(kSize) kMap_Capacity(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   

    return obj->capacity; 
}

kFx(kStatus) kMap_Find(kMap map, const void* key, void* value)
{
    kMapClass* obj = kMap_Cast_(map);   

    if (obj->bucketCount > 0)
    {
        kSize hashCode = kMap_HashKey_(map, key);
        kMapItemStruct* itemObj = obj->buckets[kMap_BucketIndex_(map, hashCode)]; 

        while (!kIsNull(itemObj))
        {
            if (kMap_KeyEquals_(map, key, kMapItem_Key_(map, itemObj)))
            {
                //if the assertion below is encountered, then the map item has been corrupted
                kAssert(hashCode == itemObj->hashCode); 

                if (!kIsNull(value))
                {
                    kItemCopy_(value, kMapItem_Value_(map, itemObj), obj->valueField.fieldSize); 
                }

                return kOK; 
            }
            itemObj = itemObj->next; 
        }
    }

    return kERROR_NOT_FOUND;    
}

kFx(kStatus) kMap_Add(kMap map, const void* key, const void* value)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = kMap_HashKey_(map, key); 
    kSize bucketIndex = 0; 
    void* keySlot = kNULL; 
    void* valueSlot = kNULL; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = kMap_BucketIndex_(map, hashCode); 
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (kMap_KeyEquals_(map, key, kMapItem_Key_(map, itemObj)))
            {
                return kERROR_ALREADY_EXISTS; 
            }
            itemObj = itemObj->next; 
        }
    }

    if (obj->count == obj->capacity)
    {
        kCheck(kMap_Reserve(map, obj->capacity+1)); 
    }

    itemObj = obj->free; 
    obj->free = itemObj->next; 
    itemObj->next = kNULL; 

    bucketIndex = kMap_BucketIndex_(map, hashCode);

    itemObj->next = obj->buckets[bucketIndex]; 
    obj->buckets[bucketIndex] = itemObj; 

    itemObj->hashCode = hashCode; 

    obj->count++; 

    keySlot = (void*)kMapItem_Key_(map, itemObj); 
    valueSlot = kMapItem_Value_(map, itemObj); 

    kItemImport_(keySlot, key, obj->keyField.type); 
    kItemImport_(valueSlot, value, obj->valueField.type); 
    
    return kOK; 
}

kFx(kStatus) kMap_Replace(kMap map, const void* key, const void* value)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = kMap_HashKey_(map, key); 
    kSize bucketIndex = 0; 
    void* keySlot = kNULL; 
    void* valueSlot = kNULL; 
    kBool found = kFALSE; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = kMap_BucketIndex_(map, hashCode);
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (kMap_KeyEquals_(map, key, kMapItem_Key_(map, itemObj)))
            {
                found = kTRUE; 
                break;
            }
            itemObj = itemObj->next; 
        }
    }

    if (!found)
    {
        if (obj->count >= obj->capacity)
        {
            kCheck(kMap_Reserve(map, obj->capacity+1)); 
        }

        itemObj = obj->free; 
        obj->free = itemObj->next; 
        itemObj->next = kNULL; 

        bucketIndex = kMap_BucketIndex_(map, hashCode);

        itemObj->next = obj->buckets[bucketIndex]; 
        obj->buckets[bucketIndex] = itemObj; 

        itemObj->hashCode = hashCode; 

        obj->count++; 
    }

    keySlot = (void*)kMapItem_Key_(map, itemObj); 
    valueSlot = kMapItem_Value_(map, itemObj); 

    kItemImport_(keySlot, key, obj->keyField.type); 
    kItemImport_(valueSlot, value, obj->valueField.type); 
    
    return kOK; 
}

kFx(kStatus) kMap_Remove(kMap map, const void* key, void* oldKey, void* oldValue)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItemStruct* previousObj = kNULL; 
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = kMap_HashKey_(map, key);  
    kSize bucketIndex = 0; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = kMap_BucketIndex_(map, hashCode); 
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (kMap_KeyEquals_(map, key, kMapItem_Key_(map, itemObj)))
            {            
                //if the assertion below is encountered, then the map item has been corrupted
                kAssert(hashCode == itemObj->hashCode); 

                if (oldKey)     kItemCopy_(oldKey, kMapItem_Key_(map, itemObj), obj->keyField.fieldSize); 
                if (oldValue)   kItemCopy_(oldValue, kMapItem_Value_(map, itemObj), obj->valueField.fieldSize); 

                if (kIsNull(previousObj))
                {
                    obj->buckets[bucketIndex] = itemObj->next; 
                }
                else
                {
                    previousObj->next = itemObj->next; 
                }

                itemObj->next = obj->free;              
                obj->free = itemObj; 

                obj->count--; 

                return kOK; 
            }

            previousObj = itemObj; 
            itemObj = itemObj->next; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kMap_Clear(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   
    kSize i; 

    if (obj->count > 0)
    {
        for (i = 0; i < obj->bucketCount; ++i)
        {
            kMapItemStruct* itemObj = obj->buckets[i]; 

            while (!kIsNull(itemObj))
            {
                kMapItemStruct* nextObj = itemObj->next; 

                itemObj->next = obj->free; 
                obj->free = itemObj; 

                itemObj = nextObj; 
            }

            obj->buckets[i] = kNULL; 
        }

        obj->count = 0; 
    }

    return kOK; 
}

kFx(kStatus) kMap_Purge(kMap map)
{
    kCheck(kObject_DisposeItems_(map)); 
    kCheck(kMap_Clear(map)); 

    return kOK; 
}

kFx(kMapItem) kMap_First(kMap map)
{
    kMapClass* obj = kMap_Cast_(map);   
    kSize i; 

    for (i = 0; i < obj->bucketCount; ++i)
    {
        if (!kIsNull(obj->buckets[i]))
        {
            return obj->buckets[i]; 
        }
    }

    return kNULL; 
}

kFx(kMapItem) kMap_Next(kMap map, kMapItem item)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItemStruct* itemObj = item; 

    if (!kIsNull(itemObj->next))
    {
        return itemObj->next; 
    }
    else
    {
        kSize hashCode = kMapItem_HashCode_(map, item); 
        kSize bucketIndex = kMap_BucketIndex_(map, hashCode); 
        kSize i; 

        for (i = bucketIndex+1; i < obj->bucketCount; ++i)
        {
            if (!kIsNull(obj->buckets[i]))
            {
                return obj->buckets[i]; 
            }
        }
    }
    
    return kNULL; 
}

kFx(kStatus) kMap_FindItem(kMap map, const void* key, kMapItem* item)
{
    kMapClass* obj = kMap_Cast_(map);   
    kSize hashCode = kMap_HashKey_(map, key); 

    if (obj->bucketCount > 0)
    {
        kMapItemStruct* itemObj = obj->buckets[kMap_BucketIndex_(map, hashCode)]; 

        while (!kIsNull(itemObj))
        {
            if (kMap_KeyEquals_(map, key, kMapItem_Key_(map, itemObj)))
            {
                //if the assertion below is encountered, then the map item has been corrupted
                kAssert(hashCode == itemObj->hashCode); 

                *item = itemObj; 
                return kOK; 
            }
            itemObj = itemObj->next; 
        }
    }

    return kERROR_NOT_FOUND;    
}

kFx(kStatus) kMap_RemoveItem(kMap map, kMapItem item)
{
    kMapClass* obj = kMap_Cast_(map);   
    kMapItemStruct* previousObj = kNULL; 
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = kMapItem_HashCode_(map, item); 
    kSize bucketIndex = 0; 
    kBool found = kFALSE; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = kMap_BucketIndex_(map, hashCode); 
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (itemObj == item)
            {
                found = kTRUE;                        

                if (kIsNull(previousObj))
                {
                    obj->buckets[bucketIndex] = itemObj->next; 
                }
                else
                {
                    previousObj->next = itemObj->next; 
                }

                itemObj->next = obj->free;              
                obj->free = itemObj; 

                obj->count--; 

                break;
            }

            previousObj = itemObj; 
            itemObj = itemObj->next; 
        }
    }

    return (found) ? kOK : kERROR_NOT_FOUND; 
}

kFx(const void*) kMap_Key(kMap map, kMapItem item)
{   
    kMapClass* obj = kMap_Cast_(map);   

    return kMapItem_Key_(obj, item); 
}

kFx(void*) kMap_Value(kMap map, kMapItem item)
{   
    kMapClass* obj = kMap_Cast_(map);   

    return kMapItem_Value_(obj, item); 
}

kFx(kStatus) kMap_SetValue(kMap map, kMapItem item, const void* value)
{
    kMapClass* obj = kMap_Cast_(map);   
    void* valueSlot = kMapItem_Value_(map, item); 

    kItemImport_(valueSlot, value, obj->valueField.type); 

    return kOK; 
}
