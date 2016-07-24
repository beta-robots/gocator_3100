/** 
 * @file    kMap.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MAP_X_H
#define K_API_MAP_X_H

kBeginHeader()

#define kMAP_CAPACITY_TO_BUCKETS(C)         (5*(C)/4)       //determines minimum buckets, assuming max load of 0.75
#define kMAP_MIN_ITEMS_PER_BLOCK            (16)            //minimum number of map items per allocated memory block

//represents a single key/value map entry
typedef struct kMapItemStruct //<K, V>
{
    struct kMapItemStruct* next;                //linked list pointer to next item
    kSize hashCode;                             //cached hash code
    //K key;                                    //key; offset/size dynamically calculated
    //V value;                                  //value; offset/size dynamically calculated
} kMapItemStruct;

//represents a block of allocated map items; multiple items
//are allocated at the same time for efficiency
typedef struct kMapItemBlock //<N>
{
    struct kMapItemBlock* next;              //linked list pointer to next memory block 
    kSize itemCount;                         //count of items in block (N)
    kMapItemStruct* items;                   //pointer to first item in block
    //kMapItemStruct item[N];                //array of map items
} kMapItemBlock; 

typedef struct kMapClass
{
    kObjectClass base;                          // base fields
    kMapItemBlock* blocks;                      // allocated memory blocks, where block contains N items
    kMapItemStruct* free;                       // linked list of free items
    kMapItemStruct** buckets;                   // array of buckets, where each bucket is a linked list of items
    kSize bucketCount;                          // count of buckets 
    kSize count;                                // current count of items
    kSize capacity;                             // maximum number of items before allocation required
    kStructField keyField;                      // key field info
    kStructField valueField;                    // value field info
    kSize itemSize;                             // size of map item, including key and value fields
    kEqualsFx equalsFx;                         // callback function for equality comparison
    kHashFx hashFx;                             // callback function for hash code generation
    void* collectionItem;                       // temp variable, required to implement kCollection interface
} kMapClass;

kDeclareClass(k, kMap, kObject) 

kFx(kStatus) kMap_Init(kMap map, kType classType, kType keyType, kType valueType, kSize initialCapacity, kAlloc allocator); 

kFx(kStatus) kMap_VInitClone(kMap map, kMap source, kAlloc allocator); 

kFx(kStatus) kMap_VRelease(kMap map); 
kFx(kStatus) kMap_VDisposeItems(kMap map); 

kFx(kSize) kMap_VSize(kObject object); 

kFx(kStatus) kMap_WriteDat6V0(kMap map, kSerializer serializer); 
kFx(kStatus) kMap_ReadDat6V0(kMap map, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kMap_Deallocate(kMap map); 

kFx(kStatus) kMap_Layout(kMap map, kType keyType, kType valueType); 

kFx(kStatus) kMap_ReserveItems(kMap map, kSize capacity);
kFx(kStatus) kMap_ReserveBuckets(kMap map);

kFx(kStatus) kMap_Rehash(kMap map);

#define kMap_(MAP)                         kCast(kMapClass*, MAP)
#define kMap_Cast_(MAP)                    kCastClass_(kMap, MAP)

#define kxMap_KeyType_(MAP)                (kMap_(MAP)->keyField.type)
#define kxMap_ValueType_(MAP)              (kMap_(MAP)->valueField.type)
#define kxMap_Count_(MAP)                  (kMap_(MAP)->count)

#define kMap_KeyIsRef_(MAP)                (kType_IsReference_(kMap_KeyType_(MAP)))
#define kMap_ValueIsRef_(MAP)              (kType_IsReference_(kMap_ValueType_(MAP)))

#define kxMap_KeyAs_(MAP, ITEM, TYPE)      (*(TYPE*)kMapItem_Key_(MAP, ITEM))
#define kxMap_ValueAs_(MAP, ITEM, TYPE)    (*(TYPE*)kMapItem_Value_(MAP, ITEM))

#define kMap_HashToIndex_(CODE, COUNT)     ((CODE) & ((COUNT) - 1))
#define kMap_BucketIndex_(MAP, CODE)       kMap_HashToIndex_(CODE, kMap_(MAP)->bucketCount)

#define kMap_HashKey_(MAP, K)                            \
    (kMap_(MAP)->hashFx ?                                \
      kMap_(MAP)->hashFx(K) :                            \
      (kMap_KeyIsRef_(MAP) ?                             \
        kObject_HashCode_(*(kObject*)(K)) :              \
        kValue_HashCode_(kMap_KeyType_(MAP), (K))))

#define kMap_KeyEquals_(MAP, K1, K2)                            \
    (kMap_(MAP)->equalsFx ?                                     \
      kMap_(MAP)->equalsFx(K1, K2) :                            \
      (kMap_KeyIsRef_(MAP) ?                                    \
        kObject_Equals_(*(kObject*)(K1), *(kObject*)(K2)) :     \
        kValue_Equals_(kMap_KeyType_(MAP), (K1), (K2))))

#define kMapItem_(ITEM)                    kCast(kMapItemStruct*, ITEM)

#define kMapItem_HashCode_(MAP, ITEM)           (kMapItem_(ITEM)->hashCode)
#define kMapItem_Key_(MAP, ITEM)                ((const void*)((kByte*)(ITEM) + kMap_(MAP)->keyField.offset))
#define kMapItem_Value_(MAP, ITEM)              ((void*)((kByte*)(ITEM) + kMap_(MAP)->valueField.offset))

kEndHeader()

#endif
