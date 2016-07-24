/** 
 * @file    kArray1.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_1_X_H
#define K_API_ARRAY_1_X_H

kBeginHeader()

typedef struct kArray1Class
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //array memory 
    kSize length;               //array length, in items
    kBool isAttached;           //is array memory externally owned?
} kArray1Class;
    
kDeclareClass(k, kArray1, kObject) 

kFx(kStatus) kArray1_Init(kArray1 array, kType classType, kType itemType, kSize length, kAlloc allocator);

kFx(kStatus) kArray1_VInitClone(kArray1 array, kArray1 source, kAlloc allocator); 

kFx(kStatus) kArray1_VRelease(kArray1 array); 
kFx(kStatus) kArray1_VDisposeItems(kArray1 array); 

kFx(kSize) kArray1_VSize(kArray1 array); 

kFx(kStatus) kArray1_WriteDat5V1(kArray1 array, kSerializer serializer); 
kFx(kStatus) kArray1_ReadDat5V1(kArray1 array, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArray1_WriteDat6V0(kArray1 array, kSerializer serializer); 
kFx(kStatus) kArray1_ReadDat6V0(kArray1 array, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArray1_Realloc(kArray1 array, kSize length); 

kFx(kIterator) kArray1_GetIterator(kArray1 array); 
kFx(kBool) kArray1_HasNext(kArray1 array, kIterator iterator); 
kFx(void*) kArray1_Next(kArray1 array, kIterator* iterator); 

#define kArray1_(ARRAY)                         kCast(kArray1Class*, ARRAY)
#define kArray1_Cast_(ARRAY)                    kCastClass_(kArray1, ARRAY)

#define kxArray1_Length_(ARRAY)                 (kArray1_(ARRAY)->length)
#define kxArray1_Count_(ARRAY)                  (kArray1_(ARRAY)->length)

#define kxArray1_ItemType_(ARRAY)               (kArray1_(ARRAY)->itemType)
#define kxArray1_ItemSize_(ARRAY)               (kArray1_(ARRAY)->itemSize)

#define kxArray1_Data_(ARRAY)                   (kArray1_(ARRAY)->items)
#define kxArray1_At_(ARRAY, INDEX)              (kItemAt_(kArray1_Data_(ARRAY), (INDEX), kArray1_ItemSize_(ARRAY)))
#define kxArray1_As_(ARRAY, INDEX, TYPE)        (*(TYPE*)kArray1_At_(ARRAY, INDEX))

#define kxArray1_DataSize_(ARRAY)               (kArray1_Count_(ARRAY) * kArray1_ItemSize_(ARRAY))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_ARRAY1                     kTypeOf(kArray1)
#   define kArray1_Destroy                  kObject_Destroy
#   define kArray1_Alloc                    kArray1_Allocate
#   define kARRAY1_DATA                     kArray1_Data_
#   define kARRAY1_AT                       kArray1_At_
#   define kARRAY1_ITEM_TYPE                kArray1_ItemType_
#   define kARRAY1_ITEM_SIZE                kArray1_ItemSize_
#   define kARRAY1_LENGTH                   kArray1_Length_
#   define kARRAY1_COUNT                    kArray1_Count_
#   define kARRAY1_BEGIN                    kArray1_Data_
#   define kARRAY1_COUNT_BYTES              kArray1_DataSize_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kArray1_Construct5(A, T, L)      kArray1_Construct(A, T, L, kNULL)
#   define kARRAY1_END(A)                   kAt_(kArray1_Data_(A), kArray1_DataSize_(A))

#endif

kEndHeader()

#endif
