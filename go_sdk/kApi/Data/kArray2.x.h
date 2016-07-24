/** 
 * @file    kArray2.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_2_X_H
#define K_API_ARRAY_2_X_H

kBeginHeader()

typedef struct kArray2Class
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //array memory 
    kSize length[2];            //array length per dimension, in items
    kBool isAttached;           //is array memory externally owned?
} kArray2Class;
    
kDeclareClass(k, kArray2, kObject) 

kFx(kStatus) kArray2_Init(kArray2 array, kType classType, kType itemType, kSize length0, kSize length1, kAlloc allocator);

kFx(kStatus) kArray2_VInitClone(kArray2 array, kArray2 source, kAlloc allocator); 

kFx(kStatus) kArray2_VRelease(kArray2 array); 
kFx(kStatus) kArray2_VDisposeItems(kArray2 array); 

kFx(kSize) kArray2_VSize(kArray2 array); 

kFx(kStatus) kArray2_WriteDat5V1(kArray2 array, kSerializer serializer); 
kFx(kStatus) kArray2_ReadDat5V1(kArray2 array, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArray2_WriteDat6V0(kArray2 array, kSerializer serializer); 
kFx(kStatus) kArray2_ReadDat6V0(kArray2 array, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArray2_Realloc(kArray2 array, kSize length0, kSize length1);

kFx(kIterator) kArray2_GetIterator(kArray2 array); 
kFx(kBool) kArray2_HasNext(kArray2 array, kIterator iterator); 
kFx(void*) kArray2_Next(kArray2 array, kIterator* iterator); 

#define kArray2_(ARRAY)                         kCast(kArray2Class*, ARRAY)
#define kArray2_Cast_(ARRAY)                    kCastClass_(kArray2, ARRAY)

#define kxArray2_Length_(ARRAY, DIM)            (kArray2_(ARRAY)->length[DIM])
#define kxArray2_Count_(ARRAY)                  (kArray2_Length_(ARRAY, 0) *  kArray2_Length_(ARRAY, 1))

#define kxArray2_ItemType_(ARRAY)               (kArray2_(ARRAY)->itemType)
#define kxArray2_ItemSize_(ARRAY)               (kArray2_(ARRAY)->itemSize)

#define kxArray2_Data_(ARRAY)                   (kArray2_(ARRAY)->items)
#define kxArray2_At_(ARRAY, I0, I1)             (kItemAt_(kArray2_Data_(ARRAY), (I0)*kArray2_Length_(ARRAY, 1) + (I1), kArray2_ItemSize_(ARRAY)))
#define kxArray2_As_(ARRAY, I0, I1, TYPE)       (*(TYPE*)kArray2_At_(ARRAY, I0, I1))

#define kxArray2_DataSize_(ARRAY)               (kArray2_Count_(ARRAY) * kArray2_ItemSize_(ARRAY))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_ARRAY2                 kTypeOf(kArray2)
#   define kArray2_Destroy              kObject_Destroy
#   define kArray2_Alloc                kArray2_Allocate
#   define kARRAY2_DATA                 kArray2_Data_
#   define kARRAY2_AT                   kArray2_At_
#   define kARRAY2_ITEM_TYPE            kArray2_ItemType_
#   define kARRAY2_ITEM_SIZE            kArray2_ItemSize_
#   define kARRAY2_LENGTH               kArray2_Length_
#   define kARRAY2_COUNT                kArray2_Count_
#   define kARRAY2_BEGIN                kArray2_Data_
#   define kARRAY2_COUNT_BYTES          kArray2_DataSize_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kArray2_Construct5(A, T, L0, L1)         kArray2_Construct(A, T, L0, L1, kNULL)
#   define kARRAY2_END(A)                           kAt_(kArray2_Data_(A), kArray2_DataSize_(A))

#endif

kEndHeader()

#endif
