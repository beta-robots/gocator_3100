/** 
 * @file    kArray3.x.h
 *
 * @internal
 * Copyright (C) 2006-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_3_X_H
#define K_API_ARRAY_3_X_H

kBeginHeader()

typedef struct kArray3Class
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //array memory 
    kSize length[3];            //array length per dimension, in items
    kBool isAttached;           //is array memory externally owned?
} kArray3Class;
    
kDeclareClass(k, kArray3, kObject) 

kFx(kStatus) kArray3_Init(kArray3 array, kType classType, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator);

kFx(kStatus) kArray3_VInitClone(kArray3 array, kArray3 source, kAlloc allocator); 

kFx(kStatus) kArray3_VRelease(kArray3 array); 
kFx(kStatus) kArray3_VDisposeItems(kArray3 array); 

kFx(kSize) kArray3_VSize(kArray3 array); 

kFx(kStatus) kArray3_WriteDat5V0(kArray3 array, kSerializer serializer); 
kFx(kStatus) kArray3_ReadDat5V0(kArray3 array, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArray3_WriteDat6V0(kArray3 array, kSerializer serializer); 
kFx(kStatus) kArray3_ReadDat6V0(kArray3 array, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArray3_Realloc(kArray3 array, kSize length0, kSize length1, kSize length2);

kFx(kIterator) kArray3_GetIterator(kArray3 array); 
kFx(kBool) kArray3_HasNext(kArray3 array, kIterator iterator); 
kFx(void*) kArray3_Next(kArray3 array, kIterator* iterator); 

#define kArray3_(ARRAY)                         kCast(kArray3Class*, ARRAY)
#define kArray3_Cast_(ARRAY)                    kCastClass_(kArray3, ARRAY)

#define kxArray3_Length_(ARRAY, DIM)            (kArray3_(ARRAY)->length[DIM])
#define kxArray3_Count_(ARRAY)                  (kArray3_Length_(ARRAY, 0) *  kArray3_Length_(ARRAY, 1) * kArray3_Length_(ARRAY, 2))

#define kxArray3_ItemType_(ARRAY)               (kArray3_(ARRAY)->itemType)
#define kxArray3_ItemSize_(ARRAY)               (kArray3_(ARRAY)->itemSize)

#define kxArray3_Data_(ARRAY)                   (kArray3_(ARRAY)->items)

#define kxArray3_At_(ARRAY, I0, I1, I2)                             \
    (kItemAt_(kArray3_Data_(ARRAY),                                 \
       (I0)*kArray3_Length_(ARRAY, 1)*kArray3_Length_(ARRAY, 2) +   \
       (I1)*kArray3_Length_(ARRAY, 2) + (I2),                       \
       kArray3_ItemSize_(ARRAY)))

#define kxArray3_As_(ARRAY, I0, I1, I2, TYPE)   (*(TYPE*)kArray3_At_(ARRAY, I0, I1, I2))

#define kxArray3_DataSize_(ARRAY)               (kArray3_Count_(ARRAY) * kArray3_ItemSize_(ARRAY))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_ARRAY3             kTypeOf(kArray3)
#   define kArray3_Destroy          kObject_Destroy
#   define kArray3_Alloc            kArray3_Allocate
#   define kARRAY3_DATA             kArray3_Data_
#   define kARRAY3_AT               kArray3_At_
#   define kARRAY3_ITEM_TYPE        kArray3_ItemType_
#   define kARRAY3_ITEM_SIZE        kArray3_ItemSize_
#   define kARRAY3_LENGTH           kArray3_Length_
#   define kARRAY3_COUNT            kArray3_Count_
#   define kARRAY3_BEGIN            kArray3_Data_
#   define kARRAY3_COUNT_BYTES      kArray3_DataSize_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kArray3_Construct5(A, T, L0, L1, L2)      kArray3_Construct(A, T, L0, L1, L2, kNULL)
#   define kARRAY3_END(A)                            kAt_(kArray3_Data_(A), kArray3_DataSize_(A))

#endif

kEndHeader()

#endif
