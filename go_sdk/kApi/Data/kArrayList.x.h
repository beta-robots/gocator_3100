/** 
 * @file    kArrayList.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_LIST_X_H
#define K_API_ARRAY_LIST_X_H

kBeginHeader()

#define kARRAY_LIST_MIN_CAPACITY                (16)        ///< Minimum number of allocated elements.
#define kARRAY_LIST_GROWTH_FACTOR               (2)         ///< Factor by which capacity is increased upon exhaustion.

typedef struct kArrayListClass
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //item array
    kSize count;                //current number of elements
    kSize capacity;             //maximum elements before reallocation
    kBool isAttached;           //is array memory externally owned?
} kArrayListClass;

kDeclareClass(k, kArrayList, kObject) 

kFx(kStatus) kArrayList_Init(kArrayList list, kType classType, kType itemType, kSize initialCapacity, kAlloc allocator); 

kFx(kStatus) kArrayList_VInitClone(kArrayList list, kArrayList source, kAlloc allocator); 

kFx(kStatus) kArrayList_VRelease(kArrayList list); 
kFx(kStatus) kArrayList_VDisposeItems(kArrayList list); 

kFx(kSize) kArrayList_VSize(kArrayList list); 

kFx(kStatus) kArrayList_WriteDat5V1(kArrayList list, kSerializer serializer); 
kFx(kStatus) kArrayList_ReadDat5V1(kArrayList list, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kArrayList_WriteDat6V0(kArrayList list, kSerializer serializer); 
kFx(kStatus) kArrayList_ReadDat6V0(kArrayList list, kSerializer serializer, kAlloc allocator); 

kFx(kIterator) kArrayList_GetIterator(kArrayList list); 
kFx(kBool) kArrayList_HasNext(kArrayList list, kIterator iterator); 
kFx(void*) kArrayList_Next(kArrayList list, kIterator* iterator); 

#define kArrayList_(LIST)                           kCast(kArrayListClass*, LIST)
#define kArrayList_Cast_(LIST)                      kCastClass_(kArrayList, LIST)

#define kArrayList_Delete                           kArrayList_Remove

#define kxArrayList_Count_(LIST)                    (kArrayList_(LIST)->count)
#define kxArrayList_Capacity_(LIST)                 (kArrayList_(LIST)->capacity)

#define kxArrayList_ItemSize_(LIST)                 (kArrayList_(LIST)->itemSize)
#define kxArrayList_ItemType_(LIST)                 (kArrayList_(LIST)->itemType)
#define kArrayList_ItemIsRef_(LIST)                 (kType_IsReference_(kArrayList_ItemType_(LIST)))

#define kxArrayList_Data_(LIST)                     (kArrayList_(LIST)->items)
#define kxArrayList_DataSize_(LIST)                 (kArrayList_Count_(LIST) * kArrayList_ItemSize_(LIST))

#define kxArrayList_At_(LIST, INDEX)                (kItemAt_(kArrayList_Data_(LIST), (INDEX), kArrayList_ItemSize_(LIST)))
#define kxArrayList_As_(LIST, INDEX, TYPE)          (kAs_(kArrayList_At_(LIST, INDEX), TYPE))

#define kxArrayList_First_(LIST)                    (kArrayList_Data_(LIST))
#define kxArrayList_Last_(LIST)                     (kArrayList_At_(LIST, kArrayList_Count_(LIST)-1))

#define kxArrayList_Begin_(LIST)                    (kArrayList_Data_(LIST))
#define kxArrayList_End_(LIST)                      (kArrayList_At_(LIST, kArrayList_Count_(LIST)))

#define kxArrayList_RBegin_(LIST)                   (kArrayList_Last(LIST))
#define kxArrayList_REnd_(LIST)                     (kAt_(kArrayList_Data_(LIST), -1*(kSSize)kArrayList_ItemSize_(LIST)))

#define kxArrayList_AddCount_(LIST, C)                                      \
    (((C) > (kArrayList_Capacity_(LIST) - kArrayList_Count_(LIST))) ?       \
        kArrayList_AddCount(LIST, C) :                                      \
        (kArrayList_(LIST)->count += (C), kOK))         

#define kxArrayList_RemoveCount_(LIST, C)                                   \
    (((C) > kArrayList_Count_(LIST)) ?                                      \
        kERROR_STATE :                                                      \
        (kArrayList_(LIST)->count -= (C), kOK))         
        
#define kxArrayList_Clear_(LIST)                                            \
    (kArrayList_(LIST)->count = 0, kOK)


/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_ARRAY_LIST             kTypeOf(kArrayList)
#   define kArrayList_Destroy           kObject_Destroy
#   define kArrayList_Alloc             kArrayList_Allocate
#   define kARRAY_LIST_DATA             kArrayList_Data_
#   define kARRAY_LIST_AT               kArrayList_At_
#   define kARRAY_LIST_ITEM_TYPE        kArrayList_ItemType_
#   define kARRAY_LIST_ITEM_SIZE        kArrayList_ItemSize_
#   define kARRAY_LIST_COUNT            kArrayList_Count_
#   define kARRAY_LIST_CAPACITY         kArrayList_Capacity_
#   define kARRAY_LIST_RESIZE           kArrayList_Resize
#   define kARRAY_LIST_CLEAR            kArrayList_Clear_
#   define kARRAY_LIST_BEGIN            kArrayList_Data_
#   define kARRAY_LIST_COUNT_BYTES      kArrayList_DataSize_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kArrayList_Construct5(A, T, C)       kArrayList_Construct(A, T, C, kNULL)
#   define kARRAY_LIST_END(L)                   kArrayList_At_(L, kArrayList_Count_(L))
#   define kARRAY_LIST_CAPACITY_BYTES(L)        (kArrayList_Capacity_(L) * kArrayList_ItemSize_(L))

#endif

kEndHeader()

#endif
