/** 
 * @file    kQueue.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_QUEUE_X_H
#define K_API_QUEUE_X_H

kBeginHeader()

#define kQUEUE_MIN_CAPACITY                (16)
#define kQUEUE_GROWTH_FACTOR               (2)

typedef struct kQueueClass
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //item array
    kSize count;                //current number of elements
    kSize head;                 //array index of first item
    kSize capacity;             //maximum elements before reallocation
    kSize divisor;              //use for index calculation
    kBool isAttached;           //is array memory externally owned?
} kQueueClass;

kDeclareClass(k, kQueue, kObject) 

kFx(kStatus) kQueue_Init(kQueue queue, kType classType, kType itemType, kSize initialCapacity, kAlloc allocator); 

kFx(kStatus) kQueue_VInitClone(kQueue queue, kQueue source, kAlloc allocator); 

kFx(kStatus) kQueue_VRelease(kQueue queue); 
kFx(kStatus) kQueue_VDisposeItems(kQueue queue); 

kFx(kSize) kQueue_VSize(kQueue queue); 

kFx(kStatus) kQueue_WriteDat5V1(kQueue queue, kSerializer serializer); 
kFx(kStatus) kQueue_ReadDat5V1(kQueue queue, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kQueue_WriteDat6V0(kQueue queue, kSerializer serializer); 
kFx(kStatus) kQueue_ReadDat6V0(kQueue queue, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kQueue_Resize(kQueue queue, kSize count); 

kFx(kIterator) kQueue_GetIterator(kQueue queue); 
kFx(kBool) kQueue_HasNext(kQueue queue, kIterator iterator); 
kFx(void*) kQueue_Next(kQueue queue, kIterator* iterator); 

#define kQueue_(QUEUE)                      kCast(kQueueClass*, QUEUE)
#define kQueue_Cast_(QUEUE)                 kCastClass_(kQueue, QUEUE)

#define kxQueue_Count_(QUEUE)               (kQueue_(QUEUE)->count)
#define kxQueue_Capacity_(QUEUE)            (kQueue_(QUEUE)->capacity)
#define kQueue_Divisor_(QUEUE)              (kQueue_(QUEUE)->divisor)

#define kQueue_Head_(QUEUE)                 (kQueue_(QUEUE)->head)

#define kQueue_IsSplit_(QUEUE)              ((kQueue_Head_(QUEUE) + kQueue_Count_(QUEUE)) > kQueue_Capacity_(QUEUE))
#define kQueue_HeadCount_(QUEUE)            (kQueue_IsSplit_(QUEUE) ? (kQueue_Capacity_(QUEUE) - kQueue_Head_(QUEUE)) : kQueue_Count_(QUEUE))
#define kQueue_TailCount_(QUEUE)            (kQueue_IsSplit_(QUEUE) ? (kQueue_Head_(QUEUE) + kQueue_Count_(QUEUE) - kQueue_Capacity_(QUEUE)) : 0)

#define kxQueue_ItemType_(QUEUE)            (kQueue_(QUEUE)->itemType)
#define kxQueue_ItemSize_(QUEUE)            (kQueue_(QUEUE)->itemSize)

#define kQueue_Data_(QUEUE)                 (kQueue_(QUEUE)->items)
#define kQueue_DataSize_(QUEUE)             (kQueue_Count_(QUEUE) * kQueue_ItemSize_(QUEUE))

#define kxQueue_As_(QUEUE, INDEX, TYPE)     (*(TYPE*)kQueue_At_(QUEUE, INDEX))

#define kxQueue_At_(QUEUE, INDEX)                                                                                               \
    (((kQueue_(QUEUE)->head + (INDEX)) >= kQueue_(QUEUE)->capacity) ?                                                           \
       kItemAt_(kQueue_Data_(QUEUE), kQueue_(QUEUE)->head + (INDEX) - kQueue_(QUEUE)->capacity, kQueue_ItemSize_(QUEUE)) :      \
       kItemAt_(kQueue_Data_(QUEUE), kQueue_(QUEUE)->head + (INDEX), kQueue_ItemSize_(QUEUE)))

#define kxQueue_AddCount_(QUEUE, C)                                      \
    (((C) > (kQueue_Capacity_(QUEUE) - kQueue_Count_(QUEUE))) ?          \
        kQueue_AddCount(QUEUE, C) :                                      \
        (kQueue_(QUEUE)->count += (C), kOK))         

#define kxQueue_RemoveCount_(QUEUE, C)                                                                                  \
    (((C) > kQueue_Count_(QUEUE)) ?                                                                                     \
        kERROR_STATE :                                                                                                  \
        (kQueue_(QUEUE)->count -= (C),                                                                                  \
         kQueue_(QUEUE)->head = (kQueue_(QUEUE)->head + (C)),                                                           \
         kQueue_(QUEUE)->head -= ((kQueue_(QUEUE)->head >= kQueue_(QUEUE)->capacity) ? kQueue_(QUEUE)->capacity : 0),   \
         kOK))
        
#define kxQueue_Clear_(QUEUE)                                           \
    (kQueue_(QUEUE)->count = 0, kQueue_(QUEUE)->head = 0, kOK)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_QUEUE                      kTypeOf(kQueue)
#   define kQueue_Destroy                   kObject_Destroy
#   define kQueue_Alloc                     kQueue_Allocate
#   define kQUEUE_AT                        kQueue_At_
#   define kQUEUE_ITEM_TYPE                 kQueue_ItemType_
#   define kQUEUE_ITEM_SIZE                 kQueue_ItemSize_
#   define kQUEUE_COUNT                     kQueue_Count_
#   define kQUEUE_CAPACITY                  kQueue_Capacity_
#   define kQUEUE_DATA                      kQueue_Data_
#   define kQUEUE_COUNT_BYTES               kQueue_DataSize_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kQueue_Construct5(Q, T, C)       kQueue_Construct(Q, T, C, kNULL)
#   define kQUEUE_CAPACITY_BYTES            (kQueue_Capacity_(QUEUE) * kQueue_ItemSize_(QUEUE))

#endif

kEndHeader()

#endif
