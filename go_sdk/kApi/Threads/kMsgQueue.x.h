/** 
 * @file    kMsgQueue.x.h
 *
 * @internal
 * Copyright (C) 2011-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MSG_QUEUE_X_H
#define K_API_MSG_QUEUE_X_H

kBeginHeader()

#define kMSG_QUEUE_GROWTH_FACTOR        (2)
#define kMSG_QUEUE_MIN_CAPACITY         (16)

typedef void* kMsgQueueEntry; 

typedef struct kMsgQueueEntryStruct//<T>
{
    kSize size;                       //cached item size (for ref types, kObject_Size result)
    //T item;                         //queue item
} kMsgQueueEntryStruct; 

typedef struct kMsgQueueClass
{
    kObjectClass base; 
    void* entries;                      //data buffer
    kSize entrySize;                    //size of kMsgQueueEntry<T> instance
    kStructField itemField;             //information about generic item field
    kSize entryCapacity;                //maximum count of items before reallocation
    kSize entryDivisor;                 //use for index calculation 
    kSize count;                        //current count of items 
    kSize pruneCount;                   //current count of pruned items 
    kSize size;                         //total size of current items 
    kSize maxCount;                     //maximum count of items 
    kSize maxSize;                      //maximum total size of items 
    kSize head;                         //current read index 
    kSemaphore canRemove;               //counts available items 
    kLock lock;                         //mutual exclusion lock 
    kCallback onDrop;                   //drop handler
    k64u dropCount;                     //number of items dropped 
} kMsgQueueClass;

kDeclareClass(k, kMsgQueue, kObject)

kFx(kStatus) kMsgQueue_Init(kMsgQueue queue, kType type, kType itemType, kAlloc allocator); 
kFx(kStatus) kMsgQueue_VRelease(kMsgQueue queue); 
kFx(kStatus) kMsgQueue_VDisposeItems(kMsgQueue queue); 
kFx(kSize) kMsgQueue_VSize(kMsgQueue queue); 

kFx(kBool) kMsgQueue_ExceedsCapacity(kMsgQueue queue); 

kFx(kStatus) kMsgQueue_Layout(kMsgQueue queue, kType itemType); 

kFx(kStatus) kMsgQueue_Prune(kMsgQueue queue); 

#define kMsgQueue_(M)                   (kCast(kMsgQueueClass*, M))
#define kMsgQueue_Cast_(M)              (kCastClass_(kMsgQueue, M))

#define kMsgQueue_DropHandler_(M)       (kMsgQueue_(M)->onDrop)
#define kMsgQueue_ItemType_(M)          (kMsgQueue_(M)->itemField.type)

#define kMsgQueue_IsSplit_(M)           ((kMsgQueue_(M)->head + kMsgQueue_(M)->count) > kMsgQueue_(M)->entryCapacity)
#define kMsgQueue_HeadCount_(M)         (kMsgQueue_IsSplit_(M) ? (kMsgQueue_(M)->entryCapacity - kMsgQueue_(M)->head) : kMsgQueue_(M)->count)
#define kMsgQueue_TailCount_(M)         (kMsgQueue_IsSplit_(M) ? (kMsgQueue_(M)->head + kMsgQueue_(M)->count - kMsgQueue_(M)->entryCapacity) : 0)

#define kMsgQueue_ItemIsRef_(M)         kType_IsReference_(kMsgQueue_(M)->itemField.type)

#define kMsgQueue_EntryAt_(M, I)        kItemAt_(kMsgQueue_(M)->entries, (kMsgQueue_(M)->head + (I)) % kMsgQueue_(M)->entryDivisor, kMsgQueue_(M)->entrySize)

#define kMsgQueueEntry_Item_(M, E)      kAt_(E, kMsgQueue_(M)->itemField.offset)

//Deprecated
typedef k32s kMsgQueueCapacityType; 

//Deprecated
#define kMSG_QUEUE_CAPACITY_TYPE_NONE            (0)        ///< Unlimited capacity.
#define kMSG_QUEUE_CAPACITY_TYPE_COUNT           (1)        ///< Item count is limited. 
#define kMSG_QUEUE_CAPACITY_TYPE_MEMORY          (2)        ///< Total data size is limited (bytes).

//Deprecated
kFx(kStatus) kMsgQueue_SetCapacity(kMsgQueue queue, kMsgQueueCapacityType capacityType, kSize capacity);

//Deprecated
kFx(kSize) kMsgQueue_Capacity(kMsgQueue queue);

kEndHeader()

#endif
