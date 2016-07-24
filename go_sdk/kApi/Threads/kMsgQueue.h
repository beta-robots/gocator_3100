/** 
 * @file    kMsgQueue.h
 * @brief   Declares the kMsgQueue class. 
 *
 * @internal
 * Copyright (C) 2011-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MSG_QUEUE_H
#define K_API_MSG_QUEUE_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @struct  kMsgQueueDropArgs
 * @ingroup kApi-Threads
 * @brief   Represents arguments passed in a kMsgQueue drop callback. 
 */
typedef struct kMsgQueueDropArgs
{
    void* item;       ///< Pointer to the item to be dropped. 
} kMsgQueueDropArgs; 

/**
 * @class   kMsgQueue
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a synchronized FIFO queue with an optional maximum size or count capacity. 
 */
//typedef kObject kMsgQueue;          // --forward-declared in kApiDef.x.h 

/** Defines the signature of a callback function to handle dropped items. */
typedef kStatus (kCall *kMsgQueueDropFx) (kPointer receiver, kMsgQueue queue, kMsgQueueDropArgs* args); 

/** 
 * Constructs a kMsgQueue object. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Receives a handle to the constructed object. 
 * @param   itemType    Type of list element (must be a reference type). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Construct(kMsgQueue* queue, kType itemType, kAlloc allocator); 

/** 
 * Adjusts the maximum amount of data retained by the queue. 
 *
 * @public                 @memberof kMsgQueue
 * @param   queue          Queue object. 
 * @param   size           Maximum total recursive size of all data items in queue, in bytes.
 * @return                 Operation status. 
 */
kFx(kStatus) kMsgQueue_SetMaxSize(kMsgQueue queue, kSize size);

/** 
 * Adjusts the maximum count of items retained by the queue. 
 *
 * @public                 @memberof kMsgQueue
 * @param   queue          Queue object. 
 * @param   count          Maximum total count of all data items in queue.
 * @return                 Operation status. 
 */
kFx(kStatus) kMsgQueue_SetMaxCount(kMsgQueue queue, kSize count);

/** 
 * Reserves memory for the specified number of items.
 *
 * @public                 @memberof kMsgQueue
 * @param   queue          Queue object. 
 * @param   count          Count of items for which to reserve capacity.
 * @return                 Operation status. 
 */
kFx(kStatus) kMsgQueue_Reserve(kMsgQueue queue, kSize count);

/** 
 * Sets the callback used when dropping an item.
 *
 * If a handler is not set, and the queue contains objects, then dropped objects are passed 
 * to kObject_Dispose.
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   onDrop      Callback function.
 * @param   receiver    Callback context. 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_SetDropHandler(kMsgQueue queue, kMsgQueueDropFx onDrop, kPointer receiver);

/** 
 * Adds an item to the queue. 
 * 
 * If queue capacity is exceeded, the oldest item in the queue will be removed. If a drop handler is 
 * installed, the handler will be called; otherwise, if the queue is an object container, then 
 * kObject_Dispose will be used to dispose the item. 
 * 
 * If the operation status returned by this function indicates an error, it is the responsibility 
 * of the caller to dispose the item (if appropriate). 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   item        Item to be added. 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Add(kMsgQueue queue, void* item);

/** 
 * Removes an item from the queue. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   item        Receives removed item (can be kNULL). 
 * @param   timeout     Timeout (microseconds).  
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Remove(kMsgQueue queue, void* item, k64u timeout);

/** 
 * Removes all items from the queue.  
 *
 * This method does not call the drop handler when removing items.  
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Clear(kMsgQueue queue);

/** 
 * Disposes any elements in the queue and sets the count of queue items to zero.
 * 
 * This method does not call the drop handler when removing items.  
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Purge(kMsgQueue queue);

/** 
 * Reports the current count of queue items. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Count of queue items. 
 */
kFx(kSize) kMsgQueue_Count(kMsgQueue queue);

/** 
 * Reports the maximum total data size of all items in the queue. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Maximum total recursive data size, in bytes.
 */
kFx(kSize) kMsgQueue_MaxSize(kMsgQueue queue);

/** 
 * Reports the maximum count of items in the queue. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Maximum count of items.
 */
kFx(kSize) kMsgQueue_MaxCount(kMsgQueue queue);

/** 
 * Reports the type of element stored in the queue. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Capacity of queue (bytes). 
 */
kFx(kType) kMsgQueue_ItemType(kMsgQueue queue);

/**
* Returns the queue element size.
*
* @public              @memberof kMsgQueue
* @param   queue       Queue object.
* @return              Item size, in bytes.
*/
kFx(kSize) kMsgQueue_ItemSize(kQueue queue);

/** 
 * Reports the current amount of data stored in the queue (in bytes).
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Size of queue (bytes). 
 */
kFx(kSize) kMsgQueue_DataSize(kMsgQueue queue);

/** 
 * Reports the count of dropped items. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Count of dropped items. 
 */
kFx(k64u) kMsgQueue_DropCount(kMsgQueue queue);

kEndHeader()

#include <kApi/Threads/kMsgQueue.x.h>

#endif
