/** 
 * @file    kQueue.h
 * @brief   Declares the kQueue class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_QUEUE_H
#define K_API_QUEUE_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class       kQueue
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a FIFO queue implemented with a dynamic array.
 * 
 * The kQueue class represents a dynamic, array-based queue of objects or values. The kQueue constructor 
 * accepts a kType value that determines the type of items that will be stored in the queue. The queue will 
 * automatically grow as new items are added. 
 * 
 * @code {.c}
 * kStatus QueueExample()
 * {
 *     kQueue queue = kNULL; 
 *     k32s values[] = { 1, 2, 3, 5, 7, 9 }; 
 *     kSize i; 
 *     
 *     kTry
 *     {
 *          //create a queue that can store 32-bit integers
 *          kTest(kQueue_Construct(&queue, kTypeOf(k32s), 0, kNULL)); 
 *          
 *          //add some initial items to the queue
 *          for (i = 0; i < kCountOf(values); ++i)
 *          {
 *              kTest(kQueue_Add(queue, &values[i]); 
 *          }
 *          
 *          //print some information about the queue and its items
 *          printf("Item type: %s\n", kType_Name(kQueue_ItemType(queue))); 
 *          printf("Count: %u\n", (k32u) kQueue_Count(queue)); 
 *  
 *          for (i = 0; i < kQueue_Count(queue); ++i)
 *          {
 *              //the kQueue_As_ macro can be used to get a queue item and cast it to the desired type; 
 *              //this is equivalent to *(k32s*)kQueue_At(queue, i);
 *              k32s value = kQueue_As_(queue, i, k32s); 
 *
 *              printf("Item %u: %d\n", (k32u)i, value);  
 *          }
 *     }
 *     kFinally
 *     {
 *          kObject_Destroy(queue); 
 * 
 *          kEndFinally(); 
 *     }
 *    
 *     return kOK; 
 * }
 * 
 * @endcode
 *
 * For queues that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the queue is destroyed. To recursively destroy both the queue and the 
 * queue items, use kObject_Dispose.  
 *
 * kQueue supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kQueue supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kQueue;       --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kQueue object.
 *
 * @public                      @memberof kQueue
 * @param   queue               Receives constructed queue object. 
 * @param   itemType            Type of queue element.
 * @param   initialCapacity     Capacity initially reserved for queue items. 
 * @param   allocator           Memory allocator (or kNULL for default). 
 * @return                      Operation status. 
 */
kFx(kStatus) kQueue_Construct(kQueue* queue, kType itemType, kSize initialCapacity, kAlloc allocator); 

/** 
 * Reallocates the queue item buffer.
 *
 * @public                      @memberof kQueue
 * @param   queue               Queue object. 
 * @param   itemType            Type of queue element.
 * @param   initialCapacity     Capacity initially reserved for queue items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kQueue_Allocate(kQueue queue, kType itemType, kSize initialCapacity); 

/** 
 * Performs a shallow copy of the source queue.  
 *
 * Source items are copied by value; if the source queue contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   source      Source queue to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Assign(kQueue queue, kQueue source); 

/** 
 * Sets the count of queue items to zero. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Clear(kQueue queue); 

/** 
 * Disposes any elements in the queue and sets the count of queue items to zero.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Purge(kQueue queue); 

/** 
 * Ensures that capacity is reserved for at least the specified number of queue items. 
 *
 * Existing queue items are preserved. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   capacity    Queue capacity, in items.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Reserve(kQueue queue, kSize capacity); 

/** 
 * Adds the specified item to the end of the queue.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   item        Pointer to item that will be copied (by value) into the queue.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Add(kQueue queue, const void* item); 

/** 
 * Removes the item at the head of the queue. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   item        Destination for the removed item (copied by value, can be null).
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Remove(kQueue queue, void* item);

/** 
 * Sets the value of an item.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index. 
 * @param   item        Item that will be copied into the queue.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_SetItem(kQueue queue, kSize index, const void* item); 

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index.  
 * @param   item        Destination for item that will be copied (by value) from the queue. 
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Item(kQueue queue, kSize index, void* item);  

/** 
 * Increases the queue count by the specified amount.
 *
 * Increases queue capacity if necessary; existing queue items are preserved. 
 * New items are not initialized. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   count       Amount to add to the existing count.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_AddCount(kQueue queue, kSize count); 

/** 
 * Decreases the queue count by the specified amount.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   count       Amount to remove from the existing count.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_RemoveCount(kQueue queue, kSize count); 

/** 
 * Returns a pointer to the specified item in the queue buffer. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index.  
 * @return              Pointer to queue element.
 */
kFx(void*) kQueue_At(kQueue queue, kSize index);

/** 
 * Returns the queue element type. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Item type. 
 */
kFx(kType) kQueue_ItemType(kQueue queue); 

/** 
 * Returns the queue element size.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Item size, in bytes. 
 */
kFx(kSize) kQueue_ItemSize(kQueue queue); 

/** 
 * Returns the current count of items in the queue. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Current count of items. 
 */
kFx(kSize) kQueue_Count(kQueue queue); 

/** 
 * Returns the number of elements for which space has been allocated.  
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Queue capacity. 
 */
kFx(kSize) kQueue_Capacity(kQueue queue); 

/** @relates kQueue @{ */

#define kQueue_AddCount_(QUEUE, C)      kxQueue_AddCount_(QUEUE, C)             ///< Macro version of kQueue_AddCount.  
#define kQueue_RemoveCount_(QUEUE, C)   kxQueue_RemoveCount_(QUEUE, C)          ///< Macro version of kQueue_RemoveCount.  
#define kQueue_Clear_(QUEUE)            kxQueue_Clear_(QUEUE)                   ///< Macro version of kQueue_Clear.  
#define kQueue_ItemType_(QUEUE)         kxQueue_ItemType_(QUEUE)                ///< Macro version of kQueue_ItemType.   
#define kQueue_ItemSize_(QUEUE)         kxQueue_ItemSize_(QUEUE)                ///< Macro version of kQueue_ItemSize.  
#define kQueue_Count_(QUEUE)            kxQueue_Count_(QUEUE)                   ///< Macro version of kQueue_Count.  
#define kQueue_Capacity_(QUEUE)         kxQueue_Capacity_(QUEUE)                ///< Macro version of kQueue_Capacity.  
#define kQueue_At_(QUEUE, INDEX)        kxQueue_At_(QUEUE, INDEX)               ///< Macro version of kQueue_At.  

/** Accesses a queue element at the specified index, and casts the value to the specified type.  */
#define kQueue_As_(QUEUE, INDEX, TYPE)  kxQueue_As_(QUEUE, INDEX, TYPE) 

/** @} */

kEndHeader()

#include <kApi/Data/kQueue.x.h>

#endif
