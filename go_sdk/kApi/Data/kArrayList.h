/** 
 * @file    kArrayList.h
 * @brief   Declares the kArrayList class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_LIST_H
#define K_API_ARRAY_LIST_H

#include <kApi/kApiDef.h> 

kBeginHeader()

/**
 * @class       kArrayList
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a list implemented with a dynamic array.
 * 
 * The kArrayList class represents a dynamic, array-based list of objects or values. The kArrayList constructor 
 * accepts a kType value that determines the type of items that will be stored in the list. The list will 
 * automatically grow as new items are added. 
 * 
 * @code {.c}
 * kStatus ArrayListExample()
 * {
 *     kArrayList list = kNULL; 
 *     k32s values[] = { 1, 2, 3, 5, 7, 9 }; 
 *     kSize i; 
 *     
 *     kTry
 *     {
 *          //create a list that can store 32-bit integers
 *          kTest(kArrayList_Construct(&list, kTypeOf(k32s), 0, kNULL)); 
 *          
 *          //add some initial items to the list
 *          for (i = 0; i < kCountOf(values); ++i)
 *          {
 *              kTest(kArrayList_Add(list, &values[i]); 
 *          }
 *          
 *          //print some information about the list and its items
 *          printf("Item type: %s\n", kType_Name(kArrayList_ItemType(list))); 
 *          printf("Count: %u\n", (k32u) kArrayList_Count(list)); 
 *  
 *          for (i = 0; i < kArrayList_Count(list); ++i)
 *          {
 *              //the kArrayList_As_ macro can be used to get a list item and cast it to the desired type; 
 *              //this is equivalent to *(k32s*)kArrayList_At(list, i);
 *              k32s value = kArrayList_As_(list, i, k32s); 
 *
 *              printf("Item %u: %d\n", (k32u)i, value);  
 *          }
 *     }
 *     kFinally
 *     {
 *          kObject_Destroy(list); 
 * 
 *          kEndFinally(); 
 *     }
 *    
 *     return kOK; 
 * }
 * 
 * @endcode
 *
 * For lists that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the list is destroyed. To recursively destroy both the list and the 
 * list items, use kObject_Dispose. 
 * 
 * kArrayList supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kArrayList supports the kdat5 and kdat6 serialization protocols. 
 */
//typedef kObject kArrayList;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a kArrayList object.
 *
 * @public                      @memberof kArrayList
 * @param   list                Receives constructed list object. 
 * @param   itemType            Type of list element.
 * @param   initialCapacity     Capacity initially reserved for list items. 
 * @param   allocator           Memory allocator (or kNULL for default). 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Construct(kArrayList* list, kType itemType, kSize initialCapacity, kAlloc allocator);

/** 
 * Reallocates the list item buffer.
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   itemType            Type of list element.
 * @param   initialCapacity     Capacity initially reserved for list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Allocate(kArrayList list, kType itemType, kSize initialCapacity);

/** 
 * Attaches the list object to an external buffer. 
 * 
 * The list count is set to the same value as the list capacity argument. 
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   items               Item buffer.
 * @param   itemType            Type of list element.
 * @param   capacity            List capacity. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Attach(kArrayList list, void* items, kType itemType, kSize capacity); 

/** 
 * Copies the specified items into the list, replacing existing contents. 
 *
 * The list count is set to the value of the count argument. 
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   items               Item buffer. 
 * @param   itemType            Type of list element.
 * @param   count               Count of list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Import(kArrayList list, const void* items, kType itemType, kSize count); 

/** 
 * Appends the specified items to the list. 
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   items               Item buffer. 
 * @param   count               Count of list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Append(kArrayList list, const void* items, kSize count); 

/** 
 * Performs a shallow copy of the source list.  
 *
 * Source items are copied by value; if the source list contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   source      Source list to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Assign(kArrayList list, kArrayList source);

/** 
 * Sets the count of list items to zero. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Clear(kArrayList list); 

/** 
 * Disposes any elements in the list and sets the count of list items to zero.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Purge(kArrayList list); 

/** 
 * Sets the memory for all list elements to zero. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Zero(kArrayList list); 

/** 
 * Adds the specified item to the end of the list.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   item        Pointer to item that will be copied (by value) into the list.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Add(kArrayList list, const void* item);

/** 
 * Inserts an item into the list at the specified position.
 *
 * Increases list capacity, if necessary.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   before      Item will be inserted before the item at this index.
 * @param   item        Pointer to item that will be copied (by value) into the list.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Insert(kArrayList list, kSize before, const void* item); 

/** 
 * Removes an item from the list at the specified index. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item at this index will be removed from the list.
 * @param   item        Destination for the removed item (copied by value, can be null).
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Remove(kArrayList list, kSize index, void* item); 

/** 
 * Sets the value of an item.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index. 
 * @param   item        Item that will be copied into the list.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_SetItem(kArrayList list, kSize index,  const void* item);

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index.  
 * @param   item        Destination for item that will be copied (by value) from the list. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Item(kArrayList list, kSize index, void* item); 

/** 
 * Sets the current count of list items to the specified value.
 *
 * Increases list capacity if necessary; existing list items are preserved. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   count       List size, in items.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Resize(kArrayList list, kSize count); 

/** 
 * Increases the list count by the specified amount.
 *
 * Increases list capacity if necessary; existing list items are preserved. 
 * New items are not initialized. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   count       Amount to add to the existing count.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_AddCount(kArrayList list, kSize count); 

/** 
 * Decreases the list count by the specified amount.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   count       Amount to remove from the existing count.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_RemoveCount(kArrayList list, kSize count); 

/** 
 * Ensures that capacity is reserved for at least the specified number of list items. 
 *
 * Existing list items are preserved. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   capacity    List capacity, in items.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Reserve(kArrayList list, kSize capacity); 

/** 
 * Returns a pointer to the list item buffer. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to list items. 
 */
kFx(void*) kArrayList_Data(kArrayList list);

/** 
 * Returns the total size of list data (Count x ItemSize), in bytes. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Data size, in bytes.  
 */
kFx(kSize) kArrayList_DataSize(kArrayList list);

/** 
 * Returns a pointer to the specified item in the list buffer. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index.
 * @return              Pointer to list element.
 */
kFx(void*) kArrayList_At(kArrayList list, kSize index); 

/** 
 * Returns the list element type. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Item type. 
 */
kFx(kType) kArrayList_ItemType(kArrayList list); 

/** 
 * Returns the list element size.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Item size, in bytes. 
 */
kFx(kSize) kArrayList_ItemSize(kArrayList list); 

/** 
 * Returns the current count of items in the list. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Current count of items. 
 */
kFx(kSize) kArrayList_Count(kArrayList list); 

/** 
 * Returns the number of elements for which space has been allocated.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              List capacity. 
 */
kFx(kSize) kArrayList_Capacity(kArrayList list); 

/** @relates  kArrayList @{ */

#define kArrayList_AddCount_(LIST, C)           kxArrayList_AddCount_(LIST, C)          ///< Macro version of kArrayList_AddCount.  
#define kArrayList_RemoveCount_(LIST, C)        kxArrayList_RemoveCount_(LIST, C)       ///< Macro version of kArrayList_RemoveCount.  
#define kArrayList_Clear_(LIST)                 kxArrayList_Clear_(LIST)                ///< Macro version of kArrayList_Clear.  

#define kArrayList_ItemType_(LIST)              kxArrayList_ItemType_(LIST)             ///< Macro version of kArrayList_ItemType.   
#define kArrayList_ItemSize_(LIST)              kxArrayList_ItemSize_(LIST)             ///< Macro version of kArrayList_ItemSize.  
#define kArrayList_Count_(LIST)                 kxArrayList_Count_(LIST)                ///< Macro version of kArrayList_Count.  
#define kArrayList_Capacity_(LIST)              kxArrayList_Capacity_(LIST)             ///< Macro version of kArrayList_Capacity.  

#define kArrayList_Data_(LIST)                  kxArrayList_Data_(LIST)                 ///< Macro version of kArrayList_Data.  
#define kArrayList_DataSize_(LIST)              kxArrayList_DataSize_(LIST)             ///< Macro version of kArrayList_DataSize.  

#define kArrayList_At_(LIST, INDEX)             kxArrayList_At_(LIST, INDEX)            ///< Macro version of kArrayList_At.  

#define kArrayList_First_(LIST)                 kxArrayList_First_(LIST)                ///< Gets a pointer to the first item.
#define kArrayList_Last_(LIST)                  kxArrayList_Last_(LIST)                 ///< Gets a pointer to the last item. 

#define kArrayList_Begin_(LIST)                 kxArrayList_Begin_(LIST)                ///< Gets a pointer to the first item.
#define kArrayList_End_(LIST)                   kxArrayList_End_(LIST)                  ///< Gets a pointer to one past the last item.

#define kArrayList_RBegin_(LIST)                kxArrayList_RBegin_(LIST)               ///< Gets a pointer to the last item. 
#define kArrayList_REnd_(LIST)                  kxArrayList_REnd_(LIST)                 ///< Gets a pointer to one before the first item.

#define kArrayList_As_(LIST, INDEX, TYPE)       kxArrayList_As_(LIST, INDEX, TYPE)      ///< Gets an item pointer and casts to the specified type.

/** @} */

kEndHeader()

#include <kApi/Data/kArrayList.x.h>

#endif
