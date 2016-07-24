/** 
 * @file    kArray1.h
 * @brief   Declares the kArray1 class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_1_H
#define K_API_ARRAY_1_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class       kArray1
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a 1D array.
 * 
 * The kArray1 class represents a 1D array of objects or values. The kArray1 constructor accepts arguments
 * that determine the array item type (kType) and array length. 
 * 
 * For arrays that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the array is destroyed. To recursively destroy both the array and the 
 * array items, use kObject_Dispose. 
 * 
 * kArray1 supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kArray1 supports the kdat5 and kdat6 serialization protocols. 
 */
//typedef kObject kArray1;   --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kArray1 object.
 *
 * @public              @memberof kArray1
 * @param   array       Receives the constructed object.  
 * @param   itemType    Type of array element.
 * @param   length      Length of array.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Construct(kArray1* array, kType itemType, kSize length, kAlloc allocator);

/** 
 * Reallocates the internal array item buffer. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   itemType    Type of array element.
 * @param   length      Length of array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Allocate(kArray1 array, kType itemType, kSize length); 

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   items       External item buffer. 
 * @param   itemType    Type of array element.
 * @param   length      Length of array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Attach(kArray1 array, void* items, kType itemType, kSize length); 

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Assign(kArray1 array, kArray1 source);

/** 
 * Sets all array element bits to zero.
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Zero(kArray1 array); 

/** 
 * Sets the value of an item. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Array item index. 
 * @param   item        Pointer to item that will be copied (by value) into the array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_SetItem(kArray1 array, kSize index, const void* item); 

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Array item index. 
 * @param   item        Destination for item that will be copied (by value) from the array. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Item(kArray1 array, kSize index, void* item); 

/** 
 * Returns a pointer to the array item buffer.  
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Pointer to array item buffer. 
 */
kFx(void*) kArray1_Data(kArray1 array); 

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Size of array item buffer (bytes). 
 */
kFx(kSize) kArray1_DataSize(kArray1 array); 

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Item index. 
 * @return              Pointer to item. 
 */
kFx(void*) kArray1_At(kArray1 array, kSize index); 

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array item type. 
 */
kFx(kType) kArray1_ItemType(kArray1 array); 

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array item size. 
 */
kFx(kSize) kArray1_ItemSize(kArray1 array); 

/** 
 * Returns the array length, in elements. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array length (in elements). 
 */
kFx(kSize) kArray1_Length(kArray1 array); 

/** 
 * Returns the array item count, in elements. 
 *
 * This method is provided for symmetry with kArray2/kArray3.  In practice, 
 * the item count for a 1D array is always equal to the length. 
 * 
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array item count (in elements). 
 */
kFx(kSize) kArray1_Count(kArray1 array); 

/** @relates  kArray1 @{ */

#define kArray1_ItemType_(ARRAY)                kxArray1_ItemType_(ARRAY)           ///< Macro version of kArray1_ItemType.
#define kArray1_ItemSize_(ARRAY)                kxArray1_ItemSize_(ARRAY)           ///< Macro version of kArray1_ItemSize.    
#define kArray1_Length_(ARRAY)                  kxArray1_Length_(ARRAY)             ///< Macro version of kArray1_Length.    
#define kArray1_Count_(ARRAY)                   kxArray1_Count_(ARRAY)              ///< Macro version of kArray1_Count.    
#define kArray1_Data_(ARRAY)                    kxArray1_Data_(ARRAY)               ///< Macro version of kArray1_Data.    
#define kArray1_DataSize_(ARRAY)                kxArray1_DataSize_(ARRAY)           ///< Macro version of kArray1_DataSize.    
#define kArray1_At_(ARRAY, INDEX)               kxArray1_At_(ARRAY, INDEX)          ///< Macro version of kArray1_At.    

/** Accesses an array element at the specified index, and casts the value to the specified type. */
#define kArray1_As_(ARRAY, INDEX, TYPE)         kxArray1_As_(ARRAY, INDEX, TYPE) 

/** @} */


kEndHeader()

#include <kApi/Data/kArray1.x.h>

#endif
