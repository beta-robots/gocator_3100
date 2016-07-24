/** 
 * @file    kArray2.h
 * @brief   Declares the kArray2 class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_2_H
#define K_API_ARRAY_2_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class       kArray2
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a 2D array.
 * 
 * The kArray2 class represents a 2D array of objects or values. The kArray2 constructor accepts arguments
 * that determine the array item type (kType) and array dimension lengths. 
 * 
 * For arrays that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the array is destroyed. To recursively destroy both the array and the 
 * array items, use kObject_Dispose. 
 * 
 * kArray2 supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 *
 * kArray2 supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kArray2;   --forward-declared in kApiDef.x.h  

/** 
 * Constructs a kArray2 object.
 *
 * @public              @memberof kArray2
 * @param   array       Receives the constructed object.  
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension (innermost). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Construct(kArray2* array, kType itemType, kSize length0, kSize length1, kAlloc allocator);

/** 
 * Reallocates the internal array item buffer. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Allocate(kArray2 array, kType itemType, kSize length0, kSize length1); 

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   items       External item buffer. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Attach(kArray2 array, void* items, kType itemType, kSize length0, kSize length1); 

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Assign(kArray2 array, kArray2 source);

/** 
 * Sets all array element bits to zero.
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Zero(kArray2 array); 

/** 
 * Sets the value of an item. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   item        Pointer to item that will be copied (by value) into the array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_SetItem(kArray2 array, kSize index0, kSize index1, const void* item); 

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   item        Destination for item that will be copied (by value) from the array. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Item(kArray2 array, kSize index0, kSize index1, void* item); 

/** 
 * Returns a pointer to the array item buffer.  
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Pointer to array item buffer. 
 */
kFx(void*) kArray2_Data(kArray2 array); 

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Size of array item buffer (bytes). 
 */
kFx(kSize) kArray2_DataSize(kArray2 array); 

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @return              Pointer to item. 
 */
kFx(void*) kArray2_At(kArray2 array, kSize index0, kSize index1); 

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Array item type. 
 */
kFx(kType) kArray2_ItemType(kArray2 array); 

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Array item size. 
 */
kFx(kSize) kArray2_ItemSize(kArray2 array); 

/** 
 * Returns the length of the specified array dimension, in elements. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   dimension   Array dimension index. 
 * @return              Array dimension length (in elements). 
 */
kFx(kSize) kArray2_Length(kArray2 array, kSize dimension); 

/** 
 * Returns the array item count, in elements. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Array item count (in elements). 
 */
kFx(kSize) kArray2_Count(kArray2 array); 

/** @relates  kArray2 @{ */

#define kArray2_ItemType_(ARRAY)                kxArray2_ItemType_(ARRAY)           ///< Macro version of kArray2_ItemType.    
#define kArray2_ItemSize_(ARRAY)                kxArray2_ItemSize_(ARRAY)           ///< Macro version of kArray2_ItemSize.   
#define kArray2_Length_(ARRAY, DIM)             kxArray2_Length_(ARRAY, DIM)        ///< Macro version of kArray2_Length.       
#define kArray2_Count_(ARRAY)                   kxArray2_Count_(ARRAY)              ///< Macro version of kArray2_Count.    
#define kArray2_Data_(ARRAY)                    kxArray2_Data_(ARRAY)               ///< Macro version of kArray2_Data.    
#define kArray2_DataSize_(ARRAY)                kxArray2_DataSize_(ARRAY)           ///< Macro version of kArray2_DataSize.   
#define kArray2_At_(ARRAY, I0, I1)              kxArray2_At_(ARRAY, I0, I1)         ///< Macro version of kArray2_At.    

/** Accesses an array element at the specified indices, and casts the value to the specified type.  */
#define kArray2_As_(ARRAY, I0, I1, TYPE)        kxArray2_As_(ARRAY, I0, I1, TYPE) 

/** @} */


kEndHeader()

#include <kApi/Data/kArray2.x.h>

#endif
