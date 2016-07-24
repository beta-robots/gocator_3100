/** 
 * @file    kArray3.h
 * @brief   Declares the kArray3 class. 
 *
 * @internal
 * Copyright (C) 2006-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_3_H
#define K_API_ARRAY_3_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class       kArray3
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a 3D array.
 * 
 * The kArray3 class represents a 3D array of objects or values. The kArray3 constructor accepts arguments
 * that determine the array item type (kType) and array dimension lengths. 
 * 
 * For arrays that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the array is destroyed. To recursively destroy both the array and the 
 * array items, use kObject_Dispose. 
 * 
 * kArray3 supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kArray3 supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kArray3;    --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kArray3 object.
 *
 * @public              @memberof kArray3
 * @param   array       Receives the constructed object.  
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension. 
 * @param   length2     Length of third array dimension (innermost). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Construct(kArray3* array, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator);

/** 
 * Reallocates the internal array item buffer. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension. 
 * @param   length2     Length of third array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Allocate(kArray3 array, kType itemType, kSize length0, kSize length1, kSize length2); 

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   items       External item buffer. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension. 
 * @param   length2     Length of third array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Attach(kArray3 array, void* items, kType itemType, kSize length0, kSize length1, kSize length2); 

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Assign(kArray3 array, kArray3 source);

/** 
 * Sets all array element bits to zero.
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Zero(kArray3 array); 

/** 
 * Sets the value of an item. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @param   item        Pointer to item that will be copied (by value) into the array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_SetItem(kArray3 array, kSize index0, kSize index1, kSize index2, const void* item); 

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @param   item        Destination for item that will be copied (by value) from the array. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Item(kArray3 array, kSize index0, kSize index1, kSize index2, void* item); 

/** 
 * Returns a pointer to the array item buffer.  
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Pointer to array item buffer. 
 */
kFx(void*) kArray3_Data(kArray3 array); 

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Size of array item buffer (bytes). 
 */
kFx(kSize) kArray3_DataSize(kArray3 array); 

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @return              Pointer to item. 
 */
kFx(void*) kArray3_At(kArray3 array, kSize index0, kSize index1, kSize index2); 

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Array item type. 
 */
kFx(kType) kArray3_ItemType(kArray3 array); 

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Array item size. 
 */
kFx(kSize) kArray3_ItemSize(kArray3 array); 

/** 
 * Returns the length of the specified array dimension, in elements. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   dimension   Array dimension index. 
 * @return              Array dimension length (in elements). 
 */
kFx(kSize) kArray3_Length(kArray3 array, kSize dimension); 

/** 
 * Returns the array item count, in elements. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Array item count (in elements). 
 */
kFx(kSize) kArray3_Count(kArray3 array); 

/** @relates  kArray3 @{ */

#define kArray3_ItemType_(ARRAY)                kxArray3_ItemType_(ARRAY)           ///< Macro version of kArray3_ItemType.    
#define kArray3_ItemSize_(ARRAY)                kxArray3_ItemSize_(ARRAY)           ///< Macro version of kArray3_ItemSize.   
#define kArray3_Length_(ARRAY, DIM)             kxArray3_Length_(ARRAY, DIM)        ///< Macro version of kArray3_Length.          
#define kArray3_Count_(ARRAY)                   kxArray3_Count_(ARRAY)              ///< Macro version of kArray3_Count.    
#define kArray3_Data_(ARRAY)                    kxArray3_Data_(ARRAY)               ///< Macro version of kArray3_Data.    
#define kArray3_DataSize_(ARRAY)                kxArray3_DataSize_(ARRAY)           ///< Macro version of kArray3_DataSize.   
#define kArray3_At_(ARRAY, I0, I1, I2)          kxArray3_At_(ARRAY, I0, I1, I2)     ///< Macro version of kArray3_At.    

/** Accesses an array element at the specified indices, and casts the value to the specified type.  */
#define kArray3_As_(ARRAY, I0, I1, I2, TYPE)    kxArray3_At_(ARRAY, I0, I1, I2)    

/** @} */

kEndHeader()

#include <kApi/Data/kArray3.x.h>

#endif
