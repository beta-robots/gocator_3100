/** 
 * @file    kBox.h
 * @brief   Declares the kBox class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BOX_H
#define K_API_BOX_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kBox
 * @extends kObject
 * @ingroup kApi-Data
 * @brief   Represents an instance of a value type as an object. 
 * 
 * kBox provides a simple way to represent a single value (e.g. k32s) using a kObject-derived class instance. 
 * This approach can sometimes be used to reduce duplicated effort when both reference types and value types
 * must be supported.
 * 
 * kBox supports the kObject_Clone and kObject_Size methods.
 *
 * kBox supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kBox;    --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kBox object.
 *
 * @public              @memberof kBox
 * @param   box         Receives the constructed object.  
 * @param   itemType    Type of box element (value types only). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Construct(kBox* box, kType itemType, kAlloc allocator);

/** 
 * Reallocates the internal box item buffer. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   itemType    Type of box element.
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Allocate(kBox box, kType itemType);

/** 
 * Copies the value contained within the source box into this box. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   source      Source box to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Assign(kBox box, kBox source); 

/** 
 * Sets all box element bits to zero.
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Zero(kBox box); 

/** 
 * Sets the box value. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   item        Pointer to item that will be copied into the box.
 * @return              Operation status. 
 */
kFx(kStatus) kBox_SetItem(kBox box, const void* item); 

/** 
 * Gets the box value. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   item        Destination for value copied from the box. 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Item(kBox box, void* item); 

/** 
 * Returns a pointer to the box item buffer.  
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Pointer to box item buffer. 
 */
kFx(void*) kBox_Data(kBox box);

/** 
 * Returns the box item type. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Box item type. 
 */
kFx(kType) kBox_ItemType(kBox box);

/** 
 * Returns the box item size. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Box item size. 
 */
kFx(kSize) kBox_ItemSize(kBox box); 

/** @relates kBox @{ */

#define kBox_ItemType_(BOX)             kxBox_ItemType_(BOX)       ///< Macro version of kBox_ItemType.    
#define kBox_ItemSize_(BOX)             kxBox_ItemSize_(BOX)       ///< Macro version of kBox_ItemSize.    
#define kBox_Data_(BOX)                 kxBox_Data_(BOX)           ///< Macro version of kBox_Data.         

/** Accesses the box value and casts the value to the specified type. */
#define kBox_As_(BOX, TYPE)             kxBox_As_(BOX, TYPE)   

/** @} */

kEndHeader()

#include <kApi/Data/kBox.x.h>

#endif
