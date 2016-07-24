/** 
 * @file    kCollection.h
 * @brief   Declares the kCollection interface. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_COLLECTION_H
#define K_API_COLLECTION_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @interface kCollection
 * @ingroup   kApi-Data
 * @brief     Supports forward iteration over a collection of items.
 * @see       kIterator
 * 
 * The kCollection interface supports forward iteration over a collection of elements. Typically, 
 * the classes that implement this interface provide alternative, class-specific accessor 
 * methods with better performance.  However, the kCollection interface can be used to reduce the amount 
 * of container-specific code required to iterate over a variety of collections, in contexts where 
 * performance is not important. 
 * 
 * @code {.c}
 * kObject collection = arrayList;  
 * kIterator it = kCollection_GetIterator(collection); 
 * kType type = kCollection_ItemType(collection); 
 *
 * while (kCollection_HasNext(collection, it))
 * {
 *     void* item = kCollection_Next(collection, &it); 
 *     //...
 * }
 * @endcode
 */
//typedef kObject kCollection;   --forward-declared in kApiDef.x.h

/**
 * @struct  kIterator
 * @ingroup kApi-Data
 * @brief   Used in conjunction with the kCollection class to iterate over elements. 
 * @see     kCollection
 * 
 * kIterator is an opaque value type; an iterator instance can be copied by value, but the 
 * contents of the iterator structure should not be examined or manipulated directly.  Use 
 * kCollection methods to work with the iterator. 
 */
//typedef kPointer kIterator;   --forward-declared in kApiDef.x.h

/** 
 * Gets the collection element type.
 *
 * @public              @memberof kCollection
 * @param   collection  Collection object. 
 * @return              Item type. 
 */
kFx(kType) kCollection_ItemType(kCollection collection);

/** 
 * Gets the collection element count.
 *
 * @public              @memberof kCollection
 * @param   collection  Collection object. 
 * @return              Item count. 
 */
kFx(kSize) kCollection_Count(kCollection collection); 

/** 
 * Returns an iterator to the first element in the collection. 
 * 
 * @public              @memberof kCollection
 * @param   collection  Collection object. 
 * @return              Iterator. 
 */
kFx(kIterator) kCollection_GetIterator(kCollection collection);

/** 
 * Determines whether a collection has another item. 
 * 
 * @public              @memberof kCollection
 * @param   collection  Collection object. 
 * @param   iterator    Collection iterator. 
 * @return              kTRUE if the collection has a next element. 
 */
kFx(kBool) kCollection_HasNext(kCollection collection, kIterator iterator); 

/** 
 * Gets the next collection element and then advances the iterator.
 * 
 * @public              @memberof kCollection
 * @param   collection  Collection object. 
 * @param   iterator    Pointer to collection iterator. 
 * @return              Pointer to next collection element. 
 */
kFx(void*) kCollection_Next(kCollection collection, kIterator* iterator); 

kEndHeader()

#include <kApi/Data/kCollection.x.h>

#endif
