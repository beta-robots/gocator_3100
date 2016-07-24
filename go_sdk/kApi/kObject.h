/** 
 * @file    kObject.h
 * @brief   Declares the kObject class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_OBJECT_H
#define K_API_OBJECT_H

kBeginHeader()

/**
 * @class   kObject
 * @ingroup kApi
 * @brief   Root of all Zen classes. 
 * 
 * The kObject class provides infrastructure to support object destruction, reference counting, and type introspection. 
 *
 * The kObject_Destroy method can be used to destroy an instance of any class. The kObject_Dispose method provides 
 * support for recursive destruction, often used in conjunction with Zen data collections. Refer to @ref kApi-Destruction 
 * for more information. 
 * 
 * The kObject_Clone method can be used to create a deep copy of an object. The implementation of this method 
 * requires additional support from derived types; refer to the documentation for a specific derived type to determine 
 * whether cloning is supported. 
 * 
 * The kObject_Clone and kObject_Dispose methods are often used together to manage collections of data objects: 
 * 
 * @code {.c}
 * 
 * kList list = kNULL; 
 * kList copy = kNULL; 
 * kImage image = kNULL;
 * kSize imageCount = 10; 
 *
 * //create a list of image objects
 * kCheck(kList_Construct(&list, kTypeOf(kImage), imageCount, kNULL)); 
 * 
 * for (kSize i = 0; i < imageCount; ++i)
 * {
 *     kCheck(kImage_Construct(&image, kTypeOf(kArgb), 640, 480, kNULL)); 
 *     kList_Add(list, &image); 
 * }
 * 
 * //make a deep copy of the list; copies both the list and the images contained in the list
 * kCheck(kObject_Clone(&copy, list, kNULL)); 
 * //...
 * 
 * //clean up; destroy the lists and the images contained in the lists
 * kObject_Dispose(list); 
 * kObject_Dispose(copy);
 *
 * @endcode
 *
 * All kObject-derived instances are reference counted. The kObject_Share method can optionally be used to increment an object's 
 * reference count, while the kObject_Destroy/kObject_Dispose methods are always used to decrement an object's reference 
 * count. Refer to @ref kApi-Reference-Counting for more information.
 *
 * The kObject_Type method can be used to access type information for any kObject instance. The kType object returned
 * by this method can be used to learn about the object's class, including its name, base classes, implemented interfaces, 
 * and methods.  The kObject_Is method provides a convenient way to determine whether an object derives from a specific 
 * base class or implements an interface. 
 * 
 * The kObject_Equals and kObject_HashCode methods can be helpful for object comparisons, but require support from 
 * derived types and are infrequently overridden. The kString class overrides both of these methods to support the use of 
 * kString objects as hash keys in kMap data collections. 
 */
//typedef kPointer kObject;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a new object by copying an existing object, including any aggregated child elements. 
 *
 * If the source object is an object collection (e.g. kArrayList<kString>), any aggregated child objects 
 * are also cloned. In this case, the kObject_Dispose method can be used to free the cloned collection and 
 * its associated elements. 
 * 
 * This method will fail if the source object (or an aggregated child element) does not support cloning.
 * 
 * @public              @memberof kObject
 * @param   object      Receives the constructed object. 
 * @param   source      Source object. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 * @see                 kObject_Dispose
 */
kFx(kStatus) kObject_Clone(kObject* object, kObject source, kAlloc allocator);

/** 
 * Increments the reference count associated with this object. 
 *
 * This method is thread-safe.
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Operation status. 
 * @see                 reference-counting
 */
kFx(kStatus) kObject_Share(kObject object); 

/** 
 * Sets the object pool associated with this object. 
 *
 * Object pools can be used to implement custom lifetime management. If an object has an assigned pool, 
 * then the kObjectPool_Reclaim method will be called just prior to destruction, to provide an opportunity
 * for the object to be reclaimed. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @param   pool        Pool object (or kNULL to clear the pool assignment). 
 * @return              Operation status. 
 */
kFx(kStatus) kObject_SetPool(kObject object, kObjectPool pool); 

/** 
 * Destroys the object. 
 * 
 * The kObject_Destroy method destroys the object itself and any resources that are owned by the object.  
 * See @ref kApi-Destruction for more information. 
 * 
 * When an object is destroyed (or disposed), its reference count is decremented. The object is only truly 
 * destroyed when the reference count reaches zero. See @ref kApi-Reference-Counting for more information.
 * 
 * @public              @memberof kObject
 * @param   object      Object (or kNULL). 
 * @return              Operation status. 
 * @see                 @ref kApi-Destruction, @ref kApi-Reference-Counting
 */
kFx(kStatus) kObject_Destroy(kObject object);

/** 
 * Destroys the object and any aggregated child elements. 
 * 
 * The kObject_Dispose method destroys the object itself, any resources that are owned by the object, and 
 * if the object represents a collection of objects, any child objects in the collection. See @ref kApi-Destruction 
 * for more information. 
 * 
 * When an object is destroyed (or disposed), its reference count is decremented. The object is only truly 
 * destroyed when the reference count reaches zero. See @ref kApi-Reference-Counting for more information.
 * 
 * @public              @memberof kObject
 * @param   object      Object (or kNULL). 
 * @return              Operation status. 
 * @see                 @ref kApi-Destruction, @ref kApi-Reference-Counting
 */
kFx(kStatus) kObject_Dispose(kObject object);

/**
 * Returns the type of the object.
 *
 * Each object is an instance of a specific class type. The type handle returned by this function can be 
 * used to learn about the class. 
 * 
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Type. 
 * @see                 kType, kObject_Type_
 */      
kFx(kType) kObject_Type(kObject object);

/**
 * Determines whether this object is an instance of the specified type.
 * 
 * This function compares the type of this object with the given type. An object is considered to be an 
 * instance of a given type if a) the type represents a class and this object inherits from (or instantiates) 
 * that class, or b) the type represents an interface and this object implements the interface. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @param   type        Type. 
 * @return              kTRUE if the object is of the specified type; otherwise kFALSE. 
 * @see                 kObject_Is_, kType_Is
 */
kFx(kBool) kObject_Is(kObject object, kType type);

/**
 * Determines whether the object is equal to another object. 
 * 
 * By default, objects are compared by reference; objects are considered equal if the given handles refer to 
 * the same object instance. The Equals method can optionally be overridden to provide a more meaningful 
 * equality comparison. 
 * 
 * @public              @memberof kObject
 * @param   object      Object. 
 * @param   other       Object for comparison. 
 * @return              kTRUE if the objects are equal.
 */
kFx(kBool) kObject_Equals(kObject object, kObject other); 

/**
 * Gets a hash code representing the state of this object.
 * 
 * By default, objects return a hash code based on the object handle value. The HashCode method can optionally 
 * be overridden to provide a more useful hash. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Hash code. 
 */
kFx(kSize) kObject_HashCode(kObject object); 

/**
 * Gets the memory allocator associated with this object.  
 * 
 * Most objects are constructed with an allocator, which is used to allocate the memory required by the 
 * object. Objects retain a reference to this allocator to enable further allocations and to free memory when 
 * the object is destroyed. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Memory allocator. 
 * @see                 kAlloc, kObject_Alloc_
*/
kFx(kAlloc) kObject_Alloc(kObject object);

/**
 * Estimates the memory consumed by this object, including any aggregated child elements. 
 * 
 * This method can be optionally overridden by kObject-derived classes to report the amount of memory consumed 
 * by an object. The default implementation reports only the size of the class instance (additional allocations 
 * performed by the class are excluded). 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Object size, in bytes.  
*/
kFx(kSize) kObject_Size(kObject object); 

/**
 * Reports whether the object is currently shared (reference count greater than one). 
 * 
 * Objects are initialized with a reference count of one. The kObject_Share method can be used to increment 
 * the reference count. The kObject_Destroy and kObject_Dispose methods decrease the reference count, and 
 * when the reference count reaches zero, the object is actually destroyed/disposed. 
 * 
 * This method can be used to determine if the reference count of an object is currently greater than one. 
 * 
 * This method is thread-safe. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              kTRUE if the object is shared; kFALSE otherwise. 
 * @see                 @ref kApi-Reference-Counting, kObject_Share
*/
kFx(kBool) kObject_IsShared(kObject object); 

/** @relates kObject @{ */
#define kObject_Type_(OBJ)              kxObject_Type_(OBJ)             ///< Macro version of kObject_Type.
#define kObject_Is_(OBJ, TYPE)          kxObject_Is_(OBJ, TYPE)         ///< Macro version of kObject_Is.
#define kObject_Alloc_(OBJ)             kxObject_Allocator_(OBJ)        ///< Macro version of kObject_Alloc.
/** @} */

kEndHeader()

#include <kApi/kObject.x.h>

#endif 
