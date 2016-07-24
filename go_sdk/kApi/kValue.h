/** 
 * @file    kValue.h
 * @brief   Declares the kValue type. 
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_VALUE_H
#define K_API_VALUE_H

kBeginHeader()

/**
 * @class   kValue
 * @ingroup kApi
 * @brief   Root of all Zen value types. 
 * 
 * Value types represent structures, primitive values and enumerations. The kValue base type defines methods that can 
 * be called on any value instance. kValue does not add any public or private fields to the types that extend kValue. 
 * 
 * The kValue_Equals and kValue_HashCode methods are used to support value comparisons and are implemented 
 * by most value types. 
 * 
 * @code {.c}
 *
 * kBool EqualsExample(const kRect32s* a, const kRect32s* b)
 * {
 *     return kValue_Equals(kTypeOf(kRect32s), a, b); 
 * }
 *
 * @endcode
 *
 * Unlike reference types, value types do not carry type information. Accordingly, type information must 
 * be passed as the first argument of any kValue method. 
 */

/**
 * Determines whether a value is equal to another value. 
 * 
 * The default implementation of this method uses type introspection to compare the values field by field. 
 * This approach is not efficient, and in some cases may not produce the desired result. Value types can 
 * override this method if it is likely that equality comparisons will be required. 
 * 
 * @public              @memberof kValue
 * @param   type        Value type. 
 * @param   value       Pointer to value. 
 * @param   other       Pointer to other value. 
 * @return              kTRUE if the values are equal.
 */
kFx(kBool) kValue_Equals(kType type, const void* value, const void* other); 

/**
 * Gets a hash code representing the state of this value.
 *
 * The default implementation of this method uses type introspection to generate a hash code that combines the 
 * hash codes from individual fields. This approach is not efficient, and may not produce an optimal hash code. 
 * Value types can override this method if it is likely that hash codes will be required. 
 *
 * @public              @memberof kValue
 * @param   type        Value type. 
 * @param   value       Pointer to value.
 * @return              Hash code. 
 */
kFx(kSize) kValue_HashCode(kType type, const void* value); 

/**
 * Imports the content of another value into this value.
 *
 * This method supports importing content from an external source, which may have a different size.
 * In particular, <em>array value</em> types such as kText32 can import from a null-terminated source array of a different 
 * length. 
 *
 * @public              @memberof kValue
 * @param   type        Value type. 
 * @param   value       Pointer to value.
 * @param   source      Source for import. 
 */
kFx(void) kValue_Import(kType type, void* value, const void* source); 

kEndHeader()

#include <kApi/kValue.x.h>

#endif
