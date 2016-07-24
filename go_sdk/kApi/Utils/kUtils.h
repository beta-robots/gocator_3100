/** 
 * @file    kUtils.h
 * @brief   Utility functions.
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>         //--inclusion order controlled by kApiDef

#ifndef K_API_UTILS_H
#define K_API_UTILS_H

kBeginHeader()

/**
 * @class   kUtils
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Collection of utility functions. 
 */

/** 
 * Destroys an object and resets the object handle to kNULL. 
 *
 * @public              @memberof kUtils
 * @param   object      Pointer to object, or pointer to kNULL. 
 * @return              Operation status.
 */
kFx(kStatus) kDestroyRef(kObject* object); 

/** 
 * Disposes an object and resets the object handle to kNULL. 
 *
 * @public              @memberof kUtils
 * @param   object      Pointer to object, or pointer to kNULL. 
 * @return              Operation status.
 */
kFx(kStatus) kDisposeRef(kObject* object); 

/** 
 * Shares an object and sets a handle to refer to the shared object. 
 *
 * @public              @memberof kUtils
 * @param   object      Receives shared object handle. 
 * @param   source      Object to be shared (or kNULL). 
 * @return              Operation status.
 */
kFx(kStatus) kShareRef(kObject* object, kObject source); 

/** 
 * Loads an object from file using kDat-5 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Receives deserialized object.
 * @param   fileName    Path of the file to load. 
 * @param   allocator   Memory allocator to use for loaded object (or kNULL for default). 
 * @return              Operation status.
 */
kFx(kStatus) kLoad5(kObject* object, const kChar* fileName, kAlloc allocator); 

/** 
 * Saves an object to file using kDat-5 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Object to be serialized.
 * @param   fileName    Path of the file to save. 
 * @return              Operation status.
 */
kFx(kStatus) kSave5(kObject object, const kChar* fileName); 

/** 
 * Loads an object from file using kDat-6 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Receives deserialized object.
 * @param   fileName    Path of the file to load. 
 * @param   allocator   Memory allocator to use for loaded object (or kNULL for default). 
 * @return              Operation status.
 */
kFx(kStatus) kLoad6(kObject* object, const kChar* fileName, kAlloc allocator); 

/** 
 * Saves an object to file using kDat-6 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Object to be serialized.
 * @param   fileName    Path of the file to save. 
 * @return              Operation status.
 */
kFx(kStatus) kSave6(kObject object, const kChar* fileName); 

/** 
 * Allocates a block of memory from the application heap.
 *
 * Memory allocated with this function should be freed with the kMemFree function.
 *
 * @public              @memberof kUtils
 * @param   size        Size of memory to allocate, in bytes.
 * @param   mem         Receives a pointer to the memory block.
 * @return              Operation status.
 */
kFx(kStatus) kMemAlloc(kSize size, void* mem);

/** 
 * Allocates and zero-initializes block of memory from the application heap.
 *
 * Memory allocated with this function should be freed with the kMemFree function.
 *
 * @public              @memberof kUtils
 * @param   size        Size of memory to allocate, in bytes.
 * @param   mem         Receives a pointer to the memory block.
 * @return              Operation status.
 */
kFx(kStatus) kMemAllocZero(kSize size, void* mem);

/** 
 * Frees a block of memory that was allocated using kMemAlloc or kMemAllocZero. 
 *
 * @public              @memberof kUtils
 * @param   mem         Pointer to memory to free (or kNULL). 
 * @return              Operation status.
 */
kFx(kStatus) kMemFree(void* mem); 

/** 
 * Frees a block of memory that was allocated using kMemAlloc or kMemAllocZero and resets the memory pointer to kNULL. 
 *
 * @public              @memberof kUtils
 * @param   mem         Pointer to pointer to memory to free (or pointer to kNULL). 
 * @return              Operation status.
 */
kFx(kStatus) kMemFreeRef(void* mem); 

/** 
 * Sets a block of memory to the given byte value. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory set operation.
 * @param   fill        Value to be set.
 * @param   size        Size of memory block to be set, in bytes.
 * @return              Operation status.
 */
kFx(kStatus) kMemSet(void* dest, kByte fill, kSize size); 

/** 
 * Copies memory from a source buffer to a non-overlapping destination.  
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory copy.
 * @param   src         Source for the memory copy.
 * @param   size        Size of memory block to be copied, in bytes.
 * @return              Operation status.
 */
kFx(kStatus) kMemCopy(void* dest, const void* src, kSize size); 

/** 
 * Copies memory from a source buffer to a potentially-overlapping destination.  
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory copy.
 * @param   src         Source for the memory copy.
 * @param   size        Size of memory block to be copied, in bytes.
 * @return              Operation status.
 */
kFx(kStatus) kMemMove(void* dest, const void* src, kSize size);

/** 
 * Compares one memory buffer with another. 
 *
 * @public              @memberof kUtils
 * @param   a           First buffer. 
 * @param   b           Second buffer. 
 * @param   size        Size of memory buffers to be compared, in bytes.
 * @return              kTRUE if the memory buffers are equal; otherwise, kFALSE. 
 */
kFx(kBool) kMemEquals(const void* a, const void* b, kSize size);

/** 
 * Copies characters from source to destination. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the destination capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @param   src         Source for the string copy.
 * @return              Operation status. 
 */
kFx(kStatus) kStrCopy(kChar* dest, kSize capacity, const kChar* src); 

/** 
 * Appends characters from source to destination. 
 * 
 * If the buffer is insufficient, the concatenation will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the destination capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination string to append to.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @param   src         Source string to append.
 * @return              Operation status. 
 */
kFx(kStatus) kStrCat(kChar* dest, kSize capacity, const kChar* src); 

/** 
 * Converts characters in the given sequence to lower case. 
 *
 * This function currently supports conversion of characters only within the ASCII character range. 
 *
 * @public          @memberof kUtils
 * @param   str     Character sequence to convert.
 * @return          Operation status. 
 */
kFx(kStatus) kStrToLower(kChar* str); 

/** 
 * Tests a pair of character sequences for equality. 
 * 
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @return           kTRUE if the character sequences are equal; otherwise, kFALSE. 
 */
kFx(kBool) kStrEquals(const kChar* a, const kChar* b); 

/** 
 * Compares one string to another. 
 * 
 * The result is negative if the string a is lexically less than string b, positive 
 * if string a is lexically greater than string b, and zero if they are equal. 
 *
 * This function performs comparison of UTF-8 encoded characters by Unicode code point.
 *
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @return           Positive if a is greater than b, negative if b is greater than a; otherwise zero.
 */
kFx(k32s) kStrCompare(const kChar* a, const kChar* b); 

/** 
 * Performs a case-insenstive comparison of two strings. 
 *
 * This function currently supports comparison of characters only within the ASCII character range.
 *
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @return           Positive if a is greater than b, negative if b is greater than a; otherwise zero.
 */
kFx(k32s) kStrCompareLower(const kChar* a, const kChar* b); 

/** 
 * Determines the number of kChar units in a characater sequence. 
 * 
 * @public           @memberof kUtils
 * @param   str      Input string. 
 * @return           Number of kChar units in sequence.
 */
kFx(kSize) kStrLength(const kChar* str); 

/** 
 * Finds the first occurrence of a character sequence. 
 * 
 * @public           @memberof kUtils
 * @param   str      Input string to be searched. 
 * @param   subStr   Substring to find. 
 * @return           Pointer to first occurrence, or kNULL. 
 */
kFx(const kChar*) kStrFindFirst(const kChar* str, const kChar* subStr); 

/** 
 * Finds the last occurrence of a character sequence. 
 * 
 * @public           @memberof kUtils
 * @param   str      Input string to be searched. 
 * @param   subStr   Substring to find. 
 * @return           Pointer to last occurrence, or kNULL. 
 */
kFx(const kChar*) kStrFindLast(const kChar* str, const kChar* subStr); 

/** 
 * Formats a string using printf-style arguments.  
 *
 * This function relies on formatting support from underlying system libraries; results can vary.  
 *
 * If the output buffer is insufficient, kERROR_INCOMPLETE will be returned. In this case, the 
 * destination buffer will contain truncated, null-terminated output. 
 *
 * @public           @memberof kUtils
 * @param   dest     Destination for formatted output. 
 * @param   capacity Capacity of output buffer. 
 * @param   format   Print format string. 
 * @return           Operation status. 
 */
kFx(kStatus) kStrPrintf(kChar* dest, kSize capacity, const kChar* format, ...); 

/** 
 * Variable-argument version of kStrPrintf. 
 *
 * @public           @memberof kUtils
 * @param   dest     Destination for formatted output. 
 * @param   capacity Capacity of output buffer. 
 * @param   format   Print format string. 
 * @param   argList  Variable argument list.
 * @return           Operation status. 
 * @see              kStrPrintf
 */
kFx(kStatus) kStrPrintvf(kChar* dest, kSize capacity, const kChar* format, kVarArgList argList); 

/** 
 * Writes to logging handler (if registered). The behaviour of the logging handler is system-specific. 
 * The logging handler is guaranteed to be thread-safe, but may not be ISR-safe.
 *
 * @public              @memberof kUtils
 * @param    format     Print format string. 
 * @return              Operation status. 
 */
kFx(kStatus) kLogf(const kChar* format, ...);

/** 
 * Variable-argument version of kLogf. 
 *
 * @public              @memberof kUtils
 * @param    format     Print format string. 
 * @param    argList    Variable argument list.
 * @return              Operation status. 
 * @see                 kLogf 
 */
kFx(kStatus) kLogvf(const kChar* format, kVarArgList argList);

/** 
 * Generates a random 32-bit number.
 *
 * @public  @memberof kUtils
 * @return  Random 32-bit number.
 */
kFx(k32u) kRandom32u();

/** 
 * Generates a random 64-bit number.
 *
 * @public  @memberof kUtils
 * @return  Random 64-bit number.
 */
kFx(k64u) kRandom64u();

/** 
 * Generates a random number of type kSize. 
 *
 * @public  @memberof kUtils
 * @return  Random number.
 */
kFx(kSize) kRandomSize();

kEndHeader()

#include <kApi/Utils/kUtils.x.h>

#endif
