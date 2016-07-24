/** 
 * @file    kString.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STRING_X_H
#define K_API_STRING_X_H

kBeginHeader()

#define kSTRING_MIN_CAPACITY                (16)
#define kSTRING_GROWTH_FACTOR               (2)

typedef struct kStringClass
{
    kObjectClass base; 
    kSize allocSize;            //size of allocated array memory, in bytes
    kChar* chars;               //null-terminated character array
    kSize length;               //current number of elements (excluding null-terminator)
    kSize capacity;             //maximum elements before reallocation
    kChar nullStr;              //null string (avoids need for dynamic allocation when null content pointer passed)
} kStringClass;

kDeclareClass(k, kString, kObject) 

kFx(kStatus) kString_Init(kString str, kType classType, const kChar* content, kAlloc allocator); 

kFx(kStatus) kString_VInitClone(kString str, kString source, kAlloc allocator); 

kFx(kStatus) kString_VRelease(kString str); 

kFx(kSize) kString_VHashCode(kString str); 
kFx(kBool) kString_VEquals(kString str, kObject other); 
kFx(kSize) kString_VSize(kString str); 

kFx(kStatus) kString_WriteDat5V1(kString str, kSerializer serializer); 
kFx(kStatus) kString_ReadDat5V1(kString str, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kString_WriteDat6V0(kString str, kSerializer serializer); 
kFx(kStatus) kString_ReadDat6V0(kString str, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kString_Import(kString str, const kChar* cstr, kBool append); 

kFx(kStatus) kString_SplitAdd(const kChar* start, kSize length, kArrayList tokens, kAlloc allocator); 

#define kString_(STRING)                      kCast(kStringClass*, STRING)
#define kString_Cast_(STRING)                 kCastClass_(kString, STRING)

#define kxString_Length_(STRING)             (kString_(STRING)->length)
#define kxString_Capacity_(STRING)           (kString_(STRING)->capacity)
#define kxString_Chars_(STRING)              (kString_(STRING)->chars)

//deprecated
#define kString_SetLength_(STRING, L)   kString_SetLength(STRING, L)   


/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_STRING                     kTypeOf(kString)
#   define kString_Destroy                  kObject_Destroy
#   define kSTRING_CHARS                    kString_Chars_
#   define kSTRING_LENGTH                   kString_Length_
#   define kSTRING_CAPACITY                 kString_Capacity_
#   define kSTRING_DATA                     kString_Chars_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kString_Construct5(S, C)         kString_Construct(S, C, kNULL)
#   define kString_Split5(S, D, T)          kString_Split(S, D, T, kNULL)
#   define kSTRING_AT(STRING, INDEX)        (kString_Chars_(STRING)[INDEX])
#   define kSTRING_LENGTH_BYTES(STRING)     (kString_Length_(STRING) * sizeof(kChar))
#   define kSTRING_CAPACITY_BYTES(STRING)   (kString_Capacity_(STRING) * sizeof(kChar))

#endif

kEndHeader()

#endif
