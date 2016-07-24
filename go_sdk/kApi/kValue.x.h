/** 
 * @file    kValue.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_VALUE_X_H
#define K_API_VALUE_X_H

kBeginHeader()

typedef kStatus (kCall* kValueSerializeFx)(kType type, const void* values, kSize count, kSerializer serializer); 
typedef kStatus (kCall* kValueDeserializeFx)(kType type, const void* values, kSize count, kSerializer serializer); 

typedef struct kValueVTable
{
    kBool (kCall* VEquals)(kType type, const void* value, const void* other); 
    kSize (kCall* VHashCode)(kType type, const void* value); 
    void (kCall* VImport)(kType type, void* value, const void* source); 
} kValueVTable; 

kDeclareVirtualValue(k, kValue, kNull)

kFx(kBool) kValue_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kValue_VHashCode(kType type, const void* value); 
kFx(void) kValue_VImport(kType type, void* value, const void* source); 

#define kValue_VTable_(TYPE)                        kCast(kValueVTable*, kType_VTable_(TYPE))

#define kValue_Equals_(TYPE, VALUE, OTHER)          (kValue_VTable_(TYPE)->VEquals(TYPE, VALUE, OTHER)) 
#define kValue_HashCode_(TYPE, VALUE)               (kValue_VTable_(TYPE)->VHashCode(TYPE, VALUE)) 
#define kValue_Import_(TYPE, VALUE, SOURCE)         (kValue_VTable_(TYPE)->VImport(TYPE, VALUE, SOURCE)) 

kEndHeader()

#endif
