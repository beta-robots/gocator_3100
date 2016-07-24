/** 
 * @file    kValue.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kValue.h>

kBeginVirtualValue(k, kValue, kNull)
    kAddVMethod(kValue, kValue, VEquals)
    kAddVMethod(kValue, kValue, VHashCode)
    kAddVMethod(kValue, kValue, VImport)
kEndVirtualValue()

kFx(kBool) kValue_Equals(kType type, const void* value, const void* other)
{
    return kValue_Equals_(type, value, other); 
}

kFx(kBool) kValue_VEquals(kType type, const void* value, const void* other)
{
    kSize fieldCount = kType_FieldCount(type); 
    kSize i, j; 

    for (i = 0; i < fieldCount; ++i)
    {
        const kFieldInfo* field = kType_FieldInfoAt(type, i); 
        
        for (j = 0; j < field->count; ++j)
        {
            kSize offset = field->offset + j*kType_Size_(field->type); 

            if (!kValue_Equals_(field->type, (kByte*)value + offset, (kByte*)other + offset))
            {
                return kFALSE; 
            }
        }
    }

    return kTRUE; 
}

kFx(kSize) kValue_HashCode(kType type, const void* value)
{
    return kValue_HashCode_(type, value); 
}

kFx(kSize) kValue_VHashCode(kType type, const void* value)
{
    kSize fieldCount = kType_FieldCount(type); 
    kSize hash = 1; 
    kSize i, j; 

    for (i = 0; i < fieldCount; ++i)
    {
        const kFieldInfo* field = kType_FieldInfoAt(type, i); 
        
        for (j = 0; j < field->count; ++j)
        {
            kSize offset = field->offset + j*kType_Size_(field->type); 
            
            hash = hash * 31 + kValue_HashCode_(field->type, (kByte*)value + offset); 
        }
    }
    
    return hash; 
}

kFx(void) kValue_Import(kType type, void* value, const void* source)
{
    kValue_Import_(type, value, source); 
}

kFx(void) kValue_VImport(kType type, void* value, const void* source)
{
    kItemCopy_(value, source, kType_Size_(type)); 
}
