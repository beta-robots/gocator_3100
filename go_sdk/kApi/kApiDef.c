/** 
 * @file    kApiDef.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kStream.h>
#include <stdio.h>

kBeginVoidValue(k, kVoid, kValue)
    kAddVersion(kVoid, "kdat6", "5.7.1.0", "kVoid-0", Write, Read)
kEndVoidValue()

kFx(kStatus) kVoid_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kOK; 
}

kFx(kStatus) kVoid_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kOK; 
}

kBeginValue(k, k8u, kValue)
    kAddVersion(k8u, "kdat5", "5.0.0.0", "146-0", Write, Read)
    kAddVersion(k8u, "kdat6", "5.7.1.0", "k8u-0", Write, Read)

    kAddVMethod(k8u, kValue, VEquals)
    kAddVMethod(k8u, kValue, VHashCode)
kEndValue()

kFx(kBool) k8u_VEquals(kType type, const void* value, const void* other)
{
    return *(k8u*)value == *(k8u*)other; 
}

kFx(kSize) k8u_VHashCode(kType type, const void* value)
{
    return (kSize) *(k8u*)value; 
}

kFx(kStatus) k8u_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8uArray_(serializer, values, count); 
}

kFx(kStatus) k8u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8uArray_(serializer, values, count); 
}

kFx(kStatus) k8u_Format(k8u value, kChar* buffer, kSize capacity)
{
    return kStrFormat8u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k8u_Parse(k8u* value, const kChar* str)
{
    return kStrParse8u(value, str);
}

kBeginValue(k, k16u, kValue)
    kAddVersion(k16u, "kdat5", "5.0.0.0", "148-0", Write, Read)
    kAddVersion(k16u, "kdat6", "5.7.1.0", "k16u-0", Write, Read)

    kAddVMethod(k16u, kValue, VEquals)
    kAddVMethod(k16u, kValue, VHashCode)
kEndValue()

kFx(kBool) k16u_VEquals(kType type, const void* value, const void* other)
{
    return *(k16u*)value == *(k16u*)other; 
}

kFx(kSize) k16u_VHashCode(kType type, const void* value)
{
    return (kSize) *(k16u*)value; 
}

kFx(kStatus) k16u_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16uArray_(serializer, values, count); 
}

kFx(kStatus) k16u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16uArray_(serializer, values, count); 
}

kFx(kStatus) k16u_Format(k16u value, kChar* buffer, kSize capacity)
{
    return kStrFormat16u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k16u_Parse(k16u* value, const kChar* str)
{
    return kStrParse16u(value, str);
}

kBeginValue(k, k32u, kValue)
    kAddVersion(k32u, "kdat5", "5.0.0.0", "150-0", Write, Read)
    kAddVersion(k32u, "kdat6", "5.7.1.0", "k32u-0", Write, Read)

    kAddVMethod(k32u, kValue, VEquals)
    kAddVMethod(k32u, kValue, VHashCode)
kEndValue()

kFx(kBool) k32u_VEquals(kType type, const void* value, const void* other)
{
    return *(k32u*)value == *(k32u*)other; 
}

kFx(kSize) k32u_VHashCode(kType type, const void* value)
{
    return (kSize) *(k32u*)value; 
}

kFx(kStatus) k32u_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32uArray_(serializer, values, count); 
}

kFx(kStatus) k32u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32uArray_(serializer, values, count); 
}

kFx(kStatus) k32u_Format(k32u value, kChar* buffer, kSize capacity)
{
    return kStrFormat32u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k32u_Parse(k32u* value, const kChar* str)
{
    return kStrParse32u(value, str);
}

kBeginValue(k, k64u, kValue)
    kAddVersion(k64u, "kdat5", "5.0.0.0", "153-0", Write, Read)
    kAddVersion(k64u, "kdat6", "5.7.1.0", "k64u-0", Write, Read)

    kAddVMethod(k64u, kValue, VEquals)
    kAddVMethod(k64u, kValue, VHashCode)
kEndValue()

kFx(kBool) k64u_VEquals(kType type, const void* value, const void* other)
{
    return *(k64u*)value == *(k64u*)other; 
}

#if (K_POINTER_SIZE == 4)

kFx(kSize) k64u_VHashCode(kType type, const void* value)
{
    const k32u* v = value;
    return v[0] ^ v[1];
}

#elif (K_POINTER_SIZE == 8)

kFx(kSize) k64u_VHashCode(kType type, const void* value)
{
    return *(kSize*)value;
}

#endif

kFx(kStatus) k64u_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64uArray_(serializer, values, count); 
}

kFx(kStatus) k64u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64uArray_(serializer, values, count); 
}

kFx(kStatus) k64u_Format(k64u value, kChar* buffer, kSize capacity)
{
    return kStrFormat64u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k64u_Parse(k64u* value, const kChar* str)
{
    return kStrParse64u(value, str);
}

kBeginValue(k, k8s, kValue)
    kAddVersion(k8s, "kdat5", "5.0.0.0", "147-0", Write, Read)
    kAddVersion(k8s, "kdat6", "5.7.1.0", "k8s-0", Write, Read)

    kAddVMethod(k8s, kValue, VEquals)
    kAddVMethod(k8s, kValue, VHashCode)
kEndValue()

kFx(kBool) k8s_VEquals(kType type, const void* value, const void* other)
{
    return *(k8s*)value == *(k8s*)other; 
}

kFx(kSize) k8s_VHashCode(kType type, const void* value)
{
    return (kSize) *(k8s*)value; 
}

kFx(kStatus) k8s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8sArray_(serializer, values, count); 
}

kFx(kStatus) k8s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8sArray_(serializer, values, count); 
}

kFx(kStatus) k8s_Format(k8s value, kChar* buffer, kSize capacity)
{
    return kStrFormat8s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k8s_Parse(k8s* value, const kChar* str)
{
    return kStrParse8s(value, str);
}

kBeginValue(k, k16s, kValue)
    kAddVersion(k16s, "kdat5", "5.0.0.0", "149-0", Write, Read)
    kAddVersion(k16s, "kdat6", "5.7.1.0", "k16s-0", Write, Read)

    kAddVMethod(k16s, kValue, VEquals)
    kAddVMethod(k16s, kValue, VHashCode)
kEndValue()

kFx(kBool) k16s_VEquals(kType type, const void* value, const void* other)
{
    return *(k16s*)value == *(k16s*)other; 
}

kFx(kSize) k16s_VHashCode(kType type, const void* value)
{
    return (kSize) *(k16s*)value; 
}

kFx(kStatus) k16s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray_(serializer, values, count); 
}

kFx(kStatus) k16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray_(serializer, values, count); 
}

kFx(kStatus) k16s_Format(k16s value, kChar* buffer, kSize capacity)
{
    return kStrFormat16s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k16s_Parse(k16s* value, const kChar* str)
{
    return kStrParse16s(value, str);
}

kBeginValue(k, k32s, kValue)
    kAddVersion(k32s, "kdat5", "5.0.0.0", "151-0", Write, Read)
    kAddVersion(k32s, "kdat6", "5.7.1.0", "k32s-0", Write, Read)

    kAddVMethod(k32s, kValue, VEquals)
    kAddVMethod(k32s, kValue, VHashCode)
kEndValue()

kFx(kBool) k32s_VEquals(kType type, const void* value, const void* other)
{
    return *(k32s*)value == *(k32s*)other; 
}

kFx(kSize) k32s_VHashCode(kType type, const void* value)
{
    return (kSize) *(k32s*)value; 
}

kFx(kStatus) k32s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, count); 
}

kFx(kStatus) k32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, count); 
}

kFx(kStatus) k32s_Format(k32s value, kChar* buffer, kSize capacity)
{
    return kStrFormat32s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k32s_Parse(k32s* value, const kChar* str)
{
    return kStrParse32s(value, str);
}

kBeginValue(k, k64s, kValue)
    kAddVersion(k64s, "kdat5", "5.0.0.0", "154-0", Write, Read)
    kAddVersion(k64s, "kdat6", "5.7.1.0", "k64s-0", Write, Read)

    kAddVMethod(k64s, kValue, VEquals)
    kAddVMethod(k64s, kValue, VHashCode)
kEndValue()

kFx(kBool) k64s_VEquals(kType type, const void* value, const void* other)
{
    return *(k64s*)value == *(k64s*)other; 
}

#if (K_POINTER_SIZE == 4)

kFx(kSize) k64s_VHashCode(kType type, const void* value)
{
    const k32u* v = value; 
    return v[0] ^ v[1]; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kSize) k64s_VHashCode(kType type, const void* value)
{
    return *(kSize*)value;
}

#endif

kFx(kStatus) k64s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64sArray_(serializer, values, count); 
}

kFx(kStatus) k64s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64sArray_(serializer, values, count); 
}

kFx(kStatus) k64s_Format(k64s value, kChar* buffer, kSize capacity)
{
    return kStrFormat64s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k64s_Parse(k64s* value, const kChar* str)
{
    return kStrParse64s(value, str);
}

kBeginValue(k, k32f, kValue)
    kAddVersion(k32f, "kdat5", "5.0.0.0", "152-0", Write, Read)
    kAddVersion(k32f, "kdat6", "5.7.1.0", "k32f-0", Write, Read)

    kAddVMethod(k32f, kValue, VEquals)
    kAddVMethod(k32f, kValue, VHashCode)
kEndValue()

kFx(kBool) k32f_VEquals(kType type, const void* value, const void* other)
{
    return *(k32f*)value == *(k32f*)other; 
}

kFx(kSize) k32f_VHashCode(kType type, const void* value)
{
    const k32u* v = value; 
    return v[0]; 
}

kFx(kStatus) k32f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray_(serializer, values, count); 
}

kFx(kStatus) k32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray_(serializer, values, count); 
}

kFx(kStatus) k32f_Format(k32f value, kChar* buffer, kSize capacity)
{
    return kStrFormat32f(value, buffer, capacity, -1, kNULL);
}

kFx(kStatus) k32f_Parse(k32f* value, const kChar* str)
{
    return kStrParse32f(value, str);
}

kBeginValue(k, k64f, kValue)
    kAddVersion(k64f, "kdat5", "5.0.0.0", "155-0", Write, Read)
    kAddVersion(k64f, "kdat6", "5.7.1.0", "k64f-0", Write, Read)

    kAddVMethod(k64f, kValue, VEquals)
    kAddVMethod(k64f, kValue, VHashCode)
kEndValue()

kFx(kBool) k64f_VEquals(kType type, const void* value, const void* other)
{
    return *(k64f*)value == *(k64f*)other; 
}

#if (K_POINTER_SIZE == 4)

kFx(kSize) k64f_VHashCode(kType type, const void* value)
{
    const k32u* v = value; 
    return (kSize) (v[0] ^ v[1]); 
}

#elif (K_POINTER_SIZE == 8)

kFx(kSize) k64f_VHashCode(kType type, const void* value)
{
    return *(kSize*)value;
}

#endif

kFx(kStatus) k64f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray_(serializer, values, count); 
}

kFx(kStatus) k64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray_(serializer, values, count); 
}

kFx(kStatus) k64f_Format(k64f value, kChar* buffer, kSize capacity)
{
    return kStrFormat64f(value, buffer, capacity, -1, kNULL);
}

kFx(kStatus) k64f_Parse(k64f* value, const kChar* str)
{
    return kStrParse64f(value, str);
}

kBeginValue(k, kByte, kValue)
    kAddVersion(kByte, "kdat5", "5.0.0.0", "157-0", Write, Read)
    kAddVersion(kByte, "kdat6", "5.7.1.0", "kByte-0", Write, Read)

    kAddVMethod(kByte, kValue, VEquals)
    kAddVMethod(kByte, kValue, VHashCode)
kEndValue()

kFx(kBool) kByte_VEquals(kType type, const void* value, const void* other)
{
    return *(kByte*)value == *(kByte*)other; 
}

kFx(kSize) kByte_VHashCode(kType type, const void* value)
{
    return (kSize) *(kByte*)value; 
}

kFx(kStatus) kByte_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteByteArray_(serializer, values, count); 
}

kFx(kStatus) kByte_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadByteArray_(serializer, values, count); 
}

kBeginValue(k, kChar, kValue)
    kAddVersion(kChar, "kdat5", "5.0.0.0", "158-0", Write, Read)
    kAddVersion(kChar, "kdat6", "5.7.1.0", "kChar-0", Write, Read)

    kAddVMethod(kChar, kValue, VEquals)
    kAddVMethod(kChar, kValue, VHashCode)
kEndValue()

kFx(kBool) kChar_VEquals(kType type, const void* value, const void* other)
{
    return *(kChar*)value == *(kChar*)other; 
}

kFx(kSize) kChar_VHashCode(kType type, const void* value)
{
    return (kSize) *(kChar*)value; 
}

kFx(kStatus) kChar_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray_(serializer, values, count); 
}

kFx(kStatus) kChar_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray_(serializer, values, count); 
}

kBeginValue(k, kBool, kValue)
    kAddVersion(kBool, "kdat5", "5.0.0.0", "156-0", Write, Read)
    kAddVersion(kBool, "kdat6", "5.7.1.0", "kBool-0", Write, Read)

    kAddVMethod(kBool, kValue, VEquals)
    kAddVMethod(kBool, kValue, VHashCode)
kEndValue()

kFx(kBool) kBool_VEquals(kType type, const void* value, const void* other)
{
    return *(kBool*)value == *(kBool*)other; 
}

kFx(kSize) kBool_VHashCode(kType type, const void* value)
{
    return (kSize) *(kBool*)value; 
}

kFx(kStatus) kBool_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, count); 
}

kFx(kStatus) kBool_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, count); 
}

kFx(kStatus) kBool_Format(kBool value, kChar* buffer, kSize capacity)
{
    return kStrFormatBool(value, buffer, capacity, kNULL);
}

kFx(kStatus) kBool_Parse(kBool* value, const kChar* str)
{
    return kStrParseBool(value, str);
}

kBeginValue(k, kSize, kValue)
    kAddVersion(kSize, "kdat6", "5.7.1.0", "kSize-0", Write, Read)

    kAddVMethod(kSize, kValue, VEquals)
    kAddVMethod(kSize, kValue, VHashCode)
kEndValue()

kFx(kBool) kSize_VEquals(kType type, const void* value, const void* other)
{
    return *(kSize*)value == *(kSize*)other; 
}

kFx(kSize) kSize_VHashCode(kType type, const void* value)
{
    return *(kSize*)value; 
}
kFx(kStatus) kSize_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteSizeArray_(serializer, values, count); 
}

kFx(kStatus) kSize_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadSizeArray_(serializer, values, count); 
}

kFx(kStatus) kSize_Format(kSize value, kChar* buffer, kSize capacity)
{
    return kStrFormatSize(value, buffer, capacity, kNULL);
}

kFx(kStatus) kSize_Parse(kSize* value, const kChar* str)
{
    return kStrParseSize(value, str);
}

kBeginValue(k, kSSize, kValue)
    kAddVersion(kSSize, "kdat6", "5.7.1.0", "kSSize-0", Write, Read)

    kAddVMethod(kSSize, kValue, VEquals)
    kAddVMethod(kSSize, kValue, VHashCode)
kEndValue()

kFx(kBool) kSSize_VEquals(kType type, const void* value, const void* other)
{
    return *(kSSize*)value == *(kSSize*)other; }

kFx(kSize) kSSize_VHashCode(kType type, const void* value)
{
    return (kSize) *(kSSize*)value; 
}

kFx(kStatus) kSSize_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteSSizeArray_(serializer, values, count); 
}

kFx(kStatus) kSSize_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadSSizeArray_(serializer, values, count); 
}

kFx(kStatus) kSSize_Format(kSSize value, kChar* buffer, kSize capacity)
{
    return kStrFormatSSize(value, buffer, capacity, kNULL);
}

kFx(kStatus) kSSize_Parse(kSSize* value, const kChar* str)
{
    return kStrParseSSize(value, str);
}

kBeginValue(k, kPointer, kValue)
    kAddVMethod(kPointer, kValue, VEquals)
    kAddVMethod(kPointer, kValue, VHashCode)
kEndValue()

kFx(kBool) kPointer_VEquals(kType type, const void* value, const void* other)
{
    return *(kPointer*)value == *(kPointer*)other; 
}

kFx(kSize) kPointer_VHashCode(kType type, const void* value)
{
    return kHashPointer_(*(kPointer*)value); 
}

kBeginValue(k, kFunction, kValue)
    kAddVMethod(kFunction, kValue, VEquals)
    kAddVMethod(kFunction, kValue, VHashCode)
kEndValue()

kFx(kBool) kFunction_VEquals(kType type, const void* value, const void* other)
{
    return *(kFunction*)value == *(kFunction*)other; 
}

kFx(kSize) kFunction_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, sizeof(kFunction)); 
}

kBeginArrayValue(k, kText16, kChar, kValue)
    kAddVersion(kText16, "kdat5", "5.0.0.0", "171-0", Write, Read)
    kAddVersion(kText16, "kdat6", "5.7.1.0", "kText16-0", Write, Read)

    kAddVMethod(kText16, kValue, VEquals)
    kAddVMethod(kText16, kValue, VHashCode)
    kAddVMethod(kText16, kValue, VImport)
kEndValue()

kFx(kBool) kText16_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals(value, other); 
}

kFx(kSize) kText16_VHashCode(kType type, const void* value)
{
    const kChar* it = value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) kText16_VImport(kType type, void* value, const void* source)
{
    kStrCopy(value, sizeof(kText16), source); 
}

kFx(kStatus) kText16_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray_(serializer, values, 16*count); 
}

kFx(kStatus) kText16_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray_(serializer, values, 16*count); 
}

kBeginArrayValue(k, kText32, kChar, kValue)
    kAddVersion(kText32, "kdat5", "5.0.0.0", "172-0", Write, Read)
    kAddVersion(kText32, "kdat6", "5.7.1.0", "kText32-0", Write, Read)

    kAddVMethod(kText32, kValue, VEquals)
    kAddVMethod(kText32, kValue, VHashCode)
    kAddVMethod(kText32, kValue, VImport)
kEndValue()

kFx(kBool) kText32_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals(value, other); 
}

kFx(kSize) kText32_VHashCode(kType type, const void* value)
{
    const kChar* it = value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) kText32_VImport(kType type, void* value, const void* source)
{
    kStrCopy(value, sizeof(kText32), source); 
}

kFx(kStatus) kText32_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray_(serializer, values, 32*count); 
}

kFx(kStatus) kText32_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray_(serializer, values, 32*count); 
}

kBeginArrayValue(k, kText64, kChar, kValue)
    kAddVersion(kText64, "kdat5", "5.0.0.0", "173-0", Write, Read)
    kAddVersion(kText64, "kdat6", "5.7.1.0", "kText64-0", Write, Read)

    kAddVMethod(kText64, kValue, VEquals)
    kAddVMethod(kText64, kValue, VHashCode)
    kAddVMethod(kText64, kValue, VImport)
kEndValue()

kFx(kBool) kText64_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals(value, other); 
}

kFx(kSize) kText64_VHashCode(kType type, const void* value)
{
    const kChar* it = value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) kText64_VImport(kType type, void* value, const void* source)
{
    kStrCopy(value, sizeof(kText64), source); 
}

kFx(kStatus) kText64_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray_(serializer, values, 64*count); 
}

kFx(kStatus) kText64_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray_(serializer, values, 64*count); 
}

kBeginArrayValue(k, kText128, kChar, kValue)
    kAddVersion(kText128, "kdat5", "5.0.0.0", "174-0", Write, Read)
    kAddVersion(kText128, "kdat6", "5.7.1.0", "kText128-0", Write, Read)

    kAddVMethod(kText128, kValue, VEquals)
    kAddVMethod(kText128, kValue, VHashCode)
    kAddVMethod(kText128, kValue, VImport)
kEndValue()

kFx(kBool) kText128_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals(value, other); 
}

kFx(kSize) kText128_VHashCode(kType type, const void* value)
{
    const kChar* it = value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) kText128_VImport(kType type, void* value, const void* source)
{
    kStrCopy(value, sizeof(kText128), source); 
}

kFx(kStatus) kText128_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray_(serializer, values, 128*count); 
}

kFx(kStatus) kText128_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray_(serializer, values, 128*count); 
}

kBeginArrayValue(k, kText256, kChar, kValue)
    kAddVersion(kText256, "kdat5", "5.0.0.0", "175-0", Write, Read)
    kAddVersion(kText256, "kdat6", "5.7.1.0", "kText256-0", Write, Read)

    kAddVMethod(kText256, kValue, VEquals)
    kAddVMethod(kText256, kValue, VHashCode)
    kAddVMethod(kText256, kValue, VImport)
kEndValue()

kFx(kBool) kText256_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals(value, other); 
}

kFx(kSize) kText256_VHashCode(kType type, const void* value)
{
    const kChar* it = value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) kText256_VImport(kType type, void* value, const void* source)
{
    kStrCopy(value, sizeof(kText256), source); 
}

kFx(kStatus) kText256_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray_(serializer, values, 256*count); 
}

kFx(kStatus) kText256_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray_(serializer, values, 256*count); 
}

kBeginEnum(k, kStatus, kValue)

    kAddVersion(kStatus, "kdat6", "5.7.1.0", "kStatus-0", Write, Read)

    kAddEnumerator(kStatus, kERROR_STATE)
    kAddEnumerator(kStatus, kERROR_NOT_FOUND)
    kAddEnumerator(kStatus, kERROR_COMMAND)
    kAddEnumerator(kStatus, kERROR_PARAMETER)
    kAddEnumerator(kStatus, kERROR_UNIMPLEMENTED)
    kAddEnumerator(kStatus, kERROR_MEMORY)
    kAddEnumerator(kStatus, kERROR_TIMEOUT)
    kAddEnumerator(kStatus, kERROR_INCOMPLETE)
    kAddEnumerator(kStatus, kERROR_STREAM)
    kAddEnumerator(kStatus, kERROR_CLOSED)
    kAddEnumerator(kStatus, kERROR_ABORT)
    kAddEnumerator(kStatus, kERROR_ALREADY_EXISTS)
    kAddEnumerator(kStatus, kERROR_NETWORK)
    kAddEnumerator(kStatus, kERROR_HEAP)
    kAddEnumerator(kStatus, kERROR_FORMAT)
    kAddEnumerator(kStatus, kERROR_READ_ONLY)
    kAddEnumerator(kStatus, kERROR_WRITE_ONLY)
    kAddEnumerator(kStatus, kERROR_BUSY)
    kAddEnumerator(kStatus, kERROR_CONFLICT)
    kAddEnumerator(kStatus, kERROR_OS)
    kAddEnumerator(kStatus, kERROR_DEVICE)
    kAddEnumerator(kStatus, kERROR_FULL)
    kAddEnumerator(kStatus, kERROR_IN_PROGRESS)
    kAddEnumerator(kStatus, kERROR)
    kAddEnumerator(kStatus, kOK)

    kAddVMethod(kStatus, kValue, VEquals)
    kAddVMethod(kStatus, kValue, VHashCode)

kEndEnum()

kFx(const kChar*) kStatus_Name(kStatus status)
{
    switch(status)
    {
    case kERROR_STATE:              return "kERROR_STATE";      
    case kERROR_NOT_FOUND:          return "kERROR_NOT_FOUND"; 
    case kERROR_COMMAND:            return "kERROR_COMMAND"; 
    case kERROR_PARAMETER:          return "kERROR_PARAMETER"; 
    case kERROR_UNIMPLEMENTED:      return "kERROR_UNIMPLEMENTED"; 
    case kERROR_MEMORY:             return "kERROR_MEMORY"; 
    case kERROR_TIMEOUT:            return "kERROR_TIMEOUT"; 
    case kERROR_INCOMPLETE:         return "kERROR_INCOMPLETE"; 
    case kERROR_STREAM:             return "kERROR_STREAM"; 
    case kERROR_CLOSED:             return "kERROR_CLOSED"; 
    case kERROR_VERSION:            return "kERROR_VERSION"; 
    case kERROR_ABORT:              return "kERROR_ABORT"; 
    case kERROR_ALREADY_EXISTS:     return "kERROR_ALREADY_EXISTS"; 
    case kERROR_NETWORK:            return "kERROR_NETWORK"; 
    case kERROR_HEAP:               return "kERROR_HEAP"; 
    case kERROR_FORMAT:             return "kERROR_FORMAT"; 
    case kERROR_READ_ONLY:          return "kERROR_READ_ONLY"; 
    case kERROR_WRITE_ONLY:         return "kERROR_WRITE_ONLY"; 
    case kERROR_BUSY:               return "kERROR_BUSY"; 
    case kERROR_CONFLICT:           return "kERROR_CONFLICT"; 
    case kERROR_OS:                 return "kERROR_OS"; 
    case kERROR_DEVICE:             return "kERROR_DEVICE"; 
    case kERROR_FULL:               return "kERROR_FULL"; 
    case kERROR_IN_PROGRESS:        return "kERROR_IN_PROGRESS"; 
    case kERROR:                    return "kERROR"; 
    case kOK:                       return "kOK"; 
    default:                        return "Unknown";        
    }
}

kFx(kBool) kStatus_VEquals(kType type, const void* value, const void* other)
{
    return *(kStatus*)value == *(kStatus*)other; 
}

kFx(kSize) kStatus_VHashCode(kType type, const void* value)
{
    return (kSize) *(kStatus*)value; 
}

kFx(kStatus) kStatus_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, count); 
}

kFx(kStatus) kStatus_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, count); 
}

kBeginValue(k, kVersion, kValue)
    kAddVersion(kVersion, "kdat6", "5.7.1.0", "kVersion-0", Write, Read)

    kAddVMethod(kVersion, kValue, VEquals)
    kAddVMethod(kVersion, kValue, VHashCode)
kEndValue()

kFx(kVersion) kVersion_Create(k32u major, k32u minor, k32u release, k32u build)
{
    return kVersion_Create_(major, minor, release, build); 
}

kFx(kStatus) kVersion_Parse(kVersion* version, const kChar* buffer)
{
    k32u major, minor, release, build; 

    kCheck(sscanf(buffer, "%u.%u.%u.%u", &major, &minor, &release, &build) == 4); 

    *version = kVersion_Create_(major, minor, release, build); 

    return kOK; 
}

kFx(kStatus) kVersion_Format(kVersion version, kChar* buffer, kSize capacity)
{
    return kStrPrintf(buffer, capacity, "%u.%u.%u.%u", (k32u)kVersion_Major(version), 
        (k32u)kVersion_Minor(version), (k32u)kVersion_Release(version), (k32u)kVersion_Build(version)); 
}

kFx(k32s) kVersion_Compare(kVersion version1, kVersion version2)
{
    return version1 - version2; 
}

//K_COMPAT_5
kFx(kStatus) kVersion_Fields(kVersion version, k32u* major, k32u* minor, k32u* release, k32u* build)
{
    *major = kVersion_Major(version);
    *minor = kVersion_Minor(version);
    *release = kVersion_Release(version);
    *build = kVersion_Build(version);

    return kOK; 
}

kFx(k8u) kVersion_Major(kVersion version)
{
    return (version >> 24) & 0xFF; 
}

kFx(k8u) kVersion_Minor(kVersion version)
{
    return (version >> 16) & 0xFF; 
}

kFx(k8u) kVersion_Release(kVersion version)
{
    return (version >> 8) & 0xFF; 
}

kFx(k8u) kVersion_Build(kVersion version)
{
    return (version) & 0xFF; 
}

kFx(kBool) kVersion_VEquals(kType type, const void* value, const void* other)
{
    return *(kVersion*)value == *(kVersion*)other; 
}

kFx(kSize) kVersion_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, sizeof(kVersion)); 
}

kFx(kStatus) kVersion_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32uArray_(serializer, values, count); 
}

kFx(kStatus) kVersion_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32uArray_(serializer, values, count); 
}

kBeginValue(k, kPoint16s, kValue)
    kAddVersion(kPoint16s, "kdat5", "5.0.0.0", "161-0", Write, Read)
    kAddVersion(kPoint16s, "kdat6", "5.7.1.0", "kPoint16s-0", Write, Read)

    kAddField(kPoint16s, k16s, x)
    kAddField(kPoint16s, k16s, y)
kEndValue()

kFx(kBool) kPoint16s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint16s* a = value; 
    const kPoint16s* b = other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) kPoint16s_VHashCode(kType type, const void* value)
{
    const kPoint16s* v = value; 
    
    return (kSize) (v->x ^ v->y); 
}

kFx(kStatus) kPoint16s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray_(serializer, values, 2*count); 
}

kFx(kStatus) kPoint16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray_(serializer, values, 2*count); 
}

kBeginValue(k, kPoint32s, kValue)
    kAddVersion(kPoint32s, "kdat5", "5.0.0.0", "162-0", Write, Read)
    kAddVersion(kPoint32s, "kdat6", "5.7.1.0", "kPoint32s-0", Write, Read)

    kAddField(kPoint32s, k32s, x)
    kAddField(kPoint32s, k32s, y)
kEndValue()

kFx(kBool) kPoint32s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint32s* a = value; 
    const kPoint32s* b = other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) kPoint32s_VHashCode(kType type, const void* value)
{
    const kPoint32s* v = value; 
    return (kSize) (v->x ^ v->y); 
}

kFx(kStatus) kPoint32s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, 2*count); 
}

kFx(kStatus) kPoint32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, 2*count); 
}

kBeginValue(k, kPoint32f, kValue)
    kAddVersion(kPoint32f, "kdat5", "5.0.0.0", "163-0", Write, Read)
    kAddVersion(kPoint32f, "kdat6", "5.7.1.0", "kPoint32f-0", Write, Read)

    kAddField(kPoint32f, k32f, x)
    kAddField(kPoint32f, k32f, y)
kEndValue()

kFx(kBool) kPoint32f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint32f* a = value; 
    const kPoint32f* b = other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) kPoint32f_VHashCode(kType type, const void* value)
{
    const kPoint32f* v = value;   
    const k32u* xu = (void*) &v->x; 
    const k32u* yu = (void*) &v->y; 
    
    return (kSize) (xu[0] ^ yu[0]); 
}

kFx(kStatus) kPoint32f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray_(serializer, values, 2*count); 
}

kFx(kStatus) kPoint32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray_(serializer, values, 2*count); 
}

kBeginValue(k, kPoint64f, kValue)
    kAddVersion(kPoint64f, "kdat5", "5.0.0.0", "165-0", Write, Read)
    kAddVersion(kPoint64f, "kdat6", "5.7.1.0", "kPoint64f-0", Write, Read)

    kAddField(kPoint64f, k64f, x)
    kAddField(kPoint64f, k64f, y)
kEndValue()

kFx(kBool) kPoint64f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint64f* a = value; 
    const kPoint64f* b = other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) kPoint64f_VHashCode(kType type, const void* value)
{
    const kPoint64f* v = value;    
    const k32u* xu = (void*) &v->x; 
    const k32u* yu = (void*) &v->y; 
    
    return (kSize) (xu[0] ^ xu[1] ^ yu[0] ^ yu[1]); 
}

kFx(kStatus) kPoint64f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray_(serializer, values, 2*count); 
}

kFx(kStatus) kPoint64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray_(serializer, values, 2*count); 
}

kBeginValue(k, kPoint3d16s, kValue)
    kAddVersion(kPoint3d16s, "kdat5", "5.0.0.0", "194-0", Write, Read)
    kAddVersion(kPoint3d16s, "kdat6", "5.7.1.0", "kPoint3d16s-0", Write, Read)

    kAddField(kPoint3d16s, k16s, x)
    kAddField(kPoint3d16s, k16s, y) 
    kAddField(kPoint3d16s, k16s, z) 
kEndValue()

kFx(kBool) kPoint3d16s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d16s* a = value; 
    const kPoint3d16s* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->x == b->z); 
}

kFx(kSize) kPoint3d16s_VHashCode(kType type, const void* value)
{
    const kPoint3d16s* v = value; 
    
    return (kSize) (v->x ^ v->y ^ v->z); 
}

kFx(kStatus) kPoint3d16s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray_(serializer, values, 3*count); 
}

kFx(kStatus) kPoint3d16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray_(serializer, values, 3*count); 
}

kBeginValue(k, kPoint3d32s, kValue)
    kAddVersion(kPoint3d32s, "kdat5", "5.0.0.0", "164-0", Write, Read)
    kAddVersion(kPoint3d32s, "kdat6", "5.7.1.0", "kPoint3d32s-0", Write, Read)

    kAddField(kPoint3d32s, k32s, x)
    kAddField(kPoint3d32s, k32s, y)
    kAddField(kPoint3d32s, k32s, z) 
kEndValue()

kFx(kBool) kPoint3d32s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d32s* a = value; 
    const kPoint3d32s* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) kPoint3d32s_VHashCode(kType type, const void* value)
{
    const kPoint3d32s* v = value; 
    return (kSize) (v->x ^ v->y ^ v->z); 
}

kFx(kStatus) kPoint3d32s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, 3*count); 
}

kFx(kStatus) kPoint3d32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, 3*count); 
}

kBeginValue(k, kPoint3d32f, kValue)
    kAddVersion(kPoint3d32f, "kdat5", "5.0.0.0", "195-0", Write, Read)
    kAddVersion(kPoint3d32f, "kdat6", "5.7.1.0", "kPoint3d32f-0", Write, Read)

    kAddField(kPoint3d32f, k32f, x)
    kAddField(kPoint3d32f, k32f, y)
    kAddField(kPoint3d32f, k32f, z) 
kEndValue()

kFx(kBool) kPoint3d32f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d32f* a = value; 
    const kPoint3d32f* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) kPoint3d32f_VHashCode(kType type, const void* value)
{
    const kPoint3d32f* v = value;    
    const k32u* xu = (void*) &v->x; 
    const k32u* yu = (void*) &v->y; 
    const k32u* zu = (void*) &v->z; 
       
    return (kSize) (xu[0] ^ yu[0] ^ zu[1]); 
}

kFx(kStatus) kPoint3d32f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray_(serializer, values, 3*count); 
}

kFx(kStatus) kPoint3d32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray_(serializer, values, 3*count); 
}

kBeginValue(k, kPoint3d64f, kValue)
    kAddVersion(kPoint3d64f, "kdat5", "5.0.0.0", "196-0", Write, Read)
    kAddVersion(kPoint3d64f, "kdat6", "5.7.1.0", "kPoint3d64f-0", Write, Read)

    kAddField(kPoint3d64f, k64f, x)
    kAddField(kPoint3d64f, k64f, y)
    kAddField(kPoint3d64f, k64f, z) 
kEndValue()

kFx(kBool) kPoint3d64f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d64f* a = value; 
    const kPoint3d64f* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) kPoint3d64f_VHashCode(kType type, const void* value)
{
    const kPoint3d64f* v = value;    
    const k32u* xu = (void*) &v->x; 
    const k32u* yu = (void*) &v->y; 
    const k32u* zu = (void*) &v->z; 
    
    return (kSize) (xu[0] ^ xu[1] ^ yu[0] ^ yu[1] ^ zu[0] ^ zu[1]); 
}

kFx(kStatus) kPoint3d64f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray_(serializer, values, 3*count); 
}

kFx(kStatus) kPoint3d64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray_(serializer, values, 3*count); 
}

kBeginValue(k, kRect16s, kValue)
    kAddVersion(kRect16s, "kdat6", "5.7.1.0", "kRect16s-0", Write, Read)

    kAddField(kRect16s, k16s, x)
    kAddField(kRect16s, k16s, y)
    kAddField(kRect16s, k16s, width)
    kAddField(kRect16s, k16s, height)
kEndValue()

kFx(kBool) kRect16s_VEquals(kType type, const void* value, const void* other)
{
    const kRect16s* a = value; 
    const kRect16s* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}

kFx(kSize) kRect16s_VHashCode(kType type, const void* value)
{
    const kRect16s* v = value; 

    return (kSize) (v->x ^ v->y ^ v->width ^ v->height); 
}

kFx(kStatus) kRect16s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray_(serializer, values, 4*count); 
}

kFx(kStatus) kRect16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray_(serializer, values, 4*count); 
}

kBeginValue(k, kRect32s, kValue)
    kAddVersion(kRect32s, "kdat5", "5.0.0.0", "166-0", Write, Read)
    kAddVersion(kRect32s, "kdat6", "5.7.1.0", "kRect32s-0", Write, Read)

    kAddField(kRect32s, k32s, x)
    kAddField(kRect32s, k32s, y)
    kAddField(kRect32s, k32s, width)
    kAddField(kRect32s, k32s, height)
kEndValue()

kFx(kBool) kRect32s_VEquals(kType type, const void* value, const void* other)
{
    const kRect32s* a = value; 
    const kRect32s* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}
 
kFx(kSize) kRect32s_VHashCode(kType type, const void* value)
{
    const kRect32s* v = value; 

    return (kSize) (v->x ^ v->y ^ v->width ^ v->height); 
}

kFx(kStatus) kRect32s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, 4*count); 
}

kFx(kStatus) kRect32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, 4*count); 
}

kBeginValue(k, kRect32f, kValue)
    kAddVersion(kRect32f, "kdat5", "5.0.0.0", "167-0", Write, Read)
    kAddVersion(kRect32f, "kdat6", "5.7.1.0", "kRect32f-0", Write, Read)

    kAddField(kRect32f, k32f, x)
    kAddField(kRect32f, k32f, y)
    kAddField(kRect32f, k32f, width)
    kAddField(kRect32f, k32f, height)
kEndValue()

kFx(kBool) kRect32f_VEquals(kType type, const void* value, const void* other)
{
    const kRect32f* a = value; 
    const kRect32f* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}
 
kFx(kSize) kRect32f_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, kType_Size_(type)); 
}

kFx(kStatus) kRect32f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray_(serializer, values, 4*count); 
}

kFx(kStatus) kRect32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray_(serializer, values, 4*count); 
}

kBeginValue(k, kRect64f, kValue)
    kAddVersion(kRect64f, "kdat5", "5.0.0.0", "168-0", Write, Read)
    kAddVersion(kRect64f, "kdat6", "5.7.1.0", "kRect64f-0", Write, Read)

    kAddField(kRect64f, k64f, x)
    kAddField(kRect64f, k64f, y)
    kAddField(kRect64f, k64f, width)
    kAddField(kRect64f, k64f, height)
kEndValue()

kFx(kBool) kRect64f_VEquals(kType type, const void* value, const void* other)
{
    const kRect64f* a = value; 
    const kRect64f* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}
 
kFx(kSize) kRect64f_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, kType_Size_(type)); 
}

kFx(kStatus) kRect64f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray_(serializer, values, 4*count); 
}

kFx(kStatus) kRect64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray_(serializer, values, 4*count); 
}

kBeginValue(k, kRect3d64f, kValue)
    kAddVersion(kRect3d64f, "kdat6", "6.0.14.0", "kRect3d64f-0", Write, Read)

    kAddField(kRect3d64f, k64f, x)
    kAddField(kRect3d64f, k64f, y)
    kAddField(kRect3d64f, k64f, z)
    kAddField(kRect3d64f, k64f, width)
    kAddField(kRect3d64f, k64f, height)
    kAddField(kRect3d64f, k64f, depth)
kEndValue()

kFx(kBool) kRect3d64f_VEquals(kType type, const void* value, const void* other)
{
    const kRect3d64f* a = value; 
    const kRect3d64f* b = other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z) &&
           (a->width == b->width) && (a->height == b->height) && (a->depth == b->depth); 
}
 
kFx(kSize) kRect3d64f_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, kType_Size_(type)); 
}

kFx(kStatus) kRect3d64f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray_(serializer, values, 6*count); 
}

kFx(kStatus) kRect3d64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray_(serializer, values, 6*count); 
}

kBeginValue(k, kRotatedRect32s, kValue)
    kAddVersion(kRotatedRect32s, "kdat5", "5.0.0.0", "169-0", Write, Read)
    kAddVersion(kRotatedRect32s, "kdat6", "5.7.1.0", "kRotatedRect32s-0", Write, Read)

    kAddField(kRotatedRect32s, k32s, xc)
    kAddField(kRotatedRect32s, k32s, yc)
    kAddField(kRotatedRect32s, k32s, width)
    kAddField(kRotatedRect32s, k32s, height)
    kAddField(kRotatedRect32s, k32s, angle)
kEndValue()

kFx(kBool) kRotatedRect32s_VEquals(kType type, const void* value, const void* other)
{
    const kRotatedRect32s* a = value; 
    const kRotatedRect32s* b = other; 

    return (a->xc == b->xc) && (a->yc == b->yc) && (a->width == b->width) && (a->height == b->height) && (a->angle == b->angle); 
}
 
kFx(kSize) kRotatedRect32s_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, kType_Size_(type)); 
}

kFx(kStatus) kRotatedRect32s_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, 5*count); 
}

kFx(kStatus) kRotatedRect32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, 5*count); 
}

kBeginValue(k, kRotatedRect32f, kValue)
    kAddVersion(kRotatedRect32f, "kdat5", "5.0.0.0", "170-0", Write, Read)
    kAddVersion(kRotatedRect32f, "kdat6", "5.7.1.0", "kRotatedRect32f-0", Write, Read)

    kAddField(kRotatedRect32f, k32f, xc)
    kAddField(kRotatedRect32f, k32f, yc)
    kAddField(kRotatedRect32f, k32f, width)
    kAddField(kRotatedRect32f, k32f, height)
    kAddField(kRotatedRect32f, k32f, angle)
kEndValue()

kFx(kBool) kRotatedRect32f_VEquals(kType type, const void* value, const void* other)
{
    const kRotatedRect32f* a = value; 
    const kRotatedRect32f* b = other; 

    return (a->xc == b->xc) && (a->yc == b->yc) && (a->width == b->width) && (a->height == b->height) && (a->angle == b->angle); 
}
 
kFx(kSize) kRotatedRect32f_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, kType_Size_(type)); 
}

kFx(kStatus) kRotatedRect32f_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray_(serializer, values, 5*count); 
}

kFx(kStatus) kRotatedRect32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray_(serializer, values, 5*count); 
}

kBeginEnum(k, kPixelFormat, kValue)
    kAddVersion(kPixelFormat, "kdat6", "5.7.1.0", "kPixelFormat-0", Write, Read)

    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_NULL)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_8BPP_GREYSCALE)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_8BPP_CFA)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_8BPC_BGRX)
kEndEnum()

kFx(kBool) kPixelFormat_VEquals(kType type, const void* value, const void* other)
{
    return *(kPixelFormat*)value == *(kPixelFormat*)other; 
}

kFx(kSize) kPixelFormat_VHashCode(kType type, const void* value)
{
    return (kSize) *(kPixelFormat*)value; 
}

kFx(kStatus) kPixelFormat_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, count); 
}

kFx(kStatus) kPixelFormat_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, count); 
}

kBeginEnum(k, kCfa, kValue)
    kAddVersion(kCfa, "kdat6", "5.7.1.0", "kCfa-0", Write, Read)

    kAddEnumerator(kCfa, kCFA_NONE)
    kAddEnumerator(kCfa, kCFA_BAYER_BGGR)
    kAddEnumerator(kCfa, kCFA_BAYER_GBRG)
    kAddEnumerator(kCfa, kCFA_BAYER_RGGB)
    kAddEnumerator(kCfa, kCFA_BAYER_GRBG)
kEndEnum()

kFx(kBool) kCfa_VEquals(kType type, const void* value, const void* other)
{
    return *(kCfa*)value == *(kCfa*)other; 
}

kFx(kSize) kCfa_VHashCode(kType type, const void* value)
{
    return (kSize) *(kCfa*)value; 
}

kFx(kStatus) kCfa_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, count); 
}

kFx(kStatus) kCfa_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, count); 
}

kBeginValue(k, kRgb, kValue)
    kAddVersion(kRgb, "kdat5", "5.0.0.0", "179-0", Write, Read)
    kAddVersion(kRgb, "kdat6", "5.7.1.0", "kRgb-0", Write, Read)

    kAddField(kRgb, k8u, b)
    kAddField(kRgb, k8u, g)
    kAddField(kRgb, k8u, r)
    kAddField(kRgb, k8u, x)
kEndValue()

kFx(kBool) kRgb_VEquals(kType type, const void* value, const void* other)
{
    const kRgb* a = value; 
    const kRgb* b = other; 

    return (a->b == b->b) && (a->g == b->g) && (a->r == b->r); 
}

kFx(kSize) kRgb_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, 3); 
}

kFx(kStatus) kRgb_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8uArray_(serializer, values, 4*count); 
}

kFx(kStatus) kRgb_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8uArray_(serializer, values, 4*count); 
}

kBeginValue(k, kArgb, kValue)
    kAddVersion(kArgb, "kdat5", "5.0.0.0", "180-0", Write, Read)
    kAddVersion(kArgb, "kdat6", "5.7.1.0", "kArgb-0", Write, Read)

    kAddField(kArgb, k8u, b)
    kAddField(kArgb, k8u, g)
    kAddField(kArgb, k8u, r)
    kAddField(kArgb, k8u, a)
kEndValue()

kFx(kBool) kArgb_VEquals(kType type, const void* value, const void* other)
{
    const kArgb* a = value; 
    const kArgb* b = other; 

    return (a->b == b->b) && (a->g == b->g) && (a->r == b->r) && (a->a == b->a); 
}

kFx(kSize) kArgb_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, kType_Size_(type)); 
}

kFx(kStatus) kArgb_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8uArray_(serializer, values, 4*count); 
}

kFx(kStatus) kArgb_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8uArray_(serializer, values, 4*count); 
}

kBeginEnum(k, kComparison, kValue)
    kAddEnumerator(kComparison, kCOMPARISON_EQ)
    kAddEnumerator(kComparison, kCOMPARISON_NEQ)
    kAddEnumerator(kComparison, kCOMPARISON_LT)
    kAddEnumerator(kComparison, kCOMPARISON_LTE)
    kAddEnumerator(kComparison, kCOMPARISON_GT)
    kAddEnumerator(kComparison, kCOMPARISON_GTE)
kEndValue()

kFx(kBool) kComparison_VEquals(kType type, const void* value, const void* other)
{
    return *(kComparison*)value == *(kComparison*)other; 
}

kFx(kSize) kComparison_VHashCode(kType type, const void* value)
{
    return (kSize)*(kComparison*)value; 
}

kFx(kStatus) kComparison_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray_(serializer, values, count); 
}

kFx(kStatus) kComparison_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray_(serializer, values, count); 
}

kBeginValue(k, kCallbackFx, kValue)
    kAddVMethod(kCallbackFx, kValue, VEquals)
    kAddVMethod(kCallbackFx, kValue, VHashCode)
kEndValue()

kFx(kBool) kCallbackFx_VEquals(kType type, const void* value, const void* other)
{
    return *(kCallbackFx*)value == *(kCallbackFx*)other; 
}

kFx(kSize) kCallbackFx_VHashCode(kType type, const void* value)
{
    return kHashBytes(value, sizeof(kCallbackFx)); 
}

kBeginValue(k, kCallback, kValue)
    kAddField(kCallback, kCallbackFx, function);  
    kAddField(kCallback, kPointer, receiver);  

    kAddVMethod(kCallback, kValue, VEquals)
    kAddVMethod(kCallback, kValue, VHashCode)
kEndValue()

kFx(kBool) kCallback_VEquals(kType type, const void* value, const void* other)
{
    const kCallback* a = value; 
    const kCallback* b = other; 
    
    return (a->function == b->function) && (a->receiver == b->receiver); 
}

kFx(kSize) kCallback_VHashCode(kType type, const void* value)
{
    const kCallback* v = value; 

    return kValue_HashCode_(kTypeOf(kCallbackFx), &v->function) ^ 
           kValue_HashCode_(kTypeOf(kPointer), &v->receiver); 
}

kBeginBitEnum(k, kFileMode, kValue)
    kAddEnumerator(kFileMode, kFILE_MODE_READ)
    kAddEnumerator(kFileMode, kFILE_MODE_WRITE)
    kAddEnumerator(kFileMode, kFILE_MODE_UPDATE)
kEndBitEnum()

kBeginEnum(k, kSeekOrigin, kValue)
    kAddEnumerator(kSeekOrigin, kSEEK_ORIGIN_BEGIN)
    kAddEnumerator(kSeekOrigin, kSEEK_ORIGIN_CURRENT)
    kAddEnumerator(kSeekOrigin, kSEEK_ORIGIN_END)
kEndEnum()
