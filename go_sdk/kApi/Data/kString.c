/** 
 * @file    kString.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kString.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Io/kSerializer.h>
#include <stdio.h>
#include <ctype.h>

kBeginClass(k, kString, kObject) 
    
    //serialization versions
    kAddVersion(kString, "kdat5", "5.0.0.0", "43-1", WriteDat5V1, ReadDat5V1)
    kAddVersion(kString, "kdat6", "5.7.1.0", "kString-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kString, kObject, VRelease)
    kAddVMethod(kString, kObject, VInitClone)
    kAddVMethod(kString, kObject, VHashCode)
    kAddVMethod(kString, kObject, VEquals)
    kAddVMethod(kString, kObject, VSize)

kEndClass() 

kFx(kStatus) kString_Construct(kString* str, const kChar* content, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kString), str)); 

    if (!kSuccess(status = kString_Init(*str, kTypeOf(kString), content, alloc)))
    {
        kAlloc_FreeRef(alloc, str); 
    }

    return status; 
} 

kFx(kStatus) kString_Init(kString str, kType classType, const kChar* content, kAlloc allocator)
{
    kStringClass* obj = str; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(str, classType, allocator)); 

    obj->allocSize = 0; 
    obj->chars = &obj->nullStr; 
    obj->length = 0; 
    obj->capacity = 0; 
    obj->nullStr = 0; 

    kTry
    {
        kTest(kString_Import(str, content, kFALSE)); 
    }
    kCatch(&status)
    {
        kString_VRelease(str); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kString_VInitClone(kString str, kString source, kAlloc allocator)
{
    kStringClass* obj = str; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(str, kObject_Type_(source), allocator)); 

    obj->allocSize = 0; 
    obj->chars = &obj->nullStr; 
    obj->length = 0; 
    obj->capacity = 0; 
    obj->nullStr = 0; 

    kTry
    {
        kTest(kString_Import(str, kString_Chars(source), kFALSE)); 
    }
    kCatch(&status)
    {
        kString_VRelease(str); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kString_VRelease(kString str)
{
    kStringClass* obj = kString_Cast_(str); 

    if (obj->allocSize > 0)
    {
        kCheck(kObject_FreeMem_(str, obj->chars)); 
    }

    kCheck(kObject_VRelease_(str)); 

    return kOK; 
}

kFx(kSize) kString_VSize(kString str)
{
    kStringClass* obj = kString_Cast_(str); 
    kSize size = sizeof(kStringClass) + obj->allocSize; 

    return size; 
}

kFx(kSize) kString_VHashCode(kString str)
{
    kStringClass* obj = kString_Cast_(str); 
    const kChar* it = kString_Chars_(obj); 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(kBool) kString_VEquals(kString str, kObject other)
{
    kAssertType(str, kString); 

    if (kObject_Type_(other) == kTypeOf(kString))
    {
        return kStrEquals(kString_Chars_(str), kString_Chars_(other)); 
    }

    return kFALSE; 
}

kFx(kStatus) kString_WriteDat5V1(kString str, kSerializer serializer)
{
    kStringClass* obj = kString_Cast_(str); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize_(serializer, obj->capacity)); 
    kCheck(kSerializer_Write32s_(serializer, kTRUE));  //isDynamic

    kCheck(kSerializer_WriteSize_(serializer, obj->length)); 
    kCheck(kSerializer_WriteType_(serializer, kTypeOf(kChar), &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, kTypeOf(kChar), itemVersion, obj->chars, obj->length)); 

    return kOK; 
}

kFx(kStatus) kString_ReadDat5V1(kString str, kSerializer serializer, kAlloc allocator)
{
    kStringClass *obj = str; 
    kSize capacity = 0, length = 0; 
    kTypeVersion itemVersion;
    k32s isDynamic; 
    kType itemType = kNULL;            
    kStatus status; 

    kCheck(kSerializer_ReadSize_(serializer, &capacity));      
    kCheck(kSerializer_Read32s_(serializer, &isDynamic));

    kCheck(kSerializer_ReadSize_(serializer, &length));
    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion)); 

    kCheck(kString_Init(str, kTypeOf(kString), kNULL, allocator)); 

    kTry
    {
        kTest(kString_Reserve(str, length)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->chars, length)); 
        obj->length = length; 
        obj->chars[length] = 0; 
    }
    kCatch(&status)
    {
        kString_VRelease(str); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kString_WriteDat6V0(kString str, kSerializer serializer)
{
    kStringClass* obj = kString_Cast_(str); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType_(serializer, kTypeOf(kChar), &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->length)); 
    kCheck(kSerializer_WriteItems(serializer, kTypeOf(kChar), itemVersion, obj->chars, obj->length)); 

    return kOK; 
}

kFx(kStatus) kString_ReadDat6V0(kString str, kSerializer serializer, kAlloc allocator)
{
    kStringClass *obj = str; 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length = 0; 
    kStatus status; 

    kCheck(kSerializer_ReadType_(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &length)); 

    kCheck(kString_Init(str, kTypeOf(kString), kNULL, allocator)); 

    kTry
    {
        kTest(kString_Reserve(str, length)); 

        kTest(kSerializer_ReadItems_(serializer, itemType, itemVersion, obj->chars, length)); 

        obj->length = length; 
        obj->chars[length] = 0; 
    }
    kCatch(&status)
    {
        kString_VRelease(str); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kString_Assign(kString str, kString source)
{
    return kString_Import(str, kString_Chars(source), kFALSE); 
}

kFx(kStatus) kString_Clear(kString str)
{
    kStringClass* obj = kString_Cast_(str); 
    
    obj->chars[0] = 0; 
    obj->length = 0; 

    return kOK; 
}

kFx(kStatus) kString_Reserve(kString str, kSize minimumCapacity)
{
    kStringClass* obj = kString_Cast_(str); 
    kType itemType = kTypeOf(kChar); 
    kSize itemSize = kType_Size_(itemType); 

    if (obj->capacity < minimumCapacity)
    {
        kSize grownCapacity = kMax_(minimumCapacity, kSTRING_GROWTH_FACTOR*obj->capacity);
        kSize newCapacity = kMax_(grownCapacity, kSTRING_MIN_CAPACITY); 
        kSize newSize = kMax_(obj->allocSize, (newCapacity+1)*itemSize);    //+1 for null terminator

        if (newSize > obj->allocSize)
        {
            kChar* oldChars = obj->chars; 
            kChar* newChars = kNULL; 

            kCheck(kObject_GetMem_(str, newSize, &newChars)); 

            if (obj->capacity > 0)
            {
                kMemCopy(newChars, oldChars, (obj->capacity+1)*itemSize);    //+1 for null terminator
            }

            obj->chars = newChars; 
            obj->allocSize = newSize; 

            if (oldChars != &obj->nullStr)
            {
                kCheck(kObject_FreeMem_(str, oldChars)); 
            }
        }

        obj->capacity = newCapacity;
    }
    
    return kOK; 
}

kFx(kStatus) kString_SetLength(kString str, kSize length)
{
    kStringClass* obj = kString_Cast_(str);

    if (length > obj->capacity)
    {
        kCheck(kString_Reserve(str, length));
    }

    obj->length = length;
    obj->chars[length] = 0;

    return kOK;
}

kFx(k32s) kString_Compare(kString str, const kChar* content)
{
    kStringClass* obj = kString_Cast_(str); 

    return kStrCompare(kString_Chars_(obj), content); 
}

kFx(kBool) kString_Equals(kString str, const kChar* content)
{
    kStringClass* obj = kString_Cast_(str); 

    return kStrEquals(kString_Chars_(obj), content); 
}

kFx(kStatus) kString_Trim(kString str)
{
    kStringClass* obj = kString_Cast_(str); 
    kSize it = 0; 

    //remove leading
    while (isspace(obj->chars[it]))
    {
        it++; 
    }

    kCheck(kMemMove(obj->chars, &obj->chars[it], obj->length + 1 - it)); 
    obj->length -= it; 

    //remove trailing
    if (obj->length > 0)
    {
        it = obj->length - 1; 

        while ((it > 0) && isspace(obj->chars[it]))
        {
            it--; 
        }

        obj->length = it + 1; 
        obj->chars[obj->length] = 0; 
    }

    return kOK; 
}

kFx(kStatus) kString_Setf(kString str, const kChar* format, ...)
{
    kVarArgList argList; 
    kStatus status; 
   
    kVarArgList_Start_(argList, format);
    {
        status = kString_Setvf(str, format, argList); 
    }
    kVarArgList_End_(argList);
    
    return status; 
}

kFx(kStatus) kString_Setvf(kString str, const kChar* format, kVarArgList argList)
{
    kStringClass* obj = kString_Cast_(str); 
    kVarArgList argListCopy; 
    k32s charCount; 

    kVarArgList_Copy_(argListCopy, argList);
    {
        charCount = kStrMeasuref(format, argListCopy);  
    }
    kVarArgList_End_(argListCopy);

    kCheckArgs(charCount >= 0); 

    kCheck(kString_Reserve(str, (kSize)charCount)); 
            
    if (vsprintf(obj->chars, format, argList) < 0)
    {
        return kERROR; 
    }

    obj->length = (kSize) charCount; 

    return kOK; 
}

kFx(kStatus) kString_Set(kString str, const kChar* content)
{
    return kString_Import(str, content, kFALSE);   
}

kFx(kStatus) kString_Addf(kString str, const kChar* format, ...)
{
    kVarArgList argList;  
    kStatus status; 
   
    kVarArgList_Start_(argList, format);
    {
        status = kString_Addvf(str, format, argList); 
    }
    kVarArgList_End_(argList);
 
    return status;   
}

kFx(kStatus) kString_Addvf(kString str, const kChar* format, kVarArgList argList)
{
    kStringClass* obj = kString_Cast_(str); 
    kSize length = kString_Length_(str);    
    kVarArgList argListCopy; 
    k32s charCount; 

    kVarArgList_Copy_(argListCopy, argList);
    {
        charCount = kStrMeasuref(format, argListCopy);  
    }
    kVarArgList_End_(argListCopy);

    kCheckArgs(charCount >= 0); 

    kCheck(kString_Reserve(str, length + (kSize)charCount)); 
        
    if (vsprintf(obj->chars + length, format, argList) < 0)
    {
        return kERROR; 
    }

    obj->length = length + (kSize)charCount; 

    return kOK; 
}

kFx(kStatus) kString_Add(kString str, const kChar* content)
{
    return kString_Import(str, content, kTRUE);   
}

kFx(kStatus) kString_AddSubstring(kString str, const kChar* content, kSize start, kSize count)
{
    kStringClass* obj = kString_Cast_(str); 
    kSize newLength = obj->length + count;

    kCheck(kString_Reserve(str, newLength));
    
    kMemCopy(obj->chars + obj->length, content + start, count * sizeof(kChar));

    obj->chars[newLength] = 0; 
    obj->length = newLength;

    return kOK;
}

kFx(kStatus) kString_SplitAdd(const kChar* start, kSize length, kArrayList tokens, kAlloc allocator)
{
    kString token = kNULL; 
    kStatus exception;

    kTry
    {
        kTest(kString_Construct(&token, kNULL, allocator));
        kTest(kString_AddSubstring(token, start, 0, length)); 

        kTest(kArrayList_Add(tokens, &token)); 
    }
    kCatch(&exception)
    {
        kObject_Destroy(token); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kString_Split(kString str, const kChar* delimiters, kArrayList* tokens, kAlloc allocator)
{
    kStringClass* obj = kString_Cast_(str); 
    const kChar* start = kString_Chars(obj); 
    const kChar* it = start;
    kArrayList output = kNULL; 
    kStatus exception;
    
    kTry
    {
        kTest(kArrayList_Construct(&output, kTypeOf(kString), 0, allocator)); 
        
        while (*it != 0)
        {
            const kChar* delimIt = delimiters; 

            while (*delimIt != 0)
            {
                if (*it == *delimIt++)
                {
                    kTest(kString_SplitAdd(start, (it-start), output, allocator)); 
                    start = it+1; 
                    break;
                }
            }
            ++it; 
        }    
        
        kTest(kString_SplitAdd(start, (it-start), output, allocator)); 

        *tokens = output; 
    }
    kCatch(&exception)
    {
        kObject_Dispose(output); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kString_Import(kString str, const kChar* cstr, kBool append)
{
    kStringClass* obj = kString_Cast_(str); 
    kSize cstrLength, combinedLength;

    if (kIsNull(cstr) && !append)
    {
        obj->length = 0; 

        if (obj->allocSize == 0)
        {
            obj->chars = &obj->nullStr; 
        }
        else
        {
            obj->chars[0] = 0; 
        }
    }
    else if (!kIsNull(cstr))
    {
        if (!append)
        {
            obj->length = 0;
        }
  
        cstrLength = kStrLength(cstr); 
        combinedLength = cstrLength + obj->length;

        kCheck(kString_Reserve(str, combinedLength)); 
    
        kMemCopy(obj->chars + obj->length, cstr, cstrLength*sizeof(kChar));

        obj->length = combinedLength;
        obj->chars[combinedLength] = 0; 
    }
    
    return kOK;
}

kFx(kChar*) kString_Chars(kString str)
{
    kStringClass* obj = kString_Cast_(str); 

    return kString_Chars_(obj); 
}

kFx(kSize) kString_Length(kString str)
{
    kStringClass* obj = kString_Cast_(str); 

    return kString_Length_(obj); 
}

kFx(kSize) kString_Capacity(kString str)
{
    kStringClass* obj = kString_Cast_(str); 

    return kString_Capacity_(obj); 
}
