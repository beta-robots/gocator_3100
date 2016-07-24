/*
 * kUtils.c
 * 
 * The Zen API 
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kUtils.h>
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/Data/kImage.h>
#include <kApi/Io/kDat5Serializer.h>
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Data/kMath.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

kBeginStaticClass(k, kUtils)    
kEndStaticClass()

kFx(kStatus) kUtils_InitStatic()
{
    //if a custom random number generator has not be registered, initialize the standard 
    //generator and set a default handler
    if (kIsNull(kApiLib_RandomHandler_()))
    {
        srand((unsigned int) time(kNULL)); 

        kCheck(kApiLib_SetRandomHandler(kDefaultRandom)); 
    }

    return kOK; 
}

kFx(kStatus) kUtils_ReleaseStatic()
{    
    return kOK; 
}

kFx(kStatus) kMemAlloc(kSize size, void* mem)
{
    kSetAs_(mem, kNULL, kPointer); 

    if (size > 0)
    {
        kAlloc alloc = kAlloc_App_(); 
        void* allocation = kNULL; 
        kSize allocationSize = kALIGN_ANY_SIZE + size; 

        kCheck(kAlloc_Get(alloc, allocationSize, &allocation)); 

        kSetAs_(allocation, alloc, kObject); 
        kSetAs_(mem, kAt_(allocation, kALIGN_ANY_SIZE), kPointer); 
    }

    return kOK; 
}

kFx(kStatus) kMemAllocZero(kSize size, void* mem)
{
    kCheck(kMemAlloc(size, mem)); 
    kCheck(kMemSet(kAs_(mem, kPointer), 0, size)); 

    return kOK; 
}

kFx(kStatus) kMemFree(void* mem)
{
    if (!kIsNull(mem))
    {
        void* allocation = kAt_(mem, -kALIGN_ANY_SIZE); 
        kAlloc alloc = kAs_(allocation, kObject); 

        kCheck(kAlloc_Free(alloc, allocation)); 
    }
    
    return kOK; 
}

kFx(kStatus) kMemFreeRef(void* mem)
{
    kCheck(kMemFree(kAs_(mem, kPointer))); 

    kSetAs_(mem, kNULL, kPointer); 

    return kOK; 
}

kFx(kStatus) kDestroyRef(kObject* object)
{
    kStatus result = kObject_Destroy(*object);     

    *object = kNULL; 

    return result; 
}

kFx(kStatus) kDisposeRef(kObject* object)
{
    kStatus result = kObject_Dispose(*object);     

    *object = kNULL; 

    return result; 
}

kFx(kStatus) kShareRef(kObject* object, kObject source)
{
    if (!kIsNull(source))
    {
        kCheck(kObject_Share(source)); 

        if (!kIsNull(object))
        {
            *object = source; 
        }
    }
    else
    {
        *object = kNULL; 
    }

    return kOK; 
}

kFx(kStatus) kLoad5(kObject* object, const kChar* fileName, kAlloc allocator)
{
    return kSerializer_LoadObject(object, kTypeOf(kDat5Serializer), fileName, allocator); 
}

kFx(kStatus) kSave5(kObject object, const kChar* fileName)
{
    return kSerializer_SaveObject(object, kTypeOf(kDat5Serializer), fileName); 
}

kFx(kStatus) kLoad6(kObject* object, const kChar* fileName, kAlloc allocator)
{
    return kSerializer_LoadObject(object, kTypeOf(kDat6Serializer), fileName, allocator); 
}

kFx(kStatus) kSave6(kObject object, const kChar* fileName)
{
    return kSerializer_SaveObject(object, kTypeOf(kDat6Serializer), fileName); 
}

kFx(kStatus) kMemSet(void* dest, kByte fill, kSize size)
{
    if (!kIsNull(kApiLib_MemSetHandler_()))
    {
        return kApiLib_MemSetHandler_()(dest, fill, size); 
    }
    else
    {
        memset(dest, fill, size); 
        return kOK; 
    }
}

kFx(kStatus) kMemCopy(void* dest, const void* src, kSize size)
{
    if (!kIsNull(kApiLib_MemCopyHandler_()))
    {
        return kApiLib_MemCopyHandler_()(dest, src, size); 
    }
    else
    {
        memcpy(dest, src, size); 
        return kOK; 
    }
}

kFx(kStatus) kMemMove(void* dest, const void* src, kSize size)
{
    if (!kIsNull(kApiLib_MemMoveHandler_()))
    {
        return kApiLib_MemMoveHandler_()(dest, src, size); 
    }
    else
    {
        memmove(dest, src, size); 
        return kOK; 
    }
}

kFx(kBool) kMemEquals(const void* a, const void* b, kSize size)
{
    return (memcmp(a, b, size) == 0); 
}

kFx(kStatus) kStrCopyEx(kChar* dest, kSize capacity, const kChar* src, kChar** endPos)
{
    kChar* destLast = dest + capacity - 1;  

    kCheckArgs(!kIsNull(dest) && !kIsNull(src) && (capacity > 0)); 

    while ((*src != 0) && (dest != destLast))
    {
        *dest++ = *src++; 
    }

    if (!kIsNull(endPos))
    {
        *endPos = dest;
    }

    *dest = 0; 

    return (*src == 0) ? kOK : kERROR_INCOMPLETE; 
}

kFx(kStatus) kStrCopy(kChar* dest, kSize capacity, const kChar* src)
{
    return kStrCopyEx(dest, capacity, src, kNULL); 
}

kFx(kStatus) kStrCat(kChar* dest, kSize capacity, const kChar* src)
{
    kSize length = kStrLength(dest); 

    return kStrCopy(&dest[length], capacity-length, src); 
}

kFx(kStatus) kStrToLower(kChar* str)
{
    while (*str != 0)
    {
        *str = kStrAsciiToLower(*str); 
        str++; 
    }

    return kOK; 
}

kFx(kBool) kStrEquals(const kChar* a, const kChar* b)
{
    return (strcmp(a, b) == 0); 
}

kFx(k32s) kStrCompare(const kChar* a, const kChar* b)
{
    return strcmp(a, b); 
}

kFx(k32s) kStrCompareLower(const kChar* a, const kChar* b)
{
    while (kStrAsciiToLower(*a) == kStrAsciiToLower(*b))
    {
        if (*a == 0)
        {
            return 0; 
        }

        a++; 
        b++; 
    }

    return kStrAsciiToLower(*a) - kStrAsciiToLower(*b);  
}

kFx(kSize) kStrLength(const kChar* str)
{
    return strlen(str); 
}

kFx(const kChar*) kStrFindFirst(const kChar* str, const kChar* subStr)
{
    return strstr(str, subStr); 
}

kFx(const kChar*) kStrFindLast(const kChar* str, const kChar* subStr)
{
    kSize strSize = strlen(str); 
    kSize subSize = strlen(subStr); 
    const kChar* it = str + (strSize - subSize); 
    kSize i; 

    while (it >= str)
    {
        for (i = 0; i < subSize; ++i)
        {
            if (it[i] != subStr[i])
            {
                break; 
            }
        }

        if (i == subSize)
        {
            return it; 
        }

        it--; 
    }

    return kNULL; 
}

kFx(kStatus) kStrPrintf(kChar* dest, kSize capacity, const kChar* format, ...)
{
    kVarArgList argList; 
    kStatus status; 
   
    kVarArgList_Start_(argList, format);
    {
        status = kStrPrintvf(dest, capacity, format, argList); 
    }
    kVarArgList_End_(argList); 

    return status; 
}

kFx(kChar) kStrAsciiToLower(kChar ch)
{
    switch (ch)
    {
    case 'A':       return 'a'; 
    case 'B':       return 'b'; 
    case 'C':       return 'c'; 
    case 'D':       return 'd'; 
    case 'E':       return 'e'; 
    case 'F':       return 'f'; 
    case 'G':       return 'g'; 
    case 'H':       return 'h'; 
    case 'I':       return 'i'; 
    case 'J':       return 'j'; 
    case 'K':       return 'k'; 
    case 'L':       return 'l'; 
    case 'M':       return 'm'; 
    case 'N':       return 'n'; 
    case 'O':       return 'o'; 
    case 'P':       return 'p'; 
    case 'Q':       return 'q'; 
    case 'R':       return 'r'; 
    case 'S':       return 's'; 
    case 'T':       return 't'; 
    case 'U':       return 'u'; 
    case 'V':       return 'v'; 
    case 'W':       return 'w'; 
    case 'X':       return 'x'; 
    case 'Y':       return 'y'; 
    case 'Z':       return 'z'; 
    default:        return ch; 
    }
}

kFx(kBool) kStrAsciiIsSpace(kChar ch)
{
    //ignoring white spaces like BS, TAB, LF, VT, FF, CR, NBSP
    switch (ch)
    {
    case ' ':       return kTRUE; 
    default:        return kFALSE; 
    }
}

kFx(kBool) kStrAsciiIsLetter(kChar ch)
{
    if (((ch >= 'a') && (ch <= 'z') ) || ((ch >= 'A') && (ch <= 'Z')))
    {
        return kTRUE;
    }

    return kFALSE;
}

kFx(kBool) kStrAsciiIsDigit(kChar ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return kTRUE;
    }

    return kFALSE;
}

#if defined(K_MSVC)

kFx(kStatus) kStrPrintvf(kChar* dest, kSize capacity, const kChar* format, kVarArgList argList)
{
    kSSize written; 
   
    kCheckArgs(capacity > 0); 
        
    written = _vsnprintf(dest, capacity, format, argList); 
    
    if (written < 0 || written >= (kSSize) capacity)
    {
        dest[capacity-1] = 0; 
    }

    return (written < 0 || written >= (kSSize) capacity) ? kERROR_INCOMPLETE : kOK; 
}

#else

kFx(kStatus) kStrPrintvf(kChar* dest, kSize capacity, const kChar* format, kVarArgList argList)
{
    kSSize written; 
   
    kCheckArgs(capacity > 0); 
        
    written = vsnprintf(dest, capacity, format, argList); 

    if (written >= (kSSize) capacity)
    {
        dest[capacity-1] = 0; 
    }

    return (written >= (kSSize)capacity) ? kERROR_INCOMPLETE : kOK; 
}

#endif

#if defined(K_MSVC)

kFx(k32s) kStrMeasuref(const kChar* format, kVarArgList argList)
{
    return _vscprintf(format, argList); 
}

#else

kFx(k32s) kStrMeasuref(const kChar* format, kVarArgList argList)
{
    return vsnprintf(kNULL, 0, format, argList);
}

#endif

kFx(kStatus) kLogf(const kChar* format, ...)
{
    if (kApiLib_LogfHandler_())
    {
        kVarArgList argList; 
        kStatus status; 

        kVarArgList_Start_(argList, format);
        {
            status = kApiLib_LogfHandler_()(format, argList); 
        }
        kVarArgList_End_(argList);

        return status; 
    }
    else
    {       
        return kOK; 
    }
}
kFx(kStatus) kLogvf(const kChar* format, kVarArgList argList)
{
    if (kApiLib_LogfHandler_())
    {
        return kApiLib_LogfHandler_()(format, argList); 
    }
    else
    {       
        return kOK; 
    }
}

kFx(kStatus) kReallocContent(kType type, void* itemsRef, kSize* allocSize, kSize newCount, kAlloc allocator)
{
    kSize itemSize = kType_Size_(type); 
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kSize oldSize = *allocSize; 
    kSize newSize = kMax_(oldSize, newCount*itemSize); 
    kByte* oldItems = *(void**)itemsRef; 
    kByte* newItems = oldItems; 
   
    if (newSize > oldSize)
    {
        kCheck(kAlloc_Get(alloc, newSize, &newItems));         
        kCheck(kAlloc_Free(alloc, oldItems)); 
    }
        
    if (kType_IsReference_(type))
    {
        kMemSet(newItems, 0, itemSize*newCount);  
    }

    *(void**)itemsRef = newItems; 
    *allocSize = newSize; 
    
    return kOK; 
}

kFx(kStatus) kOverrideFunctions(void* base, kSize baseSize, void* overrides)
{
    kSize count = baseSize / sizeof(kFunction); 
    kFunction* baseFx = base; 
    kFunction* overrideFx = overrides; 
    kSize i; 

    for (i = 0; i < count; ++i)
    {
        if (!kIsNull(overrideFx[i]))
        {
            baseFx[i] = overrideFx[i]; 
        }
    }

    return kOK; 
}

kFx(kStatus) kZeroContent(kType type, void* items, kSize count)
{
    return kMemSet(items, 0, count*kType_Size_(type)); 
}

kFx(kStatus) kCopyContent(kType type, void* dest, const void* src, kSize count)
{
    return kMemCopy(dest, src, count*kType_Size_(type)); 
}

kFx(kStatus) kCloneContent(kType type, void* dest, const void* src, kSize count, kAlloc allocator)
{
    if (kType_IsValue_(type))
    {
        kCheck(kMemCopy(dest, src, count*kType_Size_(type))); 
    }
    else
    {
        kAlloc alloc = kAlloc_Fallback_(allocator); 
        const kObject* in = (const kObject*) src; 
        kObject* out = dest;  
        kStatus status; 
        kSize i = 0; 

        kTry
        {
            for (i = 0; i < count; ++i)
            {
                kTest(kObject_Clone(&out[i], in[i], alloc));                  
            }
        }
        kCatch(&status)
        {
            kDisposeContent(type, dest, i); 
            kZeroContent(type, dest, count); 
            kEndCatch(status); 
        }
    }       

    return kOK; 
}

kFx(kStatus) kDisposeContent(kType type, void* items, kSize count)
{
    if (kType_IsReference_(type))
    {
        kObject* elements = items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                kCheck(kObject_Dispose(elements[i]));             
                elements[i] = kNULL; 
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kShareContent(kType type, void* items, kSize count)
{
    if (kType_IsReference_(type))
    {
        kObject* elements = items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                kCheck(kObject_Share_(elements[i]));             
            }
        }
    }

    return kOK; 
}

kFx(kSize) kSizeContent(kType type, const void* items, kSize count)
{
    kSize size = 0; 

    if (kType_IsReference_(type))
    {
        const kObject* elements = (const kObject*) items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                size += kObject_Size(elements[i]); 
            }
        }
    }

    return size; 
}

/* http://en.wikipedia.org/wiki/Jenkins_hash_function -- one-at-a-time */
kFx(kSize) kHashBytes(const void* key, kSize length)
{
    const kByte* input = key; 
    kSize hash = 0; 
    kSize i = 0; 
    
    for (i = 0; i < length; ++i)
    {
        hash += input[i];
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }

    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);
    
    return hash;
}

/* Kernighan, B., Pike, R. 1999. The Practice of Programming. p 56-57. */
kFx(kSize) kHashStr(const kChar* str)
{
    const kChar* it = str; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(kSize) kHashPointer(void* pointer)
{
    return kHashPointer_(pointer); 
}

kFx(k32u) kReverseBits32(k32u input, k32u bitCount)
{
    k32u output = 0; 
    k32u i; 

    for (i = 1; i <= bitCount; ++i)
    {
        output |= (input & 1) << (bitCount - i); 
        input = input >> 1; 
    }

    return output; 
}

kFx(kStatus) kBmpLoad(kObject *image, const char* fileName, kAlloc allocator)
{
    kFile file = kNULL;
    kSerializer serializer = kNULL;
    kImage out = kNULL;
    kRgb* palette = kNULL;
    k16u bmType, bpl, bppIn, pixelSizeIn, backPorch, outWidth, planes;
    k32u fileEnd, reserved, offBitCount, infoHeaderSize, width;
    k32u height, compression, outSize, paletteSize, i; 
    k32s x, y;
    kType type = kNULL;
    kStatus exception;

    kTry
    {
        kTest(kFile_Construct(&file, fileName, kFILE_MODE_READ, kNULL));
        kTest(kFile_SetReadBuffer(file, 16384)); 

        kTest(kSerializer_Construct(&serializer, file, kNULL, kNULL));

        kTest(kSerializer_Read16u(serializer, &bmType));
        kTest(kSerializer_Read32u(serializer, &fileEnd));
        kTest(kSerializer_Read32u(serializer, &reserved));
        kTest(kSerializer_Read32u(serializer, &offBitCount));

        kTest(kSerializer_Read32u(serializer, &infoHeaderSize));

        kTest(kFile_Length(file) == fileEnd); /* corrupted file? */

        if (infoHeaderSize == 40)
        {
            kTest(kSerializer_Read32u(serializer, &width));
            kTest(kSerializer_Read32u(serializer, &height));
            kTest(kSerializer_Read16u(serializer, &planes));
            kTest(kSerializer_Read16u(serializer, &bppIn));
            kTest(kSerializer_Read32u(serializer, &compression));
            kTest(kSerializer_Read32u(serializer, &outSize));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
        }
        else
        {
            /* TODO: add support for other header sizes */
            kThrow(kERROR);
        }

        if (outSize == 0) 
        {
            outSize = width * height * bppIn / 8;
        }

        kTest(planes == 1);
        kTest(compression == 0);

        kTest(width != 0);
        kTest(height != 0);

        paletteSize = offBitCount - infoHeaderSize - 14;

        if (paletteSize > 0)
        {
            kTest(kMemAlloc(paletteSize, &palette)); 
            kTest(kSerializer_ReadByteArray(serializer, (kByte*)palette, paletteSize));
        }

        kTest(outSize == (k32u)(kFile_Length(file) - kFile_Position(file))); /* corrupted file? */

        switch(bppIn)
        {
        case 8:
            type = kTypeOf(k8u);
            break;
        case 24:
            type = kTypeOf(kRgb);
            break;
        case 32:
            type = kTypeOf(kArgb);
            break;
        default:
            kThrow(kERROR);
        }

      //  kTest(outSize % height == 0);
        bpl = (k16u) (outSize / height); 

        kTest(bppIn % 8 == 0); /* does not support packed pixels */
        pixelSizeIn = bppIn / 8;

        backPorch = (k16u) (bpl - width * pixelSizeIn); 
        outWidth = (k16u) (width + backPorch); 

        kTest(kImage_Construct(&out, type, width, height, allocator));
        kTest(kImage_Zero(out));

        for (y = height - 1; y >= 0; y--)
        {
            for (x = 0; x < (k32s) width; x++)
            {
                if (paletteSize && (type == kTypeOf(k8u)))
                {
                    k32u pixel = 0;
                    kRgb* entry;

                    kTest(kSerializer_ReadByteArray(serializer, (kByte*)&pixel, pixelSizeIn));
                    
                    if (pixel > paletteSize/sizeof(kRgb))
                    {
                        kThrow(kERROR);
                    }
                    
                    entry = &palette[pixel];
                    *(kByte*)(kImage_At_(out, x, y)) = (entry->r+entry->g+entry->b) / 3;
                }
                else
                {
                    kTest(kSerializer_ReadByteArray(serializer, kImage_At_(out, x, y), pixelSizeIn));
                }
            }
            for (i = 0; i < outWidth - width; i++)
            {
                kTest(kSerializer_Read8u(serializer, (k8u*)&reserved));
            }
        }
        *image = out;
    }
    kCatchEx(&exception)
    {
        kDestroyRef(&out);
        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kMemFreeRef(&palette);
        kDestroyRef(&serializer);
        kDestroyRef(&file);
        kEndFinallyEx();
    }

    return kOK;
}


kFx(kStatus) kBmpSave(kObject image, const char* fileName)
{
    kFile file = kNULL;
    kSerializer serializer = kNULL;
    kType type = kImage_PixelType_(image);
    k32u width = (k32u)kImage_Width_(image);
    k32u height = (k32u)kImage_Height_(image);
    kByte* data = kImage_Data_(image);
    const k32u pixelSizeIn = (k32u) kImage_PixelSize_(image);
    const k32u paletteEntryCount = (type == kTypeOf(k8u)) ? 256 : 0;
    const k32u offBitCount = 54 + 4 * paletteEntryCount;
    static const k32u infoHeaderSize = 40;
    static const k32u bmType = 19778;
    k32u i, pixelSizeOut, outWidth, backPorch, outSize; 
    k32s x, y;

    if      (type == kTypeOf(k8u))      pixelSizeOut = 1;
    else if (type == kTypeOf(kRgb))     pixelSizeOut = 3;
    else if (type == kTypeOf(kArgb))    pixelSizeOut = 4;
    else                                return kERROR_PARAMETER;
 
    kTry
    {
        kTest(kFile_Construct(&file, fileName, kFILE_MODE_WRITE, kNULL));
        kTest(kSerializer_Construct(&serializer, file, kNULL, kNULL));

        backPorch = (4 - ((width * pixelSizeOut) % 4)) % 4;
        outWidth = width + backPorch;
        outSize = (width * pixelSizeOut + backPorch) * height;

        kTest(kSerializer_Write16u(serializer, (k16u)bmType));
        kTest(kSerializer_Write32u(serializer, offBitCount + outSize));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, offBitCount));

        kTest(kSerializer_Write32u(serializer, infoHeaderSize));
        kTest(kSerializer_Write32u(serializer, width));
        kTest(kSerializer_Write32u(serializer, height));
        kTest(kSerializer_Write16u(serializer, 1));
        kTest(kSerializer_Write16u(serializer, (k16u)pixelSizeOut * 8));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, outSize));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, 0));

        /* This is a grayscale palette */
        for (i = 0; i < paletteEntryCount; i++)
        {
            kTest(kSerializer_Write32u(serializer, i * 0x00010101));
        }

        for (y = height - 1; y >= 0; y--)
        {
            for (x = 0; x < (k32s) width; x++)
            {
                kTest(kSerializer_WriteByteArray(serializer, &data[(y * width + x) * pixelSizeIn], pixelSizeOut));
            }
            for (i = 0; i < outWidth - width; i++)
            {
                kTest(kSerializer_Write8u(serializer, 0));
            }
        }

        kTest(kSerializer_Flush(serializer)); 
    }
    kFinally
    {
        kObject_Destroy(serializer);
        kObject_Destroy(file);
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) kObjectFromBytes6(kObject* object, const kByte* data, kSize size, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kMemory memory = kNULL; 
    kSerializer serializer = kNULL; 

    kTry
    {
        kTest(kMemory_Construct(&memory, kNULL)); 
        kTest(kMemory_Attach(memory, (void*)data, 0, size, size)); 

        kTest(kSerializer_Construct(&serializer, memory, kTypeOf(kDat6Serializer), kNULL)); 

        kTest(kSerializer_ReadObject(serializer, object, alloc)); 
    }
    kFinally
    {
        kCheck(kObject_Destroy(serializer)); 
        kCheck(kObject_Destroy(memory)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kObjectToBytes6(kObject object, const kByte** data, kSize* size, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kMemory memory = kNULL; 
    kSerializer serializer = kNULL; 
    kByte* output = kNULL; 

    kTry
    {
        kTest(kMemory_Construct(&memory, kNULL)); 

        kTest(kSerializer_Construct(&serializer, memory, kTypeOf(kDat6Serializer), kNULL)); 

        kTest(kSerializer_WriteObject(serializer, object)); 

        kTest(kAlloc_Get(alloc, (kSize)kMemory_Length(memory), &output)); 

        kTest(kMemCopy(output, kMemory_At(memory, 0), (kSize)kMemory_Length(memory))); 

        *size = (kSize) kMemory_Length(memory); 
        *data = output; 
        output = kNULL; 
    }
    kFinally
    {
        kCheck(kObject_Destroy(serializer)); 
        kCheck(kObject_Destroy(memory)); 
        kCheck(kAlloc_Free(alloc, output)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDefaultMemAlloc(kPointer receiver, kSize size, void* mem)
{
    *(void**)mem = kNULL; 

    if (size > 0)
    {
        void* allocation = malloc(size); 

        kAssert(((kSize)allocation & (kALIGN_ANY_SIZE-1)) == 0); 

        *(void**)mem = allocation; 

        return kIsNull(allocation) ? kERROR_MEMORY : kOK; 
    }

    return kOK; 
}

kFx(kStatus) kDefaultMemFree(kPointer receiver, void* mem)
{
    free(mem); 

    return kOK; 
}

kFx(kStatus) kSysMemAlloc(kSize size, void* mem)
{
    kCheck(kApiLib_MemAllocHandler_()(kApiLib_MemAllocProvider_(), size, mem)); 

    kCheck(kMemSet(*(void**)mem, 0, size)); 

    return kOK; 
}

kFx(kStatus) kSysMemFree(void* mem)
{
    return kApiLib_MemFreeHandler_()(kApiLib_MemAllocProvider_(), mem); 
}

kFx(kStatus) kSysMemReallocList(void* list, kSize count, kSize itemSize, kSize* capacity, kSize initialCapacity, kSize requiredCapacity)
{
    const kSize GROWTH_FACTOR = 2; 
    kSize oldCapacity = *capacity; 

    if (oldCapacity < requiredCapacity)
    {
        kSize newCapacity = 0; 
        void* newList = kNULL;  

        newCapacity = kMax_(requiredCapacity, GROWTH_FACTOR*oldCapacity);
        newCapacity = kMax_(newCapacity, initialCapacity); 

        kCheck(kSysMemAlloc(newCapacity*itemSize, &newList)); 

        kMemCopy(newList, *(void**)list, itemSize*count); 
        
        kSysMemFree(*(void**)list); 

        *(void**)list = newList; 
        *capacity = newCapacity; 
    }

    return kOK; 
}

kFx(k32u) kDefaultRandom()
{
    //standard guarantees that rand will generate at least 15 bits
    return ((k32u)rand() << 17) ^ ((k32u)rand() << 8) ^ ((k32u)rand()); 
}

kFx(k32u) kRandom32u()
{
    return kApiLib_RandomHandler_()(); 
}

kFx(k64u) kRandom64u()
{
    k64u lower = kRandom32u(); 
    k64u upper = kRandom32u(); 

    return (upper << 32) | lower; 
}

kFx(kSize) kRandomSize()
{
    return (K_POINTER_SIZE == 8) ? (kSize)kRandom64u() : (kSize)kRandom32u(); 
}

kFx(k64u) kTimeToKernelTime(k64u time)
{
    k64u ticks;
    k64u truncatedTime;

    if (time == kINFINITE)
    {
        return K_OS_INFINITE;        
    }
    
    //divider == 1 is common; treat as special case (64-bit divides are expensive on some platforms that we support)
    if (kApiLib_KernelTimerDivider_() == 1)
    {
        ticks = time / kApiLib_KernelTimerMultiplier_();
        truncatedTime = kApiLib_KernelTimerMultiplier_() * ticks; 
    }
    else
    {
        ticks = kApiLib_KernelTimerDivider_()*(time) / kApiLib_KernelTimerMultiplier_(); 
        truncatedTime = kApiLib_KernelTimerMultiplier_()*(ticks) / kApiLib_KernelTimerDivider_();        
    }

    return (truncatedTime == time) ? ticks : ticks + 1; 
}

#if defined (K_DARWIN)

kFx(kStatus) kFormatTimeout(k64u timeout, struct timespec* tm)
{
    struct timeval tv;
    k64u absTime = 0; 
    
    if (gettimeofday(&tv, kNULL) != 0)
    {
        return kERROR; 
    }
    
    absTime = k64U(1000000)*tv.tv_sec + tv.tv_usec;
    absTime += timeout; 
    
    tm->tv_sec = (time_t) (absTime / 1000000); 
    tm->tv_nsec =  (long)(absTime % 1000000) * 1000; 
    
    return kOK; 
}

#elif defined (K_POSIX)

kFx(kStatus) kFormatTimeout(k64u timeout, struct timespec* tm)
{
    k64u absTime; 
    
    if (clock_gettime(CLOCK_REALTIME, tm) != 0)
    {
        return kERROR; 
    }

    absTime = k64U(1000000)*tm->tv_sec + tm->tv_nsec/k64U(1000);
    absTime += timeout; 

    tm->tv_sec = (time_t) (absTime / 1000000); 
    tm->tv_nsec =  (long)(absTime % 1000000) * 1000; 

    return kOK;  
}

#endif

kFx(void) kUpdateProgress(kCallbackFx progress, kPointer receiver, kPointer sender, k64u* updateTime, k32u progressValue)
{
    if (!kIsNull(progress))
    {
        const k64u updateIntervalUs = 1000000;
        k64u now = kTimer_Now(); 
        kBool shouldUpdate = kIsNull(updateTime) || (*updateTime == k64U_NULL) || ((now - *updateTime) >= updateIntervalUs); 

        if (shouldUpdate)
        {
            progress(receiver, sender, &progressValue); 

            if (!kIsNull(updateTime))
            {
                *updateTime = now; 
            }
        }       
    }
}


//K_COMPAT_5
kFx(void*) kMalloc(kSize size)
{
    kPointer mem = kNULL; 

    return kSuccess(kMemAlloc(size, &mem)) ? mem : kNULL;  
}

kFx(kStatus) kStrFormat8u(k8u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParse8u(k8u* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (k8u) temp;
    return kOK;
}

kFx(kStatus) kStrFormat8s(k8s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParse8s(k8s* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (k8s) temp;
    return kOK;
}

kFx(kStatus) kStrFormat16u(k16u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParse16u(k16u* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (k16u) temp;
    return kOK;
}

kFx(kStatus) kStrFormat16s(k16s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParse16s(k16s* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (k16s) temp;
    return kOK;
}

kFx(kStatus) kStrFormat32u(k32u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParse32u(k32u* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (k32u) temp;
    return kOK;
}

kFx(kStatus) kStrFormat32s(k32s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParse32s(k32s* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (k32s) temp;
    return kOK;
}

kFx(kStatus) kStrFormat64u(k64u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    kChar tmpBuf[21];
    kChar* writePos = kNULL;

    *(writePos = &tmpBuf[sizeof(tmpBuf) - 1]) = 0;

    do
    {
        *--writePos = value % 10 + '0';
    }
    while (value /= 10);

    return kStrCopyEx(buffer, capacity, writePos, endPos);
}

kFx(kStatus) kStrParse64u(k64u* value, const kChar* str)
{
    kSize len = kStrLength(str);
    k64u v = 0;

    while (len && kStrAsciiIsSpace(*str))
    {
        str++;
        len--;
    }

    while (len && *str >= '0' && *str <= '9')
    {
        v = v * 10 + *str - '0';
        len--;
        str++;
    }

    *value = v;
    return kOK;
}

kFx(kStatus) kStrFormat64s(k64s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    kChar tmpBuf[21];
    kChar* writePos = kNULL;
    kBool isNegative = value < 0;

    *(writePos = &tmpBuf[sizeof(tmpBuf) - 1]) = 0;

    do
    {
        *--writePos = kMath_Abs_(value % 10) + '0';
    }
    while (value /= 10);

    if (isNegative)
    {
        *--writePos = '-';
    }

    return kStrCopyEx(buffer, capacity, writePos, endPos);
}

kFx(kStatus) kStrParse64s(k64s* value, const kChar* str)
{
    kSize len = kStrLength(str);
    kBool isNegative = kFALSE;
    k64u v = 0;
    kChar tmp;

    while (len && kStrAsciiIsSpace(*str))
    {
        str++;
        len--;
    }

    if (len)
    {
        if (*str == '-' || *str == '+')
        {
            isNegative = (*str == '-');
            str++;
            len--;
        }
    }

    while (len && (tmp = *str - '0') >= 0 && tmp < 10)
    {
        v = v * 10 + tmp;
        len--;
        str++;
    }

    if (isNegative)
    {
        *value = - (k64s) v;
    }
    else
    {
        *value = v;
    }

    return kOK;
}

kFx(kStatus) kStrFormatBool(kBool value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat32s(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParseBool(kBool* value, const kChar* str)
{
    k32s temp;

    kCheck(k32s_Parse(&temp, str));

    *value = (kBool) temp;
    return kOK;
}

kFx(kStatus) kStrFormatSize(kSize value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParseSize(kSize* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (kSize) temp;
    return kOK;
}

kFx(kStatus) kStrFormatSSize(kSSize value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return kStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) kStrParseSSize(kSSize* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (kSSize) temp;
    return kOK;
}

kFx(kStatus) kStrFormat32f(k32f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos)
{
    return kStrFormat64f(value, buffer, capacity, digitCount, endPos);
}

kFx(kStatus) kStrParse32f(k32f* value, const kChar* str)
{
    k64f temp;

    kCheck(k64f_Parse(&temp, str));

    *value = (k32f) temp;
    return kOK;
}

kFx(kStatus) kStrFormat64f(k64f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos)
{
    kChar valueBuffer[100];
    kChar formatBuffer[510];
    kChar* pos = formatBuffer + sizeof(formatBuffer) - 1;
    k32s exp = 0;
    k32s i;
    kBool isNegative;
    kBool digitFound = kFALSE;
    k32s decPt;
    kChar* result = kNULL;

    kCheckErr(capacity > 1, PARAMETER);

    if (digitCount == 0) digitCount = 1;
    if (digitCount < 0)  digitCount = 6;

    *(pos--) = '\0'; 

    if (kIsNanOrInf(value))
    {
        return kStrCopyEx(buffer, capacity, "NAN", endPos);
    }

    kCheck(kStrEcvt(value, valueBuffer, kCountOf(valueBuffer), digitCount, &decPt, &isNegative, &result));

    if (decPt < -3 || decPt > digitCount)
    {
        k64u expValue;

        for (; decPt > 1; decPt--, exp++);
        for (; decPt < 1; decPt++, exp--);

        expValue = (exp > 0) ? exp : -exp;

        do
        {
            *(pos--) = expValue % 10 + '0';
        }
        while (expValue /= 10);

        if (exp < 10 && exp > -10)
        {
            *(pos--) = '0';
        }

        *(pos--) = (exp < 0) ? '-' : '+';
        *(pos--) = 'e';
    }

    for (i = (k32s) kStrLength(result) - 1; i >= decPt; --i)
    {
        kChar digit = (i >= 0) ? result[i] : '0'; 

        if (digit != '0' || digitFound)
        {
            *(pos--) = digit;

            digitFound = kTRUE;
        }
    }

    if (digitFound)
    {
        *(pos--) = '.';
    }

    if (decPt > 0)
    {
        kChar* tmpPtr = result + decPt - 1;

        for (; tmpPtr >= result; tmpPtr--)
        {
            *(pos--) = *tmpPtr;
        }
    }
    else
    {
        *(pos--) = '0';
    }

    if (isNegative)
    {
        *(pos--) = '-'; 
    }

    return kStrCopyEx(buffer, capacity, pos+1, endPos);
}

kFx(kStatus) kStrParse64f(k64f* value, const kChar* str)
{
    kStatus status = kOK;
    k64f result = 0;
    kChar tmp;
    const kChar* readPos = str;
    k32s exp = 0;
    k32s count = 0;
    kBool isValid = kFALSE;
    kBool isNegative;

    while (kStrAsciiIsSpace(*readPos))
    {
        ++readPos;
    }

    if ((isNegative = ((tmp = *readPos) == '-')) || (tmp == '+'))
    {
        ++readPos;
        isValid = kTRUE;
    }

    for (; (tmp = *readPos) >= '0' && tmp <= '9'; ++readPos)
    {
        result = result * 10 + tmp - '0';
        isValid = kTRUE;
    }

    if (tmp == '.')
    {
        while ((tmp = *++readPos) >= '0' && tmp <= '9')
        {
            result = result * 10 + tmp - '0'; 
            isValid = kTRUE;
            --exp;
        }
    }

    if (isNegative)
    {
        result = -result;
    }

    if (isValid && kStrAsciiToLower(*readPos) == 'e')
    {
        if ((isNegative = ((tmp = *++readPos) == '-')) || (tmp == '+'))
        {
            tmp = *++readPos;
        }
 
        for (count = 0; tmp >= '0' && tmp <= '9'; tmp = *++readPos)
        {
            if ((k32S_MAX - abs(exp) - (tmp - '0')) / 10 > count)
            {
                count *= 10; 
                count += tmp - '0';
            }
            else
            {
                count = k32S_MAX - exp;
                break;
            }
        }

        if (isNegative)  exp -= count;
        else             exp += count;
    }

    if (result != 0.0)
    {
        if (exp > DBL_MAX_10_EXP)
        {
            status = kERROR_FORMAT;

            result = (result < 0) ? -HUGE_VAL : HUGE_VAL;
        }
        else if (exp < DBL_MIN_10_EXP)
        {
            status = kERROR_FORMAT;

            result = 0.0;
        }
        else if (exp < 0)
        {
            for (count = 0, exp = -exp; exp; count++, exp >>= 1)
            {
                if (exp & 1)
                {
                    result /= pow(10, 1 << count);
                }
            }
       }
       else
       {
            for (count = 0; exp; count++, exp >>= 1)
            {
                if (exp & 1)
                {
                    result *= pow(10, 1 << count);
                }
            }
       }
    }

    *value = result;
    return status;
}

kFx(kBool) kIsNanOrInf(k64f value)
{
    k64u temp = *(k64u*) &value;

    return (temp & k64U(0x7FF0000000000000)) == k64U(0x7FF0000000000000);
}

kFx(kStatus) kStrEcvt(k64f value, kChar* buffer, kSize capacity, k32s digitCount, k32s* decPt, kBool* isNegative, kChar** endPos)
{
    kChar* pos = kNULL;
    k32s digits = 0;
    k32s temp;

    buffer[0] = '0';
    digitCount++;

    if ((*isNegative = (value < 0)))
    {
        value = -value;
    }

    while (value > k32S_MAX) { value /= 10; digits++; }
    while (value && value < 1) { value *= 10; digits--; }

    kCheck(kStrFormat32s((k32s) value, buffer+1, capacity-1, &pos));

    temp = (k32s) (pos - buffer) - 1;
    *decPt  = digits + temp;

    if (temp >= (kSSize) digitCount)
    {
        pos = buffer + digitCount + 1;
    }
    else if ((digitCount -= temp) > 0) do
    {
        value -= (k64s) value;
        *pos++ = (k32s) (value *= 10.0) + '0';
    } 
    while (--digitCount);

    if (*--pos >= '5')
    {
        kChar* ptr = pos;
        while ((*--ptr += 1) > '9') *ptr = '0';

        if (ptr == buffer)
        {
            *--pos = 0;
            *decPt += 1;
            *endPos = buffer;
            return kOK;
        }
    }

    *pos = 0;
    *endPos = buffer + 1;
    return kOK;
}
