/** 
 * @file    kSerializer.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Io/kFile.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>

kBeginValue(k, kSerializerWriteSection, kValue)
    kAddField(kSerializerWriteSection, kPointer, buffer) 
    kAddField(kSerializerWriteSection, kSize, start)
kEndValue()

kBeginClass(k, kSerializer, kObject)
    kAddVMethod(kSerializer, kObject, VRelease)
    kAddVMethod(kSerializer, kSerializer, VInit)
    kAddVMethod(kSerializer, kSerializer, VCanAutoFlush)
    kAddVMethod(kSerializer, kSerializer, VReset)
    kAddVMethod(kSerializer, kSerializer, VWriteObject)
    kAddVMethod(kSerializer, kSerializer, VReadObject) 
    kAddVMethod(kSerializer, kSerializer, VWriteType)
    kAddVMethod(kSerializer, kSerializer, VReadType)
kEndClass()

kFx(kStatus) kSerializer_Construct(kSerializer* serializer, kStream stream, kType serializerType, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kIsNull(serializerType) ? kTypeOf(kSerializer) : serializerType; 
    kStatus status;    

    kCheckArgs((type == kTypeOf(kSerializer))|| kType_Overrides_(type, kSerializer, VInit)); 

    kCheck(kAlloc_GetObject_(alloc, type, serializer)); 

    if (!kSuccess(status = kCast(kSerializerVTable*, kType_VTable_(type))->VInit(*serializer, type, stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

kFx(kStatus) kSerializer_LoadObject(kObject* object, kType serializerType, const kChar* filePath, kAlloc readAlloc)
{
    kFile stream = kNULL; 
    kSerializer reader = kNULL; 

    kCheckArgs(!kIsNull(serializerType)); 

    kTry
    {        
        kTest(kFile_Construct(&stream, filePath, kFILE_MODE_READ, kNULL)); 
        kTest(kFile_SetReadBuffer(stream, kSERIALIZER_DEFAULT_BUFFER_SIZE)); 

        kTest(kSerializer_Construct(&reader, stream, serializerType, kNULL)); 
        kTest(kSerializer_ReadObject_(reader, object, readAlloc));
    }
    kFinally
    {
        kCheck(kObject_Destroy(reader)); 
        kCheck(kObject_Destroy(stream)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_SaveObject(kObject object, kType serializerType, const kChar* filePath)
{
    kSerializer writer = kNULL; 
    kFile stream = kNULL; 

    kCheckArgs(!kIsNull(serializerType)); 

    kTry
    {        
        kTest(kFile_Construct(&stream, filePath, kFILE_MODE_WRITE, kNULL)); 

        kTest(kSerializer_Construct(&writer, stream, serializerType, kNULL)); 

        kTest(kSerializer_WriteObject_(writer, object)); 
        kTest(kSerializer_Flush(writer)); 
    }
    kFinally
    {
        kCheck(kObject_Destroy(writer)); 
        kCheck(kObject_Destroy(stream)); 

        kEndFinally(); 
    }

    return kOK; 
}


kFx(kStatus) kSerializer_VInit(kSerializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kSerializerClass* obj = serializer;
    kStatus status; 
    
    kCheck(kObject_Init_(serializer, type, allocator)); 

    kInitFields_(kSerializer, serializer); 

    obj->stream = stream; 
    obj->swap = !K_IS_LITTLE_ENDIAN; 
    obj->sizeEncoding = 4; 
    obj->bufferSize = kSERIALIZER_DEFAULT_BUFFER_SIZE; 
    obj->formatVersion = k32U_NULL; 
    obj->tempStatus = kOK; 

    kTry
    {
        kTest(kMap_Construct(&obj->assemblyVersions, kTypeOf(kAssembly), kTypeOf(kVersion), 16, allocator)); 

        kTest(kArrayList_Construct(&obj->writeSections, kTypeOf(kSerializerWriteSection), 16, allocator)); 
        kTest(kArrayList_Construct(&obj->readSections, kTypeOf(k64u), 16, allocator)); 

        kTest(kSerializer_AddBuffer(serializer)); 
    }
    kCatch(&status)
    {
        kSerializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_VRelease(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSerializerBuffer* buffer = kNULL; 

    kCheck(kSerializer_ClearBuffers(serializer)); 
 
    buffer = obj->freeBuffers; 

    while (!kIsNull(buffer))
    {
        kSerializerBuffer* next = buffer->next; 
        
        kCheck(kObject_FreeMem(serializer, buffer)); 

        buffer = next; 
    }

    kCheck(kObject_Destroy(obj->assemblyVersions)); 
    kCheck(kObject_Destroy(obj->readSections)); 
    kCheck(kObject_Destroy(obj->writeSections)); 
        
    kCheck(kObject_VRelease_(serializer)); 

    return kOK; 
}

kFx(kBool) kSerializer_SetBigEndian(kSerializer serializer, kBool isBigEndian)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kBool platformIsBigEndian = !K_IS_LITTLE_ENDIAN; 

    obj->swap = platformIsBigEndian ^ isBigEndian; 

    return kOK; 
}

kFx(kBool) kSerializer_IsBigEndian(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kBool platformIsBigEndian = !K_IS_LITTLE_ENDIAN; 

    return platformIsBigEndian ^ obj->swap; 
}

kFx(kBool) kSerializer_CanAutoFlush(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_CanAutoFlush_(obj); 
}

kFx(kBool) kSerializer_VCanAutoFlush(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return (kArrayList_Count_(obj->writeSections) == 0); 
}

kFx(kStatus) kSerializer_Flush(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
  
    kCheck(kSerializer_FlushBuffers(serializer)); 
    kCheck(kSerializer_AddBuffer(serializer)); 

    kCheck(kStream_Flush_(obj->stream)); 

    return kOK; 
}

kFx(kStatus) kSerializer_Reset(kSerializer serializer)
{
    return kSerializer_Reset_(serializer); 
}

kFx(kStatus) kSerializer_VReset(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    kCheck(kSerializer_ClearBuffers(serializer)); 
    kCheck(kSerializer_AddBuffer(serializer)); 

    kCheck(kArrayList_Clear_(obj->readSections)); 
    kCheck(kArrayList_Clear_(obj->writeSections)); 

    return kOK; 
}

kFx(kStream) kSerializer_Stream(kSerializer serializer)
{  
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Stream_(obj); 
}

kFx(kStatus) kSerializer_WriteObject(kSerializer serializer, kObject object)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteObject_(obj, object); 
}

kFx(kStatus) kSerializer_VWriteObject(kSerializer serializer, kObject object)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kSerializer_ReadObject(kSerializer serializer, kObject* object, kAlloc allocator)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadObject_(obj, object, allocator); 
}

kFx(kStatus) kSerializer_VReadObject(kSerializer serializer, kObject* object, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kSerializer_WriteType(kSerializer serializer, kType type, kTypeVersion* version)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteType_(obj, type, version); 
}

kFx(kStatus) kSerializer_VWriteType(kSerializer serializer, kType type, kTypeVersion* version)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kSerializer_ReadType(kSerializer serializer, kType* type, kTypeVersion* version)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadType_(obj, type, version); 
}

kFx(kStatus) kSerializer_VReadType(kSerializer serializer, kType* type, kTypeVersion* version)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kSerializer_WriteItems(kSerializer serializer, kType type, kTypeVersion version, const void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    if (kType_IsReference_(type))
    {
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_WriteObject_(obj, ((kObject*)items)[i])); 
        }
    }
    else
    {
        kCheck(kType_SerializeFx_(type, version, kValue)(type, items, count, serializer)); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_ReadItems(kSerializer serializer, kType type, kTypeVersion version, void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
   
    if (kType_IsReference_(type))
    {
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_ReadObject_(serializer, &((kObject*)items)[i], obj->readAlloc)); 
        }
    }
    else
    {
        kCheck(kType_DeserializeFx_(type, version, kValue)(type, items, count, serializer)); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_BeginWrite(kSerializer serializer, kType sizeType, kBool includeSize)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSerializerWriteSection* section = kNULL; 

    if      (sizeType == kTypeOf(k8u))       kCheck(kSerializer_Write8u_(serializer, 0)); 
    else if (sizeType == kTypeOf(k16u))      kCheck(kSerializer_Write16u_(serializer, 0)); 
    else if (sizeType == kTypeOf(k32u))      kCheck(kSerializer_Write32u_(serializer, 0)); 
    else if (sizeType == kTypeOf(k64u))      kCheck(kSerializer_Write64u_(serializer, 0)); 
    else                                     return kERROR_PARAMETER; 

    kCheck(kArrayList_AddCount_(obj->writeSections, 1)); 
    section = kArrayList_Last_(obj->writeSections); 

    section->buffer = obj->currentBuffer; 
    section->start = obj->currentBuffer->written - kType_Size_(sizeType); 
    section->type = sizeType; 
    section->includeSize = includeSize; 

    return kOK; 
}

kFx(kStatus) kSerializer_EndWrite(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSerializerWriteSection* section = kArrayList_Last_(obj->writeSections); 
    kSerializerBuffer* buffer = kNULL; 
    kSize size = 0; 

    kAssert(kArrayList_Count(obj->writeSections) > 0); 

    buffer = section->buffer; 
    size = (section->buffer->written - section->start); 
    
    buffer = buffer->next; 

    while (buffer != obj->currentBuffer->next)
    {
        size += buffer->written;
        buffer = buffer->next; 
    }

    if (!section->includeSize)
    {
        size -= kType_Size_(section->type); 
    }

    if (section->type == kTypeOf(k8u))       
    {
        k8u sizeField = (k8u)(size); 

        kCheckState(size <= k8U_MAX); 

        *(k8u*) ((kByte*)section->buffer->data + section->start) = sizeField; 
    }
    else if (section->type == kTypeOf(k16u))      
    {
        k16u sizeField = (k16u) size; 

        kCheckState(size <= k16U_MAX); 

        kSerializer_Reorder2_(serializer, (kByte*)section->buffer->data + section->start, &sizeField); 
    }
    else if (section->type == kTypeOf(k32u))      
    {
        k32u sizeField = (k32u) size; 

        kCheckState(size <= k32U_MAX); 

        kSerializer_Reorder4_(serializer, (kByte*)section->buffer->data + section->start, &sizeField); 
    }
    else if (section->type == kTypeOf(k64u))      
    {
        k64u sizeField = (k64u) size; 

        kCheckState(size <= k64U_MAX); 

        kSerializer_Reorder8_(serializer, (kByte*)section->buffer->data + section->start, &sizeField); 
    }
    else
    {
        return kERROR_FORMAT; 
    }

    kCheck(kArrayList_RemoveCount_(obj->writeSections, 1)); 

    return kOK; 
}

kFx(kStatus) kSerializer_BeginRead(kSerializer serializer, kType sizeType, kBool includeSize)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    k64u readSize = 0; 
    k64u readEnd = 0; 

    if (sizeType == kTypeOf(k8u))       
    {
        k8u sizeField; 

        kCheck(kSerializer_Read8u_(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else if (sizeType == kTypeOf(k16u))      
    {
        k16u sizeField; 

        kCheck(kSerializer_Read16u_(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else if (sizeType == kTypeOf(k32u))      
    {
        k32u sizeField; 

        kCheck(kSerializer_Read32u_(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else if (sizeType == kTypeOf(k64u))      
    {
        k64u sizeField; 

        kCheck(kSerializer_Read64u_(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else
    {
        return kERROR_PARAMETER; 
    }

    if (includeSize)
    {
        readSize -= kType_Size_(sizeType); 
    }
    
    readEnd = kStream_BytesRead_(obj->stream) + readSize; 
    
    kCheck(kArrayList_AddCount_(obj->readSections, 1));     
    kSetAs_(kArrayList_Last_(obj->readSections), readEnd, k64u); 

    return kOK; 
}

kFx(kStatus) kSerializer_EndRead(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    k64u readLocation = kStream_BytesRead_(obj->stream); 
    k64u readSection; 
    kSize advance = 0; 
   
    kAssert(kArrayList_Count(obj->readSections) > 0); 

    readSection = kAs_(kArrayList_Last_(obj->readSections), k64u); 

    kCheck(kArrayList_RemoveCount_(obj->readSections, 1));   

    kCheckState(readSection >= readLocation); 
        
    advance = (kSize) (readSection - readLocation); 

    if (advance > 0)
    {
        kCheck(kSerializer_AdvanceRead_(serializer, advance)); 
    }

    return kOK; 
}

kFx(kBool) kSerializer_ReadCompleted(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    kAssert(kArrayList_Count(obj->readSections) > 0); 

    return kSerializer_ReadCompleted_(obj); 
}

kFx(kStatus) kSerializer_AdvanceRead(kSerializer serializer, kSize offset)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kByte bitBucket[128]; 
    kSize remaining = offset; 
   
    while (remaining > 0)
    {
        kSize readAmount = kMin_(remaining, sizeof(bitBucket)); 

        kCheck(kStream_Read_(obj->stream, bitBucket, readAmount)); 
        
        remaining -= readAmount; 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_SetVersion(kSerializer serializer, kAssembly assembly, kVersion version)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    
    if (kIsNull(assembly))
    {
        obj->formatVersion = version; 
    }
    else
    {
        kCheck(kMap_Replace(obj->assemblyVersions, &assembly, &version)); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_SetSizeEncoding(kSerializer serializer, k32u byteCount)
{
    kSerializerClass* obj = serializer;

    kCheckArgs((byteCount == 4) || (byteCount == 8)); 

    obj->sizeEncoding = byteCount; 

    return kOK; 
}

kFx(kStatus) kSerializer_AddBuffer(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSerializerBuffer* next = kNULL; 
    
    if (kIsNull(obj->freeBuffers))
    {
        kCheck(kSerializer_AllocateBuffer(serializer));        
    }

    if (kIsNull(obj->activeBuffers))
    {
        obj->activeBuffers = obj->freeBuffers; 
    }
    else
    {
        next = obj->currentBuffer->next; 
        obj->currentBuffer->next = obj->freeBuffers; 
    }
    
    obj->currentBuffer = obj->freeBuffers; 
    obj->freeBuffers = obj->freeBuffers->next; 
    obj->currentBuffer->next = next;

    obj->currentBuffer->written = 0; 

    return kOK; 
}

kFx(kStatus) kSerializer_AllocateBuffer(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize headerSize = sizeof(kSerializerBuffer); 
    kSize dataOffset = kAlign_(headerSize, kALIGN_ANY); 
    kSize size = dataOffset + obj->bufferSize; 
    kSerializerBuffer* buffer = kNULL; 
    
    kCheck(kObject_GetMem_(serializer, size, &buffer)); 

    buffer->capacity = obj->bufferSize; 
    buffer->written = 0; 
    buffer->data = ((kByte*)buffer) + dataOffset; 

    buffer->next = obj->freeBuffers; 
    obj->freeBuffers = buffer; 

    return kOK; 
}

kFx(kStatus) kSerializer_InsertHeader(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSerializerBuffer* next = obj->activeBuffers; 
    
    if (kIsNull(obj->freeBuffers))
    {
        kCheck(kSerializer_AllocateBuffer(serializer));        
    }

    obj->activeBuffers = obj->freeBuffers; 
    obj->freeBuffers = obj->freeBuffers->next; 
    obj->activeBuffers->next = next; 
 
    obj->currentBuffer = obj->activeBuffers; 
    obj->currentBuffer->written = 0; 

    return kOK; 
}

kFx(kStatus) kSerializer_ClearBuffers(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    while (!kIsNull(obj->activeBuffers))
    {
        kSerializerBuffer* next = obj->activeBuffers->next; 

        obj->activeBuffers->next = obj->freeBuffers; 
        obj->freeBuffers = obj->activeBuffers; 
        obj->activeBuffers = next; 
    }

    obj->currentBuffer = kNULL; 

    return kOK; 
}

kFx(kStatus) kSerializer_FlushBuffers(kSerializer serializer)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    while (!kIsNull(obj->activeBuffers))
    {
        kSerializerBuffer* next = obj->activeBuffers->next; 

        kCheck(kStream_Write_(obj->stream, obj->activeBuffers->data, obj->activeBuffers->written)); 

        obj->activeBuffers->next = obj->freeBuffers; 
        obj->freeBuffers = obj->activeBuffers;

        obj->activeBuffers = next; 
    }

    obj->currentBuffer = kNULL; 
   
    return kOK; 
}

kFx(kStatus) kSerializer_FindCompatibleVersion(kSerializer serializer, kType type, kTypeVersion* version)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kAssembly assembly = kType_Assembly_(type); 
    kVersion requestedVersion; 
    kSSize versionCount = (kSSize) kType_VersionCount(type); 
    kSSize i = 0; 

    if (obj->formatVersion != k32U_NULL)
    {
        requestedVersion = obj->formatVersion; 
    }
    else if (!kSuccess(kMap_Find(obj->assemblyVersions, &assembly, &requestedVersion)))
    {
        requestedVersion = k32U_NULL;   //use the highest available version
    }

    //type versions are stored by increasing format version; 
    //can start at end of list and work backwards to find a compatible match.
    for (i = versionCount-1; i >= 0; --i)
    {
        kTypeVersion typeVer = kType_VersionAt(type, (kSize)i); 

        if (kStrEquals(obj->format, kType_VersionFormat(type, typeVer)) && (kType_VersionFormatVersion(type, typeVer) <= requestedVersion))
        {
            *version = typeVer; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kSerializer_Write1N(kSerializer serializer, const void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize remaining = count; 
    kBool autoFlush = kSerializer_CanAutoFlush_(serializer); 

    if (autoFlush && (count > kSERIALIZER_MAX_BUFFERED_WRITE_SIZE))
    {
        kCheck(kSerializer_FlushBuffers(serializer));
        
        kCheck(kStream_Write_(obj->stream, items, count));
            
        kCheck(kSerializer_AddBuffer(serializer));     	
    }
    else
    {    
    	while (remaining > 0)
    	{
    		kSize maxItems = obj->currentBuffer->capacity - obj->currentBuffer->written; 
    		kSize itemCount = kMin_(maxItems, remaining); 
    		const kByte* src = (const kByte*)items + (count - remaining); 
    		kByte* dest = (kByte*)obj->currentBuffer->data + obj->currentBuffer->written; 
   
    		kCheck(kMemCopy(dest, src, itemCount)); 

    		obj->currentBuffer->written += itemCount; 

    		remaining -= itemCount; 

    		if (remaining > 0)
    		{
    			if (autoFlush)  
    			{
    				kCheck(kSerializer_FlushBuffers(serializer));
    			}
    			kCheck(kSerializer_AddBuffer(serializer)); 
    		}
    	}
    }

    return kOK; 
}

kFx(kStatus) kSerializer_Write2N(kSerializer serializer, const void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    const k32u ITEM_SHIFT = 1; 
    const kSize ITEM_SIZE = (kSize) (1 << ITEM_SHIFT); 
    kBool autoFlush = kSerializer_CanAutoFlush_(serializer); 
    kSize remaining = count; 
    kSize i; 

    if (autoFlush && !obj->swap && (count > kSERIALIZER_MAX_BUFFERED_WRITE_SIZE))
    {
        kCheck(kSerializer_FlushBuffers(serializer));
        
        kCheck(kStream_Write_(obj->stream, items, ITEM_SIZE * count));
            
        kCheck(kSerializer_AddBuffer(serializer));     	
    }
    else
    {
        while (remaining > 0)
        {
            kSize maxItems = (obj->currentBuffer->capacity - obj->currentBuffer->written) >> ITEM_SHIFT;
            kSize itemCount = kMin_(maxItems, remaining);
            const kByte* src = (kByte*)items + ITEM_SIZE * (count - remaining);
            kByte* dest = (kByte*)obj->currentBuffer->data + obj->currentBuffer->written;

            if (obj->swap)
            {
                for (i = 0; i < itemCount; ++i)
                {
                    kItemSwap2_(&dest[ITEM_SIZE * i], &src[ITEM_SIZE * i]);
                }
            }
            else
            {
                kCheck(kMemCopy(dest, src, ITEM_SIZE * itemCount));
            }

            obj->currentBuffer->written += ITEM_SIZE * itemCount;

            remaining -= itemCount;

            if (remaining > 0)
            {
                if (autoFlush)
                {
                    kCheck(kSerializer_FlushBuffers(serializer));
                }
                kCheck(kSerializer_AddBuffer(serializer));
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_Write4N(kSerializer serializer, const void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    const k32u ITEM_SHIFT = 2; 
    const kSize ITEM_SIZE = (kSize) (1 << ITEM_SHIFT); 
    kBool autoFlush = kSerializer_CanAutoFlush_(serializer); 
    kSize remaining = count; 
    kSize i; 

    if (autoFlush && !obj->swap && (count > kSERIALIZER_MAX_BUFFERED_WRITE_SIZE))
    {
        kCheck(kSerializer_FlushBuffers(serializer));
        
        kCheck(kStream_Write_(obj->stream, items, ITEM_SIZE * count));
            
        kCheck(kSerializer_AddBuffer(serializer));     	
    }
    else
    {
        while (remaining > 0)
        {
            kSize maxItems = (obj->currentBuffer->capacity - obj->currentBuffer->written) >> ITEM_SHIFT;
            kSize itemCount = kMin_(maxItems, remaining);
            const kByte* src = (kByte*)items + ITEM_SIZE * (count - remaining);
            kByte* dest = (kByte*)obj->currentBuffer->data + obj->currentBuffer->written;

            if (obj->swap)
            {
                for (i = 0; i < itemCount; ++i)
                {
                    kItemSwap4_(&dest[ITEM_SIZE * i], &src[ITEM_SIZE * i]);
                }
            }
            else
            {
                kCheck(kMemCopy(dest, src, ITEM_SIZE * itemCount));
            }

            obj->currentBuffer->written += ITEM_SIZE * itemCount;

            remaining -= itemCount;

            if (remaining > 0)
            {
                if (autoFlush)
                {
                    kCheck(kSerializer_FlushBuffers(serializer));
                }
                kCheck(kSerializer_AddBuffer(serializer));
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_Write8N(kSerializer serializer, const void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    const k32u ITEM_SHIFT = 3; 
    const kSize ITEM_SIZE = (kSize) (1 << ITEM_SHIFT); 
    kBool autoFlush = kSerializer_CanAutoFlush_(serializer); 
    kSize remaining = count; 
    kSize i; 

    if (autoFlush && !obj->swap && (count > kSERIALIZER_MAX_BUFFERED_WRITE_SIZE))
    {
        kCheck(kSerializer_FlushBuffers(serializer));
        
        kCheck(kStream_Write_(obj->stream, items, ITEM_SIZE * count));
            
        kCheck(kSerializer_AddBuffer(serializer));     	
    }
    else
    {
        while (remaining > 0)
        {
            kSize maxItems = (obj->currentBuffer->capacity - obj->currentBuffer->written) >> ITEM_SHIFT;
            kSize itemCount = kMin_(maxItems, remaining);
            const kByte* src = (kByte*)items + ITEM_SIZE * (count - remaining);
            kByte* dest = (kByte*)obj->currentBuffer->data + obj->currentBuffer->written;

            if (obj->swap)
            {
                for (i = 0; i < itemCount; ++i)
                {
                    kItemSwap8_(&dest[ITEM_SIZE * i], &src[ITEM_SIZE * i]);
                }
            }
            else
            {
                kCheck(kMemCopy(dest, src, ITEM_SIZE * itemCount));
            }

            obj->currentBuffer->written += ITEM_SIZE * itemCount;

            remaining -= itemCount;

            if (remaining > 0)
            {
                if (autoFlush)
                {
                    kCheck(kSerializer_FlushBuffers(serializer));
                }
                kCheck(kSerializer_AddBuffer(serializer));
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_Read1N(kSerializer serializer, void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    
    kCheck(kStream_Read_(obj->stream, items, count)); 

    return kOK; 
}

kFx(kStatus) kSerializer_Read2N(kSerializer serializer, void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    
    kCheck(kStream_Read_(obj->stream, items, 2*count)); 

    if (obj->swap)
    {
        k16u* items2 = (k16u*)items; 
        k16u temp; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kItemSwap2_(&temp, &items2[i]); 
            items2[i] = temp; 
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_Read4N(kSerializer serializer, void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    
    kCheck(kStream_Read_(obj->stream, items, 4*count)); 

    if (obj->swap)
    {
        k32u* items4 = (k32u*)items; 
        k32u temp; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kItemSwap4_(&temp, &items4[i]); 
            items4[i] = temp; 
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_Read8N(kSerializer serializer, void* items, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    
    kCheck(kStream_Read_(obj->stream, items, 8*count)); 

    if (obj->swap)
    {
        k64u* items8 = (k64u*)items; 
        k64u temp; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kItemSwap8_(&temp, &items8[i]); 
            items8[i] = temp; 
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_WriteText(kSerializer serializer, const kChar* data)
{
    kChar chr = 0; 

    do
    {
        chr = *data++; 

        kCheck(kSerializer_WriteChar_(serializer, chr)); 
    }
    while (chr != 0); 

    return kOK; 
}

kFx(kStatus) kSerializer_ReadText(kSerializer serializer, kChar* data, kSize capacity)
{
    kChar* end = data + capacity; 
    kChar chr = 0; 
    
    kCheckArgs(capacity > 0); 

    do
    {
        kCheck(kSerializer_ReadChar_(serializer, &chr)); 

        *data++ = chr; 
    }
    while ((chr != 0) && (data != end)); 

    return (chr == 0) ? kOK : kERROR_INCOMPLETE; 
}

kFx(kStatus) kSerializer_WriteByte(kSerializer serializer, kByte data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteByte_(obj, data); 
}

kFx(kStatus) kSerializer_WriteByteArray(kSerializer serializer, const kByte* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteByteArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_ReadByte(kSerializer serializer, kByte* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadByte_(obj, data); 
}

kFx(kStatus) kSerializer_ReadByteArray(kSerializer serializer, kByte* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadByteArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_WriteChar(kSerializer serializer, kChar data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteChar_(obj, data); 
}

kFx(kStatus) kSerializer_WriteCharArray(kSerializer serializer, const kChar* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteCharArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_ReadChar(kSerializer serializer, kChar* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadChar_(obj, data); 
}

kFx(kStatus) kSerializer_ReadCharArray(kSerializer serializer, kChar* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadCharArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write8u(kSerializer serializer, k8u data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write8u_(obj, data); 
}

kFx(kStatus) kSerializer_Write8uArray(kSerializer serializer, const k8u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write8uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read8u(kSerializer serializer, k8u* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read8u_(obj, data); 
}

kFx(kStatus) kSerializer_Read8uArray(kSerializer serializer, k8u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read8uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write8s(kSerializer serializer, k8s data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write8s_(obj, data); 
}

kFx(kStatus) kSerializer_Write8sArray(kSerializer serializer, const k8s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write8sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read8s(kSerializer serializer, k8s* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read8s_(obj, data); 
}

kFx(kStatus) kSerializer_Read8sArray(kSerializer serializer, k8s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read8sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write16u(kSerializer serializer, k16u data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write16u_(obj, data); 
}

kFx(kStatus) kSerializer_Write16uArray(kSerializer serializer, const k16u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write16uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read16u(kSerializer serializer, k16u* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read16u_(obj, data); 
}

kFx(kStatus) kSerializer_Read16uArray(kSerializer serializer, k16u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read16uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write16s(kSerializer serializer, k16s data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write16s_(obj, data); 
}

kFx(kStatus) kSerializer_Write16sArray(kSerializer serializer, const k16s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write16sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read16s(kSerializer serializer, k16s* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read16s_(obj, data); 
}

kFx(kStatus) kSerializer_Read16sArray(kSerializer serializer, k16s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read16sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write32u(kSerializer serializer, k32u data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write32u_(obj, data); 
}

kFx(kStatus) kSerializer_Write32uArray(kSerializer serializer, const k32u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write32uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read32u(kSerializer serializer, k32u* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read32u_(obj, data); 
}

kFx(kStatus) kSerializer_Read32uArray(kSerializer serializer, k32u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read32uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write32s(kSerializer serializer, k32s data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write32s_(obj, data); 
}

kFx(kStatus) kSerializer_Write32sArray(kSerializer serializer, const k32s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write32sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read32s(kSerializer serializer, k32s* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read32s_(obj, data); 
}

kFx(kStatus) kSerializer_Read32sArray(kSerializer serializer, k32s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read32sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write64u(kSerializer serializer, k64u data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write64u_(obj, data); 
}

kFx(kStatus) kSerializer_Write64uArray(kSerializer serializer, const k64u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write64uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read64u(kSerializer serializer, k64u* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read64u_(obj, data); 
}

kFx(kStatus) kSerializer_Read64uArray(kSerializer serializer, k64u* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read64uArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write64s(kSerializer serializer, k64s data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write64s_(obj, data); 
}

kFx(kStatus) kSerializer_Write64sArray(kSerializer serializer, const k64s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write64sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read64s(kSerializer serializer, k64s* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read64s_(obj, data); 
}

kFx(kStatus) kSerializer_Read64sArray(kSerializer serializer, k64s* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read64sArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write32f(kSerializer serializer, k32f data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write32f_(obj, data); 
}

kFx(kStatus) kSerializer_Write32fArray(kSerializer serializer, const k32f* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write32fArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read32f(kSerializer serializer, k32f* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read32f_(obj, data); 
}

kFx(kStatus) kSerializer_Read32fArray(kSerializer serializer, k32f* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read32fArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Write64f(kSerializer serializer, k64f data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write64f_(obj, data); 
}

kFx(kStatus) kSerializer_Write64fArray(kSerializer serializer, const k64f* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Write64fArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_Read64f(kSerializer serializer, k64f* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read64f_(obj, data); 
}

kFx(kStatus) kSerializer_Read64fArray(kSerializer serializer, k64f* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_Read64fArray_(obj, data, count); 
}

kFx(kStatus) kSerializer_WriteSize(kSerializer serializer, kSize data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteSize_(obj, data); 
}

#if (K_POINTER_SIZE == 4)

kFx(kStatus) kSerializer_WriteSizeArray(kSerializer serializer, const kSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Write4N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_Write64u_(serializer, data[i])); 
        }
    }
    return kOK; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kStatus) kSerializer_WriteSizeArray(kSerializer serializer, const kSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Write8N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            if (data[i] > k32U_MAX)
            {
                return kERROR_FORMAT; 
            }
            else
            {
                kCheck(kSerializer_Write32u_(serializer, data[i])); 
            }
        }
    }
    return kOK; 
}

#endif

kFx(kStatus) kSerializer_ReadSize(kSerializer serializer, kSize* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadSize_(obj, data); 
}


#if (K_POINTER_SIZE == 4)

kFx(kStatus) kSerializer_ReadSizeArray(kSerializer serializer, kSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Read4N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k64u value; 

            kCheck(kSerializer_Read64u_(serializer, &value)); 

            if (value > k32U_MAX)
            {
                return kERROR_FORMAT; 
            }
            else
            {
                kSetAs_(&data[i], value, kSize); 
            }
        }
    }
    return kOK; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kStatus) kSerializer_ReadSizeArray(kSerializer serializer, kSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Read8N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k32u value; 

            kCheck(kSerializer_Read32u_(serializer, &value)); 
        
            kSetAs_(&data[i], value, kSize); 
        }
    }

    return kOK; 
}

#endif


kFx(kStatus) kSerializer_WriteSSize(kSerializer serializer, kSSize data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_WriteSSize_(obj, data); 
}

#if (K_POINTER_SIZE == 4)

kFx(kStatus) kSerializer_WriteSSizeArray(kSerializer serializer, const kSSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Write4N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_Write64s_(serializer, data[i])); 
        }
    }
    return kOK; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kStatus) kSerializer_WriteSSizeArray(kSerializer serializer, const kSSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Write8N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            if ((data[i] > k32S_MAX) || (data[i] < k32S_MIN))
            {
                return kERROR_FORMAT; 
            }
            else
            {
                kCheck(kSerializer_Write32s_(serializer, data[i])); 
            }
        }
    }
    return kOK; 
}

#endif

kFx(kStatus) kSerializer_ReadSSize(kSerializer serializer, kSSize* data)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);

    return kSerializer_ReadSSize_(obj, data); 
}


#if (K_POINTER_SIZE == 4)

kFx(kStatus) kSerializer_ReadSSizeArray(kSerializer serializer, kSSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Read4N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k64s value; 

            kCheck(kSerializer_Read64s_(serializer, &value)); 

            if ((value > k32S_MAX) || (value < k32S_MIN))
            {
                return kERROR_FORMAT; 
            }
            else
            {
                kSetAs_(&data[i], value, kSSize); 
            }
        }
    }
    return kOK; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kStatus) kSerializer_ReadSSizeArray(kSerializer serializer, kSSize* data, kSize count)
{
    kSerializerClass* obj = kSerializer_Cast_(serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Read8N(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k32s value; 

            kCheck(kSerializer_Read32s_(serializer, &value)); 
        
            kSetAs_(&data[i], value, kSSize); 
        }
    }

    return kOK; 
}

#endif
