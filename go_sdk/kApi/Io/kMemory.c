/** 
 * @file    kMemory.c
 * @brief   Declares the kMemory class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kMemory.h>

kBeginClass(k, kMemory, kStream)
    kAddVMethod(kMemory, kObject, VRelease)
    kAddVMethod(kMemory, kStream, VReadSomeImpl)
    kAddVMethod(kMemory, kStream, VWriteImpl)
    kAddVMethod(kMemory, kStream, VSeek)
    kAddVMethod(kMemory, kStream, VFlush)
kEndClass()

kFx(kStatus) kMemory_Construct(kMemory* memory, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kMemory); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, memory)); 

    if (!kSuccess(status = kMemory_Init(*memory, type, alloc)))
    {
        kAlloc_FreeRef(alloc, memory); 
    }

    return status; 
} 

kFx(kStatus) kMemory_Init(kMemory memory, kType type, kAlloc allocator)
{
    kMemoryClass *obj = memory;  
 
    kCheck(kStream_Init(memory, type, allocator));

    obj->buffer = 0; 
    obj->position = 0; 
    obj->length = 0; 
    obj->capacity = 0; 
    obj->owned = kTRUE; 
    obj->lastMode = kMEMORY_MODE_NULL; 
       
    return kOK;
}

kFx(kStatus) kMemory_VRelease(kMemory memory)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    if (obj->owned)
    {
        kCheck(kObject_FreeMemRef_(memory, &obj->buffer)); 
    }

    kCheck(kStream_VRelease(memory)); 
    
    return kOK;
}

kFx(kStatus) kMemory_Attach(kMemory memory, void* buffer, kSize position, kSize length, kSize capacity)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  
    
    if (obj->owned)
    {
        kCheck(kObject_FreeMemRef_(memory, &obj->buffer)); 
        obj->capacity = 0; 
    }       

    obj->buffer = buffer; 
    obj->position = position; 
    obj->length = length; 
    obj->capacity = capacity; 
    obj->owned = kFALSE; 

    obj->base.readBegin = obj->base.readEnd = obj->base.readCapacity = 0; 
    obj->base.readBuffer = 0; 
    obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 
    obj->base.writeBuffer = 0; 

    obj->base.bytesRead = 0; 
    obj->base.bytesWritten = 0; 

    obj->lastMode = kMEMORY_MODE_NULL; 

    return kOK; 
}

kFx(kBool) kMemory_IsAttached(kMemory memory)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    return !obj->owned;     
}

kFx(kStatus) kMemory_Allocate(kMemory memory, kSize initialCapacity)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  
       
    if (obj->owned)
    {
        kCheck(kObject_FreeMemRef_(memory, &obj->buffer)); 
        obj->capacity = 0; 
    }       

    kCheck(kObject_GetMem_(memory, initialCapacity, &obj->buffer)); 

    obj->position = 0; 
    obj->length = 0; 
    obj->capacity = initialCapacity; 
    obj->owned = kTRUE; 

    obj->base.readBegin = obj->base.readEnd = obj->base.readCapacity = 0; 
    obj->base.readBuffer = 0; 
    obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 
    obj->base.writeBuffer = 0; 

    obj->base.bytesRead = 0; 
    obj->base.bytesWritten = 0; 

    obj->lastMode = kMEMORY_MODE_NULL; 

    return kOK; 
}

kFx(kStatus) kMemory_Reserve(kMemory memory, kSize minimumCapacity)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    if (obj->capacity < minimumCapacity)
    {
        kSize newCapacity = 0; 
        void* newBuffer = kNULL;  

        newCapacity = kMax_(minimumCapacity, kMEMORY_GROWTH_FACTOR*obj->capacity);
        newCapacity = kMax_(newCapacity, kMEMORY_MIN_CAPACITY); 

        kCheck(kObject_GetMem_(memory, newCapacity, &newBuffer)); 

        kMemCopy(newBuffer, obj->buffer, obj->length);        
        
        kObject_FreeMem_(memory, obj->buffer); 

        obj->buffer = newBuffer; 
        obj->capacity = newCapacity; 
    }
    
    return kOK; 
}

kFx(kStatus) kMemory_VReadSomeImpl(kMemory memory, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  
    kSize copyCount; 
           
    kCheckState((kMemory_Position(memory) + minCount) <= obj->length); 

    //configure the stream for reading, if necessary
    if (obj->lastMode != kMEMORY_MODE_READ)
    {
        kCheck(kStream_Flush_(memory)); 

        obj->base.readBuffer = kAt_(obj->buffer, obj->position); 
        obj->base.readCapacity = obj->length - obj->position; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = obj->length - obj->position; 
        obj->base.writeBuffer = 0; 
        obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 

        obj->base.bytesRead += (obj->base.readEnd - obj->base.readBegin); 

        obj->lastMode = kMEMORY_MODE_READ; 
    }

    //read the requested bytes
    copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin); 

    kCheck(kMemCopy(buffer, &obj->base.readBuffer[obj->base.readBegin], copyCount)); 
    obj->base.readBegin += copyCount; 

    if (!kIsNull(bytesRead))
    {
        *bytesRead = copyCount; 
    }
    
    return kOK; 
}

kFx(kStatus) kMemory_VWriteImpl(kMemory memory, const void* buffer, kSize size)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    //extend the memory buffer, if necessary
    if ((kMemory_Position(memory) + size) > obj->capacity)
    {
        if (!obj->owned)
        {
            return kERROR_STREAM; 
        }
        
        kCheck(kStream_Flush_(memory)); 
        kCheck(kMemory_Reserve(memory, (kSize)kMemory_Position(memory) + size)); 
    }

    //configure the stream for writing, if necessary
    if (obj->lastMode != kMEMORY_MODE_WRITE)
    {
        kCheck(kStream_Flush_(memory)); 

        obj->base.writeBuffer = kAt_(obj->buffer, obj->position); 
        obj->base.writeBegin = 0; 
        obj->base.writeEnd = (obj->capacity - obj->position); 
        obj->base.writeCapacity = (obj->capacity - obj->position); 
        obj->base.readBuffer = kNULL; 
        obj->base.readCapacity = 0; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = 0; 
        obj->lastMode = kMEMORY_MODE_WRITE; 
    }

    //write the requested bytes
    kCheck(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size)); 
    obj->base.writeBegin += size; 

    return kOK; 
}

kFx(kStatus) kMemory_VSeek(kMemory memory, k64s offset, kSeekOrigin origin)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  
    k64u originPosition;   
    k64u newPosition; 

    kCheck(kStream_Flush_(memory)); 

    switch(origin)
    {
    case kSEEK_ORIGIN_BEGIN:        originPosition = 0;                 break;
    case kSEEK_ORIGIN_CURRENT:      originPosition = obj->position;     break;
    case kSEEK_ORIGIN_END:          originPosition = obj->length;       break;
    default:                        return kERROR_PARAMETER;
    }

    newPosition = (k64u) (originPosition + offset); 

    kCheckArgs(newPosition <= obj->length); 

    obj->position = (kSize) newPosition; 

    return kOK; 
}

kFx(kStatus) kMemory_VFlush(kMemory memory)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  
    
    if (obj->lastMode == kMEMORY_MODE_READ)
    {
        obj->base.bytesRead -= (k64u) (obj->base.readEnd - obj->base.readBegin);
        obj->position += obj->base.readBegin; 
    }
    else if (obj->lastMode == kMEMORY_MODE_WRITE)
    {
        obj->base.bytesWritten += (k64u)obj->base.writeBegin; 
        obj->position += obj->base.writeBegin; 
        obj->length = kMax_(obj->length, obj->position); 
    }
   
    obj->base.readBuffer = kNULL; 
    obj->base.readCapacity = 0; 
    obj->base.readBegin = 0; 
    obj->base.readEnd = 0; 
    obj->base.writeBuffer = kNULL; 
    obj->base.writeCapacity = 0; 
    obj->base.writeEnd = 0; 
    obj->base.writeBegin = 0; 

    obj->lastMode = kMEMORY_MODE_NULL; 

    return kOK; 
}

kFx(kStatus) kMemory_SetLength(kMemory memory, kSize length)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  
    
    kCheck(kStream_Flush_(memory)); 
    kCheck(kMemory_Reserve(memory, length)); 

    obj->length = length; 
    obj->position = 0; 

    return kOK; 
}

kFx(k64u) kMemory_Length(kMemory memory)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    if (obj->lastMode == kMEMORY_MODE_WRITE)
    {
        return obj->length + obj->base.writeBegin; 
    }
    else
    {
        return obj->length; 
    }
}

kFx(kSize) kMemory_Capacity(kMemory memory)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    return obj->capacity; 
}

kFx(k64u) kMemory_Position(kMemory memory)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    if (obj->lastMode == kMEMORY_MODE_WRITE)
    {
        return obj->position + obj->base.writeBegin; 
    }
    else if (obj->lastMode == kMEMORY_MODE_READ)
    {
        return obj->position + obj->base.readBegin;
    }
    else
    {
        return obj->position; 
    }
}

kFx(void*) kMemory_At(kMemory memory, kSize offset)
{
    kMemoryClass *obj = kMemory_Cast_(memory);  

    return (offset > obj->length) ? kNULL : &obj->buffer[offset]; 
}

//K_COMPAT_5
kFx(kStatus) kMemory_Construct5(kMemory* memory, void* buffer, kSize capacity)
{
    kStatus exception = kOK; 
    kMemory output = kNULL; 

    kTry
    {
        kTest(kMemory_Construct(&output, kNULL)); 

        if (kIsNull(buffer))
        {
            kTest(kMemory_Allocate(output, capacity)); 
        }
        else
        {
            kTest(kMemory_Attach(output, buffer, 0, capacity, capacity)); 
        }

        *memory = output; 
    }
    kCatch(&exception)
    {
        kObject_Destroy(output); 
        kEndCatch(exception); 
    }

    return kOK; 
}
