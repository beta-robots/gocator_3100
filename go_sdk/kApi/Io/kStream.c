/** 
 * @file    kStream.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kStream.h>

kBeginVirtualClass(k, kStream, kObject)

    kAddFlags(kStream, kTYPE_FLAGS_ABSTRACT)

    kAddVMethod(kStream, kStream, VReadSomeImpl)
    kAddVMethod(kStream, kStream, VReadImpl)        //deprecated
    kAddVMethod(kStream, kStream, VWriteImpl)
    kAddVMethod(kStream, kStream, VSeek)
    kAddVMethod(kStream, kStream, VFlush)

kEndVirtualClass()

kFx(kStatus) kStream_Init(kStream stream, kType type, kAlloc allocator)
{
    kCheck(kObject_Init(stream, type, allocator)); 

    kInitFields_(kStream, stream); 

    return kOK; 
}

kFx(kStatus) kStream_VRelease(kStream stream)
{
    return kObject_VRelease(stream);
}

kFx(kStatus) kStream_Read(kStream stream, void* buffer, kSize size)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    return kStream_Read_(obj, buffer, size); 
}

kFx(kStatus) kStream_ReadSome(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kAssertType(stream, kStream); 

    return kStream_VTable_(stream)->VReadSomeImpl(stream, buffer, minCount, maxCount, bytesRead); 
}

kFx(kStatus) kStream_VReadSomeImpl(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    //legacy support: for classes that haven't been updated to implement VReadSomeImpl, try VReadImpl instead
    kCheck(kStream_VTable_(stream)->VReadImpl(stream, buffer, minCount)); 
    
    if (!kIsNull(bytesRead))
    {
        *bytesRead = minCount; 
    }

    return kOK; 
}

//deprecated
kFx(kStatus) kStream_VReadImpl(kStream stream, void* buffer, kSize size)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kStream_Write(kStream stream, const void* buffer, kSize size)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    return kStream_Write_(obj, buffer, size); 
}

kFx(kStatus) kStream_VWriteImpl(kStream stream, const void* buffer, kSize size)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kStream_Copy(kStream stream, kStream source, kSize size)
{
    return kStream_CopyEx(stream, source, size, kNULL, kNULL); 
}

kFx(kStatus) kStream_CopyEx(kStream stream, kStream source, kSize size, kCallbackFx progress, kPointer context)
{
    kByte buffer[256]; 
    kSize bytesCopied = 0; 
    k64u updateTime = k64U_NULL; 

    while (bytesCopied < size)
    {
        kSize bytesRemaining = size - bytesCopied; 
        kSize copySize = kMin_(bytesRemaining, kCountOf(buffer)); 

        kCheck(kStream_Read(source, buffer, copySize)); 
        kCheck(kStream_Write(stream, buffer, copySize)); 

        bytesCopied += copySize; 

        kUpdateProgress(progress, context, kNULL, &updateTime, (k32u)(100*bytesCopied/size)); 
    }

    kUpdateProgress(progress, context, kNULL, kNULL, 100); 
  
    return kOK; 
}

kFx(kStatus) kStream_Seek(kStream stream, k64s offset, kSeekOrigin origin)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    return kStream_VTable_(stream)->VSeek(obj, offset, origin); 
}

kFx(kStatus) kStream_VSeek(kStream stream, k64s offset, kSeekOrigin origin)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kStream_Flush(kStream stream)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    return kStream_VTable_(stream)->VFlush(obj); 
}

kFx(kStatus) kStream_VFlush(kStream stream)
{
    return kOK; 
}

kFx(k64u) kStream_BytesRead(kStream stream)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    return kStream_BytesRead_(obj); 
}

kFx(k64u) kStream_BytesWritten(kStream stream)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    return kStream_BytesWritten_(obj); 
}

kFx(kStatus) kStream_ClearStats(kStream stream)
{
    kStreamClass* obj = kStream_Cast_(stream); 

    obj->bytesRead = 0; 
    obj->bytesWritten = 0; 

    return kOK; 
}
