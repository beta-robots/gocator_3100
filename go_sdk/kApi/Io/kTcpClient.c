/** 
 * @file    kTcpClient.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kTcpClient.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Threads/kTimer.h>

kBeginClass(k, kTcpClient, kStream)
    kAddVMethod(kTcpClient, kObject, VRelease)
    kAddVMethod(kTcpClient, kStream, VReadSomeImpl)
    kAddVMethod(kTcpClient, kStream, VWriteImpl)
    kAddVMethod(kTcpClient, kStream, VFlush)
kEndClass()

kFx(kStatus) kTcpClient_Construct(kTcpClient* client, kIpVersion ipVersion, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kTcpClient), client)); 

    if (!kSuccess(status = kTcpClient_Init(*client, kTypeOf(kTcpClient), ipVersion, alloc)))
    {
        kAlloc_FreeRef(alloc, client); 
    }

    return status; 
} 

kFx(kStatus) kTcpClient_ConstructFromSocket(kTcpClient* client, kSocket socket, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kTcpClient); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, client)); 

    if (!kSuccess(status = kTcpClient_InitFromSocket(*client, type, socket, alloc)))
    {
        kAlloc_FreeRef(alloc, client); 
    }

    return status; 
} 

kFx(kStatus) kTcpClient_Init(kTcpClient client, kType type, kIpVersion ipVersion, kAlloc allocator)
{
    kSocket socket = kNULL; 
    kStatus status;     

    kTry
    {
        kTest(kSocket_Construct(&socket, kIP_VERSION_4, kSOCKET_TYPE_TCP, allocator));
        kTest(kSocket_Bind(socket, kIpAddress_Any(ipVersion), kIP_PORT_ANY));

        kTest(kTcpClient_InitFromSocket(client, type, socket, allocator));
    }
    kCatch(&status)
    {
        kDestroyRef(&socket); 
        kEndCatch(status); 
    }
    
    return kOK; 
}

kFx(kStatus) kTcpClient_InitFromSocket(kTcpClient client, kType type, kSocket socket, kAlloc allocator)
{
    kTcpClientClass* obj = client; 
    kStatus status; 

    kCheckArgs(!kIsNull(socket)); 
    
    kCheck(kStream_Init(client, type, allocator)); 
    
    kInitFields_(kTcpClient, client); 

    kTry
    {
        kTest(kSocket_SetBlocking(socket, kFALSE)); 

        obj->socket = socket;
    }
    kCatch(&status)
    {
        kTcpClient_VRelease(client); 
        kEndCatch(status); 
    }
  
    return kOK; 
}

kFx(kStatus) kTcpClient_VRelease(kTcpClient client)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
        
    kCheck(kObject_Destroy(obj->socket));    
    
    kCheck(kObject_FreeMem(client, obj->base.readBuffer));   
    kCheck(kObject_FreeMem(client, obj->base.writeBuffer));

    kCheck(kStream_VRelease(client)); 
        
    return kOK;
}

kFx(kStatus) kTcpClient_SetWriteBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
    
    kCheck(kStream_Flush_(client)); 
    
    if (socketSize >= 0)
    {
        kCheck(kSocket_SetWriteBuffer(obj->socket, (kSize)socketSize));
    }

    if (clientSize >= 0)
    {
        kCheck(kObject_FreeMemRef(client, &obj->base.writeBuffer)); 
        
        obj->base.writeCapacity = 0; 
        obj->base.writeBegin = 0; 
        obj->base.writeEnd = 0; 

        if (clientSize > 0)
        {
            kCheck(kObject_GetMem(client, (kSize)clientSize, &obj->base.writeBuffer)); 
            obj->base.writeCapacity = clientSize; 
            obj->base.writeEnd = clientSize; 
        }
    }
    
    return kOK;
}

kFx(kStatus) kTcpClient_SetReadBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    kCheckState((obj->base.readEnd - obj->base.readBegin) == 0);

    if (socketSize >= 0)
    {
        kCheck(kSocket_SetReadBuffer(obj->socket, (kSize)socketSize));
    }

    if (clientSize >= 0)
    {
        kCheck(kObject_FreeMemRef(client, &obj->base.readBuffer)); 

        obj->base.readCapacity = 0; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = 0; 

        if (clientSize > 0)
        {
            kCheck(kObject_GetMem(client, (kSize)clientSize, &obj->base.readBuffer));                 
            obj->base.readCapacity = clientSize; 
        }
    }
    
    return kOK;
}

kFx(kStatus) kTcpClient_SetWriteTimeout(kTcpClient client, k64u timeout)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    obj->writeTimeout = timeout; 

    return kOK; 
}

kFx(kStatus) kTcpClient_SetReadTimeout(kTcpClient client, k64u timeout)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    obj->readTimeout = timeout; 

    return kOK; 
}

kFx(kStatus) kTcpClient_SetCancelHandler(kTcpClient client, kCallbackFx function, kPointer receiver)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
        
    obj->cancelQuery.function = function; 
    obj->cancelQuery.receiver = receiver; 

    return kOK; 
}

kFx(kStatus) kTcpClient_Cancel(kTcpClient client)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
        
    kAtomic32s_Exchange_(&obj->isCancelled, kTRUE);

    return kOK; 
}

kFx(kStatus) kTcpClient_Connect(kTcpClient client, kIpAddress address, k32u port, k64u timeout)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
   
    return kSocket_Connect(obj->socket, address, port, timeout);
}

kFx(kStatus) kTcpClient_Wait(kTcpClient client, k64u timeout)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
    
    if (kTcpClient_Available(client) > 0)
    {
        return kOK;
    }
    else
    {
        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 

        return kSocket_Wait(obj->socket, timeout);    
    }
}

kFx(kStatus) kTcpClient_VFlush(kTcpClient client)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
        
    if (obj->base.writeBegin > 0)
    {
        kCheck(kTcpClient_WriteAll(client, obj->base.writeBuffer, obj->base.writeBegin));
        obj->base.writeBegin = 0; 
    }
            
    return kOK;
}

kFx(kStatus) kTcpClient_VReadSomeImpl(kTcpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
    kByte* dest = buffer;  
    kSize readCount = 0; 
    kSize copyCount, mediumCount; 
    
    //consume any bytes in the read buffer first
    if ((obj->base.readEnd - obj->base.readBegin) > 0)
    {
        copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin);

        kMemCopy(dest, &obj->base.readBuffer[obj->base.readBegin], copyCount);     
        
        obj->base.readBegin += copyCount; 
        readCount += copyCount; 
    }

    //if the request is not yet satisfied
    if (readCount < minCount)
    {
        //if the request is larger than the internal read buffer, read directly into the caller's buffer; else read into the internal buffer
        if ((maxCount - readCount) >= obj->base.readCapacity)
        {            
            kCheck(kTcpClient_ReadAtLeast(client, &dest[readCount], minCount-readCount, maxCount-readCount, &mediumCount));

            readCount += mediumCount; 
        }
        else
        {
            kCheck(kTcpClient_ReadAtLeast(client, &obj->base.readBuffer[0], minCount - readCount, obj->base.readCapacity, &mediumCount));    

            copyCount = kMin_(mediumCount, maxCount - readCount); 

            kCheck(kMemCopy(&dest[readCount], &obj->base.readBuffer[0], copyCount)); 

            obj->base.readBegin = copyCount; 
            obj->base.readEnd = mediumCount; 
            readCount += copyCount; 
        }
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = readCount; 
    }

    return kOK; 
}

kFx(kStatus) kTcpClient_VWriteImpl(kTcpClient client, const void* buffer, kSize size)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    //if the internal write buffer already has content, but not enough free space, flush
    if ((obj->base.writeBegin > 0) && ((obj->base.writeEnd - obj->base.writeBegin) < size))
    {
        kCheck(kStream_Flush_(client)); 
    }

    //if the write is larger than the internal write buffer, write directly to the medium; else write to the internal buffer
    if ((obj->base.writeEnd - obj->base.writeBegin) < size)
    {
        kCheck(kTcpClient_WriteAll(client, buffer, size)); 
    }
    else
    {
        kCheck(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size)); 
        obj->base.writeBegin += size; 
    }

    return kOK; 
}

kFx(kStatus) kTcpClient_ReadAtLeast(kTcpClient client, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
    k64u startTime = (obj->readTimeout == 0) ? 0 : kTimer_Now(); 
    kBool hasTimedOut = kFALSE; 
    kStatus result; 
    kSize totalRead = 0; 
    kSize read; 

    kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 

    do
    {
        kCheck(kTcpClient_Status(client)); 

        hasTimedOut = (obj->readTimeout == 0) ? kFALSE : ((kTimer_Now() - startTime) >= obj->readTimeout); 

        result = kSocket_Wait(obj->socket, kTCP_CLIENT_CANCEL_QUERY_INTERVAL); 

        if (result == kOK)
        {
            kCheck(kSocket_Read(obj->socket, &buffer[totalRead], maxCount-totalRead, &read));         
            obj->base.bytesRead += (k64u) read; 
            totalRead += read; 
        }
        else if ((result == kERROR_TIMEOUT) && !kIsNull(obj->cancelQuery.function))
        {
            if (!kSuccess(obj->cancelQuery.function(obj->cancelQuery.receiver, client, kNULL)))
            {
                kAtomic32s_Exchange_(&obj->isCancelled, kTRUE);
                return kERROR_ABORT; 
            }
        }
        else if (result != kERROR_TIMEOUT)
        {
            return result; 
        }
    }
    while ((totalRead < minCount) && !hasTimedOut); 

    if (totalRead < minCount)
    {
        obj->timedOut = kTRUE; 
        return kERROR_TIMEOUT; 
    }

    *bytesRead = totalRead; 

    return kOK;  
}

kFx(kStatus) kTcpClient_WriteAll(kTcpClient client, const kByte* buffer, kSize count)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
    k64u startTime = (obj->writeTimeout == 0) ? 0 : kTimer_Now(); 
    kBool hasTimedOut = kFALSE; 
    kStatus result; 
    kSize totalWritten = 0; 
    kSize written; 

    kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_WRITE)); 

    do
    {
        kCheck(kTcpClient_Status(client)); 

        hasTimedOut = (obj->writeTimeout == 0) ? kFALSE : ((kTimer_Now() - startTime) >= obj->writeTimeout); 

        result = kSocket_Wait(obj->socket, kTCP_CLIENT_CANCEL_QUERY_INTERVAL); 
     
        if (result == kOK)
        {
            kCheck(kSocket_Write(obj->socket, &buffer[totalWritten], count-totalWritten, &written));         
            obj->base.bytesWritten += (k64u) written; 
            totalWritten += written; 
        }
        else if ((result == kERROR_TIMEOUT) && !kIsNull(obj->cancelQuery.function))
        {
            if (!kSuccess(obj->cancelQuery.function(obj->cancelQuery.receiver, client, kNULL)))
            {
                kAtomic32s_Exchange_(&obj->isCancelled, kTRUE);
                return kERROR_ABORT;
            }
        }
        else if (result != kERROR_TIMEOUT)
        {
            return result; 
        }
    }
    while ((totalWritten < count) && !hasTimedOut); 
  
    if (totalWritten < count)
    {
        obj->timedOut = kTRUE; 
        return kERROR_TIMEOUT; 
    }

    return kOK; 
}

kFx(kSocket) kTcpClient_Socket(kTcpClient client)
{ 
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    return obj->socket; 
}

kFx(kSize) kTcpClient_Available(kTcpClient client)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    return obj->base.readEnd - obj->base.readBegin;
}

kFx(kStatus) kTcpClient_LocalEndPoint(kTcpClient client, kIpEndPoint* endPoint)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 
        
    return kSocket_LocalEndPoint(obj->socket, endPoint);
}

kFx(kStatus) kTcpClient_RemoteEndPoint(kTcpClient client, kIpEndPoint* endPoint)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    return kSocket_RemoteEndPoint(obj->socket, endPoint);
}

kFx(kStatus) kTcpClient_Status(kTcpClient client)
{
    kTcpClientClass* obj = kTcpClient_Cast_(client); 

    if (kAtomic32s_Get_(&obj->isCancelled))
    {
        return kERROR_ABORT; 
    }
    else if (obj->timedOut)
    {
        return kERROR_TIMEOUT; 
    }
    else
    {
        return kSocket_Status_(obj->socket); 
    }
}
