/** 
 * @file    kUdpClient.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kUdpClient.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Threads/kTimer.h>

kBeginClass(k, kUdpClient, kStream)
    kAddVMethod(kUdpClient, kObject, VRelease)
    kAddVMethod(kUdpClient, kStream, VReadSomeImpl)
    kAddVMethod(kUdpClient, kStream, VWriteImpl)
    kAddVMethod(kUdpClient, kStream, VFlush)
kEndClass()

kFx(kStatus) kUdpClient_Construct(kUdpClient* client, kIpVersion ipVersion, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kUdpClient), client)); 

    if (!kSuccess(status = kUdpClient_Init(*client, kTypeOf(kUdpClient), ipVersion, alloc)))
    {
        kAlloc_FreeRef(alloc, client); 
    }

    return status; 
} 

kFx(kStatus) kUdpClient_Init(kUdpClient client, kType type, kIpVersion ipVersion, kAlloc allocator)
{
    kUdpClientClass* obj = client; 
    kStatus status; 

    kCheck(kStream_Init(client, type, allocator)); 

    obj->socket = kNULL; 

    kTry
    {
        kTest(kSocket_Construct(&obj->socket, ipVersion, kSOCKET_TYPE_UDP, allocator));
        kTest(kSocket_SetBlocking(obj->socket, kFALSE));
    }
    kCatch(&status)
    {
        kUdpClient_VRelease(client); 
        kEndCatch(status); 
    }
    
    return kOK;
}

kFx(kStatus) kUdpClient_VRelease(kUdpClient client)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
        
    kCheck(kObject_Destroy(obj->socket));    

    kCheck(kObject_FreeMem(client, obj->base.readBuffer));
    kCheck(kObject_FreeMem(client, obj->base.writeBuffer));                

    kCheck(kStream_VRelease(client)); 
    
    return kOK;
}

kFx(kStatus) kUdpClient_Bind(kUdpClient client, kIpAddress address, k32u port)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client);       

//Work-around for DSP/BIOS; see kUdpClient_EnableBroadcastReceive comments
#if defined(K_DSP_BIOS)
    if (obj->isBroacastReceiver)
    {
        address = kIpAddress_AnyV4(); 
    }
#endif

    return kSocket_Bind(obj->socket, address, port);
}

kFx(kStatus) kUdpClient_ReadFrom(kUdpClient client, kIpEndPoint* endPoint, void* buffer, kSize capacity, kSize* received, k64u timeout)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    kStatus opStatus; 

    opStatus = kSocket_ReadFrom(obj->socket, endPoint, buffer, capacity, received); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 
        kCheck(kSocket_Wait(obj->socket, timeout)); 
         
        opStatus = kSocket_ReadFrom(obj->socket, endPoint, buffer, capacity, received); 
    }

    if (kSuccess(opStatus))
    {
        obj->base.bytesRead += (k64u) *received; 
    }

    return opStatus; 
}

kFx(kStatus) kUdpClient_WriteTo(kUdpClient client, const void* buffer, kSize size, kIpAddress address, k32u port, k64u timeout)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client);     
    kStatus opStatus; 

    opStatus = kSocket_WriteTo(obj->socket, address, port, buffer, size); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        k64u startTime = kTimer_Now(); 
        k64u elapsed = 0; 
        k64u opTimeout = 0; 

        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_WRITE)); 
        
        do
        {
            elapsed = kTimer_Now() - startTime; 

            if (elapsed < timeout)
            {
                opTimeout = (timeout == kINFINITE) ? timeout : timeout - elapsed; 

                if (kSuccess(kSocket_Wait(obj->socket, opTimeout)))
                {
                    opStatus = kSocket_WriteTo(obj->socket, address, port, buffer, size); 
                }
            }
        }
        while ((opStatus == kERROR_BUSY) && (elapsed < timeout)); 
    }

    if (opStatus == kERROR_BUSY)
    {
        opStatus = kERROR_TIMEOUT; 
    }

    if (kSuccess(opStatus))
    {
        obj->base.bytesWritten += (k64u) size; 
    }
   
    return opStatus; 
}

kFx(kStatus) kUdpClient_Receive(kUdpClient client, kIpEndPoint* endPoint, kSize* received, k64u timeout)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    kStatus opStatus; 

    kCheckState(!kIsNull(obj->base.readBuffer)); 

    opStatus = kSocket_ReadFrom(obj->socket, endPoint, obj->base.readBuffer, obj->base.readCapacity, &obj->base.readEnd); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 
        kCheck(kSocket_Wait(obj->socket, timeout)); 
        
        opStatus = kSocket_ReadFrom(obj->socket, endPoint, obj->base.readBuffer, obj->base.readCapacity, &obj->base.readEnd); 
    }

    if (kSuccess(opStatus))
    {
        obj->base.readBegin = 0; 
        obj->base.bytesRead += (k64u) obj->base.readEnd; 

        if (!kIsNull(received))
        {
            *received = obj->base.readEnd;
        }
    }
    
    return opStatus;
}

kFx(kStatus) kUdpClient_Send(kUdpClient client, kIpAddress address, k32u port, k64u timeout, kBool clear)
{    
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    kStatus opStatus; 
    
    kCheckState(!kIsNull(obj->base.writeBuffer)); 

    opStatus = kSocket_WriteTo(obj->socket, address, port, obj->base.writeBuffer, obj->base.writeBegin); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        k64u startTime = kTimer_Now(); 
        k64u elapsed = 0; 
        k64u opTimeout = 0; 

        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_WRITE)); 

        do
        {
            elapsed = kTimer_Now() - startTime; 

            if (elapsed < timeout)
            {
                opTimeout = (timeout == kINFINITE) ? timeout : timeout - elapsed; 

                if (kSuccess(kSocket_Wait(obj->socket, opTimeout)))
                {
                    opStatus = kSocket_WriteTo(obj->socket, address, port, obj->base.writeBuffer, obj->base.writeBegin); 
                }
            }
        }
        while ((opStatus == kERROR_BUSY) && (elapsed < timeout)); 
    }

    if (opStatus == kERROR_BUSY)
    {
        opStatus = kERROR_TIMEOUT; 
    }

    if (kSuccess(opStatus))
    {
        obj->base.bytesWritten += (k64u)obj->base.writeBegin; 
    }

    if (clear)
    {
        obj->base.writeBegin = 0; 
    }

    return opStatus; 
}

kFx(kStatus) kUdpClient_Clear(kUdpClient client)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 

    obj->base.writeBegin = 0; 

    return kOK; 
}

kFx(kStatus) kUdpClient_VReadSomeImpl(kUdpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    kSize copyCount; 
    
    kCheckState(!kIsNull(obj->base.readBuffer)); 
    kCheckState(minCount <= (obj->base.readEnd - obj->base.readBegin)); 
      
    copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin); 

    kMemCopy(buffer, &obj->base.readBuffer[obj->base.readBegin], copyCount); 
    obj->base.readBegin += copyCount; 

    if (!kIsNull(bytesRead))
    {
        *bytesRead = copyCount; 
    }
         
    return kOK;
}

kFx(kStatus) kUdpClient_VWriteImpl(kUdpClient client, const void* buffer, kSize size)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 

    kCheckState(!kIsNull(obj->base.writeBuffer)); 
    kCheckState(size <= (obj->base.writeEnd - obj->base.writeBegin)); 

    kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size); 
    obj->base.writeBegin += size; 
    
    return kOK;
}

kFx(kStatus) kUdpClient_VFlush(kUdpClient client)
{
    return kOK; 
}

kFx(kStatus) kUdpClient_EnableBroadcast(kUdpClient client, kBool broadcast)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    
    return kSocket_EnableBroadcast(obj->socket, broadcast);
}

kFx(kStatus) kUdpClient_EnableBroadcastReceive(kUdpClient client, kBool broadcast)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    
    obj->isBroacastReceiver = broadcast; 

    return kOK;
}

kFx(kStatus) kUdpClient_EnableReuseAddress(kUdpClient client, kBool reuse)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
    
    return kSocket_EnableReuseAddress(obj->socket, reuse);
}

kFx(kStatus) kUdpClient_SetWriteBuffers(kUdpClient client, kSSize socketSize, kSSize clientSize)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
        
    kCheckState(obj->base.writeBegin == 0);
    
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

kFx(kStatus) kUdpClient_SetReadBuffers(kUdpClient client, kSSize socketSize, kSSize clientSize)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 

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

kFx(kSocket) kUdpClient_Socket(kUdpClient client)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
        
    return obj->socket;
}

kFx(kStatus) kUdpClient_LocalEndPoint(kUdpClient client, kIpEndPoint* endPoint)
{
    kUdpClientClass* obj = kUdpClient_Cast_(client); 
        
    return kSocket_LocalEndPoint(obj->socket,endPoint);  
}
