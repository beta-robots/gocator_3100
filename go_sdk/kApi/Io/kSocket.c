/** 
 * @file    kSocket.c
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kSocket.h>
#include <stdio.h>

/*
 * kSocketType
 */
kBeginEnum(k, kSocketType, kValue)
    kAddEnumerator(kSocketType, kSOCKET_TYPE_TCP)
    kAddEnumerator(kSocketType, kSOCKET_TYPE_UDP)
kEndEnum()

/*
 * kSocketWait
 */
kBeginBitEnum(k, kSocketEvent, kValue)
    kAddEnumerator(kSocketEvent, kSOCKET_EVENT_READ)
    kAddEnumerator(kSocketEvent, kSOCKET_EVENT_WRITE)
    kAddEnumerator(kSocketEvent, kSOCKET_EVENT_EXCEPT)
kEndBitEnum()

/*
 * kSocket
 */

kBeginClass(k, kSocket, kObject)
    kAddVMethod(kSocket, kObject, VRelease)
kEndClass()

kFx(kStatus) kSocket_WaitAny(const kSocket* sockts, kSize count, k64u timeout)
{    
    struct timeval tv;
    fd_set readSet, writeSet, exceptSet;
    kSocketHandle highest = 0;
    int result;
    kSize i;    

    kCheckArgs(count <= FD_SETSIZE); 

    FD_ZERO(&readSet); 
    FD_ZERO(&writeSet); 
    FD_ZERO(&exceptSet); 

    for (i = 0; i < count; i++)
    {
        kSocketClass* obj = kSocket_Cast_(sockts[i]); 

        if (obj->handle > highest)
        {
            highest = obj->handle;
        }

        if (obj->eventTypes & kSOCKET_EVENT_READ)      FD_SET(obj->handle, &readSet); 
        if (obj->eventTypes & kSOCKET_EVENT_WRITE)     FD_SET(obj->handle, &writeSet); 
        if (obj->eventTypes & kSOCKET_EVENT_EXCEPT)    FD_SET(obj->handle, &exceptSet); 

        obj->eventStatus = 0; 
    }

    result = kSocket_Select((int) (highest) + 1, &readSet, &writeSet, &exceptSet, kSocket_FormatTimeVal(&tv, timeout));

    if (result == 0) 
    {
        return kERROR_TIMEOUT;
    }
    else if (result < 0)
    {
        return kERROR_STREAM;
    }

    for (i = 0; i < count; i++)
    {
        kSocketClass* obj = sockts[i];

        if ((obj->eventTypes & kSOCKET_EVENT_READ)  && FD_ISSET(obj->handle, &readSet))      obj->eventStatus |= kSOCKET_EVENT_READ; 
        if ((obj->eventTypes & kSOCKET_EVENT_WRITE) && FD_ISSET(obj->handle, &writeSet))     obj->eventStatus |= kSOCKET_EVENT_WRITE;  
        if ((obj->eventTypes & kSOCKET_EVENT_EXCEPT) && FD_ISSET(obj->handle, &exceptSet))   obj->eventStatus |= kSOCKET_EVENT_EXCEPT; 
    }

    return kOK;
}

kFx(struct timeval*) kSocket_FormatTimeVal(struct timeval* tv, k64u timeout)
{
    if (timeout == kINFINITE)
    {
        return kNULL; 
    }
  
    tv->tv_sec = (long) timeout / 1000000;
    tv->tv_usec = (int) (timeout % 1000000);
   
    return tv;  
}

kFx(kStatus) kSocket_Construct(kSocket* sockt, kIpVersion ipVersion, kSocketType socketType, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kSocket); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, sockt)); 

    if (!kSuccess(status = kSocket_Init(*sockt, type, ipVersion, socketType, alloc)))
    {
        kAlloc_FreeRef(alloc, sockt); 
    }

    return status; 
} 

kFx(kStatus) kSocket_ConstructFromHandle(kSocket* sockt, kIpVersion ipVersion, kSocketType socktType, kSocketHandle handle, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kSocket); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, sockt)); 

    if (!kSuccess(status = kSocket_InitFromHandle(*sockt, type, ipVersion, socktType, handle, alloc)))
    {
        kAlloc_FreeRef(alloc, sockt); 
    }

    return status; 
} 

kFx(kStatus) kSocket_Init(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socketType, kAlloc allocator)
{
    if (ipVersion != kIP_VERSION_4)
    {
        return kERROR_UNIMPLEMENTED; 
    }
    else
    {
        k32s sockFamily = AF_INET; 
        k32s sockType = (socketType == kSOCKET_TYPE_TCP) ? SOCK_STREAM : SOCK_DGRAM; 
        kSocketHandle handle = socket(sockFamily, sockType, 0);
        kStatus status = kOK;

        if (handle == kSOCKET_INVALID_SOCKET)
        {
            status = kERROR_NETWORK; 
        }
        else if (!kSuccess(status = kSocket_InitFromHandle(sockt, type, ipVersion, socketType, handle, allocator)))
        {
            kSocket_CloseHandle(handle); 
        }

        return status; 
    }
}

kFx(kStatus) kSocket_InitFromHandle(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socktType, kSocketHandle handle, kAlloc allocator)
{
    kSocketClass* obj = sockt;  

    kCheckArgs(handle != kSOCKET_INVALID_SOCKET); 
    
    kCheck(kObject_Init(sockt, type, allocator)); 

    obj->ipVersion = ipVersion; 
    obj->socketType = socktType; 
    obj->handle = handle; 
    obj->eventTypes = kSOCKET_EVENT_READ; 
    obj->eventStatus = 0; 
    obj->status = kOK;
    obj->isBlocking = kTRUE; 

    return kOK; 
}

kFx(kStatus) kSocket_VRelease(kSocket sockt)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    
    if (obj->handle != kSOCKET_INVALID_SOCKET)
    {
        if (kSuccess(obj->status))
        {
            kSocket_ShutdownHandle(obj->handle); 
        }

        kCheck(kSocket_CloseHandle(obj->handle)); 
    }

    kCheck(kObject_VRelease(sockt)); 

    return kOK;
}

kFx(kStatus) kSocket_Bind(kSocket sockt, kIpAddress address, k32u port)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct sockaddr_in sockAddr;

    kCheck(obj->status);
    kCheckArgs(address.version == kIP_VERSION_4); 
    
    kCheck(kIpAddress_ToSockAddr(address, port, &sockAddr));           
    kCheck(bind(obj->handle, (struct sockaddr*)&sockAddr, sizeof(sockAddr)) == 0); 

    return kOK; 
}

kFx(kStatus) kSocket_Connect(kSocket sockt, kIpAddress address, k32u port, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kBool wasBlocking = obj->isBlocking; 
    struct sockaddr_in sockAddr;
    fd_set exceptSet; 
    fd_set writeSet; 
    int selectResult = 0; 
    struct timeval tv; 
    kStatus exception;

    kCheck(obj->status);
    kCheckArgs(address.version  == kIP_VERSION_4); 

    kTry
    {     
        kTest(kSocket_SetBlocking(sockt, kFALSE));
        kTest(kIpAddress_ToSockAddr(address, port, &sockAddr));
    
        if (connect(obj->handle, (struct sockaddr*)&sockAddr, sizeof(sockAddr)) == kSOCKET_ERROR)
        {   
            if (kSocket_GetLastError() == kSOCKET_WOULD_BLOCK || kSocket_GetLastError() == kSOCKET_IN_PROGRESS)
            {                
                FD_ZERO(&writeSet); 
                FD_ZERO(&exceptSet); 

                FD_SET(obj->handle, &writeSet); 
                FD_SET(obj->handle, &exceptSet); 

                selectResult = kSocket_Select((int)(obj->handle)+1, NULL, &writeSet, &exceptSet, kSocket_FormatTimeVal(&tv, timeout)); 

                if ((selectResult == 1) && !FD_ISSET(obj->handle, &exceptSet))
                {
                    int errorCode; 
                    socklen_t errorSize = sizeof(int); 

                    kTest(getsockopt(obj->handle, SOL_SOCKET, SO_ERROR, (void*)&errorCode, &errorSize) == 0); 

                    if (errorCode != 0)
                    {
                        kThrow(kERROR_NETWORK); 
                    }
                }
                else if (selectResult == 0)
                {
                    kThrow(kERROR_TIMEOUT); 
                }
                else
                {
                    kThrow(kERROR_NETWORK); 
                }               
            }
            else
            {
                kThrow(kERROR_NETWORK);
            }
        }   
    
        if (wasBlocking)
        {
            kTest(kSocket_SetBlocking(sockt, wasBlocking));
        }
    }
    kCatch(&exception)
    {
        obj->status = exception;
        kEndCatch(exception); 
    }

    return kOK;
}

kFx(kStatus) kSocket_Accept(kSocket sockt, kSocket* connection, kAlloc allocator)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kSocketHandle handle = kSOCKET_INVALID_SOCKET; 
    kStatus status = kOK; 

    kCheck(obj->status);
        
    if ((handle = accept(obj->handle, 0, 0)) == kSOCKET_INVALID_SOCKET)
    {
        *connection = kNULL; 
    }
    else if (!kSuccess(status = kSocket_ConstructFromHandle(connection, obj->ipVersion, kSOCKET_TYPE_TCP, handle, allocator)))
    {
        kSocket_CloseHandle(handle); 
    }

    return status; 
}

kFx(kStatus) kSocket_Listen(kSocket sockt, kSize backlog)
{ 
    kSocketClass* obj = kSocket_Cast_(sockt); 

    kCheck(obj->status);

    kCheck(listen(obj->handle, (int) backlog) != kSOCKET_ERROR); 

    return kOK;
}

kFx(kStatus) kSocket_SetEvents(kSocket sockt, kSocketEvent events)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    obj->eventTypes = events;       

    return kOK; 
}

kFx(kSocketEvent) kSocket_Events(kSocket sockt)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return obj->eventStatus; 
}

kFx(kStatus) kSocket_Wait(kSocket sockt, k64u timeout)
{      
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv;
    fd_set readSet, writeSet, exceptSet;  
    kStatus result; 

    kCheck(obj->status);

    FD_ZERO(&readSet);
    FD_ZERO(&writeSet);
    FD_ZERO(&exceptSet);

    if (obj->eventTypes & kSOCKET_EVENT_READ)      FD_SET(obj->handle, &readSet); 
    if (obj->eventTypes & kSOCKET_EVENT_WRITE)     FD_SET(obj->handle, &writeSet); 
    if (obj->eventTypes & kSOCKET_EVENT_EXCEPT)     FD_SET(obj->handle, &exceptSet); 

    obj->eventStatus = 0; 

    result = kSocket_Select((int)(obj->handle)+1, &readSet, &writeSet, &exceptSet, kSocket_FormatTimeVal(&tv, timeout)); 

    if (result == 0) 
    {
        return kERROR_TIMEOUT;
    }
    else if (result < 0)
    {
        obj->status = kERROR_STREAM; 
        return obj->status;
    }

    if ((obj->eventTypes & kSOCKET_EVENT_READ)   && FD_ISSET(obj->handle, &readSet))    obj->eventStatus |= kSOCKET_EVENT_READ; 
    if ((obj->eventTypes & kSOCKET_EVENT_WRITE)  && FD_ISSET(obj->handle, &writeSet))   obj->eventStatus |= kSOCKET_EVENT_WRITE;  
    if ((obj->eventTypes & kSOCKET_EVENT_EXCEPT) && FD_ISSET(obj->handle, &exceptSet))  obj->eventStatus |= kSOCKET_EVENT_EXCEPT;  

    return kOK; 
}

kFx(kStatus) kSocket_Read(kSocket sockt, void* buffer, kSize size, kSize* read)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);

    if (size == 0)
    {
        return kOK; 
    }

    result = kSocket_Recv(sockt, buffer, size);
    
    if (result > 0)
    {
        if (!kIsNull(read))
        {
            *read = (kSize) result; 
        }
    }
    else if (result == 0)
    {
        opStatus = kERROR_CLOSED; 
        obj->status = kERROR_CLOSED; 
    }
    else
    {
        socketError = kSocket_GetLastError(); 
        
        if (socketError == kSOCKET_TIMEDOUT)
        {
            opStatus = kERROR_TIMEOUT; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_TIMEOUT; 
            }
        }
        else if (socketError == kSOCKET_WOULD_BLOCK)
        {
            opStatus = kERROR_BUSY; 
        }
        else
        {
            opStatus = kERROR_STREAM; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_STREAM; 
            }
        }
    }

    return opStatus;
}

kFx(kStatus) kSocket_ReadFrom(kSocket sockt, kIpEndPoint *endPoint, void* buffer, kSize size, kSize* read)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct sockaddr_in sockAddr;
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);

    result = kSocket_RecvFrom(sockt, buffer, size, &sockAddr);
    
    if (result > 0)
    {
        if (!kIsNull(endPoint))
        {
            kCheck(kIpAddress_FromSockAddr(&sockAddr, &endPoint->address, &endPoint->port));
        }
        if (!kIsNull(read))
        {
            *read = (kSize) result; 
        }
    }
    else if (result == 0)
    {
        opStatus = kERROR_CLOSED; 
        obj->status = kERROR_CLOSED; 
    }
    else
    {
        socketError = kSocket_GetLastError(); 
        
        if (socketError == kSOCKET_TIMEDOUT)
        {
            opStatus = kERROR_TIMEOUT; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_TIMEOUT; 
            }
        }
        else if (socketError == kSOCKET_WOULD_BLOCK)
        {
            opStatus = kERROR_BUSY; 
        }
        else if (socketError == kSOCKET_MSGSIZE)
        {
            opStatus = kERROR_INCOMPLETE; 
        }
        else
        {
            opStatus = kERROR_STREAM; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_STREAM; 
            }
        }
    }
    
    return opStatus;
}

kFx(kStatus) kSocket_Write(kSocket sockt, const void* buffer, kSize size, kSize* written)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);

    *written = 0; 

    do 
    {        
        result = kSocket_Send(sockt, (const kByte*)buffer + *written, size - *written);

        if (result >= 0)
        {
            *written += (kSize) result; 
        }
        else
        {
            socketError = kSocket_GetLastError(); 

            if (socketError == kSOCKET_TIMEDOUT)
            {
                opStatus = kERROR_TIMEOUT; 

                if (obj->socketType == kSOCKET_TYPE_TCP)
                {
                    obj->status = kERROR_TIMEOUT; 
                }
            }
            else if(socketError == kSOCKET_WOULD_BLOCK)
            {
                opStatus = kERROR_BUSY; 
            }
            else
            {
                opStatus = kERROR_STREAM; 

                if (obj->socketType == kSOCKET_TYPE_TCP)
                {
                    obj->status = kERROR_STREAM; 
                }
            }
        }
    }
    while (obj->isBlocking && !kIsError(opStatus) && (*written < size)); 

    return opStatus;
}

kFx(kStatus) kSocket_WriteTo(kSocket sockt, kIpAddress address, k32u port, const void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct sockaddr_in sockAddr;
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);
    kCheckArgs(address.version == kIP_VERSION_4); 

    kCheck(kIpAddress_ToSockAddr(address, port, &sockAddr));

    result = kSocket_SendTo(sockt, buffer, size, &sockAddr);

    if (result < 0)
    {
        socketError = kSocket_GetLastError(); 

        if (socketError == kSOCKET_TIMEDOUT)
        {
            opStatus = kERROR_TIMEOUT; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_TIMEOUT; 
            }
        }
        else if(socketError == kSOCKET_WOULD_BLOCK)
        {
            opStatus = kERROR_BUSY; 
        }
        else if (socketError == kSOCKET_MSGSIZE)
        {
            opStatus = kERROR_INCOMPLETE; 
        }
        else
        {
            opStatus = kERROR_STREAM; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_STREAM; 
            }
        }
    }
    else if ((kSize)result != size)
    {
        opStatus = kERROR_INCOMPLETE; 
    }   

    return opStatus; 
}

kFx(kStatus) kSocket_EnableBroadcast(kSocket sockt, kBool broadcast)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kBool value = (broadcast) ? kTRUE : kFALSE;
    
    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_BROADCAST, (char*)&value, sizeof(kBool)) != kSOCKET_ERROR); 

    return kOK;   
}

kFx(kStatus) kSocket_EnableReuseAddress(kSocket sockt, kBool reuse)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kBool value = reuse; 
    
    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(kBool)) != kSOCKET_ERROR);

    return kOK;   
}

kFx(kStatus) kSocket_SetNoDelay(kSocket sockt, kBool noDelay)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kBool value = noDelay; 
    
    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, IPPROTO_TCP, TCP_NODELAY, (char*)&value, sizeof(kBool)) != kSOCKET_ERROR); 

    return kOK;   
}

kFx(kStatus) kSocket_SetLingerTime(kSocket sockt, k64u lingerTime)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct linger linger;

    kCheck(obj->status);

    linger.l_onoff = 1;
    linger.l_linger = (unsigned short) ((lingerTime + 999999)/1000000);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_LINGER, (char*) &linger, sizeof(struct linger)) != kSOCKET_ERROR); 
   
    return kOK;   
}

kFx(kStatus) kSocket_LocalEndPoint(kSocket sockt, kIpEndPoint* endPoint)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct sockaddr_in sockAddr;
    socklen_t sockAddrLength = (socklen_t) sizeof(sockAddr);
    
    kCheck(obj->status);

    kCheck(getsockname(obj->handle, (struct sockaddr*)&sockAddr, &sockAddrLength) != kSOCKET_ERROR);    
    kCheck(kIpAddress_FromSockAddr(&sockAddr, &endPoint->address, &endPoint->port));
 
    return kOK;
}

kFx(kStatus) kSocket_RemoteEndPoint(kSocket sockt, kIpEndPoint* endPoint)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct sockaddr_in sockAddr;
    socklen_t sockAddrLength = (socklen_t) sizeof(sockAddr);
     
    kCheck(obj->status);

    kCheck(getpeername(obj->handle, (struct sockaddr*)&sockAddr, &sockAddrLength) == 0); 
    kCheck(kIpAddress_FromSockAddr(&sockAddr,&endPoint->address, &endPoint->port));
    
    return kOK;
}

kFx(kStatus) kSocket_Status(kSocket sockt)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return obj->status; 
}

#if defined(K_WINDOWS)

kFx(kSSize) kSocket_Recv(kSocket sockt, void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return recv(obj->handle, buffer, (int) kMin_(size, kSOCKET_MAX_IO_SIZE), 0); 
}

kFx(kSSize) kSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, (int) kMin_(size, kSOCKET_MAX_IO_SIZE), 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) kSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return send(obj->handle, buffer, (int) kMin_(size, kSOCKET_MAX_IO_SIZE), 0); 
}

kFx(kSSize) kSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return sendto(obj->handle, buffer, (int) kMin_(size, kSOCKET_MAX_IO_SIZE), 0, (const struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) kSocket_GetLastError()
{
    return WSAGetLastError();
}

kFx(kStatus) kSocket_ShutdownHandle(kSocketHandle socktHandle)
{
     shutdown(socktHandle, SD_BOTH); 

     return kOK; 
}

kFx(kStatus) kSocket_CloseHandle(kSocketHandle socktHandle)
{    
    if (closesocket(socktHandle) != 0)
    {
        return kERROR_NETWORK; 
    }

    return kOK;
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    unsigned long nonBlocking = !isBlocking;
       
    kCheck(ioctlsocket(obj->handle, FIONBIO, &nonBlocking) == 0);

    obj->isBlocking = isBlocking; 
    
    return kOK;
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) (timeout + 999)/1000;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 

    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) (timeout + 999)/1000;
    
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 

    return kOK;  
}

kFx(kStatus) kSocket_BindToDevice(kSocket socket, const kChar* interfaceName)
{
    //not supported; has no effect
    return kOK; 
}

#elif defined (K_DSP_BIOS)

kFx(kSSize) kSocket_Recv(kSocket sockt, void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    return recv(obj->handle, buffer, size, 0); 
}

kFx(kSSize) kSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    socklen_t sockAddrLength = sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, size, 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) kSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return send(obj->handle, (void*)buffer, size, 0); 
}

kFx(kSSize) kSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    socklen_t sockAddrLength = (socklen_t)sizeof(struct sockaddr_in);

    return sendto(obj->handle, (void*)buffer, size, 0, (struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) kSocket_GetLastError()
{
    return fdError();
}

kFx(kStatus) kSocket_ShutdownHandle(kSocketHandle socktHandle)
{    
    struct linger linger;

    linger.l_onoff = 1;
    linger.l_linger = 0;

    //DSP/BIOS doesn't seem to implement shutdown properly, so we set the linger time to zero to force connection reset
    setsockopt(socktHandle, SOL_SOCKET, SO_LINGER, (char*) &linger, sizeof(struct linger)); 
  
    shutdown(socktHandle, SHUT_RDWR); 

    return kOK; 
}

kFx(kStatus) kSocket_CloseHandle(kSocketHandle socktHandle)
{
    fdClose(socktHandle);

    return kOK; 
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int blockingOpt = (int) isBlocking; 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_BLOCKING, &blockingOpt, sizeof(blockingOpt)) != kSOCKET_ERROR);

    obj->isBlocking = isBlocking;
    
    return kOK;     
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    //using this option with a UDP socket generates "00000.553 mmFree: Double Free" condition in NDK stack (1.93)
    if (obj->socketType != kSOCKET_TYPE_UDP)
    {
        kCheckErr(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR, OS); 
    }
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    //using this option with a UDP socket generates "00000.553 mmFree: Double Free" condition in NDK stack (1.93)
    if (obj->socketType != kSOCKET_TYPE_UDP)
    {
        kCheckErr(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR, OS); 
    }
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv; 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, kSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != kSOCKET_ERROR);
    
    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, kSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != kSOCKET_ERROR);
    
    return kOK;  
}

kFx(kStatus) kSocket_BindToDevice(kSocket socket, const kChar* interfaceName)
{
    //not supported; has no effect
    return kOK; 
}

#elif defined (K_VX_KERNEL)

kFx(kSSize) kSocket_Recv(kSocket sockt, void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    return recv(obj->handle, buffer, size, 0); 
}

kFx(kSSize) kSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int sockAddrLength = (int) sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, size, 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) kSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return send(obj->handle, buffer, size, 0); 
}

kFx(kSSize) kSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return sendto(obj->handle, buffer, size, 0, (const struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) kSocket_GetLastError()
{
    return errno;
}

kFx(kStatus) kSocket_ShutdownHandle(kSocketHandle socktHandle)
{
     shutdown(socktHandle, SHUT_RDWR); 

     return kOK; 
}

kFx(kStatus) kSocket_CloseHandle(kSocketHandle socktHandle)
{
    close(socktHandle);

    return kOK; 
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int nonBlocking = !isBlocking; 
        
    if (ioctl(obj->handle, FIONBIO, (int)&nonBlocking) == ERROR)
    {
        return kERROR_OS; 
    }
 
    obj->isBlocking = isBlocking;

    return kOK;
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    /*
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv;
    int result = setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, kSocket_FormatTimeVal(&tv, timeout), sizeof(tv)); 

    kCheck(result != kSOCKET_ERROR);
    */
    
   /* 
    * TODO: This option doesn't appear to be implemented in VxWorks 6.9; code above compiles, but always return an error.
    * For now, we'll just return kOK; should file a bug report with Wind River, or just remove kSocket timeout feature if not needed.    
    */
    
    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, kSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != kSOCKET_ERROR);
    
    return kOK;  
}

#elif defined (K_POSIX)

kFx(kSSize) kSocket_Recv(kSocket sockt, void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    return recv(obj->handle, buffer, size, 0); 
}

kFx(kSSize) kSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    unsigned int sockAddrLength = sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, size, 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) kSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 

    return send(obj->handle, buffer, size, 0); 
}

kFx(kSSize) kSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return sendto(obj->handle, buffer, size, 0, (const struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) kSocket_GetLastError()
{
    return errno;
}

kFx(kStatus) kSocket_ShutdownHandle(kSocketHandle socktHandle)
{
     shutdown(socktHandle, SHUT_RDWR); 

     return kOK; 
}

kFx(kStatus) kSocket_CloseHandle(kSocketHandle socktHandle)
{
    close(socktHandle);

    return kOK; 
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int flags = fcntl(obj->handle, F_GETFL);

    if (isBlocking)
    {
        fcntl(obj->handle, F_SETFL, flags &~O_NONBLOCK);
    }
    else
    {
        fcntl(obj->handle, F_SETFL, O_NONBLOCK);
    }

    obj->isBlocking = isBlocking;

    return kOK;
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    int value = (int) kMin_(size, kSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != kSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv; 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, kSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != kSOCKET_ERROR);
    
    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct timeval tv;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, kSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != kSOCKET_ERROR);
    
    return kOK;  
}

#if !defined(K_DARWIN)

kFx(kStatus) kSocket_BindToDevice(kSocket sockt, const kChar* interfaceName)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    struct ifreq sender;

    kCheckArgs(interfaceName[0] != 0); 

    kMemSet(&sender, 0, sizeof(sender));       
    kStrCopy(sender.ifr_name, kCountOf(sender.ifr_name), interfaceName); 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_BINDTODEVICE, &sender, sizeof(sender)) != kSOCKET_ERROR);

    return kOK;
}
#endif

#endif

//K_COMPAT_5
kFx(kStatus) kSocket_WaitAny5(const kSocket* sockets, k32u count, kBool* status, k64u timeout)
{
    kSize i; 

    for (i = 0; i < count; ++i)
    {
        status[i] = kFALSE; 
        kCheck(kSocket_SetEvents(sockets[i], kSOCKET_EVENT_READ)); 
    }

    kCheck(kSocket_WaitAny(sockets, count, timeout)); 

    for (i = 0; i < count; ++i)
    {
        status[i] = (kSocket_Events(sockets[i]) & kSOCKET_EVENT_READ) != 0; 
    }

    return kOK; 
}

//K_COMPAT_5
kFx(kStatus) kSocket_Write5(kSocket sockt, const void* buffer, kSize size)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
    kSize written; 

    //the original FS socket only supported blocking read/write
    kAssert(obj->isBlocking); 
    kCheckState(obj->isBlocking); 

    return kSocket_Write(sockt, buffer, size, &written); 
}

//K_COMPAT_5
kFx(kStatus) kSocket_ClearErrors(kSocket sockt)
{
    kSocketClass* obj = kSocket_Cast_(sockt); 
   
    obj->status = kOK; 

    return kOK; 
}
