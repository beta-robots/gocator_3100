/** 
 * @file    kHttpServer.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServer.h>
#include <kApi/Data/kList.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kTcpServer.h>
#include <kApi/Threads/kThread.h>

kBeginClass(k, kHttpServer, kObject)
    kAddVMethod(kHttpServer, kObject, VRelease)
kEndClass()

kFx(kStatus) kHttpServer_Construct(kHttpServer* server, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kHttpServer); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, server)); 

    if (!kSuccess(status = kHttpServer_Init(*server, type, alloc)))
    {
        kAlloc_FreeRef(alloc, server); 
    }

    return status; 
} 

kFx(kStatus) kHttpServer_Init(kHttpServer server, kType type, kAlloc allocator)
{
    kHttpServerClass* obj = server; 
    
    kCheck(kObject_Init(server, type, allocator)); 

    kInitFields_(kHttpServer, server); 

    obj->localEndPoint.address = kIpAddress_AnyV4(); 
    obj->localEndPoint.port = kHTTP_SERVER_DEFAULT_PORT; 
    obj->channelCapacity = kHTTP_SERVER_DEFAULT_CHANNEL_CAPACITY; 

    return kOK;     
}

kFx(kStatus) kHttpServer_VRelease(kHttpServer server)
{
    kHttpServer_Stop(server); 

    kCheck(kObject_VRelease(server)); 

    return kOK;   
}

kFx(kStatus) kHttpServer_SetAddress(kHttpServer server, kIpAddress address)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->localEndPoint.address = address; 

    return kOK; 
}

kFx(kStatus) kHttpServer_SetPort(kHttpServer server, k32u port)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->localEndPoint.port = port; 

    return kOK; 
}

kFx(kStatus) kHttpServer_SetMaxConnections(kHttpServer server, kSize capacity)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->channelCapacity = capacity; 

    return kOK; 
}

kFx(kStatus) kHttpServer_SetHandler(kHttpServer server, kCallbackFx function, kPointer receiver)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->handler.function = function; 
    obj->handler.receiver = receiver; 

    return kOK; 
}

kFx(kStatus) kHttpServer_Start(kHttpServer server)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 
    kStatus status; 

    kCheckState(kIsNull(obj->thread)); 
    kCheckState(!kIsNull(obj->handler.function)); 

    obj->shouldQuit = kFALSE; 

    kTry
    {
        kTest(kList_Construct(&obj->channels, kTypeOf(kObject), obj->channelCapacity, kObject_Alloc_(server))); 

        //configure server socket
        kTest(kTcpServer_Construct(&obj->listener, obj->localEndPoint.address.version, kObject_Alloc_(server))); 

        kTest(kTcpServer_SetReadBuffers(obj->listener, kHTTP_SERVER_SOCKET_READ_BUFFER, kHTTP_SERVER_CLIENT_READ_BUFFER)); 
        kTest(kTcpServer_SetWriteBuffers(obj->listener, kHTTP_SERVER_SOCKET_WRITE_BUFFER, kHTTP_SERVER_CLIENT_WRITE_BUFFER)); 
        kTest(kTcpServer_EnableReuseAddress(obj->listener, kTRUE)); 

        kTest(kTcpServer_Listen(obj->listener, obj->localEndPoint.address, obj->localEndPoint.port, kHTTP_SERVER_LISTEN_BACKLOG));

        //start thread to accept incoming connections
        kTest(kThread_Construct(&obj->thread, kObject_Alloc_(server))); 
        kTest(kThread_Start(obj->thread, kHttpServer_ThreadEntry, server)); 
    }
    kCatch(&status)
    {
        kHttpServer_Stop(server); 
        kEndCatch(status); 
    }
        
    return kOK; 
}

kFx(kStatus) kHttpServer_Stop(kHttpServer server)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    if (!kIsNull(obj->thread))
    {
        obj->shouldQuit = kTRUE; 
        kCheck(kDestroyRef(&obj->thread)); 
    }

    kCheck(kDestroyRef(&obj->listener)); 
    kCheck(kDisposeRef(&obj->channels)); 

    return kOK; 
}

kFx(kStatus) kHttpServer_ThreadEntry(kHttpServer server)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    //run until the server is stopped
    while (!obj->shouldQuit)
    {
        kTcpClient client = kNULL; 
        kHttpServerChannel channel = kNULL; 

        //wait to accept a new connection; the wait time is limited to kHTTP_SERVER_QUIT_QUERY_PERIOD, so that the 
        //'shouldQuit' flag is periodically polled
        if (kSuccess(kTcpServer_Accept(obj->listener, kHTTP_SERVER_QUIT_QUERY_PERIOD, &client, kObject_Alloc_(server))))
        {
            if (!kIsNull(client))
            {
                //construct a new channel object to represent the connection, and add it to the list of active channels
                if (!kSuccess(kHttpServerChannel_Construct(&channel, server, client, kObject_Alloc_(server))))
                {
                    kObject_Destroy(client);                      
                }
                else
                {
                    kHttpServer_AddChannel(server, channel); 
                }
            }
        }
    }

    return kOK;
}

kFx(kStatus) kHttpServer_AddChannel(kHttpServer server, kHttpServerChannel channel)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 
    kStatus status; 

    //add new channel to channel list
    if (!kSuccess(status = kList_Add(obj->channels, &channel, kNULL)))
    {
        kObject_Destroy(channel); 
        return status; 
    }

    //remove any closed channels, or channels in excess of max connections
    kCheck(kHttpServer_CullChannels(server)); 

    return kOK; 
}

kFx(kStatus) kHttpServer_CullChannels(kHttpServer server)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 
    kListItem channelIt = 0; 

    //remove any inactive channels
    channelIt = kList_First(obj->channels); 
    while (!kIsNull(channelIt))
    {
        kListItem current = channelIt; 
        kHttpServerChannel channel = kList_As_(obj->channels, current, kObject); 

        channelIt = kList_Next(obj->channels, channelIt); 

        if (kHttpServerChannel_IsClosed(channel))
        {
            kCheck(kList_Remove(obj->channels, current)); 
            kCheck(kObject_Destroy(channel)); 
        }
    }

    //remove oldest remaining channels until less than capacity
    while (kList_Count(obj->channels) > obj->channelCapacity)
    {      
        kListItem current = kList_First(obj->channels); 
        kHttpServerChannel channel = kList_As_(obj->channels, current, kObject); 

        kCheck(kList_Remove(obj->channels, current)); 
        kCheck(kObject_Destroy(channel)); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServer_LocalEndPoint(kHttpServer server, kIpEndPoint* endPoint)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    kCheckState(!kIsNull(obj->listener)); 

    kCheck(kTcpServer_LocalEndPoint(obj->listener, endPoint)); 
    
    return kOK; 
}

kFx(kCallback) kHttpServer_Handler(kHttpServer server)
{
    kHttpServerClass* obj = kHttpServer_Cast_(server); 

    return obj->handler; 
}
