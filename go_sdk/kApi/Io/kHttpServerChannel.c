/** 
 * @file    kHttpServerChannel.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kHttpServer.h>
#include <kApi/Io/kHttpServerRequest.h>
#include <kApi/Io/kHttpServerResponse.h>
#include <kApi/Io/kTcpClient.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kThread.h>
#include <ctype.h>

kBeginClass(k, kHttpServerChannel, kObject)
    kAddVMethod(kHttpServerChannel, kObject, VRelease)
kEndClass()

kFx(kStatus) kHttpServerChannel_Construct(kHttpServerChannel* channel, kHttpServer server, kTcpClient client, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kHttpServerChannel); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, channel)); 

    if (!kSuccess(status = kHttpServerChannel_Init(*channel, type, server, client, alloc)))
    {
        kAlloc_FreeRef(alloc, channel); 
    }

    return status; 
} 

kFx(kStatus) kHttpServerChannel_Init(kHttpServerChannel channel, kType type, kHttpServer server, kTcpClient client, kAlloc allocator)
{
    kHttpServerChannelClass* obj = channel; 
    kStatus status; 

    kCheck(kObject_Init(channel, type, allocator)); 

    kInitFields_(kHttpServerChannel, channel); 

    obj->server = server; 
    obj->client = client; 
    
    kTry
    {
        kTest(kLock_Construct(&obj->clientLock, allocator)); 

        kTest(kHttpServerRequest_Construct(&obj->request, channel, allocator)); 
        kTest(kHttpServerResponse_Construct(&obj->response, channel, allocator)); 

        //start thread to process incoming http requests
        kTest(kThread_Construct(&obj->thread, allocator)); 
        kTest(kThread_Start(obj->thread, kHttpServerChannel_ThreadEntry, channel)); 
    }
    kCatch(&status)
    {
        kHttpServerChannel_VRelease(channel); 
        kEndCatch(status); 
    }

    return kOK;     
}

kFx(kStatus) kHttpServerChannel_VRelease(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    //abort any ongoing communication operations
    kCheck(kHttpServerChannel_CancelClient(channel)); 

    kCheck(kObject_Destroy(obj->thread)); 
    kCheck(kObject_Destroy(obj->client)); 
    kCheck(kObject_Destroy(obj->request)); 
    kCheck(kObject_Destroy(obj->response)); 
    kCheck(kObject_Destroy(obj->clientLock)); 

    kCheck(kObject_VRelease(channel)); 

    return kOK;   
}

kFx(kStatus) kHttpServerChannel_ThreadEntry(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 
    kStatus result; 

    //process messages until: 
    //a) the user-provided callback requests close ('connection: closed'), in which case obj->client will be null
    //b) the channel stream is detached by the user-provided callback, in which case obj->client will be null
    //c) the channel stream is closed unexpectedly by the client, in which case 'result' will most likely be kERROR_CLOSED
    //d) the channel stream is cancelled by the server (parent), in which case 'result' will most likely be kERROR_ABORT
    //e) an unexpected/unhandled error occurs  
    do
    {
        result = kHttpServerChannel_ProcessMessage(channel); 
    }
    while (kSuccess(result) && !kIsNull(obj->client)); 

    //ensure that the channel stream is closed, if it isn't already
    kCheck(kHttpServerChannel_DestroyClient(channel)); 

    return result;   
}

kFx(kStatus) kHttpServerChannel_ProcessMessage(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 
    kCallback handler = kHttpServer_Handler(obj->server); 
    kStatus status = kOK; 

    //process request line and headers
    kCheck(kHttpServerRequest_Begin(obj->request));    

    //if requested, automatically send a 100-continue message
    if (kHttpServerRequest_ExpectsContinue(obj->request))
    {
        kCheck(kHttpServerChannel_SendContinue(channel));    
    }

    //initialize response 
    kCheck(kHttpServerResponse_Begin(obj->response)); 

    //invoke user callback to perform message processing
    status = handler.function(handler.receiver, obj->server, channel);

    //if the stream wasn't detached by the user callback...
    if (!kIsNull(obj->client))
    {
        if (kSuccess(status))
        {
            //if processing completed normally, finalize the response
            kCheck(kHttpServerResponse_End(obj->response)); 
        }
        else if (!kHttpServerResponse_MessageStarted(obj->response) && kSuccess(kTcpClient_Status(obj->client)))
        {
            //if an unhandled processing error occurred, the stream is still valid, and response headers haven't 
            //already been transmitted, discard any partially-formatted response and send an 'internal server error' 
            //message instead
            kCheck(kHttpServerResponse_Begin(obj->response)); 
            kCheck(kHttpServerResponse_SetStatus(obj->response, kHTTP_STATUS_INTERNAL_SERVER_ERROR)); 
            kCheck(kHttpServerResponse_SetClosed(obj->response, kTRUE)); 
            kCheck(kHttpServerResponse_End(obj->response)); 
        }

        //if the response included a 'connection: closed' header, close the tcp connection here
        if (kHttpServerResponse_Closed(obj->response))
        {
            kCheck(kHttpServerChannel_DestroyClient(channel)); 
        }
    }

    return status; 
}

kFx(kStatus) kHttpServerChannel_SendContinue(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    kCheck(kHttpServerResponse_Begin(obj->response)); 
    kCheck(kHttpServerResponse_SetStatus(obj->response, kHTTP_STATUS_CONTINUE)); 
    kCheck(kHttpServerResponse_End(obj->response)); 

    return kOK; 
}

kFx(kHttpServerRequest) kHttpServerChannel_Request(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    return obj->request; 
}

kFx(kHttpServerResponse) kHttpServerChannel_Response(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    return obj->response; 
}

kFx(kStatus) kHttpServerChannel_CancelClient(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    //check client lock for null because this function can be called during init failure clean-up
    if (!kIsNull(obj->clientLock))
    {
        kLock_Enter(obj->clientLock); 
        {
            if (!kIsNull(obj->client))
            {
                kTcpClient_Cancel(obj->client); 
            }           
        }
        kLock_Exit(obj->clientLock); 
    }

    return kOK;   
}

kFx(kStatus) kHttpServerChannel_DestroyClient(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    kLock_Enter(obj->clientLock); 
    {
        kDestroyRef(&obj->client); 
    }
    kLock_Exit(obj->clientLock); 

    return kOK;   
}

kFx(kStatus) kHttpServerChannel_DetachClient(kHttpServerChannel channel, kTcpClient* client)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    //finalize the outgoing response before surrendering control of the client
    kCheck(kHttpServerResponse_End(obj->response)); 

    kLock_Enter(obj->clientLock); 
    {
        *client = obj->client; 
        obj->client = kNULL; 
    }
    kLock_Exit(obj->clientLock); 

    return kOK;   
}

kFx(kBool) kHttpServerChannel_IsClosed(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    return kSuccess(kThread_Join(obj->thread, 0, kNULL)); 
}

kFx(kTcpClient) kHttpServerChannel_Client(kHttpServerChannel channel)
{
    kHttpServerChannelClass* obj = kHttpServerChannel_Cast_(channel); 

    return obj->client; 
}

kFx(kStatus) kHttpServerChannel_NormalizeHeaderCaps(kChar* str)
{    
    if (*str != 0)
    {        
        *str = toupper(*str); 

        while (!kIsNull(str = strchr(str, '-')))
        {
            str++; 
                
            if (*str != 0)
            {        
                *str = toupper(*str); 
            }
        }
    }

    return kOK; 
}

