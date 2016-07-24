/** 
 * @file    GoReceiver.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoReceiver.h>
#include <GoSdk/Internal/GoSerializer.h>
#include <kApi/Utils/kUtils.h>

kBeginClass(Go, GoReceiver, kObject)
    kAddVMethod(GoReceiver, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoReceiver_Construct(GoReceiver* receiver, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoReceiver), receiver)); 

    if (!kSuccess(status = GoReceiver_Init(*receiver, kTypeOf(GoReceiver), alloc)))
    {
        kAlloc_FreeRef(alloc, receiver); 
    }

    return status; 
} 

GoFx(kStatus) GoReceiver_Init(GoReceiver receiver, kType type, kAlloc alloc)
{
    GoReceiverClass* obj = receiver;

    kCheck(kObject_Init(receiver, type, alloc)); 
    kInitFields_(GoReceiver, receiver);

    obj->socketBufferSize = -1; 
    obj->clientBufferSize = -1; 

    return kOK; 
}

GoFx(kStatus) GoReceiver_VRelease(GoReceiver receiver)
{
    kCheck(GoReceiver_Close(receiver)); 

    kCheck(kObject_VRelease(receiver)); 

    return kOK; 
}

GoFx(kStatus) GoReceiver_SetBuffers(GoReceiver receiver, kSSize socketSize, kSSize clientSize)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 

    kCheckState(!GoReceiver_IsOpen(receiver)); 

    if (socketSize > 0) obj->socketBufferSize = socketSize; 
    if (clientSize > 0) obj->clientBufferSize = clientSize; 

    return kOK; 
}

GoFx(kStatus) GoReceiver_SetCancelHandler(GoReceiver receiver, kCallbackFx function, kPointer context)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 

    kCheckState(!GoReceiver_IsOpen(receiver)); 

    obj->onCancel = function; 
    obj->onCancelContext = context; 

    return kOK; 
}

GoFx(kStatus) GoReceiver_SetMessageHandler(GoReceiver receiver, GoReceiverMessageFx function, kPointer context)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 

    kCheckState(!GoReceiver_IsOpen(receiver)); 

    obj->onMessage = function; 
    obj->onMessageContext = context; 

    return kOK; 
}

GoFx(kStatus) GoReceiver_CancelHandler(GoReceiver receiver, kObject sender, kPointer args)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 

    if (obj->quit)
    {
        return kERROR_ABORT; 
    }
    else if (obj->onCancel)
    {
        return obj->onCancel(obj->onCancelContext, receiver, kNULL); 
    }

    return kOK; 
}

GoFx(kStatus) GoReceiver_Open(GoReceiver receiver, kIpAddress address, k32u port)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 
    kStatus status; 

    kCheck(GoReceiver_Close(receiver)); 

    kCheckState(!kIsNull(obj->onMessage)); 

    obj->quit = kFALSE; 

    kTry
    {
        kTest(kTcpClient_Construct(&obj->client, kIP_VERSION_4, kObject_Alloc_(receiver))); 
        kTest(kTcpClient_SetCancelHandler(obj->client, GoReceiver_CancelHandler, receiver)); 
        kTest(kTcpClient_SetReadBuffers(obj->client, obj->socketBufferSize, obj->clientBufferSize)); 
        kTest(kTcpClient_Connect(obj->client, address, port, GO_RECEIVER_CONNECT_TIMEOUT)); 

        kTest(kSerializer_Construct(&obj->reader, obj->client, kTypeOf(GoSerializer), kObject_Alloc_(receiver))); 

        kTest(kThread_Construct(&obj->thread, kObject_Alloc_(receiver))); 
        kTest(kThread_Start(obj->thread, GoReceiver_ThreadEntry, receiver)); 
    }
    kCatch(&status)
    {
        GoReceiver_Close(receiver); 

        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoReceiver_Close(GoReceiver receiver)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 

    obj->quit = kTRUE; 
    
    kCheck(kDestroyRef(&obj->thread)); 
    kCheck(kDestroyRef(&obj->reader)); 
    kCheck(kDestroyRef(&obj->client)); 
         
    return kOK; 
}

GoFx(kStatus) GoReceiver_ThreadEntry(GoReceiver receiver)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 
    kStatus status; 

    while (!obj->quit)
    {
        if (kSuccess(status = kTcpClient_Wait(obj->client, GO_RECEIVER_QUIT_QUERY_INTERVAL)))
        {
            kCheck(obj->onMessage(obj->onMessageContext, receiver, obj->reader));    
        }      
        else if (status != kERROR_TIMEOUT)
        {
            kCheck(status); 
        }
    }
    
    return kOK; 
}

GoFx(kBool) GoReceiver_IsOpen(GoReceiver receiver)
{
    GoReceiverClass* obj = GoReceiver_Cast_(receiver); 
    return !kIsNull(obj->client); 
}
