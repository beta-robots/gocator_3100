/** 
 * @file    GoReceiver.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_RECEIVER_X_H
#define GO_SDK_RECEIVER_X_H

#include <GoSdk/Internal/GoReceiver.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Io/kTcpClient.h>
#include <kApi/Io/kSerializer.h>
kBeginHeader()

#define GO_RECEIVER_CONNECT_TIMEOUT             (1000000)
#define GO_RECEIVER_QUIT_QUERY_INTERVAL         (100000)

typedef struct GoReceiverClass
{
    kObjectClass base; 

    volatile kBool quit; 
    kThread thread; 
    kTcpClient client; 
    kSerializer reader; 

    kCallbackFx onCancel; 
    kPointer onCancelContext; 

    GoReceiverMessageFx onMessage; 
    kPointer onMessageContext; 

    kSSize socketBufferSize; 
    kSSize clientBufferSize; 
} GoReceiverClass; 

kDeclareClass(Go, GoReceiver, kObject)

#define GoReceiver_Cast_(CONTEXT)    kCastClass_(GoReceiver, CONTEXT)

GoFx(kStatus) GoReceiver_Init(GoReceiver receiver, kType type, kAlloc alloc);
GoFx(kStatus) GoReceiver_VRelease(GoReceiver receiver);

GoFx(kStatus) GoReceiver_CancelHandler(GoReceiver receiver, kObject sender, kPointer args); 

GoFx(kStatus) GoReceiver_ThreadEntry(GoReceiver receiver); 

kEndHeader()

#endif
