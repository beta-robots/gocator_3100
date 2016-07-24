/** 
 * @file    kHttpServerChannel.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_CHANNEL_X_H
#define K_API_HTTP_SERVER_CHANNEL_X_H

kBeginHeader()

typedef struct kHttpServerChannelClass
{
    kObjectClass base; 
    kHttpServer server;                 //Parent. 
    kTcpClient client;                  //TCP connection. 
    kLock clientLock;                   //Provides atomic cancel/destroy/attach operations. 
    kThread thread;                     //Processing thread.
    kHttpServerRequest request;         //Request processor.
    kHttpServerResponse response;       //Response processor. 
} kHttpServerChannelClass;

kDeclareClass(k, kHttpServerChannel, kObject)

kFx(kStatus) kHttpServerChannel_Construct(kHttpServerChannel* channel, kHttpServer server, kTcpClient client, kAlloc allocator); 

kFx(kStatus) kHttpServerChannel_Init(kHttpServerChannel channel, kType type, kHttpServer server, kTcpClient client, kAlloc allocator); 
kFx(kStatus) kHttpServerChannel_VRelease(kHttpServerChannel channel);

kFx(kStatus) kHttpServerChannel_ThreadEntry(kHttpServerChannel channel); 
kFx(kStatus) kHttpServerChannel_ProcessMessage(kHttpServerChannel channel); 
kFx(kStatus) kHttpServerChannel_SendContinue(kHttpServerChannel channel); 

kFx(kStatus) kHttpServerChannel_DestroyClient(kHttpServerChannel channel); 
kFx(kStatus) kHttpServerChannel_CancelClient(kHttpServerChannel channel); 

kFx(kBool) kHttpServerChannel_IsClosed(kHttpServerChannel channel); 
kFx(kTcpClient) kHttpServerChannel_Client(kHttpServerChannel channel); 

kFx(kStatus) kHttpServerChannel_NormalizeHeaderCaps(kChar* str); 

#define kHttpServerChannel_Cast_(C)         (kCastClass_(kHttpServerChannel, C))

kEndHeader()

#endif
