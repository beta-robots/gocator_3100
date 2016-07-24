/** 
 * @file    kHttpServer.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_X_H
#define K_API_HTTP_SERVER_X_H

kBeginHeader()

#define kHTTP_SERVER_DEFAULT_CHANNEL_CAPACITY           (10)            //Default maximum simultaneous connections.
#define kHTTP_SERVER_LISTEN_BACKLOG                     (16)            //Backlog parameter used with tcp listen function.
#define kHTTP_SERVER_DEFAULT_PORT                       (80)            //Default server port.
#define kHTTP_SERVER_QUIT_QUERY_PERIOD                  (50000)         //Interval to check for quit flag. 

#define kHTTP_SERVER_SOCKET_READ_BUFFER                 (4096)          //Connection socket read buffer size. 
#define kHTTP_SERVER_CLIENT_READ_BUFFER                 (4096)          //Connection stream read buffer size.
#define kHTTP_SERVER_SOCKET_WRITE_BUFFER                (4096)          //Connection socket write buffer size. 
#define kHTTP_SERVER_CLIENT_WRITE_BUFFER                (4096)          //Connection stream write buffer size.

typedef struct kHttpServerClass
{
    kObjectClass base; 
    kIpEndPoint localEndPoint;          //Local end-point information.
    kSize channelCapacity;              //Maximum simultaneous connections. 
    kCallback handler;                  //Request callback. 
    kTcpServer listener;                //TCP server socket. 
    kThread thread;                     //Thread to accept connections.
    volatile kBool shouldQuit;          //Thread quit flag. 
    kList channels;                     //Remote connections -- kList<kHttpServerChannel>.
} kHttpServerClass;

kDeclareClass(k, kHttpServer, kObject)

kFx(kStatus) kHttpServer_Init(kHttpServer server, kType type, kAlloc allocator); 
kFx(kStatus) kHttpServer_VRelease(kHttpServer server);

kFx(kStatus) kHttpServer_ThreadEntry(kHttpServer server); 

kFx(kStatus) kHttpServer_AddChannel(kHttpServer server, kHttpServerChannel channel); 
kFx(kStatus) kHttpServer_CullChannels(kHttpServer server); 

kFx(kCallback) kHttpServer_Handler(kHttpServer server); 

#define kHttpServer_Cast_(SV)       (kCastClass_(kHttpServer, SV))

kEndHeader()

#endif
