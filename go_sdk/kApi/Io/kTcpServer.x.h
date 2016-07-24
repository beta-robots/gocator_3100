/** 
 * @file    kTcpServer.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_SERVER_X_H
#define K_API_TCP_SERVER_X_H

kBeginHeader()

typedef struct kTcpServerClass
{
    kObjectClass base; 
    kSocket socket;                     //Socket object. 
    kSSize clientWriteBufferSize;       //Client write buffer size for accepted sockets.
    kSSize clientReadBufferSize;        //Client read buffer size for accepted sockets.
} kTcpServerClass;

kDeclareClass(k, kTcpServer, kObject)

kFx(kStatus) kTcpServer_Init(kTcpServer server, kType type, kIpVersion ipVersion, kAlloc allocator); 
kFx(kStatus) kTcpServer_VRelease(kTcpServer server);

#define kTcpServer_Cast_(TCP)                       (kCastClass_(kTcpServer, TCP))

kEndHeader()

#endif
