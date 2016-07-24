/** 
 * @file    kTcpClient.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_CLIENT_X_H
#define K_API_TCP_CLIENT_X_H

#include <kApi/Io/kStream.h>

kBeginHeader()

#define kTCP_CLIENT_CANCEL_QUERY_INTERVAL       (100000)

typedef struct kTcpClientClass
{
    kStreamClass base; 
    kSocket socket;                     //Socket object.        
    k64u writeTimeout;                  //Timeout for write operations (microseconds).
    k64u readTimeout;                   //Timeout for read operations (microseconds).
    kCallback cancelQuery;              //User-provided cancellation handler. 
    kAtomic32s isCancelled;             //Has I/O been cancelled by client owner?
    kBool timedOut;                     //Has client experienced a timeout error?
} kTcpClientClass;

kDeclareClass(k, kTcpClient, kStream)

kFx(kStatus) kTcpClient_ConstructFromSocket(kTcpClient* client, kSocket socket, kAlloc allocator); 
kFx(kStatus) kTcpClient_Init(kTcpClient client, kType type, kIpVersion ipVersion, kAlloc allocator); 
kFx(kStatus) kTcpClient_InitFromSocket(kTcpClient client, kType type, kSocket socket, kAlloc allocator); 
kFx(kStatus) kTcpClient_VRelease(kTcpClient client);

kFx(kStatus) kTcpClient_VReadSomeImpl(kTcpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kTcpClient_VWriteImpl(kTcpClient client, const void* buffer, kSize size);
kFx(kStatus) kTcpClient_VSeek(kTcpClient client);
kFx(kStatus) kTcpClient_VFlush(kTcpClient client);

kFx(kStatus) kTcpClient_ReadAtLeast(kTcpClient client, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead); 
kFx(kStatus) kTcpClient_WriteAll(kTcpClient client, const kByte* buffer, kSize count);

#define kTcpClient_Cast_(TCP)                       (kCastClass_(kTcpClient, TCP))

kEndHeader()

#endif
