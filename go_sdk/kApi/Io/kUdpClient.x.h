/** 
 * @file    kUdpClient.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_UDP_CLIENT_X_H
#define K_API_UDP_CLIENT_X_H

#include <kApi/Io/kStream.h>

kBeginHeader()

typedef struct kUdpClientClass
{
    kStreamClass base; 
    kSocket socket;                 //Socket object. 
    kBool isBroacastReceiver;       //Work-around for DSP/BIOS; see kUdpClient_EnableBroadcastReceive comments. 
} kUdpClientClass;

kDeclareClass(k, kUdpClient, kStream)

#define kUdpClient_Cast_(UDP)       (kCastClass_(kUdpClient, UDP))

kFx(kStatus) kUdpClient_Init(kUdpClient client, kType type, kIpVersion ipVersion, kAlloc allocator); 
kFx(kStatus) kUdpClient_VRelease(kUdpClient client);

kFx(kStatus) kUdpClient_VReadSomeImpl(kUdpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kUdpClient_VWriteImpl(kUdpClient client, const void* buffer, kSize size);
kFx(kStatus) kUdpClient_VFlush(kUdpClient client);

kEndHeader()

#endif
