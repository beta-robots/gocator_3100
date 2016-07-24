/** 
 * @file    kNetwork.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_NETWORK_X_H
#define K_API_NETWORK_X_H

kBeginHeader()

kDeclareEnum(k, kIpVersion, kValue)

kDeclareValue(k, kIpAddress, kValue)

kFx(kBool) kIpAddress_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kIpAddress_VHashCode(kType type, const void* value); 

kDeclareValue(k, kIpEndPoint, kValue)

kDeclareValue(k, kIpEntry, kValue)

typedef struct kNetworkStatic
{
    k32u placeholder;       //unused
} kNetworkStatic; 

kDeclareStaticClass(k, kNetwork)

kFx(kStatus) kNetwork_InitStatic(); 
kFx(kStatus) kNetwork_ReleaseStatic(); 

#if defined(K_PLATFORM)

kFx(kStatus) kIpAddress_ToSockAddr(kIpAddress address, k32u port, struct sockaddr_in* sockAddr);
kFx(kStatus) kIpAddress_FromSockAddr(struct sockaddr_in* sockAddr, kIpAddress *address, k32u* port);

kFx(kStatus) kNetwork_OsOpen(); 
kFx(kStatus) kNetwork_OsClose(); 

#endif

//deprecated
kFx(kStatus) kNetwork_FindAddresses(const kChar* unused, kArrayList addresses); 

#define kNetwork_FindEntries kNetwork_LocalIpInterfaces

kEndHeader()

#endif
