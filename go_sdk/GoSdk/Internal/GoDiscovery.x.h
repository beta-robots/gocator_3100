/** 
 * @file    GoDiscovery.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_X_H
#define GO_SDK_DISCOVERY_X_H

#include <GoSdk/Internal/GoDiscovery.h>
#include <kApi/Threads/kPeriodic.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Io/kUdpClient.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Io/kUdpClient.h>
#include <kApi/Data/kMap.h>
#include <kApi/Io/kUdpClient.h>
#include <kApi/Io/kSocket.h>
kBeginHeader()

kDeclareValue(Go, GoDiscoveryInfo, kValue)

typedef struct GoDiscoveryInterface
{
    kIpAddress address; 
    kUdpClient client; 
    kSerializer writer; 
} GoDiscoveryInterface; 

kDeclareValue(Go, GoDiscoveryInterface, kValue)

/* 
 * GoDiscovery class
 */

#define GO_DISCOVERY_PORT                           (3220)
#define GO_DISOVERY_SIGNATURE                       (0x504455494D4CLL)
#define GO_DISCOVERY_MAX_INTERFACES                 (32)

#define GO_DISCOVERY_GET_ADDRESS                    (0x0001)
#define GO_DISCOVERY_GET_ADDRESS_SIZE               (32)
#define GO_DISCOVERY_GET_ADDRESS_REPLY              (0x1001)
#define GO_DISCOVERY_GET_ADDRESS_TIMEOUT            (500000)

#define GO_DISCOVERY_SET_ADDRESS                    (0x0002)
#define GO_DISCOVERY_SET_ADDRESS_SIZE               (72)
#define GO_DISCOVERY_SET_ADDRESS_REPLY              (0x1002)
#define GO_DISCOVERY_SET_ADDRESS_TIMEOUT            (6000000)

#define GO_DISCOVERY_SET_ADDRESS_TEST               (0x0E00)
#define GO_DISCOVERY_SET_ADDRESS_TEST_SIZE          (72)
#define GO_DISCOVERY_SET_ADDRESS_TEST_REPLY         (0x1E00)
#define GO_DISCOVERY_SET_ADDRESS_TEST_TIMEOUT       (6000000)

typedef struct GoDiscoveryClass
{
    kObjectClass base; 

    k32u localPort; 
    k64u enumPeriod; 
    kCallback onEnumerate; 

    kArrayList infoList;

    kArrayList interfaces; 
    kUdpClient receiver; 
    kSerializer reader;

    kPeriodic eventTimer; 
    kTimer stopwatch; 
    kBool enumPending; 
    k64u runCount; 
} GoDiscoveryClass; 

kDeclareClass(Go, GoDiscovery, kObject)

#define GoDiscovery_Cast_(CONTEXT)    kCastClass_(GoDiscovery, CONTEXT)

GoFx(kStatus) GoDiscovery_Init(GoDiscovery discovery, kType type, kAlloc alloc);
GoFx(kStatus) GoDiscovery_VRelease(GoDiscovery discovery);

GoFx(kStatus) GoDiscovery_ParseGetReply(GoDiscovery discovery, GoDiscoveryInfo* info);
GoFx(kStatus) GoDiscovery_ParseSetReply(GoDiscovery discovery, k32u* deviceId);
GoFx(kStatus) GoDiscovery_ParseSetTestReply(GoDiscovery discovery, k32u* deviceId);

GoFx(kStatus) GoDiscovery_BeginEnum(GoDiscovery discovery);
GoFx(kStatus) GoDiscovery_EndEnum(GoDiscovery discovery, kArrayList list); 

GoFx(kStatus) GoDiscovery_RemoveDuplicates(kArrayList infoList);

GoFx(kStatus) GoDiscovery_OnEnumElapsed(GoDiscovery discovery, kPeriodic timer);

GoFx(kStatus) GoDiscovery_SetAddressTest(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address); 

kEndHeader()

#endif
