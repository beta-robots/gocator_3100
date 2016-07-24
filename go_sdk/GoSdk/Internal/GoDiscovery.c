/** 
 * @file    GoDiscovery.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoDiscovery.h>
#include <kApi/Utils/kUtils.h>

kBeginValue(Go, GoDiscoveryInfo, kValue)
    kAddField(GoDiscoveryInfo, k32u, id)
    kAddField(GoDiscoveryInfo, GoAddressInfo, address) 
kEndValue()

kBeginValue(Go, GoDiscoveryInterface, kValue)
    kAddField(GoDiscoveryInterface, kIpAddress, address)
    kAddField(GoDiscoveryInterface, kUdpClient, client)
    kAddField(GoDiscoveryInterface, kSerializer, writer)
kEndValue()


/* 
 * GoDiscovery class
 */

kBeginClass(Go, GoDiscovery, kObject)
    kAddVMethod(GoDiscovery, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoDiscovery_Construct(GoDiscovery* discovery, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDiscovery), discovery)); 

    if (!kSuccess(status = GoDiscovery_Init(*discovery, kTypeOf(GoDiscovery), alloc)))
    {
        kAlloc_FreeRef(alloc, discovery); 
    }

    return status; 
} 

GoFx(kStatus) GoDiscovery_Init(GoDiscovery discovery, kType type, kAlloc alloc)
{
    GoDiscoveryClass* obj = discovery; 
    kArrayList addresses = kNULL; 
    kStatus status; 
    kIpEndPoint localEndPoint; 
    kSize i; 

    kCheck(kObject_Init(discovery, type, alloc)); 
    kZeroDerivedFields_(GoDiscovery, discovery); 

    kTry
    {
        kTest(kArrayList_Construct(&obj->infoList, kTypeOf(GoDiscoveryInfo), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->interfaces, kTypeOf(GoDiscoveryInterface), 0, alloc)); 

        kTest(kUdpClient_Construct(&obj->receiver, kIP_VERSION_4, alloc));
        kTest(kUdpClient_EnableReuseAddress(obj->receiver, kTRUE));
        kTest(kUdpClient_SetReadBuffers(obj->receiver, -1, 2048)); 
        kTest(kUdpClient_Bind(obj->receiver, kIpAddress_AnyV4(), kIP_PORT_ANY));

        kTest(kUdpClient_LocalEndPoint(obj->receiver, &localEndPoint)); 
        obj->localPort = localEndPoint.port; 

        kTest(kSerializer_Construct(&obj->reader, obj->receiver, kNULL, alloc));               

        kTest(kPeriodic_Construct(&obj->eventTimer, alloc)); 
        kTest(kTimer_Construct(&obj->stopwatch, alloc)); 

        kTest(kArrayList_Construct(&addresses, kTypeOf(kIpEntry), 0, alloc));
        kTest(kNetwork_LocalIpInterfaces(addresses));

        for (i = 0; i < kArrayList_Count_(addresses); ++i)
        {
            GoDiscoveryInterface iface; 
            kIpEntry entry;           

            entry = kArrayList_As_(addresses, i, kIpEntry);
            iface.address = entry.address;
            iface.client = kNULL;  
            iface.writer = kNULL;

            kTest(kUdpClient_Construct(&iface.client, kIP_VERSION_4, alloc));
            kTest(kUdpClient_EnableBroadcast(iface.client, kTRUE));
            kTest(kUdpClient_EnableReuseAddress(iface.client, kTRUE));
            
            if (kSuccess(kUdpClient_Bind(iface.client, iface.address, obj->localPort)))
            {
                kTest(kUdpClient_SetWriteBuffers(iface.client, -1, 2048));
                kTest(kSerializer_Construct(&iface.writer, iface.client, kNULL, alloc)); 
                kTest(kArrayList_Add(obj->interfaces, &iface)); 
            }
            else
            {
                kDestroyRef(&iface.client);
            }
        }            
    }
    kCatchEx(&status)
    {
        GoDiscovery_VRelease(discovery); 
        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        kDisposeRef(&addresses); 
        kEndFinallyEx(); 
    }

    return kOK; 
}

GoFx(kStatus) GoDiscovery_VRelease(GoDiscovery discovery)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    kSize i; 

    if (!kIsNull(obj->eventTimer))
    {
        kCheck(GoDiscovery_StopEnum(discovery)); 
    }

    kCheck(kDisposeRef(&obj->infoList)); 

    if (!kIsNull(obj->interfaces))
    {
        for (i = 0; i < kArrayList_Count_(obj->interfaces); ++i)
        {
            GoDiscoveryInterface* iface = kArrayList_At_(obj->interfaces, i); 

            kCheck(kDestroyRef(&iface->writer)); 
            kCheck(kDestroyRef(&iface->client));  
        }

        kCheck(kDisposeRef(&obj->interfaces)); 
    }

    kCheck(kDestroyRef(&obj->reader)); 
    kCheck(kDestroyRef(&obj->receiver)); 

    kCheck(kDestroyRef(&obj->eventTimer)); 
    kCheck(kDestroyRef(&obj->stopwatch)); 

    return kObject_VRelease(discovery);
}

GoFx(kStatus) GoDiscovery_WriteIpAddress(kSerializer serializer, kIpAddress address)
{
    kByte bytes[8]; 
    
    bytes[0] = 0; 
    bytes[1] = 0; 
    bytes[2] = 0; 
    bytes[3] = 0; 
    bytes[4] = (address.address[0] & 0xFF); 
    bytes[5] = (address.address[1] & 0xFF); 
    bytes[6] = (address.address[2] & 0xFF); 
    bytes[7] = (address.address[3] & 0xFF); 

    kCheck(kSerializer_WriteByteArray_(serializer, bytes, kCountOf(bytes))); 

    return kOK; 
}

GoFx(kStatus) GoDiscovery_ReadIpAddress(kSerializer serializer, kIpAddress* address)
{
    kByte bytes[8]; 
    kSize i;

    kCheck(kSerializer_ReadByteArray_(serializer, bytes, kCountOf(bytes))); 

    address->address[0] = bytes[4];
    address->address[1] = bytes[5];
    address->address[2] = bytes[6];
    address->address[3] = bytes[7];

    for (i = 4; i < 16; i++)
    {
        address->address[i] = 0;
    }

    address->version = kIP_VERSION_4;

    return kOK; 
}

GoFx(kStatus) GoDiscovery_Enumerate(GoDiscovery discovery, kArrayList infoList)
{
    kCheck(GoDiscovery_BeginEnum(discovery)); 
    kCheck(kThread_Sleep(GO_DISCOVERY_GET_ADDRESS_TIMEOUT)); 
    kCheck(GoDiscovery_EndEnum(discovery, infoList));

    return kOK; 
}

GoFx(kStatus) GoDiscovery_GetAddress(GoDiscovery discovery, k32u deviceId, GoAddressInfo* address)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    GoDiscoveryInfo info; 
    kIpEndPoint remoteEndPoint;
    kSize i; 
    kStatus sendStatus = kERROR_NETWORK;

    for (i = 0; i < kArrayList_Count_(obj->interfaces); ++i)
    {        
        GoDiscoveryInterface* iface = kArrayList_At_(obj->interfaces, i); 

        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS_SIZE)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
        kCheck(kSerializer_Write64s(iface->writer, deviceId));

        kCheck(kSerializer_Flush_(iface->writer)); 
        if kSuccess(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_GET_ADDRESS_TIMEOUT, kTRUE))
        {
            sendStatus = kOK;
        }
    }

    kCheck(sendStatus);
    kCheck(kTimer_Start(obj->stopwatch, GO_DISCOVERY_GET_ADDRESS_TIMEOUT)); 

    do
    {
        if (kSuccess(kUdpClient_Receive(obj->receiver, &remoteEndPoint, kNULL, kTimer_Remaining(obj->stopwatch)))
            && remoteEndPoint.port == GO_DISCOVERY_PORT
            && kSuccess(GoDiscovery_ParseGetReply(discovery, &info))
            && info.id == deviceId)
        {                    
            *address = info.address; 
            return kOK; 
        }
    } while (!kTimer_IsExpired(obj->stopwatch));

    return kERROR_NOT_FOUND; 
}

GoFx(kStatus) GoDiscovery_SetAddress(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    k32u responseDeviceId; 
    kIpEndPoint remoteEndPoint;
    kIpAddress zero; 
    kSize i; 
    kStatus sendStatus = kERROR_NETWORK;

    for (i = 0; i < 16; i++)
    {
        zero.address[i] = 0;
    }
    
    for (i = 0; i < kArrayList_Count_(obj->interfaces); ++i)
    {        
        GoDiscoveryInterface* iface = kArrayList_At_(obj->interfaces, i); 

        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS_SIZE)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
        kCheck(kSerializer_Write64s(iface->writer, deviceId));
        kCheck(kSerializer_Write64s(iface->writer, address->useDhcp));

        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->address));
        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->mask));
        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->gateway));
        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : zero));

        kCheck(kSerializer_Flush_(iface->writer)); 
        if kSuccess(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_SET_ADDRESS_TIMEOUT, kTRUE))
        {
            sendStatus = kOK;
        }
    }

    kCheck(sendStatus);
    kCheck(kTimer_Start(obj->stopwatch, GO_DISCOVERY_SET_ADDRESS_TIMEOUT)); 

    do
    {
        if (kSuccess(kUdpClient_Receive(obj->receiver, &remoteEndPoint, kNULL, kTimer_Remaining(obj->stopwatch)))
            && remoteEndPoint.port == GO_DISCOVERY_PORT
            && kSuccess(GoDiscovery_ParseSetReply(discovery, &responseDeviceId))
            && responseDeviceId == deviceId)
        {
            return kOK; 
        }
    } while (!kTimer_IsExpired(obj->stopwatch));

    return kERROR_NOT_FOUND; 
}

GoFx(kStatus) GoDiscovery_SetAddressTest(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    k32u responseDeviceId; 
    kIpEndPoint remoteEndPoint;
    kIpAddress zero; 
    kSize i; 

    for (i = 0; i < 16; i++)
    {
        zero.address[i] = 0;
    }
    
    for (i = 0; i < kArrayList_Count_(obj->interfaces); ++i)
    {        
        GoDiscoveryInterface* iface = kArrayList_At_(obj->interfaces, i); 

        kCheck(kSocket_Connect(kUdpClient_Socket(iface->client), kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, 1000000));
        
        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS_SIZE)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS_TEST)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
        kCheck(kSerializer_Write64s(iface->writer, deviceId));
        kCheck(kSerializer_Write64s(iface->writer, address->useDhcp));

        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->address));
        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->mask));
        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->gateway));
        kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : zero));

        kCheck(kSerializer_Flush_(iface->writer)); 
        kCheck(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_SET_ADDRESS_TIMEOUT, kTRUE));
    }

    kCheck(kTimer_Start(obj->stopwatch, GO_DISCOVERY_SET_ADDRESS_TIMEOUT)); 

    do
    {
        if (kSuccess(kUdpClient_Receive(obj->receiver, &remoteEndPoint, kNULL, kTimer_Remaining(obj->stopwatch)))
            && remoteEndPoint.port == GO_DISCOVERY_PORT
            && kSuccess(GoDiscovery_ParseSetTestReply(discovery, &responseDeviceId))
            && responseDeviceId == deviceId)
        {
            return kOK; 
        }
    } while (!kTimer_IsExpired(obj->stopwatch));

    return kERROR_NOT_FOUND; 
}

GoFx(kStatus) GoDiscovery_ParseGetReply(GoDiscovery discovery, GoDiscoveryInfo* info)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    k64s size, responseId, signature, status, deviceId, useDhcp; 
    kIpAddress tempIp; 

    kCheck(kSerializer_Read64s(obj->reader, &size));

    kCheck(kSerializer_Read64s(obj->reader, &responseId));
    kCheck(responseId == GO_DISCOVERY_GET_ADDRESS_REPLY); 

    kCheck(kSerializer_Read64s(obj->reader, &status));
    kCheck(status); 

    kCheck(kSerializer_Read64s(obj->reader, &signature));
    kCheck(signature == GO_DISOVERY_SIGNATURE); 
    
    kCheck(kSerializer_Read64s(obj->reader, &deviceId));
    info->id = (k32u) deviceId; 

    kCheck(kSerializer_Read64s(obj->reader, &useDhcp));
    info->address.useDhcp = (kBool)useDhcp; 

    kCheck(GoDiscovery_ReadIpAddress(obj->reader, &info->address.address));
    kCheck(GoDiscovery_ReadIpAddress(obj->reader, &info->address.mask));
    kCheck(GoDiscovery_ReadIpAddress(obj->reader, &info->address.gateway));
    kCheck(GoDiscovery_ReadIpAddress(obj->reader, &tempIp));

    return kOK; 
}

GoFx(kStatus) GoDiscovery_ParseSetReply(GoDiscovery discovery, k32u* deviceId)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    k64s responseDeviceId, responseSignature;    
    k64s responseSize, responseId, responseStatus;

    kCheck(kSerializer_Read64s(obj->reader, &responseSize));

    kCheck(kSerializer_Read64s(obj->reader, &responseId));
    kCheck(responseId == GO_DISCOVERY_SET_ADDRESS_REPLY); 

    kCheck(kSerializer_Read64s(obj->reader, &responseStatus));
    kCheck(responseStatus); 

    kCheck(kSerializer_Read64s(obj->reader, &responseSignature));
    kCheck(responseSignature == GO_DISOVERY_SIGNATURE); 

    kCheck(kSerializer_Read64s(obj->reader, &responseDeviceId));

    *deviceId = (k32u) responseDeviceId; 

    return kOK; 
}  

GoFx(kStatus) GoDiscovery_ParseSetTestReply(GoDiscovery discovery, k32u* deviceId)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    k64s responseDeviceId, responseSignature;    
    k64s responseSize, responseId, responseStatus;

    kCheck(kSerializer_Read64s(obj->reader, &responseSize));

    kCheck(kSerializer_Read64s(obj->reader, &responseId));
    kCheck(responseId == GO_DISCOVERY_SET_ADDRESS_TEST_REPLY); 

    kCheck(kSerializer_Read64s(obj->reader, &responseStatus));
    kCheck(responseStatus); 

    kCheck(kSerializer_Read64s(obj->reader, &responseSignature));
    kCheck(responseSignature == GO_DISOVERY_SIGNATURE); 

    kCheck(kSerializer_Read64s(obj->reader, &responseDeviceId));

    *deviceId = (k32u) responseDeviceId; 

    return kOK; 
}   

GoFx(kStatus) GoDiscovery_SetEnumPeriod(GoDiscovery discovery, k64u period)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 

    obj->enumPeriod = period; 

    return kOK; 
}

GoFx(kStatus) GoDiscovery_SetEnumHandler(GoDiscovery discovery, GoDiscoveryEnumFx function, kPointer receiver)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 

    obj->onEnumerate.function = (kCallbackFx) function; 
    obj->onEnumerate.receiver = receiver; 

    return kOK; 
}

GoFx(kStatus) GoDiscovery_StartEnum(GoDiscovery discovery, kBool waitFirst)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 

    kCheckState(!kIsNull(obj->onEnumerate.function)); 
    kCheckState(!kPeriodic_Enabled(obj->eventTimer)); 

    obj->runCount = 0; 

    if (waitFirst)
    {
        kCheck(GoDiscovery_Enumerate(discovery, obj->infoList)); 
        kCheck(obj->onEnumerate.function(obj->onEnumerate.receiver, discovery, obj->infoList));        
    }

    kCheck(kPeriodic_Start(obj->eventTimer, 0, GoDiscovery_OnEnumElapsed, discovery)); 

    return kOK; 
}

GoFx(kStatus) GoDiscovery_StopEnum(GoDiscovery discovery)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 

    kCheck(kPeriodic_Stop(obj->eventTimer)); 

    obj->runCount = 0; 

    return kOK; 
}

GoFx(kStatus) GoDiscovery_BeginEnum(GoDiscovery discovery)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    kSize i;
       
    for (i = 0; i < kArrayList_Count_(obj->interfaces); ++i)
    {        
        GoDiscoveryInterface* iface = kArrayList_At_(obj->interfaces, i); 

        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS_SIZE)); 
        kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS));
        kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
        kCheck(kSerializer_Write64s(iface->writer, 0));

        kCheck(kSerializer_Flush_(iface->writer));         
        kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_GET_ADDRESS_TIMEOUT, kTRUE);   //no check made due to a possibility of the current network interface being invalid
    }

    return kOK; 
}

GoFx(kStatus) GoDiscovery_EndEnum(GoDiscovery discovery, kArrayList list)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    GoDiscoveryInfo info; 
    kIpEndPoint remoteEndPoint;

    kCheck(kArrayList_Allocate(list, kTypeOf(GoDiscoveryInfo), 0)); 

    while (kSuccess(kSocket_Wait(kUdpClient_Socket(obj->receiver), 0)))
    {
        kCheck(kUdpClient_Receive(obj->receiver, &remoteEndPoint, kNULL, 1000000));

        if (remoteEndPoint.port == GO_DISCOVERY_PORT
            && kSuccess(GoDiscovery_ParseGetReply(discovery, &info)))
        {
            kCheck(kArrayList_Add(list, &info)); 
        }
    }

    kCheck(GoDiscovery_RemoveDuplicates(list)); 

    return kOK; 
}

GoFx(kStatus) GoDiscovery_OnEnumElapsed(GoDiscovery discovery, kPeriodic timer)
{
    GoDiscoveryClass* obj = GoDiscovery_Cast_(discovery); 
    
    if (!obj->enumPending)       
    {
        kCheck(kTimer_Start(obj->stopwatch, obj->enumPeriod)); 
        kCheck(GoDiscovery_BeginEnum(discovery)); 
        obj->enumPending = kTRUE; 

        kCheck(kPeriodic_Start(obj->eventTimer, GO_DISCOVERY_GET_ADDRESS_TIMEOUT, GoDiscovery_OnEnumElapsed, discovery)); 
    }
    else
    {
        kCheck(GoDiscovery_EndEnum(discovery, obj->infoList)); 
        kCheck(obj->onEnumerate.function(obj->onEnumerate.receiver, discovery, obj->infoList)); 

        obj->enumPending = kFALSE; 

        kCheck(kPeriodic_Start(obj->eventTimer, kTimer_Remaining(obj->stopwatch), GoDiscovery_OnEnumElapsed, discovery)); 

        obj->runCount++; 
    }

    return kOK; 
}

GoFx(kStatus) GoDiscovery_RemoveDuplicates(kArrayList infoList)
{
    kSize count = kArrayList_Count(infoList); 
    kSize i = 0; 
    kSize j = 0; 

    while (i < count)
    {
        GoDiscoveryInfo* itemI = kArrayList_At_(infoList, i); 
        
        j = i+1;         

        while (j < count)
        {
            GoDiscoveryInfo* itemJ = kArrayList_At_(infoList, j); 

            if (itemI->id == itemJ->id)
            {
                kCheck(kArrayList_Remove(infoList, j, kNULL)); 
                count--; 
            }
            else
            {
                ++j; 
            }
        }
        ++i; 
    }       

    return kOK; 
}
