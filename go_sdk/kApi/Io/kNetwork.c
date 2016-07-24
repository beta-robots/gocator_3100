/** 
 * @file    kNetwork.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kNetwork.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Data/kArrayList.h>
#include <stdio.h>

/* 
 * kIpVersion
 */
kBeginEnum(k, kIpVersion, kValue)
    kAddEnumerator(kIpVersion, kIP_VERSION_4)
kEndValue()

/* 
 * kIpAddress
 */
kBeginValue(k, kIpAddress, kValue)
    kAddField(kIpAddress, kIpVersion, version)
    kAddField(kIpAddress, kByte, address)

    kAddVMethod(kIpAddress, kValue, VEquals)
    kAddVMethod(kIpAddress, kValue, VHashCode)
kEndValue()

kFx(kBool) kIpAddress_VEquals(kType type, const void* value, const void* other)
{
    const kIpAddress* a = value; 
    const kIpAddress* b = other; 

    if (a->version == b->version)
    {
        if (a->version == kIP_VERSION_4)
        {
            return kMemEquals(a->address, b->address, 4); 
        }
    }

    return kFALSE;
}

kFx(kSize) kIpAddress_VHashCode(kType type, const void* value)
{
    const kIpAddress* address = value; 

    if (address->version == kIP_VERSION_4)
    {
        return kHashBytes(address->address, 4); 
    }
    else
    {
        return kHashBytes(address->address, sizeof(address->address)); 
    }
}

kFx(kIpAddress) kIpAddress_Any(kIpVersion version)
{    
    kAssert(version == kIP_VERSION_4); 

    return kIpAddress_AnyV4(); 
}

kFx(kIpAddress) kIpAddress_AnyV4()
{
    kIpAddress address = { kIP_VERSION_4 }; 

    return address; 
}

kFx(kIpAddress) kIpAddress_BroadcastV4()
{   
    kIpAddress address = { kIP_VERSION_4 }; 

    address.address[0] = 0xFF; 
    address.address[1] = 0xFF; 
    address.address[2] = 0xFF; 
    address.address[3] = 0xFF; 

    return address;   
}

kFx(kIpAddress) kIpAddress_Loopback(kIpVersion version)
{
    kAssert(version == kIP_VERSION_4); 

    return kIpAddress_LoopbackV4(); 
}

kFx(kIpAddress) kIpAddress_LoopbackV4()
{
    kIpAddress address = { kIP_VERSION_4 }; 

    address.address[0] = 0x7F; 
    address.address[1] = 0x00; 
    address.address[2] = 0x00; 
    address.address[3] = 0x01; 

    return address; 
}

kFx(kStatus) kIpAddress_ToSockAddr(kIpAddress address, k32u port, struct sockaddr_in *sockAddr)
{
    kMemSet(sockAddr, 0, sizeof(struct sockaddr_in));

    sockAddr->sin_family = AF_INET;
    sockAddr->sin_addr.s_addr = kIpAddress_ToNet32u(address); 
    sockAddr->sin_port = htons((k16s)port); 

    return kOK;
}

kFx(kStatus) kIpAddress_FromSockAddr(struct sockaddr_in* sockAddr, kIpAddress *address, k32u *port)
{   
    if (!kIsNull(address))
    {
        *address = kIpAddress_FromNet32u(sockAddr->sin_addr.s_addr); 
    }

    if (!kIsNull(port))
    {
        *port =  htons(sockAddr->sin_port); 
    }

    return kOK;
}

kFx(kStatus) kIpAddress_Parse(kIpAddress* address, const kChar* text)
{
    k32u a, b, c, d;

    if ((sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) || 
        (a > k8U_MAX) || (b > k8U_MAX) || (c > k8U_MAX) || (d > k8U_MAX))
    {
        return kERROR_PARAMETER; 
    }

    kItemZero_(address, sizeof(kIpAddress)); 

    address->version = kIP_VERSION_4; 

    address->address[0] = a; 
    address->address[1] = b; 
    address->address[2] = c; 
    address->address[3] = d; 
     
    return kOK;
}

kFx(kStatus) kIpAddress_Format(kIpAddress address, kChar* text, kSize capacity)
{        
    kCheckArgs(address.version == kIP_VERSION_4); 

    kStrPrintf(text, capacity, "%u.%u.%u.%u", address.address[0], address.address[1], address.address[2], address.address[3]); 
    
    return kOK;
}

kFx(kBool) kIpAddress_Equals(kIpAddress a, kIpAddress b)
{
    if (a.version == kIP_VERSION_4)
    {
        return (b.version == a.version) && kMemEquals(&a.address[0], &b.address[0], 4); 
    }
    else
    {
        return kFALSE; 
    }    
}

kFx(kBool) kIpAddress_IsLoopback(kIpAddress address)
{
    return kIpAddress_Equals(address, kIpAddress_Loopback(address.version)); 
}

kFx(k32u) kIpAddress_ToHost32u(kIpAddress address)
{
    kAssert(address.version == kIP_VERSION_4); 

    return (address.address[0] << 24) | (address.address[1] << 16) | (address.address[2] << 8) | (address.address[3]); 
}

kFx(k32u) kIpAddress_ToNet32u(kIpAddress address)
{
    kAssert(address.version == kIP_VERSION_4); 

    return (address.address[3] << 24) | (address.address[2] << 16) | (address.address[1] << 8) | (address.address[0]); 
}

kFx(kIpAddress) kIpAddress_FromHost32u(k32u address)
{
    kIpAddress out = { kIP_VERSION_4 }; 

    out.address[0] = (k8u) ((address >> 24) & 0xFF);
    out.address[1] = (k8u) ((address >> 16) & 0xFF);
    out.address[2] = (k8u) ((address >>  8) & 0xFF);
    out.address[3] = (k8u) (address & 0xFF);

    return out; 
}

kFx(kIpAddress) kIpAddress_FromNet32u(k32u address)
{
    kIpAddress out = { kIP_VERSION_4 }; 

    out.address[3] = (k8u) ((address >> 24) & 0xFF);
    out.address[2] = (k8u) ((address >> 16) & 0xFF);
    out.address[1] = (k8u) ((address >>  8) & 0xFF);
    out.address[0] = (k8u) (address & 0xFF);

    return out; 
}

/* 
 * kIpEndPoint
 */
kBeginValue(k, kIpEndPoint, kValue)
    kAddField(kIpEndPoint, kIpAddress, address)
    kAddField(kIpEndPoint, k32u, port)
kEndValue()

/* 
 * kIpEntry
 */
kBeginValue(k, kIpEntry, kValue)
    kAddField(kIpEntry, kIpAddress, address)
    kAddField(kIpEntry, kText128, name)
kEndValue()


/* 
 * kNetwork
 */
kBeginStaticClass(k, kNetwork)    
kEndStaticClass()

kFx(kStatus) kNetwork_InitStatic()
{
    if (kApiLib_NetworkInitializationEnabled_())
    {
        return kNetwork_OsOpen(); 
    }
  
    return kOK; 
}

kFx(kStatus) kNetwork_ReleaseStatic()
{    
    if (kApiLib_NetworkInitializationEnabled_())
    {
        kCheck(kNetwork_OsClose()); 
    }

    return kOK; 
}

//deprecated
kFx(kStatus) kNetwork_FindAddresses(const kChar* unused, kArrayList addresses)
{
    kArrayList entries = kNULL; 
    kSize i; 

    kTry
    {
        kTest(kArrayList_Construct(&entries, kTypeOf(kIpEntry), 0, kObject_Alloc_(addresses))); 
        
        kTest(kNetwork_LocalIpInterfaces(entries)); 
        
        kTest(kArrayList_Allocate(addresses, kTypeOf(kIpAddress), kArrayList_Count(entries))); 

        for (i = 0; i < kArrayList_Count(entries); ++i)
        {
            kIpEntry* entry = kArrayList_At(entries, i); 

            kTest(kArrayList_Add(addresses, &entry->address)); 
        }
    }
    kFinally
    {
        kObject_Destroy(entries); 
        kEndFinally(); 
    }

    return kOK; 
}

#if defined(K_WINDOWS)

kFx(kStatus) kNetwork_OsOpen()
{
    WORD wVersionRequested = MAKEWORD(2, 2);  
    WSADATA wsaData;

    kCheck(WSAStartup(wVersionRequested, &wsaData) == 0); 

    if((LOBYTE(wsaData.wVersion) != 2) || (HIBYTE(wsaData.wVersion) != 2)) 
    {
        WSACleanup();
        return kERROR_NETWORK;
    }

    return kOK; 
}

kFx(kStatus) kNetwork_OsClose()
{
    kCheck(WSACleanup() == 0); 

    return kOK; 
}

kFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries)
{
    k32u bufferLength = 0;
    k32u opStatus;
    PIP_ADAPTER_ADDRESSES result = NULL;
    const k32u MAX_ITERATIONS = 16; 
    k32u iterationCount = 0;

    kCheck(kArrayList_Allocate(entries, kTypeOf(kIpEntry), 0));

    //get initial guess for required buffer size
    if (GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, kNULL, &bufferLength) != ERROR_BUFFER_OVERFLOW)
    {
        return kOK; 
    }

    //the required buffer size may change between when we request it and when it's needed (below); iterate 
    do
    {
        kCheck(kMemAlloc(bufferLength, &result));

        opStatus = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, result, &bufferLength);

        if (opStatus != ERROR_SUCCESS)
        {
            kMemFreeRef(&result); 

            if (opStatus != ERROR_BUFFER_OVERFLOW)
            {
                return kERROR_NETWORK; 
            }
        }

        iterationCount++; 
    } 
    while ((opStatus != ERROR_SUCCESS) && (iterationCount < MAX_ITERATIONS));

    if (opStatus == ERROR_SUCCESS)
    {
        PIP_ADAPTER_ADDRESSES it = result;

        while (!kIsNull(it))
        {
            PIP_ADAPTER_UNICAST_ADDRESS unicastIt = it->FirstUnicastAddress;

            while (!kIsNull(unicastIt))
            {
                struct sockaddr_in* addr = (struct sockaddr_in *) unicastIt->Address.lpSockaddr;
                kIpEntry entry;

                kZero_(entry);

                if (kSuccess(kIpAddress_FromSockAddr(addr, &entry.address, kNULL)))
                {
                    //Loopback is not reported by kNetwork_LocalIpInterfaces
                    if (!kIpAddress_IsLoopback(entry.address))
                    {
                        if (WideCharToMultiByte(CP_UTF8, 0, it->FriendlyName, -1, entry.name, kCountOf(entry.name), kNULL, kNULL) == 0)
                        {
                            entry.name[0] = 0;
                        }

                        kArrayList_Add(entries, &entry);
                    }
                }

                unicastIt = unicastIt->Next;
            }

            it = it->Next;
        }
    }

    kMemFree(result);

    return (opStatus == ERROR_SUCCESS) ? kOK : kERROR_NETWORK; 
}

#elif defined (K_DSP_BIOS)

kFx(kStatus) kNetwork_OsOpen()
{       
     return kOK; 
}

kFx(kStatus) kNetwork_OsClose()
{     
     return kOK; 
}

kFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries)
{
    kSize i; 

    kCheck(kArrayList_Allocate(entries, kTypeOf(kIpEntry), 0)); 

    //maximum of two network entries supported
    for (i = 0; i < 2; ++i)
    {
        IPN stackIpAddr; 

        //gets the "primary" address associated with a local network interface; 
        //will not report additional addresses for a multi-homed interface
        if (NtIfIdx2Ip(i, &stackIpAddr) == 1)
        {
            kIpEntry entry = { 0 }; 

            entry.address = kIpAddress_FromNet32u(stackIpAddr); 

            //TODO: add name support, if needed
            entry.name[0] = 0; 

            kCheck(kArrayList_Add(entries, &entry)); 
        }
    }

    if (kArrayList_Count_(entries) < 1)
    {
        return kERROR_NOT_FOUND; 
    }       

    return kOK; 
}

#elif defined (K_VX_KERNEL)

kFx(kStatus) kNetwork_OsOpen()
{       
     return kOK; 
}

kFx(kStatus) kNetwork_OsClose()
{     
     return kOK; 
}

kFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries)
{
    kSocket sock = kNULL;
    const kSize bufferSize = 2048; 
    kByte* buffer = kNULL; 
    struct ifconf ifConfig = {0};
    struct ifreq *ifIt = NULL;

    kTry
    {
        kTest(kArrayList_Allocate(entries, kTypeOf(kIpEntry), 0)); 
        kTest(kMemAllocZero(bufferSize, &buffer)); 

        kTest(kSocket_Construct(&sock, kIP_VERSION_4, kSOCKET_TYPE_UDP, kNULL)); 
          
        ifConfig.ifc_len = bufferSize;
        ifConfig.ifc_buf = (kChar*) buffer;
                
        kTest(ioctl(kSocket_Handle_(sock), SIOCGIFCONF, &ifConfig) == 0); 
                
        ifIt = ifConfig.ifc_req;

        while ((kByte*)ifIt < (buffer + ifConfig.ifc_len))
        {            
            switch (ifIt->ifr_addr.sa_family) 
            {           
            case AF_INET:
                {
                    struct sockaddr_in* addr = (struct sockaddr_in*) &ifIt->ifr_addr;
                    kIpEntry entry; 
                    
                    kStrCopy(entry.name, sizeof(entry.name), ifIt->ifr_name); 
                    entry.address = kIpAddress_FromNet32u(addr->sin_addr.s_addr); 
                  
                    //Loopback is not reported by kNetwork_LocalIpInterfaces
                    if (!kIpAddress_IsLoopback(entry.address))
                    {                
                        kTest(kArrayList_Add(entries, &entry)); 
                    }
                    break;
                }                
            default:
                break;
            }
                    
            ifIt = (struct ifreq*)((kByte*)ifIt +_SIZEOF_ADDR_IFREQ(*ifIt)); 
        }
                  
    }
    kFinally
    {
        kMemFree(buffer); 
        kObject_Destroy(sock); 
        kEndFinally();
    }

    return kOK;
}

#elif defined (K_LINUX)

kFx(kStatus) kNetwork_OsOpen()
{       
     return kOK; 
}

kFx(kStatus) kNetwork_OsClose()
{     
     return kOK; 
}

kFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries)
{
    kSocket sock = kNULL;
    const kSize bufferSize = 2048; 
    kByte* buffer = kNULL; 
    struct ifconf ifconfig = {0};
    struct ifreq *ifrequest = NULL;
    int interfaceCount = 0;
    int i;

    kTry
    {
        kTest(kArrayList_Allocate(entries, kTypeOf(kIpEntry), 0)); 

        kTest(kMemAllocZero(bufferSize, &buffer)); 

        kTest(kSocket_Construct(&sock, kIP_VERSION_4, kSOCKET_TYPE_UDP, kNULL)); 
  
        ifconfig.ifc_len = bufferSize;
        ifconfig.ifc_buf = (kChar*) buffer;
        
        kTest(ioctl(kSocket_Handle_(sock), SIOCGIFCONF, &ifconfig) == 0); 
        
        ifrequest = ifconfig.ifc_req;
        interfaceCount = ifconfig.ifc_len / sizeof(struct ifreq);
        
        for (i = 0; i < interfaceCount; ++i)
        {
            struct ifreq *item = &ifrequest[i];
             
            if (item->ifr_addr.sa_family == AF_INET)
            {
                struct sockaddr_in* addr = (struct sockaddr_in*) &item->ifr_addr;
                kIpEntry entry; 

                kZero_(entry); 

                entry.address = kIpAddress_FromNet32u(addr->sin_addr.s_addr);                 
                kStrCopy(entry.name, kCountOf(entry.name), item->ifr_name); 
                                
                //Loopback is not reported by kNetwork_LocalIpInterfaces
                if (!kIpAddress_IsLoopback(entry.address))
                {                
                    kTest(kArrayList_Add(entries, &entry)); 
                }
            }
        }
    }
    kFinally
    {
        kMemFree(buffer); 
        kObject_Destroy(sock); 
        kEndFinally();
    }

    return kOK;
}

#elif defined (K_DARWIN)

kFx(kStatus) kNetwork_OsOpen()
{       
    return kOK; 
}

kFx(kStatus) kNetwork_OsClose()
{     
    return kOK; 
}

kFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries)
{
    struct ifaddrs *ifap, *item;
    
    if(getifaddrs(&ifap) != 0)
    {
        //could test errno to create a more meaningful error
        return kERROR;
    }
    
    kTry
    {
        for(item = ifap; item != kNULL; item = item->ifa_next)
        {
            if(item->ifa_addr->sa_family == AF_INET)
            {
                struct sockaddr_in* addr = (struct sockaddr_in*) item->ifa_addr;
                kIpEntry entry;
                
                kZero_(entry);
                
                entry.address = kIpAddress_FromNet32u(addr->sin_addr.s_addr);
                kStrCopy(entry.name, kCountOf(entry.name), item->ifa_name);
                
                //Loopback is not reported by kNetwork_LocalIpInterfaces
                if (!kIpAddress_IsLoopback(entry.address))
                {
                    kTest(kArrayList_Add(entries, &entry));
                }
            }
        }
        
    }
    kFinally
    {
        freeifaddrs(ifap);
        kEndFinally();
    }
    
    return kOK;
}

#endif

