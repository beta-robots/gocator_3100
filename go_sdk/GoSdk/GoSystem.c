/** 
 * @file    GoSystem.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSystem.h>
#include <kApi/Utils/kUtils.h>

kBeginClass(Go, GoSystem, kObject)
    kAddVMethod(GoSystem, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSystem_Construct(GoSystem* system, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSystem), system)); 

    if (!kSuccess(status = GoSystem_Init(*system, kTypeOf(GoSystem), alloc)))
    {
        kAlloc_FreeRef(alloc, system); 
    }

    return status; 
} 

GoFx(kStatus) GoSystem_Init(GoSystem system, kType type, kAlloc alloc)
{
    GoSystemClass* obj = system; 
    kStatus status; 

    kCheck(kObject_Init(system, type, alloc)); 
    kInitFields_(GoSystem, system); 

    kTry
    {
        kTest(kLock_Construct(&obj->stateLock, alloc)); 
        kTest(kTimer_Construct(&obj->timer, alloc)); 

        kTest(GoDiscovery_Construct(&obj->discovery, alloc)); 
        kTest(GoDiscovery_SetEnumPeriod(obj->discovery, GO_SYSTEM_DISCOVERY_PERIOD)); 
        kTest(GoDiscovery_SetEnumHandler(obj->discovery, GoSystem_OnDiscovery, system)); 

        kTest(kPeriodic_Construct(&obj->healthCheck, alloc)); 

        kTest(kArrayList_Construct(&obj->allSensors, kTypeOf(GoSensor), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->onlineSensors, kTypeOf(GoSensor), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->bankList, kTypeOf(GoMultiplexBank), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->tempList, kTypeOf(GoSensor), 0, alloc)); 

        kTest(kMsgQueue_Construct(&obj->dataQueue, kTypeOf(GoDataSet), alloc)); 
        kTest(kMsgQueue_SetCapacity(obj->dataQueue, kMSG_QUEUE_CAPACITY_TYPE_MEMORY, GO_SYSTEM_DEFAULT_DATA_CAPACITY));
        kTest(kMsgQueue_Construct(&obj->healthQueue, kTypeOf(GoDataSet), alloc)); 
        kTest(kMsgQueue_SetCapacity(obj->healthQueue, kMSG_QUEUE_CAPACITY_TYPE_MEMORY, GO_SYSTEM_DEFAULT_HEALTH_CAPACITY));

        kTest(kPeriodic_Start(obj->healthCheck, GO_SYSTEM_HEALTH_CHECK_PERIOD, GoSystem_OnHealthCheck, system)); 
        kTest(GoDiscovery_StartEnum(obj->discovery, kTRUE)); 

        kTest(GoSystem_Refresh(system)); 
    }
    kCatch(&status)
    {
        GoSystem_VRelease(system); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_VRelease(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kCheck(GoSystem_SetDataHandler(system, kNULL, kNULL)); 
    kCheck(GoSystem_SetHealthHandler(system, kNULL, kNULL)); 

    if (!kIsNull(obj->healthCheck))
    {
        kCheck(kPeriodic_Stop(obj->healthCheck));
    }

    kCheck(kDestroyRef(&obj->healthCheck)); 
    kCheck(kDestroyRef(&obj->discovery)); 
    
    kCheck(kDestroyRef(&obj->onlineSensors)); //under normal circumstances, kDisposeRef would be called 
                                                //on these allSensors and onlineSensors lists, but they share 
                                                //GoSensor objects
    kCheck(kDisposeRef(&obj->allSensors)); 
    kCheck(kDestroyRef(&obj->tempList)); 
    kCheck(kDestroyRef(&obj->bankList)); 

    kCheck(kDisposeRef(&obj->dataQueue)); 
    kCheck(kDisposeRef(&obj->healthQueue)); 

    kCheck(kDestroyRef(&obj->timer)); 
    kCheck(kDestroyRef(&obj->stateLock)); 

    return kObject_VRelease(system); 
}

GoFx(kStatus) GoSystem_OnDiscovery(GoSystem system, GoDiscovery discovery, kArrayList list)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    GoSensor newSensor = kNULL; 
    kSize i; 

    kAssert(kArrayList_ItemType(list) == kTypeOf(GoDiscoveryInfo)); 

    kLock_Enter(obj->stateLock);

    kTry
    {
        kSize currentCount = kArrayList_Count_(obj->allSensors); 
        kSize incomingCount = kArrayList_Count_(list); 

        for (i = 0; i < currentCount; ++i)
        {
            GoSensorClass* sensorObj = kArrayList_As_(obj->allSensors, i, GoSensor); 
            kSize historyMask = kCountOf(sensorObj->discoveryHistory) - 1; 
            
            sensorObj->discoveryHistory[sensorObj->discoveryCount & historyMask] = kFALSE; 
        }

        for (i = 0; i < incomingCount; ++i)
        {
            GoDiscoveryInfo* incomingItem = (GoDiscoveryInfo*)kArrayList_At(list, i); 
            GoSensorClass* sensorObj = kNULL; 
            kSize historyMask = kCountOf(sensorObj->discoveryHistory) - 1; 

            if (!kSuccess(GoSystem_FindSensorAll(system, incomingItem->id, &sensorObj)))
            {              
                kTest(GoSensor_Construct(&newSensor, system, incomingItem, kObject_Alloc_(system))); 
                kTest(kArrayList_Add(obj->allSensors, &newSensor)); 
                
                sensorObj = newSensor;
                newSensor = kNULL; 
            }

            sensorObj->address = incomingItem->address; 
            sensorObj->discoveryHistory[sensorObj->discoveryCount & historyMask] = kTRUE; 
            sensorObj->discoveryCount++; 
        }

        for (i = 0; i < currentCount; ++i)
        {
            GoSensorClass* sensorObj = kArrayList_As_(obj->allSensors, i, GoSensor); 
            sensorObj->discoveryCount++; 
        }
    }
    kFinally
    {
        kObject_Destroy(newSensor); 
        kLock_Exit(obj->stateLock); 
        kEndFinally(); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_OnHealthCheck(GoSystem system, kPeriodic timer)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kLock_Enter(obj->stateLock);

    kTry
    {
        for (i = 0; i < kArrayList_Count_(obj->onlineSensors); ++i)
        {
            GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 
            kTest(GoSensor_CheckHealth(sensor)); 
        }
    }
    kFinally
    {
        kLock_Exit(obj->stateLock); 
        kEndFinally(); 
    }

    return kOK; 
}


GoFx(kBool) GoSystem_HasChanges(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kBool hasChanges = kFALSE; 
    kSize i; 

    kLock_Enter(obj->stateLock); 
    {
        if (kArrayList_Count(obj->onlineSensors) != kArrayList_Count(obj->allSensors))
        {
            hasChanges = kTRUE; 
        }
        else
        {
            for (i = 0; i < kArrayList_Count_(obj->allSensors); ++i)
            {
                GoSensor sensor = kArrayList_As_(obj->allSensors, i, GoSensor); 
                GoState state = GoSensor_KnownState(sensor);       // avoids communication

                if (GoState_ShouldRefresh(state))
                {
                    hasChanges = kTRUE; 
                    break;
                }
            }
        }
    }
    kLock_Exit(obj->stateLock); 

    return hasChanges; 
}

GoFx(kStatus) GoSystem_Refresh(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i = 0; 

    for (i = 0; i < kArrayList_Count(obj->allSensors); ++i)
    {
        kCheck(GoSensor_Refresh(kArrayList_As_(obj->allSensors, i, GoSensor))); 
    }
    
    kCheck(GoSystem_RefreshSensorList(system)); 

    return kOK; 
}

GoFx(kStatus) GoSystem_RefreshSensorList(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i = 0; 

    kLock_Enter(obj->stateLock); 

    kTry
    {        
        kTest(kArrayList_Clear(obj->onlineSensors)); 

        while (i < kArrayList_Count(obj->allSensors))
        {
            GoSensor sensor = kArrayList_As_(obj->allSensors, i, GoSensor); 
            GoState state = GoSensor_State(sensor);

            if (state == GO_STATE_OFFLINE)
            {
                kTest(kArrayList_Delete(obj->allSensors, i, kNULL)); 
                kTest(kObject_Destroy(sensor)); 
            }
            else
            {
                kTest(kArrayList_Add(obj->onlineSensors, &sensor)); 
                i++; 
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->stateLock); 
        kEndFinally(); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_Connect(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {       
        kCheck(GoSensor_Connect(kArrayList_As_(obj->onlineSensors, i, GoSensor))); 
    }
   
    return kOK; 
}

GoFx(kStatus) GoSystem_Disconnect(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {       
        kCheck(GoSensor_Disconnect(kArrayList_As_(obj->onlineSensors, i, GoSensor))); 
    }

    return kOK; 
}

GoFx(kVersion) GoSystem_ProtocolVersion()
{
    return GoSdk_ProtocolVersion(); 
}

GoFx(kVersion) GoSystem_SdkVersion()
{
    return GoSdk_Version(); 
}

GoFx(kStatus) GoSystem_Start(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY)
        {
            kCheck(GoSensor_BeginStart(sensor)); 
            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 

        kCheck(GoSensor_EndStart(sensor)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_ScheduledStart(GoSystem system, k64s value)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY)
        {
            kCheck(GoSensor_BeginScheduledStart(sensor, value)); 
            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 

        kCheck(GoSensor_EndScheduledStart(sensor)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_StartAlignment(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY)
        {
            kCheck(GoSensor_BeginAlign(sensor)); 
            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 

        kCheck(GoSensor_EndStart(sensor)); 
    }

    return kOK; 
}


GoFx(kStatus) GoSystem_StartExposureAutoSet(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY
            && GoSensor_Role(sensor) == GO_ROLE_MAIN)
        {
            kCheck(GoSensor_BeginExposureAutoSet(sensor, GO_ROLE_MAIN)); 

            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 

        kCheck(GoSensor_EndStart(sensor)); 
    }

    // Exposure auto set all buddy sensors
    kCheck(kArrayList_Clear(obj->tempList));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY
            && GoSensor_Role(sensor) == GO_ROLE_MAIN
            && GoSensor_HasBuddy(sensor))
        {
            kCheck(GoSensor_BeginExposureAutoSet(sensor, GO_ROLE_BUDDY)); 

            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 

        kCheck(GoSensor_EndStart(sensor)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_Stop(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_IsConnected(sensor)
            && GoSensor_IsResponsive(sensor))
        {
            kCheck(GoSensor_BeginStop(sensor)); 
            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 

        kCheck(GoSensor_EndStop(sensor)); 
    }
    
    return kOK; 
}

GoFx(kStatus) GoSystem_Reset(GoSystem system, kBool wait)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor) && (GoSensor_State(sensor) != GO_STATE_BUSY))
        {
            kCheck(GoSensor_Reset(sensor, kFALSE)); 
            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    if (wait)
    {
        kCheck(kTimer_Start(obj->timer, GO_SYSTEM_RESET_TIMEOUT)); 

        for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
        {
            GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor);                

            kCheck(GoSensor_WaitForReconnect(sensor, kTimer_Remaining(obj->timer))); 
        } 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_Cancel(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        kCheck(GoSensor_Cancel(kArrayList_As_(obj->onlineSensors, i, GoSensor))); 
    }

    return kOK; 
}

GoFx(kSize) GoSystem_SensorCount(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    return kArrayList_Count(obj->onlineSensors); 
}

GoFx(GoSensor) GoSystem_SensorAt(GoSystem system, kSize index)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kAssert(index < GoSystem_SensorCount(system));

    return kArrayList_As_(obj->onlineSensors, index, GoSensor); 
}

GoFx(kStatus) GoSystem_FindSensorById(GoSystem system, k32u id, GoSensor* sensor)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensorAt = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_Id(sensorAt) == id)
        {
            *sensor = sensorAt; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND;  
}

GoFx(kStatus) GoSystem_FindSensorByIpAddress(GoSystem system, const kIpAddress* address, GoSensor* sensor)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensorAt = kArrayList_As_(obj->onlineSensors, i, GoSensor); 
        GoAddressInfo addressInfo;
        
        kCheck(GoSensor_Address(sensorAt, &addressInfo));

        if (kIpAddress_Equals(addressInfo.address, *address))
        {
            *sensor = sensorAt; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND;  
}

GoFx(kStatus) GoSystem_FindSensorAll(GoSystem system, kSize id, GoSensor* sensor)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->allSensors); ++i)
    {
        GoSensor sensorAt = kArrayList_As_(obj->allSensors, i, GoSensor); 

        if (GoSensor_Id(sensorAt) == id)
        {
            *sensor = sensorAt; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND;  
}

GoFx(kStatus) GoSystem_Timestamp(GoSystem system, k64u* time)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 
        GoState state = GoSensor_State(sensor); 

        if (GoState_IsNormal(state))
        {
            return GoSensor_Timestamp(sensor, time);             
        }
    }

    return kERROR_NOT_FOUND; 
}

GoFx(kStatus) GoSystem_Encoder(GoSystem system, k64s* encoder)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 
        GoState state = GoSensor_State(sensor); 

        if (GoState_IsNormal(state))
        {
            return GoSensor_Encoder(sensor, encoder); 
        }
    }

    return kERROR_NOT_FOUND; 
}

GoFx(kStatus) GoSystem_LockState(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    return kLock_Enter(obj->stateLock); 
}

GoFx(kStatus) GoSystem_UnlockState(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    return kLock_Exit(obj->stateLock); 
}
 
GoFx(kStatus) GoSystem_SetDataHandler(GoSystem system, GoDataFx function, kPointer receiver)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    obj->dataQuit = kTRUE; 
    kCheck(kDestroyRef(&obj->dataThread)); 

    obj->dataQuit = kFALSE; 
    obj->onData.function = function; 
    obj->onData.receiver = receiver; 

    if (obj->onData.function)
    {
        kCheck(kThread_Construct(&obj->dataThread, kObject_Alloc_(system))); 
        kCheck(kThread_Start(obj->dataThread, GoSystem_DataThreadEntry, system)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_SetDataCapacity(GoSystem system, kSize capacity)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kCheckArgs(0 < capacity && capacity <= kSIZE_MAX);
    kCheck(kMsgQueue_SetCapacity(obj->dataQueue, kMSG_QUEUE_CAPACITY_TYPE_MEMORY, capacity)); 

    return kOK; 
}

GoFx(kSize) GoSystem_DataCapacity(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    return kMsgQueue_Capacity(obj->dataQueue); 
}

GoFx(kStatus) GoSystem_EnableData(GoSystem system, kBool enable)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i; 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 
        GoState state = GoSensor_State(sensor); 

        if (GoState_IsNormal(state))
        {
            kCheck(GoSensor_EnableData(sensor, enable)); 
        }
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_ClearData(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kObject data = kNULL; 
    kSize i; 

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0)); 

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->onlineSensors, i, GoSensor); 

        if (GoSensor_DataEnabled(sensor))
        {
            kCheck(GoSensor_EnableData(sensor, kFALSE)); 
            kCheck(kArrayList_Add(obj->tempList, &sensor)); 
        }
    }

    while (kSuccess(kMsgQueue_Remove(obj->dataQueue, &data, 0)))
    {
        kCheck(kObject_Dispose(data)); 
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_As_(obj->tempList, i, GoSensor); 
        kCheck(GoSensor_EnableData(sensor, kTRUE)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_ReceiveData(GoSystem system, GoDataSet* data, k64u timeout)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    
    return kMsgQueue_Remove(obj->dataQueue, data, timeout); 
}

GoFx(kStatus) GoSystem_DataThreadEntry(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    GoDataSet data = kNULL; 
    kStatus status;    
    
    while (!obj->dataQuit)
    {
        if (kSuccess(status = kMsgQueue_Remove(obj->dataQueue, &data, GO_SYSTEM_QUIT_QUERY_INTERVAL)))
        {
            kCheck(obj->onData.function(obj->onData.receiver, system, data)); 
        }
        else if (status != kERROR_TIMEOUT)
        {
            kCheck(status); 
        }
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_OnData(GoSystem system, GoSensor sensor, GoDataSet data)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kCheck(kMsgQueue_Add(obj->dataQueue, &data)); 
    
    return kOK; 
}

GoFx(kStatus) GoSystem_SetHealthHandler(GoSystem system, GoDataFx function, kPointer receiver)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    obj->healthQuit = kTRUE; 
    kCheck(kDestroyRef(&obj->healthThread)); 

    obj->healthQuit = kFALSE; 
    obj->onHealth.function = function; 
    obj->onHealth.receiver = receiver; 

    if (obj->onHealth.function)
    {
        kCheck(kThread_Construct(&obj->healthThread, kObject_Alloc_(system))); 
        kCheck(kThread_Start(obj->healthThread, GoSystem_HealthThreadEntry, system)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_SetHealthCapacity(GoSystem system, kSize capacity)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kCheck(kMsgQueue_SetCapacity(obj->healthQueue, kMSG_QUEUE_CAPACITY_TYPE_MEMORY, capacity)); 

    return kOK; 
}

GoFx(kSize) GoSystem_HealthCapacity(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    return kMsgQueue_Capacity(obj->healthQueue); 
}

GoFx(kStatus) GoSystem_ReceiveHealth(GoSystem system, GoDataSet* health, k64u timeout)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    
    return kMsgQueue_Remove(obj->healthQueue, health, timeout); 
}

GoFx(kStatus) GoSystem_ClearHealth(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kObject health = kNULL; 

    while (kSuccess(kMsgQueue_Remove(obj->healthQueue, &health, 0)))
    {
        kCheck(kObject_Dispose(health)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_HealthThreadEntry(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    GoDataSet health = kNULL; 
    kStatus status; 
    
    while (!obj->healthQuit)
    {
        if (kSuccess(status = kMsgQueue_Remove(obj->healthQueue, &health, GO_SYSTEM_QUIT_QUERY_INTERVAL)))
        {
            kCheck(obj->onHealth.function(obj->onHealth.receiver, system, health)); 
        }
        else if (status != kERROR_TIMEOUT)
        {
            kCheck(status); 
        }
    }

    return kOK; 
}

GoFx(kStatus) GoSystem_OnHealth(GoSystem system, GoSensor sensor, GoDataSet health)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kCheck(kMsgQueue_Add(obj->healthQueue, &health)); 
    
    return kOK; 
}

GoFx(kSize) GoSystem_MultiplexBankCount(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    return kArrayList_Count(obj->bankList);
}

GoFx(GoMultiplexBank) GoSystem_MultiplexBankAt(GoSystem system, kSize index)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 

    kAssert(index < kArrayList_Count(obj->bankList));

    return kArrayList_As_(obj->bankList, index, GoMultiplexBank);
}

GoFx(kStatus) GoSystem_AddMultiplexBank(GoSystem system, GoMultiplexBank* bank)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    GoMultiplexBank newBank = kNULL;
    kStatus exception = kOK;
    k32u nextId = 0;
    k32u maxId = 0;
    k32u currentItemId;
    kBool unusedIdFound = kFALSE;
    kSize i;

    kTry
    {
        for (i = 0; i < kArrayList_Count(obj->bankList); i++)
        {
            currentItemId = GoMultiplexBank_Id(kArrayList_As_(obj->bankList, i, GoMultiplexBank));

            if (currentItemId > maxId)
            {
                maxId = currentItemId;
            }
        }

        nextId = maxId + 1;

        kTest(GoMultiplexBank_Construct(&newBank, nextId, kObject_Alloc(system)));
        kTest(kArrayList_Add(obj->bankList, &newBank));
    }
    kCatch(&exception)
    {
        kDestroyRef(&newBank);
        kEndCatch(exception);
    }
    
    if (!kIsNull(bank))
    {
        *bank = newBank;
    }

    return kOK;
}

GoFx(kStatus) GoSystem_RemoveMultiplexBank(GoSystem system, kSize index)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    GoMultiplexBank bankToDestroy = kNULL;

    kCheckArgs(index < kArrayList_Count(obj->bankList));

    kCheck(kArrayList_Remove(obj->bankList, index, &bankToDestroy));
    kDestroyRef(&bankToDestroy);

    return kOK;
}

GoFx(kStatus) GoSystem_ClearMultiplexBanks(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    GoMultiplexBank bank = kNULL;

    while (kArrayList_Count(obj->bankList) != 0)
    {
        kCheck(kArrayList_Remove(obj->bankList, 0, &bank));
        kCheck(kDestroyRef(&bank)); //sensor objects within the bank are not destroyed
    }
    
    return kOK;
}

GoFx(kStatus) GoSystem_UpdateMultiplexDelay(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i, j;
    k64f exposureDuration = 0.0;
    kBool tempConnection = kFALSE;

    for (i = 0; i < kArrayList_Count(obj->bankList); i++)
    {
        GoMultiplexBank currentBank = kArrayList_As_(obj->bankList, i, GoMultiplexBank);
        k64f currentExposureDuration;

        for (j = 0; j < GoMultiplexBank_SensorCount(currentBank); j++)
        {
            GoSensor sensor = GoMultiplexBank_SensorAt(currentBank, j);
            GoLayout layout = kNULL;

            if (!GoSensor_IsConnected(sensor))
            {
                kCheck(GoSensor_Connect(sensor));
                tempConnection = kTRUE;
            }

            layout = GoSetup_Layout(GoSensor_Setup(sensor));
            
            kCheck(GoLayout_EnableMultiplexSingle(layout, kTRUE));
            kCheck(GoLayout_SetMultiplexSingleDelay(layout, exposureDuration));

            if (tempConnection)
            {
                kCheck(GoSensor_Disconnect(sensor));
                tempConnection = kFALSE;
            }
        }

        currentExposureDuration = GoSystem_MaxBankExposureDuration(system, currentBank);
        exposureDuration += currentExposureDuration;
    }

    return kOK;
}

GoFx(k64f) GoSystem_MaxMinimumMultiplexPeriod(GoSystem system)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i, j;
    kBool tempConnection = kFALSE;
    k64f maxPeriod = 0.0;

    for (i = 0; i < kArrayList_Count(obj->bankList); i++)
    {
        GoMultiplexBank currentBank = kArrayList_As_(obj->bankList, i, GoMultiplexBank);
        k64f periodMin;

        for (j = 0; j < GoMultiplexBank_SensorCount(currentBank); j++)
        {
            GoSensor sensor = GoMultiplexBank_SensorAt(currentBank, j);
            GoLayout layout = kNULL;

            if (!GoSensor_IsConnected(sensor))
            {
                kCheck(GoSensor_Connect(sensor));
                tempConnection = kTRUE;
            }

            layout = GoSetup_Layout(GoSensor_Setup(sensor));

            periodMin = GoLayout_MultiplexSinglePeriodMin(layout);

            if (tempConnection)
            {
                kCheck(GoSensor_Disconnect(sensor));
                tempConnection = kFALSE;
            }

            if (periodMin > maxPeriod)
            {
                maxPeriod = periodMin;
            }
        }        
    }

    return maxPeriod; 
}

GoFx(kStatus) GoSystem_UpdateMultiplexPeriod(GoSystem system, k64f period)
{
    GoSystemClass* obj = GoSystem_Cast_(system); 
    kSize i, j;
    k64f exposureDuration = 0.0;
    kBool tempConnection = kFALSE;
    k64f maxPeriod = GoSystem_MaxMinimumMultiplexPeriod(system);

    if (period == 0.0)
    {
        period = maxPeriod;    
    }
    else
    {
        if (period < maxPeriod)
        {
            return kERROR_PARAMETER;
        }
    }

    for (i = 0; i < kArrayList_Count(obj->bankList); i++)
    {
        GoMultiplexBank currentBank = kArrayList_As_(obj->bankList, i, GoMultiplexBank);
        k64f periodMin;

        for (j = 0; j < GoMultiplexBank_SensorCount(currentBank); j++)
        {
            GoSensor sensor = GoMultiplexBank_SensorAt(currentBank, j);
            GoLayout layout = kNULL;

            if (!GoSensor_IsConnected(sensor))
            {
                kCheck(GoSensor_Connect(sensor));
                tempConnection = kTRUE;
            }

            layout = GoSetup_Layout(GoSensor_Setup(sensor));

            periodMin = GoLayout_SetMultiplexSinglePeriod(layout, period);

            if (tempConnection)
            {
                kCheck(GoSensor_Disconnect(sensor));
                tempConnection = kFALSE;
            }

            if (periodMin > maxPeriod)
            {
                maxPeriod = periodMin;
            }
        }        
    }

    return kOK;
}

GoFx(k64f) GoSystem_MaxBankExposureDuration(GoSystem system, GoMultiplexBank bank)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 
    k64f maxExposureDuration = 0.0;
    k64f exposureDurationToCompare = 0.0;
    kBool tempConnection = kFALSE;    
    kSize i;
    
    for (i = 0; i < GoMultiplexBank_SensorCount(bank); i++)
    {
        GoSensor sensor = GoMultiplexBank_SensorAt(bank, i);
        GoSetup setup = kNULL;

        if (!GoSensor_IsConnected(sensor))
        {
            kCheck(GoSensor_Connect(sensor));
            tempConnection = kTRUE;
        }

        setup = GoSensor_Setup(sensor);

        exposureDurationToCompare = GoLayout_MultiplexSingleExposureDuration(GoSetup_Layout(setup));

        if (maxExposureDuration < exposureDurationToCompare)
        {
            maxExposureDuration = exposureDurationToCompare;
        }
        
        if (tempConnection)
        {
            kCheck(GoSensor_Disconnect(sensor));
            tempConnection = kFALSE;
        }
    }

    return maxExposureDuration;
}

GoFx(kStatus) GoSystem_UpdateAllMultiplexParameters(GoSystem system, k64f period)
{
    kCheck(GoSystem_UpdateMultiplexDelay(system));
    kCheck(GoSystem_UpdateMultiplexPeriod(system, period));

    return kOK;
}
