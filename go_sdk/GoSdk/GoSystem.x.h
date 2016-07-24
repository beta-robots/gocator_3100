/** 
 * @file    GoSystem.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SYSTEM_X_H
#define GO_SDK_SYSTEM_X_H

#include <GoSdk/GoSystem.h>
#include <GoSdk/Internal/GoDiscovery.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kMsgQueue.h>
#include <kApi/Threads/kPeriodic.h>
kBeginHeader()

#define GO_SYSTEM_DISCOVERY_PERIOD                  (2000000)           //period of background discovery check (us)
#define GO_SYSTEM_HEALTH_CHECK_PERIOD               (2000000)           //period of background health check (us)

#define GO_SYSTEM_DEFAULT_DATA_CAPACITY             (50000000)          //default capacity of data message queue (bytes)
#define GO_SYSTEM_DEFAULT_HEALTH_CAPACITY           (1000000)           //default capacity of health message queue (bytes)

#define GO_SYSTEM_QUIT_QUERY_INTERVAL               (100000)            //duration that most threads can block before checking quit flag (us)

#define GO_SYSTEM_RESET_HOLD_INTERVAL               (10000000)          //time that sensor remains in "resetting" state (us)
#define GO_SYSTEM_RESET_QUERY_INTERVAL              (2000000)           //duration between status checks during reset/wait (us)
#define GO_SYSTEM_RESET_TIMEOUT                     (90000000)          //total timeout for reset operation (us)
#define GO_SYSTEM_RESET_INCOMPLETE_TIMEOUT          (15000000)          //timeout for incomplete status to resolve after reset reconnection (us)

typedef struct GoSystemClass
{
    kObjectClass base; 
    
    kLock stateLock;                            //protects any state accessed from multiple threads
    kTimer timer;                               //utilty timer (resets, etc.)

    GoDiscovery discovery;                      //periodically discovers sensors in background thread 
    kPeriodic healthCheck;                      //periodically verifies continuity of health messages in background thread 
    
    kArrayList allSensors;                      //list of all sensors (kArrayList<GoSensor>)
    kArrayList onlineSensors;                   //list of all published sensors (kArrayList<GoSensor>)
    kArrayList tempList;                        //temp list used in some methods

    volatile kBool dataQuit;                    //flag to exit data dispatch thread
    kMsgQueue dataQueue;                        //queue of received data messages
    kThread dataThread;                         //data dispatch thread
    kCallback onData;                           //data callback

    volatile kBool healthQuit;                  //flag to exit health dispatch thread
    kMsgQueue healthQueue;                      //queue of received health messages
    kThread healthThread;                       //health dispatch thread
    kCallback onHealth;                         //health callback

    kArrayList bankList;                        //list of all multiplexing banks 
} GoSystemClass; 

kDeclareClass(Go, GoSystem, kObject)

#define GoSystem_Cast_(CONTEXT)    kCastClass_(GoSystem, CONTEXT)

GoFx(kStatus) GoSystem_Init(GoSystem system, kType type, kAlloc alloc);
GoFx(kStatus) GoSystem_VRelease(GoSystem system);

GoFx(kStatus) GoSystem_OnDiscovery(GoSystem system, GoDiscovery discovery, kArrayList list); 
GoFx(kStatus) GoSystem_OnHealthCheck(GoSystem system, kPeriodic timer); 

GoFx(kStatus) GoSystem_RefreshSensorList(GoSystem system); 

GoFx(kStatus) GoSystem_FindSensorAll(GoSystem system, kSize id, GoSensor* sensor); 

GoFx(kStatus) GoSystem_StartMultiplex(GoSystem system);
GoFx(kStatus) GoSystem_StartMultiplexAlignment(GoSystem system);
GoFx(kStatus) GoSystem_StartMultiplexExposureAutoSet(GoSystem system);
GoFx(kStatus) GoSystem_StopMultiplex(GoSystem system);

GoFx(kStatus) GoSystem_LockState(GoSystem system); 
GoFx(kStatus) GoSystem_UnlockState(GoSystem system); 

GoFx(kStatus) GoSystem_DataThreadEntry(GoSystem system); 
GoFx(kStatus) GoSystem_OnData(GoSystem system, GoSensor sensor, GoDataSet data); 

GoFx(kStatus) GoSystem_SetHealthCapacity(GoSystem system, kSize bytes);
GoFx(kSize) GoSystem_HealthCapacity(GoSystem system);
GoFx(kStatus) GoSystem_HealthThreadEntry(GoSystem system); 
GoFx(kStatus) GoSystem_OnHealth(GoSystem system, GoSensor sensor, GoDataSet health); 

kEndHeader()

#endif