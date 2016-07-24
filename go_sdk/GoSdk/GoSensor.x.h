/** 
 * @file    GoSensor.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SENSOR_X_H
#define GO_SDK_SENSOR_X_H

#include <GoSdk/Internal/GoControl.h>
#include <GoSdk/Internal/GoDiscovery.h>
#include <GoSdk/Internal/GoReceiver.h>
#include <GoSdk/Tools/GoTools.h>
#include <GoSdk/GoTransform.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Data/kArrayList.h>
kBeginHeader()

#define GO_SENSOR_CONFIG_SCHEMA_VERSION             (101)
#define GO_SENSOR_LIVE_CONFIG_NAME                  "_live.cfg"

#define GO_SENSOR_TRANSFORM_SCHEMA_VERSION          (100)
#define GO_SENSOR_LIVE_TRANSFORM_NAME               "_live.tfm"

#define GO_SENSOR_JOB_VERSION                       (101)

#define GO_SENSOR_CONSISTENCY_INTERVAL              (4000000)           //hold time, after foreground info is updated, 
                                                                        //before consistency with background info is checked (us)

#define GO_SENSOR_UPGRADE_TIMEOUT                   (120000000)         //timeout for sensor upgrade operations (us)

#define GO_SENSOR_DATA_SOCKET_BUFFER                (65536)             //socket read buffer size for data connections
#define GO_SENSOR_DATA_STREAM_BUFFER                (65536)             //client read buffer size for data connections

#define GO_SENSOR_HEALTH_SOCKET_BUFFER              (16384)             //socket read buffer size for health connections
#define GO_SENSOR_HEALTH_STREAM_BUFFER              (16384)             //client read buffer size for health connections

#define GO_SENSOR_DATA_PORT                         (3196)              //gocator data port number
#define GO_SENSOR_HEALTH_PORT                       (3194)              //gocator health port number

#define GO_SENSOR_DISCOVERY_CHECK_COUNT             (3)                 //number of failed discovery checks before sensor considered absent
#define GO_SENSOR_HEALTH_CHECK_COUNT                (3)                 //number of failed health checks before sensor considered absent

typedef struct GoSensorClass
{
    kObjectClass base; 

    kObject system;                             //system object (parent)

    k32u deviceId;                              //serial number
    kText32 model;                              //sensor model
    kVersion firmwareVersion;                   //firmware version
    kVersion protocolVersion;                   //remote protocol version

    //begin synchronized 
    //(system state lock)
    kBool discoveryHistory[8];                  //discovery presence history (^2)
    k64u discoveryCount;                        //discovery presence history count 
    kBool healthCheckEnabled;                   //is health checking currently enabled?
    kBool healthHistory[8];                     //health presence history (^2)
    k64u healthCheckCount;                      //health presence history count 
    k64u healthMsgCount;                        //count of all health messages received from sensor 
    k64u previousHealthMsgCount;                //count of all health messages at last health check 
    k64u sensorInfoTime;                        //time of last foreground update of sensor info
    GoAddressInfo address;                      //current network address
    GoRole role;                                //sensor role, as reported via sensor info
    k32u buddyId;                               //buddy identifier, as reported via sensor info
    GoAlignmentState alignmentState;            //alignment state 
    GoState runState;                           //sensor state, as reported by sensor
    GoUser user;                                //current signed-in user 
    kBool isResetting;                          //is the sensor currently resetting?
    kBool isCancelled;                          //was i/o cancelled by user?
    kBool isConnected;                          //is sensor currently connected?
    kBool isCompatible;                         //is sensor protocol compatible with client protocol?
    //end synchronized

    kArrayList fileList;                        //list of sensor file names (kArrayList<kText64>)
    kBool fileListValid;                        //is file list valid?
    kArrayList directoryList;                   //list of directory file names (kArrayList<kText64>)
    kBool isDirectoryListValid;                 //is the current directory list valid?

    kArrayList partModelList;                   //list of part model configurations in the live job (of type GoPartModel)
    kBool isSyncPartModels;                     //is the part model list currently being synchronized?

    kXml configXml;                             //the last config retrieved. Used for forwards compatible config writes.
    kXml configXmlItem;                         //node reference from the last config retrieved.
    kBool configValid;                          //is config valid?
    kBool configModified;                       //has config been locally modified?
    kBool isSyncConfig;                         //is the config currently being synchronized?

    kXml transformXml;                          //the last transform retrieved. Used for forwards compatible transform writes.
    kXml transformXmlItem;                      //node reference from the last transform retrieved.
    kBool transformValid;                       //is transform valid?
    kBool transformModified;                    //has transform been locally modified?
    kBool isSyncTransform;                      //is the transform currently being synchronized?

    kTimer timer;                               //utility timer (upgrades, etc.)
    GoSensorInfo localSensorInfo;               //temp variable, used during info read
    kArrayList remoteSensorInfo;                //of type GoSensorInfo - represents visible remote sensor data
    kBool infoValid;                            //is info valid?

    kPeriodic resetTimer;                       //provides a timed event to end reset state

    GoControl control;                          //control/upgrade connection
    GoReceiver data;                            //data connection
    GoSensorDataSetFx onDataSet;                //callback to a custom data handling function
    kPointer onDataSetContext;                  //context to be passed into the onDataSet function
    GoReceiver health;                          //health connection

    GoSetup setup;                              //setup module
    GoTools tools;                              //tools module
    GoOutput output;                            //output module
    GoTransform transform;                      //transformation module
} GoSensorClass; 

kDeclareClass(Go, GoSensor, kObject)

#define GoSensor_Cast_(CONTEXT)    kCastClass_(GoSensor, CONTEXT)

//system - a GoSystem handle
GoFx(kStatus) GoSensor_Construct(GoSensor* sensor, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc allocator);

GoFx(kStatus) GoSensor_Init(GoSensor sensor, kType type, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc alloc);
GoFx(kStatus) GoSensor_VRelease(GoSensor sensor);

GoFx(kStatus) GoSensor_SyncConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_SetConnected(GoSensor sensor, kBool isConnected, kBool isCompatible); 

GoFx(kStatus) GoSensor_Refresh(GoSensor sensor); 

GoFx(kStatus) GoSensor_Invalidate(GoSensor sensor); 

GoFx(kStatus) GoSensor_CacheConfig(GoSensor sensor); 
GoFx(kStatus) GoSensor_FlushConfig(GoSensor sensor); 
GoFx(kStatus) GoSensor_InvalidateConfig(GoSensor sensor); 
GoFx(kBool) GoSensor_ConfigValid(GoSensor sensor); 
GoFx(kStatus) GoSensor_SetConfigModified(GoSensor sensor); 
GoFx(kBool) GoSensor_ConfigModified(GoSensor sensor); 
GoFx(kStatus) GoSensor_ReadConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_WriteConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_GetLiveConfig(GoSensor sensor, kXml* xml, kAlloc allocator); 
GoFx(kStatus) GoSensor_SetLiveConfig(GoSensor sensor, kXml xml); 

GoFx(kStatus) GoSensor_SyncTransform(GoSensor sensor); 
GoFx(kStatus) GoSensor_CacheTransform(GoSensor sensor); 
GoFx(kStatus) GoSensor_FlushTransform(GoSensor sensor); 
GoFx(kStatus) GoSensor_InvalidateTransform(GoSensor sensor); 
GoFx(kBool) GoSensor_TransformValid(GoSensor sensor); 
GoFx(kStatus) GoSensor_SetTransformModified(GoSensor sensor); 
GoFx(kBool) GoSensor_TransformModified(GoSensor sensor); 
GoFx(kStatus) GoSensor_ReadTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_WriteTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_GetLiveTransform(GoSensor sensor, kXml* xml, kAlloc allocator); 
GoFx(kStatus) GoSensor_SetLiveTransform(GoSensor sensor, kXml xml); 

GoFx(kStatus) GoSensor_SyncPartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidatePartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_CachePartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_FlushPartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadPartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadPartModel(GoSensor sensor, GoPartModel model);
GoFx(kStatus) GoSensor_WritePartModel(GoSensor sensor, GoPartModel model);

GoFx(kStatus) GoSensor_CacheFileList(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadFileList(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidateFileList(GoSensor sensor);
GoFx(kBool) GoSensor_FileListValid(GoSensor sensor);

GoFx(kStatus) GoSensor_CacheDirectoryList(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive);
GoFx(kStatus) GoSensor_ListDirectory(GoSensor sensor, const kChar* extension, const kChar* root, kBool isRecursive, kArrayList fileList);
GoFx(kStatus) GoSensor_InvalidateDirectoryList(GoSensor sensor);
GoFx(kBool) GoSensor_DirectoryListValid(GoSensor sensor);

GoFx(kStatus) GoSensor_CacheInfo(GoSensor sensor); 
GoFx(kStatus) GoSensor_ReadInfo(GoSensor sensor);
GoFx(kBool) GoSensor_InfoValid(GoSensor sensor); 
GoFx(kStatus) GoSensor_InvalidateInfo(GoSensor sensor); 

GoFx(kStatus) GoSensor_InvalidateRole(GoSensor sensor); 

GoFx(GoState) GoSensor_KnownState(GoSensor sensor); 
GoFx(kBool) GoSensor_IsResponsive(GoSensor sensor); 
GoFx(kBool) GoSensor_IsReadable(GoSensor sensor); 
GoFx(kBool) GoSensor_IsConfigurable(GoSensor sensor); 
GoFx(kBool) GoSensor_IsReady(GoSensor sensor); 
GoFx(kBool) GoSensor_IsRunning(GoSensor sensor); 
GoFx(kBool) GoSensor_IsNormal(GoSensor sensor); 
GoFx(kBool) GoSensor_IsCancelled(GoSensor sensor); 
GoFx(kBool) GoSensor_ConsistencyIntervalElapsed(GoSensor sensor);

GoFx(kBool) GoSensor_DataEnabled(GoSensor sensor); 

GoFx(kStatus) GoSensor_EnableHealth(GoSensor sensor, kBool enable); 
GoFx(kStatus) GoSensor_CheckHealth(GoSensor sensor); 
GoFx(kStatus) GoSensor_UpdateHealthInfo(GoSensor sensor, GoDataSet healthSet); 

GoFx(kBool) GoSensor_IsInconsistent(GoSensor sensor); 
GoFx(kBool) GoSensor_IsDiscoveryOnline(GoSensor sensor); 
GoFx(kBool) GoSensor_IsHealthOnline(GoSensor sensor); 

GoFx(kStatus) GoSensor_OnCancelQuery(GoSensor sensor, kObject sender, kPointer args); 

GoFx(kStatus) GoSensor_LockState(GoSensor sensor); 
GoFx(kStatus) GoSensor_UnlockState(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginReset(GoSensor sensor); 
GoFx(kStatus) GoSensor_OnResetHoldComplete(GoSensor sensor, kPeriodic timer); 

GoFx(kStatus) GoSensor_WaitForReboot(GoSensor sensor, k64u timeout); 
GoFx(kStatus) GoSensor_WaitForReconnect(GoSensor sensor, k64u timeout); 

GoFx(kStatus) GoSensor_BeginStart(GoSensor sensor); 
GoFx(kStatus) GoSensor_EndStart(GoSensor sensor); 

GoFx(kStatus) GoSensor_BeginScheduledStart(GoSensor sensor, k64s value);
GoFx(kStatus) GoSensor_EndScheduledStart(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginStop(GoSensor sensor); 
GoFx(kStatus) GoSensor_EndStop(GoSensor sensor); 

GoFx(kStatus) GoSensor_BeginAlign(GoSensor sensor); 
GoFx(kStatus) GoSensor_EndAlign(GoSensor sensor); 

GoFx(kStatus) GoSensor_BeginExposureAutoSet(GoSensor sensor, GoRole role); 
GoFx(kStatus) GoSensor_EndExposureAutoSet(GoSensor sensor); 

GoFx(kStatus) GoSensor_OnData(GoSensor sensor, GoReceiver receiver, kSerializer reader);
GoFx(kStatus) GoSensor_OnHealth(GoSensor sensor, GoReceiver receiver, kSerializer reader);

GoFx(kStatus) GoSensor_AddTool(GoSensor sensor, const kChar* type, const kChar* name);
GoFx(kStatus) GoSensor_AddMeasurement(GoSensor sensor, kSize index, const kChar* type);

kEndHeader()

#endif
