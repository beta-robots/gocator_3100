/** 
 * @file    GoControl.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_CONTROL_X_H
#define GO_SDK_CONTROL_X_H

#include <GoSdk/Internal/GoControl.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Io/kTcpClient.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Threads/kTimer.h>
kBeginHeader()

#define GO_CONTROL_PORT_CONTROL                     (3190)
#define GO_CONTROL_PORT_UPGRADE                     (3192)

#define GO_CONTROL_CHANNEL_CONTROL                  (0)
#define GO_CONTROL_CHANNEL_UPGRADE                  (1)
#define GO_CONTROL_CHANNEL_COUNT                    (2)

#define GO_CONTROL_CONNECT_TIMEOUT                  (1000000)
#define GO_CONTROL_CANCEL_QUERY_INTERVAL            (100000)

#define GO_CONTROL_BEGIN_UPGRADE                    (0x0000)
#define GO_CONTROL_BEGIN_UPGRADE_TIMEOUT            (40000000)

#define GO_CONTROL_GET_UPGRADE_STATUS               (0x0001)
#define GO_CONTROL_GET_UPGRADE_STATUS_TIMEOUT       (3000000)

#define GO_CONTROL_GET_UPGRADE_LOG                  (0x0002)
#define GO_CONTROL_GET_UPGRADE_LOG_TIMEOUT          (15000000)

#define GO_CONTROL_START                            (0x100D)
#define GO_CONTROL_START_TIMEOUT                    (15000000)

#define GO_CONTROL_SCHEDULED_START                  (0x100F)
#define GO_CONTROL_SCHEDULED_START_TIMEOUT          (15000000)

#define GO_CONTROL_START_ALIGNMENT                  (0x4600)
#define GO_CONTROL_START_ALIGNMENT_TIMEOUT          (15000000)

#define GO_CONTROL_CLEAR_ALIGNMENT                  (0x4102)
#define GO_CONTROL_CLEAR_ALIGNMENT_TIMEOUT          (3000000)

#define GO_CONTROL_START_EXPOSURE_AUTO_SET          (0x4601)
#define GO_CONTROL_START_EXPOSURE_AUTO_SET_TIMEOUT  (5000000)

#define GO_CONTROL_STOP                             (0x1001)
#define GO_CONTROL_STOP_TIMEOUT                     (15000000)

#define GO_CONTROL_GET_STATES                       (0x4525)
#define GO_CONTROL_GET_STATES_TIMEOUT               (3000000)

#define GO_CONTROL_GET_MODE                         (0x1005)
#define GO_CONTROL_GET_MODE_TIMEOUT                 (3000000)

#define GO_CONTROL_SET_ALIGNMENT_REFERENCE          (0x4103)
#define GO_CONTROL_SET_ALIGNMENT_REFERENCE_TIMEOUT  (3000000)

#define GO_CONTROL_GET_ALIGNMENT_REFERENCE          (0x4104)
#define GO_CONTROL_GET_ALIGNMENT_REFERENCE_TIMEOUT  (3000000)

#define GO_CONTROL_GET_FILE_LIST                    (0x101A)
#define GO_CONTROL_GET_FILE_LIST_TIMEOUT            (3000000)

#define GO_CONTROL_WRITE_FILE                       (0x1006)
#define GO_CONTROL_WRITE_FILE_TIMEOUT               (15000000)

#define GO_CONTROL_READ_FILE                        (0x1007)
#define GO_CONTROL_READ_FILE_TIMEOUT                (15000000)

#define GO_CONTROL_CLEAR_LOG                        (0x101D)

#define GO_CONTROL_COPY_FILE                        (0x101B)
#define GO_CONTROL_COPY_FILE_TIMEOUT                (15000000)

#define GO_CONTROL_DELETE_FILE                      (0x1008)
#define GO_CONTROL_DELETE_FILE_TIMEOUT              (15000000)

#define GO_CONTROL_BACKUP                           (0x1013)
#define GO_CONTROL_BACKUP_TIMEOUT                   (15000000)

#define GO_CONTROL_RESTORE                          (0x1014)
#define GO_CONTROL_RESTORE_TIMEOUT                  (60000000)

#define GO_CONTROL_PING                             (0x100E)
#define GO_CONTROL_PING_TIMEOUT                     (1000000)

#define GO_CONTROL_GET_TIME_STAMP                   (0x100A)
#define GO_CONTROL_GET_TIME_STAMP_TIMEOUT           (1000000)

#define GO_CONTROL_GET_ENCODER                      (0x101C)
#define GO_CONTROL_GET_ENCODER_TIMEOUT              (1000000)

#define GO_CONTROL_RESET_ENCODER                    (0x101E)
#define GO_CONTROL_RESET_ENCODER_TIMEOUT            (1000000)

#define GO_CONTROL_GET_VERSION                      (0x4511)
#define GO_CONTROL_GET_VERSION_TIMEOUT              (1000000)

#define GO_CONTROL_LOG_IN                           (0x4003)
#define GO_CONTROL_LOG_IN_TIMEOUT                   (500000)

#define GO_CONTROL_CHANGE_PASSWORD                  (0x4004)
#define GO_CONTROL_CHANGE_PASSWORD_TIMEOUT          (3000000)

#define GO_CONTROL_CHANGE_BUDDY                     (0x4005)
#define GO_CONTROL_CHANGE_BUDDY_TIMEOUT             (10000000)

#define GO_CONTROL_GET_SYSTEM_INFO                  (0x4002)
#define GO_CONTROL_GET_SYSTEM_INFO_TIMEOUT          (2000000)

#define GO_CONTROL_GET_DEFAULT_JOB                  (0x4100)
#define GO_CONTROL_GET_DEFAULT_JOB_TIMEOUT          (1000000)

#define GO_CONTROL_SET_DEFAULT_JOB                  (0x4101)
#define GO_CONTROL_SET_DEFAULT_JOB_TIMEOUT          (15000000)

#define GO_CONTROL_GET_LOADED_JOB                   (0x4512)
#define GO_CONTROL_GET_LOADED_FILE_TIMEOUT          (1000000)

#define GO_CONTROL_TRIGGER                          (0x4510)
#define GO_CONTROL_TRIGGER_TIMEOUT                  (1000000)

#define GO_CONTROL_RESET                            (0x4300)
#define GO_CONTROL_RESET_TIMEOUT                    (3000000)

#define GO_CONTROL_RESTORE_FACTORY                  (0x4301)
#define GO_CONTROL_RESTORE_FACTORY_TIMEOUT          (15000000)

#define GO_CONTROL_SCHEDULE_DIGITAL                 (0x4518)
#define GO_CONTROL_SCHEDULE_DIGITAL_TIMEOUT         (1000000)

#define GO_CONTROL_SCHEDULE_ANALOG                  (0x4519)
#define GO_CONTROL_SCHEDULE_ANALOG_TIMEOUT          (1000000)

#define GO_CONTROL_DEFAULT_TIMEOUT                  (1000000)

#define GO_CONTROL_SET_RECORDING_ENABLED            (0x4516)
#define GO_CONTROL_GET_RECORDING_ENABLED            (0x4517)

#define GO_CONTROL_SET_INPUT_SOURCE                 (0x4523)
#define GO_CONTROL_SET_INPUT_SOURCE_TIMEOUT         (3000000)

#define GO_CONTROL_GET_INPUT_SOURCE                 (0x4524)
#define GO_CONTROL_GET_INPUT_SOURCE_TIMEOUT         (3000000)

#define GO_CONTROL_CLEAR_REPLAY_DATA                (0x4513)
#define GO_CONTROL_PLAYBACK_SEEK                    (0x4503)
#define GO_CONTROL_STEP_PLAYBACK                    (0x4501)

#define GO_CONTROL_PLAYBACK_POSITION                (0x4502)
#define GO_CONTROL_PLAYBACK_POSITION_TIMEOUT        (3000000)

#define GO_CONTROL_SIMULATE                         (0x4522)
#define GO_CONTROL_PLAY                             (0x4521)

#define GO_CONTROL_EXPORT_CSV                       (0x4507)
#define GO_CONTROL_EXPORT_CSV_TIMEOUT               (kINFINITE)

#define GO_CONTROL_READ_FILE_STREAMED               (0x4529)
#define GO_CONTROL_READ_FILE_STREAMED_TIMEOUT       (kINFINITE)

#define GO_CONTROL_EXPORT_BITMAP                    (0x4508)
#define GO_CONTROL_EXPORT_BITMAP_TIMEOUT            (kINFINITE)

#define GO_CONTROL_CONTINUE                         (0x5000)
#define GO_CONTROL_CLEAR_MEASUREMENT_STATS          (0x4526)

#define GO_CONTROL_SET_AUTOSTART_ENABLED            (0x452B)
#define GO_CONTROL_SET_AUTOSTART_ENABLED_TIMEOUT    (1000000)

#define GO_CONTROL_GET_AUTOSTART_ENABLED            (0x452C)
#define GO_CONTROL_GET_AUTOSTART_ENABLED_TIMEOUT    (1000000)

#define GO_CONTROL_CREATE_MODEL                     (0x4602)
#define GO_CONTROL_CREATE_MODEL_TIMEOUT             (5000000)
#define GO_CONTROL_SEND_MODEL                       (0x4603)
#define GO_CONTROL_SEND_MODEL_TIMEOUT               (1000000)
#define GO_CONTROL_DETECT_MODEL_EDGES               (0x4604)
#define GO_CONTROL_DETECT_MODEL_EDGES_TIMEOUT       (1000000)

#define GO_CONTROL_SET_IS_EMULATOR                  (0x452F)
#define GO_CONTROL_SET_IS_EMULATOR_TIMEOUT          (1000000)

#define GO_CONTROL_SET_REPLAY_PROTECTION_ENABLED            (0x452D)
#define GO_CONTROL_SET_REPLAY_PROTECTION_ENABLED_TIMEOUT    (1000000)

#define GO_CONTROL_GET_REPLAY_PROTECTION_ENABLED            (0x452E)
#define GO_CONTROL_GET_REPLAY_PROTECTION_ENABLED_TIMEOUT    (1000000)

#define GO_CONTROL_LIST_DIRECTORY                   (0x4605)
#define GO_CONTROL_LIST_DIRECTORY_TIMEOUT           (1000000)

#define GO_CONTROL_ADD_TOOL                         (0x4530)
#define GO_CONTROL_ADD_TOOL_TIMEOUT                 (1000000)

#define GO_CONTROL_ADD_MEASUREMENT                  (0x4531)
#define GO_CONTROL_ADD_MEASUREMENT_TIMEOUT          (1000000)

typedef struct GoControlChannel
{
    GoControl parent; 
    k32u port; 
    kBool isConnected; 
    k16u commandId; 
    kBool legacyConnection;

    kTcpClient client; 
    kSerializer serializer; 
    kTimer timer; 
} GoControlChannel; 

typedef struct GoControlClass
{
    kObjectClass base; 

    kCallback onCancel; 

    kVersion protocolVersion; 
    kBool isCompatible; 

    kBool busy; 

    kIpAddress remoteAddress; 

    GoControlChannel channels[GO_CONTROL_CHANNEL_COUNT]; 
    GoControlChannel* control; 
    GoControlChannel* upgrade; 
} GoControlClass; 

kDeclareClass(Go, GoControl, kObject)

#define GoControl_Cast_(CONTEXT)    kCastClass_(GoControl, CONTEXT)

GoFx(kStatus) GoControl_Init(GoControl control, kType type, kAlloc allocator);
GoFx(kStatus) GoControl_VRelease(GoControl control);

GoFx(kStatus) GoControl_InitChannel(GoControl control, GoControlChannel* channel);
GoFx(kStatus) GoControl_ConnectChannel(GoControl control, GoControlChannel* channel); 

GoFx(kStatus) GoControl_CancelHandler(kPointer channel, kObject sender, kPointer args); 

GoFx(kStatus) GoControl_BeginLegacyCommand(GoControl control, GoControlChannel* channel, k16u commandId, k64u timeout);
GoFx(kStatus) GoControl_LegacySendAndReceive(GoControl control, GoControlChannel* channel);

GoFx(kStatus) GoControl_BeginCommand(GoControl control, GoControlChannel* channel, k16u commandId, k64u timeout); 
GoFx(kStatus) GoControl_Send(GoControl control, GoControlChannel* channel); 
GoFx(kStatus) GoControl_Wait(GoControl control, GoControlChannel* channel); 
GoFx(kStatus) GoControl_Receive(GoControl control, GoControlChannel* channel); 
GoFx(kStatus) GoControl_SendAndReceive(GoControl control, GoControlChannel* channel); 
GoFx(kStatus) GoControl_EndResponse(GoControl control, GoControlChannel* channel); 

GoFx(kStatus) GoControl_GetProtocolVersion(GoControl control, kVersion* version); 

GoFx(kStatus) GoControl_ReadFileStreamed(GoControl control, const kChar* srcFileName, const kChar* dstFileName);

/** 
 * Lists the contents of a directory based on the given extension.
 *
 * @public              @memberof GoControl
 * @param   control     GoControl object.
 * @param   extension   File extension to filter by
 * @param   root        Root directory location
 * @param   isRecursive kTRUE to recurse through all subdirectories and kFALSE to only list the contents of the current directory.
 * @param   fileList    The resulting file path list of directory contents.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ListDirectory(GoControl control, const kChar* extension, const kChar* root, kBool isRecursive, kArrayList fileList);

GoFx(kStatus) GoControl_CreateModel(GoControl control, const kChar* name);
GoFx(kStatus) GoControl_DetectModelEdges(GoControl control, const kChar* name, k16u sensitivity);

GoFx(kStatus) GoControl_SetIsEmulator(GoControl control, kBool value);

GoFx(kStatus) GoControl_SetReplayProtectionEnabled(GoControl control, kBool enable);
GoFx(kStatus) GoControl_GetReplayProtectionEnabled(GoControl control, kBool* enabled);

GoFx(kStatus) GoControl_AddTool(GoControl control, const kChar* type, const kChar* name);
GoFx(kStatus) GoControl_AddMeasurement(GoControl control, k32u index, const kChar* type);

kEndHeader()

#endif
