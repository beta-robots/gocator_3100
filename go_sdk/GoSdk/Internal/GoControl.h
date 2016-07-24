/** 
 * @file    GoControl.h
 * @brief   Declares the GoControl class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_CONTROL_H
#define GO_SDK_CONTROL_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSensorInfo.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kFile.h>
kBeginHeader()

/**
 * @class   GoControl
 * @extends kObject
 * @ingroup GoSdk-Internal
 * @brief   Represents a set of sensor command connections (control, upgrade). 
 */
typedef kObject GoControl; 

/** 
 * Constructs a GoControl object.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     Receives constructed control object. 
 * @param   allocator   Memory allocator (or kNULL for default).
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Construct(GoControl* control, kAlloc allocator); 

/** 
 * Sets an I/O cancellation query handler for this control object. 
 * 
 * The I/O cancellation query handler will be polled periodically when I/O is blocked
 * for a non-negligible amount of time. If the cancellation handler returns kERROR_ABORT, 
 * ongoing communication will be terminated. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     Control object.
 * @param   function    I/O cancellation callback function (or kNULL to unregister).
 * @param   receiver    Receiver argument for callback. 
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_SetCancelHandler(GoControl control, kCallbackFx function, kPointer receiver); 

/** 
 * Opens connections to the specified sensor IP address. 
 *
 * The Open function immediately opens a control connection. Later, while in the 
 * open state, if an upgrade command is given, an upgrade connection will be established
 * automatically at that time. 
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   address     Sensor IP address.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Open(GoControl control, kIpAddress address); 

/** 
 * Closes all open command connections.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Close(GoControl control); 

/** 
 * Reports whether the control object has been opened. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              kTRUE if connected; kFALSE otherwise.            
 */
GoFx(kBool) GoControl_IsConnected(GoControl control); 

/** 
 * Gets the connected sensor's protocol version. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Protocol version.            
 */
GoFx(kVersion) GoControl_ProtocolVersion(GoControl control); 

/** 
 * Reports whether the GoControl object is compatible with the sensor firmware.
 * 
 * Compatibility is determined by comparing the major protocol version reported 
 * by the sensor with the major protocol version supported by this library. If 
 * the major versions match, then compatibility is established.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              kTRUE if compatible; kFALSE otherwise.            
 */
GoFx(kBool) GoControl_IsCompatible(GoControl control); 

/** 
 * Logs into the sensor using the specified user name and password. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   user        User account.
 * @param   password    User password.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Login(GoControl control, GoUser user, const kChar* password); 

/** 
 * Changes the password associated with the specified user account.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   user        User account.
 * @param   password    New password.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ChangePassword(GoControl control, GoUser user, const kChar* password); 

/** 
 * Gets current sensor state information.
 *
 * @public                  @memberof GoControl
 * @version                 Introduced in firmware 4.0.10.27
 * @param   control         GoControl object.
 * @param   localInfo       Receives local sensor information.
 * @param   remoteInfoList  Updates a list of remote sensor information.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoControl_GetSensorInfo(GoControl control, GoSensorInfo localInfo, kArrayList remoteInfoList);

/** 
 * Gets the sensor's scan mode.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   mode        Receives the scan mode.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetScanMode(GoControl control, GoMode* mode); 

/** 
 * Assigns or removes a buddy sensor. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   add         kTRUE to add; kFALSE to remove.
 * @param   buddyId     Buddy device id. 
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ChangeBuddy(GoControl control, kBool add, k32u buddyId); 

/** 
 * Sends a start command to a sensor with the currently selected input source, but doesn't wait for the response.
 * 
 * Use the GoControl_EndStart function to wait for the sensor's reply.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 * @see                 GoControl_EndStart, GoControl_SetInputSource, GoControl_GetInputSource
 */
GoFx(kStatus) GoControl_BeginStart(GoControl control); 

/** 
 * Waits for a start response from a sensor. 
 * 
 * Call this function sometime after calling BeginStart. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_EndStart(GoControl control); 

/** 
 * Sends a scheduled start command to a sensor with the currently selected input source, but doesn't wait for the response.
 * 
 * Use the GoControl_EndScheduledStart function to wait for the sensor's reply.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.1.3.106
 * @param   control     GoControl object.
 * @param   value       Scheduled start value (in uS for time trigger and ticks for encoder trigger)
 * @return              Operation status.            
 * @see                 GoControl_EndScheduledStart, GoControl_SetInputSource, GoControl_GetInputSource
 */
GoFx(kStatus) GoControl_BeginScheduledStart(GoControl control, k64s value); 

/** 
 * Waits for a scheduled start response from a sensor. 
 * 
 * Call this function sometime after calling BeginScheduledStart. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.1.3.106
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_EndScheduledStart(GoControl control); 


/** 
 * Sends a stop command to a sensor, but doesn't wait for the response.
 * 
 * Use the GoControl_EndStop function to wait for the sensor's reply.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 * @see                 GoControl_EndStop
 */
GoFx(kStatus) GoControl_BeginStop(GoControl control); 

/** 
 * Waits for a stop response from a sensor. 
 * 
 * Call this function sometime after calling BeginStop. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_EndStop(GoControl control); 


/** 
 * Sends a alignment command to a sensor, but doesn't wait for the response.
 * 
 * Use the GoControl_EndAlignment function to wait for the sensor's reply.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 * @see                 GoControl_EndAlignment
 */
GoFx(kStatus) GoControl_BeginAlignment(GoControl control);

/** 
 * Waits for a alignment response from a sensor. 
 * 
 * Call this function sometime after calling BeginAlignment. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_EndAlignment(GoControl control); 


/** 
 * Clears the sensor alignment.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.1.3.106
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ClearAlignment(GoControl control);


/** 
 * Sets the alignment reference for a sensor.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   reference   Alignment reference.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_SetAlignmentReference(GoControl control, GoAlignmentRef reference);

/** 
 * Gets the alignment reference for a sensor.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   reference   Alignment reference.
 * @return              Operation status.
 */
GoFx(kStatus) GoControl_GetAlignmentReference(GoControl control, GoAlignmentRef* reference);


/** 
 * Sends an exposure auto set command to a sensor, but doesn't wait for the response.
 * 
 * Use the GoControl_EndExposureAutoSet function to wait for the sensor's reply.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param  role         The sensor's role.
 * @return              Operation status.            
 * @see                 GoControl_EndExposureAutoSet
 */
GoFx(kStatus) GoControl_BeginExposureAutoSet(GoControl control, GoRole role);

/** 
 * Waits for a exposure auto set response from a sensor. 
 * 
 * Call this function sometime after calling BeginExposureAutoSet. 
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_EndExposureAutoSet(GoControl control); 


/** 
 * Reads the list of available sensor files. 
 * 
 * @public                      @memberof GoControl
 * @version                     Introduced in firmware 4.0.10.27
 * @param   control             GoControl object.
 * @param   files               List to be populated with file names (kArrayList<kText64>). 
 * @param   extensionFilter     Can be used to filter the file list: "job", "rec" or null for all.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoControl_ReadFileList(GoControl control, kArrayList files, const kChar* extensionFilter);

/** 
 * Reads a file from the connected sensor.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileName    Name of remote file to be read.
 * @param   data        Receives a pointer to a buffer containing the file data.
 * @param   size        Receives the size of the allocated buffer.
 * @param   allocator   Memory allocator, used to allocate the file buffer (or kNULL for default).
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ReadFile(GoControl control, const kChar* fileName, kByte** data, kSize* size, kAlloc allocator);

/** 
 * Clears the sensor log file.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ClearLog(GoControl control);

/** 
 * Writes a file to the connected sensor.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileName    Name of remote file to be written.
 * @param   data        Pointer to buffer containing the file data.
 * @param   size        Size of the file.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_WriteFile(GoControl control, const kChar* fileName, const kByte* data, kSize size);

/** 
 * Copies a file within the connected sensor.
 * 
 * @public                  @memberof GoControl
 * @version                 Introduced in firmware 4.0.10.27
 * @param   control         GoControl object.
 * @param   source          Source name for the file to be copied.
 * @param   destination     Destination name for the file (maximum 63 characters).
 * @return                  Operation status.            
 */
GoFx(kStatus) GoControl_CopyFile(GoControl control, const kChar* source, const kChar* destination);

/** 
 * Deletes a file within the connected sensor.
 * 
 * @public                  @memberof GoControl
 * @version                 Introduced in firmware 4.0.10.27
 * @param   control         GoControl object.
 * @param   fileName        Name of the file to be deleted.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoControl_DeleteFile(GoControl control, const kChar* fileName);

/** 
 * Gets the name of the default job file to be loaded on boot.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileName    Receives name of the default job.
 * @param   capacity    Name buffer capacity.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetDefaultJob(GoControl control, kChar* fileName, kSize capacity);

/** 
 * Sets a default job file to be loaded on boot.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileName    Name of the default file.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_SetDefaultJob(GoControl control, const kChar* fileName);

/** 
 * Gets the name of the loaded job file and whether it has been modified since loading.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileName    Receives name of the loaded file.
 * @param   capacity    Name buffer capacity.
 * @param   isModified  Receives the status of whether the loaded job has been modified.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetLoadedJob(GoControl control, kChar* fileName, kSize capacity, kBool* isModified);

/** 
 * Restores factory default settings.
 * 
 * @public                  @memberof GoControl
 * @version                 Introduced in firmware 4.0.10.27
 * @param   control         GoControl object.
 * @param   restoreAddress  kTRUE to restore the factory default IP address; kFALSE otherwise.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoControl_RestoreFactory(GoControl control, kBool restoreAddress); 

/** 
 * Reboots the main sensor and any connected buddy sensors.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Reset(GoControl control); 

/** 
 * Begins a sensor firmware upgrade.
 * 
 * Use the GoControl_GetUpgradeStatus function to poll for upgrade completion.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   data        Pointer to buffer containing upgrade file.
 * @param   size        Size of upgrade file.
 * @return              Operation status.            
 * @see                 GoControl_GetUpgradeStatus
 */
GoFx(kStatus) GoControl_BeginUpgrade(GoControl control, void* data, kSize size); 

/** 
 * Polls for upgrade status.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   complete    Receives boolean indicating upgrade completion.
 * @param   succeeded   If complete, receives boolean indicating whether upgrade was successful.
 * @param   progress    If not complete, receives integer percentage indicating progress.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetUpgradeStatus(GoControl control, kBool* complete, kBool* succeeded, k32s* progress); 

/** 
 * Gets the current time stamp value(common among all synchronized sensors).
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   time        Receives the time stamp value.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetTimestamp(GoControl control, k64u* time); 

/** 
 * Gets the current system encoder value.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   encoder     Receives encoder value.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetEncoder(GoControl control, k64s* encoder); 

/** 
 * Sends a software trigger to the sensor.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Trigger(GoControl control); 

/** 
 * Creates and downloads a backup of sensor files.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileData    Receives a pointer to a buffer containing the backup data.
 * @param   size        Receives the size of the allocated buffer.
 * @param   allocator   Memory allocator, used to allocate the backup buffer (or kNULL for default).
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Backup(GoControl control, kByte** fileData, kSize* size, kAlloc allocator); 

/** 
 * Restores a backup of sensor files.
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   fileData    Pointer to a buffer containing the backup data to be restored.
 * @param   size        Size of the backup buffer.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_Restore(GoControl control, const kByte* fileData, kSize size); 

/** 
 * Schedules a digital output. 
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   index       The digital output index.
 * @param   target      The time or position target (uS or mm), depending on the configured domain. Ignored if 
 *                      scheduling disabled or pulsed mode enabled.
 * @param   value       The value of scheduled output (0-Low or 1-High). Ignored if pulsed mode enabled.                      
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ScheduleDigital(GoControl control, k16u index, k64s target, k8u value); 

/** 
 * Schedules an analog output. 
 * 
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   index       The analog output index.
 * @param   target      The time or position target (uS or mm), depending on the configured domain. Ignored if 
 *                      scheduling disabled.
 * @param   value       The value of the scheduled output (mA).               
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ScheduleAnalog(GoControl control, k16u index, k64s target, k32s value); 

/** 
 * Retrieves a set of various sensor states for the sensor associated with the control connection.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   states      A struct of current sensor states.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetStates(GoControl control, GoStates* states);

/** 
 * Enables recording on the sensor.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   enable      Enable or disable recording.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_SetRecordingEnabled(GoControl control, kBool enable); 

/** 
 * Gets the sensor's recording state.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   enabled     Receives the recording state.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetRecordingEnabled(GoControl control, kBool* enabled); 

/** 
 * Sets the sensor's data input source.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   source      The input source to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_SetInputSource(GoControl control, GoInputSource source); 

/** 
 * Gets the sensor's data input source.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   source      Receives the data source used by the sensor.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetInputSource(GoControl control, GoInputSource* source); 

/** 
 * Clear the sensor's replay data.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ClearReplayData(GoControl control); 

/** 
 * Simulate the current frame stored on the sensor's live replay buffer.
 *
 * @public                  @memberof GoControl
 * @version                 Introduced in firmware 4.0.10.27
 * @param   control         GoControl object.
 * @param   isBufferValid   Represents whether the specified data input source contained valid data to simulate against.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoControl_Simulate(GoControl control, kBool* isBufferValid); 

/** 
 * Clear the sensor's measurement statistics.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ClearMeasurementStats(GoControl control); 

/** 
 * Seek to the specified frame position for a replay.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   position    Replay frame position.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_PlaybackSeek(GoControl control, kSize position); 

/** 
 * Advance one frame in a replay.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   direction   The direction to seek. 
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_PlaybackStep(GoControl control, GoSeekDirection direction); 

/** 
 * Get the current frame position in a replay.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   position    Replay frame position.
 * @param   count       Replay frame count.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_PlaybackPosition(GoControl control, kSize* position, kSize* count);

/** 
 * Export an intensity bitmap file to local storage.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   type        The type of data to export.
 * @param   source      The data source to obtain data from.
 * @param   dstFileName The destination file name.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ExportBitmap(GoControl control, 
                                     GoReplayExportSourceType type, 
                                     GoDataSource source,
                                     const kChar* dstFileName);

/** 
 * Export a CSV file to local storage.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   dstFileName The destination file name.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_ExportCsv(GoControl control, const kChar* dstFileName);

/** 
 * Enable or disable sensor AutoStart.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   enable      kTRUE to enable AutoStart, kFALSE to disable it.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_SetAutoStartEnabled(GoControl control, kBool enable);

/** 
 * Returns the state of sensor AutoStart.
 *
 * @public              @memberof GoControl
 * @version             Introduced in firmware 4.0.10.27
 * @param   control     GoControl object.
 * @param   enabled     A pointer to store the boolean state of sensor AutoStart. kTRUE if enabled and kFALSE if disabled.
 * @return              Operation status.            
 */
GoFx(kStatus) GoControl_GetAutoStartEnabled(GoControl control, kBool* enabled);

kEndHeader()
#include <GoSdk/Internal/GoControl.x.h>

#endif
