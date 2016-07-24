/** 
 * @file    GoSystem.h
 * @brief   Declares the GoSystem class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SYSTEM_H
#define GO_SDK_SYSTEM_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoMultiplexBank.h>

kBeginHeader()

/**
 * @class   GoSystem
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a system of Gocator devices.
 * 
 * During construction, a GoSystem object uses Gocator Discovery Protocol to locate devices. 
 * Subsequently, the list of detected devices can be accessed using GoSystem_SensorCount and 
 * GoSystem_SensorAt.
 * 
 * Immediately after construction, all sensor objects are in the <em>online</em> state. This state 
 * indicates that a sensor has been detected, but that the client has not yet established a control 
 * connection to the sensor. The only sensor functions than can be used in this state are GoSensor_State,
 * GoSensor_Address, and GoSensor_SetAddress. 
 *
 * Because Gocator Discovery Protocol works <em>across</em> subnets, but Gocator Control Protocol 
 * does not, it is possible for a sensor to be successfully detected but for subsequent connection 
 * attempts to fail. In this case, the GoSensor_SetAddress function can be used to reconfigure the 
 * sensor to operate on the same subnet as the client.
 * 
 * If the client and device are configured to operate on the same subnet, the GoSystem_Connect or 
 * GoSensor_Connect functions can be used to establish control connections. 
 *
 * @see     GoSystem_Construct, GoSystem_Connect, GoSensor_State, GoSensor_Address, GoSensor_SetAddress. 
 */
typedef kObject GoSystem; 

/** Defines the signature for a GoSystem data/health handler. 
 *  
 *  NOTE: The GoDataSet received by this function must be destroyed after use,
 *        otherwise a memory leak will result.
 *
 *  @see    GoDestroy
*/
typedef kStatus (kCall* GoDataFx)(kPointer context, GoSensor system, GoDataSet data); 

/** 
 * Constructs a GoSystem object.
 *
 * During construction, a GoSystem object uses Gocator Discovery Protocol to locate sensors. 
 * The list of detected sensors can be accessed using the GoSystem_SensorCount and GoSystem_SensorAt 
 * functions.
 *
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      Receives constructed system object. 
 * @param   allocator   Memory allocator (or kNULL for default)
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_Construct(GoSystem* system, kAlloc allocator);

/** 
 * Establishes control connections to all sensors. 
 *
 * A control connection is required before calling any sensor function except GoSensor_State, 
 * GoSensor_Address, or GoSensor_SetAddress. 
 *
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_Connect(GoSystem system);

/** 
 * Terminates control connections to all sensors. 
 *
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_Disconnect(GoSystem system);

/** 
 * Reports whether the system has changes that require a refresh. 
 * 
 * Sensors can undergo autonomous state changes that require client state to be refreshed 
 * (e.g. sensor goes offline). The GoSystem_HasChanges function can be used to determine 
 * if such changes have occurred.  
 * 
 * The GoSystem_HasChanges function does not perform communication, and consequently, will not 
 * require the caller to block for a long duration. If changes are detected, the GoSystem_Refresh 
 * function can be called to resolve the changes. 
 * 
 * This method is thread-safe.
 *
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              kTRUE if the system has changes.
 */
GoFx(kBool) GoSystem_HasChanges(GoSystem system);

/** 
 * Updates client state to reflect any changes that have occurred in the sensor network. 
 * 
 * Sensors can undergo autonomous state changes that require client state to be refreshed 
 * (e.g. sensor goes offline). The GoSystem_Refresh function resynchronizes client state 
 * with the current state of the sensor network. 
 * 
 * Calling this function may destroy or modify local sensor objects. The GoSystem_HasChanges 
 * function can be used prior to calling GoSystem_Refresh, to determine whether a refresh is 
 * needed.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_Refresh(GoSystem system);

/** 
 * Reports the Gocator Protocol version implemented by this library. 
 * 
 * A Gocator Protocol version number has a major component and a minor component. 
 * If the major version implemented by this library is the same as the major
 * version implemented by a sensor device, then communication can proceed.  Otherwise, 
 * the sensor should be upgraded or a newer version of this library should be obtained. 
 * Sensors with incompatible major versions will be reported as being in the 
 * <em>incompatible</em> state upon connection.
 * 
 * If major versions match, but the minor version implemented by this library 
 * is lower than the minor version implemented by the sensor, then some sensor features
 * will not be accessible through this library. 
 *
 * If major versions match, but the minor version implemented by this library 
 * is higher than the minor version implemented by the sensor, then some features
 * exposed by this library may be unimplemented by the sensor. 
 * 
 * This method is thread-safe.
 * 
 * @public      @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @return      Protocol version implemented by this library.
 * @see         GoSensor_ProtocolVersion, GoSensor_State        
 */
GoFx(kVersion) GoSystem_ProtocolVersion();

/** 
 * Reports the Gocator firmware version that was built alongside this library.
 * 
 * A Gocator SDK version number has a major component and a minor component. 
 * If the major version implemented by this library is the same as the major
 * version implemented by a sensor device, then communication can proceed.  Otherwise, 
 * the sensor should be upgraded or a version of this library matching the sensor 
 * should be obtained. 
 * 
 * When it comes to the matter of mismatched SDK and firmware versions, so long
 * as the protocol versions match up, they should be compatible.
 * 
 * @public      @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @return      Firmware version that was built alongside this library.
 * @see         GoSensor_FirmwareVersion, GoSystem_ProtocolVersion
 */
GoFx(kVersion) GoSystem_SdkVersion();

/** 
 * Sets a callback function that can be used to receive sensor data messages asynchronously.
 * 
 * Sensor data messages can be received synchronously using the GoSystem_ReceiveData function 
 * or asynchronously by registering a callback function.  If a callback function is registered, 
 * a background thread will be created to perform notifications.
 * 
 * To unregister a previously-registered data handler, call this function using kNULL in place 
 * of the callback function argument.  
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   function    Data callback function (or kNULL to unregister).
 * @param   receiver    Receiver argument for callback. 
 * @return              Operation status. 
 * @see                 GoDataFx, GoSystem_ReceiveData, GoSystem_SetDataCapacity, GoSystem_EnableData
 */
GoFx(kStatus) GoSystem_SetDataHandler(GoSystem system, GoDataFx function, kPointer receiver); 

/** 
 * Sets the maximum amount of memory that can be used to buffer received data messages. 
 * 
 * Received data messages are enqueued until they are accepted by the caller. This function 
 * determines the maximum size, in bytes, of enqueued messages. The default maximum size is 50 MB. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   capacity    Data queue capacity, in bytes. 
 * @return              Operation status. 
 * @see                 GoSystem_DataCapacity
 */
GoFx(kStatus) GoSystem_SetDataCapacity(GoSystem system, kSize capacity);

/** 
 * Reports that maximum amount of memory that can be used to buffer received data messages. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Data queue capacity, in bytes.  
 * @see                 GoSystem_SetDataCapacity
 */
GoFx(kSize) GoSystem_DataCapacity(GoSystem system);

/** 
 * Establishes data connections to all connected sensors currently in the <em>ready</em> or <em>running</em> states. 
 * 
 * Data connections are not automatically established when sensor control connection are established. 
 * Use this function (or GoSensor_EnableData) to enable/disable data connections.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   enable      kTRUE to enable data connections; kFALSE to disable. 
 * @return              Operation status. 
 * @see                 GoSystem_ReceiveData, GoSensor_EnableData
 */
GoFx(kStatus) GoSystem_EnableData(GoSystem system, kBool enable);

/** 
 * Clears any buffered data messages. 
 * 
 * When stopping and then restarting a system, it may be desirable to ensure that no messages from 
 * the previous session remain in any buffers. The GoSystem_ClearData function closes any open data 
 * channels, destroys any received messages, and then reopens data channels. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_ClearData(GoSystem system);

/** 
 * Receives a set of sensor data messages. 
 * 
 * Sensor data messages can be received synchronously using this function or asynchronously by 
 * registering a callback with the GoSystem_SetDataHandler function.  
 * 
 * NOTE: Data received with this function must be destroyed after use, 
 *       otherwise a memory leak will result.
 *
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   data        Set of sensor data messages. 
 * @param   timeout     Duration to wait for messages, in microseconds. 
 * @return              Operation status. 
 * @see                 GoDestroy, GoSystem_SetDataHandler, GoSystem_SetDataCapacity, GoSystem_EnableData
 */
GoFx(kStatus) GoSystem_ReceiveData(GoSystem system, GoDataSet* data, k64u timeout);

/** 
 * Sets a callback function that can be used to receive sensor health messages asynchronously.
 * 
 * Sensor health messages can be received synchronously using the GoSystem_ReceiveHealth function 
 * or asynchronously by registering a callback function. If a callback function is registered, 
 * a background thread will be created to perform notifications.
 * 
 * To unregister a previously-registered health handler, call this function using kNULL in place 
 * of the callback function argument.  
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   function    Health callback function (or kNULL to unregister).
 * @param   receiver    Receiver argument, passed to callback. 
 * @return              Operation status. 
 * @see                 GoDataFx, GoSystem_ReceiveHealth, GoSystem_SetDataCapacity, GoSystem_EnableData
 */
GoFx(kStatus) GoSystem_SetHealthHandler(GoSystem system, GoDataFx function, kPointer receiver);

/** 
 * Receives a set of sensor health messages. 
 * 
 * Sensor health messages can be received synchronously using this function or asynchronously by 
 * registering a callback with the GoSystem_SetHealthHandler function.  
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   data        Set of sensor health messages. 
 * @param   timeout     Duration to wait for messages, in microseconds. 
 * @return              Operation status. 
 * @see                 GoSystem_SetHealthHandler
 */
GoFx(kStatus) GoSystem_ReceiveHealth(GoSystem system, GoDataSet* data, k64u timeout);

/** 
 * Clears any buffered health messages. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_ClearHealth(GoSystem system);

/** 
 * Starts all sensors that are currently in the <em>ready</em> state.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 * @see                 GoSystem_Stop, GoSensor_Start, GoSensor_Stop
 */
GoFx(kStatus) GoSystem_Start(GoSystem system);

/** 
 * Starts all sensors that are currently in the <em>ready</em> state at a scheduled value.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.1.3.106
 * @param   system      GoSystem object.  
 * @param   value       The value at which to start the sensor. It is in uS when time triggered and ticks when encoder triggered.
 * @return              Operation status. 
 * @see                 GoSystem_Stop, GoSensor_ScheduledStart, GoSensor_Stop, GoSetup_SetTriggerSource, GoSetup_TriggerSource
 */
GoFx(kStatus) GoSystem_ScheduledStart(GoSystem system, k64s value);

/** 
 * Performs alignment for sensors in the <em>ready</em> state.
 *
 * NOTE: This operation will result in sensor starts for the duration of the
 * alignment. It can be canceled via GoSystem_Stop. This function's operation
 * status does not correspond to the actual alignment result. In order to
 * retrieve the alignment result, you must enable the data channel before calling
 * this function, receive an alignment data message and then check its status.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 * @see                 GoSystem_EnableData, GoSystem_ReceiveData, GoSystem_Stop
 * @remark              Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSystem_StartAlignment(GoSystem system);

/** 
 * Performs exposure auto set for sensors in the <em>ready</em> state.
 * 
 * NOTE: This operation will result in sensor starts for the duration of the
 * exposure AutoSet. A successful operation status does NOT modify the configuration.
 * You must retrieve the resulting exposure values and set it for the appropriate
 * exposure setting. This involves enabling the data connection prior to running 
 * exposure auto set and then receiving an exposure auto set message, which 
 * you can then use to query the status and access the resulting value.
 *
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 * @see                 GoSystem_EnableData, GoSystem_ReceiveData
 */
GoFx(kStatus) GoSystem_StartExposureAutoSet(GoSystem system);

/** 
 * Stops all connected sensors. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 * @see                 GoSystem_Start, GoSensor_Start, GoSensor_Stop
 */
GoFx(kStatus) GoSystem_Stop(GoSystem system);


/** 
 * Reboots all connected sensors. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   wait        kTRUE to wait for reboot complete; kFALSE to return immediately.
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_Reset(GoSystem system, kBool wait);

/** 
 * Aborts ongoing sensor communication. 
 *
 * This method asynchronously aborts ongoing communication; the next time that any 
 * I/O operation blocks for an extended period of time, it will be terminated. This method 
 * is thread-safe. 
 * 
 * In order to resume communication, call GoSystem_Refresh or GoSystem_Connect. 
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status. 
 */
GoFx(kStatus) GoSystem_Cancel(GoSystem system);

/** 
 * Gets the number of sensors in the system.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Count of sensors in the system.
 * @see                 GoSystem_SensorAt
 */
GoFx(kSize) GoSystem_SensorCount(GoSystem system);

/** 
 * Gets the sensor object at the specified index.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   index       Sensor index.
 * @return              Sensor object.
 * @see                 GoSystem_SensorCount
 */
GoFx(GoSensor) GoSystem_SensorAt(GoSystem system, kSize index);

/** 
 * Gets the sensor object with the specified device id (serial number).
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   id          Device identifier. 
 * @param   sensor      Receives sensor object. 
 * @return              Operation status (kERROR_NOT_FOUND if sensor not found).
 * @see                 GoSystem_SensorCount, GoSystem_SensorAt
 */
GoFx(kStatus) GoSystem_FindSensorById(GoSystem system, k32u id, GoSensor* sensor);

/** 
 * Gets the sensor object with the specified IP address.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   address     Pointer to sensor IP address.
 * @param   sensor      Receives sensor object. 
 * @return              Operation status (kERROR_NOT_FOUND if sensor not found).
 * @see                 GoSystem_SensorCount, GoSystem_SensorAt
 */
GoFx(kStatus) GoSystem_FindSensorByIpAddress(GoSystem system, const kIpAddress* address, GoSensor* sensor);

/** 
 * Gets the current time stamp from the sensor network.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   time        Receives current time stamp(microseconds). 
 * @return              Operation status (kERROR_NOT_FOUND if no sensors available).
 * @see                 GoSystem_GetEncoder
 */
GoFx(kStatus) GoSystem_Timestamp(GoSystem system, k64u* time);

/** 
 * Gets the current encoder value from the sensor network.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   encoder     Receives current encoder value (ticks). 
 * @return              Operation status (kERROR_NOT_FOUND if no sensors available).
 * @see                 GoSystem_GetTime
 */
GoFx(kStatus) GoSystem_Encoder(GoSystem system, k64s* encoder);


/// @cond (Gocator_1x00 || Gocator_2x00)

/** 
 * Gets the current multiplex bank count.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              The current number of defined multiplex banks.
 */
GoFx(kSize) GoSystem_MultiplexBankCount(GoSystem system);

/** 
 * Gets a multiplex bank corresponding to the given index.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   index       The index of the multiplex bank to retrieve.
 * @return              The multiplex bank at the given index or kNULL if an invalid index is given.
 */
GoFx(GoMultiplexBank) GoSystem_MultiplexBankAt(GoSystem system, kSize index);

/** 
 * Adds a multiplex bank with a unique ID to the system.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object. 
 * @param   bank        A pointer to the newly created GoMultiplexBank object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSystem_AddMultiplexBank(GoSystem system, GoMultiplexBank* bank);

/** 
 * Removes a multiplex bank at the given index.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   index       The index of the multiplex bank to remove.
 * @return              Operation status.
 */
GoFx(kStatus) GoSystem_RemoveMultiplexBank(GoSystem system, kSize index);

/** 
 * Removes all multiplex banks.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status.
 */
GoFx(kStatus) GoSystem_ClearMultiplexBanks(GoSystem system);

/** 
 * Gets the maximum sensor exposure duration in the given multiplex bank.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   bank        The multiplex bank to check.
 * @return              The maximum sensor exposure duration contained in the multiplexing bank.
 */
GoFx(k64f) GoSystem_MaxBankExposureDuration(GoSystem system, GoMultiplexBank bank);

/** 
 * Automatically update the single multiplexing delay and period configuration for all sensors contained in all defined multiplex banks.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   period      The exposure period to set among all multiplexed sensors. A value of 0.0 will result in automatic period determination.
 * @return              Operation status.
 */
GoFx(kStatus) GoSystem_UpdateAllMultiplexParameters(GoSystem system, k64f period);

/** 
 * Automatically update the multiplexing single delay configuration for all sensors contained in all defined multiplex banks.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status.
 */
GoFx(kStatus) GoSystem_UpdateMultiplexDelay(GoSystem system);

/** 
 * Returns the maximum value of all multiplexed sensor's minimum multiplexing periods.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @return              Operation status.
 */
GoFx(k64f) GoSystem_MaxMinimumMultiplexPeriod(GoSystem system);

/** 
 * Automatically update the multiplexing single period configuration for all sensors contained in all defined multiplex banks.
 * 
 * @public              @memberof GoSystem
 * @version             Introduced in firmware 4.0.10.27
 * @param   system      GoSystem object.  
 * @param   period      The exposure period to set among all multiplexed sensors. A value of 0.0 will result in automatic period determination.
 * @return              Operation status.
 */
GoFx(kStatus) GoSystem_UpdateMultiplexPeriod(GoSystem system, k64f period);

/// @endcond

kEndHeader()
#include <GoSdk/GoSystem.x.h>

#endif
