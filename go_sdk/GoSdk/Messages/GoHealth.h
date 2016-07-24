/** 
 * @file    GoHealth.h
 * @brief   Declares the GoHealthMsg class and related types.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_HEALTH_H
#define GO_SDK_HEALTH_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Io/kSerializer.h>
kBeginHeader()

/**
 * @struct  GoHealthIndicatorId
 * @ingroup GoSdk-HealthChannel
 * @brief   Represents a health indicator ID.
 *
 * The following enumerators are defined:
 * - #GO_HEALTH_ENCODER_VALUE
 * - #GO_HEALTH_ENCODER_FREQUENCY
 * - #GO_HEALTH_FIRMWARE_VERSION
 * - #GO_HEALTH_FIRESYNC_VERSION
 * - #GO_HEALTH_UPTIME
 * - #GO_HEALTH_TEMPERATURE
 * - #GO_HEALTH_PROJECTOR_TEMPERATURE
 * - #GO_HEALTH_LASER_TEMPERATURE
 * - #GO_HEALTH_LASER_OVERHEAT
 * - #GO_HEALTH_LASER_OVERHEAT_DURATION
 * - #GO_HEALTH_MEMORY_USED
 * - #GO_HEALTH_MEMORY_CAPACITY
 * - #GO_HEALTH_STORAGE_USED
 * - #GO_HEALTH_STORAGE_CAPACITY
 * - #GO_HEALTH_CPU_USED
 * - #GO_HEALTH_SYNC_SOURCE
 * - #GO_HEALTH_NET_OUT_USED
 * - #GO_HEALTH_NET_OUT_RATE
 * - #GO_HEALTH_NET_OUT_CAPACITY
 * - #GO_HEALTH_NET_OUT_LINK_STATUS
 * - #GO_HEALTH_DIGITAL_INPUTS
 * - #GO_HEALTH_CAMERA_SEARCH_COUNT
 * - #GO_HEALTH_STATE
 * - #GO_HEALTH_SPEED
 * - #GO_HEALTH_MAXSPEED
 * - #GO_HEALTH_SPOT_COUNT
 * - #GO_HEALTH_MAX_SPOT_COUNT
 * - #GO_HEALTH_SCAN_COUNT
 * - #GO_HEALTH_PLAYBACK_POSITION
 * - #GO_HEALTH_PLAYBACK_COUNT
 * - #GO_HEALTH_DIGITAL_OUTPUT_HIGH_COUNT
 * - #GO_HEALTH_DIGITAL_OUTPUT_LOW_COUNT
 * - #GO_HEALTH_PROCESSING_LATENCY_LAST
 * - #GO_HEALTH_PROCESSING_LATENCY_MAX
 * - #GO_HEALTH_PROCESSING_DROPS
 * - #GO_HEALTH_TRIGGER_DROPS
 * - #GO_HEALTH_OUTPUT_DROPS
 * - #GO_HEALTH_ANALOG_DROPS
 * - #GO_HEALTH_DIGITAL_DROPS
 * - #GO_HEALTH_SERIAL_DROPS
 * - #GO_HEALTH_ETHERNET_DROPS
 * - #GO_HEALTH_RANGE_VALID_COUNT
 * - #GO_HEALTH_RANGE_INVALID_COUNT
 * - #GO_HEALTH_ANCHOR_INVALID_COUNT
 * - #GO_HEALTH_MEASUREMENT
 * - #GO_HEALTH_MEASUREMENT_PASS
 * - #GO_HEALTH_MEASUREMENT_FAIL
 * - #GO_HEALTH_MEASUREMENT_MIN
 * - #GO_HEALTH_MEASUREMENT_MAX
 * - #GO_HEALTH_MEASUREMENT_AVERAGE
 * - #GO_HEALTH_MEASUREMENT_STDEV
 * - #GO_HEALTH_MEASUREMENT_INVALID_COUNT
 * - #GO_HEALTH_MEASUREMENT_OVERFLOW_COUNT
 */
typedef k32s GoHealthIndicatorId;
/** @name    GoHealthIndicatorId
 *@{*/ 
#define GO_HEALTH_ENCODER_VALUE                        (1003)      ///< Current system encoder tick.
#define GO_HEALTH_ENCODER_FREQUENCY                    (1005)      ///< Current system encoder frequency (ticks/s).

#define GO_HEALTH_FIRMWARE_VERSION                     (2000)      ///< Firmware application version.
#define GO_HEALTH_FIRESYNC_VERSION                     (20600)     ///< FireSync version

#define GO_HEALTH_UPTIME                               (2017)      ///< Time elapsed since boot-up or reset (seconds).

#define GO_HEALTH_TEMPERATURE                          (2002)      ///< Internal temperature (degrees Celsius).
#define GO_HEALTH_PROJECTOR_TEMPERATURE                (2404)      ///< Projector temperature (degrees Celsius).
#define GO_HEALTH_LASER_TEMPERATURE                    (2028)      ///< Laser temperature (degrees Celsius).  Available only on 3B-class devices.
#define GO_HEALTH_LASER_OVERHEAT                       (20020)     ///< Indicates whether the laser is overheating.
#define GO_HEALTH_LASER_OVERHEAT_DURATION              (20021)     ///< Indicates how long the laser has been overheating if it is overheating.

#define GO_HEALTH_MEMORY_USED                          (2003)      ///< Amount of memory currently used (bytes).
#define GO_HEALTH_MEMORY_CAPACITY                      (2004)      ///< Total amount of memory available (bytes).
#define GO_HEALTH_STORAGE_USED                         (2005)      ///< Amount of non-volatile storage used (bytes).
#define GO_HEALTH_STORAGE_CAPACITY                     (2006)      ///< Total amount of non-volatile storage available (bytes).
#define GO_HEALTH_CPU_USED                             (2007)      ///< CPU usage (percentage of maximum).

#define GO_HEALTH_SYNC_SOURCE                          (2043)      ///< Sensor synchronization source. (1 - Master, 2 - Device/Gocator)
#define GO_HEALTH_NET_OUT_USED                         (21003)     ///< Current outbound network count (bytes).
#define GO_HEALTH_NET_OUT_RATE                         (21004)     ///< Current outbound network throughput (bytes/second). 
#define GO_HEALTH_NET_OUT_CAPACITY                     (2009)      ///< Total available outbound network throughput (bytes/s).
#define GO_HEALTH_NET_OUT_LINK_STATUS                  (2034)      ///< The ethernet output's current network link status

#define GO_HEALTH_DIGITAL_INPUTS                       (2024)      ///< Current digital input status (one bit per input).
#define GO_HEALTH_CAMERA_SEARCH_COUNT                  (2217)      ///< Number of search states.

#define GO_HEALTH_STATE                                (20000)     ///< Current system state.
#define GO_HEALTH_SPEED                                (20001)     ///< Current speed (Hz).
#define GO_HEALTH_MAXSPEED                             (20002)     ///< Maximum speed (Hz).
#define GO_HEALTH_SPOT_COUNT                           (20003)     ///< Number of found spots in the last profile
#define GO_HEALTH_MAX_SPOT_COUNT                       (20004)     ///< Maximum number of spots that can be found
#define GO_HEALTH_SCAN_COUNT                           (20005)     ///< The number of scanned profiles or surfaces.

#define GO_HEALTH_PLAYBACK_POSITION                    (20023)     ///< Indicates the current replay playback index.
#define GO_HEALTH_PLAYBACK_COUNT                       (20024)     ///< Indicates the number of frames present in the current replay.

#define GO_HEALTH_DIGITAL_OUTPUT_HIGH_COUNT            (21006)     ///< The number scans with high digital output pulses.
#define GO_HEALTH_DIGITAL_OUTPUT_LOW_COUNT             (21007)     ///< The number scans with no digital output pulse.

#define GO_HEALTH_PROCESSING_LATENCY_LAST              (21001)     ///< Last reported processing latency value (in uS).
#define GO_HEALTH_PROCESSING_LATENCY_MAX               (21002)     ///< Maximum reported processing latency.
#define GO_HEALTH_PROCESSING_DROPS                     (21000)     ///< Current number of processing drops.
#define GO_HEALTH_TRIGGER_DROPS                        (21010)     ///< Current number of trigger drops.
#define GO_HEALTH_OUTPUT_DROPS                         (21011)     ///< Current number of output drops. Sum of all output drops.

#define GO_HEALTH_ANALOG_DROPS                         (2501)      ///< Current number of analog output drops.
#define GO_HEALTH_DIGITAL_DROPS                        (2601)      ///< Current number of digital output drops.
#define GO_HEALTH_SERIAL_DROPS                         (2701)      ///< Current number of serial output drops.
#define GO_HEALTH_ETHERNET_DROPS                       (21005)     ///< Current number of ethernet output drops.

#define GO_HEALTH_RANGE_VALID_COUNT                    (21100)     ///< Current number of frames with valid range data.
#define GO_HEALTH_RANGE_INVALID_COUNT                  (21101)     ///< Current number of frames with invalid range data.
#define GO_HEALTH_ANCHOR_INVALID_COUNT                 (21200)     ///< Number of frames with anchor invalid.

#define GO_HEALTH_ENCODER_Z_INDEX_PULSE_DROPS          (22000)     ///< Encoder z-index pulse drops

#define GO_HEALTH_MEASUREMENT                          (30000)     ///< Measurement value.
#define GO_HEALTH_MEASUREMENT_PASS                     (30001)     ///< Number of pass decisions.
#define GO_HEALTH_MEASUREMENT_FAIL                     (30002)     ///< Number of fail decisions.
#define GO_HEALTH_MEASUREMENT_MIN                      (30003)     ///< Minimum measurement value.
#define GO_HEALTH_MEASUREMENT_MAX                      (30004)     ///< Maximum measurement value.
#define GO_HEALTH_MEASUREMENT_AVERAGE                  (30005)     ///< Average measurement value.
#define GO_HEALTH_MEASUREMENT_STDEV                    (30006)     ///< Measurement value standard deviation.
#define GO_HEALTH_MEASUREMENT_INVALID_COUNT            (30007)     ///< Number of invalid values.
#define GO_HEALTH_MEASUREMENT_OVERFLOW_COUNT           (30008)     ///< Number of values which exceed the numerical limit of an output protocol's measurement value field.
/**@}*/


/**
 * @struct  GoIndicator
 * @extends kValue
 * @ingroup GoSdk-HealthChannel
 * @brief   Represents health indicator. 
 */
typedef struct GoIndicator
{
    k32u id;                ///< Indicator ID (e.g. GO_HEALTH_CPU_USED)
    k32u instance;          ///< Indicator instance number.
    k64s value;             ///< Indicator value. 
} GoIndicator; 

/**
 * @class   GoHealthMsg
 * @extends kObject
 * @ingroup GoSdk-HealthChannel
 * @brief   Represents health information from a single sensor. 
 */
typedef kObject GoHealthMsg; 

/** 
 * Gets the health source.
 *
 * @public             @memberof GoHealthMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Health source. 
 */
GoFx(GoDataSource) GoHealthMsg_Source(GoHealthMsg msg);

/** 
 * Count of health indicators in this message. 
 *
 * @public             @memberof GoHealthMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of indicators. 
 */
GoFx(kSize) GoHealthMsg_Count(GoHealthMsg msg);

/** 
 * Gets the health indicator at the specified index.
 *
 * @public             @memberof GoHealthMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Indicator index. 
 * @return             Indicator pointer.
 */
GoFx(GoIndicator*) GoHealthMsg_At(GoHealthMsg msg, kSize index);

/** 
 * Finds the health indicator with the matching ID. Returns kNULL if not found.
 *
 * @public             @memberof GoHealthMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   id         Indicator index. 
 * @param   instance   Indicator instance.
 * @return             Indicator pointer.
 */
GoFx(GoIndicator*) GoHealthMsg_Find(GoHealthMsg msg, k32u id, k32u instance);

kEndHeader()
#include <GoSdk/Messages/GoHealth.x.h>

#endif
