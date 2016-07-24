/**
 * @file    GoSdkDef.h
 * @brief   Essential SDK declarations.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DEF_H
#define GO_SDK_DEF_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kNetwork.h>

kBeginHeader()

#if defined (GO_EMIT)
#    define GoFx(TYPE)    kExportFx(TYPE)            ///< GoSdk function declaration helper.
#    define GoDx(TYPE)    kExportDx(TYPE)            ///< GoSdk data declaration helper.
#elif defined (GO_STATIC)
#    define GoFx(TYPE)    kInFx(TYPE)
#    define GoDx(TYPE)    kInDx(TYPE)
#else
#    define GoFx(TYPE)    kImportFx(TYPE)
#    define GoDx(TYPE)    kImportDx(TYPE)
#endif

/**
 * Returns the SDK version.
 *
 * @public          @memberof GoSdk
 * @version         Introduced in firmware 4.0.10.27
 * @return          Protocol version.
 */
GoFx(kVersion) GoSdk_Version();

/**
 * Returns the protocol version associated with the SDK.
 *
 * @public          @memberof GoSdk
 * @version         Introduced in firmware 4.0.10.27
 * @return          Protocol version.
 */
GoFx(kVersion) GoSdk_ProtocolVersion();

/**
 * Frees the memory associated with a given kObject sourced class handle.
 *
 * @public              @memberof GoSdk
 * @version             Introduced in firmware 4.0.10.27
 * @param   object      A kObject.
 * @return              Operation status.
 */
GoFx(kStatus) GoDestroy(kObject object);

/**
 * @struct  GoUpgradeFxArgs
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents arguments provided to an upgrade callback function.
 */
typedef struct GoUpgradeFxArgs
{
    k64f progress;          ///< Upgrade progress (percentage).
} GoUpgradeFxArgs;

typedef kStatus (kCall* GoUpgradeFx) (kPointer receiver, kObject sender, GoUpgradeFxArgs* args);


/**
 * @struct  GoUser
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a user id.
 *
 * The following enumerators are defined:
 * - #GO_USER_NONE
 * - #GO_USER_ADMIN
 * - #GO_USER_TECH
 */
typedef k32s GoUser;
/** @name    GoUser
 *@{*/ 
#define GO_USER_NONE        (0)         ///< No user.
#define GO_USER_ADMIN       (1)         ///< Administrator user.
#define GO_USER_TECH        (2)         ///< Technician user.
/**@}*/ 

/**
 * @struct  GoState
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the current state of a sensor object.
 *
 * The following enumerators are defined:
 * - #GO_STATE_ONLINE
 * - #GO_STATE_OFFLINE
 * - #GO_STATE_RESETTING
 * - #GO_STATE_INCOMPATIBLE
 * - #GO_STATE_INCONSISTENT
 * - #GO_STATE_UNRESPONSIVE
 * - #GO_STATE_CANCELLED
 * - #GO_STATE_INCOMPLETE
 * - #GO_STATE_BUSY
 * - #GO_STATE_READY
 * - #GO_STATE_RUNNING
 */
typedef k32s GoState;
/** @name    GoState
 *@{*/ 
#define GO_STATE_ONLINE              (0)        ///< Sensor disconnected, but detected via discovery.
#define GO_STATE_OFFLINE             (1)        ///< Sensor disconnected and no longer detected via discovery (refresh system to eliminate sensor).
#define GO_STATE_RESETTING           (2)        ///< Sensor disconnected and currently resetting (wait for completion).
#define GO_STATE_INCOMPATIBLE        (4)        ///< Sensor connected, but protocol incompatible with client (upgrade required).
#define GO_STATE_INCONSISTENT        (5)        ///< Sensor connected, but remote state was changed (refresh sensor).
#define GO_STATE_UNRESPONSIVE        (6)        ///< Sensor connected, but no longer detected via health or discovery (disconnect).
#define GO_STATE_CANCELLED           (7)        ///< Sensor connected, but communication aborted via GoSensor_Cancel function (disconnect or refresh sensor).
#define GO_STATE_INCOMPLETE          (8)        ///< Sensor connected, but a required buddy sensor is not present (wait or remove buddy association).
#define GO_STATE_BUSY                (9)        ///< Sensor connected, but currently controlled by another sensor (cannot be configured directly).
#define GO_STATE_READY               (10)       ///< Sensor connected and ready to accept configuration commands.
#define GO_STATE_RUNNING             (11)       ///< Sensor connected and currently running.
/**@}*/ 

/**
 * @struct  GoRole
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a user role.
 *
 * The following enumerators are defined:
 * - #GO_ROLE_MAIN
 * - #GO_ROLE_BUDDY
 */
typedef k32s GoRole;
/** @name    GoRole
 *@{*/ 
#define GO_ROLE_MAIN                 (0)        ///< Sensor is operating as a main sensor.
#define GO_ROLE_BUDDY                (1)        ///< Sensor is operating as a buddy sensor.
/**@}*/ 

/**
 * @struct  GoAlignmentState
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment state.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_STATE_NOT_ALIGNED
 * - #GO_ALIGNMENT_STATE_ALIGNED
 */
typedef k32s GoAlignmentState;
/** @name    GoAlignmentState
 *@{*/ 
#define GO_ALIGNMENT_STATE_NOT_ALIGNED       (0) ///< Sensor is not aligned.
#define GO_ALIGNMENT_STATE_ALIGNED           (1) ///< Sensor is aligned.
/**@}*/ 

/**
 * @struct  GoAlignmentRef
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment reference.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_REF_FIXED
 * - #GO_ALIGNMENT_REF_DYNAMIC
 */
typedef k32s GoAlignmentRef;
/** @name    GoAlignmentRef
 *@{*/ 
#define GO_ALIGNMENT_REF_FIXED             (0)  ///< The alignment used will be specific to the sensor.
#define GO_ALIGNMENT_REF_DYNAMIC           (1)  ///< The alignment used will be specific to the current job if saved.
/**@}*/

/**
 * @struct  GoMode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a scan mode.
 *
 * The following enumerators are defined:
 * - #GO_MODE_UNKNOWN
 * - #GO_MODE_VIDEO
 * - #GO_MODE_RANGE
 * - #GO_MODE_PROFILE
 * - #GO_MODE_SURFACE
 */ 
typedef k32s GoMode;
/** @name    GoMode
 *@{*/ 
#define GO_MODE_UNKNOWN                     (-1)    ///< Unknown scan mode.
#define GO_MODE_VIDEO                       (0)     ///< Video scan mode.
#define GO_MODE_RANGE                       (1)     ///< Range scan mode.
#define GO_MODE_PROFILE                     (2)     ///< Profile scan mode.
#define GO_MODE_SURFACE                     (3)     ///< Surface scan mode.
/**@}*/ 


/**
 * @struct  GoTrigger
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a trigger.
 *
 * The following enumerators are defined:
 * - #GO_TRIGGER_TIME
 * - #GO_TRIGGER_ENCODER
 * - #GO_TRIGGER_INPUT
 * - #GO_TRIGGER_SOFTWARE
 */ 
typedef k32s GoTrigger;
/** @name    GoTrigger
 *@{*/ 
#define GO_TRIGGER_TIME             (0)     ///< The sensor will be time triggered.
#define GO_TRIGGER_ENCODER          (1)     ///< The sensor will be encoder triggered.
#define GO_TRIGGER_INPUT            (2)     ///< The sensor will be digital input triggered.
#define GO_TRIGGER_SOFTWARE         (3)     ///< The sensor will be software triggered.
/** @} */

/**
 * @struct  GoEncoderTriggerMode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an encoder's triggering behavior.
 *
 * The following enumerators are defined:
 * - #GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE
 * - #GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE
 * - #GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL
 */ 
typedef k32s GoEncoderTriggerMode;
/** @name    GoEncoderTriggerMode
 *@{*/ 
#define GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE      (0)        ///< Do not reverse trigger. Track reverse motion to prevent repeat forward triggers.
#define GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE     (1)        ///< Do not reverse trigger. Forward trigger unconditionally.
#define GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL      (2)        ///< Forward and reverse trigger.
/** @} */

/**
 * @struct  GoFrameRateMaxSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the current maximum frame rate limiting source.
 *
 * The following enumerators are defined:
 * - #GO_FRAME_RATE_MAX_SOURCE_CAMERA
 * - #GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION
 */ 
typedef k32s GoFrameRateMaxSource;
/** @name    GoFrameRateMaxSource
 *@{*/ 
#define GO_FRAME_RATE_MAX_SOURCE_CAMERA            (0)        ///< Limited by the sensor's camera configuration.
#define GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION    (1)        ///< Limited by part detection logic.
/** @} */

/**
 * @struct  GoEncoderSpacingMinSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the current encoder period limiting source.
 *
 * The following enumerators are defined:
 * - #GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION
 * - #GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION
 */ 
typedef k32s GoEncoderSpacingMinSource;
/** @name    GoEncoderSpacingMinSource
 *@{*/ 
#define GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION         (0)            ///< Limited by encoder resolution.
#define GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION     (1)            ///< Limited by part detection logic.
/**@}*/ 

/**
 * @struct  GoTriggerUnits
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the system's primary synchronization domain
 *
 * The following enumerators are defined:
 * - #GO_TRIGGER_UNIT_TIME
 * - #GO_TRIGGER_UNIT_ENCODER
 */
typedef k32s GoTriggerUnits;
/** @name    GoTriggerUnits
 *@{*/ 
#define GO_TRIGGER_UNIT_TIME                      (0)           ///< Base the system on the internal clock.
#define GO_TRIGGER_UNIT_ENCODER                   (1)           ///< Base the system on the encoder.
/**@}*/ 

/**
 * @struct  GoExposureMode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents all possible exposure modes.
 *
 * The following enumerators are defined:
 * - #GO_EXPOSURE_MODE_SINGLE
 * - #GO_EXPOSURE_MODE_MULTIPLE
 * - #GO_EXPOSURE_MODE_DYNAMIC
 */ 
typedef k32s GoExposureMode;
/** @name    GoExposureMode
 *@{*/ 
#define GO_EXPOSURE_MODE_SINGLE         (0)             ///< Single exposure mode.
#define GO_EXPOSURE_MODE_MULTIPLE       (1)             ///< Multiple exposure mode.
#define GO_EXPOSURE_MODE_DYNAMIC        (2)             ///< Dynamic exposure mode.
/**@}*/ 

/**
 * @struct  GoOrientation
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a sensor orientation type.
 *
 * The following enumerators are defined:
 * - #GO_ORIENTATION_WIDE
 * - #GO_ORIENTATION_OPPOSITE
 * - #GO_ORIENTATION_REVERSE
 */
typedef k32s GoOrientation;
/** @name    GoOrientation
 *@{*/ 
#define GO_ORIENTATION_WIDE                        (0)   ///< Wide sensor orientation.
#define GO_ORIENTATION_OPPOSITE                    (1)   ///< Opposite sensor orientation.
#define GO_ORIENTATION_REVERSE                     (2)   ///< Reverse sensor orientation.
/**@}*/

/**
 * @struct  GoInputSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a data input source.
 *
 * The following enumerators are defined:
 * - #GO_INPUT_SOURCE_LIVE
 * - #GO_INPUT_SOURCE_RECORDING
 */
typedef k32s GoInputSource;
/** @name    GoInputSource
 *@{*/ 
#define GO_INPUT_SOURCE_LIVE          (0)       ///< The current data input source is from live sensor data.
#define GO_INPUT_SOURCE_RECORDING     (1)       ///< The current data source is from a replay.
/**@}*/

/**
 * @struct  GoSeekDirection
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a playback seek direction.
 *
 * The following enumerators are defined:
 * - #GO_SEEK_DIRECTION_FORWARD
 * - #GO_SEEK_DIRECTION_BACKWARD
 */
typedef k32s GoSeekDirection;
/** @name    GoSeekDirection
 *@{*/ 
#define GO_SEEK_DIRECTION_FORWARD      (0)      ///< Seek forward in the current replay.
#define GO_SEEK_DIRECTION_BACKWARD     (1)      ///< Seek backward in the current replay.
/**@}*/

/**
 * @struct  GoDataSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a data source.
 *
 * The following enumerators are defined:
 *  #GO_DATA_SOURCE_NONE
 *  #GO_DATA_SOURCE_TOP
 *  #GO_DATA_SOURCE_BOTTOM
 *  #GO_DATA_SOURCE_TOP_LEFT
 *  #GO_DATA_SOURCE_TOP_RIGHT
 *  #GO_DATA_SOURCE_TOP_BOTTOM
 *  #GO_DATA_SOURCE_LEFT_RIGHT
 */
typedef k32s GoDataSource;
/** @name    GoDataSource
 *@{*/ 
#define GO_DATA_SOURCE_NONE         (-1)    ///< Used to represent a buddy device when the buddy is not connected
#define GO_DATA_SOURCE_TOP          (0)     ///< Represents main device when in a single sensor or opposite orientation buddy setup. Also represents the combined main and buddy in a wide or reverse orientation
#define GO_DATA_SOURCE_BOTTOM       (1)     ///< Represents the buddy device in an opposite orientation buddy configuration
#define GO_DATA_SOURCE_TOP_LEFT     (2)     ///< Represents the main device in a wide or reverse orientation buddy configuration
#define GO_DATA_SOURCE_TOP_RIGHT    (3)     ///< Represents the buddy device in a wide or reverse orientation buddy configuration
#define GO_DATA_SOURCE_TOP_BOTTOM   (4)     ///< Represents both the main and buddy devices in a opposite orientation
#define GO_DATA_SOURCE_LEFT_RIGHT   (5)     ///< Represents a buddy configuration where data from the two devices are not merged (e.g. buddied 1000 series sensors in a wide layout)
/**@}*/

/**
 * @struct  GoSpacingIntervalType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents spacing interval types.
 *
 * The following enumerators are defined:
 * - #GO_SPACING_INTERVAL_TYPE_MAX_RES
 * - #GO_SPACING_INTERVAL_TYPE_BALANCED
 * - #GO_SPACING_INTERVAL_TYPE_MAX_SPEED
 */
typedef k32s GoSpacingIntervalType;
/** @name    GoSpacingIntervalType
 *@{*/ 
#define GO_SPACING_INTERVAL_TYPE_MAX_RES              (0)   ///< Maximum resolution spacing interval type.
#define GO_SPACING_INTERVAL_TYPE_BALANCED             (1)   ///< Balanced spacing interval type.
#define GO_SPACING_INTERVAL_TYPE_MAX_SPEED            (2)   ///< Maximum speed spacing interval type.
#define GO_SPACING_INTERVAL_TYPE_CUSTOM               (3)   ///< The user specified custom interval.
/**@}*/

/**
 * @struct  GoTriggerSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a trigger source type.
 *
 * The following enumerators are defined:
 * - #GO_TRIGGER_SOURCE_TIME
 * - #GO_TRIGGER_SOURCE_ENCODER
 * - #GO_TRIGGER_SOURCE_INPUT
 * - #GO_TRIGGER_SOURCE_SOFTWARE
 */
typedef k32s GoTriggerSource;
/** @name    GoTriggerSource
 *@{*/ 
#define GO_TRIGGER_SOURCE_TIME                     (0)    ///< Trigger on internal clock.
#define GO_TRIGGER_SOURCE_ENCODER                  (1)    ///< Trigger on encoder.
#define GO_TRIGGER_SOURCE_INPUT                    (2)    ///< Trigger on digital input.
#define GO_TRIGGER_SOURCE_SOFTWARE                 (3)    ///< Trigger on software messages.
/**@}*/

/**
 * @struct  GoAlignmentType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment type.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_TYPE_STATIONARY
 * - #GO_ALIGNMENT_TYPE_MOVING
 */
typedef k32s GoAlignmentType;
/** @name    GoAlignmentType
 *@{*/ 
#define GO_ALIGNMENT_TYPE_STATIONARY                 (0)    ///< Stationary target alignment type.
#define GO_ALIGNMENT_TYPE_MOVING                     (1)    ///< Moving target alignment type.
/**@}*/

/**
 * @struct  GoAlignmentTarget
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a alignment target type.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_TARGET_NONE
 * - #GO_ALIGNMENT_TARGET_DISK
 * - #GO_ALIGNMENT_TARGET_BAR
 * - #GO_ALIGNMENT_TARGET_PLATE
 */
typedef k32s GoAlignmentTarget;
/** @name    GoAlignmentTarget
 *@{*/ 
#define GO_ALIGNMENT_TARGET_NONE                 (0)                    ///< No calibration target.
#define GO_ALIGNMENT_TARGET_DISK                 (1)                    ///< Calibration disk.
#define GO_ALIGNMENT_TARGET_BAR                  (2)                    ///< Calibration bar.
#define GO_ALIGNMENT_TARGET_PLATE                (3)                    ///< Calibration plate.
/**@}*/


/**
 * @struct  GoReplayExportSourceType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the replay export source type.
 *
 * The following enumerators are defined:
 * - #GO_REPLAY_EXPORT_SOURCE_PRIMARY
 * - #GO_REPLAY_EXPORT_SOURCE_INTENSITY
 */
typedef k32s GoReplayExportSourceType;
/** @name    GoReplayExportSourceType
 *@{*/ 
#define GO_REPLAY_EXPORT_SOURCE_PRIMARY                     (0) ///< Primary data(relevant to the current scan mode) replay export.
#define GO_REPLAY_EXPORT_SOURCE_INTENSITY                   (1) ///< Intensity data replay export.
/**@}*/

/**
 * @struct  GoFamily
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the supported Gocator hardware families.
 *
 * The following enumerators are defined:
 * - #GO_FAMILY_1000
 * - #GO_FAMILY_2000
 * - #GO_FAMILY_3000
 */
typedef k32s GoFamily;
/** @name    GoFamily
 *@{*/ 
#define GO_FAMILY_1000              (0) ///< 1x00 series sensors.
#define GO_FAMILY_2000              (1) ///< 2x00 series sensors.
#define GO_FAMILY_3000              (2) ///< 3x00 series sensors.
/**@}*/

/**
 * @struct  GoDecision
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the measurement output decision values. Bit 0 represents the decision value, while bits 1 through 7 represent the decision code, outlined by GoDecisionCode.
 * @see     GoDecisionCode
 *
 * The following enumerators are defined:
 * - #GO_DECISION_FAIL
 * - #GO_DECISION_PASS
 */
typedef k8u GoDecision;
/** @name    GoDecision
 *@{*/ 
#define GO_DECISION_FAIL            (0)     ///< The measurement value is either valid and falls outside the defined passing decision range or is invalid. The failure error code can be used to determine whether the value was valid.
#define GO_DECISION_PASS            (1)     ///< The measurement value is valid and it falls within the defined passing decision range.
/**@}*/


/**
 * @struct  GoDecisionCode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the possible measurement decision codes.
 *
 * The following enumerators are defined:
 * - #GO_DECISION_CODE_OK
 * - #GO_DECISION_CODE_INVALID_ANCHOR
 * - #GO_DECISION_CODE_INVALID_VALUE
 */
typedef k8u GoDecisionCode;
/** @name    GoDecisionCode
 *@{*/ 
#define GO_DECISION_CODE_OK              (0)    ///< The measurement value is valid and it falls outside the defined passing decision range.
#define GO_DECISION_CODE_INVALID_VALUE   (1)    ///< The measurement value is invalid.
#define GO_DECISION_CODE_INVALID_ANCHOR  (2)    ///< The tool associated with the measurement is anchored is has received invalid measurement data from its anchoring source(s).
/**@}*/

/**
 * @struct  GoStates
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Sensor state, login, alignment information, recording state, playback source, uptime, playback information, and auto-start setting state.
 */
typedef struct GoStates
{
    GoState sensorState;                    ///< The state of the sensor.
    GoUser loginType;                       ///< The logged in user.
    GoAlignmentRef alignmentReference;      ///< The alignment reference of the sensor.
    GoAlignmentState alignmentState;        ///< The alignment state of the sensor.
    kBool recordingEnabled;                 ///< The current state of recording on the sensor.
    k32u playbackSource;                    ///< The current playback source of the sensor.
    k32u uptimeSec;                         ///< Sensor uptime in seconds.
    k32u uptimeMicrosec;                    ///< Sensor uptime in microseconds.
    k32u playbackPos;                       ///< The playback position index.
    k32u playbackCount;                     ///< The playback count.
    kBool autoStartEnabled;                 ///< The auto-start enabled state.
} GoStates;

/**
 * @struct  GoAddressInfo
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Sensor network address settings.
 */
typedef struct GoAddressInfo
{
    kBool useDhcp;              ///< Sensor uses DHCP?
    kIpAddress address;         ///< Sensor IP address.
    kIpAddress mask;            ///< Sensor subnet bit-mask.
    kIpAddress gateway;         ///< Sensor gateway address.
} GoAddressInfo;

/**
 * @struct  GoElement64f
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a 64-bit floating point configuration element with a range and enabled state.
 */
typedef struct GoElement64f
{
    kBool enabled;              ///< Represents whether the element value is currently used. (not always applicable)
    k64f systemValue;           ///< The system value. (not always applicable)
    k64f value;                 ///< The element's double field value.
    k64f max;                   ///< The maximum allowable value that can be set for this element.
    k64f min;                   ///< The minimum allowable value that can be set for this element.
} GoElement64f;

/**
 * @struct  GoElement32u
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a 32-bit unsigned integer configuration element with a range and enabled state.
 */
typedef struct GoElement32u
{
    kBool enabled;              ///< Represents whether the element value is currently used.
    k32u systemValue;           ///< The system value. (not always applicable)
    k32u value;                 ///< The element's 32-bit unsigned field value.
    k32u max;                   ///< The maximum allowable value that can be set for this element.
    k32u min;                   ///< The minimum allowable value that can be set for this element.
} GoElement32u;

/**
 * @struct  GoActiveAreaConfig
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an active area configuration element.
 */
typedef struct GoActiveAreaConfig
{
    GoElement64f x;             ///< The X offset of the active area.
    GoElement64f y;             ///< The Y offset of the active area.
    GoElement64f z;             ///< The Z offset of the active area.
    GoElement64f height;        ///< The height of the active area.
    GoElement64f length;        ///< The length of the active area.
    GoElement64f width;         ///< The width of the active area.
} GoActiveAreaConfig;

/**
 * @struct  GoTransformation
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment element.
 */
typedef struct GoTransformation
{
    k64f x;                     ///< The X offset of the transformation.
    k64f y;                     ///< The Y offset of the transformation.
    k64f z;                     ///< The Z offset of the transformation.
    k64f xAngle;                ///< The X angle of the transformation.
    k64f yAngle;                ///< The Y angle of the transformation.
    k64f zAngle;                ///< The Z angle of the transformation.
} GoTransformation;

/**
 * @struct  GoTransformedDataRegion
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a transformed data region.
 */
typedef struct GoTransformedDataRegion
{
    k64f x;                     ///< The X offset of the transformed data region.
    k64f y;                     ///< The Y offset of the transformed data region.
    k64f z;                     ///< The Z offset of the transformed data region.
    k64f width;                 ///< The width of the transformed data region.
    k64f length;                ///< The length of the transformed data region.
    k64f height;                ///< The height of the transformed data region.
} GoTransformedDataRegion;


/**
 * @struct  GoAsciiOperation
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an ASCII protocol operational type.
 *
 * The following enumerators are defined:
 * - #GO_ASCII_OPERATION_ASYNCHRONOUS
 * - #GO_ASCII_OPERATION_POLLING
 */
typedef k32s GoAsciiOperation;
/** @name    GoAsciiOperation
 *@{*/ 
#define GO_ASCII_OPERATION_ASYNCHRONOUS                      (0)          ///< Selected measurement output will be sent upon sensor start.
#define GO_ASCII_OPERATION_POLLING                           (1)          ///< Measurement output will only be sent as requested.
/**@}*/

/**
 * @struct  GoSelcomFormat
 * @extends kValue
 * @ingroup GoSdk-Serial
 * @brief   Represents the selcom format followed on the serial output.
 *
 * The following enumerators are defined:
 * - #GO_SELCOM_FORMAT_SLS
 * - #GO_SELCOM_FORMAT_12BIT_ST
 * - #GO_SELCOM_FORMAT_14BIT
 * - #GO_SELCOM_FORMAT_14BIT_ST
 */
typedef k32s GoSelcomFormat;
/** @name    GoSelcomFormat
 *@{*/ 
#define GO_SELCOM_FORMAT_SLS                       (0)                    ///< Selcom uses the SLS format
#define GO_SELCOM_FORMAT_12BIT_ST                  (1)                    ///< Selcom uses the 12-Bit Search/Track format
#define GO_SELCOM_FORMAT_14BIT                     (2)                    ///< Selcom uses the 14-Bit format
#define GO_SELCOM_FORMAT_14BIT_ST                  (3)                    ///< Selcom uses the 14-Bit Search/Track format
/**@}*/

/**
 * @struct  GoSerialProtocol
 * @extends kValue
 * @ingroup GoSdk-Serial
 * @brief   Represents all serial output protocols.
 *
 * The following enumerators are defined:
 * - #GO_SERIAL_PROTOCOL_GOCATOR
 * - #GO_SERIAL_PROTOCOL_SELCOM
 */
typedef k32s GoSerialProtocol;
/** @name    GoSerialProtocol
 *@{*/ 
#define GO_SERIAL_PROTOCOL_GOCATOR               (0)            ///< Gocator serial protocol.
#define GO_SERIAL_PROTOCOL_SELCOM                (1)            ///< Selcom serial protocol.
/**@}*/


/**
 * @struct  GoAnalogTrigger
 * @extends kValue
 * @ingroup GoSdk-Analog
 * @brief   Represents an analog output trigger.
 *
 * The following enumerators are defined:
 * - #GO_ANALOG_TRIGGER_MEASUREMENT
 * - #GO_ANALOG_TRIGGER_SOFTWARE
 */
typedef k32s GoAnalogTrigger;
/** @name    GoAnalogTrigger
 *@{*/ 
#define GO_ANALOG_TRIGGER_MEASUREMENT          (0)  ///< Analog output triggered by measurement data.
#define GO_ANALOG_TRIGGER_SOFTWARE             (1)  ///< Analog output triggered by software.
/**@}*/

/**
 * @struct  GoDigitalPass
 * @extends kValue
 * @ingroup GoSdk-Digital
 * @brief   Represents a digital output condition.
 *
 * The following enumerators are defined:
 * - #GO_DIGITAL_PASS_TRUE
 * - #GO_DIGITAL_PASS_FALSE
 * - #GO_DIGITAL_PASS_ALWAYS
 */
typedef k32s GoDigitalPass;
/** @name    GoDigitalPass
 *@{*/ 
#define GO_DIGITAL_PASS_TRUE       (0)  ///< Digital output triggers when all selected measurements pass.
#define GO_DIGITAL_PASS_FALSE      (1)  ///< Digital output triggers when all selected measurements fail.
#define GO_DIGITAL_PASS_ALWAYS     (2)  ///< Digital output triggers on every scan.
/**@}*/

/**
 * @struct  GoDigitalSignal
 * @extends kValue
 * @ingroup GoSdk-Digital
 * @brief   Represents a digital output signal type.
 *
 * The following enumerators are defined:
 * - #GO_DIGITAL_SIGNAL_PULSED
 * - #GO_DIGITAL_SIGNAL_CONTINUOUS
 */
typedef k32s GoDigitalSignal;
/** @name    GoDigitalSignal
 *@{*/ 
#define GO_DIGITAL_SIGNAL_PULSED               (0)  ///< Digital output is pulsed when triggered.
#define GO_DIGITAL_SIGNAL_CONTINUOUS           (1)  ///< Digital output is continuous when triggered.
/**@}*/

/**
 * @struct  GoDigitalEvent
 * @extends kValue
 * @ingroup GoSdk-Digital
 * @brief   Represents a digital output event.
 *
 * The following enumerators are defined:
 * - #GO_DIGITAL_EVENT_MEASUREMENT
 * - #GO_DIGITAL_EVENT_SOFTWARE
 */
typedef k32s GoDigitalEvent;
/** @name    GoDigitalEvent
 *@{*/ 
#define GO_DIGITAL_EVENT_MEASUREMENT            (1) ///< Digital output is triggered by measurement data.
#define GO_DIGITAL_EVENT_SOFTWARE               (2) ///< Digital output is triggered by software.
/**@}*/

/**
 * @struct  GoAnalogEvent
 * @extends kValue
 * @ingroup GoSdk-Analog
 * @brief   Represents a analog output event.
 *
 * The following enumerators are defined:
 * - #GO_ANALOG_EVENT_MEASURMENT
 * - #GO_ANALOG_EVENT_SOFTWARE
 */
typedef k32s GoAnalogEvent;
/** @name    GoAnalogEvent
 *@{*/ 
#define GO_ANALOG_EVENT_MEASURMENT          (1) ///< Analog output is triggered by measurement data.
#define GO_ANALOG_EVENT_SOFTWARE            (2) ///< Analog output is triggered by software.
/**@}*/

/**
 * @struct  GoEthernetProtocol
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents a ethernet output protocol.
 *
 * The following enumerators are defined:
 * - #GO_ETHERNET_PROTOCOL_GOCATOR
 * - #GO_ETHERNET_PROTOCOL_MODBUS
 * - #GO_ETHERNET_PROTOCOL_ETHERNET_IP
 * - #GO_ETHERNET_PROTOCOL_ASCII
 */
typedef k32s GoEthernetProtocol;
/** @name    GoEthernetProtocol
 *@{*/ 
#define GO_ETHERNET_PROTOCOL_GOCATOR        (0) ///< Gocator ethernet protocol.
#define GO_ETHERNET_PROTOCOL_MODBUS         (1) ///< Modbus ethernet protocol.
#define GO_ETHERNET_PROTOCOL_ETHERNET_IP    (2) ///< EthernetIP ethernet protocol.
#define GO_ETHERNET_PROTOCOL_ASCII          (3) ///< ASCII ethernet protocol.
/**@}*/


/**
 * @struct  GoEndianType
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents an endian output type.
 *
 * The following enumerators are defined:
 * - #GO_ENDIAN_TYPE_BIG
 * - #GO_ENDIAN_TYPE_LITTLE
 */
typedef k32s GoEndianType;
/** @name    GoEndianType
 *@{*/ 
#define GO_ENDIAN_TYPE_BIG                  (0) ///< Big Endian output.
#define GO_ENDIAN_TYPE_LITTLE               (1) ///< Little Endian output.
/**@}*/


/**
 * @struct  GoOutputSource
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents output sources.
 *
 * The following enumerators are defined:
 * - #GO_OUTPUT_SOURCE_NONE
 * - #GO_OUTPUT_SOURCE_VIDEO
 * - #GO_OUTPUT_SOURCE_RANGE
 * - #GO_OUTPUT_SOURCE_PROFILE
 * - #GO_OUTPUT_SOURCE_SURFACE
 * - #GO_OUTPUT_SOURCE_RANGE_INTENSITY
 * - #GO_OUTPUT_SOURCE_PROFILE_INTENSITY
 * - #GO_OUTPUT_SOURCE_SURFACE_INTENSITY
 * - #GO_OUTPUT_SOURCE_MEASUREMENT
 */
typedef k32s GoOutputSource;
/** @name    GoOutputSource
 *@{*/ 
#define GO_OUTPUT_SOURCE_NONE                          (0)  ///< Unknown output source.
#define GO_OUTPUT_SOURCE_VIDEO                         (1)  ///< Output video data.
#define GO_OUTPUT_SOURCE_RANGE                         (2)  ///< Output range data.
#define GO_OUTPUT_SOURCE_PROFILE                       (3)  ///< Output profile data.
#define GO_OUTPUT_SOURCE_SURFACE                       (4)  ///< Output surface data.
#define GO_OUTPUT_SOURCE_RANGE_INTENSITY               (5)  ///< Output range intensity data.
#define GO_OUTPUT_SOURCE_PROFILE_INTENSITY             (6)  ///< Output profile intensity data.
#define GO_OUTPUT_SOURCE_SURFACE_INTENSITY             (7)  ///< Output surface intensity data.
#define GO_OUTPUT_SOURCE_MEASUREMENT                   (8)  ///< Output measurement data.
/**@}*/

/**
 * @struct  GoOutputDelayDomain
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an output delay domain.
 *
 * The following enumerators are defined:
 * - #GO_OUTPUT_DELAY_DOMAIN_TIME
 * - #GO_OUTPUT_DELAY_DOMAIN_ENCODER
 */
typedef k32s GoOutputDelayDomain;
/** @name    GoOutputDelayDomain
 *@{*/ 
#define GO_OUTPUT_DELAY_DOMAIN_TIME                    (0)  ///< Time(uS) based delay domain.
#define GO_OUTPUT_DELAY_DOMAIN_ENCODER                 (1)  ///< Encoder tick delay domain.
/**@}*/

/**
 * @struct  GoPixelType
 * @ingroup GoSdk
 * @brief   Represents a video message pixel type.
 *
 * The following enumerators are defined:
 * - #GO_PIXEL_TYPE_8U
 * - #GO_PIXEL_TYPE_RGB
 */
typedef k32s GoPixelType;
/** @name    GoPixelType
 *@{*/ 
#define GO_PIXEL_TYPE_8U        (0)         ///< Each pixel is represented as unsigned 8-bit values.
#define GO_PIXEL_TYPE_RGB       (1)         ///< Each pixel is represented as three unsigned 8-bit values.
/**@}*/

/**
 * @struct  GoToolType
 * @extends kValue
 * @ingroup GoSdk-Tools
 * @brief   Lists all tool types.
 *
 * The following enumerators are defined:
 * - #GO_TOOL_UNKNOWN
 * - #GO_TOOL_RANGE_POSITION
 * - #GO_TOOL_RANGE_THICKNESS
 * - #GO_TOOL_PROFILE_AREA
 * - #GO_TOOL_PROFILE_BOUNDING_BOX
 * - #GO_TOOL_PROFILE_CIRCLE
 * - #GO_TOOL_PROFILE_DIMENSION
 * - #GO_TOOL_PROFILE_GROOVE
 * - #GO_TOOL_PROFILE_INTERSECT
 * - #GO_TOOL_PROFILE_LINE
 * - #GO_TOOL_PROFILE_PANEL
 * - #GO_TOOL_PROFILE_POSITION
 * - #GO_TOOL_PROFILE_STRIP
 * - #GO_TOOL_SURFACE_BOUNDING_BOX
 * - #GO_TOOL_SURFACE_COUNTERSUNK_HOLE
 * - #GO_TOOL_SURFACE_ELLIPSE
 * - #GO_TOOL_SURFACE_HOLE
 * - #GO_TOOL_SURFACE_OPENING
 * - #GO_TOOL_SURFACE_PLANE
 * - #GO_TOOL_SURFACE_POSITION
 * - #GO_TOOL_SURFACE_STUD
 * - #GO_TOOL_SURFACE_VOLUME
 * - #GO_TOOL_SCRIPT
 */
typedef k32s GoToolType;
/** @name    GoToolType
 *@{*/ 
#define GO_TOOL_UNKNOWN                             (-1)   ///< Unknown tool.
#define GO_TOOL_RANGE_POSITION                      (0)    ///< Range Position tool.
#define GO_TOOL_RANGE_THICKNESS                     (1)    ///< Range Thickness tool.
#define GO_TOOL_PROFILE_AREA                        (2)    ///< Profile Area tool.
#define GO_TOOL_PROFILE_BOUNDING_BOX                (21)   ///< Profile Bounding Box tool.
#define GO_TOOL_PROFILE_CIRCLE                      (3)    ///< Profile Circle tool.
#define GO_TOOL_PROFILE_DIMENSION                   (4)    ///< Profile Dimension tool.
#define GO_TOOL_PROFILE_GROOVE                      (5)    ///< Profile Groove tool.
#define GO_TOOL_PROFILE_INTERSECT                   (6)    ///< Profile Intersect tool.
#define GO_TOOL_PROFILE_LINE                        (7)    ///< Profile Line tool.
#define GO_TOOL_PROFILE_PANEL                       (8)    ///< Profile Panel tool.
#define GO_TOOL_PROFILE_POSITION                    (9)    ///< Profile Position tool.
#define GO_TOOL_PROFILE_STRIP                       (10)   ///< Profile Strip tool.
#define GO_TOOL_SURFACE_BOUNDING_BOX                (11)   ///< Surface Bounding Box tool.
#define GO_TOOL_SURFACE_COUNTERSUNK_HOLE            (20)   ///< Surface Countersunk Hole tool.
#define GO_TOOL_SURFACE_ELLIPSE                     (12)   ///< Surface Ellipse tool.
#define GO_TOOL_SURFACE_HOLE                        (13)   ///< Surface Hole tool.
#define GO_TOOL_SURFACE_OPENING                     (14)   ///< Surface Opening tool.
#define GO_TOOL_SURFACE_PLANE                       (15)   ///< Surface Plane tool.
#define GO_TOOL_SURFACE_POSITION                    (16)   ///< Surface Position tool.
#define GO_TOOL_SURFACE_STUD                        (17)   ///< Surface Stud tool.
#define GO_TOOL_SURFACE_VOLUME                      (18)   ///< Surface Volume tool.
#define GO_TOOL_SCRIPT                              (19)   ///< Script tool.
#define GO_TOOL_EXTENSIBLE                          (1000)   
/**@}*/


/**
 * @struct  GoMeasurementType
 * @extends kValue
 * @ingroup GoSdk-Tools
 * @brief   Lists all measurement types.
 *
 * The following enumerators are defined:
 * - #GO_MEASUREMENT_UNKNOWN
 * - #GO_MEASUREMENT_RANGE_POSITION_Z
 * - #GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS
 * - #GO_MEASUREMENT_PROFILE_AREA_AREA
 * - #GO_MEASUREMENT_PROFILE_AREA_CENTROID_X
 * - #GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_X
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_Z
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z
 * - #GO_MEASUREMENT_PROFILE_GROOVE_X
 * - #GO_MEASUREMENT_PROFILE_GROOVE_Z
 * - #GO_MEASUREMENT_PROFILE_GROOVE_WIDTH
 * - #GO_MEASUREMENT_PROFILE_GROOVE_DEPTH
 * - #GO_MEASUREMENT_PROFILE_INTERSECT_X
 * - #GO_MEASUREMENT_PROFILE_INTERSECT_Z
 * - #GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE
 * - #GO_MEASUREMENT_PROFILE_LINE_STDDEV
 * - #GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN
 * - #GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX
 * - #GO_MEASUREMENT_PROFILE_LINE_PERCENTILE
 * - #GO_MEASUREMENT_PROFILE_PANEL_GAP
 * - #GO_MEASUREMENT_PROFILE_PANEL_FLUSH
 * - #GO_MEASUREMENT_PROFILE_POSITION_X
 * - #GO_MEASUREMENT_PROFILE_POSITION_Z
 * - #GO_MEASUREMENT_PROFILE_STRIP_POSITION_X
 * - #GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z
 * - #GO_MEASUREMENT_PROFILE_STRIP_WIDTH
 * - #GO_MEASUREMENT_PROFILE_STRIP_HEIGHT
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X           
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y           
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z           
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH       
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE 
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE     
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE     
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE
 * - #GO_MEASUREMENT_SURFACE_HOLE_X
 * - #GO_MEASUREMENT_SURFACE_HOLE_Y
 * - #GO_MEASUREMENT_SURFACE_HOLE_Z
 * - #GO_MEASUREMENT_SURFACE_HOLE_RADIUS
 * - #GO_MEASUREMENT_SURFACE_OPENING_X
 * - #GO_MEASUREMENT_SURFACE_OPENING_Y
 * - #GO_MEASUREMENT_SURFACE_OPENING_Z
 * - #GO_MEASUREMENT_SURFACE_OPENING_WIDTH
 * - #GO_MEASUREMENT_SURFACE_OPENING_LENGTH
 * - #GO_MEASUREMENT_SURFACE_OPENING_ANGLE
 * - #GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE
 * - #GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE
 * - #GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET
 * - #GO_MEASUREMENT_SURFACE_POSITION_X
 * - #GO_MEASUREMENT_SURFACE_POSITION_Y
 * - #GO_MEASUREMENT_SURFACE_POSITION_Z
 * - #GO_MEASUREMENT_SURFACE_STUD_BASE_X
 * - #GO_MEASUREMENT_SURFACE_STUD_BASE_Y
 * - #GO_MEASUREMENT_SURFACE_STUD_BASE_Z
 * - #GO_MEASUREMENT_SURFACE_STUD_TIP_X
 * - #GO_MEASUREMENT_SURFACE_STUD_TIP_Y
 * - #GO_MEASUREMENT_SURFACE_STUD_TIP_Z
 * - #GO_MEASUREMENT_SURFACE_STUD_RADIUS
 * - #GO_MEASUREMENT_SURFACE_VOLUME_AREA
 * - #GO_MEASUREMENT_SURFACE_VOLUME_VOLUME
 * - #GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS
 * - #GO_MEASUREMENT_SCRIPT_OUTPUT
 */
typedef k32s GoMeasurementType;
/** @name    GoMeasurementType
 *@{*/ 
#define GO_MEASUREMENT_UNKNOWN                                  (-1)    ///< Unknown measurement.
#define GO_MEASUREMENT_RANGE_POSITION_Z                         (0)     ///< Range Position tool Z measurement.
#define GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS                (1)     ///< Range Thickness tool Thickness measurement.
#define GO_MEASUREMENT_PROFILE_AREA_AREA                        (2)     ///< Profile Area tool Area measurement.
#define GO_MEASUREMENT_PROFILE_AREA_CENTROID_X                  (3)     ///< Profile Area tool Centroid X measurement.
#define GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z                  (4)     ///< Profile Area tool Centroid Z measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X                   (82)    ///< Profile Bounding Box X measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z                   (83)    ///< Profile Bounding Box Z measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT              (84)    ///< Profile Bounding Box Height measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH               (85)    ///< Profile Bounding Box Width measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X            (86)    ///< Profile Bounding Box Global X measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_X                         (5)     ///< Profile Circle tool X measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_Z                         (6)     ///< Profile Circle tool Z measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS                    (7)     ///< Profile Circle tool Radius measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH                  (8)     ///< Profile Dimension tool Width measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT                 (9)     ///< Profile Dimension tool Height measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE               (10)    ///< Profile Dimension tool Distance measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X               (11)    ///< Profile Dimension tool Center X measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z               (12)    ///< Profile Dimension tool Center Z measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_X                         (13)    ///< Profile Groove tool X measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_Z                         (14)    ///< Profile Groove tool Z measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_WIDTH                     (15)    ///< Profile Groove tool Width measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_DEPTH                     (16)    ///< Profile Groove tool Depth measurement.
#define GO_MEASUREMENT_PROFILE_INTERSECT_X                      (17)    ///< Profile Intersect tool X measurement.
#define GO_MEASUREMENT_PROFILE_INTERSECT_Z                      (18)    ///< Profile Intersect tool Z measurement.
#define GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE                  (19)    ///< Profile Intersect tool Angle measurement.
#define GO_MEASUREMENT_PROFILE_LINE_STDDEV                      (20)    ///< Profile Line tool Standard Deviation measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN                   (21)    ///< Profile Line tool Minimum Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX                   (22)    ///< Profile Line tool Maximum Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_PERCENTILE                  (23)    ///< Profile Line tool Percentile measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_GAP                        (24)    ///< Profile Panel tool Gap measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_FLUSH                      (25)    ///< Profile Panel tool Flush measurement.
#define GO_MEASUREMENT_PROFILE_POSITION_X                       (26)    ///< Profile Position tool X measurement.
#define GO_MEASUREMENT_PROFILE_POSITION_Z                       (27)    ///< Profile Position tool Z measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_POSITION_X                 (28)    ///< Profile Strip tool X Position measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z                 (29)    ///< Profile Strip tool Z Position measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_WIDTH                      (30)    ///< Profile Strip tool Width measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_HEIGHT                     (31)    ///< Profile Strip tool Height measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X                   (32)    ///< Surface Bounding Box X measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y                   (33)    ///< Surface Bounding Box Y measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z                   (34)    ///< Surface Bounding Box Z measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE              (35)    ///< Surface Bounding Box Z Angle measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT              (36)    ///< Surface Bounding Box Height measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH               (37)    ///< Surface Bounding Box Width measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH              (38)    ///< Surface Bounding Box Length measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X            (39)    ///< Surface Bounding Box Global X measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y            (40)    ///< Surface Bounding Box Global Y measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE      (41)    ///< Surface Bounding Box Global Z Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X               (42)    ///< Surface Countersunk Hole tool X position measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y               (43)    ///< Surface Countersunk Hole tool Y position measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z               (44)    ///< Surface Countersunk Hole tool Z position measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS    (45)    ///< Surface Countersunk Hole tool Outer Radius measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH           (46)    ///< Surface Countersunk Hole tool Depth measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS    (47)    ///< Surface Countersunk Hole tool Bevel Radius measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE     (48)    ///< Surface Countersunk Hole tool Bevel Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE         (49)    ///< Surface Countersunk Hole tool X Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE         (50)    ///< Surface Countersunk Hole tool Y Angle measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR                    (51)    ///< Surface Ellipse tool Major measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR                    (52)    ///< Surface Ellipse tool Minor measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO                    (53)    ///< Surface Ellipse tool Ratio measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE                   (54)    ///< Surface Ellipse tool Z Angle measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_X                           (55)    ///< Surface Hole tool X measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_Y                           (56)    ///< Surface Hole tool Y measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_Z                           (57)    ///< Surface Hole tool Z measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_RADIUS                      (58)    ///< Surface Hole tool Radius measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_X                        (59)    ///< Surface Opening tool X measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_Y                        (60)    ///< Surface Opening tool Y measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_Z                        (61)    ///< Surface Opening tool Z measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_WIDTH                    (62)    ///< Surface Opening tool Width measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_LENGTH                   (63)    ///< Surface Opening tool Length measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_ANGLE                    (64)    ///< Surface Opening tool Angle measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE                    (65)    ///< Surface Plane tool X Angle measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE                    (66)    ///< Surface Plane tool Y Angle measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET                   (67)    ///< Surface Plane tool Z Offset measurement.
#define GO_MEASUREMENT_SURFACE_POSITION_X                       (68)    ///< Surface Position tool X measurement.
#define GO_MEASUREMENT_SURFACE_POSITION_Y                       (69)    ///< Surface Position tool Y measurement.
#define GO_MEASUREMENT_SURFACE_POSITION_Z                       (70)    ///< Surface Position tool Z measurement.
#define GO_MEASUREMENT_SURFACE_STUD_BASE_X                      (71)    ///< Surface Stud tool Base X measurement.
#define GO_MEASUREMENT_SURFACE_STUD_BASE_Y                      (72)    ///< Surface Stud tool Base Y measurement.
#define GO_MEASUREMENT_SURFACE_STUD_BASE_Z                      (73)    ///< Surface Stud tool Base Z measurement.
#define GO_MEASUREMENT_SURFACE_STUD_TIP_X                       (74)    ///< Surface Stud tool Tip X measurement.
#define GO_MEASUREMENT_SURFACE_STUD_TIP_Y                       (75)    ///< Surface Stud tool Tip Y measurement.
#define GO_MEASUREMENT_SURFACE_STUD_TIP_Z                       (76)    ///< Surface Stud tool Tip Z measurement.
#define GO_MEASUREMENT_SURFACE_STUD_RADIUS                      (77)    ///< Surface Stud tool Radius measurement.
#define GO_MEASUREMENT_SURFACE_VOLUME_AREA                      (78)    ///< Surface Volume tool Area measurement.
#define GO_MEASUREMENT_SURFACE_VOLUME_VOLUME                    (79)    ///< Surface Volume tool Volume measurement.
#define GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS                 (80)    ///< Surface Volume tool Thickness measurement.
#define GO_MEASUREMENT_SCRIPT_OUTPUT                            (81)    ///< Script tool Output.
#define GO_MEASUREMENT_EXTENSIBLE                               (87)    ///< Extensible tool measurement.

/**@}*/


/**
 * @struct  GoDataMessageType
 * @extends kValue
 * @ingroup GoSdk-DataChannel
 * @brief   Lists all data message types.
 *
 * The following enumerators are defined:
 * - #GO_DATA_MESSAGE_TYPE_UNKNOWN
 * - #GO_DATA_MESSAGE_TYPE_STAMP
 * - #GO_DATA_MESSAGE_TYPE_HEALTH
 * - #GO_DATA_MESSAGE_TYPE_VIDEO
 * - #GO_DATA_MESSAGE_TYPE_RANGE
 * - #GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_PROFILE
 * - #GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE
 * - #GO_DATA_MESSAGE_TYPE_SURFACE
 * - #GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_MEASUREMENT
 * - #GO_DATA_MESSAGE_TYPE_ALIGNMENT
 * - #GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL
 */
typedef k32s GoDataMessageType;
/** @name    GoDataMessageType
 *@{*/ 
#define GO_DATA_MESSAGE_TYPE_UNKNOWN                                 (-1)   ///< Unknown message type.
#define GO_DATA_MESSAGE_TYPE_STAMP                                   (0)    ///< Stamp message type.
#define GO_DATA_MESSAGE_TYPE_HEALTH                                  (1)    ///< Health message type.
#define GO_DATA_MESSAGE_TYPE_VIDEO                                   (2)    ///< Video message type.
#define GO_DATA_MESSAGE_TYPE_RANGE                                   (3)    ///< Range message type.
#define GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY                         (4)    ///< Range Intensity message type.
#define GO_DATA_MESSAGE_TYPE_PROFILE                                 (5)    ///< Profile message type.
#define GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY                       (6)    ///< Profile(or Resampled if uniform spacing is enabled) Intensity message type.
#define GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE                       (7)    ///< Resampled Profile message type.
#define GO_DATA_MESSAGE_TYPE_SURFACE                                 (8)    ///< Surface message type.
#define GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY                       (9)    ///< Surface Intensity message type.
#define GO_DATA_MESSAGE_TYPE_MEASUREMENT                             (10)   ///< Measurement message type.
#define GO_DATA_MESSAGE_TYPE_ALIGNMENT                               (11)   ///< Alignment result message type.
#define GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL                            (12)   ///< Exposure AutoSet/Calibration result message type.
#define GO_DATA_MESSAGE_TYPE_EDGE_MATCH                              (16)   ///< Part matching edge algorithm message type.
#define GO_DATA_MESSAGE_TYPE_ELLIPSE_MATCH                           (17)   ///< Part matching ellipse algorithm message type.
#define GO_DATA_MESSAGE_TYPE_BOUNDING_BOX_MATCH                      (18)   ///< Part matching bounding box algorithm message type.
/**@}*/

/**@}*/

/// @cond (Gocator_1x00 || Gocator_2x00)

/**
 * @struct  GoMaterialType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a material acquisition type.
 *
 * The following enumerators are defined:
 * - #GO_MATERIAL_TYPE_CUSTOM
 * - #GO_MATERIAL_TYPE_DIFFUSE
 */
typedef k32s GoMaterialType;
/** @name    GoMaterialType
 *@{*/ 
#define GO_MATERIAL_TYPE_CUSTOM              (0)    ///< Custom material acquisition type.
#define GO_MATERIAL_TYPE_DIFFUSE             (1)    ///< Diffuse material acquisition type.
/**@}*/

/**
 * @struct  GoSpotSelectionType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a spot selection type.
 *
 * The following enumerators are defined:
 * - #GO_SPOT_SELECTION_TYPE_BEST
 * - #GO_SPOT_SELECTION_TYPE_TOP
 * - #GO_SPOT_SELECTION_TYPE_BOTTOM
 */
typedef k32s GoSpotSelectionType;
/** @name    GoSpotSelectionType
 *@{*/ 
#define GO_SPOT_SELECTION_TYPE_BEST              (0)    ///< Select the spot with the best value.
#define GO_SPOT_SELECTION_TYPE_TOP               (1)    ///< Select the top-most spot.
#define GO_SPOT_SELECTION_TYPE_BOTTOM            (2)    ///< Select the bottom-most spot.
/**@}*/

/// @endcond

/// @cond (Gocator_1x00 || Gocator_2x00)
/**
 * @struct  GoProfileStripBaseType
 * @extends kValue
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile strip tool base type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_BASE_TYPE_NONE
 * - #GO_PROFILE_STRIP_BASE_TYPE_FLAT
 */
typedef k32s GoProfileStripBaseType;
/** @name    GoProfileStripBaseType
 *@{*/ 
#define GO_PROFILE_STRIP_BASE_TYPE_NONE     (0)     ///< No strip base type.
#define GO_PROFILE_STRIP_BASE_TYPE_FLAT     (1)     ///< Flat strip base type.
/**@}*/

/**
 * @struct  GoProfileStripEdgeType
 * @extends kValue
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile strip tool edge type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_EDGE_TYPE_RISING
 * - #GO_PROFILE_STRIP_EDGE_TYPE_FALLING
 * - #GO_PROFILE_STRIP_EDGE_TYPE_DATA_END
 * - #GO_PROFILE_STRIP_EDGE_TYPE_VOID
 */
typedef k32s GoProfileStripEdgeType;
/** @name    GoProfileStripEdgeType
 *@{*/ 
#define GO_PROFILE_STRIP_EDGE_TYPE_RISING           (1)     ///< Rising strip edge type.
#define GO_PROFILE_STRIP_EDGE_TYPE_FALLING          (2)     ///< Falling strip edge type.
#define GO_PROFILE_STRIP_EDGE_TYPE_DATA_END         (4)     ///< Data end strip edge type.
#define GO_PROFILE_STRIP_EDGE_TYPE_VOID             (8)     ///< Void strip edge type.
/**@}*/


/**
 * @struct  GoProfileFeatureType
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile feature point type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_FEATURE_TYPE_MAX_Z
 * - #GO_PROFILE_FEATURE_TYPE_MIN_Z
 * - #GO_PROFILE_FEATURE_TYPE_MAX_X
 * - #GO_PROFILE_FEATURE_TYPE_MIN_X
 * - #GO_PROFILE_FEATURE_TYPE_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_AVERAGE
 * - #GO_PROFILE_FEATURE_TYPE_RISING_EDGE
 * - #GO_PROFILE_FEATURE_TYPE_FALLING_EDGE
 * - #GO_PROFILE_FEATURE_TYPE_ANY_EDGE
 * - #GO_PROFILE_FEATURE_TYPE_TOP_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_LEFT_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_MEDIAN
 */
typedef k32s GoProfileFeatureType;         
/** @name    GoProfileFeatureType
 *@{*/ 
#define GO_PROFILE_FEATURE_TYPE_MAX_Z               (0)                     ///< Point with the maximum Z value.
#define GO_PROFILE_FEATURE_TYPE_MIN_Z               (1)                     ///< Point with the minimum Z value.
#define GO_PROFILE_FEATURE_TYPE_MAX_X               (2)                     ///< Point with the maximum X value.
#define GO_PROFILE_FEATURE_TYPE_MIN_X               (3)                     ///< Point with the minimum X value.
#define GO_PROFILE_FEATURE_TYPE_CORNER              (4)                     ///< Dominant corner.
#define GO_PROFILE_FEATURE_TYPE_AVERAGE             (5)                     ///< Average of points.
#define GO_PROFILE_FEATURE_TYPE_RISING_EDGE         (6)                     ///< Rising edge.
#define GO_PROFILE_FEATURE_TYPE_FALLING_EDGE        (7)                     ///< Falling edge.
#define GO_PROFILE_FEATURE_TYPE_ANY_EDGE            (8)                     ///< Rising or falling edge.
#define GO_PROFILE_FEATURE_TYPE_TOP_CORNER          (9)                     ///< Top-most corner.
#define GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER       (10)                    ///< Bottom-most corner.
#define GO_PROFILE_FEATURE_TYPE_LEFT_CORNER         (11)                    ///< Left-most corner.
#define GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER        (12)                    ///< Right-most corner.
#define GO_PROFILE_FEATURE_TYPE_MEDIAN              (13)                    ///< Median of points.
/**@}*/                                       

/**
 * @struct  GoProfileGapAxis
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile gap measurement axis.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GAP_AXIS_EDGE
 * - #GO_PROFILE_GAP_AXIS_SURFACE
 * - #GO_PROFILE_GAP_AXIS_DISTANCE
 */
typedef k32s GoProfileGapAxis;   
/** @name    GoProfileGapAxis
 *@{*/ 
#define GO_PROFILE_GAP_AXIS_EDGE                    (0)                     ///< Measure the gap along the edge normal.
#define GO_PROFILE_GAP_AXIS_SURFACE                 (1)                     ///< Measure the gap along the surface line.  
#define GO_PROFILE_GAP_AXIS_DISTANCE                (2)                     ///< Measure the shortest distance between the two edges.  
/**@}*/

/**
 * @struct  GoProfileEdgeType
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile edge type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_EDGE_TYPE_TANGENT
 * - #GO_PROFILE_EDGE_TYPE_CORNER
 */
typedef k32s GoProfileEdgeType;   
/** @name    GoProfileEdgeType
 *@{*/ 
#define GO_PROFILE_EDGE_TYPE_TANGENT                (0)                     ///< Detect the edge by looking for the tangent.
#define GO_PROFILE_EDGE_TYPE_CORNER                 (1)                     ///< Detect the edge by looking for the corner.    
/**@}*/

/**
 * @struct  GoProfileLineType
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines whether to use a line based on a Profile Line fit, or based on the x-axis.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_BASELINE_TYPE_X_AXIS
 * - #GO_PROFILE_BASELINE_TYPE_Z_AXIS
 * - #GO_PROFILE_BASELINE_TYPE_LINE
 */
typedef k32s GoProfileBaseline;  
/** @name    GoProfileBaseline
 *@{*/ 
#define GO_PROFILE_BASELINE_TYPE_X_AXIS                 (0)             ///< Use the X-Axis.
#define GO_PROFILE_BASELINE_TYPE_Z_AXIS                 (1)             ///< Use the Z-Axis.
#define GO_PROFILE_BASELINE_TYPE_LINE                   (2)             ///< Use the line fit.
/**@}*/

/**
 * @struct  GoProfileAreaType
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines how to calculate profile area
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_AREA_TYPE_OBJECT
 * - #GO_PROFILE_AREA_TYPE_CLEARANCE
 */
typedef k32s GoProfileAreaType;  
/** @name    GoProfileAreaType
 *@{*/ 
#define GO_PROFILE_AREA_TYPE_OBJECT                 (0)                     ///< Sum the profile area that is above the line.
#define GO_PROFILE_AREA_TYPE_CLEARANCE              (1)                     ///< Sum the profile area that is below the line.
/**@}*/

/**
 * @struct  GoProfileGapSide
 * @ingroup GoSdk-ProfileTools
 * @brief   Selects which edge to use as the reference in a panel tool.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_PANEL_SIDE_LEFT
 * - #GO_PROFILE_PANEL_SIDE_RIGHT
 */
typedef k32s GoProfilePanelSide;
/** @name    GoProfilePanelSide
 *@{*/ 
#define GO_PROFILE_PANEL_SIDE_LEFT                  (0)                     ///< Use the left edge.
#define GO_PROFILE_PANEL_SIDE_RIGHT                 (1)                     ///< Use the right edge.
/**@}*/

/**
 * @struct  GoProfileGrooveShape
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile edge type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GROOVE_SHAPE_U
 * - #GO_PROFILE_GROOVE_SHAPE_V
 * - #GO_PROFILE_GROOVE_SHAPE_OPEN
 */
typedef k32s GoProfileGrooveShape;  
/** @name    GoProfileGrooveShape
 *@{*/ 
#define GO_PROFILE_GROOVE_SHAPE_U                   (0)                     ///< Detect grooves that are U shaped.
#define GO_PROFILE_GROOVE_SHAPE_V                   (1)                     ///< Detect grooves that are V shaped.
#define GO_PROFILE_GROOVE_SHAPE_OPEN                (2)                     ///< Detect grooves that are open.
/**@}*/

/**
 * @struct  GoProfileGrooveSelectType
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which groove to select when multiple are present.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH
 * - #GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX
 * - #GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX
 */
typedef k32s GoProfileGrooveSelectType;
/** @name    GoProfileGrooveSelectType
 *@{*/ 
#define GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH           (0)             ///< Select the groove with the maximum depth.
#define GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX          (1)             ///< Select the groove with the currently selected index starting from the left side.
#define GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX         (2)             ///< Select the groove with the currently selected index starting from the right side.
/**@}*/

/**
 * @struct  GoProfileGrooveLocation
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which groove position to return.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GROOVE_LOCATION_BOTTOM
 * - #GO_PROFILE_GROOVE_LOCATION_LEFT
 * - #GO_PROFILE_GROOVE_LOCATION_RIGHT
 */
typedef k32s GoProfileGrooveLocation;
/** @name    GoProfileGrooveLocation
 *@{*/ 
#define GO_PROFILE_GROOVE_LOCATION_BOTTOM           (0)                     ///< Return the position of the bottom of the groove.
#define GO_PROFILE_GROOVE_LOCATION_LEFT             (1)                     ///< Return the position of the left corner of the groove.
#define GO_PROFILE_GROOVE_LOCATION_RIGHT            (2)                     ///< Return the position of the right corner of the groove.
/**@}*/

/**
 * @struct  GoProfileStripSelectType
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which Strip to select when multiple are present.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_SELECT_TYPE_BEST
 * - #GO_PROFILE_STRIP_SELECT_TYPE_LEFT_INDEX
 * - #GO_PROFILE_STRIP_SELECT_TYPE_RIGHT_INDEX
 */
typedef k32s GoProfileStripSelectType;
/** @name    GoProfileStripSelectType
 *@{*/ 
#define GO_PROFILE_STRIP_SELECT_TYPE_BEST                (0)             ///< Select the best strip.
#define GO_PROFILE_STRIP_SELECT_TYPE_LEFT_INDEX          (1)             ///< Select the strip with the currently selected index starting from the left side.
#define GO_PROFILE_STRIP_SELECT_TYPE_RIGHT_INDEX         (2)             ///< Select the strip with the currently selected index starting from the right side.
/**@}*/

/**
 * @struct  GoProfileStripLocation
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which Strip position to return.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_LOCATION_LEFT
 * - #GO_PROFILE_STRIP_LOCATION_RIGHT
 * - #GO_PROFILE_STRIP_LOCATION_BOTTOM
 */
typedef k32s GoProfileStripLocation;
/** @name    GoProfileStripLocation
 *@{*/ 
#define GO_PROFILE_STRIP_LOCATION_LEFT             (0)         ///< Return the position of the left corner of the Strip.
#define GO_PROFILE_STRIP_LOCATION_RIGHT            (1)         ///< Return the position of the right corner of the Strip.
#define GO_PROFILE_STRIP_LOCATION_BOTTOM           (2)         ///< Return the position of the center of the Strip.
/**@}*/

/**
* @struct  GoProfileGenerationType
* @extends kValue
* @ingroup GoSdk-Profile
* @brief   Represents a profile generation type.
*
* The following enumerators are defined:
* - #GO_PROFILE_GENERATION_TYPE_CONTINUOUS
* - #GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH
* - #GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH
* - #GO_PROFILE_GENERATION_TYPE_ROTATIONAL
*/
typedef k32s GoProfileGenerationType;
/** @name    GoProfileGenerationType
*@{*/
#define GO_PROFILE_GENERATION_TYPE_CONTINUOUS           (0) ///< Continuous Profile generation.
#define GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH         (1) ///< Fixed length Profile generation.
#define GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH      (2) ///< Variable length Profile generation.
#define GO_PROFILE_GENERATION_TYPE_ROTATIONAL           (3) ///< Rotational Profile generation.
/**@}*/

/**
* @struct  GoProfileGenerationStartTrigger
* @extends kValue
* @ingroup GoSdk-Profile
* @brief   Represents a profile generation start trigger.
*
* The following enumerators are defined:
* - #GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL
* - #GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL
*/
typedef k32s GoProfileGenerationStartTrigger;
/** @name    GoProfileGenerationStartTrigger
*@{*/
#define GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL  (0) ///< Sequential start trigger.
#define GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL     (1) ///< Digital input start trigger.
/**@}*/

/// @endcond

/// @cond (Gocator_2x00 || Gocator_3x00)

/**
 * @struct  GoPartFrameOfReference
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a part detection frame of reference.
 *
 * The following enumerators are defined:
 * - #GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR
 * - #GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN
 * - #GO_PART_FRAME_OF_REFERENCE_TYPE_PART
 */
typedef k32s GoPartFrameOfReference;
/** @name    GoPartFrameOfReference
 *@{*/ 
#define GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR                (0)   ///< Sensor frame of reference. 2x00 only.
#define GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN                  (0)   ///< Scan frame of reference. 3x00 only. Value duplication is intentional.
#define GO_PART_FRAME_OF_REFERENCE_TYPE_PART                  (1)   ///< Part frame of reference.
/**@}*/

/**
 * @struct  GoPartHeightThresholdDirection
 * @extends kValue
 * @ingroup GoSdk-Surface
 * @brief   Represents a part detection height threshold direction.
 *
 * The following enumerators are defined:
 * - #GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE
 * - #GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW
 */
typedef k32s GoPartHeightThresholdDirection;
/** @name    GoPartHeightThresholdDirection
 *@{*/ 
#define GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE                 (0)    ///< Height threshold direction is above the Z-axis.
#define GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW                 (1)    ///< Height threshold direction is below the Z-axis.
/**@}*/

/**
 * @struct  GoSurfaceGenerationType
 * @extends kValue
 * @ingroup GoSdk-Surface
 * @brief   Represents a surface generation type.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_GENERATION_TYPE_CONTINUOUS
 * - #GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH
 * - #GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH
 * - #GO_SURFACE_GENERATION_TYPE_ROTATIONAL
 */
typedef k32s GoSurfaceGenerationType;
/** @name    GoSurfaceGenerationType
 *@{*/ 
#define GO_SURFACE_GENERATION_TYPE_CONTINUOUS           (0) ///< Continuous surface generation.
#define GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH         (1) ///< Fixed length surface generation.
#define GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH      (2) ///< Variable length surface generation.
#define GO_SURFACE_GENERATION_TYPE_ROTATIONAL           (3) ///< Rotational surface generation.
/**@}*/

/**
 * @struct  GoSurfaceGenerationStartTrigger
 * @extends kValue
 * @ingroup GoSdk-Surface
 * @brief   Represents a surface generation start trigger.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL
 * - #GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL
 */
typedef k32s GoSurfaceGenerationStartTrigger;
/** @name    GoSurfaceGenerationStartTrigger
 *@{*/ 
#define GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL  (0) ///< Sequential start trigger.
#define GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL     (1) ///< Digital input start trigger.
/**@}*/

/**
 * @struct  GoSurfaceLocation
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface location.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_LOCATION_TYPE_MAX
 * - #GO_SURFACE_LOCATION_TYPE_MIN
 * - #GO_SURFACE_LOCATION_TYPE_2D_CENTROID
 * - #GO_SURFACE_LOCATION_TYPE_3D_CENTROID
 * - #GO_SURFACE_LOCATION_TYPE_AVG
 * - #GO_SURFACE_LOCATION_TYPE_MEDIAN
 */
typedef k32s GoSurfaceLocation;
/** @name    GoSurfaceLocation
 *@{*/ 
#define GO_SURFACE_LOCATION_TYPE_MAX                (0)         ///< Location based on the maximum point.
#define GO_SURFACE_LOCATION_TYPE_MIN                (1)         ///< Location based on the minimum point.
#define GO_SURFACE_LOCATION_TYPE_2D_CENTROID        (2)         ///< Location based on a 2d centroid.
#define GO_SURFACE_LOCATION_TYPE_3D_CENTROID        (3)         ///< Location based on a 3d centroid.
#define GO_SURFACE_LOCATION_TYPE_AVG                (4)         ///< Location based on the average point.
#define GO_SURFACE_LOCATION_TYPE_MEDIAN             (5)         ///< Location based on the median point.
/**@}*/

/**
 * @struct  GoSurfaceFeatureType
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface feature type.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_FEATURE_TYPE_AVERAGE
 * - #GO_SURFACE_FEATURE_TYPE_CENTROID
 * - #GO_SURFACE_FEATURE_TYPE_X_MAX
 * - #GO_SURFACE_FEATURE_TYPE_X_MIN
 * - #GO_SURFACE_FEATURE_TYPE_Y_MAX
 * - #GO_SURFACE_FEATURE_TYPE_Y_MIN
 * - #GO_SURFACE_FEATURE_TYPE_Z_MAX
 * - #GO_SURFACE_FEATURE_TYPE_Z_MIN
 * - #GO_SURFACE_FEATURE_TYPE_MEDIAN
 */
typedef k32s GoSurfaceFeatureType;
/** @name    GoSurfaceFeatureType
 *@{*/ 
#define GO_SURFACE_FEATURE_TYPE_AVERAGE            (0)          ///< Feature based on the average.
#define GO_SURFACE_FEATURE_TYPE_CENTROID           (1)          ///< Feature based on the centroid.
#define GO_SURFACE_FEATURE_TYPE_X_MAX              (2)          ///< Feature based on the X maximum point.
#define GO_SURFACE_FEATURE_TYPE_X_MIN              (3)          ///< Feature based on the X minimum point.
#define GO_SURFACE_FEATURE_TYPE_Y_MAX              (4)          ///< Feature based on the Y maximum point.
#define GO_SURFACE_FEATURE_TYPE_Y_MIN              (5)          ///< Feature based on the Y minimum point.
#define GO_SURFACE_FEATURE_TYPE_Z_MAX              (6)          ///< Feature based on the Z maximum point.
#define GO_SURFACE_FEATURE_TYPE_Z_MIN              (7)          ///< Feature based on the Z minimum point.
#define GO_SURFACE_FEATURE_TYPE_MEDIAN             (8)          ///< Feature based on the median.
/**@}*/

/**
 * @struct  GoSurfaceOpeningType
 * @extends kValue
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface opening tool type.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT
 * - #GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE
 */
typedef k32s GoSurfaceOpeningType;
/** @name    GoSurfaceOpeningType
 *@{*/ 
#define GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT            (0) ///< Rounded slot opening type.
#define GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE       (1) ///< Rectangular opening type.
/**@}*/

/**
 * @struct  GoPartMatchAlgorithm
 * @extends kValue
 * @ingroup GoSdk-Surface
 * @brief   Represents a part matching algorithm.
 *
 * The following enumerators are defined:
 * - #GO_PART_MATCH_ALGORITHM_EDGE
 * - #GO_PART_MATCH_ALGORITHM_BOUNDING_BOX
 * - #GO_PART_MATCH_ALGORITHM_ELLIPSE
 */
typedef k32s GoPartMatchAlgorithm;
/** @name    GoPartMatchAlgorithm
 *@{*/ 
#define GO_PART_MATCH_ALGORITHM_EDGE                      (0)   ///< Edge based part match algorithm.
#define GO_PART_MATCH_ALGORITHM_BOUNDING_BOX              (1)   ///< Bounding box based part match algorithm.
#define GO_PART_MATCH_ALGORITHM_ELLIPSE                   (2)   ///< Ellipse based part match algorithm.
/**@}*/

#define GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS     (2)     ///< The maximum number of reference regions permitted for the Surface Counter Sunk Hole Tool.
#define GO_SURFACE_HOLE_MAX_REF_REGIONS                 (2)     ///< The maximum number of reference regions permitted for the Surface Hole Tool.
#define GO_SURFACE_OPENING_MAX_REF_REGIONS              (2)     ///< The maximum number of reference regions permitted for the Surface Opening Tool.
#define GO_SURFACE_PLANE_MAX_REGIONS                    (2)     ///< The maximum number of reference regions permitted for the Surface Plane Tool.
#define GO_SURFACE_STUD_MAX_REF_REGIONS                 (2)     ///< The maximum number of reference regions permitted for the Surface Stud Tool.

/**
 * @struct  GoImageType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an image type.
 *
 * The following enumerators are defined:
 * - #GO_IMAGE_TYPE_HEIGHTMAP
 * - #GO_IMAGE_TYPE_INTENSITY
 */
typedef k32s GoImageType;
/** @name    GoImageType
 *@{*/ 
#define GO_IMAGE_TYPE_HEIGHTMAP                         (0)     ///< Heightmap image type.
#define GO_IMAGE_TYPE_INTENSITY                         (1)     ///< Intensity image type.
/**@}*/

/// @endcond


/**
 * @struct  GoGammaType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a material gamma type.
 *
 * The following enumerators are defined:
 * - #GO_GAMMA_TYPE_NONE
 * - #GO_GAMMA_TYPE_LOW
 * - #GO_GAMMA_TYPE_MEDIUM
 * - #GO_GAMMA_TYPE_HIGH
 */
typedef k32s GoGammaType;
/** @name    GoGammaType
 *@{*/ 
#define GO_GAMMA_TYPE_NONE                              (0)     ///< None. No imager gamma / multi-slope configuration will occur.
#define GO_GAMMA_TYPE_LOW                               (1)     ///< Low. 
#define GO_GAMMA_TYPE_MEDIUM                            (2)     ///< Medium. 
#define GO_GAMMA_TYPE_HIGH                              (3)     ///< High.
/**@}*/

/**
 * @struct  GoPatternSequenceType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a pattern sequence type.
 *
 * The following enumerators are defined:
 * - #GO_PATTERN_SEQUENCE_TYPE_DEFAULT
 * - #GO_PATTERN_SEQUENCE_TYPE_CUSTOM
 * 
 */
typedef k32s GoPatternSequenceType;
/** @name    GoPatternSequenceType
 *@{*/ 

#define GO_PATTERN_SEQUENCE_TYPE_DEFAULT                (0)     ///< Default sequence pattern.
#define GO_PATTERN_SEQUENCE_TYPE_CUSTOM                 (100)   ///< Custom sequence pattern.
/**@}*/


/**
 * @struct  GoImplicitTriggerOverride
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an EthernetIP implicit messaging trigger override.
 *
 * The following enumerators are defined:
 * - #GO_IMPLICIT_TRIGGER_OVERRIDE_OFF
 * - #GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC
 * - #GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE
 * 
 */
typedef k32s GoImplicitTriggerOverride;
/** @name    GoImplicitTriggerOverride
 *@{*/ 

#define GO_IMPLICIT_TRIGGER_OVERRIDE_OFF                (0)     ///< Use the implicit output trigger specified in the connection header.
#define GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC             (1)     ///< Utilize cyclic implicit messaging trigger behavior regardless of what is specified in the connection header.
#define GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE    (2)     ///< Utilize change of state implicit messaging trigger behavior regardless of what is specified in the connection header.
/**@}*/

/**
 * @struct  GoAlignmentStatus
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the operation status of an alignment.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_STATUS_OK
 * - #GO_ALIGNMENT_STATUS_GENERAL_FAILURE
 * - #GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA
 * - #GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA
 * - #GO_ALIGNMENT_STATUS_INVALID_TARGET
 * - #GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION
 * - #GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND
 * - #GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE
 * - #GO_ALIGNMENT_STATUS_ABORT
 * - #GO_ALIGNMENT_STATUS_TIMEOUT
 * - #GO_ALIGNMENT_STATUS_INVALID_PARAMETER
 * 
 */
typedef k32s GoAlignmentStatus;
/** @name    GoAlignmentStatus
 *@{*/ 

#define GO_ALIGNMENT_STATUS_OK                              (1)                 ///< Alignment operation succeeded.
#define GO_ALIGNMENT_STATUS_GENERAL_FAILURE                 (0)                 ///< Alignment operation failed.
#define GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA              (-1)                ///< Stationary alignment failed due to no data being received. Please ensure the target is in range.
#define GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA        (-2)                ///< Moving alignment failed due to insufficient data. 
#define GO_ALIGNMENT_STATUS_INVALID_TARGET                  (-3)                ///< Invalid target detected. Examples include the target dimensions being too small, the target touches both sides of the field of view, or there is insufficient data after some internal filtering.
#define GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION      (-4)                ///< Target detected in an unexpected position. Please ensure the target is stable and there are no obstructions.
#define GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND              (-5)                ///< No reference hole was found during bar alignment. Please ensure the holes can be seen and that the target parameters match their physical dimensions.
#define GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE        (-6)                ///< No change in encoder value occurred during moving alignment. Please ensure the encoder is connected and the target is moving.
#define GO_ALIGNMENT_STATUS_ABORT                           (kERROR_ABORT)      ///< The alignment was aborted by the user.
#define GO_ALIGNMENT_STATUS_TIMEOUT                         (kERROR_TIMEOUT)    ///< The alignment timed out.
#define GO_ALIGNMENT_STATUS_INVALID_PARAMETER               (kERROR_PARAMETER)  ///< The alignment failed due to incorrected parameters.
/**@}*/

kEndHeader()
#include <GoSdk/GoSdkDef.x.h>

#endif
