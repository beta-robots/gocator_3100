/** 
 * @file    GoSdkDef.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DEF_X_H
#define GO_SDK_DEF_X_H

kBeginHeader()

#define GO_SDK_PROTOCOL_VERSION    "101.4.0.0"

#define GO_ROLE_INVALID              (-1)       // Internal role: sensor role not yet determined.  
#define GO_STATE_CONNECTED           (3)        // Internal state: sensor connected, but state is otherwise unknown. 

#define GO_MEASUREMENT_UNASSIGNED_ID                    (-1)

typedef kObject GoData;


/**
 * @struct  GoAsciiConfig
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an ASCII protocol configuration element.
 */
typedef struct GoAsciiConfig
{
    GoAsciiOperation operation;     ///< only used for Ethernet output
    k32u controlPort;               ///< only used for Ethernet output
    k32u dataPort;                  ///< only used for Ethernet output
    k32u healthPort;                ///< only used for Ethernet output
    kString customFormat;
    kBool customFormatEnabled;
    kString delimiter;
    kString terminator;
    kString invalidValue;
} GoAsciiConfig;

/**
 * @struct  GoEipConfig
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents an EthernetIP protocol configuration element.
 */
typedef struct GoEipConfig
{
    kBool bufferEnabled;
    kBool implicitOutputEnabled;
    GoEndianType endianOutputType;
    GoImplicitTriggerOverride implicitTriggerOverride;
} GoEipConfig;

/**
 * @struct  GoModbusConfig
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents a Modbus protocol configuration element.
 */
typedef struct GoModbusConfig
{
    kBool bufferEnabled;
} GoModbusConfig;


/**
 * @struct  GoSelcomConfig
 * @extends kValue
 * @ingroup GoSdk-Serial
 * @brief   Represents a Selcom protocol configuration element.
 */
typedef struct GoSelcomConfig
{
    GoSelcomFormat format;
    kArrayList formatOptions;
    k32u rate;
    kArrayList rateOptions;
    k64f dataScaleMin;
    k64f dataScaleMax;
} GoSelcomConfig;

kDeclareEnum(Go, GoState, kValue)

GoFx(kBool) GoState_IsConnected(GoState state); 
GoFx(kBool) GoState_IsResponsive(GoState state); 
GoFx(kBool) GoState_IsReadable(GoState state); 
GoFx(kBool) GoState_IsConfigurable(GoState state); 
GoFx(kBool) GoState_IsNormal(GoState state); 
GoFx(kBool) GoState_ShouldRefresh(GoState state); 

kDeclareEnum(Go, GoAlignmentRef, kValue)
kDeclareEnum(Go, GoAlignmentState, kValue)
kDeclareEnum(Go, GoAlignmentStatus, kValue)
kDeclareEnum(Go, GoAlignmentTarget, kValue)
kDeclareEnum(Go, GoAlignmentType, kValue)
kDeclareEnum(Go, GoAnalogEvent, kValue)
kDeclareEnum(Go, GoAnalogTrigger, kValue)
kDeclareEnum(Go, GoAsciiOperation, kValue)
kDeclareEnum(Go, GoDataMessageType, kValue)
kDeclareEnum(Go, GoDataSource, kValue)
kDeclareEnum(Go, GoDecision, kValue)
kDeclareEnum(Go, GoDecisionCode, kValue)
kDeclareEnum(Go, GoDigitalEvent, kValue)
kDeclareEnum(Go, GoDigitalPass, kValue)
kDeclareEnum(Go, GoDigitalSignal, kValue)
kDeclareEnum(Go, GoEndianType, kValue)
kDeclareEnum(Go, GoEncoderSpacingMinSource, kValue)
kDeclareEnum(Go, GoEncoderTriggerMode, kValue)
kDeclareEnum(Go, GoEthernetProtocol, kValue)
kDeclareEnum(Go, GoExposureMode, kValue)
kDeclareEnum(Go, GoExtMeasurementType, kValue)
kDeclareEnum(Go, GoExtParamType, kValue)
kDeclareEnum(Go, GoFamily, kValue)
kDeclareEnum(Go, GoFrameRateMaxSource, kValue)
kDeclareEnum(Go, GoImageType, kValue)
kDeclareEnum(Go, GoImplicitTriggerOverride, kValue)
kDeclareEnum(Go, GoInputSource, kValue)
kDeclareEnum(Go, GoMode, kValue)
kDeclareEnum(Go, GoOrientation, kValue)
kDeclareEnum(Go, GoOutputDelayDomain, kValue)
kDeclareEnum(Go, GoOutputSource, kValue)
kDeclareEnum(Go, GoPartFrameOfReference, kValue)
kDeclareEnum(Go, GoPartHeightThresholdDirection, kValue)
kDeclareEnum(Go, GoPartMatchAlgorithm, kValue)
kDeclareEnum(Go, GoPatternSequenceType, kValue)
kDeclareEnum(Go, GoPixelType, kValue)
kDeclareEnum(Go, GoProfileAreaType, kValue)
kDeclareEnum(Go, GoProfileBaseline, kValue)
kDeclareEnum(Go, GoProfileEdgeType, kValue)
kDeclareEnum(Go, GoProfileFeatureType, kValue)
kDeclareEnum(Go, GoProfileGapAxis, kValue)
kDeclareEnum(Go, GoProfileGenerationStartTrigger, kValue)
kDeclareEnum(Go, GoProfileGenerationType, kValue)
kDeclareEnum(Go, GoProfileGrooveSelectType, kValue)
kDeclareEnum(Go, GoProfileGrooveShape, kValue)
kDeclareEnum(Go, GoProfilePanelSide, kValue)
kDeclareEnum(Go, GoProfileStripBaseType, kValue)
kDeclareEnum(Go, GoProfileStripEdgeType, kValue)
kDeclareEnum(Go, GoReplayExportSourceType, kValue)
kDeclareEnum(Go, GoRole, kValue)
kDeclareEnum(Go, GoSeekDirection, kValue)
kDeclareEnum(Go, GoSelcomFormat, kValue)
kDeclareEnum(Go, GoSerialProtocol, kValue)
kDeclareEnum(Go, GoSpacingIntervalType, kValue)
kDeclareEnum(Go, GoSurfaceGenerationStartTrigger, kValue)
kDeclareEnum(Go, GoSurfaceGenerationType, kValue)
kDeclareEnum(Go, GoSurfaceLocation, kValue)
kDeclareEnum(Go, GoSurfaceOpeningType, kValue)
kDeclareEnum(Go, GoTrigger, kValue)
kDeclareEnum(Go, GoTriggerUnits, kValue)
kDeclareEnum(Go, GoUser, kValue)

kDeclareValue(Go, GoActiveAreaConfig, kValue)
kDeclareValue(Go, GoAddressInfo, kValue)
kDeclareValue(Go, GoAsciiConfig, kValue)
kDeclareValue(Go, GoEipConfig, kValue)
kDeclareValue(Go, GoElement32u, kValue)
kDeclareValue(Go, GoElement64f, kValue)
kDeclareValue(Go, GoModbusConfig, kValue)
kDeclareValue(Go, GoMeasurementOption, kValue)

GoFx(kBool) GoAddressInfo_VEquals(kType type, const void* value, const void* other); 

typedef struct GoTypePair
{
    kType type;
    k32s enumValue;
} GoTypePair;

kDeclareValue(Go, GoSelcomConfig, kValue)

kDeclareValue(Go, GoStates, kValue)

kDeclareValue(Go, GoTransformation, kValue)
kDeclareValue(Go, GoTransformedDataRegion, kValue)

kDeclareValue(Go, GoUpgradeFxArgs, kValue)

//@cond Private
/**
* @struct  GoExtMeasurementType
* @extends kValue
* @ingroup GoSdk-DataChannel
* @brief   Lists all extensible tool types.
*
* The following enumerators are defined:
* - #GO_EXT_MEASUREMENT_TYPE_GENERIC
* -
*/
typedef k32s GoExtMeasurementType;
/** @name    GoExtMeasurementType
*@{*/
#define GO_EXT_MEASUREMENT_TYPE_GENERIC                             (0)     ///< Generic measurement type.
#define GO_EXT_MEASUREMENT_TYPE_X                                   (1)     ///< X Value measurement type.
#define GO_EXT_MEASUREMENT_TYPE_Y                                   (2)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_Z                                   (3)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_X_ANGLE                             (4)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_Y_ANGLE                             (5)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_Z_ANGLE                             (6)     ///< Surface tool type.
//@endcond

typedef struct GoMeasurementOption
{
    kText64 name;
    kSize minCount;
    kSize maxCount;
} GoMeasurementOption;

typedef k32s GoExtParamType;

#define GO_EXT_PARAM_TYPE_UNKNOWN                      (-1)
#define GO_EXT_PARAM_TYPE_BOOL                         (0)
#define GO_EXT_PARAM_TYPE_INT                          (1)
#define GO_EXT_PARAM_TYPE_FLOAT                        (2)
#define GO_EXT_PARAM_TYPE_STRING                       (3)
#define GO_EXT_PARAM_TYPE_PROFILE_REGION               (4)
#define GO_EXT_PARAM_TYPE_SURFACE_REGION_2D            (5)
#define GO_EXT_PARAM_TYPE_REGION_3D                    (6)


typedef k32s GoContainerType;

#define GO_CONTAINER_TYPE_NONE                              (0)
#define GO_CONTAINER_TYPE_CSV                               (1)
#define GO_CONTAINER_TYPE_ITEMS                             (2)


typedef k32s GoUnitType;

#define GO_UNIT_TYPE_NONE                                   (0)
#define GO_UNIT_TYPE_NANOMETERS                             (1)
#define GO_UNIT_TYPE_MICROMETERS                            (2)
#define GO_UNIT_TYPE_MILLIMETERS                            (3)
#define GO_UNIT_TYPE_CENTIMETERS                            (4)


kEndHeader()

#endif
