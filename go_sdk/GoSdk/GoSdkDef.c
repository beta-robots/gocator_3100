/** 
 * @file    GoSdkDef.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSdkLib.x.h>
#include <stdio.h>

kBeginEnum(Go, GoUser, kValue)
    kAddEnumerator(GoUser, GO_USER_NONE)
    kAddEnumerator(GoUser, GO_USER_ADMIN)
    kAddEnumerator(GoUser, GO_USER_TECH)
kEndEnum()

kBeginEnum(Go, GoState, kValue)
    kAddEnumerator(GoState, GO_STATE_ONLINE)
    kAddEnumerator(GoState, GO_STATE_OFFLINE)
    kAddEnumerator(GoState, GO_STATE_RESETTING)
    kAddEnumerator(GoState, GO_STATE_INCONSISTENT)
    kAddEnumerator(GoState, GO_STATE_UNRESPONSIVE)
    kAddEnumerator(GoState, GO_STATE_CANCELLED)
    kAddEnumerator(GoState, GO_STATE_INCOMPLETE)
    kAddEnumerator(GoState, GO_STATE_BUSY)
    kAddEnumerator(GoState, GO_STATE_READY)
    kAddEnumerator(GoState, GO_STATE_RUNNING)
kEndEnum()

GoFx(kBool) GoState_IsConnected(GoState state)
{
    return (state != GO_STATE_OFFLINE) && 
           (state != GO_STATE_ONLINE) && 
           (state != GO_STATE_RESETTING); 
}

GoFx(kBool) GoState_IsResponsive(GoState state)
{
    return (state != GO_STATE_OFFLINE)      && 
           (state != GO_STATE_ONLINE)       &&
           (state != GO_STATE_RESETTING)    && 
           (state != GO_STATE_CANCELLED)    && 
           (state != GO_STATE_UNRESPONSIVE); 
}

GoFx(kBool) GoState_IsReadable(GoState state)
{
    return (state == GO_STATE_INCOMPLETE)   || 
           (state == GO_STATE_READY)        || 
           (state == GO_STATE_RUNNING); 
}

GoFx(kBool) GoState_IsConfigurable(GoState state)
{
    return (state == GO_STATE_READY); 
}

GoFx(kBool) GoState_IsNormal(GoState state)
{
    return (state == GO_STATE_READY)        || 
           (state == GO_STATE_RUNNING); 
}

GoFx(kBool) GoState_ShouldRefresh(GoState state)
{
    return (state == GO_STATE_OFFLINE)      || 
           (state == GO_STATE_CANCELLED)    || 
           (state == GO_STATE_UNRESPONSIVE) || 
           (state == GO_STATE_INCONSISTENT); 
}

kBeginEnum(Go, GoRole, kValue)
    kAddEnumerator(GoRole, GO_ROLE_MAIN)
    kAddEnumerator(GoRole, GO_ROLE_BUDDY)
kEndEnum()

kBeginEnum(Go, GoAlignmentStatus, kValue)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_OK)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_GENERAL_FAILURE)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_INVALID_TARGET)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_ABORT)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_TIMEOUT)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_INVALID_PARAMETER)
kEndEnum()


kBeginEnum(Go, GoAlignmentRef, kValue)
    kAddEnumerator(GoAlignmentRef, GO_ALIGNMENT_REF_FIXED)
    kAddEnumerator(GoAlignmentRef, GO_ALIGNMENT_REF_DYNAMIC)
kEndEnum()

kBeginEnum(Go, GoAlignmentState, kValue)
    kAddEnumerator(GoAlignmentState, GO_ALIGNMENT_STATE_NOT_ALIGNED)
    kAddEnumerator(GoAlignmentState, GO_ALIGNMENT_STATE_ALIGNED)
kEndEnum()

kBeginEnum(Go, GoDecision, kValue)
    kAddEnumerator(GoDecision, GO_DECISION_FAIL)
    kAddEnumerator(GoDecision, GO_DECISION_PASS)
kEndEnum()

kBeginEnum(Go, GoDecisionCode, kValue)
    kAddEnumerator(GoDecisionCode, GO_DECISION_CODE_OK)
    kAddEnumerator(GoDecisionCode, GO_DECISION_CODE_INVALID_VALUE)
    kAddEnumerator(GoDecisionCode, GO_DECISION_CODE_INVALID_ANCHOR)
kEndEnum()

kBeginEnum(Go, GoMode, kValue)
    kAddEnumerator(GoMode, GO_MODE_UNKNOWN)
    kAddEnumerator(GoMode, GO_MODE_VIDEO)
    kAddEnumerator(GoMode, GO_MODE_RANGE)
    kAddEnumerator(GoMode, GO_MODE_PROFILE)
    kAddEnumerator(GoMode, GO_MODE_SURFACE)
kEndEnum()

kBeginEnum(Go, GoOrientation, kValue)
    kAddEnumerator(GoOrientation, GO_ORIENTATION_WIDE)
    kAddEnumerator(GoOrientation, GO_ORIENTATION_OPPOSITE)
    kAddEnumerator(GoOrientation, GO_ORIENTATION_REVERSE)
kEndEnum()

kBeginEnum(Go, GoExposureMode, kValue)
    kAddEnumerator(GoExposureMode, GO_EXPOSURE_MODE_SINGLE)
    kAddEnumerator(GoExposureMode, GO_EXPOSURE_MODE_MULTIPLE)
    kAddEnumerator(GoExposureMode, GO_EXPOSURE_MODE_DYNAMIC)
kEndEnum()

kBeginEnum(Go, GoAlignmentTarget, kValue)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_NONE)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_BAR)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_DISK)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_PLATE)
kEndEnum()

kBeginEnum(Go, GoImplicitTriggerOverride, kValue)
    kAddEnumerator(GoImplicitTriggerOverride, GO_IMPLICIT_TRIGGER_OVERRIDE_OFF)
    kAddEnumerator(GoImplicitTriggerOverride, GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC)
    kAddEnumerator(GoImplicitTriggerOverride, GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE)
kEndEnum()

kBeginEnum(Go, GoPatternSequenceType, kValue)
    kAddEnumerator(GoPatternSequenceType, GO_PATTERN_SEQUENCE_TYPE_DEFAULT)
    kAddEnumerator(GoPatternSequenceType, GO_PATTERN_SEQUENCE_TYPE_CUSTOM)
kEndEnum()


kBeginEnum(Go, GoDataSource, kValue)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_NONE)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP_LEFT)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP_RIGHT)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP_BOTTOM)
kEndEnum()

kBeginEnum(Go, GoDigitalPass, kValue)
    kAddEnumerator(GoDigitalPass, GO_DIGITAL_PASS_TRUE)
    kAddEnumerator(GoDigitalPass, GO_DIGITAL_PASS_FALSE)
    kAddEnumerator(GoDigitalPass, GO_DIGITAL_PASS_ALWAYS)
kEndEnum()

kBeginEnum(Go, GoTrigger, kValue)
    kAddEnumerator(GoTrigger, GO_TRIGGER_TIME)
    kAddEnumerator(GoTrigger, GO_TRIGGER_ENCODER)
    kAddEnumerator(GoTrigger, GO_TRIGGER_INPUT)
    kAddEnumerator(GoTrigger, GO_TRIGGER_SOFTWARE)
kEndEnum()

kBeginEnum(Go, GoTriggerUnits, kValue)
    kAddEnumerator(GoTriggerUnits, GO_TRIGGER_UNIT_TIME)
    kAddEnumerator(GoTriggerUnits, GO_TRIGGER_UNIT_ENCODER)
kEndEnum()

kBeginEnum(Go, GoEncoderTriggerMode, kValue)
    kAddEnumerator(GoEncoderTriggerMode, GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE)
    kAddEnumerator(GoEncoderTriggerMode, GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE)
    kAddEnumerator(GoEncoderTriggerMode, GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL)
kEndEnum()

kBeginEnum(Go, GoFrameRateMaxSource, kValue)
    kAddEnumerator(GoFrameRateMaxSource, GO_FRAME_RATE_MAX_SOURCE_CAMERA)
    kAddEnumerator(GoFrameRateMaxSource, GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION)
kEndEnum()

kBeginEnum(Go, GoEncoderSpacingMinSource, kValue)
    kAddEnumerator(GoEncoderSpacingMinSource, GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION)
    kAddEnumerator(GoEncoderSpacingMinSource, GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION)
kEndEnum()

kBeginEnum(Go, GoAnalogTrigger, kValue)
    kAddEnumerator(GoAnalogTrigger, GO_ANALOG_TRIGGER_MEASUREMENT)
    kAddEnumerator(GoAnalogTrigger, GO_ANALOG_TRIGGER_SOFTWARE)
kEndEnum()

kBeginEnum(Go, GoDigitalSignal, kValue)
    kAddEnumerator(GoDigitalSignal, GO_DIGITAL_SIGNAL_PULSED)
    kAddEnumerator(GoDigitalSignal, GO_DIGITAL_SIGNAL_CONTINUOUS)
kEndEnum()

kBeginEnum(Go, GoDigitalEvent, kValue)
    kAddEnumerator(GoDigitalEvent, GO_DIGITAL_EVENT_MEASUREMENT)
    kAddEnumerator(GoDigitalEvent, GO_DIGITAL_EVENT_SOFTWARE)
kEndEnum()

kBeginEnum(Go, GoAnalogEvent, kValue)
    kAddEnumerator(GoAnalogEvent, GO_ANALOG_EVENT_MEASURMENT)
    kAddEnumerator(GoAnalogEvent, GO_ANALOG_EVENT_SOFTWARE)
kEndEnum()

kBeginEnum(Go, GoEthernetProtocol, kValue)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_GOCATOR)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_MODBUS)
kEndEnum()

kBeginEnum(Go, GoOutputSource, kValue)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_NONE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_VIDEO)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_RANGE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_PROFILE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_SURFACE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_RANGE_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_PROFILE_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_SURFACE_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_MEASUREMENT)
kEndEnum()

kBeginEnum(Go, GoOutputDelayDomain, kValue)
    kAddEnumerator(GoOutputDelayDomain, GO_OUTPUT_DELAY_DOMAIN_TIME)
    kAddEnumerator(GoOutputDelayDomain, GO_OUTPUT_DELAY_DOMAIN_ENCODER)
kEndEnum()

kBeginEnum(Go, GoPartFrameOfReference, kValue)
    kAddEnumerator(GoPartFrameOfReference, GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR)
    kAddEnumerator(GoPartFrameOfReference, GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN)
    kAddEnumerator(GoPartFrameOfReference, GO_PART_FRAME_OF_REFERENCE_TYPE_PART)
kEndEnum()

kBeginEnum(Go, GoSpacingIntervalType, kValue)
    kAddEnumerator(GoSpacingIntervalType, GO_SPACING_INTERVAL_TYPE_MAX_RES)
    kAddEnumerator(GoSpacingIntervalType, GO_SPACING_INTERVAL_TYPE_BALANCED)
    kAddEnumerator(GoSpacingIntervalType, GO_SPACING_INTERVAL_TYPE_MAX_SPEED)
kEndEnum()

kBeginValue(Go, GoMeasurementOption, kValue)
    kAddField(GoMeasurementOption, kText64, name)
    kAddField(GoMeasurementOption, kSize, minCount)
    kAddField(GoMeasurementOption, kSize, maxCount)
kEndValue()

kBeginValue(Go, GoAddressInfo, kValue)
    kAddField(GoAddressInfo, kBool, useDhcp)
    kAddField(GoAddressInfo, kIpAddress, address)
    kAddField(GoAddressInfo, kIpAddress, mask)
    kAddField(GoAddressInfo, kIpAddress, gateway)

    kAddVMethod(GoAddressInfo, kValue, VEquals)
kEndValue()

GoFx(kBool) GoAddressInfo_VEquals(kType type, const void* value, const void* other)
{
    const GoAddressInfo* a = value; 
    const GoAddressInfo* b = other; 
    kSize i;

    if (a->useDhcp == b->useDhcp)
    {
        for (i = 0; i < 16; i++)
        {
            if (a->address.address[i] != b->address.address[i]
                || a->mask.address[i] != b->mask.address[i]
                || a->gateway.address[i] != b->gateway.address[i])
            {
                return kFALSE;
            }
        }

        return kTRUE;
    }

    return kFALSE;
}

kBeginEnum(Go, GoPartHeightThresholdDirection, kValue)
    kAddEnumerator(GoPartHeightThresholdDirection, GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE)
    kAddEnumerator(GoPartHeightThresholdDirection, GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW)
kEndEnum()

kBeginEnum(Go, GoSelcomFormat, kValue)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_SLS)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_12BIT_ST)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_14BIT)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_14BIT_ST)
kEndEnum()

kBeginEnum(Go, GoProfileGenerationType, kValue)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_CONTINUOUS)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_ROTATIONAL)
kEndEnum()

kBeginEnum(Go, GoSurfaceGenerationType, kValue)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_CONTINUOUS)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_ROTATIONAL)
kEndEnum()

kBeginEnum(Go, GoProfileStripBaseType, kValue)
    kAddEnumerator(GoProfileStripBaseType, GO_PROFILE_STRIP_BASE_TYPE_NONE)
    kAddEnumerator(GoProfileStripBaseType, GO_PROFILE_STRIP_BASE_TYPE_FLAT)
kEndEnum()

kBeginEnum(Go, GoSerialProtocol, kValue)
    kAddEnumerator(GoSerialProtocol, GO_SERIAL_PROTOCOL_GOCATOR)
    kAddEnumerator(GoSerialProtocol, GO_SERIAL_PROTOCOL_SELCOM)
kEndEnum()

kBeginEnum(Go, GoEndianType, kValue)
    kAddEnumerator(GoEndianType, GO_ENDIAN_TYPE_BIG)
    kAddEnumerator(GoEndianType, GO_ENDIAN_TYPE_LITTLE)
kEndEnum()

kBeginEnum(Go, GoAlignmentType, kValue)
    kAddEnumerator(GoAlignmentType, GO_ALIGNMENT_TYPE_STATIONARY)
    kAddEnumerator(GoAlignmentType, GO_ALIGNMENT_TYPE_MOVING)
kEndEnum()

kBeginEnum(Go, GoAsciiOperation, kValue)
    kAddEnumerator(GoAsciiOperation, GO_ASCII_OPERATION_ASYNCHRONOUS)
    kAddEnumerator(GoAsciiOperation, GO_ASCII_OPERATION_POLLING)
kEndEnum()

kBeginEnum(Go, GoProfileGenerationStartTrigger, kValue)
    kAddEnumerator(GoProfileGenerationStartTrigger, GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL)
    kAddEnumerator(GoProfileGenerationStartTrigger, GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL)
kEndEnum()

kBeginEnum(Go, GoSurfaceGenerationStartTrigger, kValue)
    kAddEnumerator(GoSurfaceGenerationStartTrigger, GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL)
    kAddEnumerator(GoSurfaceGenerationStartTrigger, GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL)
kEndEnum()

kBeginEnum(Go, GoInputSource, kValue)
    kAddEnumerator(GoInputSource, GO_INPUT_SOURCE_LIVE)
    kAddEnumerator(GoInputSource, GO_INPUT_SOURCE_RECORDING)
kEndEnum()

kBeginEnum(Go, GoExtMeasurementType, kValue)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_GENERIC)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_X)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Y)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Z)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_X_ANGLE)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Y_ANGLE)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Z_ANGLE)
kEndEnum()


kBeginEnum(Go, GoExtParamType, kValue)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_BOOL)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_FLOAT)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_INT)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_STRING)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_PROFILE_REGION)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_REGION_3D)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_SURFACE_REGION_2D)
kEndEnum()

kBeginEnum(Go, GoFamily, kValue)
    kAddEnumerator(GoFamily, GO_FAMILY_1000)
    kAddEnumerator(GoFamily, GO_FAMILY_2000)
    kAddEnumerator(GoFamily, GO_FAMILY_3000)
kEndEnum()

kBeginEnum(Go, GoSurfaceOpeningType, kValue)
    kAddEnumerator(GoSurfaceOpeningType, GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT)
    kAddEnumerator(GoSurfaceOpeningType, GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE)
kEndEnum()

kBeginEnum(Go, GoDataMessageType, kValue)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_UNKNOWN)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_STAMP)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_HEALTH)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_VIDEO)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_RANGE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_PROFILE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SURFACE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_MEASUREMENT)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_ALIGNMENT)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL)
kEndEnum()

kBeginEnum(Go, GoPixelType, kValue)
    kAddEnumerator(GoPixelType, GO_PIXEL_TYPE_8U)
    kAddEnumerator(GoPixelType, GO_PIXEL_TYPE_RGB)
kEndEnum()

kBeginEnum(Go, GoReplayExportSourceType, kValue)
    kAddEnumerator(GoReplayExportSourceType, GO_REPLAY_EXPORT_SOURCE_PRIMARY)
    kAddEnumerator(GoReplayExportSourceType, GO_REPLAY_EXPORT_SOURCE_INTENSITY)
kEndEnum()

kBeginEnum(Go, GoSeekDirection, kValue)
    kAddEnumerator(GoSeekDirection, GO_SEEK_DIRECTION_FORWARD)
    kAddEnumerator(GoSeekDirection, GO_SEEK_DIRECTION_BACKWARD)
kEndEnum()

kBeginEnum(Go, GoProfileStripEdgeType, kValue)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_RISING)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_FALLING)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_DATA_END)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_VOID)
kEndEnum()

kBeginEnum(Go, GoProfileFeatureType, kValue)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MAX_Z)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MIN_Z)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MAX_X)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MIN_X)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_AVERAGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_RISING_EDGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_FALLING_EDGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_ANY_EDGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_TOP_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_LEFT_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MEDIAN)
kEndEnum()                                          

kBeginEnum(Go, GoProfileGapAxis, kValue)
    kAddEnumerator(GoProfileGapAxis, GO_PROFILE_GAP_AXIS_EDGE)
    kAddEnumerator(GoProfileGapAxis, GO_PROFILE_GAP_AXIS_SURFACE)
    kAddEnumerator(GoProfileGapAxis, GO_PROFILE_GAP_AXIS_DISTANCE)
kEndEnum()              

kBeginEnum(Go, GoProfileEdgeType, kValue)
    kAddEnumerator(GoProfileEdgeType, GO_PROFILE_EDGE_TYPE_TANGENT)
    kAddEnumerator(GoProfileEdgeType, GO_PROFILE_EDGE_TYPE_CORNER)
kEndEnum()   

kBeginEnum(Go, GoProfileGrooveShape, kValue)
    kAddEnumerator(GoProfileGrooveShape, GO_PROFILE_GROOVE_SHAPE_U)
    kAddEnumerator(GoProfileGrooveShape, GO_PROFILE_GROOVE_SHAPE_V)
    kAddEnumerator(GoProfileGrooveShape, GO_PROFILE_GROOVE_SHAPE_OPEN)
kEndEnum()   

kBeginEnum(Go, GoProfileGrooveSelectType, kValue)
    kAddEnumerator(GoProfileGrooveSelectType, GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH)
    kAddEnumerator(GoProfileGrooveSelectType, GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX)
    kAddEnumerator(GoProfileGrooveSelectType, GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX)
kEndEnum()  

kBeginEnum(Go, GoProfileBaseline, kValue)
    kAddEnumerator(GoProfileBaseline, GO_PROFILE_BASELINE_TYPE_LINE)
    kAddEnumerator(GoProfileBaseline, GO_PROFILE_BASELINE_TYPE_X_AXIS)
kEndEnum()  

kBeginEnum(Go, GoProfileAreaType, kValue)
    kAddEnumerator(GoProfileAreaType, GO_PROFILE_AREA_TYPE_OBJECT)
    kAddEnumerator(GoProfileAreaType, GO_PROFILE_AREA_TYPE_CLEARANCE)
kEndEnum()  

kBeginEnum(Go, GoProfilePanelSide, kValue)
    kAddEnumerator(GoProfilePanelSide, GO_PROFILE_PANEL_SIDE_LEFT)
    kAddEnumerator(GoProfilePanelSide, GO_PROFILE_PANEL_SIDE_RIGHT)
kEndEnum()  

kBeginEnum(Go, GoSurfaceLocation, kValue)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_MAX)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_MIN)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_2D_CENTROID)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_3D_CENTROID)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_AVG)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_MEDIAN)
kEndEnum()      

GoFx(kVersion) GoSdk_ProtocolVersion()
{
    kVersion version = 0; 
        
    if (kSuccess(kVersion_Parse(&version, GO_SDK_PROTOCOL_VERSION)))
    {
        return version; 
    }
    else
    {
        return 0; 
    }
}

GoFx(kVersion) GoSdk_Version()
{
    return kVersion_Create(GO_SDK_VERSION_MAJOR, GO_SDK_VERSION_MINOR, GO_SDK_VERSION_RELEASE, GO_SDK_VERSION_BUILD);
}

GoFx(kStatus) GoDestroy(kObject object)
{
    return kObject_Destroy(object);
}
