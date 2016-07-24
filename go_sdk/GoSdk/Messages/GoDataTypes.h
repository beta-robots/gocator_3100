/** 
 * @file    GoDataTypes.h
 * @brief   Declares Gocator data message classes and related types.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DATA_TYPES_H
#define GO_SDK_DATA_TYPES_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoDataMsg
 * @extends kObject
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a base message sourced from the data channel.
 */
typedef kObject GoDataMsg; 

/** 
 * Returns the message type for a data channel message given in a GoDataSet.
 *
 * @public             @memberof GoDataMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   message    A data channel message.
 * @return             A GoDataMessageType value.
 */
GoFx(GoDataMessageType) GoDataMsg_Type(GoDataMsg message);

/**
 * @struct  GoStamp
 * @extends kValue
 * @ingroup GoSdk-DataChannel
 * @brief   Represents an acquisition stamp. 
 */
typedef struct GoStamp
{
    k64u frameIndex;            ///< Frame index (counts up from zero).
    k64u timestamp;             ///< Timestamp in internal units approximating nanoseconds where the true time in ns = timestamp value / 1.024. 
    k64s encoder;               ///< Position (encoder ticks). 
    k64s encoderAtZ;            ///< Encoder value latched at z-index mark (encoder ticks).
    k64u status;                
    /**<
    * Bit mask containing frame information:
    *
    * - Bit 0: Represents sensor digital input state.
    * - Bit 4: Represents Master digital input state.
    * - Bits 8 and 9: Represents inter-frame digital pulse trigger 
                    (Master digital input if a Master is connected, otherwise 
                    sensor digital input. Value is cleared after each frame 
                    and clamped at 3 if more than 3 pulses are received).
    */
    k32u id;                       ///< Source device ID. 
    k32u reserved32u;              ///< Reserved. 
    k64u reserved64u;              ///< Reserved. 
} GoStamp; 

/**
 * @class   GoStampMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing a set of acquisition stamps. 
 */
typedef GoDataMsg GoStampMsg; 

/** 
 * Gets the stamp source.
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Stamp source. 
 */
GoFx(GoDataSource) GoStampMsg_Source(GoStampMsg msg);

/** 
 * Returns the number of stamps contained in this message. 
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of stamps. 
 */
GoFx(kSize) GoStampMsg_Count(GoStampMsg msg);

/** 
 * Gets the stamp at the specified index.
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Stamp index. 
 * @return             Stamp pointer.
 */
GoFx(GoStamp*) GoStampMsg_At(GoStampMsg msg, kSize index);

/**
 * @class   GoVideoMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a video image. 
 */
typedef GoDataMsg GoVideoMsg; 

/** 
 * Gets the video source.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Video source. 
 */
GoFx(GoDataSource) GoVideoMsg_Source(GoVideoMsg msg);


/** 
 * Gets the camera index that the video data originates from.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Camera index. 
 */
GoFx(kSize) GoVideoMsg_CameraIndex(GoVideoMsg msg);


/** 
 * Gets the image width, in pixels.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Image width, in pixels.
 */
GoFx(kSize) GoVideoMsg_Width(GoVideoMsg msg);

/** 
 * Gets the image height, in pixels.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Image height, in pixels.
 */
GoFx(kSize) GoVideoMsg_Height(GoVideoMsg msg);

/** 
 * Gets the data type used to represent an image pixel. 
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Pixel type.
 */
GoFx(GoPixelType) GoVideoMsg_PixelType(GoVideoMsg msg);

/** 
 * Gets the image pixel size, in bytes.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Pixel size, in bytes.
 */
GoFx(kSize) GoVideoMsg_PixelSize(GoVideoMsg msg);

/** 
 * Gets the pixel format descriptor. 
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Pixel format.
 */
GoFx(kPixelFormat) GoVideoMsg_PixelFormat(GoVideoMsg msg);

/** 
 * Gets the image color filter array. 
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Color filter array.
 */
GoFx(kCfa) GoVideoMsg_Cfa(GoVideoMsg msg);

/** 
 * Gets a pointer to a row within the image.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   rowIndex   Row index. 
 * @return             Row pointer.
 */
GoFx(void*) GoVideoMsg_RowAt(GoVideoMsg msg, kSize rowIndex);

/** 
 * Gets the exposure index.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure index.
 */
GoFx(kSize) GoVideoMsg_ExposureIndex(GoVideoMsg msg);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoVideoMsg_Exposure(GoVideoMsg msg);

/// @cond (Gocator_1x00)
/**
 * @class   GoRangeMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of range data.
 */
typedef GoDataMsg GoRangeMsg; 

/** 
 * Gets the Range source.
 *
 * @public             @memberof GoRangeMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Range source. 
 */
GoFx(GoDataSource) GoRangeMsg_Source(GoRangeMsg msg);

/** 
 * Gets the count of Range data in this message.
 *
 * @public             @memberof GoRangeMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of Range data.
 */
GoFx(kSize) GoRangeMsg_Count(GoRangeMsg msg);


/** 
 * Gets the Range z-resolution, in nanometers.
 *
 * @public             @memberof GoRangeMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z resolution (nm). 
 */
GoFx(k32u) GoRangeMsg_ZResolution(GoRangeMsg msg);

/** 
 * Gets the Range z-offset, in micrometers.
 *
 * @public             @memberof GoRangeMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z offset (um). 
 */
GoFx(k32s) GoRangeMsg_ZOffset(GoRangeMsg msg);

/** 
 * Gets a pointer to Range data.
 *
 * @public             @memberof GoRangeMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Range array index.
 * @return             Range pointer.
 */
GoFx(k16s*) GoRangeMsg_At(GoRangeMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoRangeMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoRangeMsg_Exposure(GoRangeMsg msg);


/**
 * @class   GoRangeIntensityMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of range intensity data.
 */
typedef GoDataMsg GoRangeIntensityMsg; 

/** 
 * Gets the range intensity source.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Range intensity source. 
 */
GoFx(GoDataSource) GoRangeIntensityMsg_Source(GoRangeIntensityMsg msg);

/** 
 * Gets the count of range intensity data in this message.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of range intensity data.
 */
GoFx(kSize) GoRangeIntensityMsg_Count(GoRangeIntensityMsg msg);

/** 
 * Gets a pointer to range intensity data.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Range intensity array index.
 * @return             Range intensity data pointer.
 */
GoFx(k8u*) GoRangeIntensityMsg_At(GoRangeIntensityMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoRangeIntensityMsg_Exposure(GoRangeIntensityMsg msg);

/// @endcond

/// @cond (Gocator_1x00 || Gocator_2x00)
/**
 * @class   GoProfileMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of profile arrays.
 */
typedef GoDataMsg GoProfileMsg; 

/** 
 * Gets the profile source.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Profile source. 
 */
GoFx(GoDataSource) GoProfileMsg_Source(GoProfileMsg msg);

/** 
 * Gets the count of profile arrays in this message.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of profile arrays.
 */
GoFx(kSize) GoProfileMsg_Count(GoProfileMsg msg);

/** 
 * Gets the count of ranges in each profile array.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Range count. 
 */
GoFx(kSize) GoProfileMsg_Width(GoProfileMsg msg);

/** 
 * Gets the profile x-resolution, in nanometers.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X resolution (nm). 
 */
GoFx(k32u) GoProfileMsg_XResolution(GoProfileMsg msg);

/** 
 * Gets the profile z-resolution, in nanometers.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z resolution (nm). 
 */
GoFx(k32u) GoProfileMsg_ZResolution(GoProfileMsg msg);

/** 
 * Gets the profile x-offset, in micrometers.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X offset (um). 
 */
GoFx(k32s) GoProfileMsg_XOffset(GoProfileMsg msg);

/** 
 * Gets the profile z-offset, in micrometers.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z offset (um). 
 */
GoFx(k32s) GoProfileMsg_ZOffset(GoProfileMsg msg);

/** 
 * Gets a pointer to a profile array.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Profile array index.
 * @return             Profile pointer.
 */
GoFx(kPoint16s*) GoProfileMsg_At(GoProfileMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoProfileMsg_Exposure(GoProfileMsg msg);


/** 
 * Gets the source camera index.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.1.3.106
 * @param   msg        Message object. 
 * @return             Camera index (0 - Front camera, 1 - Back camera).
 */
GoFx(k8u) GoProfileMsg_CameraIndex(GoProfileMsg msg);


/**
 * @class   GoResampledProfileMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of re-sampled profile arrays.
 */
typedef GoDataMsg GoResampledProfileMsg; 

/** 
 * Gets the profile source.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Profile source. 
 */
GoFx(GoDataSource) GoResampledProfileMsg_Source(GoResampledProfileMsg msg);

/** 
 * Gets the count of re-sampled profile arrays in this message.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of profile arrays.
 */
GoFx(kSize) GoResampledProfileMsg_Count(GoResampledProfileMsg msg);

/** 
 * Gets the count of points in each re-sampled profile array.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Point count. 
 */
GoFx(kSize) GoResampledProfileMsg_Width(GoResampledProfileMsg msg);

/** 
 * Gets the x-resolution, in nanometers.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X resolution (nm). 
 */
GoFx(k32u) GoResampledProfileMsg_XResolution(GoResampledProfileMsg msg);

/** 
 * Gets the profile z-resolution, in nanometers.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z resolution (nm). 
 */
GoFx(k32u) GoResampledProfileMsg_ZResolution(GoResampledProfileMsg msg);

/** 
 * Gets the profile x-offset, in micrometers.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X offset (um). 
 */
GoFx(k32s) GoResampledProfileMsg_XOffset(GoResampledProfileMsg msg);

/** 
 * Gets the profile z-offset, in micrometers.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z offset (um). 
 */
GoFx(k32s) GoResampledProfileMsg_ZOffset(GoResampledProfileMsg msg);

/** 
 * Gets a pointer to a re-sampled profile array.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Profile array index.
 * @return             Data pointer.
 */
GoFx(k16s*) GoResampledProfileMsg_At(GoResampledProfileMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoResampledProfileMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoResampledProfileMsg_Exposure(GoResampledProfileMsg msg);


/**
 * @class   GoProfileIntensityMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of profile intensity arrays.
 */
typedef GoDataMsg GoProfileIntensityMsg; 

/** 
 * Gets the profile source.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Profile source. 
 */
GoFx(GoDataSource) GoProfileIntensityMsg_Source(GoProfileIntensityMsg msg);

/** 
 * Gets the count of profile intensity arrays in this message.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of profile arrays.
 */
GoFx(kSize) GoProfileIntensityMsg_Count(GoProfileIntensityMsg msg);

/** 
 * Gets the count of intensity values in each profile intensity array.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Intensity count. 
 */
GoFx(kSize) GoProfileIntensityMsg_Width(GoProfileIntensityMsg msg);

/** 
 * Gets the x-resolution, in nanometers.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X resolution (nm). 
 */
GoFx(k32u) GoProfileIntensityMsg_XResolution(GoProfileIntensityMsg msg);

/** 
 * Gets the profile x-offset, in micrometers.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X offset (um). 
 */
GoFx(k32s) GoProfileIntensityMsg_XOffset(GoProfileIntensityMsg msg);

/** 
 * Gets the associated profile type (points or ranges). 
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             kTRUE for ranges; kFALSE for points.
 */
GoFx(kBool) GoProfileIntensityMsg_IsResampled(GoProfileIntensityMsg msg);

/** 
 * Gets a pointer to a profile intensity array.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Profile intensity array index.
 * @return             Data pointer.
 */
GoFx(k8u*) GoProfileIntensityMsg_At(GoProfileIntensityMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoProfileIntensityMsg_Exposure(GoProfileIntensityMsg msg);

/** 
 * Gets the source camera index.
 *
 * @public             @memberof GoProfileMsg
 * @version            Introduced in firmware 4.1.3.106
 * @param   msg        Message object. 
 * @return             Camera index (0 - Front camera, 1 - Back camera).
 */
GoFx(k8u) GoProfileIntensityMsg_CameraIndex(GoProfileMsg msg);

/// @endcond

/// @cond (Gocator_2x00 || Gocator_3x00)
/**
 * @class   GoSurfaceMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a surface array.
 */
typedef GoDataMsg GoSurfaceMsg;

/** 
 * Gets the profile source.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Profile source. 
 */
GoFx(GoDataSource) GoSurfaceMsg_Source(GoSurfaceMsg msg);

/** 
 * Gets the length of the surface (row count).
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Surface length. 
 */
GoFx(kSize) GoSurfaceMsg_Length(GoSurfaceMsg msg);

/** 
 * Gets the width of the surface (column count).
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Surface width; 
 */
GoFx(kSize) GoSurfaceMsg_Width(GoSurfaceMsg msg);

/** 
 * Gets the surface x-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X resolution (nm). 
 */
GoFx(k32u) GoSurfaceMsg_XResolution(GoSurfaceMsg msg);

/** 
 * Gets the surface y-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Y resolution (nm). 
 */
GoFx(k32u) GoSurfaceMsg_YResolution(GoSurfaceMsg msg);

/** 
 * Gets the surface z-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z resolution (nm). 
 */
GoFx(k32u) GoSurfaceMsg_ZResolution(GoSurfaceMsg msg);

/** 
 * Gets the surface x-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X offset (um). 
 */
GoFx(k32s) GoSurfaceMsg_XOffset(GoSurfaceMsg msg);

/** 
 * Gets the surface y-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Y offset (um). 
 */
GoFx(k32s) GoSurfaceMsg_YOffset(GoSurfaceMsg msg);

/** 
 * Gets the surface z-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Z offset (um). 
 */
GoFx(k32s) GoSurfaceMsg_ZOffset(GoSurfaceMsg msg);

/** 
 * Gets the surface z-angle, in microdegrees.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.1.3.106
 * @param   msg        Message object. 
 * @return             Z angle (microdegrees). 
 */
GoFx(k32s) GoSurfaceMsg_ZAngle(GoSurfaceMsg msg);

/** 
* Gets the surface ID.
*
* @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object. 
* @return             Surface ID.
*/
GoFx(k32u) GoSurfaceMsg_SurfaceId(GoSurfaceMsg msg);

/** 
 * Gets a pointer to a surface row.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Surface row index.
 * @return             Row pointer.
 */
GoFx(k16s*) GoSurfaceMsg_RowAt(GoSurfaceMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoSurfaceMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoSurfaceMsg_Exposure(GoSurfaceMsg msg);

/**
 * @class   GoSurfaceIntensityMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a surface intensity array.
 */
typedef GoDataMsg GoSurfaceIntensityMsg; 

/** 
 * Gets the profile source.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Profile source. 
 */
GoFx(GoDataSource) GoSurfaceIntensityMsg_Source(GoSurfaceIntensityMsg msg);

/** 
 * Gets the length of the surface (row count).
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Surface length. 
 */
GoFx(kSize) GoSurfaceIntensityMsg_Length(GoSurfaceIntensityMsg msg);

/** 
 * Gets the width of the surface (column count).
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Surface width; 
 */
GoFx(kSize) GoSurfaceIntensityMsg_Width(GoSurfaceIntensityMsg msg);

/** 
 * Gets the surface x-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X resolution (nm). 
 */
GoFx(k32u) GoSurfaceIntensityMsg_XResolution(GoSurfaceIntensityMsg msg);

/** 
 * Gets the surface y-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Y resolution (nm). 
 */
GoFx(k32u) GoSurfaceIntensityMsg_YResolution(GoSurfaceIntensityMsg msg);

/** 
 * Gets the surface x-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             X offset (um). 
 */
GoFx(k32s) GoSurfaceIntensityMsg_XOffset(GoSurfaceIntensityMsg msg);

/** 
 * Gets the surface y-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Y offset (um). 
 */
GoFx(k32s) GoSurfaceIntensityMsg_YOffset(GoSurfaceIntensityMsg msg);

/** 
 * Gets the surface ID.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Surface ID.
 */
GoFx(k32s) GoSurfaceIntensityMsg_SurfaceId(GoSurfaceIntensityMsg msg);

/** 
 * Gets a pointer to a surface intensity row.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Surface intensity row index.
 * @return             Data pointer.
 */
GoFx(k8u*) GoSurfaceIntensityMsg_RowAt(GoSurfaceIntensityMsg msg, kSize index);

/** 
 * Gets the exposure.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure in uS.
 */
GoFx(k32u) GoSurfaceIntensityMsg_Exposure(GoSurfaceIntensityMsg msg);

/// @endcond


/**
 * @struct  GoMeasurementData
 * @extends kValue
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a measurement result.
 */
typedef struct GoMeasurementData
{
    k64f value;                      ///< Measurement value. 
    GoDecision decision;             ///< Measurement decision value.
    GoDecisionCode decisionCode;     ///< Measurement decision code - relevant only when the value represents a failure.   
} GoMeasurementData; 

/**
 * Gets the measurement data value.
 *
 * @public             @memberof GoMeasurementData
 * @version             Introduced in firmware 4.2.4.7
 * @param   data       Data object.
 * @return             Measurement value.
 */
GoFx(k64f) GoMeasurementData_Value(GoMeasurementData data);

/**
 * Gets the measurement data decision.
 *
 * @public             @memberof GoMeasurementData
 * @version             Introduced in firmware 4.2.4.7
 * @param   data       Data object.
 * @return             Measurement decision.
 */
GoFx(GoDecision) GoMeasurementData_Decision(GoMeasurementData data);

/**
 * Gets the measurement decision code.
 *
 * @public             @memberof GoMeasurementData
 * @version             Introduced in firmware 4.2.4.7
 * @param   data       Data object.
 * @return             Measurement decision code.
 */
GoFx(GoDecisionCode) GoMeasurementData_DecisionCode(GoMeasurementData data);


/**
 * @class   GoMeasurementMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing a set of GoMeasurementData. 
 */
typedef GoDataMsg GoMeasurementMsg; 

/** 
 * Gets the measurement identifier.
 *
 * @public             @memberof GoMeasurementMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Measurement identifier. 
 */
GoFx(k16u) GoMeasurementMsg_Id(GoMeasurementMsg msg);

/** 
 * Count of measurements in this message. 
 *
 * @public             @memberof GoMeasurementMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Count of measurements. 
 */
GoFx(kSize) GoMeasurementMsg_Count(GoMeasurementMsg msg);

/** 
 * Gets the measurement at the specified index.
 *
 * @public             @memberof GoMeasurementMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @param   index      Measurement index. 
 * @return             Measurement pointer.
 */
GoFx(GoMeasurementData*) GoMeasurementMsg_At(GoMeasurementMsg msg, kSize index);


/**
 * @class   GoAlignMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing an alignment result. 
 */
typedef GoDataMsg GoAlignMsg; 

/** 
 * Gets the alignment result.
 *
 * @public             @memberof GoAlignMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Alignment result.
 */
GoFx(kStatus) GoAlignMsg_Status(GoAlignMsg msg);

/**
 * @class   GoExposureCalMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing exposure calibration results. 
 */
typedef GoDataMsg GoExposureCalMsg; 

/** 
 * Gets the exposure calibration result.
 *
 * @public             @memberof GoExposureCalMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Exposure calibration result.
 */
GoFx(kStatus) GoExposureCalMsg_Status(GoExposureCalMsg msg);

/** 
 * Gets the calibrated exposure.
 *
 * @public             @memberof GoExposureCalMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object. 
 * @return             Calibrated exposure value in uS.
 */
GoFx(k32u) GoExposureCalMsg_Exposure(GoExposureCalMsg msg);


/**
 * @class   GoEdgeMatchMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing edge based part matching results. 
 */
typedef GoDataMsg GoEdgeMatchMsg; 

/**
 * Gets the edge match decision.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match decision.
 */
GoFx(k8u) GoEdgeMatchMsg_Decision(GoEdgeMatchMsg msg);

/**
 * Gets the edge match X offset.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match X offset.
 */
GoFx(k64f) GoEdgeMatchMsg_XOffset(GoEdgeMatchMsg msg);

/**
 * Gets the edge match Y offset.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match Y offset.
 */
GoFx(k64f) GoEdgeMatchMsg_YOffset(GoEdgeMatchMsg msg);

/**
 * Gets the edge match Z angle.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match Z angle.
 */
GoFx(k64f) GoEdgeMatchMsg_ZAngle(GoEdgeMatchMsg msg);

/**
 * Gets the edge match quality value.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match quality value.
 */
GoFx(k64f) GoEdgeMatchMsg_QualityValue(GoEdgeMatchMsg msg);

/**
 * Gets the edge match quality decision.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match quality decision.
 */
GoFx(k8u) GoEdgeMatchMsg_QualityDecision(GoEdgeMatchMsg msg);


/**
 * @class   GoEllipseMatchMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing ellipse based part matching results. 
 */
typedef GoDataMsg GoEllipseMatchMsg;

/**
 * Gets the ellipse match decision.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match quality decision.
 */
GoFx(k8u) GoEllipseMatchMsg_Decision(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match X offset.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match X offset.
 */
GoFx(k64f) GoEllipseMatchMsg_XOffset(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match Y offset.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match Y offset.
 */
GoFx(k64f) GoEllipseMatchMsg_YOffset(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match Z angle.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match Z angle.
 */
GoFx(k64f) GoEllipseMatchMsg_ZAngle(GoEllipseMatchMsg msg);

/**
* Gets the ellipse match minor value.
*
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match minor value.
 */
GoFx(k64f) GoEllipseMatchMsg_MinorValue(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match minor decision.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match minor decision.
 */
GoFx(k8u) GoEllipseMatchMsg_MinorDecision(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match major value.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match major value.
 */
GoFx(k64f) GoEllipseMatchMsg_MajorValue(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match major decision.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match major decision.
 */
GoFx(k8u) GoEllipseMatchMsg_MajorDecision(GoEllipseMatchMsg msg);


/**
 * @class   GoBoundingBoxMatchMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing bounding box based part matching results. 
 */
typedef GoDataMsg GoBoundingBoxMatchMsg; 

/**
 * Gets the bounding box match major value.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match major value.
 */
GoFx(k8u) GoBoundingBoxMatchMsg_Decision(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match X offset.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match X offset.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_XOffset(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match Y offset.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match Y offset.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_YOffset(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match Z angle.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match Z angle.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_ZAngle(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match length value.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match length value.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_LengthValue(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match length decision.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match length decision.
 */
GoFx(k8u) GoBoundingBoxMatchMsg_LengthDecision(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match width value.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match width value.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_WidthValue(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match width decision.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match width decision.
 */
GoFx(k8u) GoBoundingBoxMatchMsg_WidthDecision(GoBoundingBoxMatchMsg msg);

kEndHeader()
#include <GoSdk/Messages/GoDataTypes.x.h>

#endif
