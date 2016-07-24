/** 
 * @file    GoDataTypes.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DATA_TYPES_X_H
#define GO_SDK_DATA_TYPES_X_H

kBeginHeader()
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArray2.h>
#include <kApi/Data/kImage.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kSerializer.h>

typedef struct GoDataMsgVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(GoDataMsg msg, kType type, kAlloc allocator); 
} GoDataMsgVTable; 

typedef struct GoDataMsgClass
{
    kObjectClass base;
    GoDataMessageType typeId;
} GoDataMsgClass; 

kDeclareVirtualClass(Go, GoDataMsg, kObject)

#define GoDataMsg_Cast_(CONTEXT)        kCastClass_(GoDataMsg, CONTEXT)
#define GoDataMsg_VTable_(MODE)         (kCast(GoDataMsgVTable*, kType_VTable_(kObject_Type_(MODE))))

GoFx(kStatus) GoDataMsg_Construct(GoDataMsg* msg, kAlloc allocator);
GoFx(kStatus) GoDataMsg_VInit(GoDataMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoDataMsg_Init(GoDataMsg msg, kType type, GoDataMessageType typeId, kAlloc alloc);
GoFx(kStatus) GoDataMsg_VInitClone(GoDataMsg msg, GoDataMsg source, kAlloc alloc);
GoFx(kStatus) GoDataMsg_VRelease(GoDataMsg msg);

/* 
 * GoStamp
 */

kDeclareValue(Go, GoStamp, kValue)

/* 
 * GoStampMsg
 */

#define GO_STAMP_MSG_STAMP_SIZE_1_56         (56)   

typedef struct GoStampMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;            // stamp source
    kArray1 stamps;                 // stamps (kArray1<GoStamp>)
} GoStampMsgClass; 

kDeclareClass(Go, GoStampMsg, GoDataMsg)

GoFx(kStatus) GoStampMsg_Construct(GoStampMsg* msg, kAlloc allocator);
GoFx(kStatus) GoStampMsg_VInit(GoStampMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoStampMsg_Allocate(GoStampMsg msg, kSize count);
GoFx(kStatus) GoStampMsg_VInitClone(GoStampMsg msg, GoStampMsg source, kAlloc alloc); 
GoFx(kStatus) GoStampMsg_VRelease(GoStampMsg msg); 
GoFx(kSize) GoStampMsg_VSize(GoStampMsg msg); 
GoFx(kStatus) GoStampMsg_WriteV1(GoStampMsg msg, kSerializer serializer);
GoFx(kStatus) GoStampMsg_ReadV1(GoStampMsg msg, kSerializer serializer, kAlloc alloc);

#define GoStampMsg_(D)                      kCast(GoStampMsgClass*, D)
#define GoStampMsg_SetContent_(D, V)        (GoStampMsg_(D)->stamps = (V), kOK)
#define GoStampMsg_Content_(D)              (GoStampMsg_(D)->stamps)
#define GoStampMsg_SetSource_(D, V)         (GoStampMsg_(D)->source = (V), kOK)


/* 
 * GoVideoMsg
 */

#define GO_VIDEO_MSG_ATTR_2_20           (20)

typedef struct GoVideoMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                  // image source
    kSize cameraIndex;                    // camera index
    kImage content;                       // image content
    kSize exposureIndex;                  // exposure index
    k32u exposure;                      // exposure (nS)
} GoVideoMsgClass; 

kDeclareClass(Go, GoVideoMsg, GoDataMsg)

GoFx(kStatus) GoVideoMsg_Construct(GoVideoMsg* msg, kAlloc allocator);
GoFx(kStatus) GoVideoMsg_VInit(GoVideoMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoVideoMsg_VInitClone(GoVideoMsg msg, GoVideoMsg source, kAlloc alloc); 
GoFx(kStatus) GoVideoMsg_Allocate(GoVideoMsg msg, kType pixelType, kSize width, kSize height);
GoFx(kStatus) GoVideoMsg_VRelease(GoVideoMsg msg);
GoFx(kSize) GoVideoMsg_VSize(GoVideoMsg msg); 
GoFx(kStatus) GoVideoMsg_WriteV2(GoVideoMsg msg, kSerializer serializer);
GoFx(kStatus) GoVideoMsg_ReadV2(GoVideoMsg msg, kSerializer serializer, kAlloc alloc);

#define GoVideoMsg_(D)                          kCast(GoVideoMsgClass*, D)
#define GoVideoMsg_SetContent_(D, V)            (GoVideoMsg_(D)->content = (V), kOK)
#define GoVideoMsg_Content_(D)                  (GoVideoMsg_(D)->content)
#define GoVideoMsg_SetSource_(D, V)             (GoVideoMsg_(D)->source = (V), kOK)
#define GoVideoMsg_SetExposureIndex_(D, V)      (GoVideoMsg_(D)->exposureIndex = (V), kOK)
#define GoVideoMsg_SetExposure_(D, V)           (GoVideoMsg_(D)->exposure = (V), kOK)


/* 
 * GoRangeMsg
 */

#define GO_RANGE_MSG_ATTR_3_20           (20)

typedef struct GoRangeMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                   // Range source
    k32u zResolution;                      // z-resolution (nm)
    k32s zOffset;                          // z-offset (um)
    kArray1 content;                       // Range content (kArray1<k16s>)
    k32u exposure;                      // exposure (nS)
} GoRangeMsgClass; 

kDeclareClass(Go, GoRangeMsg, GoDataMsg)

GoFx(kStatus) GoRangeMsg_Construct(GoRangeMsg* msg, kAlloc allocator);
GoFx(kStatus) GoRangeMsg_VInit(GoRangeMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoRangeMsg_VInitClone(GoRangeMsg msg, GoRangeMsg source, kAlloc alloc); 
GoFx(kStatus) GoRangeMsg_Allocate(GoRangeMsg msg, kSize count, kSize width);
GoFx(kStatus) GoRangeMsg_VRelease(GoRangeMsg msg);
GoFx(kSize) GoRangeMsg_VSize(GoRangeMsg msg); 
GoFx(kStatus) GoRangeMsg_WriteV3(GoRangeMsg msg, kSerializer serializer);
GoFx(kStatus) GoRangeMsg_ReadV3(GoRangeMsg msg, kSerializer serializer, kAlloc alloc);

#define GoRangeMsg_(D)                          kCast(GoRangeMsgClass*, D)
#define GoRangeMsg_SetContent_(D, V)            (GoRangeMsg_(D)->content = (V), kOK)
#define GoRangeMsg_Content_(D)                  (GoRangeMsg_(D)->content)
#define GoRangeMsg_SetSource_(D, V)             (GoRangeMsg_(D)->source = (V), kOK)
#define GoRangeMsg_SetZResolution_(D, V)        (GoRangeMsg_(D)->zResolution = (V), kOK)
#define GoRangeMsg_SetZOffset_(D, V)            (GoRangeMsg_(D)->zOffset = (V), kOK)
#define GoRangeMsg_SetExposure_(D, V)           (GoRangeMsg_(D)->exposure = (V), kOK)

/* 
 * GoRangeIntensityMsg
 */

#define GO_RANGE_INTENSITY_MSG_ATTR_4_12           (12)

typedef struct GoRangeIntensityMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                   // RangeIntensity source
    kArray1 content;                       // RangeIntensity content (kArray1<k8u>)
    k32u exposure;
} GoRangeIntensityMsgClass; 

kDeclareClass(Go, GoRangeIntensityMsg, GoDataMsg)

GoFx(kStatus) GoRangeIntensityMsg_Construct(GoRangeIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoRangeIntensityMsg_VInit(GoRangeIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoRangeIntensityMsg_VInitClone(GoRangeIntensityMsg msg, GoRangeIntensityMsg source, kAlloc alloc); 
GoFx(kStatus) GoRangeIntensityMsg_Allocate(GoRangeIntensityMsg msg, kSize count);
GoFx(kStatus) GoRangeIntensityMsg_VRelease(GoRangeIntensityMsg msg);
GoFx(kSize) GoRangeIntensityMsg_VSize(GoRangeIntensityMsg msg); 
GoFx(kStatus) GoRangeIntensityMsg_WriteV4(GoRangeIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoRangeIntensityMsg_ReadV4(GoRangeIntensityMsg msg, kSerializer serializer, kAlloc alloc);

#define GoRangeIntensityMsg_(D)                          kCast(GoRangeIntensityMsgClass*, D)
#define GoRangeIntensityMsg_SetContent_(D, V)            (GoRangeIntensityMsg_(D)->content = (V), kOK)
#define GoRangeIntensityMsg_Content_(D)                  (GoRangeIntensityMsg_(D)->content)
#define GoRangeIntensityMsg_SetSource_(D, V)             (GoRangeIntensityMsg_(D)->source = (V), kOK)
#define GoRangeIntensityMsg_SetExposure_(D, V)           (GoRangeIntensityMsg_(D)->exposure = (V), kOK)


/* 
 * GoProfileMsg
 */

#define GO_PROFILE_MSG_ATTR_5_32           (32)

typedef struct GoProfileMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                // profile source
    k32u xResolution;                      // x-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // profile content (kArray2<kPoint16s>)
    k32u exposure;                        // exposure (nS)
    k8u cameraIndex;                    // camera index (0 - Front, 1 - Back)
} GoProfileMsgClass; 

kDeclareClass(Go, GoProfileMsg, GoDataMsg)

GoFx(kStatus) GoProfileMsg_Construct(GoProfileMsg* msg, kAlloc allocator);
GoFx(kStatus) GoProfileMsg_VInit(GoProfileMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoProfileMsg_VInitClone(GoProfileMsg msg, GoProfileMsg source, kAlloc alloc); 
GoFx(kStatus) GoProfileMsg_Allocate(GoProfileMsg msg, kSize count, kSize width);
GoFx(kStatus) GoProfileMsg_VRelease(GoProfileMsg msg);
GoFx(kSize) GoProfileMsg_VSize(GoProfileMsg msg); 
GoFx(kStatus) GoProfileMsg_WriteV5(GoProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoProfileMsg_ReadV5(GoProfileMsg msg, kSerializer serializer, kAlloc alloc);

#define GoProfileMsg_(D)                          kCast(GoProfileMsgClass*, D)
#define GoProfileMsg_SetContent_(D, V)            (GoProfileMsg_(D)->content = (V), kOK)
#define GoProfileMsg_Content_(D)                  (GoProfileMsg_(D)->content)
#define GoProfileMsg_SetSource_(D, V)             (GoProfileMsg_(D)->source = (V), kOK)
#define GoProfileMsg_SetXResolution_(D, V)        (GoProfileMsg_(D)->xResolution = (V), kOK)
#define GoProfileMsg_SetZResolution_(D, V)        (GoProfileMsg_(D)->zResolution = (V), kOK)
#define GoProfileMsg_SetXOffset_(D, V)            (GoProfileMsg_(D)->xOffset = (V), kOK)
#define GoProfileMsg_SetZOffset_(D, V)            (GoProfileMsg_(D)->zOffset = (V), kOK)
#define GoProfileMsg_SetExposure_(D, V)           (GoProfileMsg_(D)->exposure = (V), kOK)
#define GoProfileMsg_SetCameraIndex_(D, V)        (GoProfileMsg_(D)->cameraIndex = (V), kOK)


/* 
 * GoResampledProfileMsg
 */

#define GO_RESAMPLED_PROFILE_MSG_ATTR_6_32           (32)

typedef struct GoResampledProfileMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                   // profile source
    k32u xResolution;                      // x-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // profile content (kArray2<k16s>)
    k32u exposure;                        // exposure (nS)
} GoResampledProfileMsgClass; 

kDeclareClass(Go, GoResampledProfileMsg, GoDataMsg)

GoFx(kStatus) GoResampledProfileMsg_Construct(GoResampledProfileMsg* msg, kAlloc allocator);
GoFx(kStatus) GoResampledProfileMsg_VInit(GoResampledProfileMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoResampledProfileMsg_VInitClone(GoResampledProfileMsg msg, GoResampledProfileMsg source, kAlloc alloc); 
GoFx(kStatus) GoResampledProfileMsg_Allocate(GoResampledProfileMsg msg, kSize count, kSize width);
GoFx(kStatus) GoResampledProfileMsg_VRelease(GoResampledProfileMsg msg);
GoFx(kSize) GoResampledProfileMsg_VSize(GoResampledProfileMsg msg); 
GoFx(kStatus) GoResampledProfileMsg_WriteV6(GoResampledProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoResampledProfileMsg_ReadV6(GoResampledProfileMsg msg, kSerializer serializer, kAlloc alloc);

#define GoResampledProfileMsg_(D)                          kCast(GoResampledProfileMsgClass*, D)
#define GoResampledProfileMsg_SetContent_(D, V)            (GoResampledProfileMsg_(D)->content = (V), kOK)
#define GoResampledProfileMsg_Content_(D)                  (GoResampledProfileMsg_(D)->content)
#define GoResampledProfileMsg_SetSource_(D, V)             (GoResampledProfileMsg_(D)->source = (V), kOK)
#define GoResampledProfileMsg_SetXResolution_(D, V)        (GoResampledProfileMsg_(D)->xResolution = (V), kOK)
#define GoResampledProfileMsg_SetZResolution_(D, V)        (GoResampledProfileMsg_(D)->zResolution = (V), kOK)
#define GoResampledProfileMsg_SetXOffset_(D, V)            (GoResampledProfileMsg_(D)->xOffset = (V), kOK)
#define GoResampledProfileMsg_SetZOffset_(D, V)            (GoResampledProfileMsg_(D)->zOffset = (V), kOK)
#define GoResampledProfileMsg_SetExposure_(D, V)           (GoResampledProfileMsg_(D)->exposure = (V), kOK)

/* 
 * GoProfileIntensityMsg
 */

#define GO_PROFILE_INTENSITY_MSG_ATTR_7_24           (24)

typedef struct GoProfileIntensityMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                   // profile source
    k32u xResolution;                      // x-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    kArray2 content;                       // intensity content (kArray2<k8u>)
    k32u exposure;                         // exposure (nS)
    k8u cameraIndex;                       // camera index (0 - Front, 1 - Back)
} GoProfileIntensityMsgClass; 

kDeclareClass(Go, GoProfileIntensityMsg, GoDataMsg)

GoFx(kStatus) GoProfileIntensityMsg_Construct(GoProfileIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoProfileIntensityMsg_VInit(GoProfileIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoProfileIntensityMsg_VInitClone(GoProfileIntensityMsg msg, GoProfileIntensityMsg source, kAlloc alloc); 
GoFx(kStatus) GoProfileIntensityMsg_Allocate(GoProfileIntensityMsg msg, kSize height, kSize width);
GoFx(kStatus) GoProfileIntensityMsg_VRelease(GoProfileIntensityMsg msg);
GoFx(kSize) GoProfileIntensityMsg_VSize(GoProfileIntensityMsg msg); 
GoFx(kStatus) GoProfileIntensityMsg_WriteV7(GoProfileIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoProfileIntensityMsg_ReadV7(GoProfileIntensityMsg msg, kSerializer serializer, kAlloc alloc);

#define GoProfileIntensityMsg_(D)                          kCast(GoProfileIntensityMsgClass*, D)
#define GoProfileIntensityMsg_SetContent_(D, V)            (GoProfileIntensityMsg_(D)->content = (V), kOK)
#define GoProfileIntensityMsg_Content_(D)                  (GoProfileIntensityMsg_(D)->content)
#define GoProfileIntensityMsg_SetSource_(D, V)             (GoProfileIntensityMsg_(D)->source = (V), kOK)
#define GoProfileIntensityMsg_SetXResolution_(D, V)        (GoProfileIntensityMsg_(D)->xResolution = (V), kOK)
#define GoProfileIntensityMsg_SetXOffset_(D, V)            (GoProfileIntensityMsg_(D)->xOffset = (V), kOK)
#define GoProfileIntensityMsg_SetExposure_(D, V)           (GoProfileIntensityMsg_(D)->exposure = (V), kOK)
#define GoProfileIntensityMsg_SetCameraIndex_(D, V)        (GoProfileIntensityMsg_(D)->cameraIndex = (V), kOK)

/* 
 * GoSurfaceMsg
 */

#define GO_SURFACE_MSG_ATTR_8_44           (44)

typedef struct GoSurfaceMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                   // surface source
    k32u xResolution;                      // x-resolution (nm)
    k32u yResolution;                      // y-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s yOffset;                          // y-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // surface content (kArray2<k16s>)
    k32u exposure;                         // exposure (nS)
    k32s zAngle;                           // z-angle (micro-degrees)
} GoSurfaceMsgClass; 

kDeclareClass(Go, GoSurfaceMsg, GoDataMsg)

GoFx(kStatus) GoSurfaceMsg_Construct(GoSurfaceMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSurfaceMsg_VInit(GoSurfaceMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSurfaceMsg_VInitClone(GoSurfaceMsg msg, GoSurfaceMsg source, kAlloc alloc); 
GoFx(kStatus) GoSurfaceMsg_Allocate(GoSurfaceMsg msg, kSize length, kSize width);
GoFx(kStatus) GoSurfaceMsg_VRelease(GoSurfaceMsg msg);
GoFx(kSize) GoSurfaceMsg_VSize(GoSurfaceMsg msg); 
GoFx(kStatus) GoSurfaceMsg_WriteV8(GoSurfaceMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfaceMsg_ReadV8(GoSurfaceMsg msg, kSerializer serializer, kAlloc alloc);

#define GoSurfaceMsg_(D)                          kCast(GoSurfaceMsgClass*, D)
#define GoSurfaceMsg_SetContent_(D, V)            (GoSurfaceMsg_(D)->content = (V), kOK)
#define GoSurfaceMsg_Content_(D)                  (GoSurfaceMsg_(D)->content)
#define GoSurfaceMsg_SetSource_(D, V)             (GoSurfaceMsg_(D)->source = (V), kOK)
#define GoSurfaceMsg_SetXResolution_(D, V)        (GoSurfaceMsg_(D)->xResolution = (V), kOK)
#define GoSurfaceMsg_SetYResolution_(D, V)        (GoSurfaceMsg_(D)->yResolution = (V), kOK)
#define GoSurfaceMsg_SetZResolution_(D, V)        (GoSurfaceMsg_(D)->zResolution = (V), kOK)
#define GoSurfaceMsg_SetXOffset_(D, V)            (GoSurfaceMsg_(D)->xOffset = (V), kOK)
#define GoSurfaceMsg_SetYOffset_(D, V)            (GoSurfaceMsg_(D)->yOffset = (V), kOK)
#define GoSurfaceMsg_SetZOffset_(D, V)            (GoSurfaceMsg_(D)->zOffset = (V), kOK)
#define GoSurfaceMsg_SetExposure_(D, V)           (GoSurfaceMsg_(D)->exposure = (V), kOK)
#define GoSurfaceMsg_SetZAngle_(D, V)             (GoSurfaceMsg_(D)->zAngle = (V), kOK)


/* 
 * GoSurfaceIntensityMsg
 */

#define GO_SURFACE_INTENSITY_MSG_ATTR_9_32           (32)

typedef struct GoSurfaceIntensityMsgClass
{
    GoDataMsgClass base; 
    GoDataSource source;                   // surface intensity source
    k32u xResolution;                      // x-resolution (nm)
    k32u yResolution;                      // y-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s yOffset;                          // y-offset (um)
    kArray2 content;                       // intensity content (kArray2<k8u>)
    k32u exposure;                         // exposure (nS)
} GoSurfaceIntensityMsgClass; 

kDeclareClass(Go, GoSurfaceIntensityMsg, GoDataMsg)

GoFx(kStatus) GoSurfaceIntensityMsg_Construct(GoSurfaceIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSurfaceIntensityMsg_VInit(GoSurfaceIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSurfaceIntensityMsg_VInitClone(GoSurfaceIntensityMsg msg, GoSurfaceIntensityMsg source, kAlloc alloc); 
GoFx(kStatus) GoSurfaceIntensityMsg_Allocate(GoSurfaceIntensityMsg msg, kSize count, kSize width);
GoFx(kStatus) GoSurfaceIntensityMsg_VRelease(GoSurfaceIntensityMsg msg);
GoFx(kSize) GoSurfaceIntensityMsg_VSize(GoSurfaceIntensityMsg msg); 
GoFx(kStatus) GoSurfaceIntensityMsg_WriteV9(GoSurfaceIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfaceIntensityMsg_ReadV9(GoSurfaceIntensityMsg msg, kSerializer serializer, kAlloc alloc);

#define GoSurfaceIntensityMsg_(D)                          kCast(GoSurfaceIntensityMsgClass*, D)
#define GoSurfaceIntensityMsg_SetContent_(D, V)            (GoSurfaceIntensityMsg_(D)->content = (V), kOK)
#define GoSurfaceIntensityMsg_Content_(D)                  (GoSurfaceIntensityMsg_(D)->content)
#define GoSurfaceIntensityMsg_SetSource_(D, V)             (GoSurfaceIntensityMsg_(D)->source = (V), kOK)
#define GoSurfaceIntensityMsg_SetXResolution_(D, V)        (GoSurfaceIntensityMsg_(D)->xResolution = (V), kOK)
#define GoSurfaceIntensityMsg_SetYResolution_(D, V)        (GoSurfaceIntensityMsg_(D)->yResolution = (V), kOK)
#define GoSurfaceIntensityMsg_SetXOffset_(D, V)            (GoSurfaceIntensityMsg_(D)->xOffset = (V), kOK)
#define GoSurfaceIntensityMsg_SetYOffset_(D, V)            (GoSurfaceIntensityMsg_(D)->yOffset = (V), kOK)
#define GoSurfaceIntensityMsg_SetExposure_(D, V)            (GoSurfaceIntensityMsg_(D)->exposure = (V), kOK)


/* 
 * GoMeasurementData
 */

kDeclareValue(Go, GoMeasurementData, kValue)

/* 
 * GoMeasurementMsg
 */

#define GO_MEASUREMENT_MSG_MEASUREMENT_SIZE_10_16         (16)

typedef struct GoMeasurementMsgClass
{
    GoDataMsgClass base;
    k16u id;                        // measurement identifier
    kArray1 measurements;           // measurement (kArray1<GoMeasurementData>)
} GoMeasurementMsgClass; 

kDeclareClass(Go, GoMeasurementMsg, GoDataMsg)

GoFx(kStatus) GoMeasurementMsg_Construct(GoMeasurementMsg* msg, kAlloc allocator);
GoFx(kStatus) GoMeasurementMsg_VInit(GoMeasurementMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoMeasurementMsg_Allocate(GoMeasurementMsg msg, kSize count);
GoFx(kStatus) GoMeasurementMsg_VInitClone(GoMeasurementMsg msg, GoMeasurementMsg source, kAlloc alloc); 
GoFx(kStatus) GoMeasurementMsg_VRelease(GoMeasurementMsg msg); 
GoFx(kSize) GoMeasurementMsg_VSize(GoMeasurementMsg msg); 
GoFx(kStatus) GoMeasurementMsg_WriteV10(GoMeasurementMsg msg, kSerializer serializer);
GoFx(kStatus) GoMeasurementMsg_ReadV10(GoMeasurementMsg msg, kSerializer serializer, kAlloc alloc);

#define GoMeasurementMsg_(D)                      kCast(GoMeasurementMsgClass*, D)
#define GoMeasurementMsg_SetContent_(D, V)        (GoMeasurementMsg_(D)->measurements = (V), kOK)
#define GoMeasurementMsg_Content_(D)              (GoMeasurementMsg_(D)->measurements)
#define GoMeasurementMsg_SetId_(D, V)             (GoMeasurementMsg_(D)->id = (V), kOK)


/* 
 * GoCalMsg
 */

#define GO_CAL_MSG_ATTR_SIZE_11_8         (8)

typedef struct GoAlignMsgClass
{
    GoDataMsgClass base;
    k32u opId;
    GoAlignmentStatus status;                 // alignment result
} GoAlignMsgClass; 

kDeclareClass(Go, GoAlignMsg, GoDataMsg)

GoFx(kStatus) GoAlignMsg_Construct(GoAlignMsg* msg, kAlloc allocator);
GoFx(kStatus) GoAlignMsg_VInit(GoAlignMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoAlignMsg_VInitClone(GoAlignMsg msg, GoAlignMsg source, kAlloc alloc); 
GoFx(kStatus) GoAlignMsg_VRelease(GoAlignMsg msg); 
GoFx(kSize) GoAlignMsg_VSize(GoAlignMsg msg); 
GoFx(kStatus) GoAlignMsg_WriteV11(GoAlignMsg msg, kSerializer serializer);
GoFx(kStatus) GoAlignMsg_ReadV11(GoAlignMsg msg, kSerializer serializer, kAlloc alloc);

#define GoAlignMsg_(D)                      kCast(GoAlignMsgClass*, D)
#define GoCalMsg_SetStatus_(D, V)         (GoAlignMsg_(D)->status = (V), kOK)

/* 
 * GoExposureCalMsg
 */

#define GO_EXPOSURE_CAL_MSG_ATTR_SIZE_12_8         (8)

typedef struct GoExposureCalMsgClass
{
    GoDataMsgClass base;
    k32u opId;
    kStatus status;                 // calibration result
    k32u exposure;                  // calibrated value
} GoExposureCalMsgClass; 

kDeclareClass(Go, GoExposureCalMsg, GoDataMsg)

GoFx(kStatus) GoExposureCalMsg_Construct(GoExposureCalMsg* msg, kAlloc allocator);
GoFx(kStatus) GoExposureCalMsg_VInit(GoExposureCalMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoExposureCalMsg_VInitClone(GoExposureCalMsg msg, GoExposureCalMsg source, kAlloc alloc); 
GoFx(kStatus) GoExposureCalMsg_VRelease(GoExposureCalMsg msg); 
GoFx(kSize) GoExposureCalMsg_VSize(GoExposureCalMsg msg); 
GoFx(kStatus) GoExposureCalMsg_WriteV12(GoExposureCalMsg msg, kSerializer serializer);
GoFx(kStatus) GoExposureCalMsg_ReadV12(GoExposureCalMsg msg, kSerializer serializer, kAlloc alloc);

#define GoExposureCalMsg_(D)                      kCast(GoExposureCalMsgClass*, D)
#define GoExposureCalMsg_SetStatus_(D, V)         (GoExposureCalMsg_(D)->status = (V), kOK)
#define GoExposureCalMsg_SetExposure_(D, V)         (GoExposureCalMsg_(D)->exposure = (V), kOK)


/*
 *  GoEdgeMatch
 */

#define GO_EDGE_MATCH_MSG_ATTR_SIZE_16_36         (36)

typedef struct GoEdgeMatchMsgClass
{
    GoDataMsgClass base;

    k8u decision;
    k64f xOffset;
    k64f yOffset;
    k64f zAngle;
    k64f qualityValue;
    k8u qualityDecision;
} GoEdgeMatchMsgClass; 

kDeclareClass(Go, GoEdgeMatchMsg, GoDataMsg)

GoFx(kStatus) GoEdgeMatchMsg_Construct(GoEdgeMatchMsg* msg, kAlloc allocator);
GoFx(kStatus) GoEdgeMatchMsg_VInit(GoEdgeMatchMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoEdgeMatchMsg_VInitClone(GoEdgeMatchMsg msg, GoEdgeMatchMsg source, kAlloc alloc); 
GoFx(kStatus) GoEdgeMatchMsg_VRelease(GoEdgeMatchMsg msg); 
GoFx(kSize) GoEdgeMatchMsg_VSize(GoEdgeMatchMsg msg); 
GoFx(kStatus) GoEdgeMatchMsg_WriteV16(GoEdgeMatchMsg msg, kSerializer serializer);
GoFx(kStatus) GoEdgeMatchMsg_ReadV16(GoEdgeMatchMsg msg, kSerializer serializer, kAlloc alloc);

#define GoEdgeMatchMsg_(D)                       kCast(GoEdgeMatchMsgClass*, D)
#define GoEdgeMatchMsg_SetDecision_(D, V)        (GoEdgeMatchMsg_(D)->decision = (V), kOK)
#define GoEdgeMatchMsg_SetXOffset_(D, V)         (GoEdgeMatchMsg_(D)->xOffset = (V), kOK)
#define GoEdgeMatchMsg_SetYOffset_(D, V)         (GoEdgeMatchMsg_(D)->yOffset = (V), kOK)
#define GoEdgeMatchMsg_SetZAngle_(D, V)          (GoEdgeMatchMsg_(D)->zAngle = (V), kOK)
#define GoEdgeMatchMsg_SetQualityValue_(D, V)    (GoEdgeMatchMsg_(D)->qualityValue = (V), kOK)
#define GoEdgeMatchMsg_SetQualityDecision_(D, V) (GoEdgeMatchMsg_(D)->qualityDecision = (V), kOK)

/*
 *  GoBoundingBoxMatch
 */

#define GO_BOUNDING_BOX_MATCH_MSG_ATTR_SIZE_17_30         (30)

typedef struct GoBoundingBoxMatchMsgClass
{
    GoDataMsgClass base;

    k8s decision;
    k64f xOffset;
    k64f yOffset;
    k64f zAngle;
    k64f lengthValue;
    k8u lengthDecision;
    k64f widthValue;
    k8u widthDecision;
} GoBoundingBoxMatchMsgClass; 

kDeclareClass(Go, GoBoundingBoxMatchMsg, GoDataMsg)

GoFx(kStatus) GoBoundingBoxMatchMsg_Construct(GoBoundingBoxMatchMsg* msg, kAlloc allocator);
GoFx(kStatus) GoBoundingBoxMatchMsg_VInit(GoBoundingBoxMatchMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoBoundingBoxMatchMsg_VInitClone(GoBoundingBoxMatchMsg msg, GoBoundingBoxMatchMsg source, kAlloc alloc); 
GoFx(kStatus) GoBoundingBoxMatchMsg_VRelease(GoBoundingBoxMatchMsg msg); 
GoFx(kSize) GoBoundingBoxMatchMsg_VSize(GoBoundingBoxMatchMsg msg); 
GoFx(kStatus) GoBoundingBoxMatchMsg_WriteV17(GoBoundingBoxMatchMsg msg, kSerializer serializer);
GoFx(kStatus) GoBoundingBoxMatchMsg_ReadV17(GoBoundingBoxMatchMsg msg, kSerializer serializer, kAlloc alloc);

#define GoBoundingBoxMatchMsg_(D)                      kCast(GoBoundingBoxMatchMsgClass*, D)
#define GoBoundingBoxMatchMsg_SetDecision_(D, V)        (GoBoundingBoxMatchMsg_(D)->decision = (V), kOK)
#define GoBoundingBoxMatchMsg_SetXOffset_(D, V)         (GoBoundingBoxMatchMsg_(D)->xOffset = (V), kOK)
#define GoBoundingBoxMatchMsg_SetYOffset_(D, V)         (GoBoundingBoxMatchMsg_(D)->yOffset = (V), kOK)
#define GoBoundingBoxMatchMsg_SetZAngle_(D, V)          (GoBoundingBoxMatchMsg_(D)->zAngle = (V), kOK)
#define GoBoundingBoxMatchMsg_SetLengthValue_(D, V)     (GoBoundingBoxMatchMsg_(D)->lengthValue = (V), kOK)
#define GoBoundingBoxMatchMsg_SetLengthDecision_(D, V)  (GoBoundingBoxMatchMsg_(D)->lengthDecision = (V), kOK)
#define GoBoundingBoxMatchMsg_SetWidthValue_(D, V)      (GoBoundingBoxMatchMsg_(D)->widthValue = (V), kOK)
#define GoBoundingBoxMatchMsg_SetWidthDecision_(D, V)   (GoBoundingBoxMatchMsg_(D)->widthDecision = (V), kOK)



/*
 *  GoEllipseMatch
 */

#define GO_ELLIPSE_MATCH_MSG_ATTR_SIZE_18_30         (30)

typedef struct GoEllipseMatchMsgClass
{
    GoDataMsgClass base;

    k8s decision;
    k64f xOffset;
    k64f yOffset;
    k64f zAngle;
    k64f majorValue;
    k8u majorDecision;
    k64f minorValue;
    k8u minorDecision;
} GoEllipseMatchMsgClass; 

kDeclareClass(Go, GoEllipseMatchMsg, GoDataMsg)

GoFx(kStatus) GoEllipseMatchMsg_Construct(GoEllipseMatchMsg* msg, kAlloc allocator);
GoFx(kStatus) GoEllipseMatchMsg_VInit(GoEllipseMatchMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoEllipseMatchMsg_VInitClone(GoEllipseMatchMsg msg, GoEllipseMatchMsg source, kAlloc alloc); 
GoFx(kStatus) GoEllipseMatchMsg_VRelease(GoEllipseMatchMsg msg); 
GoFx(kSize) GoEllipseMatchMsg_VSize(GoEllipseMatchMsg msg); 
GoFx(kStatus) GoEllipseMatchMsg_WriteV18(GoEllipseMatchMsg msg, kSerializer serializer);
GoFx(kStatus) GoEllipseMatchMsg_ReadV18(GoEllipseMatchMsg msg, kSerializer serializer, kAlloc alloc);

#define GoEllipseMatchMsg_(D)                       kCast(GoEllipseMatchMsgClass*, D)
#define GoEllipseMatchMsg_SetDecision_(D, V)        (GoEllipseMatchMsg_(D)->decision = (V), kOK)
#define GoEllipseMatchMsg_SetXOffset_(D, V)         (GoEllipseMatchMsg_(D)->xOffset = (V), kOK)
#define GoEllipseMatchMsg_SetYOffset_(D, V)         (GoEllipseMatchMsg_(D)->yOffset = (V), kOK)
#define GoEllipseMatchMsg_SetZAngle_(D, V)          (GoEllipseMatchMsg_(D)->zAngle = (V), kOK)
#define GoEllipseMatchMsg_SetMajorValue_(D, V)      (GoEllipseMatchMsg_(D)->majorValue = (V), kOK)
#define GoEllipseMatchMsg_SetMajorDecision_(D, V)   (GoEllipseMatchMsg_(D)->majorDecision = (V), kOK)
#define GoEllipseMatchMsg_SetMinorValue_(D, V)      (GoEllipseMatchMsg_(D)->minorValue = (V), kOK)
#define GoEllipseMatchMsg_SetMinorDecision_(D, V)   (GoEllipseMatchMsg_(D)->minorDecision = (V), kOK)


kEndHeader()

#endif
