/** 
 * @file    GoSensorInfo.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SENSOR_INFO_X_H
#define GO_SDK_SENSOR_INFO_X_H

#include <GoSdk/GoSensorInfo.h>
kBeginHeader()

typedef struct GoSensorInfoClass
{
    kObjectClass base; 

    k32u id; 
    kVersion firmwareVersion; 
    kText32 modelName; 
    GoRole role; 
    k32u buddyId; 
    GoState state; 
    kIpAddress address;
} GoSensorInfoClass; 

kDeclareClass(Go, GoSensorInfo, kObject)

#define GoSensorInfo_Cast_(CONTEXT)    kCastClass_(GoSensorInfo, CONTEXT)

GoFx(kStatus) GoSensorInfo_Construct(GoSensorInfo* info, kAlloc allocator);
GoFx(kStatus) GoSensorInfo_Init(GoSensorInfo info, kType type, kAlloc alloc);
GoFx(kStatus) GoSensorInfo_VRelease(GoSensorInfo info);
GoFx(kStatus) GoSensorInfo_Read(GoSensorInfo info, kSerializer serializer);

/** 
 * @deprecated Gets the alignment state of the device which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              Alignment state.
 * @see                 GoSensor_AlignmentState, GoSensor_States
 */
GoFx(GoAlignmentState) GoSensorInfo_TransformState(GoSensorInfo info);

kEndHeader()

#endif
