/** 
 * @file    GoSensorInfo.h
 * @brief   Declares the GoSensorInfo class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SENSOR_INFO_H
#define GO_SDK_SENSOR_INFO_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()
#include <kApi/Io/kSerializer.h>

/**
 * @class   GoSensorInfo
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents read-only sensor information.
 */
typedef kObject GoSensorInfo; 


/** 
 * Gets the source ID of the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              The information source sensor ID.
 */
GoFx(k32u) GoSensorInfo_Id(GoSensorInfo info);

/** 
 * Gets the firmware version of the sensor which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              The firmware version.
 */
GoFx(kVersion) GoSensorInfo_Firmware(GoSensorInfo info);

/** 
 * Gets the model string of the sensor which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @param   model       A character array pointer.
 * @param   capacity    The character array capacity.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensorInfo_Model(GoSensorInfo info, kChar* model, k32u capacity);

/** 
 * Gets the role of the sensor which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              The sensor role.
 */
GoFx(GoRole) GoSensorInfo_Role(GoSensorInfo info);

/** 
 * Indicates whether the device providing the sensor information has a buddy device connected.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              kTRUE if the device has a buddy and kFALSE otherwise.
 */
GoFx(kBool) GoSensorInfo_HasBuddy(GoSensorInfo info);

/** 
 * Gets the ID of the buddy device which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              The buddy device ID. 0 is returned if there is no buddy assigned.
 */
GoFx(k32u) GoSensorInfo_BuddyId(GoSensorInfo info);

/** 
 * Gets the state of the device which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              Device state.
 */
GoFx(GoState) GoSensorInfo_State(GoSensorInfo info);

kEndHeader()

#include <GoSdk/GoSensorInfo.x.h>

#endif
