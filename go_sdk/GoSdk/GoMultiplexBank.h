/// @cond (Gocator_1x00 || Gocator_2x00)

/** 
 * @file    GoMultiplexBank.h
 * @brief   Declares the GoMultiplexBank class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_MULTIPLEXBANK_H
#define GO_MULTIPLEXBANK_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSensor.h>

kBeginHeader()

/**
 * @class   GoMultiplexBank
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a bank of related sensors to be used in multiplexing.
 */
typedef kObject GoMultiplexBank; 

/** 
 * Adds a sensor to the given multiplexing bank.
 *
 * @public                  @memberof GoMultiplexBank
 * @version                 Introduced in firmware 4.0.10.27
 * @param   bank            GoMultiplexBank object.
 * @param   sensor          A connected handle to the sensor to add.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMultiplexBank_AddSensor(GoMultiplexBank bank, GoSensor sensor);

/** 
 * Removes a sensor from the given multiplexing bank.
 *
 * @public                  @memberof GoMultiplexBank
 * @version                 Introduced in firmware 4.0.10.27
 * @param   bank            GoMultiplexBank object.
 * @param   id              The ID of the sensor in the bank to the remove.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoMultiplexBank_RemoveSensor(GoMultiplexBank bank, k32u id);

/** 
 * Returns the count corresponding to the number of sensors in the multiplexing bank.
 *
 * @public                  @memberof GoMultiplexBank
 * @version                 Introduced in firmware 4.0.10.27
 * @param   bank            GoMultiplexBank object.
 * @return                  The multiplexing bank's sensor count.            
 */
GoFx(kSize) GoMultiplexBank_SensorCount(GoMultiplexBank bank);

/** 
 * Gets a sensor handle from the given index.
 *
 * @public                  @memberof GoMultiplexBank
 * @version                 Introduced in firmware 4.0.10.27
 * @param   bank            GoMultiplexBank object.
 * @param   index           Multiplexing bank index from which to obtain a sensor handle.
 * @return                  A GoSensor object.
 */
GoFx(GoSensor) GoMultiplexBank_SensorAt(GoMultiplexBank bank, kSize index);

/** 
 * Returns a boolean value representing whether a sensor is in the multiplexing bank.
 *
 * @public                  @memberof GoMultiplexBank
 * @version                 Introduced in firmware 4.0.10.27
 * @param   bank            GoMultiplexBank object.
 * @param   id              The sensor ID to search for.
 * @return                  kTRUE if the sensor is in the multiplexing bank, kFALSE otherwise.            
 */
GoFx(kBool) GoMultiplexBank_HasSensor(GoMultiplexBank bank, k32u id);

/** 
 * Gets the ID of the given multiplexing bank.
 *
 * @public                  @memberof GoMultiplexBank
 * @version                 Introduced in firmware 4.0.10.27
 * @param   bank            GoMultiplexBank object.
 * @return                  The multiplexing bank ID.            
 */
GoFx(k32u) GoMultiplexBank_Id(GoMultiplexBank bank);

kEndHeader()
#include <GoSdk/GoMultiplexBank.x.h>

#endif

/// @endcond