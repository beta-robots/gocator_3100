/** 
 * @file    GoDiscovery.h
 * @brief   Declares discovery-related types. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_H
#define GO_SDK_DISCOVERY_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Data/kArrayList.h>

kBeginHeader()

/**
 * @struct  GoDiscoveryInfo
 * @extends kValue
 * @ingroup GoSdk-Internal
 * @brief   Represents discovery information for a single device. 
 */
typedef struct GoDiscoveryInfo
{
    k32u id;                    ///< device identifier (serial number)
    GoAddressInfo address;      ///< network configuration
} GoDiscoveryInfo; 

/**
 * @class   GoDiscovery
 * @extends kObject
 * @ingroup GoSdk-Internal
 * @brief   Represents a discovery client. 
 */
typedef kObject GoDiscovery; 

/** Defines the signature for a discovery enumeration handler. */
typedef kStatus (kCall* GoDiscoveryEnumFx)(kPointer context, GoDiscovery discovery, kArrayList info); 

/** 
 * Constructs a GoDiscovery object.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Receives constructed discovery object. 
 * @param   allocator   Memory allocator (or kNULL for default)
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_Construct(GoDiscovery* discovery, kAlloc allocator); 

/** 
 * Enumerates sensors present in the network. 
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   infoList    List to be populated with sensor descriptors (kArrayList<GoDiscoveryInfo>). 
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_Enumerate(GoDiscovery discovery, kArrayList infoList); 

/** 
 * Configures a sensor's network address settings.
 * 
 * This function uses UDP broadcasts; the sensor and can be on a different subnet than the client. 
 * 
 * The sensor will automatically reboot if the address is successfully changed. 
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   deviceId    Sensor device identifier (serial number). 
 * @param   address     New address information. 
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_SetAddress(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address); 

/** 
 * Retrieves a sensor's network address settings.
 * 
 * This function uses UDP broadcasts; the sensor and can be on a different subnet than the client. 
 * 
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   deviceId    Sensor device identifier (serial number). 
 * @param   address     Receives address information. 
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_GetAddress(GoDiscovery discovery, k32u deviceId, GoAddressInfo* address); 

/** 
 * Sets the enumeration period that will be used when background updates are enabled via StartEnum. 
 * 
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   period      Enumeration period, in microseconds.
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_SetEnumPeriod(GoDiscovery discovery, k64u period); 

/** 
 * Sets the enumeration callback to be used when background updates are enabled via StartEnum. 
 * 
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   function    Enumeration callback function (or kNULL to unregister). 
 * @param   receiver    Receiver argument for callback. 
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_SetEnumHandler(GoDiscovery discovery, GoDiscoveryEnumFx function, kPointer receiver); 

/** 
 * Starts periodic background discovery enumeration.
 * 
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   waitFirst   kTRUE to block until first enumeration cycle is completed; kFALSE otherwise.
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_StartEnum(GoDiscovery discovery, kBool waitFirst); 

/** 
 * Stops periodic background discovery enumeration.
 * 
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @return              Operation status. 
 */
GoFx(kStatus) GoDiscovery_StopEnum(GoDiscovery discovery); 

kEndHeader()
#include <GoSdk/Internal/GoDiscovery.x.h>

#endif
