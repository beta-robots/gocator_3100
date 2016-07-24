/** 
 * @file    kNetwork.h
 * @brief   IP networking definitions. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_NETWORK_H
#define K_API_NETWORK_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @struct  kIpVersion
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents an Internet Protocol version. 
 * 
 * The following enumerators are defined:  
 * - #kIP_VERSION_4
 */
typedef k32s kIpVersion; 

/** @relates kIpVersion @{ */
#define kIP_VERSION_4          (4)         ///< Internet Protocol version 4.
/** @} */

/**
 * @struct  kIpAddress
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents an IP address. 
 */
typedef struct kIpAddress
{
    kIpVersion version;                 ///< Address version.
    kByte address[16];                  ///< Address bytes (most significant byte first). 
} kIpAddress;                                 

/** 
 * Gets an address representing an automatically-assigned address.
 *
 * @public              @memberof kIpAddress
 * @param   version     IP version.
 * @return              Address value. 
 */
kFx(kIpAddress) kIpAddress_Any(kIpVersion version); 

/** 
 * Gets an address representing an automatically-assigned IPv4 address.
 *
 * @public          @memberof kIpAddress
 * @return          Address value. 
 */
kFx(kIpAddress) kIpAddress_AnyV4(); 

/** 
 * Gets an address suitable for broadcasting IPv4 datagrams. 
 * 
 * @public          @memberof kIpAddress
 * @return          Address value. 
 */
kFx(kIpAddress) kIpAddress_BroadcastV4();

/** 
 * Gets the loopback address. 
 *
 * @public              @memberof kIpAddress
 * @param   version     IP version.
 * @return              Address value. 
 */
kFx(kIpAddress) kIpAddress_Loopback(kIpVersion version);

/** 
 * Gets the IpV4 loopback address. 
 *
 * @public          @memberof kIpAddress
 * @return          Address value. 
 */
kFx(kIpAddress) kIpAddress_LoopbackV4();

/** 
 * Parses a text-formatted IP address. 
 *
 * Supports dotted-quad (IPv4) format (e.g. "192.168.1.10"). 
 *
 * @public              @memberof kIpAddress
 * @param   address     Receives the IP address. 
 * @param   text        Text-formatted IP address.
 * @return              Operation status. 
 */
kFx(kStatus) kIpAddress_Parse(kIpAddress* address, const kChar* text); 

/** 
 * Formats an IP address as a string. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address.
 * @param   text        Receives formatted string.
 * @param   capacity    Capacity of the string buffer. 
 * @return              Operation status. 
 */
kFx(kStatus) kIpAddress_Format(kIpAddress address, kChar* text, kSize capacity); 

/** 
 * Compares two addresses for equality.
 *
 * @public          @memberof kIpAddress
 * @param   a       First address.    
 * @param   b       Second address. 
 * @return          kTRUE if addresses are equal; kFALSE otherwise. 
 */
kFx(kBool) kIpAddress_Equals(kIpAddress a, kIpAddress b); 

/** 
 * Reports whether the given address is a loopback address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address. 
 * @return              kTRUE if the address is loopback; kFALSE otherwise. 
 */
kFx(kBool) kIpAddress_IsLoopback(kIpAddress address); 

/** 
 * Converts an IPv4 address to a host-endian 32-bit integer. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address.
 * @return              Host-endian integer. 
 */
kFx(k32u) kIpAddress_ToHost32u(kIpAddress address); 

/** 
 * Converts an IPv4 address to a network-endian 32-bit integer. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address.
 * @return              Network-endian integer. 
 */
kFx(k32u) kIpAddress_ToNet32u(kIpAddress address); 

/** 
 * Converts a host-endian 32-bit integer to an IPv4 address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     Host-endian integer. 
 * @return              IP address.
 */
kFx(kIpAddress) kIpAddress_FromHost32u(k32u address); 

/** 
 * Converts a network-endian 32-bit integer to an IPv4 address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     Network-endian integer. 
 * @return              IP address.
 */
kFx(kIpAddress) kIpAddress_FromNet32u(k32u address); 

/**
 * @struct  kIpEndPoint
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents an IP end point (address, port). 
 */
typedef struct kIpEndPoint              
{
    kIpAddress address;         ///< IP address.
    k32u port;                  ///< Port number. 
} kIpEndPoint;                  ///< Represents an IP end point.

/** @relates kIpEndPoint @{ */
#define kIP_PORT_ANY    (0)     ///< Used to request an automatically assigned port. 
/** @} */

/**
 * @struct  kIpEntry
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents information about a local IP address.
 */
typedef struct kIpEntry
{
    kIpAddress address;         ///< IP address. 
    kText128 name;              ///< Host interface name. 
} kIpEntry; 

/**
 * @class   kNetwork
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   A collection of static network utility methods. 
 */
typedef kObject kNetwork; 

/** 
 * Finds information about local IP configuration. 
 *
 * This function determines information pertaining to IP addresses on the local host. 
 * Some informational fields may be unavailable on some platforms. 
 * 
 * @private             @memberof kNetwork
 * @param   entries     Receives IP info (kArrayList<kIpEntry>). 
 * @return              Operation status. 
 */
kFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries); 

kEndHeader()

#include <kApi/Io/kNetwork.x.h>

#endif
