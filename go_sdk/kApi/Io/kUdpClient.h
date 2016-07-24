/** 
 * @file    kUdpClient.h
 * @brief   Declares the kUdpClient class. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_UDP_CLIENT_H
#define K_API_UDP_CLIENT_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @class   kUdpClient
 * @extends kStream
 * @ingroup kApi-Io
 * @brief   Represents a UDP client.
 */
//typedef kStream kUdpClient;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a UDP client object.
 *
 * @public              @memberof kUdpClient
 * @param   client      Destination for the constructed object handle. 
 * @param   ipVersion   Internet Protocol version.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_Construct(kUdpClient* client, kIpVersion ipVersion, kAlloc allocator); 

/** 
 * Binds the client to a local IP address and/or port. 
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   address     A local IP address, or kIpAddress_Any(). 
 * @param   port        A local port number, or kIP_PORT_ANY.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_Bind(kUdpClient client, kIpAddress address, k32u port); 

/** 
 * Blocks until a datagram is received into the provided buffer (or until a timeout occurs). 
 * 
 * This method can be used when read buffering is disabled to read a datagram directly from the 
 * underlying socket. 
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   endPoint    Receives the end point of the sender. 
 * @param   buffer      Destination for received bytes.
 * @param   capacity    Maximum count of bytes to read.
 * @param   received    Receives the size of the received datagram. 
 * @param   timeout     Timeout, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_ReadFrom(kUdpClient client, kIpEndPoint* endPoint, void* buffer, kSize capacity, kSize* received, k64u timeout); 

/** 
 * Blocks until the provided datagram is written to the underlying socket (or until a timeout occurs).
 *
 * This method can be used when write buffering is disabled to write a datagram directly to the 
 * underlying socket. 
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   buffer      Bytes to be written.
 * @param   size        Count of bytes to be written.
 * @param   address     IP address of the recipient. 
 * @param   port        Port number of the recipient. 
 * @param   timeout     Timeout, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_WriteTo(kUdpClient client, const void* buffer, kSize size, kIpAddress address, k32u port, k64u timeout); 

/** 
 * Blocks until a datagram is received into kUdpClient's read buffer (or until a timeout occurs). 
 *
 * The Receive method is used to receive a datagram into kUdpClient's internal read buffer. 
 * Once the datagram has been received, the kStream_Read method can be used to read out the 
 * datagram.
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   endPoint    Receives the end point of the sender. 
 * @param   received    Receives the size of the received datagram. 
 * @param   timeout     Timeout, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_Receive(kUdpClient client, kIpEndPoint* endPoint, kSize* received, k64u timeout); 

/** 
 * Blocks until the datagram in kUdpClient's internal write buffer is written to the underlying socket 
 * (or until a timeout occurs).
 * 
 * The Send method is used to send a datagram that has been written into kUdpClient's internal 
 * write buffer. Bytes are written into kUdpClient's internal buffer via the kStream_Write method. 
 *
 * The 'clear' argument determines whether the internal write buffer is reset after sending the message, 
 * or whether it retains the datagram for subsequent retransmission.
 *
 * @public          @memberof kUdpClient
 * @param   client  UDP client object. 
 * @param   address IP address of the recipient. 
 * @param   port    Port number of the recipient. 
 * @param   timeout Timeout, in microseconds.
 * @param   clear   Specifies whether the internal write buffer pointer is updated.
 * @return          Operation status. 
 */
kFx(kStatus) kUdpClient_Send(kUdpClient client, kIpAddress address, k32u port, k64u timeout, kBool clear); 

/** 
 * Clears the internal write buffer state. 
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_Clear(kUdpClient client);

/** 
 * Enables or disables broadcasting.
 * 
 * If broadcasting is enabled, sending to kIpAddress_BroadcastV4() will broadcast a datagram 
 * on the subnet associated with the IPv4 address to which the client is bound.
 * 
 * Broadcasts are disabled by default.
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   broadcast   kTRUE to enable broadcasts; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_EnableBroadcast(kUdpClient client, kBool broadcast);

/** 
 * Enables the ability to receive broadcast messages. 
 * 
 * This function is required to work around a limitation in DSP/BIOS that prevents sockets
 * from receiving broadcasts unless they are bound to kIpAddress_AnyV4(). When used, this 
 * function must be called before kUdpClient_Bind.  
 * 
 * This function is harmless (no effect) when used on other operating systems. 
 * 
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   broadcast   kTRUE to enable broadcasts; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_EnableBroadcastReceive(kUdpClient client, kBool broadcast);

/** 
 * Enables or disables reuse of a local end point within a short period of time.
 * 
 * This option is typically used to allow a server to rebind to a local end point 
 * while a previous socket with the same local end point is in the TIME_WAIT state. 
 * This can be useful when a server must be stopped and started within a brief interval, 
 * but there is a small risk that packets with identical source/destination information 
 * could be misdirected to the new socket. 
 * 
 * This option is disabled by default.
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   reuse       kTRUE to enable reuse of IP addresses; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_EnableReuseAddress(kUdpClient client, kBool reuse);

/** 
 * Sets the size of write buffers.
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers enable the sender to 
 * formulate a datagram over multiple writes, rather than supplying the entire datagram in  
 * a single write call.  
 * 
 * If the client buffer size is greater than zero, use the kUdpClient_Send method to 
 * send the datagram when writing is complete. If the client buffer size is zero, a datagram 
 * can be sent immediately (without buffering) using kUdpClient_WriteTo. 
 * 
 * By default, the client buffer size is zero and the socket buffer size is determined by the 
 * underlying operating system. 
 * 
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   socketSize  Size of the write buffer maintained by the underlying socket (-1 to leave unchanged). 
 * @param   clientSize  Size of the write buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_SetWriteBuffers(kUdpClient client, kSSize socketSize, kSSize clientSize);

/** 
 * Sets the size of read buffers. 
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers enable the client to read 
 * the datagram over multiple read calls, rather than receiving the entire datagram in a single read 
 * call.  
 * 
 * If the client buffer size is greater than zero, use the kUdpClient_Receive method to 
 * receive a datagram before calling kUdpClient_Read. If the client buffer size is zero, a 
 * complete datagram can be received (without buffering) using kUdpClient_ReadFrom. 
 *
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   socketSize  Size of the read buffer maintained by the underlying socket (-1 to leave unchanged).
 * @param   clientSize  Size of the read buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kUdpClient_SetReadBuffers(kUdpClient client, kSSize socketSize, kSSize clientSize); 

/** 
 * Returns the underlying kSocket object.
 * 
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @return              Socket object. 
 */
kFx(kSocket) kUdpClient_Socket(kUdpClient client);  

/** 
 * Returns the local end point for a bound client. 
 * 
 * @public              @memberof kUdpClient
 * @param   client      UDP client object. 
 * @param   endPoint    Local end point.
 * @return              Socket object. 
 */
kFx(kStatus) kUdpClient_LocalEndPoint(kUdpClient client, kIpEndPoint* endPoint); 

kEndHeader()

#include <kApi/Io/kUdpClient.x.h>

#endif
