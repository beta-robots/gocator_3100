/** 
 * @file    kTcpServer.h
 * @brief   Declares the kTcpServer class. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_SERVER_H
#define K_API_TCP_SERVER_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @class   kTcpServer
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents a TCP server.
 */
//typedef kObject kTcpServer;     --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kTcpServer object.
 *
 * @public              @memberof kTcpServer
 * @param   server      Destination for the constructed object handle. 
 * @param   ipVersion   Internet Protocol version.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_Construct(kTcpServer* server, kIpVersion ipVersion, kAlloc allocator); 

/** 
 * Sets the size of write buffers for accepted client sockets.
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small write operations. 
 * 
 * By default, the client buffer size is zero and the socket buffer size is determined by the 
 * underlying operating system. 
 *
 * @public              @memberof kTcpServer
 * @param   server      kTcpServer object.
 * @param   socketSize  Size of the write buffer maintained by the underlying socket (-1 to leave unchanged). 
 * @param   clientSize  Size of the write buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_SetWriteBuffers(kTcpServer server, kSSize socketSize, kSSize clientSize); 

/** 
 * Sets the size of read buffers for accepted client sockets. 
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small read operations. 
 *
 * @public              @memberof kTcpServer
 * @param   server      kTcpServer object.
 * @param   socketSize  Size of the read buffer maintained by the underlying socket (-1 to leave unchanged).
 * @param   clientSize  Size of the read buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_SetReadBuffers(kTcpServer server, kSSize socketSize, kSSize clientSize);

/** 
 * Enables or disables reuse of a local end point within a short period of time.
 * 
 * The option is typically used to allow a server to rebind to a local end point 
 * while a previous socket with the same local end point is in the TIME_WAIT state. 
 * This can be useful when a server must be stopped and started within a brief interval, 
 * but there is a small risk that packets with identical source/destination information 
 * could be misdirected to the new socket. 
 *
 * @public              @memberof kTcpServer
 * @param   server      kTcpServer object.
 * @param   reuse       kTRUE to enable reuse of IP addresses; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_EnableReuseAddress(kTcpServer server, kBool reuse); 

/** 
 * Places the server into the listening state, to monitor for incoming connection requests.
 * 
 * The server can be placed in the listening state only once per kTcpServer object. After 
 * the server is shut down, the kTcpServer object cannot be used to listen again on another port.
 *
 * @public              @memberof kTcpServer
 * @param   server      kTcpServer object.
 * @param   address     A local IP address to which the server should bind, or kIpAddress_Any(). 
 * @param   port        A local port number to which the server should bind, or kIP_PORT_ANY.
 * @param   backlog     The maximum number of pending connection requests to enqueue. 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_Listen(kTcpServer server, kIpAddress address, k32u port, kSize backlog); 

/** 
 * Blocks until an incoming connection is established, or the specified timeout interval elapses. 
 * 
 * The returned connection object can be kNULL if the connection was closed by the remote client 
 * before being accepted.
 *
 * @public              @memberof kTcpServer
 * @param   server      A kTcpServer object in the listening state.
 * @param   timeout     The timeout interval. 
 * @param   client      Returns a kTcpClient object representing the newly-established connection, or kNULL. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_Accept(kTcpServer server, k64u timeout, kTcpClient* client, kAlloc allocator);

/** 
 * Returns the underlying socket object.
 *
 * @public              @memberof kTcpServer
 * @param   server      kTcpServer object.
 * @return              Underlying socket object. 
 */
kFx(kSocket) kTcpServer_Socket(kTcpServer server); 

/** 
 * Returns the local end point for a listening server. 
 *
 * @public              @memberof kTcpServer
 * @param   server      kTcpServer object.
 * @param   endPoint    Local end point.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpServer_LocalEndPoint(kTcpServer server, kIpEndPoint* endPoint); 

kEndHeader()

#include <kApi/Io/kTcpServer.x.h>

#endif
