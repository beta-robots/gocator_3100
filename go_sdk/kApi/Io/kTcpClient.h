/** 
 * @file    kTcpClient.h
 * @brief   Declares the kTcpClient class. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_CLIENT_H
#define K_API_TCP_CLIENT_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @class   kTcpClient
 * @extends kStream
 * @ingroup kApi-Io
 * @brief   Represents a TCP client.
 */
//typedef kStream kTcpClient;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a TCP client object.
 *
 * @public              @memberof kTcpClient
 * @param   client      Destination for the constructed object handle. 
 * @param   ipVersion   Internet Protocol version.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Construct(kTcpClient* client, kIpVersion ipVersion, kAlloc allocator);

/** 
 * Sets the size of write buffers.
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small write operations. 
 * 
 * By default, the client buffer size is zero and the socket buffer size is determined by the 
 * underlying operating system. 
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   socketSize  Size of the write buffer maintained by the underlying socket (-1 to leave unchanged). 
 * @param   clientSize  Size of the write buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetWriteBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize); 

/** 
 * Sets the size of read buffers. 
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small read operations. 
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   socketSize  Size of the read buffer maintained by the underlying socket (-1 to leave unchanged).
 * @param   clientSize  Size of the read buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetReadBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize);

/** 
 * Sets the timeout duration for write operations.  
 *
 * By default, kTcpClient objects do not use a timeout interval and can block indefinitely.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     Timeout value, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetWriteTimeout(kTcpClient client, k64u timeout);

/** 
 * Sets the timeout duration for read operations. 
 *
 * By default, kTcpClient objects do not use a timeout interval and can block indefinitely.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     Timeout value, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetReadTimeout(kTcpClient client, k64u timeout);

/** 
 * Sets a cancel query handler, which can be used to asynchronously terminate read/write operations.
 * 
 * kTcpClient_SetCancelHandler and kTcpClient_Cancel represent alternative approaches to I/O cancellation; 
 * use one or the other approach, but not both. 
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   function    Callback function.
 * @param   receiver    Callback context.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetCancelHandler(kTcpClient client, kCallbackFx function, kPointer receiver);

/** 
 * Cancels any pending I/O operations. 
 * 
 * kTcpClient_SetCancelHandler and kTcpClient_Cancel represent alternative approaches to I/O cancellation; 
 * use one or the other approach, but not both. 
 *
 * This method is thread-safe.
 * 
 * @public              @memberof kTcpClient
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Cancel(kTcpClient client);

/** 
 * Connects to a remote end point. 
 * 
 * A connection can be established only once per kTcpClient object.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   address     The remote IP address.  
 * @param   port        The remote port number.
 * @param   timeout     Timeout interval, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Connect(kTcpClient client, kIpAddress address, k32u port, k64u timeout); 

/** 
 * Waits until the client has bytes to read or until the specified timeout period elapses. 
 * 
 * This function will return kERROR_TIMEOUT in the event that the client is not ready for reading by 
 * the end of the timeout period. 
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     Timeout interval, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Wait(kTcpClient client, k64u timeout);

/** 
 * Returns the underlying kSocket object.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Operation status. 
 */
kFx(kSocket) kTcpClient_Socket(kTcpClient client); 

/** 
 * Returns the number of bytes currently enqueued and available for reading. 
 * 
 * This function returns the count of bytes enqueued in the client's internal read buffer. This does 
 * not include any data enqueued in the underlying socket's read buffer.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Count of bytes available for reading.
 */
kFx(kSize) kTcpClient_Available(kTcpClient client); 

/** 
 * Returns the local end point for a connected client.
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   endPoint    Local end point.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_LocalEndPoint(kTcpClient client, kIpEndPoint* endPoint);

/** 
 * Returns the remote end point for a connected client. 
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   endPoint    Remote end point.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_RemoteEndPoint(kTcpClient client, kIpEndPoint* endPoint); 

/** 
 * Reports any internal errors that will prevent success of future communication attempts.
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Client status. 
 */
kFx(kStatus) kTcpClient_Status(kTcpClient client); 

kEndHeader()

#include <kApi/Io/kTcpClient.x.h>

#endif
