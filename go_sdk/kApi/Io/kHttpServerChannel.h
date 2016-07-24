/** 
 * @file    kHttpServerChannel.h
 * @brief   Declares the kHttpServerChannel class. 
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_CHANNEL_H
#define K_API_HTTP_SERVER_CHANNEL_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @class   kHttpServerChannel
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents the server side of an HTTP connection.
 */
//typedef kObject kHttpServerChannel;     --forward-declared in kApiDef.x.h 

/** 
 * Accesses a kHttpServerRequest object that can be used to parse an incoming request.
 *
 * @public              @memberof kHttpServerChannel
 * @param   channel     Channel object.
 * @return              Request object. 
 */
kFx(kHttpServerRequest) kHttpServerChannel_Request(kHttpServerChannel channel); 

/** 
 * Accesses a kHttpServerResponse object that can be used to format an outgoing response.
 *
 * @public              @memberof kHttpServerChannel
 * @param   channel     Channel object.
 * @return              Response object. 
 */
kFx(kHttpServerResponse) kHttpServerChannel_Response(kHttpServerChannel channel); 

/** 
 * Transfers ownership of the underlying TCP client object associated with this channel. 
 * 
 * Use this function to assume control of the channel's TCP client object. This function should only be called 
 * after formatting an HTTP response, just prior to returning from a request processing callback. 
 * 
 * The primary purpose of this function is to support the WebSocket protocol. If a WebSocket request is 
 * received, the server can format a 101-switching-protocols response, then call this function to take
 * control of the client for subsequent communication. 
 * 
 * Use kObject_Destroy to free the TCP client object when it is no longer needed. 
 *
 * @public              @memberof kHttpServerChannel
 * @param   channel     Channel object.
 * @param   client      Receives TCP client object. 
 * @return              Operation status.
 */
kFx(kStatus) kHttpServerChannel_DetachClient(kHttpServerChannel channel, kTcpClient* client); 

kEndHeader()

#include <kApi/Io/kHttpServerChannel.x.h>

#endif
