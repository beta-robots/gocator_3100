/** 
 * @file    kHttpServer.h
 * @brief   Declares the kHttpServer class. 
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_H
#define K_API_HTTP_SERVER_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @class   kHttpServer
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Implements a simple HTTP server.
 * 
 * The kHttpServer class implements a simple HTTP 1.1 server framework. The kHttpServer_SetHandler function 
 * can be used to register a callback function to process remote requests.  The 'args' parameter to this callback 
 * receives a kHttpServerChannel instance, which can be used to access kHttpServerRequest and 
 * kHttpServerResponse objects. These request/response objects can be used to parse an incoming request 
 * and to format an outgoing response, respectively.
 * 
 * kHttpServer uses a simple thread-per-connection model, where the maximum number of simultaneous connections
 * can be configured using the kHttpServer_SetMaxConnections function. Accordingly, kHttpServer is better suited 
 * to small/embedded applications than large-scale web applications. 
 */
//typedef kObject kHttpServer;     --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kHttpServer object.
 *
 * @public              @memberof kHttpServer
 * @param   server      Destination for the constructed object handle. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_Construct(kHttpServer* server, kAlloc allocator); 

/** 
 * Sets the local address to which the server should bind. 
 * 
 * By default, the server will bind to the address returned by kIpAddress_AnyV4(). 
 * 
 * This function can only be called while the server is stopped. 
 *
 * @public              @memberof kHttpServer
 * @param   server      Server object. 
 * @param   address     IP address. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_SetAddress(kHttpServer server, kIpAddress address); 

/** 
 * Sets the local port to which the server should bind. 
 * 
 * By default, the server will bind to port 80. 
 * 
 * This function can only be called while the server is stopped. 
 *
 * @public              @memberof kHttpServer
 * @param   server      Server object. 
 * @param   port        Port number. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_SetPort(kHttpServer server, k32u port); 

/** 
 * Sets the maximum number of simultaneous connections supported by the server. 
 * 
 * This function can only be called while the server is stopped. 
 *
 * @public              @memberof kHttpServer
 * @param   server      Server object. 
 * @param   capacity    Maximum simultaneous connection count. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_SetMaxConnections(kHttpServer server, kSize capacity); 

/** 
 * Sets the callback that will be invoked to process remote requests. 
 * 
 * The callback 'sender' argument will receive a kHttpServer instance, while the callback 'args' argument 
 * will receive a kHttpServerChannel instance. The callback receiver can use the kHttpServerChannel object to 
 * access kHttpServerRequest and kHttpServerResponse objects, which can be used to parse the incoming request 
 * and to generate a response. 
 *
 * This function can only be called while the server is stopped. 
 *
 * @public              @memberof kHttpServer
 * @param   server      Server object. 
 * @param   function    Callback function. 
 * @param   receiver    Callback receiver. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_SetHandler(kHttpServer server, kCallbackFx function, kPointer receiver); 

/** 
 * Starts the server. 
 * 
 * @public              @memberof kHttpServer
 * @param   server      kHttpServer object.
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_Start(kHttpServer server); 

/** 
 * Stops the server. 
 *
 * All asynchronous activities performed by the server will be stopped before this 
 * function returns. 
 * 
 * @public              @memberof kHttpServer
 * @param   server      kHttpServer object.
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_Stop(kHttpServer server);

/** 
 * Reports the local end-point for a running server. 
 *
 * @public              @memberof kHttpServer
 * @param   server      kHttpServer object.
 * @param   endPoint    Receives local end-point information.
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServer_LocalEndPoint(kHttpServer server, kIpEndPoint* endPoint); 

kEndHeader()

#include <kApi/Io/kHttpServer.x.h>

#endif
