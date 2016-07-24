/** 
 * @file    GoReceiver.h
 * @brief   Declares the GoReceiver class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_RECEIVER_H
#define GO_SDK_RECEIVER_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Io/kNetwork.h>
#include <kApi/Io/kSerializer.h>
kBeginHeader()

/**
 * @class   GoReceiver
 * @extends kObject
 * @ingroup GoSdk-Internal
 * @brief   Represents a data/health receiver.
 */
typedef kObject GoReceiver; 

/** Defines the signature for a data message handler. */
typedef kStatus (kCall* GoReceiverMessageFx)(kPointer context, GoReceiver receiver, kSerializer reader); 

/** 
 * Constructs a GoReceiver object.
 *
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receives constructed receiver object. 
 * @param   allocator   Memory allocator (or kNULL for default)
 * @return              Operation status. 
 */
GoFx(kStatus) GoReceiver_Construct(GoReceiver* receiver, kAlloc allocator);

/** 
 * Sets the size of buffers used for receiving data.
 *
 * Call this function before calling GoReceiver_Open. 
 * 
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receiver object. 
 * @param   socketSize  Size of the read buffer used by the underlying operating system socket (-1 to leave unchanged). 
 * @param   clientSize  Size of the read buffer used by the receiver object (-1 to leave unchanged).
 * @return              Operation status. 
 */
GoFx(kStatus) GoReceiver_SetBuffers(GoReceiver receiver, kSSize socketSize, kSSize clientSize);

/** 
 * Sets an I/O cancellation query handler for this receiver object. 
 * 
 * The I/O cancellation query handler will be polled periodically when I/O is blocked
 * for a non-negligible amount of time. If the cancellation handler returns kERROR_ABORT, 
 * ongoing communication will be terminated. 
 *
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receiver object. 
 * @param   function    I/O cancellation callback function (or kNULL to unregister).
 * @param   context     Context argument for callback (function receiver).
 * @return              Operation status.            
 */
GoFx(kStatus) GoReceiver_SetCancelHandler(GoReceiver receiver, kCallbackFx function, kPointer context); 

/** 
 * Sets a callback function that can be used to receive notifications when data is available to be read.
 * 
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receiver object. 
 * @param   function    Data callback function (or kNULL to unregister).
 * @param   context     Context argument for callback (function receiver).
 * @return              Operation status.            
 */
GoFx(kStatus) GoReceiver_SetMessageHandler(GoReceiver receiver, GoReceiverMessageFx function, kPointer context);

/** 
 * Opens a data connection to the specified sensor IP address and port.
 *
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receiver object. 
 * @param   address     Sensor IP address.
 * @param   port        Sensor data port.
 * @return              Operation status.            
 */
GoFx(kStatus) GoReceiver_Open(GoReceiver receiver, kIpAddress address, k32u port);

/** 
 * Closes the data connection. 
 *
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receiver object. 
 * @return              Operation status.            
 */
GoFx(kStatus) GoReceiver_Close(GoReceiver receiver);

/** 
 * Reports whether the receiver object has been opened. 
 *
 * @public              @memberof GoReceiver
 * @version             Introduced in firmware 4.0.10.27
 * @param   receiver    Receiver object. 
 * @return              kTRUE if open; kFALSE otherwise.
 */
GoFx(kBool) GoReceiver_IsOpen(GoReceiver receiver);

kEndHeader()
#include <GoSdk/Internal/GoReceiver.x.h>

#endif
