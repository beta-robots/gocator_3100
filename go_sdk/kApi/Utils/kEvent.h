/** 
 * @file    kEvent.h
 * @brief   Declares the kEvent class. 
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_EVENT_H
#define K_API_EVENT_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kEvent
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Represents a list of callbacks. 
 */
//typedef kObject kEvent;            --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kEvent object.
 *
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kEvent_Construct(kEvent* evnt, kAlloc allocator); 

/** 
 * Adds a listener to the event.
 *
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @param   function    Listener callback function. 
 * @param   receiver    Listener callback context. 
 * @return              Operation status. 
 */
kFx(kStatus) kEvent_Add(kEvent evnt, kCallbackFx function, kPointer receiver); 

/** 
 * Removes a listener from the event.
 *
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @param   function    Listener callback function. 
 * @param   receiver    Listener callback context. 
 * @return              Operation status. 
 */
kFx(kStatus) kEvent_Remove(kEvent evnt, kCallbackFx function, kPointer receiver); 

/** 
 * Removes all listeners from the event.
 *
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @return              Operation status. 
 */
kFx(kStatus) kEvent_Clear(kEvent evnt); 

/** 
 * Notifies all event listeners. 
 *
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @param   sender      Sender of event notification.
 * @param   args        Arguments for event notification.
 * @return              Operation status. 
 */
kFx(kStatus) kEvent_Notify(kEvent evnt, kPointer sender, void* args); 

/** 
 * Count of event listeners. 
 *
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @return              Listener count. 
 */
kFx(kSize) kEvent_Count(kEvent evnt); 

/** 
 * Gets the internal list of event listeners. 
 * 
 * This function is provided to support optimizations, and should not normally be used. The list returned 
 * by this function should be treated as read-only. 
 * 
 * @public              @memberof kEvent
 * @param   evnt        Event object. 
 * @return              Listener list -- kList<kCallback>.
 */
kFx(kList) kEvent_Listeners(kEvent evnt); 

/** @relates kEvent @{ */
#define kEvent_Listeners_(EV)           kxEvent_Listeners_(EV)          ///< Macro version of kEvent_Listeners.   
/** @} */

kEndHeader()

#include <kApi/Utils/kEvent.x.h>

#endif
