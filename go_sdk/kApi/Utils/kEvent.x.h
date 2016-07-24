/** 
 * @file    kEvent.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_EVENT_X_H
#define K_API_EVENT_X_H

#include <kApi/Data/kList.h>

kBeginHeader()

typedef struct kEventClass
{
    kObjectClass base;
    kList listeners;                //List of event listeners -- kList<kCallback>.
    kListItem notifyIt;             //List iterator used during notification.
} kEventClass; 

kDeclareClass(k, kEvent, kObject)

kFx(kStatus) kEvent_Init(kEvent evnt, kType type, kAlloc allocator); 

kFx(kStatus) kEvent_VInitClone(kEvent evnt, kEvent source, kAlloc allocator); 

kFx(kStatus) kEvent_VRelease(kEvent evnt); 

#define kEvent_(EV)                 kCast_(kEventClass*, EV)
#define kEvent_Cast_(EV)            (kCastClass_(kEvent, EV))

#define kxEvent_Listeners_(EV)      (kEvent_(EV)->listeners)

kEndHeader()

#endif
