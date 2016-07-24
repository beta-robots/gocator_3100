/** 
 * @file    GoHealthMsg.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_HEALTH_X_H
#define GO_SDK_HEALTH_X_H

#include <kApi/Data/kArray1.h>
kBeginHeader()

kDeclareValue(Go, GoIndicator, kValue)

typedef struct GoHealthMsgClass
{
    kObjectClass base; 
    GoDataSource source; 
    kArray1 indicators; 
} GoHealthMsgClass; 

kDeclareClass(Go, GoHealthMsg, kObject)

GoFx(kStatus) GoHealthMsg_Construct(GoHealthMsg* msg, kAlloc allocator);
GoFx(kStatus) GoHealthMsg_Init(GoHealthMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoHealthMsg_VInitClone(GoHealthMsg msg, GoHealthMsg source, kAlloc alloc); 
GoFx(kStatus) GoHealthMsg_Allocate(GoHealthMsg msg, kSize count);
GoFx(kStatus) GoHealthMsg_VRelease(GoHealthMsg msg);
GoFx(kSize) GoHealthMsg_VSize(GoHealthMsg msg); 
GoFx(kStatus) GoHealthMsg_WriteV0(GoHealthMsg msg, kSerializer serializer);
GoFx(kStatus) GoHealthMsg_ReadV0(GoHealthMsg msg, kSerializer serializer, kAlloc alloc);

#define GoHealthMsg_(D)                          kCast(GoHealthMsgClass*, D)
#define GoHealthMsg_SetContent_(D, V)            (GoHealthMsg_(D)->indicators = (V), kOK)
#define GoHealthMsg_Content_(D)                  (GoHealthMsg_(D)->indicators)
#define GoHealthMsg_SetSource_(D, V)             (GoHealthMsg_(D)->source = (V), kOK)

kEndHeader()

#endif