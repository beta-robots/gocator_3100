/** 
 * @file    kBackTrace.x.h
 *
 * @internal
 * Copyright (C) 2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BACK_TRACE_X_H
#define K_API_BACK_TRACE_X_H

#include <kApi/Data/kList.h>

kBeginHeader()

#define kBACK_TRACE_CAPACITY                    (256)               ///< Maximum supported call stack depth. 

typedef struct kBackTraceClass
{
    kObjectClass base;
    kPointer functions[kBACK_TRACE_CAPACITY];           //Function call pointers. 
    kSize depth;                                        //Total call stack depth. 
    kSize skip;                                         //Number of frames to omit in reported depth. 
} kBackTraceClass; 

typedef struct kBackTraceVTable
{
    kObjectVTable base;
} kBackTraceVTable; 

typedef struct kBackTraceStatic
{
    kLock lock;                 //Used on Windows for exlusive access to symbol information.
    kPointer service;           //Used on Widnows to represent a handle to the symbol service.
} kBackTraceStatic; 

kDeclareFullClass(k, kBackTrace, kObject)

kFx(kStatus) kBackTrace_InitStatic(); 
kFx(kStatus) kBackTrace_ReleaseStatic(); 

kFx(void) kBackTrace_Lock(); 
kFx(void) kBackTrace_Unlock(); 

kFx(kStatus) kBackTrace_OnAssemblyLoad(kBackTrace trace, kAssembly assembly, kPointer unused); 

kFx(kStatus) kBackTrace_Init(kBackTrace trace, kType type, kAlloc alloc); 
kFx(kStatus) kBackTrace_VInitClone(kBackTrace trace, kObject source, kAlloc alloc); 
kFx(kStatus) kBackTrace_VRelease(kBackTrace trace); 

kFx(kBool) kBackTrace_VEquals(kBackTrace trace, kObject other); 
kFx(kSize) kBackTrace_VHashCode(kBackTrace trace); 

#define kBackTrace_(B)              (kCast_(kBackTraceClass*, B))
#define kBackTrace_Cast_(B)         (kCastClass_(kBackTrace, B))

kEndHeader()

#endif
