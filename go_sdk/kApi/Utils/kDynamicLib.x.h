/** 
 * @file    kDynamicLib.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DYNAMICLIB_X_H
#define K_API_DYNAMICLIB_X_H

kBeginHeader()

kDeclareClass(k, kDynamicLib, kObject)

typedef struct kDynamicLibClass
{
    kObjectClass base;
    kPointer handle;
} kDynamicLibClass;

kFx(kStatus) kDynamicLib_Init(kDynamicLib library, kType type, const kChar* path, kAlloc allocator);
kFx(kStatus) kDynamicLib_VRelease(kDynamicLib library); 

kFx(kStatus) kDynamicLib_OpenHandle(const kChar* path, kPointer* handle);
kFx(kStatus) kDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function);
kFx(kStatus) kDynamicLib_CloseHandle(kPointer handle);

//deprecated
#define kDynamicLib_Function     kDynamicLib_FindFunction

#define kDynamicLib_Cast_(S)                  (kCastClass_(kDynamicLib, S))

kEndHeader()

#endif
