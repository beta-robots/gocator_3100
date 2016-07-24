/** 
 * @file    kPlugin.x.h
 *
 * @internal
 * Copyright (C) 2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PLUGIN_X_H
#define K_API_PLUGIN_X_H

kBeginHeader()

kDeclareClass(k, kPlugin, kObject)

typedef kStatus(kDlCall* kPluginConstructFx)(kAssembly* assembly);

#define kPLUGIN_CONSTRUCT_FX         "kPlugin_ConstructAssembly"

typedef struct kPluginClass
{
    kObjectClass base;
    kDynamicLib library;        // Library handle.
    kAssembly assembly;         // Assembly handle.
    kPointer handle;
} kPluginClass;

kFx(kStatus) kPlugin_Init(kPlugin plugin, kType type, const kChar* path, kAlloc allocator);
kFx(kStatus) kPlugin_VRelease(kPlugin plugin); 

#define kPlugin_Cast_(S)                  (kCastClass_(kPlugin, S))

kEndHeader()

#endif
