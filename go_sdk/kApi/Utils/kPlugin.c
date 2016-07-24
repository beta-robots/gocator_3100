/** 
 * @file    kPlugin.c
 *
 * @internal
 * Copyright (C) 2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kPlugin.h>
#include <kApi/Utils/kDynamicLib.h>
#include <kApi/Io/kPath.h>

kBeginClass(k, kPlugin, kObject)
    kAddVMethod(kPlugin, kObject, VRelease)
kEndClass()

kFx(kStatus) kPlugin_Construct(kPlugin* plugin, const kChar* path, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kPlugin), plugin));

    if (!kSuccess(status = kPlugin_Init(*plugin, kTypeOf(kPlugin), path, alloc)))
    {
        kAlloc_FreeRef(alloc, plugin); 
    }

    return status; 
} 

kFx(kStatus) kPlugin_Init(kPlugin plugin, kType type, const kChar* path, kAlloc allocator)
{
    kPluginClass* obj = plugin;  
    kStatus status = kOK;
    kPluginConstructFx constructFx = kNULL;

    kCheck(kObject_Init(plugin, type, allocator)); 

    kInitFields_(kPlugin, plugin); 

    kTry
    {
        kTest(kDynamicLib_Construct(&obj->library, path, allocator)); 

        kTest(kDynamicLib_FindFunction(obj->library, kPLUGIN_CONSTRUCT_FX, (kFunction*)&constructFx)); 

        kTest(constructFx(&obj->assembly)); 
    }
    kCatch(&status)
    {
        kPlugin_VRelease(plugin); 
        kEndCatch(status);
    }

    return status; 
}

kFx(kStatus) kPlugin_VRelease(kPlugin plugin)
{
    kPluginClass* obj = kPlugin_Cast_(plugin); 

    kCheck(kObject_Destroy(obj->assembly)); 
    
    kCheck(kObject_Destroy(obj->library)); 
   
    kCheck(kObject_VRelease(plugin)); 

    return kOK;
}

kFx(kAssembly) kPlugin_Assembly(kPlugin plugin)
{
    kPluginClass* obj = kPlugin_Cast_(plugin);

    return obj->assembly; 
}
