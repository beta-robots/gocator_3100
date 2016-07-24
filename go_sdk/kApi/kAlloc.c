/** 
 * @file    kAlloc.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kAlloc.h>
#include <kApi/kApiLib.h>
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/Utils/kDebugAlloc.h>
#include <kApi/Utils/kUtils.h>

kBeginFullClass(k, kAlloc, kObject)
    kAddVMethod(kAlloc, kObject, VRelease)
    kAddVMethod(kAlloc, kAlloc, VGet)
    kAddVMethod(kAlloc, kAlloc, VFree)
kEndFullClass()

kFx(kStatus) kAlloc_InitStatic()
{
    kAllocStatic* sobj = kAlloc_Static_(); 

    //construct system memory allocator
    kCheck(kUserAlloc_Construct(&sobj->systemAlloc, kApiLib_MemAllocHandler_(), kApiLib_MemFreeHandler_(), kApiLib_MemAllocProvider_(), kNULL)); 

    //create the application memory allocator
    kCheck(kApiLib_AppAllocConstructHandler_()(&sobj->appAlloc, sobj->systemAlloc)); 

    //in debug builds, a debug allocator is layered on top of the app allocator
    if (K_DEBUG_ENABLED)
    {
        kCheck(kDebugAlloc_Construct(&sobj->appAlloc, "App", sobj->appAlloc)); 
    }

    return kOK; 
}

kFx(kStatus) kAlloc_EndInitStatic()
{
    kAllocStatic* sobj = kAlloc_Static_(); 

    if (K_DEBUG_ENABLED)
    {
        kCheck(kAssembly_AddUnloadedHandler(kAlloc_OnAssemblyUnloaded, kNULL));        
    }

    sobj->assembly = kAssemblyOf(kApiLib); 

    return kOK; 
}

kFx(kStatus) kAlloc_ReleaseStatic()
{
    kAllocStatic* sobj = kAlloc_Static_(); 

    if (kStaticInitialized(kAssembly))
    {
        kAssembly_RemoveUnloadedHandler(kAlloc_OnAssemblyUnloaded, kNULL);        
    }

    kCheck(kAlloc_FinalizeDebug(&sobj->appAlloc)); 

    kCheck(kApiLib_AppAllocDestroyHandler_()(sobj->appAlloc)); 

    kCheck(kObject_Destroy(sobj->systemAlloc)); 

    return kOK; 
}

kFx(kStatus) kAlloc_DefaultConstructAppAlloc(kAlloc* appAlloc, kAlloc systemAlloc)
{
    *appAlloc = systemAlloc; 

    return kOK; 
}

kFx(kStatus) kAlloc_DefaultDestroyAppAlloc(kAlloc appAlloc)
{
    return kOK; 
}

kFx(kStatus) kAlloc_OnAssemblyUnloaded(kPointer unused, kAssembly assembly, kPointer args)
{
    kAllocStatic* sobj = kAlloc_Static_(); 

    //detect any memory allocations that appear to be leaked objects
    if (kObject_Is(sobj->appAlloc, kTypeOf(kDebugAlloc)))
    {
        kCheck(kDebugAlloc_DetectLeakedAssemblyObjects(sobj->appAlloc, 0, assembly)); 
    }

    return kOK; 
}

kFx(kStatus) kAlloc_FinalizeDebug(kAlloc* alloc)
{
    kAllocStatic* sobj = kAlloc_Static_(); 

    if (kObject_Is(*alloc, kTypeOf(kDebugAlloc)))
    {
        kAlloc innerAlloc = kObject_Alloc(*alloc); 

        //detect any leaked kApiLib objects
        kCheck(kDebugAlloc_DetectLeakedAssemblyObjects(*alloc, 0, sobj->assembly)); 

        //log all outstanding allocations
        if (kDebugAlloc_Allocated(*alloc) > 0)
        {
            kCheck(kApiLib_SetLeaksDetected_()); 

            //emit leaks to the kApi log handler, if leak logging is enabled
            if (kApiLib_LeakLoggingEnabled_())
            {
                kCheck(kDebugAlloc_LogAllocations(*alloc, 0)); 
                kCheck(kDebugAlloc_Clear(*alloc)); 
            }
        }

        kCheck(kDestroyRef(alloc)); 
        *alloc = innerAlloc; 
    }

    return kOK; 
}

kFx(kStatus) kAlloc_Init(kAlloc alloc, kType type, kAlloc allocator)
{
    return kObject_Init(alloc, type, allocator); 
}

kFx(kStatus) kAlloc_VRelease(kAlloc alloc)
{
    return kObject_VRelease(alloc); 
}

kFx(kStatus) kAlloc_Get(kAlloc alloc, kSize size, void* mem)
{
    kAllocClass* obj = kAlloc_Cast_(alloc); 

    return kAlloc_Get_(obj, size, mem); 
}

kFx(kStatus) kAlloc_VGet(kAlloc alloc, kSize size, void* mem)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kAlloc_GetZero(kAlloc alloc, kSize size, void* mem)
{
    kAllocClass* obj = kAlloc_Cast_(alloc); 

    kCheck(kAlloc_Get_(obj, size, mem)); 
 
    kCheck(kMemSet(kAs_(mem, kPointer), 0, size)); 

    return kOK; 
}

kFx(kStatus) kAlloc_Free(kAlloc alloc, void* mem)
{
    kAllocClass* obj = kAlloc_Cast_(alloc); 

    return kAlloc_Free_(obj, mem);  
}

kFx(kStatus) kAlloc_VFree(kAlloc alloc, void* mem)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kAlloc_FreeRef(kAlloc alloc, void* mem)
{
    kAllocClass* obj = kAlloc_Cast_(alloc); 

    kCheck(kAlloc_Free_(obj, *(void**)mem)); 
    
    kSetAs_(mem, kNULL, kPointer); 

    return kOK; 
}

kFx(kAlloc) kAlloc_System()
{
    return kAlloc_System_(); 
}

kFx(kAlloc) kAlloc_App()
{
    return kAlloc_App_(); 
}

kFx(kAlloc) kAlloc_Fallback(kAlloc alloc)
{
    return kAlloc_Fallback_(alloc); 
}
