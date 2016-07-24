/** 
 * @file    kDynamicLib.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kDynamicLib.h>
#include <kApi/Io/kPath.h>
#include <stdio.h>

kBeginClass(k, kDynamicLib, kObject)
    kAddVMethod(kDynamicLib, kObject, VRelease)
kEndClass()

kFx(kStatus) kDynamicLib_Construct(kDynamicLib* library, const kChar* path, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kDynamicLib); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, library)); 

    if (!kSuccess(status = kDynamicLib_Init(*library, type, path, alloc)))
    {
        kAlloc_FreeRef(alloc, library); 
    }

    return status; 
} 

kFx(kStatus) kDynamicLib_Init(kDynamicLib library, kType type, const kChar* path, kAlloc allocator)
{
    kDynamicLibClass* obj = library;  
    kStatus status = kOK;

    kCheck(kObject_Init(library, type, allocator)); 

    kInitFields_(kDynamicLib, library); 

    if (!kSuccess(status = kDynamicLib_OpenHandle(path, &obj->handle)))
    {
        kDynamicLib_VRelease(library);
    }

    return status; 
}

kFx(kStatus) kDynamicLib_VRelease(kDynamicLib library)
{
    kDynamicLibClass* obj = kDynamicLib_Cast_(library); 

    if (!kIsNull(obj->handle))
    {
        kCheck(kDynamicLib_CloseHandle(obj->handle));
    }

    kCheck(kObject_VRelease(library)); 

    return kOK;
}

kFx(kStatus) kDynamicLib_FindFunction(kDynamicLib library, const kChar* name, kFunction* function)
{
    kDynamicLibClass* obj = kDynamicLib_Cast_(library); 

    return kDynamicLib_Resolve(obj->handle, name, function);
}

#if defined(K_WINDOWS)

kFx(kStatus) kDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(kPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    if (kIsNull(*handle = (kPointer) LoadLibraryW(wpath)))
    {
        return kERROR_NOT_FOUND; 
    }

    return kOK;
}

kFx(kStatus) kDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    FARPROC address = GetProcAddress((HMODULE)handle, name); 

    if (kIsNull(address))
    {
        return kERROR_NOT_FOUND; 
    }

    *(FARPROC*)function = address; 

    return kOK;
}

kFx(kStatus) kDynamicLib_CloseHandle(kPointer handle)
{
    if (!FreeLibrary((HMODULE)handle))
    {
        return kERROR_OS; 
    }

    return kOK;
}

#elif defined (K_DSP_BIOS)

kFx(kStatus) kDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDynamicLib_CloseHandle(kPointer handle)
{
    return kERROR_UNIMPLEMENTED; 
}

#elif defined (K_VX_KERNEL)

kFx(kStatus) kDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDynamicLib_CloseHandle(kPointer handle)
{
    return kERROR_UNIMPLEMENTED; 
}

#elif defined (K_POSIX)

kFx(kStatus) kDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    if (kIsNull(*handle = (kPointer)dlopen(path, RTLD_NOW)))
    {
        return kERROR_NOT_FOUND; 
    }

    return kOK;
}

kFx(kStatus) kDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    void* address = dlsym(handle, name); 

    if (kIsNull(address))
    {
        return kERROR_NOT_FOUND; 
    }

    *(void**)function = address;

    return kOK;
}

kFx(kStatus) kDynamicLib_CloseHandle(kPointer handle)
{
    if (dlclose(handle) != 0)
    {
        return kERROR_OS; 
    }

    return kOK;
}

#endif
