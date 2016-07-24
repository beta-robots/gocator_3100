/** 
 * @file    kAssembly.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kAssembly.h>
#include <kApi/kApiLib.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Utils/kEvent.h>

kBeginFullClass(k, kAssembly, kObject)
    kAddVMethod(kAssembly, kObject, VRelease)
kEndFullClass()

kFx(kStatus) kAssembly_InitStatic()
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 

    kCheck(kLock_Construct(&sobj->lock, kNULL)); 

    kCheck(kEvent_Construct(&sobj->onLoad, kNULL)); 
    kCheck(kEvent_Construct(&sobj->onUnload, kNULL)); 
    kCheck(kEvent_Construct(&sobj->onUnloaded, kNULL)); 

    //kAlloc and kAssembly have a mutual static dependency; this line breaks the dependency, 
    //allowing kAlloc to perform part of its initialization before kAssembly and part after
    kCheck(kAlloc_EndInitStatic()); 

    return kOK;
}

kFx(kStatus) kAssembly_ReleaseStatic()
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 

    kCheck(kDestroyRef(&sobj->lock)); 
   
    kCheck(kDestroyRef(&sobj->onLoad)); 
    kCheck(kDestroyRef(&sobj->onUnload)); 
    kCheck(kDestroyRef(&sobj->onUnloaded)); 

    return kOK;
}

kFx(kStatus) kAssembly_AddAssembly(kAssembly assembly)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 

    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kSysMemReallocList(&sobj->assemblies, sobj->assemblyCount, sizeof(kAssembly), 
            &sobj->assemblyCapacity, kASSEMBLY_INITIAL_ASSEMBLY_CAPACITY, sobj->assemblyCount+1)); 
                
        sobj->assemblies[sobj->assemblyCount++] = assembly; 

        kTest(kEvent_Notify(sobj->onLoad, assembly, kNULL)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveAssembly(kAssembly assembly)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    kSize i; 

    kCheck(kLock_Enter(sobj->lock)); 
    
    kTry
    {
        kTest(kEvent_Notify(sobj->onUnload, assembly, kNULL)); 

        for (i = 0; i < sobj->assemblyCount; ++i)
        {
            if (sobj->assemblies[i] == assembly)
            {
                kTest(kMemMove(&sobj->assemblies[i], &sobj->assemblies[i+1], sizeof(kAssembly)*(sobj->assemblyCount-i-1))); 
                sobj->assemblyCount--; 
                break;
            }
        }

        if (sobj->assemblyCount == 0)
        {
            kTest(kSysMemFree(sobj->assemblies)); 
            sobj->assemblyCapacity = 0; 
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddLoadHandler(kCallbackFx function, kPointer receiver)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Add(sobj->onLoad, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveLoadHandler(kCallbackFx function, kPointer receiver)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Remove(sobj->onLoad, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddUnloadHandler(kCallbackFx function, kPointer receiver)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Add(sobj->onUnload, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveUnloadHandler(kCallbackFx function, kPointer receiver)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Remove(sobj->onUnload, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddUnloadedHandler(kCallbackFx function, kPointer receiver)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Add(sobj->onUnloaded, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveUnloadedHandler(kCallbackFx function, kPointer receiver)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Remove(sobj->onUnloaded, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}


kFx(kStatus) kAssembly_Enumerate(kArrayList assemblies)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kArrayList_Import(assemblies, sobj->assemblies, kTypeOf(kAssembly), sobj->assemblyCount)); 
        kTest(kShareContent(kTypeOf(kAssembly), kArrayList_Data(assemblies), kArrayList_Count(assemblies))); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_ConstructInternal(kAssembly* assembly, kAssembly* reference, const kChar* name, kVersion version)
{
    kAssembly output = kNULL;
    kStatus status; 

    kCheck(kSysMemAlloc(sizeof(kAssemblyClass), &output)); 

    if (!kSuccess(status = kAssembly_Init(output, kNULL, kNULL, reference, name, version)))
    {
        kCheck(kSysMemFree(output)); 
        return status; 
    }

    *assembly = output; 
    *reference = output; 

    return kOK; 
}

kFx(kStatus) kAssembly_Init(kAssembly assembly, kType type, kAlloc allocator, kAssembly* reference, const kChar* name, kVersion version)
{
    kAssemblyClass* obj = assembly; 
    kStatus status; 

    kCheck(kObject_Init(assembly, type, allocator)); 

    kTry
    {
        kTest(kStrCopy(obj->name, kCountOf(obj->name), name)); 
        obj->version = version; 
        obj->selfReference = reference;      
    }
    kCatch(&status)
    {
        kObject_VRelease(assembly); 
        kEndCatch(status); 
    }
    
    return kOK; 
}

kFx(kStatus) kAssembly_VRelease(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 

    *obj->selfReference = kNULL; 

    if (obj->isRegistered)
    {
        kCheck(kAssembly_RemoveAssembly(assembly)); 
    }

    kCheck(kObject_Destroy(obj->typeMap)); 

    kCheck(kAssembly_ReleaseTypes(assembly)); 
    kCheck(kAssembly_ReleaseDependencies(assembly));         

    kCheck(kObject_VRelease(assembly)); 

    return kOK; 
}

kFx(kStatus) kAssembly_AddDependency(kAssembly assembly, kAssemblyConstructFx depend)
{
    kAssemblyClass* obj = assembly; 

    kCheck(kSysMemReallocList(&obj->dependCalls, obj->dependCallCount, sizeof(kAssemblyConstructFx), 
        &obj->dependCallCapacity, kASSEMBLY_INITIAL_DEPEND_CAPACITY, obj->dependCallCount+1)); 

    obj->dependCalls[obj->dependCallCount++] = depend; 

    return kOK; 
}

kFx(kStatus) kAssembly_AddType(kAssembly assembly, kAssemblyRegisterFx call, const kChar* name)
{
    kAssemblyClass* obj = assembly; 

    kCheck(kSysMemReallocList(&obj->regCalls, obj->regCallCount, sizeof(kAssemblyRegCallInfo), 
        &obj->regCallCapacity, kASSEMBLY_INITIAL_TYPE_CAPACITY, obj->regCallCount+1)); 

    obj->regCalls[obj->regCallCount].call = call; 
    kStrCopy(obj->regCalls[obj->regCallCount].name, kCountOf(obj->regCalls[obj->regCallCount].name), name); 
    obj->regCalls[obj->regCallCount].priority = k16S_MAX; 

    obj->regCallCount++; 

    return kOK; 
}

kFx(kStatus) kAssembly_AddPriority(kAssembly assembly, kAssemblyRegisterFx call, k32u priority)
{
    kAssemblyClass* obj = assembly; 
    kSize i; 

    for (i = 0; i < obj->regCallCount; ++i)
    {
        if (obj->regCalls[i].call == call)
        {
            obj->regCalls[i].priority = priority; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kAssembly_Finalize(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 

    kCheck(kAssembly_InitDependencies(assembly)); 
    kCheck(kAssembly_InitTypes(assembly)); 
    kCheck(kAssembly_MapTypes(assembly)); 

    kCheck(kAssembly_AddAssembly(assembly)); 

    obj->isRegistered = kTRUE; 

    return kOK; 
}

kFx(kStatus) kAssembly_InitDependencies(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 
    kSize callCount = obj->dependCallCount; 
    kSize i; 

    kCheck(kSysMemAlloc(callCount*sizeof(kAssembly), &obj->dependencies));     

    for (i = 0; i < callCount; ++i)
    {
        kCheck(obj->dependCalls[i](&obj->dependencies[i])); 
        obj->dependencyCount++; 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_ReleaseDependencies(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 
    kSSize count = (kSSize)obj->dependencyCount; 
    kSSize i; 

    for (i = count-1; i >= 0; --i)
    {
        kCheck(kDestroyRef(&obj->dependencies[i])); 
    }

    kCheck(kSysMemFree(obj->dependencies)); 
    kCheck(kSysMemFree(obj->dependCalls)); 

    return kOK; 
}

kFx(kStatus) kAssembly_ReorderType(kAssembly assembly, const kChar* dependentOn)
{
    kAssemblyClass* obj = assembly; 
    kSize dependentIndex = 0; 
    
    kCheck(kAssembly_FindRegCall(assembly, dependentOn, &dependentIndex)); 
    
    if (dependentIndex > obj->nowRegistering)
    {
        kAssemblyRegCallInfo callInfo = obj->regCalls[obj->nowRegistering]; 

        kMemMove(&obj->regCalls[obj->nowRegistering], &obj->regCalls[obj->nowRegistering+1], 
            (dependentIndex-obj->nowRegistering)*sizeof(callInfo)); 

        obj->regCalls[dependentIndex] = callInfo; 

        obj->reordered = kTRUE; 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_FindRegCall(kAssembly assembly, const kChar* name, kSize* index)
{
    kAssemblyClass* obj = assembly; 
    kSize i; 
    
    for (i = 0; i < obj->regCallCount; ++i)
    {
        if (kStrEquals(obj->regCalls[i].name, name))
        {
            *index = i; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kAssembly_InitTypes(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 
    kAssemblyRegCallInfo* regCalls = obj->regCalls;
    kSize count = obj->regCallCount; 
    kSize typeCount; 
    kStatus status; 
    kSize i; 

    kCheck(kSysMemAlloc(count*sizeof(kType), &obj->types));     

    i = 0; 
    while (i < count)
    {
        typeCount = obj->typeCount; 
        obj->nowRegistering = i; 
        obj->reordered = kFALSE; 

        status = regCalls[i].call(assembly, regCalls[i].name);         

        if (!kSuccess(status) && (obj->typeCount > typeCount))
        {
            kCheck(kType_VRelease(obj->types[typeCount])); 
            kCheck(kSysMemFree(obj->types[typeCount])); 

            obj->typeCount--; 
        }

        if (!obj->reordered)
        {
            kCheck(status);    
            kCheck(kType_SetInitPriority(obj->types[typeCount], regCalls[typeCount].priority)); 
            i++; 
        }              
    }

    for (i = 0; i < obj->typeCount; ++i)
    {
        kObject_(obj->types[i])->type = kTypeOf(kType); 
    }

    kObject_(assembly)->type = kTypeOf(kAssembly); 

    kCheck(kAssembly_SortTypes(assembly));  

    for (i = 0; i < obj->typeCount; ++i)
    {
        kCheck(kType_ZeroStaticData(obj->types[i]));
    }

    for (i = 0; i < obj->typeCount; ++i)
    {
        kCheck(kType_RunStaticInit(obj->types[i])); 
    }
    
    return kOK; 
}

kFx(kStatus) kAssembly_ReleaseTypes(kAssembly assembly)
{
    kAssemblyStatic* sobj = kStaticOf(kAssembly); 
    kAssemblyClass* obj = assembly; 
    kSSize count = (k32s) obj->typeCount; 
    kSSize i; 

    for (i = count-1; i >= 0; --i)
    {
        kCheck(kType_RunStaticRelease(obj->types[i])); 
    }

    if (kStaticInitialized(kAssembly))
    {
        kEvent_Notify(sobj->onUnloaded, assembly, kNULL); 
    }

    for (i = count-1; i >= 0; --i)
    {
        kCheck(kType_VRelease(obj->types[i])); 
        kCheck(kSysMemFree(obj->types[i])); 
    }

    kCheck(kSysMemFree(obj->types)); 
    kCheck(kSysMemFree(obj->regCalls)); 

    return kOK; 
}

int kAssembly_TypeSortComparator(const void* a, const void* b)
{
    return kType_Priority(*(kType*)a) - kType_Priority(*(kType*)b); 
}

kFx(kStatus) kAssembly_SortTypes(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 
    
    qsort(obj->types, obj->typeCount, sizeof(kType), kAssembly_TypeSortComparator); 

    return kOK; 
}

kFx(kStatus) kAssembly_MapTypes(kAssembly assembly)
{
    kAssemblyClass* obj = assembly; 
    kSize i;

    kCheck(kMap_Construct(&obj->typeMap, kTypeOf(kTypeName), kTypeOf(kType), 0, kNULL)); 

    for (i = 0; i < obj->typeCount; ++i)
    {
        kStatus status = kMap_Add(obj->typeMap, kType_Name(obj->types[i]), &obj->types[i]); 

        //Registered the same type multiple times?
        kAssert(kSuccess(status));  
        kCheck(status); 
    }
    
    return kOK; 
}

kFx(kStatus) kAssembly_AddValue(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize size, kSize vTableSize, kTypeFlags flags)
{      
    kAssemblyClass* obj = assembly; 
    kType output = kNULL; 
    kStatus status; 

    if (kIsNull(base) && !kStrEquals(baseName, "kNull"))
    {
        if (kSuccess(kAssembly_ReorderType(assembly, baseName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the base class with kAddType?
            kAssert(kFALSE); 
            kCheck(kERROR_STATE); 
        }
    }

    kCheck(kType_Construct(&output, type, name, assembly, flags, size, 0));     

    kTry
    {
        kTest(kType_SetBase(output, base)); 
        kTest(kType_InitMethods(output)); 
        kTest(kType_InitVTable(output, vTableSize)); 
        kTest(kType_InitInterfaces(output)); 

        obj->types[obj->typeCount++] = output; 
        *type = output; 
    }
    kCatch(&status) 
    {
        kType_VRelease(output);   
        kSysMemFree(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddClass(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize innerSize, kSize vTableSize, 
                                 void* staticData, kSize staticSize, kStaticInitFx init, kStaticReleaseFx release, volatile kBool* staticInitialized)
{
    kAssemblyClass* obj = assembly; 
    kType output = kNULL; 
    kStatus status;    

    if (kIsNull(base) && !kStrEquals(baseName, "kNull"))
    {
        if (kSuccess(kAssembly_ReorderType(assembly, baseName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the base class with kAddType?
            kAssert(kFALSE); 
            kCheck(kERROR_STATE); 
        }
    }

    kCheck(kType_Construct(&output, type, name, assembly, kTYPE_FLAGS_CLASS, sizeof(kPointer), innerSize));             

    kTry
    {
        kTest(kType_SetBase(output, base)); 
        kTest(kType_InitMethods(output)); 
        kTest(kType_InitVTable(output, vTableSize)); 
        kTest(kType_InitInterfaces(output)); 
        kTest(kType_SetStatic(output, staticData, staticSize, init, release, staticInitialized)); 
    
        obj->types[obj->typeCount++] = output; 
        *type = output; 
    }
    kCatch(&status); 
    {
        kType_VRelease(output);   
        kSysMemFree(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddInterface(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize iTableSize)
{      
    kAssemblyClass* obj = assembly; 
    kType output = kNULL; 
    kStatus status; 

    if (kIsNull(base) && !kStrEquals(baseName, "kNull"))
    {
        if (kSuccess(kAssembly_ReorderType(assembly, baseName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the base interface with kAddType?
            kAssert(kFALSE); 
            kCheck(kERROR_STATE); 
        }
    }

    kCheck(kType_Construct(&output, type, name, assembly, kTYPE_FLAGS_INTERFACE, sizeof(kPointer), 0)); 

    kTry
    {
        kTest(kType_SetBase(output, base)); 
        kTest(kType_InitVTable(output, iTableSize)); 

        obj->types[obj->typeCount++] = output; 
        *type = output; 
    }
    kCatch(&status) 
    {
        kType_VRelease(output);   
        kSysMemFree(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_FindType(kAssembly assembly, const kChar* name, kType* type)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly); 
    
    return kMap_Find(obj->typeMap, name, type); 
}

kFx(kSize) kAssembly_TypeCount(kAssembly assembly)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly); 

    return obj->typeCount; 
}

kFx(kType) kAssembly_TypeAt(kAssembly assembly, kSize index)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly); 
    
    return obj->types[index];
}

kFx(kSize) kAssembly_DependencyCount(kAssembly assembly)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly);

    return obj->dependencyCount;
}

kFx(kAssembly) kAssembly_DependencyAt(kAssembly assembly, kSize index)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly);

    return obj->dependencies[index];
}

kFx(kVersion) kAssembly_Version(kAssembly assembly)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly); 
    
    return obj->version; 
}

kFx(const kChar*) kAssembly_Name(kAssembly assembly)
{
    kAssemblyClass* obj = kAssembly_Cast_(assembly); 
    
    return obj->name; 
}
