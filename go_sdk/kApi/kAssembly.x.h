/** 
 * @file    kAssembly.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ASSEMBLY_X_H
#define K_API_ASSEMBLY_X_H

kBeginHeader()

#define kASSEMBLY_GROWTH_FACTOR                     (2)
#define kASSEMBLY_INITIAL_ASSEMBLY_CAPACITY         (32)
#define kASSEMBLY_INITIAL_DEPEND_CAPACITY           (32)
#define kASSEMBLY_INITIAL_TYPE_CAPACITY             (256)

typedef kStatus (kCall* kAssemblyConstructFx)(kAssembly* assembly); 
typedef kStatus (kCall* kAssemblyRegisterFx)(kAssembly assembly, const kChar* typeName); 

typedef struct kAssemblyRegCallInfo
{
    kAssemblyRegisterFx call; 
    kTypeName name; 
    k32u priority; 
} kAssemblyRegCallInfo; 

typedef struct kAssemblyClass
{
    kObjectClass base; 
    kText32 name; 
    kVersion version; 
    kAssembly* selfReference;
    kBool isRegistered; 
    
    kAssemblyConstructFx* dependCalls;
    kSize dependCallCount; 
    kSize dependCallCapacity; 

    kAssemblyRegCallInfo* regCalls;
    kSize regCallCount; 
    kSize regCallCapacity;
    kBool reordered; 
    kSize nowRegistering; 

    kAssembly* dependencies; 
    kSize dependencyCount; 
    kType* types; 
    kSize typeCount; 
    kMap typeMap; 
} kAssemblyClass;

typedef struct kAssemblyVTable
{
    kObjectVTable base; 
} kAssemblyVTable; 

typedef struct kAssemblyStatic
{
    kLock lock; 
    kAssembly* assemblies; 
    kSize assemblyCount; 
    kSize assemblyCapacity; 
    kEvent onLoad; 
    kEvent onUnload; 
    kEvent onUnloaded; 
} kAssemblyStatic; 

kDeclareFullClass(k, kAssembly, kObject) 

kFx(kStatus) kAssembly_InitStatic(); 
kFx(kStatus) kAssembly_ReleaseStatic(); 

kFx(kStatus) kAssembly_AddAssembly(kAssembly assembly); 
kFx(kStatus) kAssembly_RemoveAssembly(kAssembly assembly); 

kFx(kStatus) kAssembly_ConstructInternal(kAssembly* assembly, kAssembly* reference, const kChar* name, kVersion version);
kFx(kStatus) kAssembly_Init(kAssembly assembly, kType type, kAlloc allocator, kAssembly* reference, const kChar* name, kVersion version); 
kFx(kStatus) kAssembly_VRelease(kAssembly assembly); 

kFx(kStatus) kAssembly_AddDependency(kAssembly assembly, kAssemblyConstructFx depend); 
kFx(kStatus) kAssembly_AddType(kAssembly assembly, kAssemblyRegisterFx call, const kChar* name); 
kFx(kStatus) kAssembly_AddPriority(kAssembly assembly, kAssemblyRegisterFx call, k32u priority); 
kFx(kStatus) kAssembly_Finalize(kAssembly assembly); 

kFx(kStatus) kAssembly_InitDependencies(kAssembly assembly); 
kFx(kStatus) kAssembly_ReleaseDependencies(kAssembly assembly); 

kFx(kStatus) kAssembly_InitTypes(kAssembly assembly); 
kFx(kStatus) kAssembly_ReleaseTypes(kAssembly assembly); 

kFx(kStatus) kAssembly_SortTypes(kAssembly assembly); 

kFx(kStatus) kAssembly_MapTypes(kAssembly assembly); 

kFx(kStatus) kAssembly_AddValue(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize size, kSize vTableSize, kTypeFlags flags); 
kFx(kStatus) kAssembly_AddClass(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize innerSize, kSize vTableSize, 
                                 void* staticData, kSize staticSize, kStaticInitFx init, kStaticReleaseFx release, volatile kBool* staticInitialized); 
kFx(kStatus) kAssembly_AddInterface(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize iTableSize); 

kFx(kStatus) kAssembly_ReorderType(kAssembly assembly, const kChar* dependentOn); 
kFx(kStatus) kAssembly_FindRegCall(kAssembly assembly, const kChar* name, kSize* index); 

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#define kAssembly_Cast_(ASM)            kCastClass_(kAssembly, ASM)

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kType_FromName(B, T)    kAssembly_FindType(kNULL, B, T)

#endif


kEndHeader()

#endif
