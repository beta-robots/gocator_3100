/** 
 * @file    kAlloc.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ALLOC_X_H
#define K_API_ALLOC_X_H

kBeginHeader()

typedef struct kAllocStatic
{
    kAlloc systemAlloc;             //Allocates directly from the underlying system (no wrappers).
    kAlloc appAlloc;                //Created for use by application (layered on systemAlloc).
    kAssembly assembly;             //kApi assembly reference. 
} kAllocStatic;

typedef struct kAllocClass
{
    kObjectClass base; 
} kAllocClass;

typedef struct kAllocVTable
{
    kObjectVTable base; 
    kStatus (kCall* VGet)(kAlloc alloc, kSize size, void* mem); 
    kStatus (kCall* VFree)(kAlloc alloc, void* mem); 
} kAllocVTable; 

kDeclareFullClass(k, kAlloc, kObject)

kFx(kStatus) kAlloc_InitStatic(); 
kFx(kStatus) kAlloc_EndInitStatic(); 
kFx(kStatus) kAlloc_ReleaseStatic(); 

kFx(kStatus) kAlloc_Init(kAlloc alloc, kType type, kAlloc allocator); 
kFx(kStatus) kAlloc_VRelease(kAlloc alloc); 

kFx(kStatus) kAlloc_DefaultConstructAppAlloc(kAlloc* appAlloc, kAlloc systemAlloc); 
kFx(kStatus) kAlloc_DefaultDestroyAppAlloc(kAlloc appAlloc); 

kFx(kStatus) kAlloc_OnAssemblyUnloaded(kPointer unused, kAssembly assembly, kPointer args); 

kFx(kStatus) kAlloc_FinalizeDebug(kAlloc* alloc); 

kFx(kStatus) kAlloc_VGet(kAlloc alloc, kSize size, void* mem); 
kFx(kStatus) kAlloc_VFree(kAlloc alloc, void* mem); 

#define kAlloc_(ALLOC)                          (kCast(kAllocClass*, ALLOC))
#define kAlloc_Cast_(ALLOC)                     (kCastClass_(kAlloc, ALLOC))
#define kAlloc_VTable_(ALLOC)                   (kCastVTable_(kAlloc, ALLOC))
#define kAlloc_Static_()                        (kCast(kAllocStatic*, kStaticOf(kAlloc)))

#define kAlloc_GetObject(ALLOC, TYPE, MEM)      (kAlloc_Get(ALLOC, kType_InnerSize_(TYPE), MEM))
#define kAlloc_GetObject_(ALLOC, TYPE, MEM)     (kAlloc_Get_(ALLOC, kType_InnerSize_(TYPE), MEM))

#define kxAlloc_Get_(ALLOC, SIZE, MEM)          (kAlloc_VTable_(ALLOC)->VGet(ALLOC, SIZE, MEM))
#define kxAlloc_GetZero_(ALLOC, SIZE, MEM)      (kAlloc_GetZero(ALLOC, SIZE, MEM))
#define kxAlloc_Free_(ALLOC, MEM)               (kAlloc_VTable_(ALLOC)->VFree(ALLOC, MEM))
#define kxAlloc_FreeRef_(ALLOC, MEM)            (kAlloc_FreeRef(ALLOC, MEM))

#define kxAlloc_App_()                          (kAlloc_Static_()->appAlloc)
#define kxAlloc_System_()                       (kAlloc_Static_()->systemAlloc)
#define kxAlloc_Fallback_(ALLOC)                (kIsNull(ALLOC) ? kAlloc_App_() : (ALLOC))

//deprecated
#define kAlloc_AppHeap      kAlloc_App
#define kAlloc_AppHeap_()   kAlloc_App_() 
#define kAlloc_Default      kAlloc_App
#define kAlloc_Default_()   kAlloc_App_() 
#define kAlloc_SysHeap      kAlloc_App          //Note: this slightly confusing, but correct in terms of backwards compatibility. 
#define kAlloc_SysHeap_()   kAlloc_App_()       //Note: this slightly confusing, but correct in terms of backwards compatibility.                   

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_ALLOC         kTypeOf(kAlloc)                
#   define kAlloc_Delete       kAlloc_FreeRef                

#endif

kEndHeader()

#endif
