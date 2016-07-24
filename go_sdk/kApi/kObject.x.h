/** 
 * @file    kObject.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_OBJECT_X_H
#define K_API_OBJECT_X_H

kBeginHeader()

#define kOBJECT_TAG           (0x44332211)       //used for object validity check (debug feature)

typedef kStatus (kCall* kObjectSerializeFx)(kObject object, kSerializer serializer); 
typedef kStatus (kCall* kObjectDeserializeFx)(kObject object, kSerializer serializer, kAlloc allocator); 

typedef struct kObjectClass
{
    kType type;                      //instance type   
    kAlloc alloc;                    //memory allocator   
    kObjectPool pool;                //object pool (or kNULL)
    k32u tag;                        //used for validity check (lowest bit reserved for legacy 'init' bit)
    kAtomic32s refCount;             //reference count (initially one, destroy when zero)
} kObjectClass;

typedef struct kObjectVTable
{
    kStatus (kCall* VRelease)(kObject object); 
    kStatus (kCall* VDisposeItems)(kObject object); 
    kStatus (kCall* VInitClone)(kObject object, kObject source, kAlloc allocator); 
    kSize (kCall* VHashCode)(kObject object); 
    kBool (kCall* VEquals)(kObject object, kObject other); 
    kSize (kCall* VSize)(kObject object); 
} kObjectVTable; 

kDeclareVirtualClass(k, kObject, kNull)

kFx(kStatus) kObject_Release(kObject object); 
kFx(kStatus) kObject_VRelease(kObject object); 

kFx(kStatus) kObject_DisposeItems(kObject object); 
kFx(kStatus) kObject_VDisposeItems(kObject object); 

kFx(kStatus) kObject_Init(kObject object, kType type, kAlloc allocator); 

kFx(kStatus) kObject_InitClone(kObject object, kObject source, kAlloc allocator); 
kFx(kStatus) kObject_VInitClone(kObject object, kObject source, kAlloc allocator); 

kFx(kSize) kObject_VHashCode(kObject object); 
kFx(kBool) kObject_VEquals(kObject object, kObject other); 
kFx(kSize) kObject_VSize(kObject object); 

kFx(kStatus) kObject_Serialize(kObject object, kSerializer serializer); 
kFx(kStatus) kObject_Deserialize(kObject object, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kObject_GetMem(kObject object, kSize size, void* mem);
kFx(kStatus) kObject_GetMemZero(kObject object, kSize size, void* mem); 
kFx(kStatus) kObject_FreeMem(kObject object, void* mem);
kFx(kStatus) kObject_FreeMemRef(kObject object, void* mem);

#define kObject_(OBJ)                            kCast(kObjectClass*, OBJ)
#define kObject_Cast_(OBJ)                       kCastClass_(kObject, OBJ)
#define kObject_VTable_(OBJ)                     kCastVTable_(kObject, OBJ)

#define kObject_Init_(OBJ, TYPE, ALLOC)          (kObject_(OBJ)->type = (TYPE), kObject_(OBJ)->alloc = (ALLOC),         \
                                                  kObject_(OBJ)->pool = kNULL,                                          \
                                                  kObject_(OBJ)->tag = (kOBJECT_TAG),                                   \
                                                  kAtomic32s_Init_(&kObject_(OBJ)->refCount, 1), kOK)

#define kObject_DisposeItems_(OBJ)               (kObject_VTable_(OBJ)->VDisposeItems(OBJ))
#define kObject_VDisposeItems_(OBJ)              (kOK)

#define kObject_Release_(OBJ)                    (kObject_VTable_(OBJ)->VRelease(OBJ))

#define kObject_VRelease_(OBJ)                   ((kObject_(OBJ)->tag = 0), kOK)

#define kObject_Share_(OBJ)                      (kAtomic32s_Increment_(&kObject_(OBJ)->refCount), kOK)

#define kObject_SetPool_(OBJ, P)                 (kObject_(OBJ)->pool = (P), kOK)

#define kObject_Size_(OBJ)                       (kObject_VTable_(OBJ)->VSize(OBJ))

#define kObject_HashCode_(OBJ)                   (kObject_VTable_(OBJ)->VHashCode(OBJ))
#define kObject_Equals_(OBJ, OTHER)              (kObject_VTable_(OBJ)->VEquals((OBJ), OTHER))

#define kObject_GetMem_(OBJ, SIZE, MEM)          (kAlloc_Get_(kObject_Alloc_(OBJ), SIZE, MEM))
#define kObject_GetMemZero_(OBJ, SIZE, MEM)      (kAlloc_GetZero_(kObject_Alloc_(OBJ), SIZE, MEM))
#define kObject_FreeMem_(OBJ, MEM)               (kAlloc_Free_(kObject_Alloc_(OBJ), MEM))
#define kObject_FreeMemRef_(OBJ, MEM)            (kAlloc_FreeRef_(kObject_Alloc_(OBJ), MEM))

#define kObject_VerifyTag_(OBJ)                  (kObject_(OBJ)->tag == kOBJECT_TAG)
#define kxObject_Type_(OBJ)                      (kObject_(OBJ)->type)
#define kxObject_Allocator_(OBJ)                 (kObject_(OBJ)->alloc)

#define kxObject_Is_(OBJ, TYPE)                  (!kIsNull(OBJ) && !kIsNull(TYPE) && !kIsNull(kObject_(OBJ)->type) &&       \
                                                  kObject_VerifyTag_(OBJ) && kType_Is_(kObject_(OBJ)->type, TYPE))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_OBJECT            kTypeOf(kObject)
#   define kTYPE_DATA              kTypeOf(kObject)
#   define kOBJECT_TYPE            kObject_Type_
#   define kOBJECT_ALLOC           kObject_Alloc_
#   define kOBJECT_IS              kObject_Is_
#   define kData_Dispose           kObject_Dispose
#   define kData_Size              kObject_Size

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#    define kData_Clone(D, C)          kObject_Clone(C, D, kNULL)

#    define kOBJECT(OBJECT)                     ((kObjectClass*)(OBJECT))
#    define kOBJECT_INFO(OBJECT)                ((kTypeClass*)kObject_Type_(OBJECT))
#    define kOBJECT_VTABLE(OBJECT)              (kObject_VTable_(OBJECT))
#    define kOBJECT_RELEASE(OBJECT)             (kObject_Release(OBJECT))

    typedef kObject kData;

#endif

kEndHeader()

#endif
