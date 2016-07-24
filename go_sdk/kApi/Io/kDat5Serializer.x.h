/** 
 * @file    kDat5Serializer.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT5_SERIALIZER_X_H
#define K_API_DAT5_SERIALIZER_X_H

#include <kApi/Io/kSerializer.h>

kBeginHeader()

#define kDAT5_FORMAT                    "kdat5"             //format name, used for version info lookup
    
#define kDAT5_PROTOCOL_VERSION          (1)                 //this implementation supports version 1 of kDat5 protocol

#define kDAT5_SERIALIZER_CMD_NULL       (-1)                //null object
#define kDAT5_SERIALIZER_CMD_REF        (-2)                //reference to identical object (not supported)

#define kDAT5_SERIALIZER_CMD_REF        (-2)                //reference to identical object (not supported)


#define kDat5Guid_Create_(T, V)         (((V) << 16) | T)           //creates GUID for lookup, using type id and type version
#define kDat5Guid_Type_(G)              ((G) & 0xFFFF)              //gets type id from GUID
#define kDat5Guid_Version_(G)           (((G) >> 16) & 0xFFFF)      //gets type version from GUID

typedef struct kDat5SerializerTypeInfo
{
    kType type;                     //type object
    kTypeVersion version;           //type object serialization version information
    k32u id;                        //type guid
} kDat5SerializerTypeInfo; 

kDeclareValue(k, kDat5SerializerTypeInfo, kValue)

typedef struct kDat5SerializerStatic
{
    kLock lock;                     //ensures exclusive access to static content
    kMap guidToInfo;                //used by reader; maps type guid to type info (kMap<k32u, kDat5SerializerTypeInfo>)
} kDat5SerializerStatic; 

typedef struct kDat5SerializerVTable
{
    kSerializerVTable base; 
} kDat5SerializerVTable;

typedef struct kDat5SerializerClass
{    
    kSerializerClass base; 

    kSize objectDepth;              //recursive call depth for object read/write operations; flush writes when depth is zero    
    kMap guidToInfo;                //used by reader; maps type guid to type info (kMap<k32u, kDat5SerializerTypeInfo>)
    kMap typeToInfo;                //used by writer; maps type to type info (kMap<kType, kDat5SerializerTypeInfo>)
} kDat5SerializerClass;

kDeclareFullClass(k, kDat5Serializer, kSerializer)

kFx(kStatus) kDat5Serializer_InitStatic();
kFx(kStatus) kDat5Serializer_ReleaseStatic();
kFx(kStatus) kDat5Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) kDat5Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) kDat5Serializer_GetTypeLookup(kMap* map);

kFx(kStatus) kDat5Serializer_VInit(kDat5Serializer serializer, kType type, kStream stream, kAlloc allocator); 
kFx(kStatus) kDat5Serializer_VRelease(kDat5Serializer serializer); 

kFx(kStatus) kDat5Serializer_VWriteObject(kDat5Serializer serializer, kObject object); 
kFx(kStatus) kDat5Serializer_VReadObject(kDat5Serializer serializer, kObject* object, kAlloc allocator); 
kFx(kStatus) kDat5Serializer_VWriteType(kDat5Serializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) kDat5Serializer_VReadType(kDat5Serializer serializer, kType* type, kTypeVersion* version); 

//These methods shouldn't be necessary, but a bug/limitation in kImage serialization version 5-3 necessitates them. 
kFx(kStatus) kDat5Serializer_WriteTypeWithoutVersion(kDat5Serializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) kDat5Serializer_ReadTypeExplicitVersion(kDat5Serializer serializer, k32u typeVersionId, kType* type, kTypeVersion* version); 

kFx(kStatus) kDat5Serializer_WriteData(kDat5Serializer serializer, kObject object, const kChar* label);
kFx(kStatus) kDat5Serializer_ReadData(kDat5Serializer serializer, kObject* object, kChar* label, kSize capacity, kAlloc allocator);

#define kDat5Serializer_(S)                     (kCast(kDat5SerializerClass*, S))
#define kDat5Serializer_Cast_(S)                (kCastClass_(kDat5Serializer, S))
#define kDat5Serializer_VTable_(S)              (kCastVTable_(kDat5Serializer, S))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_SERIALIZER            kTypeOf(kDat5Serializer)

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kDat5Serializer_Construct5(SRL, STR)       kDat5Serializer_Construct(SRL, STR, kNULL)

#endif

kEndHeader()

#endif
