/** 
 * @file    kDat6Serializer.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT6_SERIALIZER_X_H
#define K_API_DAT6_SERIALIZER_X_H

#include <kApi/Io/kSerializer.h>

kBeginHeader()

#define kDAT6_FORMAT                             "kdat6"        //format name, used for version info lookup

#define kDAT6_SERIALIZER_CMD_SYNC                (0xA0)         //resynchronize receiver (clear type dictionary, accept size encoding)
#define kDAT6_SERIALIZER_CMD_NULL                (0xA1)         //null object
#define kDAT6_SERIALIZER_CMD_OBJECT              (0xA2)         //non-null object

typedef struct kDat6SerializerTypeInfo
{
    kType type;                     //type object
    kTypeVersion version;           //type object serialization version information
    k16u id;                        //type dictionary id
} kDat6SerializerTypeInfo; 

kDeclareValue(k, kDat6SerializerTypeInfo, kValue)

typedef struct kDat6SerializerStatic
{
    kLock lock;                     //ensures exclusive access to static content
    kMap guidToInfo;                //used by reader; maps type guid to type info (kMap<kText64s, kDat6SerializerTypeInfo)
} kDat6SerializerStatic; 

typedef struct kDat6SerializerVTable
{
    kSerializerVTable base; 
} kDat6SerializerVTable;

typedef struct kDat6SerializerClass
{    
    kSerializerClass base; 

    kBool synchronized;             //if false, sender will prepend dictionary clear command to next dictionary message
    kBool dictionaryEnabled;        //if false, writer will send a 'sync' command before each new object (avoids type dictionary)
    kSize objectDepth;              //recursive call depth for object read/write operations; flush writes when depth is zero
    
    kMap guidToInfo;                //used by reader; maps type guid to type info (kMap<kText64s, kDat6SerializerTypeInfo>)
    kArrayList readerInfo;          //used by reader; index corresponds to type info (kArrayList<kDat6SerializerTypeInfo>)
    kMap typeToInfo;                //used by writer; maps type to type info (kMap<kType, kDat6SerializerTypeInfo>)
} kDat6SerializerClass;

kDeclareFullClass(k, kDat6Serializer, kSerializer)

kFx(kStatus) kDat6Serializer_InitStatic();
kFx(kStatus) kDat6Serializer_ReleaseStatic();
kFx(kStatus) kDat6Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) kDat6Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) kDat6Serializer_GetTypeLookup(kMap* map);

kFx(kStatus) kDat6Serializer_VInit(kDat6Serializer serializer, kType type, kStream stream, kAlloc allocator); 
kFx(kStatus) kDat6Serializer_VRelease(kDat6Serializer serializer); 

kFx(kBool) kDat6Serializer_VReset(kDat6Serializer serializer); 

kFx(kStatus) kDat6Serializer_VWriteObject(kDat6Serializer serializer, kObject object); 
kFx(kStatus) kDat6Serializer_VReadObject(kDat6Serializer serializer, kObject* object, kAlloc allocator); 
kFx(kStatus) kDat6Serializer_VWriteType(kDat6Serializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) kDat6Serializer_VReadType(kDat6Serializer serializer, kType* type, kTypeVersion* version); 

kFx(kStatus) kDat6Serializer_WriteSync(kDat6Serializer serializer); 
kFx(kStatus) kDat6Serializer_ReadSync(kDat6Serializer serializer); 
kFx(kStatus) kDat6Serializer_EnableDictionary(kDat6Serializer serializer, kBool enabled); 

#define kDat6Serializer_(S)                     (kCast(kDat6SerializerClass*, S))
#define kDat6Serializer_Cast_(S)                (kCastClass_(kDat6Serializer, S))
#define kDat6Serializer_VTable_(S)              (kCastVTable_(kDat6Serializer, S))

kEndHeader()

#endif
