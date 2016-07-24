/** 
 * @file    GoSerializer.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SERIALIZER_X_H
#define GO_SDK_SERIALIZER_X_H

#include <kApi/Data/kMap.h>

kBeginHeader()

#define GO_DATA_FORMAT              "godat"        //format name, used for version info lookup

typedef struct GoSerializerTypeInfo
{
    kType type;                     //type object
    kTypeVersion version;           //type object serialization version information
    k16u id;                        //numeric guid
} GoSerializerTypeInfo; 

kDeclareValue(Go, GoSerializerTypeInfo, kValue)

typedef struct GoSerializerClass
{    
    kSerializerClass base;     
    kMap typeToInfo;              //used by writer; maps type pointer to type version info (kMap<kType, GoSerializerTypeInfo>)
    kMap idToType;                //used by reader; maps type id to type version info (kMap<k16u, GoSerializerTypeInfo>)
} GoSerializerClass;

kDeclareClass(Go, GoSerializer, kSerializer)

#define GoSerializer_Cast_(CONTEXT)    kCastClass_(GoSerializer, CONTEXT)

GoFx(kStatus) GoSerializer_VInit(GoSerializer serializer, kType type, kStream stream, kAlloc allocator); 
GoFx(kStatus) GoSerializer_VRelease(GoSerializer serializer); 

GoFx(kStatus) GoSerializer_VWriteObject(GoSerializer serializer, kObject object); 
GoFx(kStatus) GoSerializer_VReadObject(GoSerializer serializer, kObject* object, kAlloc allocator); 

GoFx(kStatus) GoSerializer_WriteTypeId(GoSerializer serializer, kType type, kBool isLast, kObjectSerializeFx* fx); 
GoFx(kStatus) GoSerializer_ReadTypeId(GoSerializer serializer, kType* type, kBool* isLast, kObjectDeserializeFx* fx); 

GoFx(kStatus) GoSerializer_ConstructItem(GoSerializer serializer, kObject* item, kType type, kObjectDeserializeFx initializer, kAlloc alloc); 

GoFx(kStatus) GoSerializer_BuildIdToTypeMap(GoSerializer serializer); 

kEndHeader()

#endif
