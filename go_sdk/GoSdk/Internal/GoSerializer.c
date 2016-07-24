/** 
 * @file    GoSerializer.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoSerializer.h>
#include <GoSdk/Messages/GoDataSet.h>
#include <GoSdk/GoSdkLib.h>
#include <stdlib.h>

kBeginValue(Go, GoSerializerTypeInfo, kValue)
    kAddField(GoSerializerTypeInfo, kType, type)
    kAddField(GoSerializerTypeInfo, kPointer, version)
    kAddField(GoSerializerTypeInfo, k16u, id)
kEndValue()

kBeginClass(Go, GoSerializer, kSerializer)    
    kAddVMethod(GoSerializer, kObject, VRelease)
    kAddVMethod(GoSerializer, kSerializer, VInit)
    kAddVMethod(GoSerializer, kSerializer, VWriteObject)
    kAddVMethod(GoSerializer, kSerializer, VReadObject)
kEndClass()

GoFx(kStatus) GoSerializer_Construct(GoSerializer* serializer, kStream stream, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(GoSerializer), serializer)); 

    if (!kSuccess(status = GoSerializer_VInit(*serializer, kTypeOf(GoSerializer), stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

GoFx(kStatus) GoSerializer_VInit(GoSerializer serializer, kType type, kStream stream, kAlloc allocator)
{
    GoSerializerClass* obj = serializer;
    kStatus status; 
    
    kCheck(kSerializer_VInit(serializer, type, stream, allocator)); 
    kInitFields_(GoSerializer, serializer);

    obj->typeToInfo = kNULL; 
    obj->idToType = kNULL; 

    kTry
    {
        kTest(kStrCopy(obj->base.format, kCountOf(obj->base.format), GO_DATA_FORMAT));     
        kTest(kSerializer_SetVersion(serializer, kNULL, GoSdk_ProtocolVersion())); 

        kTest(kMap_Construct(&obj->typeToInfo, kTypeOf(kType), kTypeOf(GoSerializerTypeInfo), 0, allocator)); 
        kTest(kMap_Construct(&obj->idToType, kTypeOf(k16u), kTypeOf(GoSerializerTypeInfo), 0, allocator)); 

        kTest(GoSerializer_BuildIdToTypeMap(serializer));   
    }
    kCatch(&status)
    {
        GoSerializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSerializer_VRelease(GoSerializer serializer)
{
    GoSerializerClass* obj = GoSerializer_Cast_(serializer);
        
    kCheck(kObject_Destroy(obj->typeToInfo)); 
    kCheck(kObject_Destroy(obj->idToType)); 

    kCheck(kSerializer_VRelease(serializer)); 

    return kOK; 
}

GoFx(kStatus) GoSerializer_VWriteObject(GoSerializer serializer, kObject object)
{
    kSize count; 
    kSize i; 
        
    kCheckArgs(!kIsNull(object) && (kObject_Type_(object) == kTypeOf(GoDataSet))); 

    count = GoDataSet_Count(object); 

    for (i = 0; i < count; ++i)
    {
        kObject item = GoDataSet_At(object, i); 
        kBool isLast = (i == (count-1)); 
        kObjectSerializeFx serializeFx = kNULL; 

        kCheck(kSerializer_BeginWrite_(serializer, kTypeOf(k32u), kTRUE)); 
        kCheck(GoSerializer_WriteTypeId(serializer, kObject_Type_(item), isLast, &serializeFx)); 

        kCheck(serializeFx(item, serializer)); 

        kCheck(kSerializer_EndWrite_(serializer)); 
    }

    kCheck(kSerializer_Flush(serializer)); 

    return kOK; 
}

GoFx(kStatus) GoSerializer_VReadObject(GoSerializer serializer, kObject* object, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    GoDataSet output = kNULL;
    kObject item = kNULL; 
    kBool isLast = kFALSE; 
    kStatus status; 

    kTry
    {
        kTest(GoDataSet_Construct(&output, alloc)); 

        while (!isLast)
        {
            kType itemType = kNULL; 
            kObjectDeserializeFx deserializeFx = kNULL; 

            kTest(kSerializer_BeginRead_(serializer, kTypeOf(k32u), kTRUE)); 

            kTest(GoSerializer_ReadTypeId(serializer, &itemType, &isLast, &deserializeFx)); 

            kTest(GoSerializer_ConstructItem(serializer, &item, itemType, deserializeFx, alloc)); 

            kTest(kSerializer_EndRead_(serializer));    

            kTest(GoDataSet_Add(output, item));  
            item = kNULL; 
        }

        *object = output; 
    }
    kCatch(&status)
    {
        kObject_Dispose(item); 
        kObject_Dispose(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSerializer_WriteTypeId(GoSerializer serializer, kType type, kBool isLast, kObjectSerializeFx* fx)
{
    GoSerializerClass* obj = GoSerializer_Cast_(serializer);
    GoSerializerTypeInfo info; 
    k16u typeId;    

    if (!kSuccess(kMap_Find(obj->typeToInfo, &type, &info)))
    {           
        info.type = type; 
        kCheck(kSerializer_FindCompatibleVersion(serializer, type, &info.version));             
        info.id = (k16u) atoi(kType_VersionGuid(info.type, info.version)); 

        kCheck(kMap_Add(obj->typeToInfo, &type, &info)); 
    }
        
    typeId = (k16u) ((isLast << 15) | info.id); 
    
    kCheck(kSerializer_Write16u_(serializer, typeId)); 

    *fx = (kObjectSerializeFx) kType_VersionSerializeFx(info.type, info.version); 

    return kOK; 
}

GoFx(kStatus) GoSerializer_ReadTypeId(GoSerializer serializer, kType* type, kBool* isLast, kObjectDeserializeFx* fx)
{
    GoSerializerClass* obj = GoSerializer_Cast_(serializer);
    GoSerializerTypeInfo info; 
    k16u typeId;    
    k16u id; 

    kCheck(kSerializer_Read16u_(serializer, &typeId)); 
    
    id = typeId & 0x7FFF; 
    *isLast = (typeId >> 15); 

    kCheck(kMap_Find(obj->idToType, &id, &info)); 

    *type = info.type; 
    *fx = (kObjectDeserializeFx) kType_VersionDeserializeFx(info.type, info.version); 
  
    return kOK; 
}

GoFx(kStatus) GoSerializer_ConstructItem(GoSerializer serializer, kObject* item, kType type, kObjectDeserializeFx initializer, kAlloc alloc)
{
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, type, item)); 

    if (!kSuccess(status = initializer(*item, serializer, alloc)))
    {
        kAlloc_FreeRef(alloc, item); 
    }

    return status; 
}

GoFx(kStatus) GoSerializer_BuildIdToTypeMap(GoSerializer serializer)
{
    GoSerializerClass* obj = GoSerializer_Cast_(serializer); 
    kAssembly assembly = kAssemblyOf(GoSdk); 
    kSize typeCount = kAssembly_TypeCount(assembly); 
    GoSerializerTypeInfo info; 
    kSize i, j; 

    kCheck(kMap_Clear(obj->idToType)); 

    for (i = 0; i < typeCount; ++i)
    {
        kType type = kAssembly_TypeAt(assembly, i); 
        kSize versionCount = kType_VersionCount(type); 

        for (j = 0; j < versionCount; ++j)
        {
           kTypeVersion version = kType_VersionAt(type, j); 
           
           if (kStrEquals(kType_VersionFormat(type, version), GO_DATA_FORMAT))
           {
               const kChar* guid = kType_VersionGuid(type, version); 
               
               info.type = type; 
               info.version = version; 
               info.id = (k16u) atoi(guid); 

               kCheck(kMap_Add(obj->idToType, &info.id, &info)); 
           }
        }
    }

    return kOK; 
}
