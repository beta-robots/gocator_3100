/** 
 * @file    kDat5Serializer.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kDat5Serializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Threads/kLock.h>
#include <stdio.h>

kBeginValue(k, kDat5SerializerTypeInfo, kValue)
    kAddField(kDat5SerializerTypeInfo, kType, type)
    kAddField(kDat5SerializerTypeInfo, kPointer, version)
    kAddField(kDat5SerializerTypeInfo, k32u, id)
kEndValue()

kBeginFullClass(k, kDat5Serializer, kSerializer)    
    kAddVMethod(kDat5Serializer, kObject, VRelease)
    kAddVMethod(kDat5Serializer, kSerializer, VInit)
    kAddVMethod(kDat5Serializer, kSerializer, VWriteObject)
    kAddVMethod(kDat5Serializer, kSerializer, VReadObject)
    kAddVMethod(kDat5Serializer, kSerializer, VWriteType)
    kAddVMethod(kDat5Serializer, kSerializer, VReadType)
kEndFullClass()

kFx(kStatus) kDat5Serializer_InitStatic()
{
    kDat5SerializerStatic* sobj = kStaticOf(kDat5Serializer); 

    kCheck(kLock_Construct(&sobj->lock, kNULL)); 
    kCheck(kMap_Construct(&sobj->guidToInfo, kTypeOf(k32u), kTypeOf(kDat5SerializerTypeInfo), 256, kNULL)); 

    kCheck(kAssembly_AddLoadHandler(kDat5Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_AddUnloadHandler(kDat5Serializer_OnAssemblyUnload, kNULL)); 

    return kOK; 
}

kFx(kStatus) kDat5Serializer_ReleaseStatic()
{
    kDat5SerializerStatic* sobj = kStaticOf(kDat5Serializer); 

    kCheck(kAssembly_RemoveLoadHandler(kDat5Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_RemoveUnloadHandler(kDat5Serializer_OnAssemblyUnload, kNULL)); 
    
    kCheck(kDestroyRef(&sobj->guidToInfo)); 
    kCheck(kDestroyRef(&sobj->lock)); 

    return kOK; 
}

kFx(kStatus) kDat5Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args)
{
    kDat5SerializerStatic* sobj = kStaticOf(kDat5Serializer); 
    kSize i, j; 

    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        if (kObject_IsShared(sobj->guidToInfo))
        {
            kMap clone = kNULL; 

            kTest(kObject_Clone(&clone, sobj->guidToInfo, kNULL)); 
            kTest(kObject_Destroy(sobj->guidToInfo)); 

            sobj->guidToInfo = clone; 
        }
        
        for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
        {
            kType type = kAssembly_TypeAt(assembly, i); 

            for (j = 0; j < kType_VersionCount(type); ++j)
            {
                kTypeVersion version = kType_VersionAt(type, j); 

                if (kStrEquals(kType_VersionFormat(type, version), kDAT5_FORMAT))
                {
                    kDat5SerializerTypeInfo info; 
                    k32u typeId, typeVersionId; 

                    info.type = type; 
                    info.version = version; 

                    if (sscanf(kType_VersionGuid(type, version), "%u-%u", &typeId, &typeVersionId) == 2)
                    {
                        info.id = kDat5Guid_Create_(typeId, typeVersionId); 

                        kTest(kMap_Replace(sobj->guidToInfo, &info.id, &info)); 
                    }
                }
            }
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock);
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args)
{
    kDat5SerializerStatic* sobj = kStaticOf(kDat5Serializer); 
    kSize i, j; 

    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        if (kObject_IsShared(sobj->guidToInfo))
        {
            kMap clone = kNULL; 

            kTest(kObject_Clone(&clone, sobj->guidToInfo, kNULL)); 
            kTest(kObject_Destroy(sobj->guidToInfo)); 

            sobj->guidToInfo = clone; 
        }
       
        for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
        {
            kType type = kAssembly_TypeAt(assembly, i); 
            kMapItem item = kNULL; 

            for (j = 0; j < kType_VersionCount(type); ++j)
            {
                kTypeVersion version = kType_VersionAt(type, j); 

                if (kStrEquals(kType_VersionFormat(type, version), kDAT5_FORMAT))
                {                   
                    k32u typeId, typeVersionId, typeGuid; 

                    if (sscanf(kType_VersionGuid(type, version), "%u-%u", &typeId, &typeVersionId) == 2)
                    {
                        typeGuid = kDat5Guid_Create_(typeId, typeVersionId); 

                        if (kSuccess(kMap_FindItem(sobj->guidToInfo, &typeGuid, &item)))
                        {
                            kTest(kMap_RemoveItem(sobj->guidToInfo, item)); 
                        }
                    }
                }
            }
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock);
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_GetTypeLookup(kMap* map)
{
    kDat5SerializerStatic* sobj = kStaticOf(kDat5Serializer); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kObject_Share(sobj->guidToInfo)); 
        *map = sobj->guidToInfo; 
    }
    kFinally
    {
        kLock_Exit(sobj->lock);
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_Construct(kDat5Serializer* serializer, kStream stream, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kDat5Serializer), serializer)); 

    if (!kSuccess(status = kDat5Serializer_VInit(*serializer, kTypeOf(kDat5Serializer), stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

kFx(kStatus) kDat5Serializer_VInit(kDat5Serializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kDat5SerializerClass* obj = serializer;
    kStatus status; 
    
    kCheck(kSerializer_VInit(serializer, type, stream, allocator)); 

    kCheck(kStrCopy(obj->base.format, kCountOf(obj->base.format), kDAT5_FORMAT));     

    obj->objectDepth = 0; 
    obj->guidToInfo = kNULL; 
    obj->typeToInfo = kNULL; 

    kTry
    {
        kTest(kDat5Serializer_GetTypeLookup(&obj->guidToInfo)); 
        kTest(kMap_Construct(&obj->typeToInfo, kTypeOf(kType), kTypeOf(kDat5SerializerTypeInfo), 64, allocator)); 
    }
    kCatch(&status)
    {
        kDat5Serializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_VRelease(kDat5Serializer serializer)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
        
    kCheck(kObject_Destroy(obj->guidToInfo)); 
    kCheck(kObject_Destroy(obj->typeToInfo)); 

    kCheck(kSerializer_VRelease(serializer)); 

    return kOK; 
}

kFx(kStatus) kDat5Serializer_WriteData(kDat5Serializer serializer, kObject object, const kChar* label)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
    kText16 dummyLabel = { 0 }; 

    obj->objectDepth++; 

    kTry
    {
        if (obj->objectDepth == 1)
        {
            kTest(kSerializer_WriteByte_(serializer, kDAT5_PROTOCOL_VERSION)); 
        }

        if (kIsNull(object))
        {
            kTest(kSerializer_Write32s_(serializer, kDAT5_SERIALIZER_CMD_NULL)); 
            kTest(kSerializer_Write32u_(serializer, 0)); 
        }
        else
        {
            kType type = kObject_Type_(object); 
            kTypeVersion version = kNULL; 
            
            kTest(kSerializer_WriteType_(serializer, type, &version));        

            kTest(kType_SerializeFx_(type, version, kObject)(object, serializer)); 

            kTest(kSerializer_Write32u_(serializer, 0));    //kData version

            if (!kIsNull(label))
            {
                if (!kSuccess(kStrCopy(dummyLabel, sizeof(dummyLabel), label))) {/* don't care */}
            }

            kTest(kSerializer_WriteCharArray_(serializer, dummyLabel, sizeof(dummyLabel)));  
        }
    }
    kFinally
    {
        obj->objectDepth--; 
        kEndFinally(); 
    }

    if (obj->objectDepth == 0)
    {
        kCheck(kSerializer_Flush(serializer)); 
        kCheckState(kArrayList_Count_(obj->base.writeSections) == 0); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_VWriteObject(kDat5Serializer serializer, kObject object)
{
    return kDat5Serializer_WriteData(serializer, object, kNULL);
}

kFx(kStatus) kDat5Serializer_ReadData(kDat5Serializer serializer, kObject* object, kChar* label, kSize capacity, kAlloc allocator)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kType type = kNULL; 
    kTypeVersion version = kNULL; 
    kByte protocolVersion = 0;
    kObject output = kNULL;
    kText16 dummyLabel; 
    k32u dataVersion; 
    kStatus status; 

    obj->objectDepth++; 

    kTry
    {
        obj->base.readAlloc = alloc; 

        if (obj->objectDepth == 1)
        {
            kTest(kSerializer_ReadByte_(serializer, &protocolVersion)); 
            kTestState(protocolVersion == kDAT5_PROTOCOL_VERSION); 
        }

        kTest(kSerializer_ReadType_(serializer, &type, &version)); 

        if (kIsNull(type))
        {
            *object = kNULL; 
        }
        else
        {
            kTest(kAlloc_GetObject_(alloc, type, &output)); 

            kTest(kType_DeserializeFx_(type, version, kObject)(output, serializer, alloc));

            kTest(kSerializer_Read32u_(serializer, &dataVersion));
            kTest(kSerializer_ReadCharArray_(serializer, dummyLabel, kCountOf(dummyLabel))); 

            if (!kIsNull(label) && capacity != 0)
            {
                kTest(kStrCopy(label, capacity, dummyLabel));
            }
        }
    }
    kCatchEx(&status)
    {
        if (!kIsNull(output)) kAlloc_FreeRef(alloc, &output); 
        kEndCatchEx(status);
    }
    kFinallyEx
    {
        obj->objectDepth--; 
        kEndFinally(); 
    }

    if (obj->objectDepth == 0)
    {
        kCheckState(kArrayList_Count_(obj->base.readSections) == 0); 
    }

    *object = output;
    return kOK; 
}

kFx(kStatus) kDat5Serializer_VReadObject(kDat5Serializer serializer, kObject* object, kAlloc allocator)
{
    return kDat5Serializer_ReadData(serializer, object, kNULL, 0, allocator);
}

kFx(kStatus) kDat5Serializer_VWriteType(kDat5Serializer serializer, kType type, kTypeVersion* version)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
    kDat5SerializerTypeInfo info; 
    k32u typeId, typeVersionId; 

    if (!kSuccess(kMap_Find(obj->typeToInfo, &type, &info)))
    {    
        kCheck(kSerializer_FindCompatibleVersion(serializer, type, version)); 

        if (sscanf(kType_VersionGuid(type, *version), "%u-%u", &typeId, &typeVersionId) != 2)
        {
            return kERROR_FORMAT; 
        }

        info.type = type; 
        info.version = *version;        
        info.id = kDat5Guid_Create_(typeId, typeVersionId); 

        kCheck(kMap_Add(obj->typeToInfo, &type, &info));         
    }
    else
    {
        typeId = kDat5Guid_Type_(info.id); 
        typeVersionId = kDat5Guid_Version_(info.id); 
    }

    kCheck(kSerializer_Write32u_(serializer, typeId)); 
    kCheck(kSerializer_Write32u_(serializer, typeVersionId)); 

    *version = info.version; 

    return kOK; 
}

kFx(kStatus) kDat5Serializer_VReadType(kDat5Serializer serializer, kType* type, kTypeVersion* version)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
    const kDat5SerializerTypeInfo* info = kNULL; 
    kMapItem item = kNULL; 
    k32s typeId; 
    k32s typeVersionId; 
    k32u guid; 

    kCheck(kSerializer_Read32s_(serializer, &typeId)); 
    kCheck(kSerializer_Read32u_(serializer, &typeVersionId)); 

    if (typeId == kDAT5_SERIALIZER_CMD_NULL)
    {
        //null is treated as special case
        *type = kNULL; 
        *version = kNULL; 

        return kOK; 
    }
    else if (typeId == kDAT5_SERIALIZER_CMD_REF)
    {
        //references not supported
        return kERROR_FORMAT; 
    }

    guid = kDat5Guid_Create_((k32u)typeId, typeVersionId); 

    kCheck(kMap_FindItem(obj->guidToInfo, &guid, &item));
    info = kMap_Value(obj->guidToInfo, item); 

    *type = info->type; 
    *version = info->version; 
  
    return kOK; 
}

kFx(kStatus) kDat5Serializer_WriteTypeWithoutVersion(kDat5Serializer serializer, kType type, kTypeVersion* version)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
    kDat5SerializerTypeInfo info; 
    k32u typeId, typeVersionId; 

    if (!kSuccess(kMap_Find(obj->typeToInfo, &type, &info)))
    {    
        kCheck(kSerializer_FindCompatibleVersion(serializer, type, version)); 

        if (sscanf(kType_VersionGuid(type, *version), "%u-%u", &typeId, &typeVersionId) != 2)
        {
            return kERROR_FORMAT; 
        }

        info.type = type; 
        info.version = *version;        
        info.id = kDat5Guid_Create_(typeId, typeVersionId); 

        kCheck(kMap_Add(obj->typeToInfo, &type, &info));         
    }
    else
    {
        typeId = kDat5Guid_Type_(info.id); 
        typeVersionId = kDat5Guid_Version_(info.id); 
    }

    kCheck(kSerializer_Write32u_(serializer, typeId)); 

    *version = info.version; 

    return kOK; 
}

kFx(kStatus) kDat5Serializer_ReadTypeExplicitVersion(kDat5Serializer serializer, k32u typeVersionId, kType* type, kTypeVersion* version)
{
    kDat5SerializerClass* obj = kDat5Serializer_Cast_(serializer); 
    const kDat5SerializerTypeInfo* info = kNULL; 
    kMapItem item = kNULL; 
    k32s typeId; 
    k32u guid; 

    kCheck(kSerializer_Read32s_(serializer, &typeId)); 
    
    if (typeId == kDAT5_SERIALIZER_CMD_NULL)
    {
        //null is treated as special case
        *type = kNULL; 
        *version = kNULL; 

        return kOK; 
    }
    else if (typeId == kDAT5_SERIALIZER_CMD_REF)
    {
        //references not supported
        return kERROR_FORMAT; 
    }

    guid = kDat5Guid_Create_((k32u)typeId, typeVersionId); 

    kCheck(kMap_FindItem(obj->guidToInfo, &guid, &item));
    info = kMap_Value(obj->guidToInfo, item); 

    *type = info->type; 
    *version = info->version; 
  
    return kOK; 
}
