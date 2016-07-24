/** 
 * @file    kDat6Serializer.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Threads/kLock.h>

kBeginValue(k, kDat6SerializerTypeInfo, kValue)
    kAddField(kDat6SerializerTypeInfo, kType, type)
    kAddField(kDat6SerializerTypeInfo, kPointer, version)
    kAddField(kDat6SerializerTypeInfo, k16u, id)
kEndValue()

kBeginFullClass(k, kDat6Serializer, kSerializer)    
    kAddVMethod(kDat6Serializer, kObject, VRelease)
    kAddVMethod(kDat6Serializer, kSerializer, VInit)
    kAddVMethod(kDat6Serializer, kSerializer, VReset)
    kAddVMethod(kDat6Serializer, kSerializer, VWriteObject)
    kAddVMethod(kDat6Serializer, kSerializer, VReadObject)
    kAddVMethod(kDat6Serializer, kSerializer, VWriteType)
    kAddVMethod(kDat6Serializer, kSerializer, VReadType)
kEndFullClass()

kFx(kStatus) kDat6Serializer_InitStatic()
{
    kDat6SerializerStatic* sobj = kStaticOf(kDat6Serializer); 

    kCheck(kLock_Construct(&sobj->lock, kNULL)); 
    kCheck(kMap_Construct(&sobj->guidToInfo, kTypeOf(kText64), kTypeOf(kDat6SerializerTypeInfo), 256, kNULL)); 

    kCheck(kAssembly_AddLoadHandler(kDat6Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_AddUnloadHandler(kDat6Serializer_OnAssemblyUnload, kNULL)); 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_ReleaseStatic()
{
    kDat6SerializerStatic* sobj = kStaticOf(kDat6Serializer); 

    kCheck(kAssembly_RemoveLoadHandler(kDat6Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_RemoveUnloadHandler(kDat6Serializer_OnAssemblyUnload, kNULL)); 
    
    kCheck(kDestroyRef(&sobj->guidToInfo)); 
    kCheck(kDestroyRef(&sobj->lock)); 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args)
{
    kDat6SerializerStatic* sobj = kStaticOf(kDat6Serializer); 
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
            kText64 guid; 

            for (j = 0; j < kType_VersionCount(type); ++j)
            {
                kTypeVersion version = kType_VersionAt(type, j); 

                if (kStrEquals(kType_VersionFormat(type, version), kDAT6_FORMAT))
                {
                    kDat6SerializerTypeInfo info; 

                    info.type = type; 
                    info.version = version; 
                    info.id = 0; 

                    kTest(kStrCopy(guid, kCountOf(guid), kType_VersionGuid(type, version))); 
                    kTest(kMap_Replace(sobj->guidToInfo, guid, &info)); 
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

kFx(kStatus) kDat6Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args)
{
    kDat6SerializerStatic* sobj = kStaticOf(kDat6Serializer); 
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

                if (kStrEquals(kType_VersionFormat(type, version), kDAT6_FORMAT))
                {                   
                    if (kSuccess(kMap_FindItem(sobj->guidToInfo, kType_VersionGuid(type, version), &item)))
                    {
                        kTest(kMap_RemoveItem(sobj->guidToInfo, item)); 
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

kFx(kStatus) kDat6Serializer_GetTypeLookup(kMap* map)
{
    kDat6SerializerStatic* sobj = kStaticOf(kDat6Serializer); 
    
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

kFx(kStatus) kDat6Serializer_Construct(kDat6Serializer* serializer, kStream stream, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kDat6Serializer), serializer)); 

    if (!kSuccess(status = kDat6Serializer_VInit(*serializer, kTypeOf(kDat6Serializer), stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

kFx(kStatus) kDat6Serializer_VInit(kDat6Serializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kDat6SerializerClass* obj = serializer;
    kStatus status; 
    
    kCheck(kSerializer_VInit(serializer, type, stream, allocator)); 

    kCheck(kStrCopy(obj->base.format, kCountOf(obj->base.format), kDAT6_FORMAT));     

    obj->base.sizeEncoding = K_POINTER_SIZE; 
    obj->synchronized = kFALSE; 
    obj->dictionaryEnabled = kTRUE; 
    obj->objectDepth = 0; 
    obj->guidToInfo = kNULL; 
    obj->readerInfo = kNULL; 
    obj->typeToInfo = kNULL; 

    kTry
    {
        kTest(kDat6Serializer_GetTypeLookup(&obj->guidToInfo)); 
        kTest(kArrayList_Construct(&obj->readerInfo, kTypeOf(kDat6SerializerTypeInfo), 64, allocator)); 
        kTest(kMap_Construct(&obj->typeToInfo, kTypeOf(kType), kTypeOf(kDat6SerializerTypeInfo), 64, allocator)); 
    }
    kCatch(&status)
    {
        kDat6Serializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VRelease(kDat6Serializer serializer)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 
        
    kCheck(kObject_Destroy(obj->guidToInfo)); 
    kCheck(kObject_Destroy(obj->readerInfo)); 
    kCheck(kObject_Destroy(obj->typeToInfo)); 

    kCheck(kSerializer_VRelease(serializer)); 

    return kOK; 
}

kFx(kBool) kDat6Serializer_VReset(kDat6Serializer serializer)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 
    
    kCheck(kMap_Clear(obj->typeToInfo));      
    kCheck(kArrayList_Clear(obj->readerInfo));

    obj->objectDepth = 0; 
    obj->synchronized = kFALSE; 

    kCheck(kSerializer_VReset(serializer)); 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VWriteObject(kDat6Serializer serializer, kObject object)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 

    obj->objectDepth++; 

    kTry
    {
        if (!obj->dictionaryEnabled && (obj->objectDepth == 1))
        {
            obj->synchronized = kFALSE; 
        }

        if (!obj->synchronized)
        {
            kTest(kSerializer_WriteByte_(serializer, kDAT6_SERIALIZER_CMD_SYNC)); 
            kTest(kDat6Serializer_WriteSync(serializer)); 
            kTest(kMap_Clear(obj->typeToInfo)); 

            obj->synchronized = kTRUE; 
        }

        if (kIsNull(object))
        {
            kTest(kSerializer_WriteByte_(serializer, kDAT6_SERIALIZER_CMD_NULL)); 
        }
        else
        {
            kType type = kObject_Type_(object); 
            kTypeVersion version = kNULL; 

            kTest(kSerializer_WriteByte_(serializer, kDAT6_SERIALIZER_CMD_OBJECT));
            
            kTest(kSerializer_WriteType_(serializer, type, &version));        

            kTest(kType_SerializeFx_(type, version, kObject)(object, serializer)); 
        }
    }
    kFinally
    {
        obj->objectDepth--; 
        kEndFinally(); 
    }

    if ((obj->objectDepth == 0) && (kArrayList_Count_(obj->base.writeSections) == 0))
    {
        kCheck(kSerializer_Flush(serializer)); 
    }

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VReadObject(kDat6Serializer serializer, kObject* object, kAlloc allocator)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kByte command = 0; 

    obj->objectDepth++; 

    kTry
    {
        obj->base.readAlloc = alloc; 

        kTest(kSerializer_ReadByte_(serializer, &command)); 

        if (command == kDAT6_SERIALIZER_CMD_SYNC)
        {
            kTest(kDat6Serializer_ReadSync(serializer));             
            kTest(kSerializer_ReadByte_(serializer, &command)); 
        }

        if (command == kDAT6_SERIALIZER_CMD_NULL)
        {
            *object = kNULL; 
        }
        else if (command == kDAT6_SERIALIZER_CMD_OBJECT)
        {
            kType type = kNULL; 
            kTypeVersion version = kNULL; 
            kStatus status; 

            kTest(kSerializer_ReadType_(serializer, &type, &version));        

            kTest(kAlloc_GetObject_(alloc, type, object)); 

            if (!kSuccess(status = kType_DeserializeFx_(type, version, kObject)(*object, serializer, alloc)))    
            {
                kAlloc_FreeRef(alloc, object); 
                kThrow(status); 
            }
        }
        else
        {
            kThrow(kERROR_STREAM); 
        }
    }
    kFinally
    {
        obj->objectDepth--; 
        kEndFinally(); 
    }
   
    return kOK; 
}

kFx(kStatus) kDat6Serializer_VWriteType(kDat6Serializer serializer, kType type, kTypeVersion* version)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 
    kDat6SerializerTypeInfo info; 

    if (!kSuccess(kMap_Find(obj->typeToInfo, &type, &info)))
    {    
        k16u dictionaryId = (k16u) kMap_Count_(obj->typeToInfo) + 1; 
        const kChar* guid = kNULL; 
        k8u size; 

        kCheck(kSerializer_FindCompatibleVersion(serializer, type, version)); 

        info.type = type; 
        info.version = *version; 
        info.id = dictionaryId; 

        kCheck(kMap_Add(obj->typeToInfo, &type, &info)); 

        guid = kType_VersionGuid(type, *version); 
        size = (k8u) kStrLength(guid); 

        kCheck(kSerializer_Write16u_(serializer, 0)); 
        kCheck(kSerializer_Write16u_(serializer, dictionaryId)); 
        kCheck(kSerializer_Write8u_(serializer, size)); 
        kCheck(kSerializer_WriteCharArray_(serializer, guid, size)); 
    }
    else
    {
        kCheck(kSerializer_Write16u_(serializer, info.id)); 
    }
       
    *version = info.version; 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VReadType(kDat6Serializer serializer, kType* type, kTypeVersion* version)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 
    kDat6SerializerTypeInfo* info = kNULL; 
    kMapItem item = kNULL; 
    k16u dictionaryId; 

    kCheck(kSerializer_Read16u_(serializer, &dictionaryId)); 

    if (dictionaryId == 0)
    {
        kText64 guid; 
        k16u expectedId = (k16u) kArrayList_Count(obj->readerInfo) + 1; 
        k8u size; 

        kCheck(kSerializer_Read16u_(serializer, &dictionaryId));         
        kCheck(dictionaryId == expectedId); 

        kCheck(kSerializer_Read8u_(serializer, &size)); 
        kCheck(size < kCountOf(guid)); 

        kCheck(kSerializer_ReadCharArray_(serializer, guid, size)); 
        guid[size] = 0; 

        kCheck(kMap_FindItem(obj->guidToInfo, guid, &item)); 
        kCheck(kArrayList_Add(obj->readerInfo, kMap_Value(obj->guidToInfo, item))); 
    }
    
    if (dictionaryId <= kArrayList_Count_(obj->readerInfo))
    {
        info = kArrayList_At_(obj->readerInfo, dictionaryId-1); 

        *type = info->type; 
        *version = info->version; 
    }
    else
    {
        return kERROR_NOT_FOUND; 
    }   
  
    return kOK; 
}

kFx(kStatus) kDat6Serializer_WriteSync(kDat6Serializer serializer)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 

    kCheck(kSerializer_BeginWrite_(serializer, kTypeOf(k16u), kTRUE)); 

    kCheck(kSerializer_WriteByte_(serializer, (kByte)obj->base.sizeEncoding)); 

    kCheck(kSerializer_EndWrite_(serializer)); 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_ReadSync(kDat6Serializer serializer)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 
    kByte sizeEncoding; 

    kCheck(kSerializer_BeginRead_(serializer, kTypeOf(k16u), kTRUE)); 

    kCheck(kSerializer_ReadByte_(serializer, &sizeEncoding)); 
                
    kCheck(kSerializer_EndRead_(serializer)); 

    if ((sizeEncoding != 4) && (sizeEncoding != 8))
    {
        return kERROR_FORMAT; 
    }

    obj->base.sizeEncoding = sizeEncoding; 

    kCheck(kArrayList_Clear(obj->readerInfo)); 
   
    return kOK; 
}

kFx(kStatus) kDat6Serializer_EnableDictionary(kDat6Serializer serializer, kBool enabled)
{
    kDat6SerializerClass* obj = kDat6Serializer_Cast_(serializer); 

    obj->dictionaryEnabled = enabled; 

    return kOK; 
}
