/** 
 * @file  kType.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kType.h>
#include <kApi/kApiLib.h>
#include <stdio.h>

kBeginClass(k, kType, kObject)
    kAddVMethod(kType, kObject, VRelease)
kEndClass()

kFx(kStatus) kType_Construct(kType* type, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize)
{
    kType output = kNULL; 
    kStatus status; 

    kCheck(kSysMemAlloc(sizeof(kTypeClass), &output)); 

    if (!kSuccess(status = kType_Init(output, kNULL, kNULL, reference, name, assembly, flags, size, innerSize)))
    {
        kSysMemFree(output); 
        return status; 
    }

    *type = output; 
          
    return kOK; 
} 

kFx(kStatus) kType_Init(kType type, kType typeType, kAlloc allocator, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize)
{
    kTypeClass* obj = type; 

    kCheck(kObject_Init(type, typeType, allocator)); 
    
    /* kType instances are zero-initialized on allocation; most fields do not require explicit initialization. */
    obj->assembly = assembly; 
    obj->selfReference = reference; 
    kCheck(kStrCopy(obj->name, kCountOf(obj->name), name));   
    obj->flags = flags; 
    obj->size = size; 
    obj->innerSize = innerSize; 
    obj->staticPriority = k16S_MAX; 
   
    return kOK; 
}

kFx(kStatus) kType_VRelease(kType type)
{
    kTypeClass* obj = type; 
    kSize i; 

    *obj->selfReference = kNULL; 

    for (i = 0; i < obj->interfaceCount; ++i)
    {            
        kCheck(kSysMemFree(obj->interfaces[i].iTable)); 
        kCheck(kSysMemFree(obj->interfaces[i].iMethodInfo)); 
    }

    kCheck(kSysMemFree(obj->vTable)); 
    kCheck(kSysMemFree(obj->vMethodInfo)); 
    kCheck(kSysMemFree(obj->cMethodInfo)); 
    kCheck(kSysMemFree(obj->fieldInfo)); 
    kCheck(kSysMemFree(obj->enumeratorInfo)); 
    kCheck(kSysMemFree(obj->versionInfo)); 

    kCheck(kObject_VRelease(type)); 

    return kOK; 
}

kFx(kStatus) kType_SetStatic(kType type, void* staticReference, kSize staticSize, kStaticInitFx init, kStaticReleaseFx release, volatile kBool* staticInitialized)
{
    kTypeClass* obj = type; 

    obj->staticData = staticReference; 
    obj->staticSize = staticSize; 
    obj->staticInit = init; 
    obj->staticRelease = release; 
    obj->staticInitialized = staticInitialized; 
    
    return kOK; 
}

kFx(kStatus) kType_SetBase(kType type, kType base)
{
    kTypeClass* obj = type; 
    kType baseIt = base; 

    obj->baseCount = 0; 

    while (!kIsNull(baseIt) && (obj->baseCount < kTYPE_MAX_BASES))
    {        
        obj->bases[obj->baseCount++] = baseIt; 
        baseIt = kType_Base_(baseIt);  
    }
   
    return !kIsNull(baseIt) ? kERROR_PARAMETER : kOK; 
}

kFx(kStatus) kType_InitMethods(kType type)
{
    kTypeClass* obj = type; 
    kType base = kType_Base(type); 
    kTypeClass* baseObj = base; 

    if (!kIsNull(base) && (baseObj->cMethodCount > 0))
    {
        kCheck(kSysMemReallocList(&obj->cMethodInfo, 0, sizeof(kMethodInfo), &obj->cMethodCapacity, 
            kTYPE_INITIAL_CMETHOD_CAPACITY, baseObj->cMethodCapacity)); 

        kCheck(kMemCopy(obj->cMethodInfo, baseObj->cMethodInfo, obj->cMethodCount * sizeof(kMethodInfo))); 
    }
    
    return kOK; 
}

kFx(kStatus) kType_InitVTable(kType type, kSize vTableSize)
{
    kTypeClass* obj = type; 
    kType base = kType_Base(type); 
    kSize vMethodCount = (vTableSize / sizeof(kPointer)); 

    kCheck(kSysMemAlloc(vMethodCount * sizeof(kPointer), &obj->vTable)); 
    kCheck(kSysMemAlloc(vMethodCount * sizeof(kMethodInfo), &obj->vMethodInfo)); 

    obj->vMethodCount = vMethodCount; 

    if (!kIsNull(base))
    {
        kTypeClass* baseObj = base; 

        kMemCopy(obj->vTable, baseObj->vTable, baseObj->vMethodCount * sizeof(kPointer)); 
        kMemCopy(obj->vMethodInfo, baseObj->vMethodInfo, baseObj->vMethodCount * sizeof(kMethodInfo)); 
    }

    return kOK; 
}

kFx(kStatus) kType_InitInterfaces(kType type)
{
    kTypeClass* obj = type; 
    kType base = kType_Base(type); 
    kSize i; 

    if (!kIsNull(base))
    {
        kTypeClass* baseObj = base;  

        for (i = 0; i < baseObj->interfaceCount; ++i)
        {   
            kSize iMethodCount = baseObj->interfaces[i].iMethodCount; 

            kCheck(kSysMemAlloc(iMethodCount * sizeof(kPointer), &obj->interfaces[i].iTable)); 
            kCheck(kSysMemAlloc(iMethodCount * sizeof(kMethodInfo), &obj->interfaces[i].iMethodInfo)); 

            kMemCopy(obj->interfaces[i].iTable, baseObj->interfaces[i].iTable, iMethodCount * sizeof(kPointer)); 
            kMemCopy(obj->interfaces[i].iMethodInfo, baseObj->interfaces[i].iMethodInfo, iMethodCount * sizeof(kMethodInfo)); 

            obj->interfaces[i].type = baseObj->interfaces[i].type; 
            obj->interfaces[i].iMethodCount = iMethodCount; 

            obj->interfaceCount++; 
        }
    }

    return kOK; 
}

kFx(kStatus) kType_ImplementInterface(kType type, kType interfaceType, const kChar* interfaceName, kSize iTableSize)
{ 
    kTypeClass* obj = type; 
    kTypeClass* iobj = interfaceType; 
    kType interfaceIt = interfaceType; 

    if (kIsNull(interfaceType))
    {
        if (kSuccess(kAssembly_ReorderType(obj->assembly, interfaceName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the interface type with kAddType?
            kAssert(kFALSE); 
            kCheck(kERROR_STATE); 
        }
    }

    kCheckArgs(!kType_IsInterface(type) && kType_IsInterface(interfaceType) && (iTableSize == iobj->vMethodCount * sizeof(kPointer))); 

    while (interfaceIt != kNULL)
    {        
        kTypeClass* itObj = interfaceIt; 

        //add an interface entry, if one does not already exist
        if (!kType_Implements(type, interfaceIt))
        {
            kSize nextIndex = obj->interfaceCount; 

            kCheckState(nextIndex < kTYPE_MAX_INTERFACES); 
           
            kCheck(kSysMemAlloc(itObj->vMethodCount * sizeof(kPointer), &obj->interfaces[nextIndex].iTable)); 
            kCheck(kSysMemAlloc(itObj->vMethodCount * sizeof(kMethodInfo), &obj->interfaces[nextIndex].iMethodInfo)); 

            kCheck(kMemCopy(obj->interfaces[nextIndex].iTable, itObj->vTable, itObj->vMethodCount * sizeof(kPointer))); 
            kCheck(kMemCopy(obj->interfaces[nextIndex].iMethodInfo, itObj->vMethodInfo, itObj->vMethodCount * sizeof(kMethodInfo))); 

            obj->interfaces[nextIndex].type = interfaceIt; 
            obj->interfaces[nextIndex].iMethodCount = itObj->vMethodCount;  
            obj->interfaceCount++; 
        }
                    
        interfaceIt = itObj->bases[0]; 
    }

    return kOK; 
}

kFx(kStatus) kType_AddVersion(kType type, const kChar* format, const kChar* formatVersion, const kChar* guid, kFunction serialize, kFunction deserialize)
{
    kTypeClass* obj = type; 
    kVersion fmtVer; 
    kSize i; 

    kCheck(kVersion_Parse(&fmtVer, formatVersion)); 

    kCheck(kSysMemReallocList(&obj->versionInfo, obj->versionCount, sizeof(kTypeVersionInfo), 
            &obj->versionCapacity, kTYPE_INITIAL_VERSION_CAPACITY, obj->versionCapacity+1)); 
    
    //insert sorted by format version
    for (i = 0; i < obj->versionCount; ++i)
    {
        if (kStrEquals(format, obj->versionInfo[i].format) && (fmtVer < obj->versionInfo[i].formatVersion))
        {
            break; 
        }
    }

    kCheck(kMemMove(&obj->versionInfo[i], &obj->versionInfo[i+1], sizeof(kTypeVersionInfo)*(obj->versionCount - i))); 
    
    kCheck(kStrCopy(obj->versionInfo[i].format, kCountOf(obj->versionInfo[i].format), format)); 
    kCheck(kStrCopy(obj->versionInfo[i].guid, kCountOf(obj->versionInfo[i].guid), guid)); 
    obj->versionInfo[i].formatVersion = fmtVer; 
    obj->versionInfo[i].serialize = serialize; 
    obj->versionInfo[i].deserialize = deserialize; 

    obj->versionCount++; 

    return kOK; 
}

kFx(kStatus) kType_AddFlags(kType type, kTypeFlags flags)
{
    kTypeClass* obj = type; 

    obj->flags |= flags; 

    return kOK; 
}

kFx(kStatus) kType_AddMethod(kType type, kFunction function, const kChar* methodName)
{
    kTypeClass* obj = type; 
    kMethodInfo* info = kNULL;

    kCheck(kSysMemReallocList(&obj->cMethodInfo, obj->cMethodCount, sizeof(kMethodInfo), 
        &obj->cMethodCapacity, kTYPE_INITIAL_CMETHOD_CAPACITY, obj->cMethodCount+1)); 
    
    info = &obj->cMethodInfo[obj->cMethodCount]; 

    kCheck(kStrPrintf(info->functionName, kCountOf(info->functionName), "%s_%s", obj->name, methodName)); 
    kCheck(kStrCopy(info->methodName, kCountOf(info->methodName), methodName)); 
    info->function = function; 

    obj->cMethodCount++; 

    return kOK; 
}

kFx(kStatus) kType_AddVMethod(kType type, kSize index, kFunction function, const kChar* methodName)
{
    kTypeClass* obj = type; 
    kMethodInfo* info = &obj->vMethodInfo[index]; 

    kCheckArgs(index < obj->vMethodCount); 

    obj->vTable[index] = function; 
    
    kCheck(kStrPrintf(info->functionName, kCountOf(info->functionName), "%s_%s", obj->name, methodName)); 
    kCheck(kStrCopy(info->methodName, kCountOf(info->methodName), &methodName[1]));   //removes the "V" prefix
    info->function = function; 

    return kOK; 
}

kFx(kStatus) kType_AddIVMethod(kType type, kType interfaceType, kSize index, kFunction function, const kChar* iMethodName, const kChar* cMethodName)
{
    kTypeClass* obj = type; 
    kSize i; 

    for (i = 0; i < obj->interfaceCount; ++i)
    {
        if (kType_Is(obj->interfaces[i].type, interfaceType))
        {
            kMethodInfo* info = &obj->interfaces[i].iMethodInfo[index]; 

            kCheckArgs(index < obj->interfaces[i].iMethodCount); 

            obj->interfaces[i].iTable[index] = function; 

            kCheck(kStrPrintf(info->functionName, kCountOf(info->functionName), "%s_%s", obj->name, cMethodName));
            kCheck(kStrCopy(info->methodName, kCountOf(info->methodName), iMethodName));
            info->function = function; 

            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kType_AddField(kType type, kType fieldType, const kChar* fieldTypeName, kSize size, kSize offset, kSize count, const kChar* fieldName)
{
    kTypeClass* obj = type;
    kFieldInfo* info = kNULL; 

    if (kIsNull(fieldType))
    {
        if (kSuccess(kAssembly_ReorderType(obj->assembly, fieldTypeName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the field type with kAddType?
            kAssert(kFALSE); 
            kCheck(kERROR_STATE); 
        }
    }

    //This assertion may be raised during type registration if the field type is incorrect. 
    if (kType_Size(fieldType)*count != size)
    {
        kAssert(kType_Size(fieldType)*count == size); 
    }

    kCheck(kSysMemReallocList(&obj->fieldInfo, obj->fieldCount, sizeof(kFieldInfo), 
        &obj->fieldCapacity, kTYPE_INITIAL_FIELD_CAPACITY, obj->fieldCount+1)); 

    info = &obj->fieldInfo[obj->fieldCount]; 

    kCheck(kStrCopy(&info->name[0], kCountOf(info->name), fieldName)); 
    info->type = fieldType; 
    info->offset = offset; 
    info->count = count; 

    obj->fieldCount++; 

    return kOK; 
}

kFx(kStatus) kType_AddEnumerator(kType type, k32s value, const kChar* name)
{
    kTypeClass* obj = type;
    kEnumeratorInfo* info = kNULL; 
    
    kCheck(kSysMemReallocList(&obj->enumeratorInfo, obj->enumeratorCount, sizeof(kEnumeratorInfo), 
        &obj->enumeratorCapacity, kTYPE_INITIAL_ENUMERATOR_CAPACITY, obj->enumeratorCount+1)); 

    info = &obj->enumeratorInfo[obj->enumeratorCount]; 

    kCheck(kStrCopy(&info->name[0], kCountOf(info->name), name)); 
    info->value = value; 

    kCheck(kType_FormatEnumeratorDisplayName(type, info->name, info->displayName, kCountOf(info->displayName))); 

    obj->enumeratorCount++; 

    return kOK; 
}

kFx(kStatus) kType_FormatEnumeratorDisplayName(kType type, const kChar* enumeratorName, kChar* buffer, kSize capacity)
{
    const kChar* typeNameIt = kType_Name(type); 
    const kChar* enumNameIt = enumeratorName; 
    kChar* bufferIt = buffer; 
    kBool firstLetter = kTRUE; 

    //skip type name prefix
    while ((*typeNameIt != 0) && (*enumNameIt != 0))
    {
        if (*enumNameIt == '_')
        {
            enumNameIt++; 
        }
        else if (kStrAsciiToLower(*typeNameIt) == kStrAsciiToLower(*enumNameIt))
        {
            enumNameIt++; 
            typeNameIt++; 
        }
        else
        {
            break;
        }
    }

    //skip leading underscore
    if (*enumNameIt == '_')
    {
        enumNameIt++; 
    }

    //copy out remaining characters
    kCheck(kStrCopy(buffer, capacity, enumNameIt)); 

    //convert from all-caps-with-underscores to first-letter-caps-with-spaces
    while (*bufferIt != 0)
    {
        if (*bufferIt == '_')
        {
            *bufferIt = ' '; 
            firstLetter = kTRUE; 
        }
        else if (!firstLetter)
        {
            *bufferIt = kStrAsciiToLower(*bufferIt); 
        }
        else
        {
            firstLetter = kFALSE; 
        }

        bufferIt++; 
    }

    return kOK; 
}

kFx(kStatus) kType_SetInitPriority(kType type, k32u priority)
{
    kTypeClass* obj = type;

    obj->staticPriority = priority; 

    return kOK; 
}

kFx(k32u) kType_Priority(kType type)
{
    kTypeClass* obj = type;
    return obj->staticPriority; 
}

kFx(kStatus) kType_ZeroStaticData(kType type)
{
    kTypeClass* obj = type;

    if (!kIsNull(obj->staticInit))
    {
        kCheck(kMemSet(obj->staticData, 0, obj->staticSize));
    }

    return kOK;
}

kFx(kStatus) kType_RunStaticInit(kType type)
{
    kTypeClass* obj = type; 
    
    if (!kIsNull(obj->staticInit))
    {
        *obj->staticInitialized = kSuccess(obj->staticInit()); 

        if (!(*obj->staticInitialized))
        {
            kLogf("kType: static initialization failed for type %s.", obj->name); 
        }        
    }

    return kOK; 
}

kFx(kStatus) kType_RunStaticRelease(kType type)
{
    kTypeClass* obj = type; 

    if (kType_StaticInitialized(type))
    {
        *obj->staticInitialized = kFALSE; 

        if (!kSuccess(obj->staticRelease()))
        {
            kLogf("kType: static release failed for type %s.", obj->name); 
        }
    }

    return kOK; 
}

kFx(kBool) kType_StaticInitialized(kType type)
{
    kTypeClass* obj = type; 
    return !kIsNull(obj->staticInitialized) && *obj->staticInitialized; 
}

kFx(const kChar*) kType_Name(kType type)
{
    kTypeClass* obj = kType_Class_(type); 
    
    return obj->name; 
}

kFx(kAssembly) kType_Assembly(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Assembly_(obj); 
}

kFx(kBool) kType_Is(kType type, kType other)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Is_(obj, other); 
}

kFx(kBool) kType_IsValue(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_IsValue_(obj); 
}

kFx(kBool) kType_IsClass(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_IsClass_(obj); 
}

kFx(kBool) kType_IsInterface(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_IsInterface_(obj); 
}

kFx(kBool) kType_IsReference(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_IsReference_(obj); 
}

kFx(kBool) kType_IsAbstract(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_IsAbstract_(obj); 
}

kFx(kBool) kType_Implements(kType type, kType interfaceType)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Implements_(obj, interfaceType); 
}

kFx(kBool) kType_Extends(kType type, kType baseType)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Extends_(obj, baseType); 
}

kFx(kType) kType_Base(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Base_(obj); 
}

kFx(kSize) kType_InterfaceCount(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return obj->interfaceCount; 
}

kFx(kType) kType_InterfaceAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 

    kAssert(index < obj->interfaceCount); 

    return obj->interfaces[index].type; 
}

kFx(kSize) kType_Size(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Size_(obj); 
}

kFx(kSize) kType_InnerSize(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_InnerSize_(obj); 
}

kFx(kSize) kType_StaticSize(kType type)
{
    kTypeClass* obj = kType_Class_(type);

    return obj->staticSize;
}

kFx(kFunction*) kType_VTable(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_VTable_(obj); 
}

kFx(kFunction*) kType_IVTable(kType type, kType interfaceType)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_IVTable_(obj, interfaceType); 
}

kFx(void*) kType_Static(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return kType_Static_(obj); 
}

kFx(kBool) kType_IsEnum(kType type)
{
    kAssertType(type, kType); 

    return kType_IsEnum_(type); 
}

kFx(kBool) kType_IsBitEnum(kType type)
{
    kAssertType(type, kType); 

    return kType_IsBitEnum_(type); 
}

kFx(kBool) kType_IsArrayValue(kType type)
{
    kAssertType(type, kType); 

    return kType_IsArrayValue_(type); 
}

kFx(kSize) kType_MethodCount(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return obj->cMethodCount; 
}

kFx(const kMethodInfo*) kType_MethodInfoAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 

    return &obj->cMethodInfo[index]; 
}

kFx(kStatus) kType_FindMethodInfo(kType type, const kChar* name, const kMethodInfo** info)
{
    kTypeClass* obj = kType_Class_(type); 
    kSize i; 
    
    for (i = 0; i < obj->cMethodCount; ++i)
    {
        if (kStrEquals(obj->cMethodInfo[i].methodName, name))
        {
            *info = &obj->cMethodInfo[i]; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kSize) kType_VMethodCount(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return obj->vMethodCount; 
}

kFx(const kMethodInfo*) kType_VMethodInfoAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 

    return &obj->vMethodInfo[index]; 
}

kFx(kSize) kType_IMethodCount(kType type, kType interfaceType)
{
    kTypeClass* obj = kType_Class_(type); 
    kInterfaceInfo* info = kType_InterfaceInfo_(obj, interfaceType);   

    return kIsNull(info) ? 0 : info->iMethodCount; 
}

kFx(const kMethodInfo*) kType_IMethodInfoAt(kType type, kType interfaceType, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 
    kInterfaceInfo* info = kType_InterfaceInfo_(obj, interfaceType);   

    return kIsNull(info) ? kNULL : &info->iMethodInfo[index]; 
}

kFx(kSize) kType_FieldCount(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return obj->fieldCount; 
}

kFx(const kFieldInfo*) kType_FieldInfoAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 

    return &obj->fieldInfo[index];
}

kFx(kSize) kType_EnumeratorCount(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return obj->enumeratorCount; 
}

kFx(const kEnumeratorInfo*) kType_EnumeratorInfoAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 

    return &obj->enumeratorInfo[index]; 
}

kFx(kStatus) kType_FindEnumeratorInfo(kType type, k32s value, const kEnumeratorInfo** info)
{
    kTypeClass* obj = kType_Class_(type); 
    kSize i; 
    
    for (i = 0; i < obj->enumeratorCount; ++i)
    {
        if (obj->enumeratorInfo[i].value == value)
        {
            *info = &obj->enumeratorInfo[i];  
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kType_FindEnumeratorInfoByName(kType type, const kChar* displayName, const kEnumeratorInfo** info)
{
    kTypeClass* obj = kType_Class_(type); 
    kSize i; 
    
    for (i = 0; i < obj->enumeratorCount; ++i)
    {
        if (kStrEquals(obj->enumeratorInfo[i].displayName, displayName))
        {
            *info = &obj->enumeratorInfo[i];  
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kType_FormatEnumerator(kType type, k32s value, kChar* displayName, kSize capacity)
{  
    const kEnumeratorInfo* info = kNULL; 

    if (!kSuccess(kType_FindEnumeratorInfo(type, value, &info)))
    {
        kCheck(kStrCopy(displayName, capacity, "[Unknown]")); 
        return kERROR_NOT_FOUND; 
    }

    return kStrCopy(displayName, capacity, info->displayName); 
}

kFx(kStatus) kType_ParseEnumerator(kType type, k32s* value, const kChar* displayName)
{  
    const kEnumeratorInfo* info = kNULL; 

    if (!kSuccess(kType_FindEnumeratorInfoByName(type, displayName, &info)))
    {
        *value = 0; 
        return kERROR_NOT_FOUND; 
    }

    *value = info->value; 
    return kOK; 
}

kFx(kSize) kType_VersionCount(kType type)
{
    kTypeClass* obj = kType_Class_(type); 

    return obj->versionCount; 
}

kFx(const kTypeVersionInfo*) kType_VersionInfoAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type);

    return &obj->versionInfo[index];
}

kFx(kTypeVersion) kType_VersionAt(kType type, kSize index)
{
    kTypeClass* obj = kType_Class_(type); 

    return &obj->versionInfo[index]; 
}

kFx(const kChar*) kType_VersionFormat(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->format;
}

kFx(kVersion) kType_VersionFormatVersion(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->formatVersion;
}

kFx(const kChar*) kType_VersionGuid(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->guid;
}

kFx(kFunction) kType_VersionSerializeFx(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->serialize;
}

kFx(kFunction) kType_VersionDeserializeFx(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->deserialize;
}

kFx(kStatus) kType_LayoutStruct(kStructField** fields, kSize fieldCount, kSize* structureSize)
{
    kSize fieldMaxPrimitive = 0; 
    kSize maxPrimitive = 0; 
    kSize totalSize = 0; 
    kSize knownFields = 0; 
    kSize i; 

    //sort fields based on increasing offset; dynamic fields (unknown offset) are sorted last
    qsort(fields, fieldCount, sizeof(kPointer), kType_LayoutStructOffsetComparator); 

    //calculate fields sizes and count the pre-determined fields
    for (i = 0; i < fieldCount; ++i)
    {
        kCheckArgs(!kIsNull(fields[i]->type)); 

        fields[i]->typeSize = kType_Size_(fields[i]->type); 
        fields[i]->fieldSize = fields[i]->typeSize * fields[i]->count; 

        if (fields[i]->offset != kSIZE_NULL)
        {
            knownFields++; 
        }
    }

    //sort dynamic fields such that larger fields come first
    qsort(&fields[knownFields], fieldCount-knownFields, sizeof(kPointer), kType_LayoutStructSizeComparator); 

    //calculate dynamic field offsets and maximum primitive size
    for (i = 0; i < fieldCount; ++i)
    {
        fieldMaxPrimitive = kType_MaxPrimitiveSize(fields[i]->type); 

        if (fields[i]->offset == kSIZE_NULL)
        {
            fields[i]->offset = kCeilUInt_(totalSize, fieldMaxPrimitive) * fieldMaxPrimitive; 
        }

        totalSize = fields[i]->offset + fields[i]->fieldSize; 
        maxPrimitive = kMax_(maxPrimitive, fieldMaxPrimitive); 
    }
    
    //pad structure for maximum primitive alignment
    if (maxPrimitive > 0)
    {
        totalSize = kCeilUInt_(totalSize, maxPrimitive) * maxPrimitive; 
    }
    
    *structureSize = totalSize; 

    return kOK; 
}

k32s kType_LayoutStructOffsetComparator(const void* field1, const void* field2)
{
    const kStructField* f1 = kAs_(field1, kStructField*); 
    const kStructField* f2 = kAs_(field2, kStructField*); 

    if      ((f1->offset == kSIZE_NULL) && (f2->offset != kSIZE_NULL))      return 1;
    else if ((f2->offset == kSIZE_NULL) && (f1->offset != kSIZE_NULL))      return -1; 
    else if ((f1->offset == kSIZE_NULL) && (f2->offset == kSIZE_NULL))      return 0; 
    else                                                                    return (k32s) ((kSSize)f1->offset - (kSSize)f2->offset); 
}

k32s kType_LayoutStructSizeComparator(const void* field1, const void* field2)
{
    const kStructField* f1 = kAs_(field1, kStructField*); 
    const kStructField* f2 = kAs_(field2, kStructField*); 

    return (k32s) ((kSSize)f2->fieldSize - (kSSize)f1->fieldSize); 
}

kFx(kSize) kType_MaxPrimitiveSize(kType type)
{
    kTypeClass* obj = kType_Class_(type); 
    kSize max = 0; 
    kSize count = kType_FieldCount(obj); 
    kSize i; 

    if (count == 0)
    {
        kSize size = kType_Size_(type); 
        
        if ((size == 1) || (size == 2) || (size == 4) || (size == 8))
        {
            return size; 
        }
        else
        {
            //struct likely defined without describing its fields; assume the worst. 
            return (1 << kALIGN_ANY); 
        }
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kSize fieldMax = kType_MaxPrimitiveSize(kType_FieldInfoAt(type, i)->type); 

            max = kMax_(max, fieldMax); 
        }
    }

    return max; 
}

