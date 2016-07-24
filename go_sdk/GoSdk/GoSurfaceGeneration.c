/** 
 * @file    GoSurfaceGeneration.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSurfaceGeneration.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoSurfaceGeneration, kObject)
    kAddVMethod(GoSurfaceGeneration, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSurfaceGeneration_Construct(GoSurfaceGeneration* surface, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceGeneration), surface)); 

    if (!kSuccess(status = GoSurfaceGeneration_Init(*surface, kTypeOf(GoSurfaceGeneration), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, surface); 
    }

    return status; 
} 

GoFx(kStatus) GoSurfaceGeneration_Init(GoSurfaceGeneration surface, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceGenerationClass* obj = surface; 

    kCheck(kObject_Init(surface, type, alloc)); 
    kInitFields_(GoSurfaceGeneration, surface);

    obj->sensor = sensor; 

    return kOK; 
}

GoFx(kStatus) GoSurfaceGeneration_VRelease(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    kCheck(kObject_VRelease(surface)); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceGeneration_Read(GoSurfaceGeneration surface, kXml xml, kXmlItem item)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 
    kXmlItem tempItem = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;
    
    kCheck(kXml_Child32s(xml, item, "Type", &obj->type));

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "FixedLength")));
    kCheck(kXml_Child32s(xml, tempItem, "StartTrigger", &obj->fixedLength.startTrigger));
    kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "Length", &obj->fixedLength.length));

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "VariableLength")));
    kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "MaxLength", &obj->variableLength.maxLength));

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Rotational")));
    kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "Circumference", &obj->rotational.circumference));

    return kOK; 
}

GoFx(kStatus) GoSurfaceGeneration_Write(GoSurfaceGeneration surface, kXml xml, kXmlItem item)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 
    kXmlItem tempItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));

    kCheck(kXml_AddItem(xml, item, "FixedLength", &tempItem));
    kCheck(kXml_SetChild32s(xml, tempItem, "StartTrigger", obj->fixedLength.startTrigger));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "Length", obj->fixedLength.length));

    kCheck(kXml_AddItem(xml, item, "VariableLength", &tempItem));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "MaxLength", obj->variableLength.maxLength));

    kCheck(kXml_AddItem(xml, item, "Rotational", &tempItem));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "Circumference", obj->rotational.circumference));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}


GoFx(kStatus) GoSurfaceGeneration_SetGenerationType(GoSurfaceGeneration surface, GoSurfaceGenerationType type)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSurfaceGenerationType) GoSurfaceGeneration_GenerationType(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    return obj->type;
}

GoFx(kStatus) GoSurfaceGenerationFixedLength_SetStartTrigger(GoSurfaceGeneration surface, GoSurfaceGenerationStartTrigger trigger)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->fixedLength.startTrigger = trigger;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSurfaceGenerationStartTrigger) GoSurfaceGenerationFixedLength_StartTrigger(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.startTrigger;
}

GoFx(kStatus) GoSurfaceGenerationFixedLength_SetLength(GoSurfaceGeneration surface, k64f length)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->fixedLength.length.min, obj->fixedLength.length.max));

    obj->fixedLength.length.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationFixedLength_Length(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.value;
}

GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMax(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.max;
}

GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMin(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.min;
}

GoFx(kStatus) GoSurfaceGenerationVariableLength_SetMaxLength(GoSurfaceGeneration surface, k64f length)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->variableLength.maxLength.min, obj->variableLength.maxLength.max));

    obj->variableLength.maxLength.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLength(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.value;
}

GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMax(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.max;
}

GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMin(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.min;
}

GoFx(kStatus) GoSurfaceGenerationRotational_SetCircumference(GoSurfaceGeneration surface, k64f value)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, obj->rotational.circumference.min, obj->rotational.circumference.max));
    obj->rotational.circumference.value = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationRotational_Circumference(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.value;
}

GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMax(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.max;
}

GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMin(GoSurfaceGeneration surface)
{
    GoSurfaceGenerationClass* obj = GoSurfaceGeneration_Cast_(surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.min;
}
