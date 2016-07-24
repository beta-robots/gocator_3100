/** 
 * @file    GoProfileGeneration.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoProfileGeneration.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoProfileGeneration, kObject)
    kAddVMethod(GoProfileGeneration, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoProfileGeneration_Construct(GoProfileGeneration* profile, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileGeneration), profile)); 

    if (!kSuccess(status = GoProfileGeneration_Init(*profile, kTypeOf(GoProfileGeneration), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, profile); 
    }

    return status; 
} 

GoFx(kStatus) GoProfileGeneration_Init(GoProfileGeneration profile, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileGenerationClass* obj = profile; 

    kCheck(kObject_Init(profile, type, alloc)); 
    kInitFields_(GoProfileGeneration, profile);

    obj->sensor = sensor; 

    return kOK; 
}

GoFx(kStatus) GoProfileGeneration_VRelease(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    kCheck(kObject_VRelease(profile)); 

    return kOK; 
}

GoFx(kStatus) GoProfileGeneration_Read(GoProfileGeneration profile, kXml xml, kXmlItem item)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 
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

GoFx(kStatus) GoProfileGeneration_Write(GoProfileGeneration profile, kXml xml, kXmlItem item)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 
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


GoFx(kStatus) GoProfileGeneration_SetGenerationType(GoProfileGeneration profile, GoProfileGenerationType type)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileGenerationType) GoProfileGeneration_GenerationType(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    return obj->type;
}

GoFx(kStatus) GoProfileGenerationFixedLength_SetStartTrigger(GoProfileGeneration profile, GoProfileGenerationStartTrigger trigger)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->fixedLength.startTrigger = trigger;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileGenerationStartTrigger) GoProfileGenerationFixedLength_StartTrigger(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.startTrigger;
}

GoFx(kStatus) GoProfileGenerationFixedLength_SetLength(GoProfileGeneration profile, k64f length)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->fixedLength.length.min, obj->fixedLength.length.max));

    obj->fixedLength.length.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileGenerationFixedLength_Length(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.value;
}

GoFx(k64f) GoProfileGenerationFixedLength_LengthLimitMax(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.max;
}

GoFx(k64f) GoProfileGenerationFixedLength_LengthLimitMin(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.min;
}

GoFx(kStatus) GoProfileGenerationVariableLength_SetMaxLength(GoProfileGeneration profile, k64f length)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->variableLength.maxLength.min, obj->variableLength.maxLength.max));

    obj->variableLength.maxLength.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileGenerationVariableLength_MaxLength(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.value;
}

GoFx(k64f) GoProfileGenerationVariableLength_MaxLengthLimitMax(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.max;
}

GoFx(k64f) GoProfileGenerationVariableLength_MaxLengthLimitMin(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.min;
}

GoFx(kStatus) GoProfileGenerationRotational_SetCircumference(GoProfileGeneration profile, k64f value)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, obj->rotational.circumference.min, obj->rotational.circumference.max));
    obj->rotational.circumference.value = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileGenerationRotational_Circumference(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.value;
}

GoFx(k64f) GoProfileGenerationRotational_CircumferenceLimitMax(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.max;
}

GoFx(k64f) GoProfileGenerationRotational_CircumferenceLimitMin(GoProfileGeneration profile)
{
    GoProfileGenerationClass* obj = GoProfileGeneration_Cast_(profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.min;
}
