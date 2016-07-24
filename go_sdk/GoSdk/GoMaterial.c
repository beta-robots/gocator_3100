/** 
 * @file    GoMaterial.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoMaterial.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoMaterial, kObject)
    kAddVMethod(GoMaterial, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoMaterial_Construct(GoMaterial* material, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoMaterial), material)); 

    if (!kSuccess(status = GoMaterial_Init(*material, kTypeOf(GoMaterial), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, material); 
    }

    return status; 
} 

GoFx(kStatus) GoMaterial_Init(GoMaterial material, kType type, kObject sensor, kAlloc alloc)
{
    GoMaterialClass* obj = material; 

    kCheck(kObject_Init(material, type, alloc)); 
    kInitFields_(GoMaterial, material);

    obj->sensor = sensor; 
    
    obj->type = GO_MATERIAL_TYPE_DIFFUSE;
    obj->spotSelectionType = GO_SPOT_SELECTION_TYPE_BEST;

    return kOK; 
}

GoFx(kStatus) GoMaterial_VRelease(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material); 

    return  kObject_VRelease(material);
}

GoFx(kStatus) GoMaterial_Read(GoMaterial material, kXml xml, kXmlItem item)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material); 
    kXmlItem tempItem = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Type")));    
    kCheck(kXml_Item32s(xml, tempItem, &obj->type)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->typeUsed));
    kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->typeSystemValue));
    
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotThreshold")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->spotThreshold.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotThreshold.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotThreshold.systemValue));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotWidthMax")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->spotWidthMax.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotWidthMax.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotWidthMax.systemValue));
    kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->spotWidthMax.min));
    kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->spotWidthMax.max));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotSelectionType")));
    kCheck(kXml_Item32s(xml, tempItem, &obj->spotSelectionType)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotSelectionTypeUsed));
    kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->systemSpotSelectionType));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "CameraGainAnalog")));
    kCheck(kXml_Item64f(xml, tempItem, &obj->cameraGainAnalog.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->cameraGainAnalog.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->cameraGainAnalog.systemValue));
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->cameraGainAnalog.min));
    kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->cameraGainAnalog.max));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "CameraGainDigital")));
    kCheck(kXml_Item64f(xml, tempItem, &obj->cameraGainDigital.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->cameraGainDigital.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->cameraGainDigital.systemValue));
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->cameraGainDigital.min));
    kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->cameraGainDigital.max));
    
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "DynamicSensitivity")));
    kCheck(kXml_Item64f(xml, tempItem, &obj->dynamicSensitivity.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->dynamicSensitivity.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->dynamicSensitivity.systemValue));
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->dynamicSensitivity.min));
    kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->dynamicSensitivity.max));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "DynamicThreshold")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->dynamicThreshold.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->dynamicThreshold.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->dynamicThreshold.systemValue));
    kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->dynamicThreshold.min));
    kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->dynamicThreshold.max));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "GammaType")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->gammaType.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->gammaType.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->gammaType.systemValue));

    return kOK; 
}

GoFx(kStatus) GoMaterial_Write(GoMaterial material, kXml xml, kXmlItem item)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material); 
    kXmlItem tempItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));
    kCheck(kXml_SetChild32u(xml, item, "SpotThreshold", obj->spotThreshold.value));
    kCheck(kXml_SetChild32u(xml, item, "SpotWidthMax", obj->spotWidthMax.value));
    kCheck(kXml_SetChild32s(xml, item, "SpotSelectionType", obj->spotSelectionType));
    kCheck(kXml_SetChild64f(xml, item, "CameraGainAnalog", obj->cameraGainAnalog.value));
    kCheck(kXml_SetChild64f(xml, item, "CameraGainDigital", obj->cameraGainDigital.value));
    kCheck(kXml_SetChild64f(xml, item, "DynamicSensitivity", obj->dynamicSensitivity.value));
    kCheck(kXml_SetChild32u(xml, item, "DynamicThreshold", obj->dynamicThreshold.value));
    kCheck(kXml_SetChild32u(xml, item, "GammaType", obj->gammaType.value));
    
    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kStatus) GoMaterial_SetType(GoMaterial material, GoMaterialType type)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->type = type;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoMaterialType) GoMaterial_Type(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->type;
}

GoFx(GoMaterialType) GoMaterial_TypeSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->typeSystemValue;
}

GoFx(kBool) GoMaterial_IsTypeUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->typeUsed;
}

GoFx(kStatus) GoMaterial_SetSpotThreshold(GoMaterial material, k32u value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotThreshold.min && value <= obj->spotThreshold.max);

    obj->spotThreshold.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoMaterial_SpotThreshold(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.value;
}

GoFx(k32u) GoMaterial_SpotThresholdLimitMin(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.min;
}

GoFx(k32u) GoMaterial_SpotThresholdLimitMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.max;
}

GoFx(kBool) GoMaterial_IsSpotThresholdUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.enabled;
}

GoFx(k32u) GoMaterial_SpotThresholdSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.systemValue;
}

GoFx(kStatus) GoMaterial_SetSpotWidthMax(GoMaterial material, k32u value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotWidthMax.min && value <= obj->spotWidthMax.max);

    obj->spotWidthMax.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoMaterial_SpotWidthMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.value;
}

GoFx(k32u) GoMaterial_SpotWidthMaxLimitMin(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.min;
}

GoFx(k32u) GoMaterial_SpotWidthMaxLimitMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.max;
}

GoFx(kBool) GoMaterial_IsSpotWidthMaxUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.enabled;
}

GoFx(k32u) GoMaterial_SpotWidthMaxSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.systemValue;
}

GoFx(kStatus) GoMaterial_SetSpotSelectionType(GoMaterial material, GoSpotSelectionType type)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->spotSelectionType = type;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionType(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotSelectionType;
}

GoFx(kBool) GoMaterial_IsSpotSelectionTypeUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotSelectionTypeUsed;
}

GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionTypeSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->systemSpotSelectionType;
}


GoFx(kStatus) GoMaterial_SetCameraGainAnalog(GoMaterial material, k64f value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->cameraGainAnalog.min && value <= obj->cameraGainAnalog.max);

    obj->cameraGainAnalog.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoMaterial_CameraGainAnalog(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.value;
}

GoFx(k64f) GoMaterial_CameraGainAnalogLimitMin(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.min;
}

GoFx(k64f) GoMaterial_CameraGainAnalogLimitMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.max;
}

GoFx(kBool) GoMaterial_IsCameraGainAnalogUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.enabled;
}

GoFx(k64f) GoMaterial_CameraGainAnalogSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.systemValue;
}

GoFx(kStatus) GoMaterial_SetCameraGainDigital(GoMaterial material, k64f value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->cameraGainDigital.min && value <= obj->cameraGainDigital.max);

    obj->cameraGainDigital.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoMaterial_CameraGainDigital(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.value;
}

GoFx(k64f) GoMaterial_CameraGainDigitalLimitMin(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.min;
}

GoFx(k64f) GoMaterial_CameraGainDigitalLimitMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.max;
}

GoFx(kBool) GoMaterial_IsCameraGainDigitalUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.enabled;
}

GoFx(k64f) GoMaterial_CameraGainDigitalSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.systemValue;
}

GoFx(kStatus) GoMaterial_SetDynamicSensitivity(GoMaterial material, k64f value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->dynamicSensitivity.min && value <= obj->dynamicSensitivity.max);

    obj->dynamicSensitivity.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoMaterial_DynamicSensitivity(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.value;
}

GoFx(k64f) GoMaterial_DynamicSensitivityLimitMin(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.min;
}

GoFx(k64f) GoMaterial_DynamicSensitivityLimitMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.max;
}

GoFx(kBool) GoMaterial_IsDynamicSensitivityUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.enabled;
}

GoFx(k64f) GoMaterial_DynamicSensitivitySystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.systemValue;
}

GoFx(kStatus) GoMaterial_SetDynamicThreshold(GoMaterial material, k32u value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->dynamicThreshold.min && value <= obj->dynamicThreshold.max);

    obj->dynamicThreshold.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoMaterial_DynamicThreshold(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.value;
}

GoFx(k32u) GoMaterial_DynamicThresholdLimitMin(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.min;
}

GoFx(k32u) GoMaterial_DynamicThresholdLimitMax(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.max;
}

GoFx(kBool) GoMaterial_IsDynamicThresholdUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.enabled;
}

GoFx(k32u) GoMaterial_DynamicThresholdSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.systemValue;
}

GoFx(kStatus) GoMaterial_SetGammaType(GoMaterial material, GoGammaType value)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->gammaType.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoGammaType) GoMaterial_GammaType(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gammaType.value;
}

GoFx(kBool) GoMaterial_IsGammaTypeUsed(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gammaType.enabled;
}

GoFx(GoGammaType) GoMaterial_GammaTypeSystemValue(GoMaterial material)
{
    GoMaterialClass* obj = GoMaterial_Cast_(material);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gammaType.systemValue;
}
