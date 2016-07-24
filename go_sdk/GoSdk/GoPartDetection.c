/** 
 * @file    GoPartDetection.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoPartDetection.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoPartDetection, kObject)
    kAddVMethod(GoPartDetection, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoPartDetection_Construct(GoPartDetection* detection, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPartDetection), detection)); 

    if (!kSuccess(status = GoPartDetection_Init(*detection, kTypeOf(GoPartDetection), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, detection); 
    }

    return status; 
} 

GoFx(kStatus) GoPartDetection_Init(GoPartDetection detection, kType type, kObject sensor, kAlloc alloc)
{
    GoPartDetectionClass* obj = detection; 

    kCheck(kObject_Init(detection, type, alloc)); 
    kInitFields_(GoPartDetection, detection);

    obj->sensor = sensor; 

    return kOK; 
}

GoFx(kStatus) GoPartDetection_VRelease(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection); 

    kCheck(kObject_VRelease(detection)); 

    return kOK; 
}

GoFx(kStatus) GoPartDetection_Read(GoPartDetection detection, kXml xml, kXmlItem item)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);
    kXmlItem tempItem = kNULL;
    kXmlItem edgeFilteringItem = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_ChildBool(xml, item, "Enabled", &obj->enabled));
    tempItem = kXml_Child(xml, item, "Enabled");
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->enabledUsed));
    kCheck(kXml_AttrBool(xml, tempItem, "value", &obj->enabledSystemValue));

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Threshold", &obj->threshold));
    kCheck(kXml_Child32s(xml, item, "ThresholdDirection", &obj->thresholdDirection));    

    tempItem = kXml_Child(xml, item, "GapWidth");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->gapWidthUsed));
    }
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "GapWidth", &obj->gapWidth));  

    tempItem = kXml_Child(xml, item, "GapLength");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->gapLengthUsed));
    }
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "GapLength", &obj->gapLength));   

    tempItem = kXml_Child(xml, item, "PaddingWidth");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->paddingWidthUsed));
    }
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "PaddingWidth", &obj->paddingWidth));  

    tempItem = kXml_Child(xml, item, "PaddingLength");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->paddingLengthUsed));
    }
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "PaddingLength", &obj->paddingLength));
    
    if (kXml_ChildExists(xml, item, "MinLength"))
    {
        tempItem = kXml_Child(xml, item, "MinLength");
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->minLengthUsed));
        kCheck(GoConfig_ReadRangeElement64f(xml, item, "MinLength", &obj->minLength));
    }

    tempItem = kXml_Child(xml, item, "MaxLength");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->maxLengthUsed));
    }
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "MaxLength", &obj->maxLength));

    tempItem = kXml_Child(xml, item, "MinArea");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->minAreaUsed));
    }
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "MinArea", &obj->minArea));

    tempItem = kXml_Child(xml, item, "FrameOfReference");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->frameOfReferenceUsed));
    }
    kCheck(kXml_Child32s(xml, item, "FrameOfReference", &obj->frameOfReference));

    edgeFilteringItem = kXml_Child(xml, item, "EdgeFiltering");
    kCheck(edgeFilteringItem != kNULL);
    if (kXml_AttrExists(xml, edgeFilteringItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, edgeFilteringItem, "used", &obj->edgeFiltering.used));
    }
    kCheck(kXml_ChildBool(xml, edgeFilteringItem, "Enabled", &obj->edgeFiltering.enabled));
    kCheck(kXml_ChildBool(xml, edgeFilteringItem, "PreserveInteriorEnabled", &obj->edgeFiltering.preserveInterior));
    kCheck(GoConfig_ReadRangeElement64f(xml, edgeFilteringItem, "ElementLength", &obj->edgeFiltering.elementLength));
    kCheck(GoConfig_ReadRangeElement64f(xml, edgeFilteringItem, "ElementWidth", &obj->edgeFiltering.elementWidth));

    return kOK;
}

GoFx(kStatus) GoPartDetection_Write(GoPartDetection detection, kXml xml, kXmlItem item)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection); 
    kXmlItem edgeFilteringItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "Enabled", obj->enabled));

    kCheck(GoConfig_WriteRangeElement64f(xml, item, "Threshold", obj->threshold));
    kCheck(kXml_SetChild32s(xml, item, "ThresholdDirection", obj->thresholdDirection));    
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "GapWidth", obj->gapWidth));  
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "GapLength", obj->gapLength));   
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "PaddingWidth", obj->paddingWidth));  
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "PaddingLength", obj->paddingLength));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "MinLength", obj->minLength));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "MaxLength", obj->maxLength));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "MinArea", obj->minArea));
    kCheck(kXml_SetChild32s(xml, item, "FrameOfReference", obj->frameOfReference));

    kCheck(kXml_AddItem(xml, item, "EdgeFiltering", &edgeFilteringItem));
    kCheck(kXml_SetChildBool(xml, edgeFilteringItem, "Enabled", obj->edgeFiltering.enabled));
    kCheck(kXml_SetChildBool(xml, edgeFilteringItem, "PreserveInteriorEnabled", obj->edgeFiltering.preserveInterior));
    kCheck(GoConfig_WriteRangeElement64f(xml, edgeFilteringItem, "ElementLength", obj->edgeFiltering.elementLength));
    kCheck(GoConfig_WriteRangeElement64f(xml, edgeFilteringItem, "ElementWidth", obj->edgeFiltering.elementWidth));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kBool) GoPartDetection_EnablePartDetectionUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabledUsed;
}

GoFx(kBool) GoPartDetection_PartDetectionEnabledSystemValue(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabledSystemValue;
}

GoFx(kBool) GoPartDetection_PartDetectionEnabled(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabled;
}


GoFx(kStatus) GoPartDetection_EnablePartDetection(GoPartDetection detection, kBool enable)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    obj->enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_ThresholdLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->threshold.min;
}

GoFx(k64f) GoPartDetection_ThresholdLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->threshold.max;
}

GoFx(kStatus) GoPartDetection_SetThreshold(GoPartDetection detection, k64f height)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(height, obj->threshold.min, obj->threshold.max));

    obj->threshold.value = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_Threshold(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    return obj->threshold.value;
}

GoFx(kStatus) GoPartDetection_SetThresholdDirection(GoPartDetection detection, GoPartHeightThresholdDirection direction)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->thresholdDirection = direction;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoPartHeightThresholdDirection) GoPartDetection_ThresholdDirection(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    return obj->thresholdDirection;
}

GoFx(kStatus) GoPartDetection_SetFrameOfReference(GoPartDetection detection, GoPartFrameOfReference frameOfRef)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->frameOfReference = frameOfRef;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoPartFrameOfReference) GoPartDetection_FrameOfReference(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->frameOfReference;
}

GoFx(kBool) GoPartDetection_FrameOfReferenceUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->frameOfReferenceUsed;
}


GoFx(k64f) GoPartDetection_GapWidthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gapWidth.min; 
}

GoFx(k64f) GoPartDetection_GapWidthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gapWidth.max; 
}

GoFx(kStatus) GoPartDetection_SetGapWidth(GoPartDetection detection, k64f gap)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(gap, obj->gapWidth.min, obj->gapWidth.max));

    obj->gapWidth.value = gap;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_GapWidth(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gapWidth.value; 
}

GoFx(kBool) GoPartDetection_GapWidthUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gapWidthUsed;
}

GoFx(k64f) GoPartDetection_GapLengthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);
    return obj->gapLength.min; 
}

GoFx(k64f) GoPartDetection_GapLengthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gapLength.max; 
}

GoFx(kStatus) GoPartDetection_SetGapLength(GoPartDetection detection, k64f gap)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(gap, obj->gapLength.min, obj->gapLength.max));

    obj->gapLength.value = gap;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_GapLength(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);
    return obj->gapLength.value; 
}

GoFx(kBool) GoPartDetection_GapLengthUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gapLengthUsed;
}

GoFx(k64f) GoPartDetection_PaddingWidthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingWidth.min; 
}

GoFx(k64f) GoPartDetection_PaddingWidthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingWidth.max; 
}

GoFx(kStatus) GoPartDetection_SetPaddingWidth(GoPartDetection detection, k64f padding)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(padding, obj->paddingWidth.min, obj->paddingWidth.max));

    obj->paddingWidth.value = padding;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_PaddingWidth(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingWidth.value; 
}

GoFx(kBool) GoPartDetection_PaddingWidthUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingWidthUsed;
}

GoFx(k64f) GoPartDetection_PaddingLengthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingLength.min; 
}

GoFx(k64f) GoPartDetection_PaddingLengthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingLength.max; 
}

GoFx(kStatus) GoPartDetection_SetPaddingLength(GoPartDetection detection, k64f padding)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(padding, obj->paddingLength.min, obj->paddingLength.max));

    obj->paddingLength.value = padding;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_PaddingLength(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingLength.value; 
}

GoFx(kBool) GoPartDetection_PaddingLengthUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->paddingLengthUsed;
}

GoFx(k64f) GoPartDetection_MinLengthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minLength.min;
}

GoFx(k64f) GoPartDetection_MinLengthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minLength.max;
}

GoFx(kStatus) GoPartDetection_SetMinLength(GoPartDetection detection, k64f length)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->minLength.min, obj->minLength.max));

    obj->minLength.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_MinLength(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minLength.value;
}


GoFx(kBool) GoPartDetection_MinLengthUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minLengthUsed;
}

GoFx(k64f) GoPartDetection_MaxLengthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->maxLength.min;
}

GoFx(k64f) GoPartDetection_MaxLengthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->maxLength.max;
}

GoFx(kStatus) GoPartDetection_SetMaxLength(GoPartDetection detection, k64f length)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->maxLength.min, obj->maxLength.max));

    obj->maxLength.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_MaxLength(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->maxLength.value; 
}


GoFx(kBool) GoPartDetection_MaxLengthUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->maxLengthUsed;
}

GoFx(kStatus) GoPartDetection_SetMinArea(GoPartDetection detection, k64f area)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(area, obj->minArea.min, obj->minArea.max));

    obj->minArea.value = area;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_MinArea(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minArea.value; 
}


GoFx(kBool) GoPartDetection_MinAreaUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minAreaUsed;
}


GoFx(k64f) GoPartDetection_MinAreaLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minArea.min; 
}

GoFx(k64f) GoPartDetection_MinAreaLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minArea.max; 
}

GoFx(kBool) GoPartDetection_EdgeFilterUsed(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.used;
}

GoFx(kBool) GoPartDetection_EdgeFilterEnabled(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.enabled;
}

GoFx(kStatus) GoPartDetection_EnableEdgeFilter(GoPartDetection detection, kBool enable)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    obj->edgeFiltering.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}


GoFx(k64f) GoPartDetection_EdgeFilterWidthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.elementWidth.min;
}

GoFx(k64f) GoPartDetection_EdgeFilterWidthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.elementWidth.max;
}

GoFx(kStatus) GoPartDetection_SetEdgeFilterWidth(GoPartDetection detection, k64f value)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, obj->edgeFiltering.elementWidth.min, obj->edgeFiltering.elementWidth.max));

    obj->edgeFiltering.elementWidth.value = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_EdgeFilterWidth(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.elementWidth.value;
}


GoFx(k64f) GoPartDetection_EdgeFilterLengthLimitMin(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.elementLength.min;
}

GoFx(k64f) GoPartDetection_EdgeFilterLengthLimitMax(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.elementLength.max;
}

GoFx(kStatus) GoPartDetection_SetEdgeFilterLength(GoPartDetection detection, k64f value)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, obj->edgeFiltering.elementLength.min, obj->edgeFiltering.elementLength.max));

    obj->edgeFiltering.elementLength.value = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartDetection_EdgeFilterLength(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.elementLength.value;
}

GoFx(kBool) GoPartDetection_EdgeFilterAnteriorPreservationEnabled(GoPartDetection detection)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeFiltering.preserveInterior;
}


GoFx(kStatus) GoPartDetection_EnableEdgeFilterAnteriorPreservation(GoPartDetection detection, kBool enable)
{
    GoPartDetectionClass* obj = GoPartDetection_Cast_(detection);

    GoSensor_SyncConfig(obj->sensor);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    obj->edgeFiltering.preserveInterior = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}