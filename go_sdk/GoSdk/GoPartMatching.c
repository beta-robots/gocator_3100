/** 
 * @file    GoPartMatching.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoPartMatching.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoPartMatching, kObject)
    kAddVMethod(GoPartMatching, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoPartMatching_Construct(GoPartMatching* detection, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPartMatching), detection)); 

    if (!kSuccess(status = GoPartMatching_Init(*detection, kTypeOf(GoPartMatching), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, detection); 
    }

    return status; 
} 

GoFx(kStatus) GoPartMatching_Init(GoPartMatching detection, kType type, kObject sensor, kAlloc alloc)
{
    GoPartMatchingClass* obj = detection; 
    kStatus exception = kOK;

    kCheck(kObject_Init(detection, type, alloc)); 
    kInitFields_(GoPartMatching, detection);

    obj->sensor = sensor; 

    return kOK; 
}

GoFx(kStatus) GoPartMatching_VRelease(GoPartMatching detection)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(detection); 
    
    return kObject_VRelease(detection); 
}

GoFx(kStatus) GoPartMatching_Read(GoPartMatching detection, kXml xml, kXmlItem item)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(detection);
    kXmlItem enabledItem = kNULL;
    kXmlItem currentTempItem = kNULL;
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_ChildBool(xml, item, "Enabled", &obj->enabled));
    enabledItem = kXml_Child(xml, item, "Enabled");
    kCheck(kXml_AttrBool(xml, enabledItem, "used", &obj->enabledUsed));

    kCheck(kXml_Child32s(xml, item, "MatchAlgo", &obj->algorithm));
    
    //Edge algorithm configuration
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Edge")));
    kCheck(kXml_ChildText(xml, tempItem, "ModelName", obj->edge.modelName, 256));        
    kCheckArgs(!kIsNull(tempItemInner = kXml_Child(xml, tempItem, "Acceptance/Quality")));
    kCheck(kXml_Child64f(xml, tempItemInner, "Min", &obj->edge.qualityDecisionMin));

    //Bounding box algorithm configuration
    kCheckArgs(!kIsNull(currentTempItem = kXml_Child(xml, item, "BoundingBox")));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, currentTempItem, "Acceptance")));
    kCheckArgs(!kIsNull(tempItemInner = kXml_Child(xml, tempItem, "Width")));
    kCheck(kXml_Child64f(xml, tempItemInner, "Max", &obj->boundingBox.widthMax));
    kCheck(kXml_Child64f(xml, tempItemInner, "Min", &obj->boundingBox.widthMin));
    kCheckArgs(!kIsNull(tempItemInner = kXml_Child(xml, tempItem, "Length")));
    kCheck(kXml_Child64f(xml, tempItemInner, "Max", &obj->boundingBox.lengthMax));
    kCheck(kXml_Child64f(xml, tempItemInner, "Min", &obj->boundingBox.lengthMin));

    //Ellipse algorithm configuration
    kCheckArgs(!kIsNull(currentTempItem = kXml_Child(xml, item, "Ellipse")));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, currentTempItem, "Acceptance")));
    kCheckArgs(!kIsNull(tempItemInner = kXml_Child(xml, tempItem, "Major")));
    kCheck(kXml_Child64f(xml, tempItemInner, "Max", &obj->ellipse.majorMax));
    kCheck(kXml_Child64f(xml, tempItemInner, "Min", &obj->ellipse.majorMin));
    kCheckArgs(!kIsNull(tempItemInner = kXml_Child(xml, tempItem, "Minor")));
    kCheck(kXml_Child64f(xml, tempItemInner, "Max", &obj->ellipse.minorMax));
    kCheck(kXml_Child64f(xml, tempItemInner, "Min", &obj->ellipse.minorMin));

    return kOK;
}

GoFx(kStatus) GoPartMatching_Write(GoPartMatching detection, kXml xml, kXmlItem item)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(detection); 
    kXmlItem currentTypeItem = kNULL;
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "Enabled", obj->enabled));
    kCheck(kXml_SetChild32s(xml, item, "MatchAlgo", obj->algorithm));

    kCheck(kXml_AddItem(xml, item, "Edge", &currentTypeItem));
    kCheck(kXml_SetChildText(xml, currentTypeItem, "ModelName", obj->edge.modelName));
    kCheck(kXml_AddItem(xml, currentTypeItem, "Acceptance", &tempItem));
    kCheck(kXml_SetChild64f(xml, tempItem, "Quality/Min", obj->edge.qualityDecisionMin));

    kCheck(kXml_AddItem(xml, item, "BoundingBox", &currentTypeItem));
    kCheck(kXml_SetChild64f(xml, currentTypeItem, "ZAngle", obj->boundingBox.zAngle));
    kCheck(kXml_AddItem(xml, currentTypeItem, "Acceptance", &tempItem));
    kCheck(kXml_AddItem(xml, tempItem, "Width", &tempItemInner));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Min", obj->boundingBox.widthMin));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Max", obj->boundingBox.widthMax));
    kCheck(kXml_AddItem(xml, tempItem, "Length", &tempItemInner));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Min", obj->boundingBox.lengthMin));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Max", obj->boundingBox.lengthMax));

    kCheck(kXml_AddItem(xml, item, "Ellipse", &currentTypeItem));
    kCheck(kXml_SetChild64f(xml, currentTypeItem, "ZAngle", obj->ellipse.zAngle));
    kCheck(kXml_AddItem(xml, currentTypeItem, "Acceptance", &tempItem));
    kCheck(kXml_AddItem(xml, tempItem, "Major", &tempItemInner));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Min", obj->ellipse.majorMin));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Max", obj->ellipse.majorMax));
    kCheck(kXml_AddItem(xml, tempItem, "Minor", &tempItemInner));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Min", obj->ellipse.minorMin));
    kCheck(kXml_SetChild64f(xml, tempItemInner, "Max", obj->ellipse.minorMax));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kBool) GoPartMatching_EnablePartMatchingUsed(GoPartMatching detection)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabledUsed;
}

GoFx(kBool) GoPartMatching_PartMatchingEnabled(GoPartMatching detection)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabled;
}

GoFx(kStatus) GoPartMatching_EnablePartMatching(GoPartMatching detection, kBool enable)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(detection); 

    GoSensor_SyncConfig(obj->sensor);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    obj->enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoPartMatching_SetAlgorithm(GoPartMatching matching, GoPartMatchAlgorithm algorithm)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->algorithm = algorithm;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoPartMatchAlgorithm) GoParthMatching_Algorithm(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->algorithm;
}

GoFx(kStatus) GoPartMatching_SetEdgeModelName(GoPartMatching matching, const kChar* name)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    kCheck(kStrCopy(obj->edge.modelName, 256, name));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(const kChar*) GoPartMatching_EdgeModelName(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edge.modelName;
}

GoFx(kStatus) GoPartMatching_SetEdgeQualityDecisionMin(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, 0.0, 100.0));

    obj->edge.qualityDecisionMin = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_EdgeQualityDecisionMin(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edge.qualityDecisionMin;
}

GoFx(kStatus) GoPartMatching_SetEllipseMajorMax(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->ellipse.majorMax = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}
GoFx(k64f) GoPartMatching_EllipseMajorMax(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ellipse.majorMax;
}

GoFx(kStatus) GoPartMatching_SetEllipseMajorMin(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->ellipse.majorMin = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}
GoFx(k64f) GoPartMatching_EllipseMajorMin(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ellipse.majorMin;
}

GoFx(kStatus) GoPartMatching_SetEllipseMinorMax(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->ellipse.minorMax = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}
GoFx(k64f) GoPartMatching_EllipseMinorMax(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ellipse.minorMax;
}

GoFx(kStatus) GoPartMatching_SetEllipseMinorMin(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->ellipse.minorMin = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}
GoFx(k64f) GoPartMatching_EllipseMinorMin(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ellipse.minorMin;
}

GoFx(kStatus) GoPartMatching_SetEllipseZAngle(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->ellipse.zAngle = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_EllipseZAngle(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ellipse.zAngle;
}


GoFx(kStatus) GoPartMatching_SetBoundingBoxWidthMax(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->boundingBox.widthMax = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_BoundingBoxWidthMax(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->boundingBox.widthMax;
}

GoFx(kStatus) GoPartMatching_SetBoundingBoxWidthMin(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->boundingBox.widthMin = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_BoundingBoxWidthMin(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->boundingBox.widthMin;
}

GoFx(kStatus) GoPartMatching_SetBoundingBoxLengthMax(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->boundingBox.lengthMax = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_BoundingBoxLengthMax(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->boundingBox.lengthMax;
}

GoFx(kStatus) GoPartMatching_SetBoundingBoxLengthMin(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->boundingBox.lengthMin = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_BoundingBoxLengthMin(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->boundingBox.lengthMin;
}

GoFx(kStatus) GoPartMatching_SetBoundingBoxZAngle(GoPartMatching matching, k64f value)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->boundingBox.zAngle = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoPartMatching_BoundingBoxZAngle(GoPartMatching matching)
{
    GoPartMatchingClass* obj = GoPartMatching_Cast_(matching);

    GoSensor_SyncConfig(obj->sensor);

    return obj->boundingBox.zAngle;
}
