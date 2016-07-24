/** 
 * @file    GoMeasurement.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoMeasurement, kObject)
    kAddVMethod(GoMeasurement, kObject, VRelease)
    kAddVMethod(GoMeasurement, GoMeasurement, VInit)
    kAddVMethod(GoMeasurement, GoMeasurement, VRead)
    kAddVMethod(GoMeasurement, GoMeasurement, VWrite)
kEndClass()

GoFx(kStatus) GoMeasurement_Construct(GoMeasurement* measurement, 
                                    kType type, 
                                    kObject sensor, 
                                    kObject srcTool, 
                                    kBool isFilterable, 
                                    kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, measurement)); 

    if (!kSuccess(status = kCast(GoMeasurementVTable*, kType_VTable_(type))->VInit(*measurement, type, sensor, srcTool, isFilterable, alloc)))
    {
        kAlloc_FreeRef(alloc, measurement); 
    }

    return status; 
} 

GoFx(kStatus) GoMeasurement_VInit(GoMeasurement measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED;  //this function must be overriden by every measurement
}

GoFx(kStatus) GoMeasurement_Init(GoMeasurement measurement, kType type, GoMeasurementType typeId, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    GoMeasurementClass* obj = measurement; 
    
    kCheck(kObject_Init_(measurement, type, alloc)); 
    kInitFields_(GoMeasurement, measurement);

    obj->isFilterable = isFilterable;
    obj->sensor = sensor;
    obj->id = GO_MEASUREMENT_UNASSIGNED_ID;
    obj->srcTool = srcTool;
    obj->enabled = kFALSE;
    obj->scale = 1.0;
    obj->smoothingWindow = 1;
    obj->typeId = typeId;

    return kOK; 
}

GoFx(kStatus) GoMeasurement_VRelease(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 
            
    kCheck(kObject_VRelease(measurement)); 

    return kOK; 
}

GoFx(kStatus) GoMeasurement_VRead(GoMeasurement measurement, kXml xml, kXmlItem item)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    obj->xml = xml;
    obj->xmlItem = item;

    if(!kSuccess(kXml_Attr32s(xml, item, "id", &obj->id)))
    {
        obj->id = GO_MEASUREMENT_UNASSIGNED_ID;
    }

    kCheck(kXml_ChildText(xml, item, "Name", obj->name, kCountOf(obj->name)));    

    if (obj->isFilterable)
    {
        kCheck(kXml_ChildBool(xml, item, "Enabled", &obj->enabled));
        kCheck(kXml_Child64f(xml, item, "DecisionMin", &obj->decisionMin));
        kCheck(kXml_Child64f(xml, item, "DecisionMax", &obj->decisionMax));       

        kCheck(kXml_ChildBool(xml, item, "HoldEnabled", &obj->holdEnabled));
        kCheck(kXml_ChildBool(xml, item, "SmoothingEnabled", &obj->smoothingEnabled));
        kCheck(kXml_Child64s(xml, item, "SmoothingWindow", &obj->smoothingWindow));
        kCheck(kXml_Child64f(xml, item, "Scale", &obj->scale));
        kCheck(kXml_Child64f(xml, item, "Offset", &obj->offset));
    }

    return kOK; 
}

GoFx(kStatus) GoMeasurement_VWrite(GoMeasurement measurement, kXml xml, kXmlItem item)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 
    kXmlItem temp = kNULL;
    kXmlItem child = kNULL;

    kCheck(kXml_SetAttr32s(xml, item, "id", obj->id));
    kCheck(kXml_SetChildText(xml, item, "Name", obj->name));

    if (obj->isFilterable)
    {
        kCheck(kXml_SetChildBool(xml, item, "Enabled", obj->enabled));
        kCheck(kXml_SetChild64f(xml, item, "DecisionMin", obj->decisionMin));
        kCheck(kXml_SetChild64f(xml, item, "DecisionMax", obj->decisionMax));    

        kCheck(kXml_SetChildBool(xml, item, "HoldEnabled", obj->holdEnabled));
        kCheck(kXml_SetChildBool(xml, item, "SmoothingEnabled", obj->smoothingEnabled));
        kCheck(kXml_SetChild64s(xml, item, "SmoothingWindow", obj->smoothingWindow));
        kCheck(kXml_SetChild64f(xml, item, "Scale", obj->scale));
        kCheck(kXml_SetChild64f(xml, item, "Offset", obj->offset));
    }    

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}


GoFx(kStatus) GoMeasurement_Read(GoMeasurement measurement, kXml xml, kXmlItem item)
{
    return kCast(GoMeasurementVTable*, kObject_VTable_(measurement))->VRead(measurement, xml, item);
}

GoFx(kStatus) GoMeasurement_Write(GoMeasurement measurement, kXml xml, kXmlItem item)
{
    return kCast(GoMeasurementVTable*, kObject_VTable_(measurement))->VWrite(measurement, xml, item);
}

GoFx(kBool) GoMeasurement_HasId(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(obj->sensor);

    return (obj->id != GO_MEASUREMENT_UNASSIGNED_ID);
}

GoFx(kStatus) GoMeasurement_ClearId(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->id = GO_MEASUREMENT_UNASSIGNED_ID;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoMeasurement_SetId(GoMeasurement measurement, k32u id)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    obj->id = (k32s) id;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoMeasurement_Id(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->id;
}

GoFx(const kChar*) GoMeasurement_Name(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement);

    GoSensor_SyncConfig(obj->sensor);

    return obj->name;
}

GoFx(kStatus) GoMeasurement_SetName(GoMeasurement measurement, const kChar* name)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    kCheck(kStrCopy(obj->name, kCountOf(obj->name), name));
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(kObject) GoMeasurement_Sensor(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement);

    return obj->sensor;
}

GoFx(kObject) GoMeasurement_SourceTool(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement);

    return obj->srcTool;
}

GoFx(kStatus) GoMeasurement_Enable(GoMeasurement measurement, kBool enable)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));

    obj->enabled = enable;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(kBool) GoMeasurement_Enabled(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->enabled;
}

GoFx(kStatus) GoMeasurement_SetDecisionMin(GoMeasurement measurement, k64f min)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->decisionMin = min;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k64f) GoMeasurement_DecisionMin(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 
    
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->decisionMin;
}

GoFx(kStatus) GoMeasurement_SetDecisionMax(GoMeasurement measurement, k64f max)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->decisionMax = max;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k64f) GoMeasurement_DecisionMax(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->decisionMax;
}

GoFx(kStatus) GoMeasurement_EnableHold(GoMeasurement measurement, kBool enable)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->holdEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(kBool) GoMeasurement_HoldEnabled(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->holdEnabled;
}

GoFx(kStatus) GoMeasurement_EnableSmoothing(GoMeasurement measurement, kBool enable)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->smoothingEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(kBool) GoMeasurement_SmoothingEnabled(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->smoothingEnabled;
}

GoFx(kStatus) GoMeasurement_SetSmoothingWindow(GoMeasurement measurement, k64s value)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->smoothingWindow = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k64s) GoMeasurement_SmoothingWindow(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->smoothingWindow;
}

GoFx(kStatus) GoMeasurement_SetScale(GoMeasurement measurement, k64f value)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->scale = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k64f) GoMeasurement_Scale(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->scale;
}

GoFx(kStatus) GoMeasurement_SetOffset(GoMeasurement measurement, k64f value)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    if (!obj->isFilterable)
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->offset = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k64f) GoMeasurement_Offset(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->offset;
}

GoFx(GoMeasurementType) GoMeasurement_Type(GoMeasurement measurement)
{
    GoMeasurementClass* obj = GoMeasurement_Cast_(measurement); 

    return obj->typeId;
}
