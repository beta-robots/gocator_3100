/** 
 * @file    GoTool.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoTool, kObject)
    kAddVMethod(GoTool, kObject, VRelease)
    kAddVMethod(GoTool, GoTool, VInit)
    kAddVMethod(GoTool, GoTool, VRead)
    kAddVMethod(GoTool, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoTool_Construct(GoTool* tool, kType type, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, tool)); 

    if (!kSuccess(status = kCast(GoToolVTable*, kType_VTable_(type))->VInit(*tool, type, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, tool); 
    }

    return status; 
} 

GoFx(kStatus) GoTool_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED; //this function must be overriden by every tool
}

GoFx(kStatus) GoTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    GoToolClass* obj = tool;
    kStatus exception;

    kCheck(kObject_Init(tool, type, alloc)); 
    kInitFields_(GoTool, tool);

    obj->sensor = sensor; 
    obj->typeId = typeId;

    kTry
    {
        kTest(kArrayList_Construct(&obj->nodesToMerge, kTypeOf(kXml), 0, alloc));
        kTest(kArrayList_Construct(&obj->measurements, kTypeOf(GoMeasurement), 0, alloc)); 
    }
    kCatch(&exception)
    {
        GoTool_VRelease(tool);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoTool_VRelease(GoTool tool)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    kCheck(kDestroyRef(&obj->nodesToMerge));
    kCheck(kDisposeRef(&obj->measurements));  

    return kObject_VRelease(tool);
}

GoFx(kStatus) GoTool_VRead(GoTool tool, kXml xml, kXmlItem item)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    kXmlItem toolNameItem = kNULL; 
    kXmlItem measurements = kNULL; 
    kXmlItem measurementItem = kNULL; 
    kType measurementType; 
    GoMeasurement measurement = kNULL; 
    kText64 toolType;
    kSize i;
    kBool insertMeasurement = kNULL;
    kBool isFilterable = kTRUE;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(!kIsNull(toolNameItem = kXml_Child(xml, item, "Name")));
    kCheck(kXml_ItemText(xml, toolNameItem, obj->name, kCountOf(obj->name)));
    kCheck(GoUtils_FormatToolType(tool, toolType, kCountOf(toolType))); 
        
    kCheck(!kIsNull(measurements = kXml_Child(xml, item, "Measurements"))); 

    for( i = 0; i < kXml_ChildCount(xml, measurements); i++)
    {
        isFilterable = kTRUE;
        measurementItem = kXml_ChildAt(xml, measurements, i);
        measurement = kNULL;
        insertMeasurement = kFALSE;

        if (kSuccess(GoMeasurements_ParseType(toolType, kXml_ItemName(xml, measurementItem), &measurementType)))
        {
            if (i >= kArrayList_Count(obj->measurements)
                || kIsNull(measurement = kArrayList_As_(obj->measurements, i, GoMeasurement))
                || !kObject_Is(measurement, measurementType))
            {
                if (measurementType == kTypeOf(GoScriptOutput))
                {
                    isFilterable = kFALSE;
                }

                if (!kIsNull(measurement))
                {
                    kCheck(kArrayList_Remove(obj->measurements, i, kNULL));
                    kDestroyRef(&measurement);
                }

                kCheck(GoMeasurement_Construct(&measurement, measurementType, obj->sensor, tool, isFilterable, kObject_Alloc(tool)));
                insertMeasurement = kTRUE;
            }

            kCheck(GoMeasurement_Read(measurement, xml, measurementItem));

            if (insertMeasurement)
            {
                kCheck(kArrayList_Insert(obj->measurements, i, &measurement));
            }
        }
        else
        {
            kCheck(kArrayList_Add(obj->nodesToMerge, &measurementItem));
        }
    }

    while (kXml_ChildCount(xml, measurements) < kArrayList_Count(obj->measurements))    //arraylist is accessed directly due to refresh behavior when the remove measurement function is used
    {
        kCheck(kArrayList_Remove(obj->measurements, kArrayList_Count(obj->measurements) - 1, &measurement));
        kDestroyRef(&measurement); 
    }

    return kOK; 
}

GoFx(kStatus) GoTool_VWrite(GoTool tool, kXml xml, kXmlItem item)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    kXmlItem temp = kNULL;
    kXmlItem child = kNULL;

    kXmlItem toolItem = kNULL; 
    kXmlItem measurementsItem = kNULL; 
    kXmlItem measurementItem = kNULL; 
    GoMeasurement measurement = kNULL; 
    kText64 measurementName;
    kSize i;
    kXml xmlCopy = kNULL;
                    
    kCheck(kXml_SetChildText(xml, item, "Name", obj->name));

    kCheck(kXml_AddItem(xml, item, "Measurements", &measurementsItem)); 
    
    for(i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        measurement = GoTool_MeasurementAt(tool, i);
        
        kCheck(GoMeasurements_FormatType(measurement, measurementName, kCountOf(measurementName)));
        kCheck(kXml_AddItem(xml, measurementsItem, measurementName, &measurementItem)); 
        kCheck(GoMeasurement_Write(measurement, xml, measurementItem)); 
    }

    //Forwards Compatibility for static tools. Merging of dynamic tools can lead to duplicate and/or overwritten elements
    //maybe remove it altogether...
    if (obj->typeId != GO_TOOL_PROFILE_GROOVE
        && obj->typeId != GO_TOOL_PROFILE_STRIP
        && obj->typeId != GO_TOOL_EXTENSIBLE)
    {
        kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));
    }
    else
    {
        for (i = 0; i < kArrayList_Count(obj->nodesToMerge); i++)
        {
            kXmlItem itemToMerge = *(kXmlItem*)kArrayList_At(obj->nodesToMerge, i);
            kCheck(kXml_CopyItem(xml, item, kNULL, obj->nodesToMerge, itemToMerge, kNULL));
        }
    }    

    return kOK; 
}

GoFx(kStatus) GoTool_Read(GoTool tool, kXml xml, kXmlItem item)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    return kCast(GoToolVTable*, kObject_VTable_(tool))->VRead(tool, xml, item);
}

GoFx(kStatus) GoTool_Write(GoTool tool, kXml xml, kXmlItem item)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    return kCast(GoToolVTable*, kObject_VTable_(tool))->VWrite(tool, xml, item);
}

GoFx(kSize) GoTool_MeasurementCount(GoTool tool)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->measurements);
}

GoFx(GoMeasurement) GoTool_MeasurementAt(GoTool tool, kSize index)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->measurements));

    return kArrayList_As_(obj->measurements, index, GoMeasurement);
}

GoFx(kStatus) GoTool_AddMeasurement(GoTool tool, kType type, kBool isFilterable, GoMeasurement* measurement)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    GoMeasurement output = kNULL;
    kStatus exception;

    //private method, no need to check configurable or set modified
    kTry
    {
        kTest(GoMeasurement_Construct(&output, type, obj->sensor, tool, isFilterable, kObject_Alloc(tool)));
        kTest(kArrayList_Add(obj->measurements, &output));

        if(!kIsNull(measurement))
        {
            *measurement = output;
        }
    }
    kCatch(&exception)
    {
        kDestroyRef(&output);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTool_RemoveMeasurement(GoTool tool, kSize index)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    GoMeasurement measurement = kNULL;

    //private method, no need to check configurable or set modified
    kCheck(kArrayList_Remove(obj->measurements, index, &measurement)); 
    kCheck(kDestroyRef(&measurement)); 

    return kOK;
}

GoFx(kStatus) GoTool_ClearMeasurements(GoTool tool)
{
    GoToolClass* obj = GoTool_Cast_(tool); 
    GoMeasurement measurement = kNULL;

    //private method, no need to check configurable or set modified
    kCheck(kArrayList_Purge(obj->measurements)); 

    return kOK;
}

GoFx(kStatus) GoTool_SetName(GoTool tool, const kChar* name)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kStrCopy(obj->name, kCountOf(obj->name), name);
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTool_Name(GoTool tool, kChar* name, kSize capacity)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    GoSensor_SyncConfig(obj->sensor);
    kStrCopy(name, capacity, obj->name);

    return kOK; 
}

GoFx(kObject) GoTool_Sensor(GoTool tool)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    return obj->sensor;
}

GoFx(GoMeasurement) GoTool_FindMeasurementByType(GoTool tool, GoMeasurementType type)
{
    kSize i;
    GoMeasurement measurement = kNULL;
    kBool found = kFALSE;

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        measurement = GoTool_MeasurementAt(tool, i);

        if (GoMeasurement_Type(measurement) == type)
        {
            found = kTRUE;
            break;
        }            
    }

    if (!found)
    {
        return kNULL;
    }

    return measurement;
}

GoFx(GoToolType) GoTool_Type(GoTool tool)
{
    GoToolClass* obj = GoTool_Cast_(tool); 

    return obj->typeId;
}

GoFx(kStatus) GoTool_SetId(GoTool tool, kSize id)
{
    GoToolClass* obj = GoTool_Cast_(tool);

    obj->id = id;

    return kOK;
}