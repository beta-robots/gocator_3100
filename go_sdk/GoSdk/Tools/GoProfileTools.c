/** 
 * @file    GoProfileTools.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoProfileTools.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoProfileTool, GoTool)
kAddVMethod(GoProfileTool, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoProfileTool_Init(GoProfileTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    GoProfileToolClass* obj = tool;

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc)); 
    kInitFields_(GoProfileTool, tool);

    obj->source = GO_DATA_SOURCE_TOP; 
    obj->xAnchor = -1;
    obj->zAnchor = -1;

    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc)); 

    kCheck(kArrayList_Construct(&obj->xAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAnchorOptions, kTypeOf(k32u), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileTool_VRelease(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    kCheck(kDisposeRef(&obj->sourceOptions)); 
    kCheck(kDisposeRef(&obj->xAnchorOptions));
    kCheck(kDisposeRef(&obj->zAnchorOptions));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoProfileTool_Read(GoProfileTool tool, kXml xml, kXmlItem item)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);
    kText128 text; 
    kXmlItem anchorItem = kNULL;
    kXmlItem tempItem = kNULL;

    kCheck(GoTool_VRead(tool, xml, item));

    kCheck(kXml_Child32s(xml, item, "Source", &obj->source));
    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Source")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->sourceOptions));  

    kCheckArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
    kCheck(kXml_Child32s(xml, anchorItem, "X", &obj->xAnchor));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "X")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->xAnchorOptions));  

    kCheckArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
    kCheck(kXml_Child32s(xml, anchorItem, "Z", &obj->zAnchor));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "Z")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->zAnchorOptions));  

    return kOK; 
}

GoFx(kStatus) GoProfileTool_Write(GoProfileTool tool, kXml xml, kXmlItem item)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);
    kXmlItem anchorItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));

    kCheck(kXml_AddItem(xml, item, "Anchor", &anchorItem));
    kCheck(kXml_SetChild32s(xml, anchorItem, "X", obj->xAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Z", obj->zAnchor));

    kCheck(GoTool_VWrite(tool, xml, item));

    return kOK; 
}

GoFx(kSize) GoProfileTool_SourceOptionCount(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(GoDataSource) GoProfileTool_SourceOptionAt(GoProfileTool tool, kSize index)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));
    
    return kArrayList_As_(obj->sourceOptions, index, GoDataSource);
}

GoFx(kStatus) GoProfileTool_SetSource(GoProfileTool tool, GoDataSource source)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->sourceOptions), kArrayList_Count(obj->sourceOptions), source));
    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoDataSource) GoProfileTool_Source(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

GoFx(kSize) GoProfileTool_XAnchorOptionCount(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->xAnchorOptions);
}

GoFx(k32u) GoProfileTool_XAnchorOptionAt(GoProfileTool tool, kSize index)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->xAnchorOptions));
    
    return kArrayList_As_(obj->xAnchorOptions, index, k32u);
}

GoFx(k32s) GoProfileTool_XAnchor(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->xAnchor;
}

GoFx(kStatus) GoProfileTool_SetXAnchor(GoProfileTool tool, k32s id)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_Data(obj->xAnchorOptions), kArrayList_Count(obj->xAnchorOptions), id));
    }

    obj->xAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileTool_XAnchorEnabled(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->xAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoProfileTool_ZAnchorOptionCount(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->zAnchorOptions);
}

GoFx(k32u) GoProfileTool_ZAnchorOptionAt(GoProfileTool tool, kSize index)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->zAnchorOptions));
    
    return kArrayList_As_(obj->zAnchorOptions, index, k32u);
}

GoFx(kStatus) GoProfileTool_SetZAnchor(GoProfileTool tool, k32s id)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_Data(obj->zAnchorOptions), kArrayList_Count(obj->zAnchorOptions), id));
    }

    obj->zAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}


GoFx(kBool) GoProfileTool_ZAnchorEnabled(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->zAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoProfileTool_ZAnchor(GoProfileTool tool)
{
    GoProfileToolClass* obj = GoProfileTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->zAnchor;
}

kBeginClass(Go, GoProfileArea, GoProfileTool)
kAddVMethod(GoProfileArea, kObject, VRelease)
kAddVMethod(GoProfileArea, GoTool, VInit)
kAddVMethod(GoProfileArea, GoTool, VRead)
kAddVMethod(GoProfileArea, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileArea_Construct(GoProfileArea* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileArea), sensor, allocator);
}

GoFx(kStatus) GoProfileArea_VInit(GoProfileArea tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileAreaClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_AREA, sensor, alloc)); 
    kInitFields_(GoProfileArea, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileAreaArea), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileAreaCentroidX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileAreaCentroidZ), kTRUE, kNULL));

    obj->type = GO_PROFILE_AREA_TYPE_OBJECT;
    obj->baseline = GO_PROFILE_BASELINE_TYPE_X_AXIS;

    kCheck(GoProfileLineRegion_Construct(&obj->line, sensor, alloc)); 
    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileArea_VRelease(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool);

    kCheck(kDestroyRef(&obj->line)); 
    kCheck(kDestroyRef(&obj->region)); 

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileArea_VRead(GoProfileArea tool, kXml xml, kXmlItem item)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool);
    kXmlItem lineItem, regionItem;
    kXmlItem tempItem = kNULL;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    tempItem = kXml_Child(xml, item, "Type");
    kCheck(kXml_Item32s(xml, tempItem, &obj->type));
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->typeUsed));

    tempItem = kXml_Child(xml, item, "Baseline");
    kCheck(kXml_Item32s(xml, tempItem, &obj->baseline));
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->baselineUsed));

    kCheck(!kIsNull(lineItem = kXml_Child(xml, item, "Line"))); 
    kCheck(GoProfileLineRegion_Read(obj->line, xml, lineItem));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileArea_VWrite(GoProfileArea tool, kXml xml, kXmlItem item)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 
    kXmlItem lineItem, regionItem;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));
    kCheck(kXml_SetChild32s(xml, item, "Baseline", obj->baseline));

    kCheck(kXml_AddItem(xml, item, "Line", &lineItem)); 
    kCheck(GoProfileLineRegion_Write(obj->line, xml, lineItem));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileBaseline) GoProfileArea_Baseline(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 
    return obj->baseline;
}

GoFx(kStatus) GoProfileArea_SetBaseline(GoProfileArea tool, GoProfileBaseline type)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->baseline = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileArea_BaselineUsed(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 

    return obj->baselineUsed;
}

GoFx(GoProfileLineRegion) GoProfileArea_LineRegion(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 
    return obj->line;
}

GoFx(GoProfileAreaType) GoProfileArea_Type(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 
    return obj->type;
}

GoFx(kStatus) GoProfileArea_SetType(GoProfileArea tool, GoProfileAreaType type)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileArea_TypeUsed(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 

    return obj->typeUsed;
}

GoFx(GoProfileRegion) GoProfileArea_Region(GoProfileArea tool)
{
    GoProfileAreaClass* obj = GoProfileArea_Cast_(tool); 
    return obj->region;
}

GoFx(GoProfileAreaArea) GoProfileArea_AreaMeasurement(GoProfileArea tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_AREA_AREA);
}

GoFx(GoProfileAreaCentroidX) GoProfileArea_CentroidXMeasurement(GoProfileArea tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_AREA_CENTROID_X);
}

GoFx(GoProfileAreaCentroidX) GoProfileArea_CentroidZMeasurement(GoProfileArea tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z);
}


kBeginClass(Go, GoProfileBox, GoProfileTool)
kAddVMethod(GoProfileBox, kObject, VRelease)
kAddVMethod(GoProfileBox, GoTool, VInit)
kAddVMethod(GoProfileBox, GoTool, VRead)
kAddVMethod(GoProfileBox, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileBox_Construct(GoProfileBox* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileBox), sensor, allocator);
}

GoFx(kStatus) GoProfileBox_VInit(GoProfileBox tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileBoxClass* obj = tool;

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_BOUNDING_BOX, sensor, alloc)); 
    kInitFields_(GoProfileBox, tool);

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc));

    obj->regionEnabled = kTRUE;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxGlobalX), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoProfileBox_VRelease(GoProfileBox tool)
{
    GoProfileBoxClass* obj = GoProfileBox_Cast_(tool);

    kDestroyRef(&obj->region);

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileBox_VRead(GoProfileBox tool, kXml xml, kXmlItem item)
{
    GoProfileBoxClass* obj = GoProfileBox_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoProfileBox_VWrite(GoProfileBox tool, kXml xml, kXmlItem item)
{
    GoProfileBoxClass* obj = GoProfileBox_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kBool) GoProfileBox_RegionEnabled(GoProfileBox tool)
{
    GoProfileBoxClass* obj = GoProfileBox_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileBox_EnableRegion(GoProfileBox tool, kBool enable)
{
    GoProfileBoxClass* obj = GoProfileBox_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileBox_Region(GoProfileBox tool)
{
    GoProfileBoxClass* obj = GoProfileBox_Cast_(tool);

    return obj->region;
}

GoFx(GoProfileBoxX) GoProfileBox_XMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X);
}

GoFx(GoProfileBoxZ) GoProfileBox_ZMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z);
}

GoFx(GoProfileBoxWidth) GoProfileBox_WidthMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH);
}

GoFx(GoProfileBoxHeight) GoProfileBox_HeightMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT);
}

GoFx(GoProfileBoxGlobalX) GoProfileBox_GlobalXMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X);
}

kBeginClass(Go, GoProfileCircle, GoProfileTool)
kAddVMethod(GoProfileCircle, kObject, VRelease)
kAddVMethod(GoProfileCircle, GoTool, VInit)
kAddVMethod(GoProfileCircle, GoTool, VRead)
kAddVMethod(GoProfileCircle, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileCircle_Construct(GoProfileCircle* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileCircle), sensor, allocator);
}

GoFx(kStatus) GoProfileCircle_VInit(GoProfileCircle tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileCircleClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_CIRCLE, sensor, alloc)); 
    kInitFields_(GoProfileCircle, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleX), kTRUE, kNULL));        
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleZ), kTRUE, kNULL));            
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleRadius), kTRUE, kNULL));        

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileCircle_VRelease(GoProfileCircle tool)
{
    GoProfileCircleClass* obj = GoProfileCircle_Cast_(tool); 

    kCheck(kDestroyRef(&obj->region));  


    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileCircle_VRead(GoProfileCircle tool, kXml xml, kXmlItem item)
{
    GoProfileCircleClass* obj = GoProfileCircle_Cast_(tool); 
    kXmlItem regionItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileCircle_VWrite(GoProfileCircle tool, kXml xml, kXmlItem item)
{
    GoProfileCircleClass* obj = GoProfileCircle_Cast_(tool); 
    kXmlItem regionItem;

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileRegion) GoProfileCircle_Region(GoProfileCircle tool)
{
    GoProfileCircleClass* obj = GoProfileCircle_Cast_(tool); 
    return obj->region;
}

GoFx(GoProfileCircleX) GoProfileCircle_XMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_X);
}

GoFx(GoProfileCircleZ) GoProfileCircle_ZMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_Z);
}

GoFx(GoProfileCircleRadius) GoProfileCircle_RadiusMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS);
}


kBeginClass(Go, GoProfileDim, GoProfileTool)
kAddVMethod(GoProfileDim, kObject, VRelease)
kAddVMethod(GoProfileDim, GoTool, VInit)
kAddVMethod(GoProfileDim, GoTool, VRead)
kAddVMethod(GoProfileDim, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileDim_Construct(GoProfileDim* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileDim), sensor, allocator);
}

GoFx(kStatus) GoProfileDim_VInit(GoProfileDim tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileDimClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_DIMENSION, sensor, alloc)); 
    kInitFields_(GoProfileDim, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimDistance), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimCenterX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimCenterZ), kTRUE, kNULL));

    kCheck(GoProfileFeature_Construct(&obj->refFeature, sensor, alloc));
    kCheck(GoProfileFeature_Construct(&obj->feature, sensor, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileDim_VRelease(GoProfileDim tool)
{
    GoProfileDimClass* obj = GoProfileDim_Cast_(tool);

    kCheck(kDestroyRef(&obj->refFeature));
    kCheck(kDestroyRef(&obj->feature));

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileDim_VRead(GoProfileDim tool, kXml xml, kXmlItem item)
{
    GoProfileDimClass* obj = GoProfileDim_Cast_(tool);
    kXmlItem refFeatureItem;
    kXmlItem featureItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(!kIsNull(refFeatureItem  = kXml_Child(xml, item, "RefFeature"))); 
    kCheck(GoProfileFeature_Read(obj->refFeature, xml, refFeatureItem));
    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature"))); 
    kCheck(GoProfileFeature_Read(obj->feature, xml, featureItem));

    return kOK;
}

GoFx(kStatus) GoProfileDim_VWrite(GoProfileDim tool, kXml xml, kXmlItem item)
{
    GoProfileDimClass* obj = GoProfileDim_Cast_(tool);
    kXmlItem refFeatureItem;
    kXmlItem featureItem;

    kCheck(kXml_AddItem(xml, item, "RefFeature", &refFeatureItem)); 
    kCheck(GoProfileFeature_Write(obj->refFeature, xml, refFeatureItem));
    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem)); 
    kCheck(GoProfileFeature_Write(obj->feature, xml, featureItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileFeature) GoProfileDim_RefFeature(GoProfileDim tool)
{
    GoProfileDimClass* obj = GoProfileDim_Cast_(tool);
    return obj->refFeature;
}

GoFx(GoProfileFeature) GoProfileDim_Feature(GoProfileDim tool)
{
    GoProfileDimClass* obj = GoProfileDim_Cast_(tool);
    return obj->feature;
}

GoFx(GoMeasurement) GoProfileDim_WidthMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH);
}

GoFx(GoMeasurement) GoProfileDim_HeightMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT);
}

GoFx(GoMeasurement) GoProfileDim_DistanceMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE);
}

GoFx(GoMeasurement) GoProfileDim_CenterXMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X);
}

GoFx(GoMeasurement) GoProfileDim_CenterZMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z);
}


kBeginClass(Go, GoProfileGroove, GoProfileTool)
kAddVMethod(GoProfileGroove, kObject, VRelease)
kAddVMethod(GoProfileGroove, GoTool, VInit)
kAddVMethod(GoProfileGroove, GoTool, VRead)
kAddVMethod(GoProfileGroove, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileGroove_Construct(GoProfileGroove* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileGroove), sensor, allocator);
}

GoFx(kStatus) GoProfileGroove_VInit(GoProfileGroove tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileGrooveClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_GROOVE, sensor, alloc)); 
    kInitFields_(GoProfileGroove, tool);

    obj->shape = GO_PROFILE_GROOVE_SHAPE_U;
    obj->minWidth = 0;
    obj->maxWidth = 0;
    obj->minDepth = 0;

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileGroove_VRelease(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = tool;

    kCheck(kDestroyRef(&obj->region)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileGroove_VRead(GoProfileGroove tool, kXml xml, kXmlItem item)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);
    kXmlItem regionItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(kXml_Child32s(xml, item, "Shape", &obj->shape));    
    kCheck(kXml_Child64f(xml, item, "MinWidth", &obj->minWidth));
    kCheck(kXml_Child64f(xml, item, "MaxWidth", &obj->maxWidth));
    kCheck(kXml_Child64f(xml, item, "MinDepth", &obj->minDepth));    

    return kOK;
}

GoFx(kStatus) GoProfileGroove_VWrite(GoProfileGroove tool, kXml xml, kXmlItem item)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);
    kXmlItem regionItem;

    kCheck(GoProfileTool_Write(tool, xml, item)); 
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));    

    kCheck(kXml_SetChild32s(xml, item, "Shape", obj->shape));    
    kCheck(kXml_SetChild64f(xml, item, "MinWidth", obj->minWidth));
    kCheck(kXml_SetChild64f(xml, item, "MaxWidth", obj->maxWidth));       
    kCheck(kXml_SetChild64f(xml, item, "MinDepth", obj->minDepth));       

    return kOK;
}

GoFx(GoProfileGrooveShape) GoProfileGroove_Shape(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);
    return obj->shape;
}

GoFx(kStatus) GoProfileGroove_SetShape(GoProfileGroove tool, GoProfileGrooveShape shape)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->shape = shape;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileGroove_MinDepth(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = tool;
    return obj->maxWidth;
}

GoFx(kStatus) GoProfileGroove_SetMinDepth(GoProfileGroove tool, k64f width)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileGroove_MaxWidth(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = tool;
    return obj->maxWidth;
}

GoFx(kStatus) GoProfileGroove_SetMaxWidth(GoProfileGroove tool, k64f width)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileGroove_MinWidth(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = tool;
    return obj->minWidth;
}

GoFx(kStatus) GoProfileGroove_SetMinWidth(GoProfileGroove tool, k64f width)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->minWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileGroove_Region(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = tool;
    return obj->region;
}

GoFx(kStatus) GoProfileGroove_AddMeasurement(GoProfileGroove tool, GoMeasurementType type, GoMeasurement* measurement)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);
    kType classType;

    if (type != GO_MEASUREMENT_PROFILE_GROOVE_X
        && type != GO_MEASUREMENT_PROFILE_GROOVE_Z
        && type != GO_MEASUREMENT_PROFILE_GROOVE_WIDTH
        && type != GO_MEASUREMENT_PROFILE_GROOVE_DEPTH)
    {
        return kERROR_PARAMETER;
    }
    else
    {
        switch(type)
        {
        case GO_MEASUREMENT_PROFILE_GROOVE_X: classType = kTypeOf(GoProfileGrooveX); break;
        case GO_MEASUREMENT_PROFILE_GROOVE_Z: classType = kTypeOf(GoProfileGrooveZ); break;
        case GO_MEASUREMENT_PROFILE_GROOVE_WIDTH: classType = kTypeOf(GoProfileGrooveWidth); break;
        case GO_MEASUREMENT_PROFILE_GROOVE_DEPTH: classType = kTypeOf(GoProfileGrooveDepth); break;
        }
    }

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoTool_AddMeasurement(tool, classType, kTRUE, measurement));
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoProfileGroove_RemoveMeasurement(GoProfileGroove tool, kSize index)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    if (index > GoTool_MeasurementCount(tool))
    {
        return kERROR_PARAMETER;
    }

    kCheck(GoTool_RemoveMeasurement(tool, index));

    return kOK;
}

GoFx(kSize) GoProfileGroove_MeasurementCount(GoProfileGroove tool)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    return GoTool_MeasurementCount(tool);
}

GoFx(GoMeasurement) GoProfileGroove_MeasurementAt(GoProfileGroove tool, kSize index)
{
    GoProfileGrooveClass* obj = GoProfileGroove_Cast_(tool);

    kAssert(index < GoTool_MeasurementCount(tool));
    
    return GoTool_MeasurementAt(tool, index);
}


kBeginClass(Go, GoProfileIntersect, GoProfileTool)
kAddVMethod(GoProfileIntersect, kObject, VRelease)
kAddVMethod(GoProfileIntersect, GoTool, VInit)
kAddVMethod(GoProfileIntersect, GoTool, VRead)
kAddVMethod(GoProfileIntersect, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileIntersect_Construct(GoProfileIntersect* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileIntersect), sensor, allocator);
}

GoFx(kStatus) GoProfileIntersect_VInit(GoProfileIntersect tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileIntersectClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_INTERSECT, sensor, alloc)); 
    kInitFields_(GoProfileIntersect, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileIntersectX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileIntersectZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileIntersectAngle), kTRUE, kNULL));

    obj->refLineType = GO_PROFILE_BASELINE_TYPE_LINE;

    kCheck(GoProfileLineRegion_Construct(&obj->refLineRegion, sensor, alloc)); 
    kCheck(GoProfileLineRegion_Construct(&obj->lineRegion, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileIntersect_VRelease(GoProfileIntersect tool)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool);

    kCheck(kDestroyRef(&obj->refLineRegion)); 
    kCheck(kDestroyRef(&obj->lineRegion)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileIntersect_VRead(GoProfileIntersect tool, kXml xml, kXmlItem item)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool); 
    kXmlItem lineItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(kXml_Child32s(xml, item, "RefType", &obj->refLineType));

    kCheck(!kIsNull(lineItem = kXml_Child(xml, item, "RefLine"))); 
    kCheck(GoProfileLineRegion_Read(obj->refLineRegion, xml, lineItem));
    kCheck(!kIsNull(lineItem = kXml_Child(xml, item, "Line"))); 
    kCheck(GoProfileLineRegion_Read(obj->lineRegion, xml, lineItem));

    return kOK;
}

GoFx(kStatus) GoProfileIntersect_VWrite(GoProfileIntersect tool, kXml xml, kXmlItem item)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool); 
    kXmlItem lineItem;

    kCheck(kXml_SetChild32s(xml, item, "RefType", obj->refLineType));

    kCheck(kXml_AddItem(xml, item, "RefLine", &lineItem)); 
    kCheck(GoProfileLineRegion_Write(obj->refLineRegion, xml, lineItem));
    kCheck(kXml_AddItem(xml, item, "Line", &lineItem)); 
    kCheck(GoProfileLineRegion_Write(obj->lineRegion, xml, lineItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileBaseline) GoProfileIntersect_RefLineType(GoProfileIntersect tool)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool); 
    return obj->refLineType;
}

GoFx(kStatus) GoProfileIntersect_SetRefLineType(GoProfileIntersect tool, GoProfileBaseline type)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refLineType = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileLineRegion) GoProfileIntersect_RefLine(GoProfileIntersect tool)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool); 
    return obj->refLineRegion;
}

GoFx(GoProfileLineRegion) GoProfileIntersect_Line(GoProfileIntersect tool)
{
    GoProfileIntersectClass* obj = GoProfileIntersect_Cast_(tool); 
    return obj->lineRegion;
}

GoFx(GoProfileIntersectX) GoProfileIntersect_XMeasurement(GoProfileIntersect tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_INTERSECT_X);
}

GoFx(GoProfileIntersectZ) GoProfileIntersect_ZMeasurement(GoProfileIntersect tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_INTERSECT_Z);
}

GoFx(GoProfileIntersectAngle) GoProfileIntersect_AngleMeasurement(GoProfileIntersect tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE);
}


kBeginClass(Go, GoProfileLine, GoProfileTool)
kAddVMethod(GoProfileLine, kObject, VRelease)
kAddVMethod(GoProfileLine, GoTool, VInit)
kAddVMethod(GoProfileLine, GoTool, VRead)
kAddVMethod(GoProfileLine, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileLine_Construct(GoProfileLine* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileLine), sensor, allocator);
}

GoFx(kStatus) GoProfileLine_VInit(GoProfileLine tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileLineClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_LINE, sensor, alloc)); 
    kInitFields_(GoProfileLine, tool);

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineStdDev), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMaxError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMinError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLinePercentile), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoProfileLine_VRelease(GoProfileLine tool)
{
    GoProfileLineClass* obj = GoProfileLine_Cast_(tool);

    kCheck(kDestroyRef(&obj->region)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileLine_VRead(GoProfileLine tool, kXml xml, kXmlItem item)
{
    GoProfileLineClass* obj = GoProfileLine_Cast_(tool);
    kXmlItem regionItem;

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Read(tool, xml, item));     

    return kOK;
}

GoFx(kStatus) GoProfileLine_VWrite(GoProfileLine tool, kXml xml, kXmlItem item)
{
    GoProfileLineClass* obj = GoProfileLine_Cast_(tool);
    kXmlItem regionItem;

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item));        

    return kOK;
}

GoFx(GoProfileRegion) GoProfileLine_Region(GoProfileLine tool)
{
    GoProfileLineClass* obj = GoProfileLine_Cast_(tool);
    return obj->region;
}

GoFx(GoProfileLineStdDev) GoProfileLine_StdDevMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_STDDEV);
}

GoFx(GoProfileLineMaxError) GoProfileLine_MaxErrorMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX);
}

GoFx(GoProfileLineMinError) GoProfileLine_MinErrorMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN);
}

GoFx(GoProfileLinePercentile) GoProfileLine_PercentileMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_PERCENTILE);
}


kBeginClass(Go, GoProfilePanel, GoProfileTool)
kAddVMethod(GoProfilePanel, kObject, VRelease)
kAddVMethod(GoProfilePanel, GoTool, VInit)
kAddVMethod(GoProfilePanel, GoTool, VRead)
kAddVMethod(GoProfilePanel, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfilePanel_Construct(GoProfilePanel* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfilePanel), sensor, allocator);
}

GoFx(kStatus) GoProfilePanel_VInit(GoProfilePanel tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfilePanelClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_PANEL, sensor, alloc)); 
    kInitFields_(GoProfilePanel, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePanelGap), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePanelFlush), kTRUE, kNULL));

    obj->maxGapWidth = 0;
    obj->refEdgeSide = GO_PROFILE_PANEL_SIDE_LEFT;

    kCheck(GoProfileEdge_Construct(&obj->leftEdge, sensor, alloc)); 
    kCheck(GoProfileEdge_Construct(&obj->rightEdge, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfilePanel_VRelease(GoProfilePanel tool)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);

    kCheck(kDestroyRef(&obj->leftEdge)); 
    kCheck(kDestroyRef(&obj->rightEdge)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfilePanel_VRead(GoProfilePanel tool, kXml xml, kXmlItem item)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);
    kXmlItem edgeItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(kXml_Child32s(xml, item, "RefSide", &obj->refEdgeSide));
    kCheck(kXml_Child64f(xml, item, "MaxGapWidth", &obj->maxGapWidth));

    kCheck(!kIsNull(edgeItem = kXml_Child(xml, item, "LeftEdge"))); 
    kCheck(GoProfileEdge_Read(obj->leftEdge, xml, edgeItem));
    kCheck(!kIsNull(edgeItem = kXml_Child(xml, item, "RightEdge"))); 
    kCheck(GoProfileEdge_Read(obj->rightEdge, xml, edgeItem));

    return kOK;
}

GoFx(kStatus) GoProfilePanel_VWrite(GoProfilePanel tool, kXml xml, kXmlItem item)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);
    kXmlItem edgeItem;

    kCheck(kXml_SetChild32s(xml, item, "RefSide", obj->refEdgeSide));
    kCheck(kXml_SetChild64f(xml, item, "MaxGapWidth", obj->maxGapWidth));

    kCheck(kXml_AddItem(xml, item, "LeftEdge", &edgeItem)); 
    kCheck(GoProfileEdge_Write(obj->leftEdge, xml, edgeItem));
    kCheck(kXml_AddItem(xml, item, "RightEdge", &edgeItem)); 
    kCheck(GoProfileEdge_Write(obj->rightEdge, xml, edgeItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(k64f) GoProfilePanel_MaxGapWidth(GoProfilePanel tool)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);
    return obj->maxGapWidth;
}

GoFx(kStatus) GoProfilePanel_SetMaxGapWidth(GoProfilePanel tool, k64f width)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxGapWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfilePanelSide) GoProfilePanel_RefEdgeSide(GoProfilePanel tool)
{
    GoProfilePanelClass* obj = tool;
    return obj->refEdgeSide;
}

GoFx(kStatus) GoProfilePanel_SetRefEdgeSide(GoProfilePanel tool, GoProfilePanelSide side)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refEdgeSide = side;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileEdge) GoProfilePanel_LeftEdge(GoProfilePanel tool)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);
    return obj->leftEdge;
}

GoFx(GoProfileEdge) GoProfilePanel_RightEdge(GoProfilePanel tool)
{
    GoProfilePanelClass* obj = GoProfilePanel_Cast_(tool);
    return obj->rightEdge;
}

GoFx(GoProfilePanelGap) GoProfilePanel_GapMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_GAP);
}

GoFx(GoProfilePanelFlush) GoProfilePanel_FlushMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_FLUSH);
}


kBeginClass(Go, GoProfilePosition, GoProfileTool)
kAddVMethod(GoProfilePosition, kObject, VRelease)
kAddVMethod(GoProfilePosition, GoTool, VInit)
kAddVMethod(GoProfilePosition, GoTool, VRead)
kAddVMethod(GoProfilePosition, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfilePosition_Construct(GoProfilePosition* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfilePosition), sensor, allocator);
}

GoFx(kStatus) GoProfilePosition_VInit(GoProfilePosition tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfilePositionClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_POSITION, sensor, alloc)); 
    kInitFields_(GoProfilePosition, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePositionX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePositionZ), kTRUE, kNULL));

    kCheck(GoProfileFeature_Construct(&obj->feature, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfilePosition_VRelease(GoProfilePosition tool)
{
    GoProfilePositionClass* obj = GoProfilePosition_Cast_(tool);

    kCheck(kDestroyRef(&obj->feature)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfilePosition_VRead(GoProfilePosition tool, kXml xml, kXmlItem item)
{
    GoProfilePositionClass* obj = GoProfilePosition_Cast_(tool);
    kXmlItem featureItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature"))); 
    kCheck(GoProfileFeature_Read(obj->feature, xml, featureItem));

    return kOK;
}

GoFx(kStatus) GoProfilePosition_VWrite(GoProfilePosition tool, kXml xml, kXmlItem item)
{
    GoProfilePositionClass* obj = GoProfilePosition_Cast_(tool);
    kXmlItem featureItem;

    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem)); 
    kCheck(GoProfileFeature_Write(obj->feature, xml, featureItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileFeature) GoProfilePosition_Feature(GoProfilePosition tool)
{
    GoProfilePositionClass* obj = GoProfilePosition_Cast_(tool);
    return obj->feature;
}

GoFx(GoProfilePositionX) GoProfilePosition_XMeasurement(GoProfilePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_POSITION_X);
}

GoFx(GoProfilePositionZ) GoProfilePosition_ZMeasurement(GoProfilePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_POSITION_Z);
}


kBeginClass(Go, GoProfileStrip, GoProfileTool)
kAddVMethod(GoProfileStrip, kObject, VRelease)
kAddVMethod(GoProfileStrip, GoTool, VInit)
kAddVMethod(GoProfileStrip, GoTool, VRead)
kAddVMethod(GoProfileStrip, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoProfileStrip_Construct(GoProfileStrip* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileStrip), sensor, allocator);
}

GoFx(kStatus) GoProfileStrip_VInit(GoProfileStrip tool, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileStripClass* obj = tool; 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_STRIP, sensor, alloc)); 
    kInitFields_(GoProfileStrip, tool);

    obj->leftEdge = GO_PROFILE_STRIP_EDGE_TYPE_RISING 
        | GO_PROFILE_STRIP_EDGE_TYPE_FALLING 
        | GO_PROFILE_STRIP_EDGE_TYPE_DATA_END 
        | GO_PROFILE_STRIP_EDGE_TYPE_VOID;

    obj->rightEdge = GO_PROFILE_STRIP_EDGE_TYPE_RISING 
        | GO_PROFILE_STRIP_EDGE_TYPE_FALLING 
        | GO_PROFILE_STRIP_EDGE_TYPE_DATA_END 
        | GO_PROFILE_STRIP_EDGE_TYPE_VOID;

    obj->tiltEnabled = kTRUE;
    obj->supportWidth = 5.0;
    obj->minHeight = 2.0;

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileStrip_VRelease(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    kDestroyRef(&obj->region);

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileStrip_VRead(GoProfileStrip tool, kXml xml, kXmlItem item)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "BaseType", &obj->baseType));
    kCheck(kXml_Child32s(xml, item, "LeftEdge", &obj->leftEdge));
    kCheck(kXml_Child32s(xml, item, "RightEdge", &obj->rightEdge));
    kCheck(kXml_ChildBool(xml, item, "TiltEnabled", &obj->tiltEnabled));

    kCheck(kXml_Child64f(xml, item, "SupportWidth", &obj->supportWidth));
    kCheck(kXml_Child64f(xml, item, "TransitionWidth", &obj->transitionWidth));
    kCheck(kXml_Child64f(xml, item, "MinWidth", &obj->minWidth));
    kCheck(kXml_Child64f(xml, item, "MinHeight", &obj->minHeight));
    kCheck(kXml_Child64f(xml, item, "MaxVoidWidth", &obj->maxVoidWidth));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoProfileStrip_VWrite(GoProfileStrip tool, kXml xml, kXmlItem item)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "BaseType", obj->baseType));
    kCheck(kXml_SetChild32s(xml, item, "LeftEdge", obj->leftEdge));
    kCheck(kXml_SetChild32s(xml, item, "RightEdge", obj->rightEdge));
    kCheck(kXml_SetChildBool(xml, item, "TiltEnabled", obj->tiltEnabled));

    kCheck(kXml_SetChild64f(xml, item, "SupportWidth", obj->supportWidth));
    kCheck(kXml_SetChild64f(xml, item, "TransitionWidth", obj->transitionWidth));
    kCheck(kXml_SetChild64f(xml, item, "MinWidth", obj->minWidth));
    kCheck(kXml_SetChild64f(xml, item, "MinHeight", obj->minHeight));
    kCheck(kXml_SetChild64f(xml, item, "MaxVoidWidth", obj->maxVoidWidth));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoProfileStrip_SetBaseType(GoProfileStrip tool, GoProfileStripBaseType type)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->baseType = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileStripBaseType) GoProfileStrip_BaseType(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->baseType;
}

GoFx(k8u) GoProfileStrip_LeftEdge(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->leftEdge;
}

GoFx(kStatus) GoProfileStrip_SetLeftEdge(GoProfileStrip tool, k8u leftEdge)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->leftEdge = leftEdge;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k8u) GoProfileStrip_RightEdge(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->rightEdge;
}

GoFx(kStatus) GoProfileStrip_SetRightEdge(GoProfileStrip tool, k8u rightEdge)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->rightEdge = rightEdge;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileStrip_TiltEnabled(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->tiltEnabled;
}
GoFx(kStatus) GoProfileStrip_EnableTilt(GoProfileStrip tool, kBool enable)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_TransitionWidth(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->transitionWidth;
}

GoFx(kStatus) GoProfileStrip_SetTransitionWidth(GoProfileStrip tool, k64f value)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->transitionWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_MinWidth(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->minWidth;
}

GoFx(kStatus) GoProfileStrip_SetMinWidth(GoProfileStrip tool, k64f value)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->minWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_MinHeight(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->minHeight;
}

GoFx(kStatus) GoProfileStrip_SetMinHeight(GoProfileStrip tool, k64f value)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->minHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_MaxVoidWidth(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->maxVoidWidth;
}

GoFx(kStatus) GoProfileStrip_SetMaxVoidWidth(GoProfileStrip tool, k64f value)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxVoidWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileStrip_Region(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return obj->region;
}

GoFx(kStatus) GoProfileStrip_AddMeasurement(GoProfileStrip tool, GoMeasurementType type, GoMeasurement* measurement)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);
    kType classType;

    if (type != GO_MEASUREMENT_PROFILE_STRIP_POSITION_X
        && type != GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z
        && type != GO_MEASUREMENT_PROFILE_STRIP_HEIGHT
        && type != GO_MEASUREMENT_PROFILE_STRIP_WIDTH)
    {
        return kERROR_PARAMETER;
    }
    else
    {
        switch(type)
        {
        case GO_MEASUREMENT_PROFILE_STRIP_POSITION_X: classType = kTypeOf(GoProfileStripX); break;
        case GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z: classType = kTypeOf(GoProfileStripZ); break;
        case GO_MEASUREMENT_PROFILE_STRIP_HEIGHT: classType = kTypeOf(GoProfileStripHeight); break;
        case GO_MEASUREMENT_PROFILE_STRIP_WIDTH: classType = kTypeOf(GoProfileStripWidth); break;
        }
    }

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoTool_AddMeasurement(tool, classType, kTRUE, measurement));
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoProfileStrip_RemoveMeasurement(GoProfileStrip tool, kSize index)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    kCheckArgs(index < GoTool_MeasurementCount(tool));
    kCheck(GoTool_RemoveMeasurement(tool, index));

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoProfileStrip_MeasurementCount(GoProfileStrip tool)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    return GoTool_MeasurementCount(tool);
}

GoFx(GoMeasurement) GoProfileStrip_MeasurementAt(GoProfileStrip tool, kSize index)
{
    GoProfileStripClass* obj = GoProfileStrip_Cast_(tool);

    kAssert(index < GoTool_MeasurementCount(tool));
    
    return GoTool_MeasurementAt(tool, index);
}
