/** 
 * @file    GoSurfaceTools.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoSurfaceTools.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoSurfaceTool, GoTool)
    kAddVMethod(GoSurfaceTool, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSurfaceTool_Init(GoSurfaceTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    GoSurfaceToolClass* obj = tool;

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc));

    obj->source = GO_DATA_SOURCE_TOP; 
    obj->xAnchor = -1;
    obj->yAnchor = -1;
    obj->zAnchor = -1;

    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc));
    kCheck(kArrayList_Construct(&obj->xAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->yAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAnchorOptions, kTypeOf(k32u), 0, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceTool_VRelease(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    kCheck(kDisposeRef(&obj->sourceOptions));
    kCheck(kDisposeRef(&obj->xAnchorOptions));
    kCheck(kDisposeRef(&obj->yAnchorOptions));
    kCheck(kDisposeRef(&obj->zAnchorOptions));

    return GoTool_VRelease(tool); 
}

GoFx(kStatus) GoSurfaceTool_Read(GoSurfaceTool tool, kXml xml, kXmlItem item)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);
    kXmlItem anchorItem = kNULL;
    kXmlItem tempItem = kNULL;
    kText128 text;

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Source")));
    kCheck(kXml_Item32s(xml, tempItem, &obj->source));    
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->sourceOptions));  

    kCheckArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
    kCheck(kXml_Child32s(xml, anchorItem, "X", &obj->xAnchor));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "X")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->xAnchorOptions));  

    kCheckArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
    kCheck(kXml_Child32s(xml, anchorItem, "Y", &obj->yAnchor));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "Y")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->yAnchorOptions));  

    kCheckArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
    kCheck(kXml_Child32s(xml, anchorItem, "Z", &obj->zAnchor));
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "Z")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->zAnchorOptions));  

    return GoTool_VRead(tool, xml, item);
}

GoFx(kStatus) GoSurfaceTool_Write(GoSurfaceTool tool, kXml xml, kXmlItem item)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);
    kXmlItem sourceItem = kNULL;
    kXmlItem anchorItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));

    kCheck(kXml_AddItem(xml, item, "Anchor", &anchorItem));
    kCheck(kXml_SetChild32s(xml, anchorItem, "X", obj->xAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Y", obj->yAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Z", obj->zAnchor));

    return GoTool_VWrite(tool, xml, item);
}

GoFx(kStatus) GoSurfaceTool_SetSource(GoSurfaceTool tool, GoDataSource source)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->sourceOptions), kArrayList_Count(obj->sourceOptions), source));
    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceTool_SourceOptionCount(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(k32u) GoSurfaceTool_SourceOptionAt(GoSurfaceTool tool, kSize index)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));

    return kArrayList_As_(obj->sourceOptions, index, k32u);
}

GoFx(GoDataSource) GoSurfaceTool_Source(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

GoFx(kSize) GoSurfaceTool_XAnchorOptionCount(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->xAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_XAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->xAnchorOptions));

    return kArrayList_As_(obj->xAnchorOptions, index, k32u);
}

GoFx(k32s) GoSurfaceTool_XAnchor(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->xAnchor;
}

GoFx(kStatus) GoSurfaceTool_SetXAnchor(GoSurfaceTool tool, k32s id)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

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

GoFx(kBool) GoSurfaceTool_XAnchorEnabled(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->xAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoSurfaceTool_YAnchorOptionCount(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->yAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_YAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->yAnchorOptions));

    return kArrayList_As_(obj->yAnchorOptions, index, k32u);
}

GoFx(k32s) GoSurfaceTool_YAnchor(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->yAnchor;
}

GoFx(kStatus) GoSurfaceTool_SetYAnchor(GoSurfaceTool tool, k32s id)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_Data(obj->yAnchorOptions), kArrayList_Count(obj->yAnchorOptions), id));
    }

    obj->yAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceTool_YAnchorEnabled(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->yAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoSurfaceTool_ZAnchorOptionCount(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->zAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_ZAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->zAnchorOptions));

    return kArrayList_As_(obj->zAnchorOptions, index, k32u);
}

GoFx(kStatus) GoSurfaceTool_SetZAnchor(GoSurfaceTool tool, k32s id)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

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


GoFx(kBool) GoSurfaceTool_ZAnchorEnabled(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->zAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoSurfaceTool_ZAnchor(GoSurfaceTool tool)
{
    GoSurfaceToolClass* obj = GoSurfaceTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->zAnchor;
}


kBeginClass(Go, GoSurfaceBox, GoSurfaceTool)
kAddVMethod(GoSurfaceBox, kObject, VRelease)
kAddVMethod(GoSurfaceBox, GoTool, VInit)
kAddVMethod(GoSurfaceBox, GoTool, VRead)
kAddVMethod(GoSurfaceBox, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceBox_Construct(GoSurfaceBox* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceBox), sensor, allocator);
}

GoFx(kStatus) GoSurfaceBox_VInit(GoSurfaceBox tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceBoxClass* obj = tool;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_BOUNDING_BOX, sensor, alloc)); 
    kInitFields_(GoSurfaceBox, tool);

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    obj->regionEnabled = kTRUE;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxLength), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxGlobalX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxGlobalY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxGlobalZAngle), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceBox_VRelease(GoSurfaceBox tool)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);

    kDestroyRef(&obj->region);

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceBox_VRead(GoSurfaceBox tool, kXml xml, kXmlItem item)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "ZRotationEnabled", &obj->zRotationEnabled));
    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceBox_VWrite(GoSurfaceBox tool, kXml xml, kXmlItem item)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "ZRotationEnabled", obj->zRotationEnabled));
    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kBool) GoSurfaceBox_ZRotationEnabled(GoSurfaceBox tool)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);

    return obj->zRotationEnabled;
}

GoFx(kStatus) GoSurfaceBox_EnableZRotation(GoSurfaceBox tool, kBool enable)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->zRotationEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceBox_RegionEnabled(GoSurfaceBox tool)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceBox_EnableRegion(GoSurfaceBox tool, kBool enable)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceBox_Region(GoSurfaceBox tool)
{
    GoSurfaceBoxClass* obj = GoSurfaceBox_Cast_(tool);

    return obj->region;
}

GoFx(GoSurfaceBoxX) GoSurfaceBox_XMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X);
}

GoFx(GoSurfaceBoxY) GoSurfaceBox_YMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y);
}

GoFx(GoSurfaceBoxZ) GoSurfaceBox_ZMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z);
}

GoFx(GoSurfaceBoxWidth) GoSurfaceBox_WidthMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH);
}

GoFx(GoSurfaceBoxLength) GoSurfaceBox_LengthMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH);
}

GoFx(GoSurfaceBoxHeight) GoSurfaceBox_HeightMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT);
}

GoFx(GoSurfaceBoxZAngle) GoSurfaceBox_ZAngleMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE);
}

GoFx(GoSurfaceBoxGlobalX) GoSurfaceBox_GlobalXMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X);
}

GoFx(GoSurfaceBoxGlobalY) GoSurfaceBox_GlobalYMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y);
}

GoFx(GoSurfaceBoxGlobalZAngle) GoSurfaceBox_GlobalZAngleMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE);
}


kBeginClass(Go, GoSurfaceCountersunkHole, GoSurfaceTool)
kAddVMethod(GoSurfaceCountersunkHole, kObject, VRelease)    
kAddVMethod(GoSurfaceCountersunkHole, GoTool, VInit)
kAddVMethod(GoSurfaceCountersunkHole, GoTool, VRead)
kAddVMethod(GoSurfaceCountersunkHole, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceCountersunkHole_Construct(GoSurfaceCountersunkHole* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceCountersunkHole), sensor, allocator);
}

GoFx(kStatus) GoSurfaceCountersunkHole_VInit(GoSurfaceCountersunkHole tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceCountersunkHoleClass* obj = tool;
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_COUNTERSUNK_HOLE, sensor, alloc)); 
    kInitFields_(GoSurfaceCountersunkHole, tool);

    obj->nominalBevelAngle = 100.0;
    obj->bevelAngleTolerance = 5.0;
    obj->nominalOuterRadius = 10.0;
    obj->outerRadiusTolerance = 1.0;
    obj->nominalInnerRadius = 4.0;
    obj->innerRadiusTolerance = 1.0;
    obj->bevelRadiusOffset = 4.0;
    obj->partialDetectionEnabled = kFALSE;
    obj->autoTiltEnabled = kTRUE;
    obj->tiltXAngle = 0.0;
    obj->tiltYAngle = 0.0;

    obj->regionEnabled = kTRUE;
    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    for (i = 0; i < GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleOuterRadius), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleDepth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleBevelAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleBevelRadius), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleXAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleYAngle), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceCountersunkHole_VRelease(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceCountersunkHole_VRead(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "NominalBevelAngle", &obj->nominalBevelAngle));
    kCheck(kXml_Child64f(xml, item, "BevelAngleTolerance", &obj->bevelAngleTolerance));

    kCheck(kXml_Child64f(xml, item, "NominalOuterRadius", &obj->nominalOuterRadius));
    kCheck(kXml_Child64f(xml, item, "OuterRadiusTolerance", &obj->outerRadiusTolerance));

    kCheck(kXml_Child64f(xml, item, "NominalInnerRadius", &obj->nominalInnerRadius));
    kCheck(kXml_Child64f(xml, item, "InnerRadiusTolerance", &obj->innerRadiusTolerance));

    kCheck(kXml_Child64f(xml, item, "BevelRadiusOffset", &obj->bevelRadiusOffset));

    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "CurveFitEnabled", kFALSE, &obj->curveFitEnabled));
    kCheck(GoConfig_Read64fOptional(xml, item, "CurveOrientation", 0.0, &obj->curveOrientation));

    kCheck(GoSurfaceTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHole_VWrite(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i; 

    kCheck(kXml_SetChild64f(xml, item, "NominalBevelAngle", obj->nominalBevelAngle));
    kCheck(kXml_SetChild64f(xml, item, "BevelAngleTolerance", obj->bevelAngleTolerance));

    kCheck(kXml_SetChild64f(xml, item, "NominalOuterRadius", obj->nominalOuterRadius));
    kCheck(kXml_SetChild64f(xml, item, "OuterRadiusTolerance", obj->outerRadiusTolerance));

    kCheck(kXml_SetChild64f(xml, item, "NominalInnerRadius", obj->nominalInnerRadius));
    kCheck(kXml_SetChild64f(xml, item, "InnerRadiusTolerance", obj->innerRadiusTolerance));

    kCheck(kXml_SetChild64f(xml, item, "BevelRadiusOffset", obj->bevelRadiusOffset));

    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(kXml_SetChildBool(xml, item, "CurveFitEnabled", obj->curveFitEnabled));
    kCheck(kXml_SetChild64f(xml, item, "CurveOrientation", obj->curveOrientation));

    kCheck(GoSurfaceTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalBevelAngle(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->nominalBevelAngle = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_NominalBevelAngle(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->nominalBevelAngle;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelAngleTolerance(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->bevelAngleTolerance = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_BevelAngleTolerance(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->bevelAngleTolerance;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalOuterRadius(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->nominalOuterRadius = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_NominalOuterRadius(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->nominalOuterRadius;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetOuterRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->outerRadiusTolerance = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_OuterRadiusTolerance(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->outerRadiusTolerance;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalInnerRadius(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->nominalInnerRadius = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_NominalInnerRadius(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->nominalInnerRadius;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetInnerRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->innerRadiusTolerance = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_InnerRadiusTolerance(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->innerRadiusTolerance;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelRadiusOffset(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->bevelRadiusOffset = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_BevelRadiusOffset(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->bevelRadiusOffset;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnablePartialDetection(GoSurfaceCountersunkHole tool, kBool enable)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->partialDetectionEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_PartialDetectionEnabled(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnableRegion(GoSurfaceCountersunkHole tool, kBool enable)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->regionEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_RegionEnabled(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(GoRegion3d) GoSurfaceCountersunkHole_Region(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->region;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnableRefRegions(GoSurfaceCountersunkHole tool, kBool enable)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->refRegionsEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_RefRegionsEnabled(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetRefRegionCount(GoSurfaceCountersunkHole tool, kSize count)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    kCheckArgs(count <= GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS);

    obj->refRegionCount = count;

    return kOK;
}

GoFx(kSize) GoSurfaceCountersunkHole_RefRegionCount(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->refRegionCount;
}

GoFx(GoSurfaceRegion2d) GoSurfaceCountersunkHole_RefRegionAt(GoSurfaceCountersunkHole tool, kSize index)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    kAssert(index < GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnableAutoTilt(GoSurfaceCountersunkHole tool, kBool enable)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->autoTiltEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_AutoTiltEnabled(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltXAngle(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->tiltXAngle = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_TiltXAngle(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltYAngle(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->tiltYAngle = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_TiltYAngle(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->tiltYAngle;
}


GoFx(kStatus) GoSurfaceCountersunkHole_EnableCurveFit(GoSurfaceCountersunkHole tool, kBool enable)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->curveFitEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_CurveFitEnabled(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->curveFitEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetCurveOrientation(GoSurfaceCountersunkHole tool, k64f value)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    obj->curveOrientation = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_CurveOrientation(GoSurfaceCountersunkHole tool)
{
    GoSurfaceCountersunkHoleClass* obj = GoSurfaceCountersunkHole_Cast_(tool);

    return obj->curveOrientation;
}

GoFx(GoSurfaceCountersunkHoleX) GoSurfaceCountersunkHole_XMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X);
}

GoFx(GoSurfaceCountersunkHoleY) GoSurfaceCountersunkHole_YMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y);
}

GoFx(GoSurfaceCountersunkHoleZ) GoSurfaceCountersunkHole_ZMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z);
}

GoFx(GoSurfaceCountersunkHoleOuterRadius) GoSurfaceCountersunkHole_OuterRadiusMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS);
}

GoFx(GoSurfaceCountersunkHoleDepth) GoSurfaceCountersunkHole_DepthMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH);
}

GoFx(GoSurfaceCountersunkHoleBevelRadius) GoSurfaceCountersunkHole_BevelRadiusMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS);
}

GoFx(GoSurfaceCountersunkHoleBevelAngle) GoSurfaceCountersunkHole_BevelAngleMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE);
}

GoFx(GoSurfaceCountersunkHoleXAngle) GoSurfaceCountersunkHole_XAngleMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE);
}

GoFx(GoSurfaceCountersunkHoleYAngle) GoSurfaceCountersunkHole_YAngleMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE);
}


kBeginClass(Go, GoSurfaceEllipse, GoSurfaceTool)
kAddVMethod(GoSurfaceEllipse, kObject, VRelease)
kAddVMethod(GoSurfaceEllipse, GoTool, VInit)
kAddVMethod(GoSurfaceEllipse, GoTool, VRead)
kAddVMethod(GoSurfaceEllipse, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceEllipse_Construct(GoSurfaceEllipse* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceEllipse), sensor, allocator);
}

GoFx(kStatus) GoSurfaceEllipse_VInit(GoSurfaceEllipse tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceEllipseClass* obj = tool;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_ELLIPSE, sensor, alloc)); 
    kInitFields_(GoSurfaceEllipse, tool);

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseMajor), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseMinor), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseRatio), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseZAngle), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceEllipse_VRelease(GoSurfaceEllipse tool)
{
    GoSurfaceEllipseClass* obj = GoSurfaceEllipse_Cast_(tool);

    kCheck(kDestroyRef(&obj->region));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceEllipse_VRead(GoSurfaceEllipse tool, kXml xml, kXmlItem item)
{
    GoSurfaceEllipseClass* obj = GoSurfaceEllipse_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipse_VWrite(GoSurfaceEllipse tool, kXml xml, kXmlItem item)
{
    GoSurfaceEllipseClass* obj = GoSurfaceEllipse_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipse_EnableRegion(GoSurfaceEllipse tool, kBool enable)
{
    GoSurfaceEllipseClass* obj = GoSurfaceEllipse_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceEllipse_RegionEnabled(GoSurfaceEllipse tool)
{
    GoSurfaceEllipseClass* obj = GoSurfaceEllipse_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(GoRegion3d) GoSurfaceEllipse_Region(GoSurfaceEllipse tool)
{
    GoSurfaceEllipseClass* obj = GoSurfaceEllipse_Cast_(tool);

    return obj->region;
}

GoFx(GoSurfaceEllipseMajor) GoSurfaceEllipse_MajorMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR);
}

GoFx(GoSurfaceEllipseMinor) GoSurfaceEllipse_MinorMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR);
}

GoFx(GoSurfaceEllipseRatio) GoSurfaceEllipse_RatioMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO);
}

GoFx(GoSurfaceEllipseZAngle) GoSurfaceEllipse_ZAngleMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE);
}


kBeginClass(Go, GoSurfaceHole, GoSurfaceTool)
kAddVMethod(GoSurfaceHole, kObject, VRelease)    
kAddVMethod(GoSurfaceHole, GoTool, VInit)
kAddVMethod(GoSurfaceHole, GoTool, VRead)
kAddVMethod(GoSurfaceHole, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceHole_Construct(GoSurfaceHole* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceHole), sensor, allocator);
}

GoFx(kStatus) GoSurfaceHole_VInit(GoSurfaceHole tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceHoleClass* obj = tool;
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_HOLE, sensor, alloc)); 
    kInitFields_(GoSurfaceHole, tool);

    obj->nominalRadius = 10.0;
    obj->radiusTolerance = 1.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleRadius), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceHole_VRelease(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceHole_VRead(GoSurfaceHole tool, kXml xml, kXmlItem item)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));
    kCheck(kXml_Child64f(xml, item, "RadiusTolerance", &obj->radiusTolerance));
    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoSurfaceTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceHole_VWrite(GoSurfaceHole tool, kXml xml, kXmlItem item)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i; 

    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius));
    kCheck(kXml_SetChild64f(xml, item, "RadiusTolerance", obj->radiusTolerance));
    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(GoSurfaceTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(k64f) GoSurfaceHole_NominalRadius(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->nominalRadius;
}

GoFx(kStatus) GoSurfaceHole_SetNominalRadius(GoSurfaceHole tool, k64f nominalRadius)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalRadius = nominalRadius;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}
GoFx(k64f) GoSurfaceHole_RadiusTolerance(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->radiusTolerance;
}

GoFx(kStatus) GoSurfaceHole_SetRadiusTolerance(GoSurfaceHole tool, k64f radiusTolerance)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->radiusTolerance = radiusTolerance;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceHole_PartialDetectionEnabled(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnablePartialDetection(GoSurfaceHole tool, kBool enable)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->partialDetectionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceHole_RegionEnabled(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableRegion(GoSurfaceHole tool, kBool enable)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceHole_Region(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->region;
}

GoFx(kBool) GoSurfaceHole_RefRegionsEnabled(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableRefRegions(GoSurfaceHole tool, kBool enable)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceHole_RefRegionCount(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->refRegionCount;
}

GoFx(kStatus) GoSurfaceHole_SetRefRegionCount(GoSurfaceHole tool, kSize count)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_HOLE_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoSurfaceHole_RefRegionAt(GoSurfaceHole tool, kSize index)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kAssert(index < GO_SURFACE_HOLE_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceHole_AutoTiltEnabled(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableAutoTilt(GoSurfaceHole tool, kBool enable)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceHole_TiltXAngle(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceHole_SetTiltXAngle(GoSurfaceHole tool, k64f value)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceHole_TiltYAngle(GoSurfaceHole tool)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceHole_SetTiltYAngle(GoSurfaceHole tool, k64f value)
{
    GoSurfaceHoleClass* obj = GoSurfaceHole_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceHoleX) GoSurfaceHole_XMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_X);
}

GoFx(GoSurfaceHoleY) GoSurfaceHole_YMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_Y);
}

GoFx(GoSurfaceHoleZ) GoSurfaceHole_ZMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_Z);
}

GoFx(GoSurfaceHoleRadius) GoSurfaceHole_RadiusMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_RADIUS);
}


kBeginClass(Go, GoSurfaceOpening, GoSurfaceTool)
kAddVMethod(GoSurfaceOpening, kObject, VRelease)
kAddVMethod(GoSurfaceOpening, GoTool, VInit)
kAddVMethod(GoSurfaceOpening, GoTool, VRead)
kAddVMethod(GoSurfaceOpening, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceOpening_Construct(GoSurfaceOpening* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceOpening), sensor, allocator);
}

GoFx(kStatus) GoSurfaceOpening_VInit(GoSurfaceOpening tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceOpeningClass* obj = tool;
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_OPENING, sensor, alloc)); 
    kInitFields_(GoSurfaceOpening, tool);

    obj->nominalWidth = 10.0;
    obj->nominalLength = 20.0;
    obj->nominalRadius = 5.0;
    obj->widthTolerance = 2.0;
    obj->lengthTolerance = 4.0;
    obj->angleTolerance = 5.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;
    obj->type = GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_OPENING_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningLength), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningAngle), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceOpening_VRelease(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_OPENING_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool); 
}

GoFx(kStatus) GoSurfaceOpening_VRead(GoSurfaceOpening tool, kXml xml, kXmlItem item)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "Type", &obj->type));

    kCheck(kXml_Child64f(xml, item, "NominalWidth", &obj->nominalWidth));
    kCheck(kXml_Child64f(xml, item, "NominalLength", &obj->nominalLength));
    kCheck(kXml_Child64f(xml, item, "NominalAngle", &obj->nominalAngle));
    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));

    kCheck(kXml_Child64f(xml, item, "WidthTolerance", &obj->widthTolerance));
    kCheck(kXml_Child64f(xml, item, "LengthTolerance", &obj->lengthTolerance));
    kCheck(kXml_Child64f(xml, item, "AngleTolerance", &obj->angleTolerance));

    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoSurfaceTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoSurfaceOpening_VWrite(GoSurfaceOpening tool, kXml xml, kXmlItem item)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i; 

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));

    kCheck(kXml_SetChild64f(xml, item, "NominalWidth", obj->nominalWidth));
    kCheck(kXml_SetChild64f(xml, item, "NominalLength", obj->nominalLength));
    kCheck(kXml_SetChild64f(xml, item, "NominalAngle", obj->nominalAngle));
    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius));

    kCheck(kXml_SetChild64f(xml, item, "WidthTolerance", obj->widthTolerance));
    kCheck(kXml_SetChild64f(xml, item, "LengthTolerance", obj->lengthTolerance));
    kCheck(kXml_SetChild64f(xml, item, "AngleTolerance", obj->angleTolerance));

    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceOpening_SetType(GoSurfaceOpening tool, GoSurfaceOpeningType type)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    obj->type = type;

    return kOK;
}

GoFx(GoSurfaceOpeningType) GoSurfaceOpening_Type(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->type;
}

GoFx(k64f) GoSurfaceOpening_NominalWidth(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->nominalWidth;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalWidth(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_NominalLength(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->nominalLength;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalLength(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalLength = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_NominalAngle(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->nominalAngle;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalAngle(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_NominalRadius(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->nominalRadius;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalRadius(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalRadius = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_WidthTolerance(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->nominalRadius;
}

GoFx(kStatus) GoSurfaceOpening_SetWidthTolerance(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->widthTolerance = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_LengthTolerance(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->lengthTolerance;
}

GoFx(kStatus) GoSurfaceOpening_SetLengthTolerance(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->lengthTolerance = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_AngleTolerance(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->angleTolerance;
}

GoFx(kStatus) GoSurfaceOpening_SetAngleTolerance(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->angleTolerance = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceOpening_PartialDetectionEnabled(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnablePartialDetection(GoSurfaceOpening tool, kBool enable)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->partialDetectionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceOpening_RegionEnabled(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableRegion(GoSurfaceOpening tool, kBool enable)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceOpening_Region(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->region;
}


GoFx(kBool) GoSurfaceOpening_RefRegionsEnabled(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableRefRegions(GoSurfaceOpening tool, kBool enable)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceOpening_RefRegionCount(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->refRegionCount;
}

GoFx(kStatus) GoSurfaceOpening_SetRefRegionCount(GoSurfaceOpening tool, kSize count)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_OPENING_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoSurfaceOpening_RefRegionAt(GoSurfaceOpening tool, kSize index)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kAssert(index < GO_SURFACE_OPENING_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceOpening_AutoTiltEnabled(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableAutoTilt(GoSurfaceOpening tool, kBool enable)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_TiltXAngle(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceOpening_SetTiltXAngle(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_TiltYAngle(GoSurfaceOpening tool)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceOpening_SetTiltYAngle(GoSurfaceOpening tool, k64f value)
{
    GoSurfaceOpeningClass* obj = GoSurfaceOpening_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceOpeningX) GoSurfaceOpening_XMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_X);
}

GoFx(GoSurfaceOpeningY) GoSurfaceOpening_YMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_Y);
}

GoFx(GoSurfaceOpeningZ) GoSurfaceOpening_ZMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_Z);
}

GoFx(GoSurfaceOpeningWidth) GoSurfaceOpening_WidthMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_WIDTH);
}

GoFx(GoSurfaceOpeningLength) GoSurfaceOpening_LengthMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_LENGTH);
}

GoFx(GoSurfaceOpeningAngle) GoSurfaceOpening_AngleMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_ANGLE);
}


kBeginClass(Go, GoSurfacePlane, GoSurfaceTool)
kAddVMethod(GoSurfacePlane, kObject, VRelease)
kAddVMethod(GoSurfacePlane, GoTool, VInit)
kAddVMethod(GoSurfacePlane, GoTool, VRead)
kAddVMethod(GoSurfacePlane, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfacePlane_Construct(GoSurfacePlane* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfacePlane), sensor, allocator);
}

GoFx(kStatus) GoSurfacePlane_VInit(GoSurfacePlane tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfacePlaneClass* obj = tool;
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_PLANE, sensor, alloc)); 
    kInitFields_(GoSurfacePlane, tool);

    obj->regionsEnabled = kTRUE;
    obj->regionCount = 1;

    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kCheck(GoRegion3d_Construct(&obj->regions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneXAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneYAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneZOffset), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfacePlane_VRelease(GoSurfacePlane tool)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);
    kSize i;

    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kDestroyRef(&obj->regions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfacePlane_VRead(GoSurfacePlane tool, kXml xml, kXmlItem item)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);
    kSize i;
    kXmlItem regionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionsEnabled", &obj->regionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RegionCount", &obj->regionCount));

    kCheck(!kIsNull(regionsItem = kXml_Child(xml, item, "Regions")));
    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, regionsItem, i)));
        kCheck(GoRegion3d_Read(obj->regions[i], xml, regionItem));
    }

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfacePlane_VWrite(GoSurfacePlane tool, kXml xml, kXmlItem item)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);
    kSize i;
    kXmlItem regionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionsEnabled", obj->regionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RegionCount", obj->regionCount));

    kCheck(kXml_AddItem(xml, item, "Regions", &regionsItem));
    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, regionsItem, "Region", &regionItem));
        kCheck(GoRegion3d_Write(obj->regions[i], xml, regionItem));
    }

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kBool) GoSurfacePlane_RegionsEnabled(GoSurfacePlane tool)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);

    return obj->regionsEnabled;
}

GoFx(kStatus) GoSurfacePlane_EnableRegions(GoSurfacePlane tool, kBool enable)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfacePlane_RegionCount(GoSurfacePlane tool)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);

    return obj->regionCount;
}

GoFx(kStatus) GoSurfacePlane_SetRegionCount(GoSurfacePlane tool, kSize count)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    kCheckArgs(count <= GO_SURFACE_PLANE_MAX_REGIONS);
    obj->regionCount = count;

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfacePlane_RegionAt(GoSurfacePlane tool, kSize index)
{
    GoSurfacePlaneClass* obj = GoSurfacePlane_Cast_(tool);

    kAssert(index < GO_SURFACE_PLANE_MAX_REGIONS);

    return obj->regions[index];
}

GoFx(GoSurfacePlaneXAngle) GoSurfacePlane_XAngleMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE);
}

GoFx(GoSurfacePlaneYAngle) GoSurfacePlane_YAngleMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE);
}

GoFx(GoSurfacePlaneZOffset) GoSurfacePlane_ZOffsetMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET);
}


kBeginClass(Go, GoSurfacePosition, GoSurfaceTool)
kAddVMethod(GoSurfacePosition, kObject, VRelease)
kAddVMethod(GoSurfacePosition, GoTool, VInit)
kAddVMethod(GoSurfacePosition, GoTool, VRead)
kAddVMethod(GoSurfacePosition, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfacePosition_Construct(GoSurfacePosition* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfacePosition), sensor, allocator);
}

GoFx(kStatus) GoSurfacePosition_VInit(GoSurfacePosition tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfacePositionClass* obj = tool;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_POSITION, sensor, alloc)); 
    kInitFields_(GoSurfacePosition, tool);

    kCheck(GoSurfaceFeature_Construct(&obj->feature, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePositionX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePositionY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePositionZ), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfacePosition_VRelease(GoSurfacePosition tool)
{
    GoSurfacePositionClass* obj = GoSurfacePosition_Cast_(tool);

    kCheck(kDestroyRef(&obj->feature));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfacePosition_VRead(GoSurfacePosition tool, kXml xml, kXml item)
{
    GoSurfacePositionClass* obj = GoSurfacePosition_Cast_(tool);
    kXmlItem featureItem = kNULL;

    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature")));
    kCheck(GoSurfaceFeature_Read(obj->feature, xml, featureItem));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfacePosition_VWrite(GoSurfacePosition tool, kXml xml, kXml item)
{
    GoSurfacePositionClass* obj = GoSurfacePosition_Cast_(tool);
    kXmlItem featureItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem));
    kCheck(GoSurfaceFeature_Write(obj->feature, xml, featureItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(GoSurfaceFeature) GoSurfacePosition_Feature(GoSurfacePosition tool)
{
    GoSurfacePositionClass* obj = GoSurfacePosition_Cast_(tool);

    return obj->feature;
}

GoFx(GoSurfacePositionX) GoSurfacePosition_XMeasurement(GoSurfacePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_POSITION_X);
}

GoFx(GoSurfacePositionY) GoSurfacePosition_YMeasurement(GoSurfacePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_POSITION_Y);
}

GoFx(GoSurfacePositionZ) GoSurfacePosition_ZMeasurement(GoSurfacePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_POSITION_Z);
}


kBeginClass(Go, GoSurfaceStud, GoSurfaceTool)
kAddVMethod(GoSurfaceStud, kObject, VRelease)
kAddVMethod(GoSurfaceStud, GoTool, VInit)
kAddVMethod(GoSurfaceStud, GoTool, VRead)
kAddVMethod(GoSurfaceStud, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceStud_Construct(GoSurfaceStud* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceStud), sensor, allocator);
}

GoFx(kStatus) GoSurfaceStud_VInit(GoSurfaceStud tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceStudClass* obj = tool;
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_STUD, sensor, alloc)); 
    kInitFields_(GoSurfaceStud, tool);

    obj->studRadius = 5.0;
    obj->studHeight = 20.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudBaseX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudBaseY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudBaseZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudTipX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudTipY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudTipZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudRadius), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceStud_VRelease(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceStud_VRead(GoSurfaceStud tool, kXml xml, kXmlItem item)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);
    kSize i;
    kXmlItem refRegionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "StudRadius", &obj->studRadius));
    kCheck(kXml_Child64f(xml, item, "StudHeight", &obj->studHeight));
    kCheck(kXml_Child64f(xml, item, "BaseHeight", &obj->baseHeight));
    kCheck(kXml_Child64f(xml, item, "TipHeight", &obj->tipHeight));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    kCheck(kXml_ChildCount(xml, refRegionsItem) == GO_SURFACE_STUD_MAX_REF_REGIONS);

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_Child(xml, refRegionsItem, "RefRegion")));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceStud_VWrite(GoSurfaceStud tool, kXml xml, kXmlItem item)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);
    kSize i;
    kXmlItem refRegionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild64f(xml, item, "StudRadius", obj->studRadius));
    kCheck(kXml_SetChild64f(xml, item, "StudHeight", obj->studHeight));
    kCheck(kXml_SetChild64f(xml, item, "BaseHeight", obj->baseHeight));
    kCheck(kXml_SetChild64f(xml, item, "TipHeight", obj->tipHeight));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }
    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}


GoFx(k64f) GoSurfaceStud_StudRadius(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->studRadius;
}

GoFx(kStatus) GoSurfaceStud_SetStudRadius(GoSurfaceStud tool, k64f value)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->studRadius = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_StudHeight(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->studHeight;
}

GoFx(kStatus) GoSurfaceStud_SetStudHeight(GoSurfaceStud tool, k64f value)

{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->studHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_BaseHeight(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->baseHeight;
}

GoFx(kStatus) GoSurfaceStud_SetBaseHeight(GoSurfaceStud tool, k64f value)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->baseHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_TipHeight(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->tipHeight;
}

GoFx(kStatus) GoSurfaceStud_SetTipHeight(GoSurfaceStud tool, k64f value)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tipHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceStud_RegionEnabled(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceStud_EnableRegion(GoSurfaceStud tool, kBool enable)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceStud_Region(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->region;
}

GoFx(kBool) GoSurfaceStud_RefRegionsEnabled(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceStud_EnableRefRegions(GoSurfaceStud tool, kBool enable)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoSurfaceStud_SetRefRegionCount(GoSurfaceStud tool, kSize count)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_STUD_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceStud_RefRegionCount(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->refRegionCount;
}

GoFx(GoSurfaceRegion2d) GoSurfaceStud_RefRegionAt(GoSurfaceStud tool, kSize index)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kAssert(index < GO_SURFACE_STUD_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceStud_AutoTiltEnabled(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceStud_EnableAutoTilt(GoSurfaceStud tool, kBool enable)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_TiltXAngle(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceStud_SetTiltXAngle(GoSurfaceStud tool, k64f value)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_TiltYAngle(GoSurfaceStud tool)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceStud_SetTiltYAngle(GoSurfaceStud tool, k64f value)
{
    GoSurfaceStudClass* obj = GoSurfaceStud_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceStudBaseX) GoSurfaceStud_BaseXMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_BASE_X);
}

GoFx(GoSurfaceStudBaseY) GoSurfaceStud_BaseYMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_BASE_Y);
}

GoFx(GoSurfaceStudBaseZ) GoSurfaceStud_BaseZMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_BASE_Z);
}

GoFx(GoSurfaceStudTipX) GoSurfaceStud_TipXMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_TIP_X);
}

GoFx(GoSurfaceStudTipY) GoSurfaceStud_TipYMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_TIP_Y);
}

GoFx(GoSurfaceStudTipZ) GoSurfaceStud_TipZMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_TIP_Z);
}

GoFx(GoSurfaceStudRadius) GoSurfaceStud_RadiusMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_RADIUS);
}


kBeginClass(Go, GoSurfaceVolume, GoSurfaceTool)
kAddVMethod(GoSurfaceVolume, kObject, VRelease)
kAddVMethod(GoSurfaceVolume, GoTool, VInit)
kAddVMethod(GoSurfaceVolume, GoTool, VRead)
kAddVMethod(GoSurfaceVolume, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoSurfaceVolume_Construct(GoSurfaceVolume* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceVolume), sensor, allocator);
}

GoFx(kStatus) GoSurfaceVolume_VInit(GoSurfaceVolume tool, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceVolumeClass* obj = tool;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_VOLUME, sensor, alloc)); 
    kInitFields_(GoSurfaceVolume, tool);

    obj->regionEnabled = kTRUE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceVolumeVolume), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceVolumeArea), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceVolumeThickness), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoSurfaceVolume_VRelease(GoSurfaceVolume tool)
{
    GoSurfaceVolumeClass* obj = GoSurfaceVolume_Cast_(tool);

    kCheck(kDestroyRef(&obj->region));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceVolume_VRead(GoSurfaceVolume tool, kXml xml, kXmlItem item)
{
    GoSurfaceVolumeClass* obj = GoSurfaceVolume_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceVolume_VWrite(GoSurfaceVolume tool, kXml xml, kXmlItem item)
{
    GoSurfaceVolumeClass* obj = GoSurfaceVolume_Cast_(tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}


GoFx(kBool) GoSurfaceVolume_RegionEnabled(GoSurfaceVolume tool)
{
    GoSurfaceVolumeClass* obj = GoSurfaceVolume_Cast_(tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceVolume_EnableRegion(GoSurfaceVolume tool, kBool enable)
{
    GoSurfaceVolumeClass* obj = GoSurfaceVolume_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    obj->regionEnabled = enable;

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceVolume_Region(GoSurfaceVolume tool)
{
    GoSurfaceVolumeClass* obj = GoSurfaceVolume_Cast_(tool);

    return obj->region;
}

GoFx(GoSurfaceVolumeVolume) GoSurfaceVolume_VolumeMeasurement(GoSurfaceVolume tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_VOLUME_VOLUME);
}

GoFx(GoSurfaceVolumeArea) GoSurfaceVolume_AreaMeasurement(GoSurfaceVolume tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_VOLUME_AREA);
}

GoFx(GoSurfaceVolumeThickness) GoSurfaceVolume_ThicknessMeasurement(GoSurfaceVolume tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS);
}
