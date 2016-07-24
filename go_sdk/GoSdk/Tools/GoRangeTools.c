#include <GoSdk/Tools/GoRangeTools.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoRangeTool, GoTool)
kAddVMethod(GoRangeTool, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoRangeTool_Init(GoRangeTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    GoRangeToolClass* obj = tool; 

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc)); 
    kInitFields_(GoRangeTool, tool);

    obj->source = GO_DATA_SOURCE_TOP; 
    obj->zAnchor = -1;
    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAnchorOptions, kTypeOf(k32u), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoRangeTool_VRelease(GoRangeTool tool)
{
    GoRangeToolClass* obj = tool; 

    kCheck(kDisposeRef(&obj->zAnchorOptions));
    kCheck(kDisposeRef(&obj->sourceOptions));
    kCheck(GoTool_VRelease(tool)); 

    return kOK; 
}

GoFx(kStatus) GoRangeTool_Read(GoRangeTool tool, kXml xml, kXmlItem item)
{
    GoRangeToolClass* obj = tool; 
    kText128 text; 
    kXmlItem tempItem = kNULL;

    kCheck(GoTool_VRead(tool, xml, item));

    kCheck(kXml_Child32s(xml, item, "Source", &obj->source));
    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Source")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->sourceOptions));  

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Anchor")));
    kCheck(kXml_Child32s(xml, tempItem, "Z", &obj->zAnchor));
    kCheck(!kIsNull(tempItem = kXml_Child(xml, tempItem, "Z")));
    kCheck(kXml_AttrText(xml, tempItem, "options", text, kCountOf(text))); 
    kCheck(GoOptionList_ParseList32u(text, obj->zAnchorOptions));  

    return kOK; 
}

GoFx(kStatus) GoRangeTool_Write(GoRangeTool tool, kXml xml, kXmlItem item)
{
    GoRangeToolClass* obj = tool; 
    kXmlItem anchorItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));

    kCheck(kXml_AddItem(xml, item, "Anchor", &anchorItem));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Z", obj->zAnchor));

    kCheck(GoTool_VWrite(tool, xml, item));

    return kOK; 
}

GoFx(kSize) GoRangeTool_SourceOptionCount(GoRangeTool tool)
{
    GoRangeToolClass* obj = GoRangeTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(GoDataSource) GoRangeTool_SourceOptionAt(GoRangeTool tool, kSize index)
{
    GoRangeToolClass* obj = GoRangeTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));
    
    return kArrayList_As_(obj->sourceOptions, index, GoDataSource);
}

GoFx(kStatus) GoRangeTool_SetSource(GoRangeTool tool, GoDataSource source)
{
    GoRangeToolClass* obj = tool; 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->sourceOptions), kArrayList_Count(obj->sourceOptions), source));
    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoDataSource) GoRangeTool_Source(GoRangeTool tool)
{
    GoRangeToolClass* obj = tool; 

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

kBeginClass(Go, GoRangePosition, GoRangeTool)
kAddVMethod(GoRangePosition, kObject, VRelease)
kAddVMethod(GoRangePosition, GoTool, VInit)
kAddVMethod(GoRangePosition, GoTool, VRead)
kAddVMethod(GoRangePosition, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoRangePosition_Construct(GoRangePosition* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoRangePosition), sensor, allocator);
}

GoFx(kStatus) GoRangePosition_VInit(GoRangePosition tool, kType type, kObject sensor, kAlloc alloc)
{
    GoRangePositionClass* obj = tool; 

    kCheck(GoRangeTool_Init(tool, type, GO_TOOL_RANGE_POSITION, sensor, alloc)); 
    kInitFields_(GoRangePosition, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoRangePositionZ), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoRangePosition_VRelease(GoRangePosition tool)
{
    GoRangePositionClass* obj = GoRangePosition_Cast_(tool);

    return GoRangeTool_VRelease(tool); 
}

GoFx(kStatus) GoRangePosition_VRead(GoRangePosition tool, kXml xml, kXmlItem item)
{
    GoRangePositionClass* obj = GoRangePosition_Cast_(tool);

    kCheck(GoRangeTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoRangePosition_VWrite(GoRangePosition tool, kXml xml, kXmlItem item)
{
    GoRangePositionClass* obj = GoRangePosition_Cast_(tool);

    kCheck(GoRangeTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoRangePositionZ) GoRangePosition_ZMeasurement(GoRangePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_RANGE_POSITION_Z);
}


kBeginClass(Go, GoRangeThickness, GoRangeTool)
kAddVMethod(GoRangeThickness, kObject, VRelease)
kAddVMethod(GoRangeThickness, GoTool, VInit)
kAddVMethod(GoRangeThickness, GoTool, VRead)
kAddVMethod(GoRangeThickness, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoRangeThickness_Construct(GoRangeThickness* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoRangeThickness), sensor, allocator);
}

GoFx(kStatus) GoRangeThickness_VInit(GoRangeThickness tool, kType type, kObject sensor, kAlloc alloc)
{
    GoRangeThicknessClass* obj = tool; 

    kCheck(GoRangeTool_Init(tool, type, GO_TOOL_RANGE_THICKNESS, sensor, alloc)); 
    kInitFields_(GoRangeThickness, tool);

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoRangeThicknessThickness), kTRUE, kNULL));

    return kOK; 
}

GoFx(kStatus) GoRangeThickness_VRelease(GoRangeThickness tool)
{
    GoRangeThicknessClass* obj = GoRangeThickness_Cast_(tool);

    return GoRangeTool_VRelease(tool);
}

GoFx(kStatus) GoRangeThickness_VRead(GoRangeThickness tool, kXml xml, kXmlItem item)
{
    GoRangeThicknessClass* obj = GoRangeThickness_Cast_(tool);

    kCheck(kXml_ChildBool(xml, item, "Absolute", &obj->absoluteEnabled));
    kCheck(GoRangeTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoRangeThickness_VWrite(GoRangeThickness tool, kXml xml, kXmlItem item)
{
    GoRangeThicknessClass* obj = GoRangeThickness_Cast_(tool);

    kCheck(kXml_SetChildBool(xml, item, "Absolute", obj->absoluteEnabled));
    kCheck(GoRangeTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kBool) GoRangeThickness_AbsoluteEnabled(GoRangeThickness tool)
{
    GoRangeThicknessClass* obj = GoRangeThickness_Cast_(tool);

    return obj->absoluteEnabled;
}

GoFx(kStatus) GoRangeThickness_EnableAbsolute(GoRangeThickness tool, kBool enable)
{
    GoRangeThicknessClass* obj = GoRangeThickness_Cast_(tool);

    obj->absoluteEnabled = enable;

    return kOK;
}

GoFx(GoRangeThickness) GoRangeThickness_ThicknessMeasurement(GoRangeThickness tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS);
}
