/** 
 * @file    GoTools.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoTools.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/Tools/GoMeasurements.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoToolOption, kObject)
    kAddVMethod(GoToolOption, kObject, VRelease)
kEndClass()


GoFx(kStatus) GoToolOption_Construct(GoToolOption* option, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoToolOption), option));

    if (!kSuccess(status = GoToolOption_Init(*option, kTypeOf(GoToolOption), alloc)))
    {
        kAlloc_FreeRef(alloc, option);
    }

    return status;
}

GoFx(kStatus) GoToolOption_Init(GoToolOption option, kType type, kAlloc alloc)
{
    GoToolOptionClass* obj = option;
    kStatus exception;

    kCheck(kObject_Init(option, type, alloc));
    kInitFields_(GoToolOption, option);

    kTry
    {
        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(GoMeasurementOption), 0, alloc));
        
    }
    kCatch(&exception)
    {
        GoToolOption_VRelease(option);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoToolOption_VRelease(GoToolOption option)
{
    GoToolOptionClass* obj = GoToolOption_Cast_(option);
    
    kCheck(kDisposeRef(&obj->measurementOptions));

    return kObject_VRelease(option);
}

GoFx(const kChar*) GoToolOption_Name(GoToolOption option)
{
    GoToolOptionClass* obj = GoToolOption_Cast_(option);

    return obj->name;
}

GoFx(kBool) GoToolOption_IsCustom(GoToolOption option)
{
    GoToolOptionClass* obj = GoToolOption_Cast_(option);

    return obj->isExtensibleTool;
}

GoFx(kSize) GoToolOption_MeasurementOptionCount(GoToolOption option)
{
    GoToolOptionClass* obj = GoToolOption_Cast_(option);

    return kArrayList_Count(obj->measurementOptions);
}

GoFx(GoMeasurementOption) GoToolOption_MeasurementOptionAt(GoToolOption option, kSize index)
{
    GoToolOptionClass* obj = GoToolOption_Cast_(option);

    kAssert(index < kArrayList_Count(obj->measurementOptions));

    return *(GoMeasurementOption*)kArrayList_At(obj->measurementOptions, index);
}



kBeginClass(Go, GoTools, kObject)
    kAddVMethod(GoTools, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoTools_Construct(GoTools* tools, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoTools), tools)); 

    if (!kSuccess(status = GoTools_Init(*tools, kTypeOf(GoTools), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, tools); 
    }

    return status; 
} 

GoFx(kStatus) GoTools_Init(GoTools tools, kType type, kObject sensor, kAlloc alloc)
{
    GoToolsClass* obj = tools; 
    kStatus exception;

    kCheck(kObject_Init(tools, type, alloc)); 
    kInitFields_(GoTools, tools);

    obj->sensor = sensor; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->tools, kTypeOf(GoTool), 0, alloc));
        kTest(kArrayList_Construct(&obj->toolOptions, kTypeOf(GoToolOption), 0, alloc));
        kTest(kArrayList_Construct(&obj->nodesToMerge, kTypeOf(kXml), 0, alloc));
    }
    kCatch(&exception)
    {
        GoTools_VRelease(tools);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoTools_VRelease(GoTools tools)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 

    kCheck(kDestroyRef(&obj->nodesToMerge));
    kCheck(kDisposeRef(&obj->tools));
    kCheck(kDisposeRef(&obj->toolOptions));
    
    return kObject_VRelease(tools); 
}

GoFx(kStatus) GoTools_Read(GoTools tools, kXml xml, kXmlItem item, kXmlItem toolOptionsItem)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 
    kXmlItem currentToolItem = kNULL;
    const kChar* toolName;
    kType type;
    kStatus exception;
    kSize i;
    GoTool tool = kNULL;
    kBool insertTool = kFALSE;

    kCheck(!kIsNull(item));
    obj->xml = xml;
    kCheck(kArrayList_Clear(obj->nodesToMerge));

    //xml forward compatibility is not supported for the tools element, as it is dynamic

    for (i = 0; i < kXml_ChildCount(xml, item); i++)
    {
        tool = kNULL;
        currentToolItem = kXml_ChildAt(xml, item, i);
        toolName = kXml_ItemName(xml, currentToolItem);
        
        if (kSuccess(GoUtils_ParseToolType(toolName, &type)))
        {
            kTry
            {
                if (i >= kArrayList_Count(obj->tools)
                || kIsNull(tool = kArrayList_As_(obj->tools, i, GoTool))
                || !kObject_Is(tool, type))
                {
                    if (!kIsNull(tool))
                    {
                        kTest(kArrayList_Remove(obj->tools, i, &tool));
                        kDisposeRef(&tool);
                    }

                    kTest(GoTool_Construct(&tool, type, obj->sensor, kObject_Alloc(tools)));
                    insertTool = kTRUE;
                }

                kTest(GoTool_Read(tool, xml, currentToolItem));
                kTest(GoTool_SetId(tool, i));

                if (insertTool)
                {
                    kTest(kArrayList_Insert(obj->tools, i, &tool));
                }
            }
            kCatch(&exception)
            {
                kDisposeRef(&tool);
                kEndCatch(exception);
            }
        }
        else
        {
            kCheck(kArrayList_Add(obj->nodesToMerge, &currentToolItem));
        }
        
    }

    while (kXml_ChildCount(xml, item) < kArrayList_Count(obj->tools))
    {
        kCheck(kArrayList_Remove(obj->tools, kArrayList_Count(obj->tools) - 1, &tool));
        kDisposeRef(&tool);
    }

    return kOK; 
}

GoFx(kStatus) GoTools_Write(GoTools tools, kXml xml, kXmlItem item)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 
    kXmlItem toolItem = kNULL;
    kText256 toolName;
    kSize i;

    for (i = 0; i < GoTools_ToolCount(tools); i++)
    {
        GoTool currentTool = GoTools_ToolAt(tools, i);

        kCheck(GoUtils_FormatToolType(currentTool, toolName, kCountOf(toolName)));
        
        kCheck(kXml_AddItem(xml, item, toolName, &toolItem));
        kCheck(GoTool_Write(currentTool, xml, toolItem));
    }

    // XML merge intentionally omitted due to the dynamic nature of the Tools element. Replaced
    //  with unrecognized node insertion
    for (i = 0; i < kArrayList_Count(obj->nodesToMerge); i++)
    {
        kXmlItem mergeItem = *(kXmlItem*)kArrayList_At(obj->nodesToMerge, i);

        kCheck(kXml_CopyItem(xml, item, kNULL, obj->xml, mergeItem, kNULL));
    }    

    return kOK; 
}

GoFx(kStatus) GoTools_LegacyOptionsRead(GoTools tools, kXml xml, kXmlItem item)
{
    GoToolsClass* obj = GoTools_Cast_(tools);
    kString csvList = kNULL;
    const kChar* optionChars = kNULL;
    kSize optionNamePos = 0;
    kArrayList tempArrayList = kNULL;
    kBool addOptionToList = kFALSE;
    kSize i;
    kXmlItem toolOptionNode = kNULL;
    GoToolOption option = kNULL;

    kTry
    {
        kTest(kString_Construct(&csvList, "", kObject_Alloc(tools)));
        kTest(kXml_AttrString(xml, item, "options", csvList));
        kTest(kString_Split(csvList, ",", &tempArrayList, kObject_Alloc(tools)));

        for (i = 0; i < kArrayList_Count(tempArrayList); i++)
        {
            kString token = *(kString*)kArrayList_At(tempArrayList, i);
            GoToolOptionClass* opt = kNULL;

            option = kNULL;
            addOptionToList = kFALSE;

            if (i >= kArrayList_Count(obj->toolOptions)
                || kIsNull(option = kArrayList_As_(obj->toolOptions, i, GoToolOption))
                || !kStrEquals(GoToolOption_Name(option), kXml_ItemName(xml, toolOptionNode)))
            {
                kTest(GoToolOption_Construct(&option, kObject_Alloc(tools)));
                addOptionToList = kTRUE;
            }

            opt = GoToolOption_Cast_(option);

            if (!kStrEquals(opt->name, kXml_ItemName(xml, toolOptionNode)))
            {
                kTest(kStrCopy(opt->name, kCountOf(opt->name), kString_Chars(token)));
            }

            if (addOptionToList)
            {
                kTest(kArrayList_Add(obj->toolOptions, &option));
            }
        }

        while (kArrayList_Count(tempArrayList) < kArrayList_Count(obj->toolOptions))
        {
            kCheck(kArrayList_Remove(obj->toolOptions, kArrayList_Count(obj->toolOptions) - 1, &option));
            kDisposeRef(&option);
        }

    }
    kFinally
    {
        kDestroyRef(&csvList);
        kDisposeRef(&tempArrayList);
        kEndFinally();
    }

    return kOK;
}
GoFx(kStatus) GoTools_ReadOptions(GoTools tools, kXml xml, kXmlItem optionsItem)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 
    kXmlItem toolOptionNode = kNULL;
    kSize i, j;
    kBool addOptionToList = kFALSE;
    GoToolOption option;
    kStatus exception = kOK;

    if (kXml_AttrExists(xml, optionsItem, "options"))
    {   
        kCheck(GoTools_LegacyOptionsRead(tools, xml, optionsItem));
    }
    else
    {
        kTry
        {
            for (i = 0; i < kXml_ChildCount(xml, optionsItem); i++)
            {
                GoToolOptionClass* opt = kNULL;
                kXmlItem measurementOptions;

                addOptionToList = kFALSE;
                option = kNULL;
                toolOptionNode = kXml_ChildAt(xml, optionsItem, i);

                if (i >= kArrayList_Count(obj->toolOptions)
                    || kIsNull(option = kArrayList_As_(obj->toolOptions, i, GoToolOption))
                    || !kStrEquals(GoToolOption_Name(option), kXml_ItemName(xml, toolOptionNode)))
                {
                    if (!kIsNull(option))
                    {
                        kTest(kArrayList_Remove(obj->toolOptions, i, &option));
                        kDestroyRef(&option);
                    }

                    kTest(GoToolOption_Construct(&option, kObject_Alloc(tools)));
                    addOptionToList = kTRUE;
                }

                if (!kStrEquals(GoToolOption_Name(option), kXml_ItemName(xml, toolOptionNode)))
                {
                    opt = GoToolOption_Cast_(option);

                    kCheck(kXml_AttrBool(xml, toolOptionNode, "isCustom", &opt->isExtensibleTool));

                    kCheck(kStrCopy(opt->name, 64, kXml_ItemName(xml, toolOptionNode)));

                    if (!kIsNull(measurementOptions = kXml_Child(xml, toolOptionNode, "MeasurementOptions")))
                    {
                        kCheck(kDisposeRef(&opt->measurementOptions));
                        kCheck(kArrayList_Construct(&opt->measurementOptions, kTypeOf(GoMeasurementOption), kXml_ChildCount(xml, measurementOptions), kObject_Alloc(option)));

                        for (j = 0; j < kXml_ChildCount(xml, measurementOptions); j++)
                        {
                            kXmlItem measurementOption = kXml_ChildAt(xml, measurementOptions, j);
                            GoMeasurementOption* optionToAdd = kNULL;

                            optionToAdd = (GoMeasurementOption*)kArrayList_At(opt->measurementOptions, j);
                            
                            kCheck(kStrCopy(optionToAdd->name, kCountOf(optionToAdd->name), kXml_ItemName(xml, measurementOption)));
                            kCheck(kXml_AttrSize(xml, measurementOption, "minCount", &optionToAdd->minCount));
                            kCheck(kXml_AttrSize(xml, measurementOption, "maxCount", &optionToAdd->maxCount));
                        }
                    }
                }                
                
                if (addOptionToList)
                {
                    kCheck(kArrayList_Add(obj->toolOptions, &option));
                }
            }
        }
        kCatch(&exception)
        {
            if (addOptionToList)
            {
                kDestroyRef(&option);
            }
            kEndCatch(exception);
        }

        while (kXml_ChildCount(xml, optionsItem) < kArrayList_Count(obj->toolOptions))
        {
            kCheck(kArrayList_Remove(obj->toolOptions, kArrayList_Count(obj->toolOptions) - 1, &option));
            kDisposeRef(&option);
        }
    }    

    return kOK; 
}

GoFx(kSize) GoTools_ToolOptionCount(GoTools tools)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 

    return kArrayList_Count(obj->toolOptions);
}

GoFx(GoToolOption) GoTools_ToolOptionAt(GoTools tools, kSize index)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 

    kAssert(index < kArrayList_Count(obj->toolOptions));
    
    return *(GoToolOption*)kArrayList_At(obj->toolOptions, index);
}

GoFx(kStatus) GoTools_AddToolByName(GoTools tools, const kChar* optionName, GoTool* tool)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 
    kSize i;
    GoToolType typeVal = GoTools_StringToBuiltInDefine(optionName);
    
    for (i = 0; i < kArrayList_Count(obj->toolOptions); i++)
    {
        GoToolOption option = *(GoToolOption*)kArrayList_At(obj->toolOptions, i);

        if (kStrEquals(GoToolOption_Name(option), optionName))
        {
            kCheck(GoSensor_AddTool(obj->sensor, optionName, optionName));
            *tool = GoTools_ToolAt(tools, GoTools_ToolCount(tools) - 1);

            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoTools_ToolCount(GoTools tools)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->tools);
}

GoFx(GoTool) GoTools_ToolAt(GoTools tools, kSize index)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->tools));
    
    return kArrayList_As_(obj->tools, index, GoTool);
}

GoFx(kStatus) GoTools_AddTool(GoTools tools, GoToolType type, GoTool* tool)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 
    GoTool newTool = kNULL;
    kStatus exception;
    kType mapType;
    kSize toolListLength = kArrayList_Count(obj->tools);

    switch(type)
    {
    case GO_TOOL_RANGE_POSITION: mapType = kTypeOf(GoRangePosition); break;
    case GO_TOOL_RANGE_THICKNESS: mapType = kTypeOf(GoRangeThickness); break;
    case GO_TOOL_PROFILE_AREA: mapType = kTypeOf(GoProfileArea); break;
    case GO_TOOL_PROFILE_BOUNDING_BOX: mapType = kTypeOf(GoProfileBox); break;
    case GO_TOOL_PROFILE_CIRCLE: mapType = kTypeOf(GoProfileCircle); break;
    case GO_TOOL_PROFILE_DIMENSION: mapType = kTypeOf(GoProfileDim); break;
    case GO_TOOL_PROFILE_GROOVE: mapType = kTypeOf(GoProfileGroove);break;
    case GO_TOOL_PROFILE_INTERSECT: mapType = kTypeOf(GoProfileIntersect); break;
    case GO_TOOL_PROFILE_LINE: mapType = kTypeOf(GoProfileLine); break;
    case GO_TOOL_PROFILE_PANEL: mapType = kTypeOf(GoProfilePanel); break;
    case GO_TOOL_PROFILE_POSITION: mapType = kTypeOf(GoProfilePosition); break;
    case GO_TOOL_PROFILE_STRIP: mapType = kTypeOf(GoProfileStrip); break;
    case GO_TOOL_SURFACE_BOUNDING_BOX: mapType = kTypeOf(GoSurfaceBox); break;
    case GO_TOOL_SURFACE_COUNTERSUNK_HOLE: mapType = kTypeOf(GoSurfaceCountersunkHole); break;
    case GO_TOOL_SURFACE_ELLIPSE: mapType = kTypeOf(GoSurfaceEllipse); break;
    case GO_TOOL_SURFACE_HOLE: mapType = kTypeOf(GoSurfaceHole); break;
    case GO_TOOL_SURFACE_OPENING: mapType = kTypeOf(GoSurfaceOpening); break;
    case GO_TOOL_SURFACE_PLANE: mapType = kTypeOf(GoSurfacePlane); break;
    case GO_TOOL_SURFACE_POSITION: mapType = kTypeOf(GoSurfacePosition); break;
    case GO_TOOL_SURFACE_STUD: mapType = kTypeOf(GoSurfaceStud); break;
    case GO_TOOL_SURFACE_VOLUME: mapType = kTypeOf(GoSurfaceVolume); break;
    case GO_TOOL_SCRIPT: mapType = kTypeOf(GoScript); break;
    default: return kERROR_PARAMETER; break;
    }

    if (kSuccess(GoSensor_ReadInfo(obj->sensor)) && GoSensor_IsConfigurable(obj->sensor))
    {
        GoSensor_CacheConfig(obj->sensor);
    }

    kTry
    {
        kTest(GoTool_Construct(&newTool, mapType, obj->sensor, kObject_Alloc(tools)));

        kTest(kArrayList_Add(obj->tools, &newTool));

        kTest(GoSensor_SetConfigModified(obj->sensor));
        kTest(GoSensor_SyncConfig(obj->sensor));
    }
    kCatch(&exception)
    {
        if (toolListLength < kArrayList_Count(obj->tools))
        {
            kArrayList_Remove(obj->tools, toolListLength, kNULL);
        }

        kDestroyRef(&newTool);
        kEndCatch(kOK);
    }

    if (!kIsNull(tool))
    {
        *tool = newTool;
    }
    
    return kOK;
}

GoFx(kStatus) GoTools_RemoveTool(GoTools tools, kSize index)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 
    GoTool tool = kNULL;

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheck(kArrayList_Remove(obj->tools, index, &tool));
    kDestroyRef(&tool);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTools_ClearTools(GoTools tools)
{
    GoToolsClass* obj = GoTools_Cast_(tools); 

    kCheck(kArrayList_Purge(obj->tools));

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}


kBeginClass(Go, GoScript, GoTool)
    kAddVMethod(GoScript, kObject, VRelease)
    kAddVMethod(GoScript, GoTool, VInit)
    kAddVMethod(GoScript, GoTool, VRead)
    kAddVMethod(GoScript, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoScript_Construct(GoScript* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoScript), sensor, allocator);
}

GoFx(kStatus) GoScript_VInit(GoScript tool, kType type, kObject sensor, kAlloc alloc)
{
    GoScriptClass* obj = tool; 
    
    kCheck(GoTool_Init(tool, type, GO_TOOL_SCRIPT, sensor, alloc)); 
    kInitFields_(GoScript, tool);

    kCheck(kString_Construct(&obj->code, "", alloc));

    return kOK; 
}

GoFx(kStatus) GoScript_VRelease(GoScript tool)
{
    GoScriptClass* obj = GoScript_Cast_(tool); 

    kCheck(kDestroyRef(&obj->code));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoScript_VRead(GoScript tool, kXml xml, kXmlItem item)
{
    GoScriptClass* obj = GoScript_Cast_(tool); 

    kCheck(GoTool_VRead(tool, xml, item)); 

    kCheck(kXml_ChildString(xml, item, "Code", obj->code));

    return kOK;
}

GoFx(kStatus) GoScript_VWrite(GoScript tool, kXml xml, kXmlItem item)
{
    GoScriptClass* obj = GoScript_Cast_(tool); 

    kCheck(kXml_SetChildText(xml, item, "Name", obj->base.name));
    kCheck(kXml_SetChildText(xml, item, "Code", kString_Chars(obj->code)));
    kCheck(GoTool_VWrite(tool, xml, item));    

    return kOK;
}

GoFx(kStatus) GoScript_Code(GoScript tool, kChar** code)
{
    GoScriptClass* obj = GoScript_Cast_(tool); 
    kSize length = kString_Length(obj->code) + 1;
    kChar* output = kNULL;

    kCheck(kMemAllocZero(sizeof(kChar) * length , &output));
    kCheck(kStrCopy(output, length, kString_Chars(obj->code)));

    *code = output;

    return kOK;
}

GoFx(kStatus) GoScript_SetCode(GoScript tool, kChar* code)
{
    GoScriptClass* obj = GoScript_Cast_(tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    if (!kIsNull(code))
    {
        kCheck(kString_Set(obj->code, code));
    }
    else
    {
        kCheck(kString_Set(obj->code, ""));
    }

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoScript_AddOutput(GoScript tool, k32u id)
{
    GoScriptClass* obj = GoScript_Cast_(tool);
    GoMeasurement output = kNULL;
    kStatus exception = kOK;
    kSize i;

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        if (id == GoMeasurement_Id(GoTool_MeasurementAt(tool, i)))
        {
            return kERROR_PARAMETER;
        }
    }

    kTry
    {
        kTest(GoTool_AddMeasurement(tool, kTypeOf(GoScriptOutput), kFALSE, &output));
        kTest(GoMeasurement_SetId(output, id));
        kTest(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
        kTest(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    }
    kCatch(&exception);
    {
        kDestroyRef(&output);
        kEndCatch(exception);
    }
    

    return kOK;
}

GoFx(kStatus) GoScript_RemoveOutput(GoScript tool, k32u id)
{
    GoScriptClass* obj = GoScript_Cast_(tool);
    kSize i;
    GoMeasurement output = kNULL;

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        output = GoTool_MeasurementAt(tool, i);
        //confirm that id is present and remove from map

        if (GoMeasurement_Id(output) == id)
        {
            kCheck(GoTool_RemoveMeasurement(tool, i));
            kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
            kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoScript_OutputCount(GoScript tool)
{
    GoScriptClass* obj = GoScript_Cast_(tool);

    return GoTool_MeasurementCount(tool);
}

GoFx(GoScriptOutput) GoScript_OutputAt(GoScript tool, kSize index)
{
    GoScriptClass* obj = GoScript_Cast_(tool);

    kAssert(index < GoTool_MeasurementCount(tool));

    return GoTool_MeasurementAt(tool, index);
}


GoFx(kStatus) GoUtils_ParseToolType(const kChar* toolName, kType* type)
{
    if      (strcmp(toolName, "RangePosition") == 0)                    *type = kTypeOf(GoRangePosition);
    else if (strcmp(toolName, "RangeThickness") == 0)                   *type = kTypeOf(GoRangeThickness);

    else if (strcmp(toolName, "ProfileArea") == 0)                      *type = kTypeOf(GoProfileArea);
    else if (strcmp(toolName, "ProfileBoundingBox") == 0)               *type = kTypeOf(GoProfileBox);
    else if (strcmp(toolName, "ProfileCircle") == 0)                    *type = kTypeOf(GoProfileCircle);
    else if (strcmp(toolName, "ProfileDimension") == 0)                 *type = kTypeOf(GoProfileDim);
    else if (strcmp(toolName, "ProfileGroove") == 0)                    *type = kTypeOf(GoProfileGroove);
    else if (strcmp(toolName, "ProfileIntersect") == 0)                 *type = kTypeOf(GoProfileIntersect);
    else if (strcmp(toolName, "ProfileLine") == 0)                      *type = kTypeOf(GoProfileLine);
    else if (strcmp(toolName, "ProfilePanel") == 0)                     *type = kTypeOf(GoProfilePanel);
    else if (strcmp(toolName, "ProfilePosition") == 0)                  *type = kTypeOf(GoProfilePosition);
    else if (strcmp(toolName, "ProfileStrip") == 0)                     *type = kTypeOf(GoProfileStrip);

    else if (strcmp(toolName, "SurfaceBoundingBox") == 0)               *type = kTypeOf(GoSurfaceBox);
    else if (strcmp(toolName, "SurfaceCsHole") == 0)                    *type = kTypeOf(GoSurfaceCountersunkHole);
    else if (strcmp(toolName, "SurfaceEllipse") == 0)                   *type = kTypeOf(GoSurfaceEllipse);
    else if (strcmp(toolName, "SurfaceHole") == 0)                      *type = kTypeOf(GoSurfaceHole);
    else if (strcmp(toolName, "SurfaceOpening") == 0)                   *type = kTypeOf(GoSurfaceOpening);
    else if (strcmp(toolName, "SurfacePlane") == 0)                     *type = kTypeOf(GoSurfacePlane);
    else if (strcmp(toolName, "SurfacePosition") == 0)                  *type = kTypeOf(GoSurfacePosition);
    else if (strcmp(toolName, "SurfaceStud") == 0)                      *type = kTypeOf(GoSurfaceStud);
    else if (strcmp(toolName, "SurfaceVolume") == 0)                    *type = kTypeOf(GoSurfaceVolume);

    else if (strcmp(toolName, "Script") == 0)                           *type = kTypeOf(GoScript);
    else if (strcmp(toolName, "Custom") == 0)                           *type = kTypeOf(GoExtTool);
    else
    {
        *type = kNULL;
        return kERROR_PARAMETER;
    }

    return kOK; 
}

GoFx(kStatus) GoUtils_FormatToolType(GoTool tool, kChar* toolName, kSize capacity)
{
    const kChar* output = kNULL; 

    if      (kObject_Is(tool, kTypeOf(GoRangePosition)))              output = "RangePosition";
    else if (kObject_Is(tool, kTypeOf(GoRangeThickness)))             output = "RangeThickness";

    else if (kObject_Is(tool, kTypeOf(GoProfileArea)))                output = "ProfileArea";
    else if (kObject_Is(tool, kTypeOf(GoProfileBox)))                 output = "ProfileBoundingBox";
    else if (kObject_Is(tool, kTypeOf(GoProfileCircle)))              output = "ProfileCircle";
    else if (kObject_Is(tool, kTypeOf(GoProfileDim)))                 output = "ProfileDimension";
    else if (kObject_Is(tool, kTypeOf(GoProfileGroove)))              output = "ProfileGroove";
    else if (kObject_Is(tool, kTypeOf(GoProfileIntersect)))           output = "ProfileIntersect";
    else if (kObject_Is(tool, kTypeOf(GoProfileLine)))                output = "ProfileLine";
    else if (kObject_Is(tool, kTypeOf(GoProfilePanel)))               output = "ProfilePanel";
    else if (kObject_Is(tool, kTypeOf(GoProfilePosition)))            output = "ProfilePosition";
    else if (kObject_Is(tool, kTypeOf(GoProfileStrip)))               output = "ProfileStrip";

    else if (kObject_Is(tool, kTypeOf(GoSurfaceBox)))                 output = "SurfaceBoundingBox";
    else if (kObject_Is(tool, kTypeOf(GoSurfaceCountersunkHole)))     output = "SurfaceCsHole";
    else if (kObject_Is(tool, kTypeOf(GoSurfaceEllipse)))             output = "SurfaceEllipse";
    else if (kObject_Is(tool, kTypeOf(GoSurfaceHole)))                output = "SurfaceHole";
    else if (kObject_Is(tool, kTypeOf(GoSurfaceOpening)))             output = "SurfaceOpening";
    else if (kObject_Is(tool, kTypeOf(GoSurfacePlane)))               output = "SurfacePlane";
    else if (kObject_Is(tool, kTypeOf(GoSurfacePosition)))            output = "SurfacePosition";
    else if (kObject_Is(tool, kTypeOf(GoSurfaceStud)))                output = "SurfaceStud";
    else if (kObject_Is(tool, kTypeOf(GoSurfaceVolume)))              output = "SurfaceVolume";

    else if (kObject_Is(tool, kTypeOf(GoScript)))                     output = "Script";
    else if (kObject_Is(tool, kTypeOf(GoExtTool)))                    output = "Custom";
    
    else return kERROR_PARAMETER; 

    kCheck(kStrCopy(toolName, capacity, output));

    return kOK; 
}

GoFx(kStatus) GoTools_AssignMeasurementId(GoTools tools, GoMeasurement measurement)
{
    k32s currentId;
    kSize i, j;
    GoTool currentTool;
    GoMeasurement currentMeasurement;
    kBool isCurrentIdUsed = kFALSE;
    kMap positiveIdMap = kNULL;

    kTry
    {
        kTest(kMap_Construct(&positiveIdMap, kTypeOf(k32s), kTypeOf(kPointer), 0, kObject_Alloc(tools)));

        for (i = 0; i < GoTools_ToolCount(tools); i++)
        {
            currentTool = GoTools_ToolAt(tools, i);

            for (j = 0; j < GoTool_MeasurementCount(currentTool); j++)
            {
                currentMeasurement = GoTool_MeasurementAt(currentTool, j);

                //enabled state isn't checked due to possibility of valid IDs in measurements to be enabled later
                if (GoMeasurement_Id(currentMeasurement) > -1)
                {
                    k32s measurementId = GoMeasurement_Id(currentMeasurement);
                    
                    kMap_Add(positiveIdMap, &measurementId, &measurementId); //no test/check is done here due to the possibility of duplicate Ids
                }
            }
        }

        for (currentId = 0; currentId < k32S_MAX; currentId++)
        {
            //if currentID not used, set the measurement with it
            if (!kSuccess(kMap_Find(positiveIdMap, &currentId, kNULL)))
            {
                kTest(GoMeasurement_SetId(measurement, currentId));
                break;
            }
        }
    }
    kFinally
    {
        kDestroyRef(&positiveIdMap);
        kEndFinally();
    }

    return kOK;
}

GoFx(GoMeasurement) GoTools_FindMeasurementById(GoTool tools, k32u id)
{
    kSize i, j;
    GoTool tool = kNULL;
    GoMeasurement measurement = kNULL;

    for (i = 0; i < GoTools_ToolCount(tools); i++)
    {
        tool = GoTools_ToolAt(tools, i);

        for (j = 0; j < GoTool_MeasurementCount(tool); j++)
        {
            measurement = GoTool_MeasurementAt(tool, j);

            if (GoMeasurement_Id(measurement) == id)
                break;
        }
    }

    return measurement;
}

GoFx(const kChar*) GoTools_BuiltInToolDefineToString(GoToolType type)
{
    switch(type)
    {
    case GO_TOOL_RANGE_POSITION: return "RangePosition";
    case GO_TOOL_RANGE_THICKNESS: return "RangeThickness";         
    case GO_TOOL_PROFILE_AREA: return "ProfileArea";            
    case GO_TOOL_PROFILE_BOUNDING_BOX: return "ProfileBoundingBox";            
    case GO_TOOL_PROFILE_CIRCLE: return "ProfileCircle";          
    case GO_TOOL_PROFILE_DIMENSION: return "ProfileDimension";       
    case GO_TOOL_PROFILE_GROOVE: return "ProfileGroove";          
    case GO_TOOL_PROFILE_INTERSECT: return "ProfileIntersect";       
    case GO_TOOL_PROFILE_LINE: return "ProfileLine";            
    case GO_TOOL_PROFILE_PANEL: return "ProfilePanel";           
    case GO_TOOL_PROFILE_POSITION: return "ProfilePosition";        
    case GO_TOOL_PROFILE_STRIP: return "ProfileStrip";           
    case GO_TOOL_SURFACE_BOUNDING_BOX: return "SurfaceBoundingBox";    
    case GO_TOOL_SURFACE_COUNTERSUNK_HOLE: return "SurfaceCountersunkHole";
    case GO_TOOL_SURFACE_ELLIPSE: return "SurfaceEllipse";         
    case GO_TOOL_SURFACE_HOLE: return "SurfaceHole";            
    case GO_TOOL_SURFACE_OPENING: return "SurfaceOpening";         
    case GO_TOOL_SURFACE_PLANE: return "SurfacePlane";           
    case GO_TOOL_SURFACE_POSITION: return "SurfacePosition";        
    case GO_TOOL_SURFACE_STUD: return "SurfaceStud";            
    case GO_TOOL_SURFACE_VOLUME: return "SurfaceVolume";          
    case GO_TOOL_SCRIPT: return "SurfaceScript";                                
    case GO_TOOL_EXTENSIBLE: return "Custom";
    }   

    return kNULL;
}

GoFx(GoToolType) GoTools_StringToBuiltInDefine(const kChar* name)
{
    if (strcmp(name, "RangePosition") == 0) return  GO_TOOL_RANGE_POSITION;
    if (strcmp(name, "RangeThickness") == 0) return  GO_TOOL_RANGE_THICKNESS;          
    if (strcmp(name, "ProfileArea") == 0) return  GO_TOOL_PROFILE_AREA;             
    if (strcmp(name, "ProfileBoundingBox") == 0) return  GO_TOOL_PROFILE_BOUNDING_BOX;             
    if (strcmp(name, "ProfileCircle") == 0) return  GO_TOOL_PROFILE_CIRCLE;           
    if (strcmp(name, "ProfileDimension") == 0) return  GO_TOOL_PROFILE_DIMENSION;        
    if (strcmp(name, "ProfileGroove") == 0) return  GO_TOOL_PROFILE_GROOVE;           
    if (strcmp(name, "ProfileIntersect") == 0) return  GO_TOOL_PROFILE_INTERSECT;        
    if (strcmp(name, "ProfileLine") == 0) return  GO_TOOL_PROFILE_LINE;             
    if (strcmp(name, "ProfilePanel") == 0) return  GO_TOOL_PROFILE_PANEL;            
    if (strcmp(name, "ProfilePosition") == 0) return  GO_TOOL_PROFILE_POSITION;         
    if (strcmp(name, "ProfileStrip") == 0) return  GO_TOOL_PROFILE_STRIP;            
    if (strcmp(name, "SurfaceBoundingBox") == 0) return  GO_TOOL_SURFACE_BOUNDING_BOX;     
    if (strcmp(name, "SurfaceCountersunkHole") == 0) return  GO_TOOL_SURFACE_COUNTERSUNK_HOLE; 
    if (strcmp(name, "SurfaceEllipse") == 0) return  GO_TOOL_SURFACE_ELLIPSE;          
    if (strcmp(name, "SurfaceHole") == 0) return  GO_TOOL_SURFACE_HOLE;             
    if (strcmp(name, "SurfaceOpening") == 0) return  GO_TOOL_SURFACE_OPENING;          
    if (strcmp(name, "SurfacePlane") == 0) return  GO_TOOL_SURFACE_PLANE;            
    if (strcmp(name, "SurfacePosition") == 0) return  GO_TOOL_SURFACE_POSITION;         
    if (strcmp(name, "SurfaceStud") == 0) return  GO_TOOL_SURFACE_STUD;             
    if (strcmp(name, "SurfaceVolume") == 0) return  GO_TOOL_SURFACE_VOLUME;           
    if (strcmp(name, "SurfaceScript") == 0) return  GO_TOOL_SCRIPT;                                 
    if (strcmp(name, "Custom") == 0) return GO_TOOL_EXTENSIBLE;

    return GO_TOOL_UNKNOWN;
}


GoFx(kStatus) GoTools_AddMeasurementByName(GoTools tools, GoTool tool, const kChar* type, GoMeasurement* measurement)
{
    GoToolsClass* obj = GoTools_Cast_(tools);
    GoToolClass* toolObj = GoTool_Cast_(tool);
    GoMeasurement output = kNULL;
    kSize i;
    kBool measurementFound = kFALSE;
    GoToolOption option = kNULL;
    kText64 toolName;
    kType optionType;

    for (i = 0; i < kArrayList_Count(obj->toolOptions); i++)
    {
        optionType = kNULL;

        option = *(GoToolOption*)kArrayList_At(obj->toolOptions, i);
        
        if (!kSuccess(GoUtils_ParseToolType(GoToolOption_Name(option), &optionType)))
        {
            if (GoToolOption_IsCustom(option) && kObject_Is(tool, kTypeOf(GoExtTool)))
            { 
                kCheck(GoExtTool_Type(tool, toolName, 64));

                if (kStrEquals(GoToolOption_Name(option), toolName))
                {
                    optionType = kTypeOf(GoExtTool);
                }
            }
            
        }

        if (optionType == kObject_Type(tool))
        {
            break;
        }
    }

    kCheckArgs(!kIsNull(option) && optionType == kObject_Type(tool));

    for (i = 0; i < GoToolOption_MeasurementOptionCount(option); i++)
    {
        GoMeasurementOption measurementOption = GoToolOption_MeasurementOptionAt(option, i);

        if (kStrEquals(measurementOption.name, type))
        {
            kCheck(GoSensor_AddMeasurement(obj->sensor, toolObj->id, type));
            measurementFound = kTRUE;
            break;
        }
    }

    kCheckArgs(measurementFound);

    if (!kIsNull(measurement))
    {
        *measurement = GoTool_MeasurementAt(tool, GoTool_MeasurementCount(tool) - 1);
    }

    return kOK;
}