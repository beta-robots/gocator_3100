/** 
 * @file    GoSetup.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSetup.h>
#include <GoSdk/GoSystem.h>
#include <GoSdk/GoUtils.h>

/* 
 * GoSetupNode
 */

kBeginClass(Go, GoSetupNode, kObject)
    kAddVMethod(GoSetupNode, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSetupNode_Construct(GoSetupNode* node, kObject setup, GoRole role, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSetupNode), node)); 

    if (!kSuccess(status = GoSetupNode_Init(*node, kTypeOf(GoSetupNode), setup, role, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, node); 
    }

    return status; 
} 

GoFx(kStatus) GoSetupNode_Init(GoSetupNode node, kType type, kObject setup, GoRole role, kObject sensor, kAlloc alloc)
{
    GoSetupNodeClass* obj = node; 
    kStatus exception = kOK;

    kCheck(kObject_Init(node, type, alloc)); 
    kInitFields_(GoSetupNode, node);

    obj->role = role;
    obj->setup = setup; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->exposureModeOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->patternSequenceTypeOptions, kTypeOf(k32s), 0, alloc));
        kTest(GoMaterial_Construct(&obj->material, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoSetupNode_VRelease(node);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoSetupNode_VRelease(GoSetupNode node)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(node);

    kCheck(kDestroyRef(&obj->material));
    kCheck(kDisposeRef(&obj->exposureModeOptions));
    kCheck(kDisposeRef(&obj->patternSequenceTypeOptions));

    return kObject_VRelease(node); 
}

GoFx(GoRole) GoSetupNode_Role(GoSetupNode node)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(node);
    return obj->role;
}

GoFx(kStatus) GoSetupNode_ReadConfig(GoSetupNode node, kXml xml, kXmlItem item)
{    
    GoSetupNodeClass* obj = GoSetupNode_Cast_(node); 

    kXmlItem exposureStepItem, tempItem;
    k32u role;
    kText256 tempText;

    obj->configXml = xml;
    obj->configXmlItem = item;

    kCheck(kXml_Attr32u(xml, item, "role", &role));
    if(role == 0)
    {
        obj->role = GO_ROLE_MAIN;
    }
    else
    {
        obj->role = GO_ROLE_BUDDY;
    }

    //Active Area
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/X", &obj->activeArea.x));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Y", &obj->activeArea.y));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Z", &obj->activeArea.z));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Width", &obj->activeArea.width));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Length", &obj->activeArea.length));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Height", &obj->activeArea.height));

    //Transformed Data Region
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/X", &obj->transformedDataRegion.x));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Y", &obj->transformedDataRegion.y));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Z", &obj->transformedDataRegion.z));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Width", &obj->transformedDataRegion.width));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Length", &obj->transformedDataRegion.length));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Height", &obj->transformedDataRegion.height));

    //Front Camera
    kCheck(kXml_Child32u(xml, item, "FrontCamera/X", &obj->frontCamera.x));
    kCheck(kXml_Child32u(xml, item, "FrontCamera/Y", &obj->frontCamera.y));
    kCheck(kXml_Child32u(xml, item, "FrontCamera/Width", &obj->frontCamera.width));
    kCheck(kXml_Child32u(xml, item, "FrontCamera/Height", &obj->frontCamera.height));

    //Back Camera
    tempItem = kXml_Child(xml, item, "BackCamera");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->backCameraUsed));
    }
    else
    {
        obj->backCameraUsed = kFALSE;
    }
    kCheck(kXml_Child32u(xml, item, "BackCamera/X", &obj->backCamera.x));
    kCheck(kXml_Child32u(xml, item, "BackCamera/Y", &obj->backCamera.y));
    kCheck(kXml_Child32u(xml, item, "BackCamera/Width", &obj->backCamera.width));
    kCheck(kXml_Child32u(xml, item, "BackCamera/Height", &obj->backCamera.height));

    tempItem = kXml_Child(xml, item, "ExposureMode");
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(kArrayList_Clear(obj->exposureModeOptions));
    kCheck(GoOptionList_ParseList32u(tempText, obj->exposureModeOptions));
    kCheck(kXml_Item32u(xml, tempItem, (k32u*)&obj->exposureMode));

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Exposure", &obj->exposure));

    kCheck(kXml_Child64f(xml, item, "DynamicExposureMin", &obj->dynamicExposureMin));
    kCheck(kXml_Child64f(xml, item, "DynamicExposureMax", &obj->dynamicExposureMax));

    exposureStepItem = kXml_Child(xml, item, "ExposureSteps");
    kCheck(kXml_ItemText(xml, exposureStepItem, tempText, kCountOf(tempText)));
    kCheck(GoOptionList_Parse64f(tempText, obj->exposureSteps, kCountOf(obj->exposureSteps), &obj->exposureStepCount));

    kCheck(kXml_ChildSize(xml, item, "IntensityStepIndex", &obj->intensityStepIndex));

    tempItem = kXml_Child(xml, item, "PatternSequenceType");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->patternSequenceTypeUsed));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->patternSequenceTypeOptions));
        kCheck(kXml_Item32s(xml, tempItem, &obj->patternSequenceType));
    }    
    kCheck(GoConfig_ReadSizeOptional(xml, item, "PatternSequenceCount", 0, &obj->patternSequenceCount));

    tempItem = kXml_Child(xml, item, "XSubsampling");
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_Parse32u(tempText, obj->xSubsamplingOptions, (kSize)kCountOf(obj->xSubsamplingOptions), &obj->xSubsamplingOptionCount));
    kCheck(kXml_Item32u(xml, tempItem, &obj->xSubsampling));

    tempItem = kXml_Child(xml, item, "ZSubsampling");
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_Parse32u(tempText, obj->zSubsamplingOptions, (kSize)kCountOf(obj->zSubsamplingOptions), &obj->zSubsamplingOptionCount));
    kCheck(kXml_Item32u(xml, tempItem, &obj->zSubsampling));

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "SpacingInterval", &obj->spacingInterval));
    tempItem = kXml_Child(xml, item, "SpacingInterval");
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spacingInterval.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->spacingInterval.systemValue));

    tempItem = kXml_Child(xml, item, "SpacingIntervalType");
    kCheck(kXml_Item32u(xml, tempItem, &obj->spacingIntervalType.value));
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spacingIntervalType.enabled));
    
    kCheck(kXml_Child32u(xml, item, "XSpacingCount", &obj->xSpacingCount));
    kCheck(kXml_Child32u(xml, item, "YSpacingCount", &obj->ySpacingCount));

    tempItem = kXml_Child(xml, item, "Tracking/Enabled");
    if (kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->tracking.used));
    }
    else
    {
        obj->tracking.used = kFALSE;
    }
    
    kCheck(kXml_Child32s(xml, item, "Tracking/Enabled", &obj->tracking.enabled));
    kCheck(kXml_Child64f(xml, item, "Tracking/SearchThreshold", &obj->tracking.searchThreshold));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Tracking/Height", &obj->tracking.trackingAreaHeight));

    //Material
    if (kXml_ChildExists(xml, item, "Material"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Material")));
        kCheck(GoMaterial_Read(obj->material, xml, tempItem));
    }

    obj->hasConfig = kTRUE;

    return kOK; 
}

GoFx(kStatus) GoSetupNode_WriteConfig(GoSetupNode node, kXml xml, kXmlItem item)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(node); 
    kXmlItem activeAreaItem, trackingItem, tempItem;
    kText256 tempText;

    if(obj->role == GO_ROLE_MAIN)
    {
        kCheck(kXml_SetAttr32u(xml, item, "role", 0));
    }
    else
    {
        kCheck(kXml_SetAttr32u(xml, item, "role", 1));
    }

    kCheck(kXml_AddItem(xml, item, "ActiveArea", &activeAreaItem));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "X", obj->activeArea.x));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Y", obj->activeArea.y));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Z", obj->activeArea.z));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Width", obj->activeArea.width));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Length", obj->activeArea.length));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Height", obj->activeArea.height));

    kCheck(kXml_SetChild32s(xml, item, "ExposureMode", obj->exposureMode));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "Exposure", obj->exposure));
    kCheck(kXml_SetChild64f(xml, item, "DynamicExposureMin", obj->dynamicExposureMin));
    kCheck(kXml_SetChild64f(xml, item, "DynamicExposureMax", obj->dynamicExposureMax));
    
    kCheck(GoOptionList_Format64f(obj->exposureSteps, obj->exposureStepCount, tempText, kCountOf(tempText)));
    kCheck(kXml_SetChildText(xml, item, "ExposureSteps", tempText));
    
    kCheck(kXml_SetChildSize(xml, item, "IntensityStepIndex", obj->intensityStepIndex));

    kCheck(kXml_SetChild32s(xml, item, "PatternSequenceType", obj->patternSequenceType));

    kCheck(kXml_SetChild32u(xml, item, "XSubsampling", obj->xSubsampling));
    kCheck(kXml_SetChild32u(xml, item, "ZSubsampling", obj->zSubsampling));

    kCheck(kXml_SetChild64f(xml, item, "SpacingInterval", obj->spacingInterval.value));
    kCheck(kXml_SetChild32u(xml, item, "SpacingIntervalType", obj->spacingIntervalType.value));

    kCheck(kXml_AddItem(xml, item, "Tracking", &tempItem));
    kCheck(kXml_SetChild32u(xml, tempItem, "Enabled", obj->tracking.enabled));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "Height", obj->tracking.trackingAreaHeight));
    kCheck(kXml_SetChild64f(xml, tempItem, "SearchThreshold", obj->tracking.searchThreshold));
    trackingItem = kXml_Child(xml, item, "Tracking");

    kCheck(kXml_AddItem(xml, item, "Material", &tempItem));
    kCheck(GoMaterial_Write(obj->material, xml, tempItem));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "ActiveArea"), xml, activeAreaItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "Tracking"), xml, trackingItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "Device"), xml, item));
    kCheck(GoUtils_XmlMerge(obj->configXml, obj->configXmlItem, xml, item));

    return kOK; 
}

GoFx(kStatus) GoSetupNode_ClearConfig(GoSetupNode node)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(node); 
    obj->hasConfig = kFALSE;
    obj->configXml = kNULL;
    obj->configXmlItem = kNULL;
    return kOK; 
}


/* 
 * GoSetup
 */

kBeginClass(Go, GoSetup, kObject)
    kAddVMethod(GoSetup, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSetup_Construct(GoSetup* setup, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSetup), setup)); 

    if (!kSuccess(status = GoSetup_Init(*setup, kTypeOf(GoSetup), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, setup); 
    }

    return status; 
} 

GoFx(kStatus) GoSetup_Init(GoSetup setup, kType type, kObject sensor, kAlloc alloc)
{
    GoSetupClass* obj = setup; 
    kStatus status; 

    kCheck(kObject_Init(setup, type, alloc)); 
    kCheck(kInitFields_(GoSetup, setup));
    
    obj->sensor = sensor; 

    obj->scanMode = GO_MODE_UNKNOWN;

    obj->trigger.source = GO_TRIGGER_TIME;
    obj->trigger.delay.value = 0;
    obj->trigger.delay.min = 0;
    obj->trigger.delay.max = 0;
    obj->trigger.gateEnabled = kFALSE;

    obj->trigger.maxFrameRateEnabled = kFALSE;
    obj->trigger.frameRate.value = 0;
    obj->trigger.frameRate.min = 0;
    obj->trigger.frameRate.max = 0;
    //Could put frameRateLimitMaxSource in here, it's in the XML

    obj->trigger.encoderTriggerMode = GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE;
    obj->trigger.encoderSpacing.value = 0;
    obj->trigger.encoderSpacing.min = 0;
    obj->trigger.encoderSpacing.max = 0;
    //Could put trigger.encoderSpacing.minSource in here, it's in the XML

    obj->intensityEnabled = kFALSE;

    obj->alignment.stationaryTarget = GO_ALIGNMENT_TARGET_NONE;
    obj->alignment.movingTarget = GO_ALIGNMENT_TARGET_DISK;
    obj->alignment.diskDiameter = 0;
    obj->alignment.diskHeight = 0;
    obj->alignment.barWidth = 0;
    obj->alignment.barHeight = 0;
    obj->alignment.barHoleCount = 0;
    obj->alignment.barHoleDistance = 0;
    obj->alignment.barHoleDiameter = 0;

    obj->filters.xSmoothing.enabled = kFALSE;
    obj->filters.xSmoothing.value = 0;
    obj->filters.xSmoothing.max = 0;
    obj->filters.xSmoothing.min = 0;

    obj->filters.ySmoothing.enabled = kFALSE;
    obj->filters.ySmoothing.value = 0;
    obj->filters.ySmoothing.max = 0;
    obj->filters.ySmoothing.min = 0;

    obj->filters.xGapFilling.enabled = kFALSE;
    obj->filters.xGapFilling.value = 0;
    obj->filters.xGapFilling.max = 0;
    obj->filters.xGapFilling.min = 0;

    obj->filters.yGapFilling.enabled = kFALSE;
    obj->filters.yGapFilling.value = 0;
    obj->filters.yGapFilling.max = 0;
    obj->filters.yGapFilling.min = 0;

    kTry
    {   
        kTest(kArrayList_Construct(&obj->nodes, kTypeOf(GoSetupNode), 0, alloc));
        kTest(kArrayList_Construct(&obj->scanModeOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.stationaryTargetOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.movingTargetOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.typeOptions, kTypeOf(k32s), 0, alloc));
        kTest(GoLayout_Construct(&obj->layout, sensor, alloc));
        kTest(GoProfileGeneration_Construct(&obj->profileGeneration, sensor, alloc));
        kTest(GoSurfaceGeneration_Construct(&obj->surfaceGeneration, sensor, alloc)); 
        kTest(GoPartDetection_Construct(&obj->partDetection, sensor, alloc)); 
        kTest(GoPartMatching_Construct(&obj->partMatching, sensor, alloc)); 
    }
    kCatch(&status)
    {
        GoSetup_VRelease(setup); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSetup_VRelease(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheck(kDestroyRef(&obj->partMatching)); 
    kCheck(kDestroyRef(&obj->partDetection)); 
    kCheck(kDestroyRef(&obj->profileGeneration));
    kCheck(kDestroyRef(&obj->surfaceGeneration)); 
    kCheck(kDestroyRef(&obj->layout));
    kCheck(kDisposeRef(&obj->alignment.stationaryTargetOptions));
    kCheck(kDisposeRef(&obj->alignment.movingTargetOptions));
    kCheck(kDisposeRef(&obj->alignment.typeOptions));
    kCheck(kDisposeRef(&obj->scanModeOptions));
    kCheck(kDisposeRef(&obj->nodes)); 
    
    kCheck(kObject_VRelease(setup)); 

    return kOK; 
}

GoFx(kStatus) GoSetup_ReadConfig(GoSetup setup, kXml xml, kXmlItem item)
{
    GoSetupClass* obj = GoSetup_Cast_(setup); 
    kText128 tempText;
    kXmlItem devicesItem = kNULL;
    kXmlItem deviceItem = kNULL;
    kXmlItem tempItem = kNULL;
    k32u tempInt;
    GoRole sensorRole;
    GoSetupNode node;
    k32u i;

    obj->configXml = xml;
    obj->configXmlItem = item;

    // System settings
    kCheck(kXml_ChildBool(xml, item, "TemperatureSafetyEnabled", &obj->temperatureSafetyEnabled));
    tempItem = kXml_Child(xml, item, "TemperatureSafetyEnabled");
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->temperatureSafetyUsed));

    // Mode and datatype selection
    tempItem = kXml_Child(xml, item, "ScanMode");
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(kArrayList_Clear(obj->scanModeOptions));
    kCheck(GoOptionList_ParseList32u(tempText, obj->scanModeOptions));
    kCheck(kXml_Item32u(xml, tempItem, (k32u*)&obj->scanMode));
    
    tempItem = kXml_Child(xml, item, "UniformSpacingEnabled");
    kCheck(kXml_ItemBool(xml, tempItem, &obj->uniformSpacingEnabled));    
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->uniformSpacingAvail));
    kCheck(kXml_AttrBool(xml, tempItem, "value", &obj->uniformSpacingEnabledSystemValue));

    tempItem = kXml_Child(xml, item, "IntensityEnabled");
    kCheck(kXml_ItemBool(xml, tempItem, &obj->intensityEnabled));    
    kCheck(GoConfig_ReadAttrBoolOptional(xml, tempItem, "used", kFALSE, &obj->intensityAvail));
    kCheck(GoConfig_ReadAttrBoolOptional(xml, tempItem, "value", kFALSE, &obj->intensityEnabledSystemValue));

    kCheck(kXml_ChildBool(xml, item, "OcclusionReductionEnabled", &obj->occlusionReductionEnabled));
    kCheck(GoConfig_ReadBoolOptional(xml, item, "ExternalInputZPulseEnabled", kFALSE, &obj->externalInputZPulseEnabled));

    //Filters
    kCheck(kXml_ChildBool(xml, item, "Filters/XGapFilling/Enabled", &obj->filters.xGapFilling.enabled));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Filters/XGapFilling/Window", &obj->filters.xGapFilling));

    kCheck(kXml_ChildBool(xml, item, "Filters/YGapFilling/Enabled", &obj->filters.yGapFilling.enabled));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Filters/YGapFilling/Window", &obj->filters.yGapFilling));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "Filters/XMedian/Enabled", kFALSE, &obj->filters.xMedian.enabled));
    kCheck(GoConfig_ReadRangeElement64fOptional(xml, item, "Filters/XMedian/Window", &obj->filters.xMedian));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "Filters/YMedian/Enabled", kFALSE, &obj->filters.yMedian.enabled));
    kCheck(GoConfig_ReadRangeElement64fOptional(xml, item, "Filters/YMedian/Window", &obj->filters.yMedian));

    kCheck(kXml_ChildBool(xml, item, "Filters/XSmoothing/Enabled", &obj->filters.xSmoothing.enabled));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Filters/XSmoothing/Window", &obj->filters.xSmoothing));

    kCheck(kXml_ChildBool(xml, item, "Filters/YSmoothing/Enabled", &obj->filters.ySmoothing.enabled));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Filters/YSmoothing/Window", &obj->filters.ySmoothing));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "Filters/XDecimation/Enabled", kFALSE, &obj->filters.xDecimation.enabled));
    kCheck(GoConfig_ReadRangeElement64fOptional(xml, item, "Filters/XDecimation/Window", &obj->filters.xDecimation));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "Filters/YDecimation/Enabled", kFALSE, &obj->filters.yDecimation.enabled));
    kCheck(GoConfig_ReadRangeElement64fOptional(xml, item, "Filters/YDecimation/Window", &obj->filters.yDecimation));
    
    //Trigger
    kCheck(kXml_Child32s(xml, item, "Trigger/Source", (k32s*) &obj->trigger.source));
    kCheck(kXml_Child32s(xml, item, "Trigger/Units", (k32s*) &obj->trigger.unit));

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Trigger/FrameRate", &obj->trigger.frameRate));
    tempItem = kXml_Child(xml, item, "Trigger/FrameRate");
    kCheck(kXml_Attr32s(xml, tempItem, "maxSource", (k32s*) &obj->trigger.frameRateMaxSource));
    kCheck(kXml_ChildBool(xml, item, "Trigger/MaxFrameRateEnabled", &obj->trigger.maxFrameRateEnabled));

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Trigger/EncoderSpacing", &obj->trigger.encoderSpacing));
    tempItem = kXml_Child(xml, item, "Trigger/EncoderSpacing");
    kCheck(kXml_Attr32s(xml, tempItem, "minSource", (k32s*) &obj->trigger.encoderSpacingMinSource));

    kCheck(kXml_Child32s(xml, item, "Trigger/EncoderTriggerMode", (k32s*) &obj->trigger.encoderTriggerMode));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Trigger/Delay", &obj->trigger.delay));
    tempItem = kXml_Child(xml, item, "Trigger/GateEnabled");
    kCheck(kXml_Item32s(xml, tempItem, &obj->trigger.gateEnabled));
    kCheck(kXml_Attr32s(xml, tempItem, "used", &obj->trigger.gateEnabledUsed));
    kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->trigger.gateEnabledSystemValue));
    
    //Layout
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Layout")));
    kCheck(GoLayout_Read(obj->layout, xml, tempItem));

    //Alignment
    kCheck(kXml_ChildBool(xml, item, "Alignment/InputTriggerEnabled", &obj->alignment.inputTriggerEnabled));
    
    tempItem = kXml_Child(xml, item, "Alignment/Type");
    kCheck(kXml_Item32s(xml, tempItem, (k32s*)&obj->alignment.type));
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.typeOptions));

    tempItem = kXml_Child(xml, item, "Alignment/StationaryTarget");
    kCheck(kXml_Item32s(xml, tempItem, (k32s*)&obj->alignment.stationaryTarget));
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.stationaryTargetOptions));

    tempItem = kXml_Child(xml, item, "Alignment/MovingTarget");
    kCheck(kXml_Item32s(xml, tempItem, (k32s*)&obj->alignment.movingTarget));
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.movingTargetOptions));

    kCheck(kXml_ChildBool(xml, item, "Alignment/EncoderCalibrateEnabled", &obj->alignment.encoderCalibrateEnabled));

    kCheck(kXml_Child64f(xml, item, "Alignment/Disk/Diameter", &obj->alignment.diskDiameter));
    kCheck(kXml_Child64f(xml, item, "Alignment/Disk/Height", &obj->alignment.diskHeight));
    kCheck(kXml_Child64f(xml, item, "Alignment/Bar/Width", &obj->alignment.barWidth));
    kCheck(kXml_Child64f(xml, item, "Alignment/Bar/Height", &obj->alignment.barHeight));
    kCheck(kXml_ChildSize(xml, item, "Alignment/Bar/HoleCount", &obj->alignment.barHoleCount));
    kCheck(kXml_Child64f(xml, item, "Alignment/Bar/HoleDistance", &obj->alignment.barHoleDistance));
    kCheck(kXml_Child64f(xml, item, "Alignment/Bar/HoleDiameter", &obj->alignment.barHoleDiameter));
    kCheck(kXml_Child64f(xml, item, "Alignment/Plate/Height", &obj->alignment.plateHeight));
    kCheck(kXml_ChildSize(xml, item, "Alignment/Plate/HoleCount", &obj->alignment.plateHoleCount));
    kCheck(kXml_Child64f(xml, item, "Alignment/Plate/RefHoleDiameter", &obj->alignment.plateRefHoleDiameter));
    kCheck(kXml_Child64f(xml, item, "Alignment/Plate/SecHoleDiameter", &obj->alignment.plateSecHoleDiameter));

    for (i = 0; i < kArrayList_Count(obj->nodes); ++i)
    {
        kCheck(GoSetupNode_ClearConfig(kArrayList_As_(obj->nodes, i, GoSetupNode)));
    }

    kCheck(!kIsNull(devicesItem = kXml_Child(xml, item, "Devices")));
    deviceItem = kXml_FirstChild(xml, devicesItem);

    while (!kIsNull(deviceItem))
    {
        kCheck(kXml_Attr32u(xml, deviceItem, "role", &tempInt));

        if(tempInt == 0)
        {
            sensorRole = GO_ROLE_MAIN;
        }
        else
        {
            sensorRole = GO_ROLE_BUDDY;
        }
        node = GoSetup_FindNode(setup, sensorRole);

        if (kIsNull(node))
        {
            kCheck(GoSetupNode_Construct(&node, setup, sensorRole, obj->sensor, kObject_Alloc_(setup)));
            kCheck(kArrayList_Add(obj->nodes, &node));
        }

        kCheck(GoSetupNode_ReadConfig(node, xml, deviceItem));

        deviceItem = kXml_NextSibling(xml, deviceItem);
    }

    if (kXml_ChildExists(xml, item, "ProfileGeneration"))
    {
        kCheck(GoProfileGeneration_Read(obj->profileGeneration, xml, kXml_Child(xml, item, "ProfileGeneration")));
    }
    
    kCheck(GoSurfaceGeneration_Read(obj->surfaceGeneration, xml, kXml_Child(xml, item, "SurfaceGeneration")));
    kCheck(GoPartDetection_Read(obj->partDetection, xml, kXml_Child(xml, item, "PartDetection")));
    
    return kOK;
}

GoFx(kStatus) GoSetup_WriteConfig(GoSetup setup, kXml xml, kXmlItem item)
{
    GoSetupClass* obj = GoSetup_Cast_(setup); 
    kXmlItem setupItem = kNULL;
    kXmlItem devicesItem = kNULL;
    kXmlItem deviceItem = kNULL;
    kXmlItem alignmentItem = kNULL;
    kXmlItem tempItem = kNULL;
    GoSetupNode node = kNULL;
    k32u i;
    kText256 text;

    kCheck(kXml_SetChildBool(xml, item, "TemperatureSafetyEnabled", obj->temperatureSafetyEnabled));

    kCheck(kXml_SetChild32u(xml, item, "ScanMode", obj->scanMode));
    kCheck(kXml_SetChildBool(xml, item, "UniformSpacingEnabled", obj->uniformSpacingEnabled));
    kCheck(kXml_SetChildBool(xml, item, "IntensityEnabled", obj->intensityEnabled));

    kCheck(kXml_SetChildBool(xml, item, "OcclusionReductionEnabled", obj->occlusionReductionEnabled));
    kCheck(kXml_SetChildBool(xml, item, "ExternalInputZPulseEnabled", obj->externalInputZPulseEnabled));

    kCheck(kXml_SetChild32s(xml, item, "Filters/XGapFilling/Enabled", obj->filters.xGapFilling.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XGapFilling/Window", obj->filters.xGapFilling.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YGapFilling/Enabled", obj->filters.yGapFilling.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YGapFilling/Window", obj->filters.yGapFilling.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XMedian/Enabled", obj->filters.xMedian.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XMedian/Window", obj->filters.xMedian.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YMedian/Enabled", obj->filters.yMedian.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YMedian/Window", obj->filters.yMedian.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XSmoothing/Enabled", obj->filters.xSmoothing.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XSmoothing/Window", obj->filters.xSmoothing.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YSmoothing/Enabled", obj->filters.ySmoothing.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YSmoothing/Window", obj->filters.ySmoothing.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XDecimation/Enabled", obj->filters.xDecimation.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XDecimation/Window", obj->filters.xDecimation.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YDecimation/Enabled", obj->filters.yDecimation.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YDecimation/Window", obj->filters.yDecimation.value));

    kCheck(kXml_SetChild32s(xml, item, "Trigger/Source", obj->trigger.source));
    kCheck(kXml_SetChild32u(xml, item, "Trigger/Units", obj->trigger.unit));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/FrameRate", obj->trigger.frameRate.value));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/MaxFrameRateEnabled", obj->trigger.maxFrameRateEnabled));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/EncoderSpacing", obj->trigger.encoderSpacing.value));
    kCheck(kXml_SetChild32s(xml, item, "Trigger/EncoderTriggerMode", obj->trigger.encoderTriggerMode));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/Delay", obj->trigger.delay.value));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/GateEnabled", obj->trigger.gateEnabled));

    //tranformed data region read only?
    kCheck(kXml_AddItem(xml, item, "Layout", &tempItem));
    kCheck(GoLayout_Write(obj->layout, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "Alignment", &alignmentItem));
    kCheck(kXml_SetChildBool(xml, alignmentItem, "InputTriggerEnabled", obj->alignment.inputTriggerEnabled));

    kCheck(kXml_AddItem(xml, alignmentItem, "Type", &tempItem));
    kCheck(GoOptionList_Format32u((k32u*)kArrayList_Data(obj->alignment.typeOptions), kArrayList_Count(obj->alignment.typeOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.type));

    kCheck(kXml_AddItem(xml, alignmentItem, "StationaryTarget", &tempItem));
    kCheck(GoOptionList_Format32u((k32u*)kArrayList_Data(obj->alignment.stationaryTargetOptions), kArrayList_Count(obj->alignment.stationaryTargetOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.stationaryTarget));

    kCheck(kXml_AddItem(xml, alignmentItem, "MovingTarget", &tempItem));
    kCheck(GoOptionList_Format32u((k32u*)kArrayList_Data(obj->alignment.movingTargetOptions), kArrayList_Count(obj->alignment.movingTargetOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.movingTarget));

    kCheck(kXml_SetChildBool(xml, alignmentItem, "EncoderCalibrateEnabled", obj->alignment.encoderCalibrateEnabled));
    
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Disk/Diameter", obj->alignment.diskDiameter));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Disk/Height", obj->alignment.diskHeight));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Bar/Width", obj->alignment.barWidth));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Bar/Height", obj->alignment.barHeight));
    kCheck(kXml_SetChildSize(xml, alignmentItem, "Bar/HoleCount", obj->alignment.barHoleCount));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Bar/HoleDistance", obj->alignment.barHoleDistance));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Bar/HoleDiameter", obj->alignment.barHoleDiameter));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Plate/Height", obj->alignment.plateHeight));
    kCheck(kXml_SetChildSize(xml, alignmentItem, "Plate/HoleCount", obj->alignment.plateHoleCount));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Plate/RefHoleDiameter", obj->alignment.plateRefHoleDiameter));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Plate/SecHoleDiameter", obj->alignment.plateSecHoleDiameter));

    kCheck(kXml_AddItem(xml, item, "Devices", &devicesItem));

    for (i = 0; i < kArrayList_Count(obj->nodes); i++)
    {
        node = kArrayList_As_(obj->nodes, i, GoSetupNode);

        kCheck(kXml_AddItem(xml, devicesItem, "Device", &deviceItem));
        kCheck(kXml_SetAttr32u(xml, deviceItem, "role", i));
        
        kCheck(GoSetupNode_WriteConfig(node, xml, deviceItem));
    }

    kCheck(kXml_AddItem(xml, item, "ProfileGeneration", &tempItem));
    kCheck(GoProfileGeneration_Write(obj->profileGeneration, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "SurfaceGeneration", &tempItem));
    kCheck(GoSurfaceGeneration_Write(obj->surfaceGeneration, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "PartDetection", &tempItem));
    kCheck(GoPartDetection_Write(obj->partDetection, xml, tempItem));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->configXml, obj->configXmlItem, xml, item));

    return kOK;
}

GoFx(GoSetupNode) GoSetup_FindNode(GoSetup setup, GoRole role)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);
    GoSetupNode* node;
    k32u i;

    for (i = 0; i < kArrayList_Count(obj->nodes); ++i)
    {
        node = kArrayList_At(obj->nodes, i);

        if (GoSetupNode_Role(*node) == role)
        {
            return *node;
        }
    }

    return kNULL;
}


GoFx(kStatus) GoSetup_EnableTemperatureSafety( GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    
    obj->temperatureSafetyEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK;
}

GoFx(kBool) GoSetup_TemperatureSafetyEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->temperatureSafetyEnabled;
}


GoFx(kBool) GoSetup_TemperatureSafetyValueUsed(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->temperatureSafetyUsed;
}


GoFx(kStatus) GoSetup_SetScanMode( GoSetup setup, GoMode mode )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->scanModeOptions), 
        kArrayList_Count(obj->scanModeOptions), 
        mode));

    obj->scanMode = mode;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK;
}

GoFx(GoMode) GoSetup_ScanMode( GoSetup setup )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->scanMode;
}

GoFx(kSize) GoSetup_ScanModeOptionCount(GoSetup setup)
{
   GoSetupClass* obj = GoSetup_Cast_(setup);

   GoSensor_SyncConfig(obj->sensor);

   return kArrayList_Count(obj->scanModeOptions);
}

GoFx(GoMode) GoSetup_ScanModeOptionAt(GoSetup setup, kSize index)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_ScanModeOptionCount(setup));

    return kArrayList_As_(obj->scanModeOptions, index, GoMode);
}

GoFx(kBool) GoSetup_UniformSpacingEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->uniformSpacingEnabled;
}

GoFx(kBool) GoSetup_UniformSpacingEnabledSystemValue(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->uniformSpacingEnabledSystemValue;
}

GoFx(kStatus) GoSetup_EnableUniformSpacing(GoSetup setup, kBool enable)
{
   GoSetupClass* obj = GoSetup_Cast_(setup);

   kCheckState(GoSensor_IsConfigurable(obj->sensor));
   kCheck(GoSensor_CacheConfig(obj->sensor));

   obj->uniformSpacingEnabled = enable;
   kCheck(GoSensor_SetConfigModified(obj->sensor));
   
   return kOK;
}

GoFx(kBool) GoSetup_UniformSpacingAvailable(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    
    return obj->uniformSpacingAvail;
}

GoFx(kBool) GoSetup_OcclusionReductionEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->occlusionReductionEnabled;
}

GoFx(kStatus) GoSetup_EnableOcclusionReduction(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->occlusionReductionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_SetTriggerUnit( GoSetup setup, GoTriggerUnits unit )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.unit = unit;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(GoTriggerUnits) GoSetup_TriggerUnit(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.unit;
}

GoFx(kStatus) GoSetup_SetTriggerSource(GoSetup setup, GoTrigger source)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.source = source;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(GoTrigger) GoSetup_TriggerSource(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.source;
}

GoFx(kStatus) GoSetup_SetTriggerDelay(GoSetup setup, k64f delay)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(delay, GoSetup_TriggerDelayLimitMin(setup), GoSetup_TriggerDelayLimitMax(setup)));

    obj->trigger.delay.value = delay;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_TriggerDelay(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.delay.value;
}

GoFx(k64f) GoSetup_TriggerDelayLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);
    
    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.delay.min;
}

GoFx(k64f) GoSetup_TriggerDelayLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);
    
    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.delay.max;
}

GoFx(kStatus) GoSetup_EnableTriggerGate(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.gateEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(kBool) GoSetup_TriggerGateEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.gateEnabled;
}

GoFx(kBool) GoSetup_TriggerGateEnabledSystemValue(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.gateEnabledSystemValue;
}

GoFx(kBool) GoSetup_TriggerGateEnabledUsed(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.gateEnabledUsed;
}

GoFx(kStatus) GoSetup_EnableMaxFrameRate(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.maxFrameRateEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(kBool) GoSetup_MaxFrameRateEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.maxFrameRateEnabled;
}

GoFx(kStatus) GoSetup_SetFrameRate(GoSetup setup, k64f frameRate)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(frameRate, GoSetup_FrameRateLimitMin(setup), GoSetup_FrameRateLimitMax(setup)));

    obj->trigger.frameRate.value = frameRate;
    kCheck(GoSensor_SetConfigModified(obj->sensor));    

    return kOK; 
}

GoFx(k64f) GoSetup_FrameRate(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.frameRate.value;
}

GoFx(k64f) GoSetup_FrameRateLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.frameRate.min;
}

GoFx(k64f) GoSetup_FrameRateLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);
    
    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.frameRate.max;
}

GoFx(kStatus) GoSetup_SetEncoderTriggerMode( GoSetup setup, GoEncoderTriggerMode mode )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.encoderTriggerMode = mode;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    
    return kOK; 
}

GoFx(GoEncoderTriggerMode) GoSetup_EncoderTriggerMode( GoSetup setup )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.encoderTriggerMode;
}

GoFx(kStatus) GoSetup_SetEncoderSpacing(GoSetup setup, k64f period)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(period, GoSetup_EncoderSpacingLimitMin(setup), GoSetup_EncoderSpacingLimitMax(setup)));

    obj->trigger.encoderSpacing.value = period;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    
    return kOK; 
}

GoFx(k64f) GoSetup_EncoderSpacing(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.encoderSpacing.value;
}

GoFx(k64f) GoSetup_EncoderSpacingLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.encoderSpacing.min;
}

GoFx(k64f) GoSetup_EncoderSpacingLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.encoderSpacing.max;
}

GoFx(kStatus) GoSetup_EnableIntensity(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->intensityEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    
    return kOK; 
}

GoFx(kBool) GoSetup_IntensityEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->intensityEnabled;
}

GoFx(kBool) GoSetup_IntensityEnabledUsed(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->intensityAvail;
}

GoFx(kBool) GoSetup_IntensityEnabledSystemValue(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->intensityEnabledSystemValue;
}

GoFx(kStatus) GoSetup_SetAlignmentType( GoSetup setup, GoAlignmentType type)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->alignment.typeOptions), 
                            kArrayList_Count(obj->alignment.typeOptions), 
                            type));

    obj->alignment.type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(GoAlignmentType) GoSetup_AlignmentType( GoSetup setup )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.type;
}

GoFx(kSize) GoSetup_AlignmentTypeOptionCount(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.typeOptions);
}

GoFx(GoAlignmentType) GoSetup_AlignmentTypeOptionAt(GoSetup setup, kSize index)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_AlignmentTypeOptionCount(setup));

    return kArrayList_As_(obj->alignment.typeOptions, index, GoAlignmentType);
}

GoFx(kStatus) GoSetup_EnableAlignmentEncoderCalibrate(GoSetup setup, kBool enabled)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.encoderCalibrateEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(kBool) GoSetup_AlignmentEncoderCalibrateEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.encoderCalibrateEnabled;
}

GoFx(kBool) GoSetup_InputTriggerEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.inputTriggerEnabled;
}

GoFx(kStatus) GoSetup_EnableInputTrigger(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.inputTriggerEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}


GoFx(kStatus) GoSetup_SetAlignmentStationaryTarget( GoSetup setup, GoAlignmentTarget target )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->alignment.stationaryTargetOptions), 
                                kArrayList_Count(obj->alignment.stationaryTargetOptions), 
                                target));

    obj->alignment.stationaryTarget = target;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentStationaryTarget( GoSetup setup )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.stationaryTarget;
}

GoFx(kSize) GoSetup_AlignmentStationaryTargetOptionCount(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.stationaryTargetOptions);
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentStationaryTargetOptionAt(GoSetup setup, kSize index)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_AlignmentStationaryTargetOptionCount(setup));

    return kArrayList_As_(obj->alignment.stationaryTargetOptions, index, GoAlignmentTarget);
}


GoFx(kStatus) GoSetup_SetAlignmentMovingTarget( GoSetup setup, GoAlignmentTarget target )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->alignment.movingTargetOptions), 
        kArrayList_Count(obj->alignment.movingTargetOptions), 
        target));

    obj->alignment.movingTarget = target;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentMovingTarget( GoSetup setup )
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.movingTarget;
}

GoFx(kSize) GoSetup_AlignmentMovingTargetOptionCount(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.movingTargetOptions);
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentMovingTargetOptionAt(GoSetup setup, kSize index)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_AlignmentMovingTargetOptionCount(setup));

    return kArrayList_As_(obj->alignment.movingTargetOptions, index, GoAlignmentTarget);
}

GoFx(kStatus) GoSetup_SetDiskDiameter(GoSetup setup, k64f diameter)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.diskDiameter = diameter;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_DiskDiameter(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.diskDiameter;
}

GoFx(kStatus) GoSetup_SetDiskHeight(GoSetup setup, k64f height)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.diskHeight = height;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_DiskHeight(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.diskHeight;
}

GoFx(kStatus) GoSetup_SetBarWidth(GoSetup setup, k64f width)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barWidth = width;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_BarWidth(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barWidth;
}

GoFx(kStatus) GoSetup_SetBarHeight(GoSetup setup, k64f height)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barHeight = height;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_BarHeight(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHeight;
}

GoFx(kStatus) GoSetup_SetBarHoleCount(GoSetup setup, kSize count)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barHoleCount = count;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(kSize) GoSetup_BarHoleCount(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleCount;
}

GoFx(kStatus) GoSetup_SetBarHoleDistance(GoSetup setup, k64f distance)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barHoleDistance = distance;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_BarHoleDistance(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleDistance;
}

GoFx(kStatus) GoSetup_SetBarHoleDiameter(GoSetup setup, k64f diameter)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barHoleDiameter = diameter;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_BarHoleDiameter(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleDiameter;
}


GoFx(kStatus) GoSetup_SetPlateHeight(GoSetup setup, k64f height)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateHeight = height;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_PlateHeight(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateHeight;
}

GoFx(kStatus) GoSetup_SetPlateHoleCount(GoSetup setup, kSize count)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateHoleCount = count;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(kSize) GoSetup_PlateHoleCount(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateHoleCount;
}

GoFx(kStatus) GoSetup_SetPlateRefHoleDiameter(GoSetup setup, k64f diameter)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateRefHoleDiameter = diameter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_PlateRefHoleDiameter(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateRefHoleDiameter;
}

GoFx(kStatus) GoSetup_SetPlateSecHoleDiameter(GoSetup setup, k64f diameter)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateSecHoleDiameter = diameter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_PlateSecHoleDiameter(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateSecHoleDiameter;
}


GoFx(kStatus) GoSetup_EnableXSmoothing(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xSmoothing.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_XSmoothingEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSmoothing.enabled;
}

GoFx(kStatus) GoSetup_SetXSmoothingWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XSmoothingWindowLimitMin(setup), GoSetup_XSmoothingWindowLimitMax(setup)));

    obj->filters.xSmoothing.value = window;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_XSmoothingWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSmoothing.value;
}

GoFx(k64f) GoSetup_XSmoothingWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xSmoothing.min;
}

GoFx(k64f) GoSetup_XSmoothingWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xSmoothing.max;
}

GoFx(kStatus) GoSetup_EnableYSmoothing(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.ySmoothing.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_YSmoothingEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySmoothing.enabled;
}

GoFx(kStatus) GoSetup_SetYSmoothingWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YSmoothingWindowLimitMin(setup), GoSetup_YSmoothingWindowLimitMax(setup)));

    obj->filters.ySmoothing.value = window;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_YSmoothingWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySmoothing.value;
}

GoFx(k64f) GoSetup_YSmoothingWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.ySmoothing.min;
}

GoFx(k64f) GoSetup_YSmoothingWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.ySmoothing.max;
}

GoFx(kStatus) GoSetup_EnableXMedian(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xMedian.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_XMedianEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xMedian.enabled;
}

GoFx(kStatus) GoSetup_SetXMedianWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XMedianWindowLimitMin(setup), GoSetup_XMedianWindowLimitMax(setup)));

    obj->filters.xMedian.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_XMedianWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xMedian.value;
}

GoFx(k64f) GoSetup_XMedianWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xMedian.min;
}

GoFx(k64f) GoSetup_XMedianWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xMedian.max;
}

GoFx(kStatus) GoSetup_EnableYMedian(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.yMedian.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_YMedianEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yMedian.enabled;
}

GoFx(kStatus) GoSetup_SetYMedianWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YMedianWindowLimitMin(setup), GoSetup_YMedianWindowLimitMax(setup)));

    obj->filters.yMedian.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_YMedianWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yMedian.value;
}

GoFx(k64f) GoSetup_YMedianWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yMedian.min;
}

GoFx(k64f) GoSetup_YMedianWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yMedian.max;
}

GoFx(kStatus) GoSetup_EnableXDecimation(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xDecimation.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_XDecimationEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xDecimation.enabled;
}

GoFx(kStatus) GoSetup_SetXDecimationWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XDecimationWindowLimitMin(setup), GoSetup_XDecimationWindowLimitMax(setup)));

    obj->filters.xDecimation.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_XDecimationWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xDecimation.value;
}

GoFx(k64f) GoSetup_XDecimationWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xDecimation.min;
}

GoFx(k64f) GoSetup_XDecimationWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xDecimation.max;
}

GoFx(kStatus) GoSetup_EnableYDecimation(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.yDecimation.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_YDecimationEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yDecimation.enabled;
}

GoFx(kStatus) GoSetup_SetYDecimationWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YDecimationWindowLimitMin(setup), GoSetup_YDecimationWindowLimitMax(setup)));

    obj->filters.yDecimation.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_YDecimationWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yDecimation.value;
}

GoFx(k64f) GoSetup_YDecimationWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yDecimation.min;
}

GoFx(k64f) GoSetup_YDecimationWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yDecimation.max;
}


GoFx(kStatus) GoSetup_EnableXGapFilling(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xGapFilling.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_XGapFillingEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xGapFilling.enabled;
}

GoFx(kStatus) GoSetup_SetXGapFillingWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XGapFillingWindowLimitMin(setup), GoSetup_XGapFillingWindowLimitMax(setup)));

    obj->filters.xGapFilling.value = window;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_XGapFillingWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xGapFilling.value;
}

GoFx(k64f) GoSetup_XGapFillingWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xGapFilling.min;
}

GoFx(k64f) GoSetup_XGapFillingWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xGapFilling.max;
}

GoFx(kStatus) GoSetup_EnableYGapFilling(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.yGapFilling.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoSetup_YGapFillingEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yGapFilling.enabled;
}

GoFx(kStatus) GoSetup_SetYGapFillingWindow(GoSetup setup, k64f window)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));    
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YGapFillingWindowLimitMin(setup), GoSetup_YGapFillingWindowLimitMax(setup)));

    obj->filters.yGapFilling.value = window;
    
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSetup_YGapFillingWindow(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yGapFilling.value;
}

GoFx(k64f) GoSetup_YGapFillingWindowLimitMin(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yGapFilling.min;
}

GoFx(k64f) GoSetup_YGapFillingWindowLimitMax(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yGapFilling.max;
}


//*****  ROLE SPECIFIC API  *****//


GoFx(k64f) GoSetup_ExposureLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->exposure.max;
}

GoFx(k64f) GoSetup_ExposureLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->exposure.min;
}

GoFx(kStatus) GoSetup_SetExposure(GoSetup setup, GoRole role, k64f exposure)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);
    
    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));

    obj->exposure.value = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));
    
    return kOK;
}

GoFx(k64f) GoSetup_Exposure(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->exposure.value;
}

GoFx(GoExposureMode) GoSetup_ExposureMode(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->exposureMode;
}

GoFx(kStatus) GoSetup_SetExposureMode(GoSetup setup, GoRole role, GoExposureMode mode)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);
    
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->exposureModeOptions), 
        kArrayList_Count(obj->exposureModeOptions), 
        mode));

    obj->exposureMode = mode;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kSize) GoSetup_ExposureModeOptionCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return kArrayList_Count(obj->exposureModeOptions);
}

GoFx(GoExposureMode) GoSetup_ExposureModeOptionAt(GoSetup setup, GoRole role, kSize index)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_ExposureModeOptionCount(setup, role));

    return kArrayList_As_(obj->exposureModeOptions, index, GoExposureMode);
}

GoFx(kStatus) GoSetup_AddExposureStep(GoSetup setup, GoRole role, k64f exposure)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);
    kSize exposureStepCount = obj->exposureStepCount;   //the limit function calls below will trigger a config read

    if(exposureStepCount < GO_MAX_EXPOSURE_COUNT)
    {
        kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));
    }
    else
    {
        return kERROR;
    }

    obj->exposureSteps[exposureStepCount] = exposure;
    obj->exposureStepCount = exposureStepCount + 1;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_ClearExposureSteps(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    obj->exposureStepCount = 0;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ExposureStepAt(GoSetup setup, GoRole role, kSize index)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_ExposureStepCount(setup, role));

    return obj->exposureSteps[index];
}

GoFx(kSize) GoSetup_ExposureStepCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->exposureStepCount;
}

GoFx(k64f) GoSetup_DynamicExposureMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->dynamicExposureMax;
}

GoFx(kStatus) GoSetup_SetDynamicExposureMax(GoSetup setup, GoRole role, k64f exposure)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));

    obj->dynamicExposureMax = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_DynamicExposureMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->dynamicExposureMin;
}

GoFx(kStatus) GoSetup_SetDynamicExposureMin(GoSetup setup, GoRole role, k64f exposure)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));

    obj->dynamicExposureMin = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_SetIntensityStepIndex(GoSetup setup, GoRole role, kSize stepIndex)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    kCheckState(stepIndex < GoSetup_ExposureStepCount(setup, role));

    obj->intensityStepIndex = stepIndex;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kSize) GoSetup_IntensityStepIndex(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->intensityStepIndex;
}

GoFx(kStatus) GoSetup_SetPatternSequenceType(GoSetup setup, GoRole role, GoPatternSequenceType type)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->patternSequenceTypeOptions), 
        kArrayList_Count(obj->patternSequenceTypeOptions), 
        type));

    obj->patternSequenceType = type;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(GoPatternSequenceType) GoSetup_PatternSequenceType(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceType;
}

GoFx(kBool) GoSetup_PatternSequenceTypeUsed(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceTypeUsed;
}

GoFx(kSize) GoSetup_PatternSequenceTypeOptionCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return kArrayList_Count(obj->patternSequenceTypeOptions);
}

GoFx(GoPatternSequenceType) GoSetup_PatternSequenceTypeOptionAt(GoSetup setup, GoRole role, kSize index)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_PatternSequenceTypeOptionCount(setup, role));

    return kArrayList_As_(obj->patternSequenceTypeOptions, index, GoPatternSequenceType);
}


GoFx(kSize) GoSetup_PatternSequenceCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceCount;
}


//transformed data region read only functions?
GoFx(k64f) GoSetup_TransformedDataRegionX(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.x;
}

GoFx(k64f) GoSetup_TransformedDataRegionY(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.y;
}

GoFx(k64f) GoSetup_TransformedDataRegionZ(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.z;
}

GoFx(k64f) GoSetup_TransformedDataRegionWidth(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.width;
}

GoFx(k64f) GoSetup_TransformedDataRegionLength(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.length;
}

GoFx(k64f) GoSetup_TransformedDataRegionHeight(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.height;
}

GoFx(k64f) GoSetup_ActiveAreaHeightLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.height.min;
}

GoFx(k64f) GoSetup_ActiveAreaHeightLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.height.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaHeight(GoSetup setup, GoRole role, k64f height)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(height, GoSetup_ActiveAreaHeightLimitMin(setup, role), GoSetup_ActiveAreaHeightLimitMax(setup, role)));

    obj->activeArea.height.value = height;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));    

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaHeight(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.height.value;
}

GoFx(k64f) GoSetup_ActiveAreaWidthLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.width.min;
}

GoFx(k64f) GoSetup_ActiveAreaWidthLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.width.max;
}

GoFx(k64f) GoSetup_SetActiveAreaWidth(GoSetup setup, GoRole role, k64f width)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(width, GoSetup_ActiveAreaWidthLimitMin(setup, role), GoSetup_ActiveAreaWidthLimitMax(setup, role)));

    obj->activeArea.width.value = width;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));    

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaWidth(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.width.value;
}


GoFx(k64f) GoSetup_ActiveAreaLengthLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.length.min;
}

GoFx(k64f) GoSetup_ActiveAreaLengthLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.length.max;
}

GoFx(k64f) GoSetup_SetActiveAreaLength(GoSetup setup, GoRole role, k64f height)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(height, GoSetup_ActiveAreaLengthLimitMin(setup, role), GoSetup_ActiveAreaLengthLimitMax(setup, role)));

    obj->activeArea.length.value = height;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));    

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaLength(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.length.value;
}

GoFx(k64f) GoSetup_ActiveAreaXLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.x.min;
}

GoFx(k64f) GoSetup_ActiveAreaXLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.x.max;
}

GoFx(k64f) GoSetup_SetActiveAreaX(GoSetup setup, GoRole role, k64f x)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(x, GoSetup_ActiveAreaXLimitMin(setup, role), GoSetup_ActiveAreaXLimitMax(setup, role)));

    obj->activeArea.x.value = x;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));    

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaX(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.x.value;
}


GoFx(k64f) GoSetup_ActiveAreaYLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.y.min;
}

GoFx(k64f) GoSetup_ActiveAreaYLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.y.max;
}

GoFx(k64f) GoSetup_SetActiveAreaY(GoSetup setup, GoRole role, k64f z)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(z, GoSetup_ActiveAreaYLimitMin(setup, role), GoSetup_ActiveAreaYLimitMax(setup, role)));

    obj->activeArea.y.value = z;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));    

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaY(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.y.value;
}


GoFx(k64f) GoSetup_ActiveAreaZLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.z.min;
}

GoFx(k64f) GoSetup_ActiveAreaZLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.z.max;
}

GoFx(k64f) GoSetup_SetActiveAreaZ(GoSetup setup, GoRole role, k64f z)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(z, GoSetup_ActiveAreaZLimitMin(setup, role), GoSetup_ActiveAreaZLimitMax(setup, role)));

    obj->activeArea.z.value = z;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));    

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaZ(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.z.value;
}


GoFx(kSize) GoSetup_XSubsamplingOptionCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->xSubsamplingOptionCount;
}

GoFx(k32u) GoSetup_XSubsamplingOptionAt(GoSetup setup, GoRole role, kSize index)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kCheckArgs(index < obj->xSubsamplingOptionCount);

    return obj->xSubsamplingOptions[index];
}

GoFx(kStatus) GoSetup_SetXSubsampling( GoSetup setup, GoRole role, k32u xSubsampling )
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);
    GoSensor_SyncConfig(setupObj->sensor);
    kCheck(GoOptionList_Check32u(obj->xSubsamplingOptions, obj->xSubsamplingOptionCount, xSubsampling));

    obj->xSubsampling = xSubsampling;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_XSubsampling(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->xSubsampling;
}

GoFx(kSize) GoSetup_ZSubsamplingOptionCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->zSubsamplingOptionCount;
}

GoFx(k32u) GoSetup_ZSubsamplingOptionAt(GoSetup setup, GoRole role, kSize index)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kCheckArgs(index < obj->zSubsamplingOptionCount);

    return obj->zSubsamplingOptions[index];
}

GoFx(kStatus) GoSetup_SetZSubsampling(GoSetup setup, GoRole role, k32u zSubsampling)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);
    GoSensor_SyncConfig(setupObj->sensor);
    kCheck(GoOptionList_Check32u(obj->zSubsamplingOptions, obj->zSubsamplingOptionCount, zSubsampling));

    obj->zSubsampling = zSubsampling;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_ZSubsampling(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->zSubsampling;
}

GoFx(k64f) GoSetup_SpacingIntervalSystemValue(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.systemValue;
}

GoFx(k64f) GoSetup_SpacingInterval(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.value;
}

GoFx(kBool) GoSetup_SpacingIntervalUsed(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.enabled;
}

GoFx(kStatus) GoSetup_SetSpacingInterval(GoSetup setup, GoRole role, k64f value)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    kCheckArgs(GoUtils_MinMax_(value, GoSetup_SpacingIntervalLimitMin(setup, role), GoSetup_SpacingIntervalLimitMax(setup, role)));

    obj->spacingInterval.value = value;

    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_SpacingIntervalLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.min;
}

GoFx(k64f) GoSetup_SpacingIntervalLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.max;
}

GoFx(GoSpacingIntervalType) GoSetup_SpacingIntervalType(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingIntervalType.value;
}

GoFx(kBool) GoSetup_SpacingIntervalTypeUsed(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingIntervalType.enabled;
}

GoFx(kStatus) GoSetup_SetSpacingIntervalType(GoSetup setup, GoRole role, GoSpacingIntervalType type)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    obj->spacingIntervalType.value = type;

    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_XSpacingCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->xSpacingCount;
}

GoFx(k32u) GoSetup_YSpacingCount(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->ySpacingCount;
}

GoFx(k32u) GoSetup_FrontCameraX(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->frontCamera.x;
}

GoFx(k32u) GoSetup_FrontCameraY(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
        
    return obj->frontCamera.y;
}

GoFx(k32u) GoSetup_FrontCameraWidth(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->frontCamera.width;
}

GoFx(k32u) GoSetup_FrontCameraHeight(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->frontCamera.height;
}


GoFx(k32u) GoSetup_BackCameraX(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->backCamera.x;
}

GoFx(k32u) GoSetup_BackCameraY(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->backCamera.y;
}

GoFx(k32u) GoSetup_BackCameraWidth(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->backCamera.width;
}

GoFx(k32u) GoSetup_BackCameraHeight(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    
    return obj->backCamera.height;
}

GoFx(k32u) GoSetup_BackCameraUsed(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->backCameraUsed;
}

GoFx(kStatus) GoSetup_EnableTracking(GoSetup setup, GoRole role, kBool enable)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    obj->tracking.enabled = enable;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_TrackingEnabled(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.enabled;
}

GoFx(kBool) GoSetup_TrackingUsed(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.used;
}

GoFx(kStatus) GoSetup_SetTrackingAreaHeight(GoSetup setup, GoRole role, k64f height)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    setupObj = obj->setup;    
    kCheckArgs(GoUtils_MinMax_(height, GoSetup_TrackingAreaHeightLimitMin(setup, role), GoSetup_TrackingAreaHeightLimitMax(setup, role)));

    obj->tracking.trackingAreaHeight.value = height;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_TrackingAreaHeight(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.trackingAreaHeight.value;
}

GoFx(k64f) GoSetup_TrackingAreaHeightLimitMin(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->tracking.trackingAreaHeight.min;
}

GoFx(k64f) GoSetup_TrackingAreaHeightLimitMax(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->tracking.trackingAreaHeight.max;
}

GoFx(kStatus) GoSetup_SetTrackingSearchThreshold(GoSetup setup, GoRole role, k64f threshold)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);
    kCheckArgs(GoUtils_MinMax_(threshold, 0, 100));

    obj->tracking.searchThreshold = threshold;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_TrackingSearchThreshold(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.searchThreshold;
}

GoFx(GoLayout) GoSetup_Layout(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    return obj->layout;
}

GoFx(GoProfileGeneration) GoSetup_ProfileGeneration(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    return obj->profileGeneration;
}

GoFx(GoSurfaceGeneration) GoSetup_SurfaceGeneration(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup); 

    return obj->surfaceGeneration; 
}

GoFx(GoPartDetection) GoSetup_PartDetection(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup); 

    return obj->partDetection; 
}

GoFx(GoPartMatching) GoSetup_PartMatching(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup); 

    return obj->partMatching; 
}

GoFx(kStatus) GoSetup_EnableExternalInputZPulse(GoSetup setup, kBool enable)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    obj->externalInputZPulseEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_ExternalInputZPulseEnabled(GoSetup setup)
{
    GoSetupClass* obj = GoSetup_Cast_(setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->externalInputZPulseEnabled;
}

GoFx(GoMaterial) GoSetup_Material(GoSetup setup, GoRole role)
{
    GoSetupNodeClass* obj = GoSetupNode_Cast_(GoSetup_FindNode(setup, role));
    GoSetupClass* setupObj = GoSetup_Cast_(obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->material;
}
