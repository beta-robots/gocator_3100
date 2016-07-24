/** 
 * @file    GoEthernet.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoEthernet.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoEthernet, kObject)
    kAddVMethod(GoEthernet, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoEthernet_Construct(GoEthernet* ethernet, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoEthernet), ethernet)); 

    if (!kSuccess(status = GoEthernet_Init(*ethernet, kTypeOf(GoEthernet), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, ethernet); 
    }

    return status; 
} 

GoFx(kStatus) GoEthernet_Init(GoEthernet ethernet, kType type, kObject sensor, kAlloc alloc)
{
    GoEthernetClass* obj = ethernet; 
    kStatus exception;

    kCheck(kObject_Init(ethernet, type, alloc)); 
    kInitFields_(GoEthernet, ethernet);

    obj->sensor = sensor; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->videoOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->videoSources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->rangeOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->rangeSources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->rangeIntensityOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->rangeIntensitySources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->profileOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->profileSources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->profileIntensityOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->profileIntensitySources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->surfaceOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->surfaceSources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->surfaceIntensityOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->surfaceIntensitySources, kTypeOf(k32u), 0, alloc)); 

        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->measurementSources, kTypeOf(k32u), 0, alloc)); 

        kTest(kString_Construct(&obj->ascii.customFormat, "", alloc));
        kTest(kString_Construct(&obj->ascii.delimiter, "", alloc));
        kTest(kString_Construct(&obj->ascii.invalidValue, "", alloc));
        kTest(kString_Construct(&obj->ascii.terminator, "", alloc));
    }
    kCatch(&exception)
    {
        GoEthernet_VRelease(ethernet);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoEthernet_VRelease(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet); 

    kCheck(kDisposeRef(&obj->videoOptions)); 
    kCheck(kDisposeRef(&obj->videoSources)); 

    kCheck(kDisposeRef(&obj->rangeOptions)); 
    kCheck(kDisposeRef(&obj->rangeSources)); 

    kCheck(kDisposeRef(&obj->rangeIntensityOptions)); 
    kCheck(kDisposeRef(&obj->rangeIntensitySources)); 

    kCheck(kDisposeRef(&obj->profileOptions)); 
    kCheck(kDisposeRef(&obj->profileSources)); 

    kCheck(kDisposeRef(&obj->profileIntensityOptions)); 
    kCheck(kDisposeRef(&obj->profileIntensitySources)); 

    kCheck(kDisposeRef(&obj->surfaceOptions)); 
    kCheck(kDisposeRef(&obj->surfaceSources)); 

    kCheck(kDisposeRef(&obj->surfaceIntensityOptions)); 
    kCheck(kDisposeRef(&obj->surfaceIntensitySources)); 

    kCheck(kDisposeRef(&obj->measurementOptions)); 
    kCheck(kDisposeRef(&obj->measurementSources)); 

    kCheck(kDestroyRef(&obj->ascii.customFormat));
    kCheck(kDestroyRef(&obj->ascii.delimiter));
    kCheck(kDestroyRef(&obj->ascii.invalidValue));
    kCheck(kDestroyRef(&obj->ascii.terminator));

    return kObject_VRelease(ethernet);
}

GoFx(kStatus) GoEthernet_Read(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kXmlItem tempItem = kNULL;
    kChar* text = kNULL; 
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(kObject_GetMemZero(ethernet, sizeof(kChar) * textCapacity, &text)); 

        kTest(kXml_Child32s(xml, item, "Protocol", (k32s*)&obj->protocol)); 

        tempItem = kXml_Child(xml, item, "Videos");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->videoOptions));  

        kTest(kXml_ChildText(xml, item, "Videos", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->videoSources));  

        tempItem = kXml_Child(xml, item, "Ranges");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->rangeOptions));  

        kTest(kXml_ChildText(xml, item, "Ranges", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->rangeSources));  

        tempItem = kXml_Child(xml, item, "RangeIntensities");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->rangeIntensityOptions));  

        kTest(kXml_ChildText(xml, item, "RangeIntensities", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->rangeIntensitySources));  

        tempItem = kXml_Child(xml, item, "Profiles");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->profileOptions));  

        kTest(kXml_ChildText(xml, item, "Profiles", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->profileSources));  

        tempItem = kXml_Child(xml, item, "ProfileIntensities");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->profileIntensityOptions));  

        kTest(kXml_ChildText(xml, item, "ProfileIntensities", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->profileIntensitySources));  

        tempItem = kXml_Child(xml, item, "Surfaces");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->surfaceOptions));  

        kTest(kXml_ChildText(xml, item, "Surfaces", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->surfaceSources));  

        tempItem = kXml_Child(xml, item, "SurfaceIntensities");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->surfaceIntensityOptions));  

        kTest(kXml_ChildText(xml, item, "SurfaceIntensities", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->surfaceIntensitySources));  

        tempItem = kXml_Child(xml, item, "Measurements");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementOptions));  

        kTest(kXml_ChildText(xml, item, "Measurements", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementSources));  

        tempItem = kXml_Child(xml, item, "EIP");
        kTest(kXml_ChildBool(xml, tempItem, "BufferEnabled", &obj->eip.bufferEnabled));
        kTest(GoConfig_Read32sOptional(xml, tempItem, "EndianOutputType", kFALSE, &obj->eip.endianOutputType));
        kTest(GoConfig_ReadBoolOptional(xml, tempItem, "ImplicitOutputEnabled", kFALSE, &obj->eip.implicitOutputEnabled));
        kTest(GoConfig_Read32sOptional(xml, tempItem, "ImplicitTriggerOverride", kFALSE, &obj->eip.implicitTriggerOverride));

        tempItem = kXml_Child(xml, item, "Modbus");
        kTest(kXml_ChildBool(xml, tempItem, "BufferEnabled", &obj->modbus.bufferEnabled));

        //ASCII config
        tempItem = kXml_Child(xml, item, "Ascii");
        kTest(kXml_Child32u(xml, tempItem, "Operation", (k32u*)&obj->ascii.operation));
        kTest(kXml_Child32u(xml, tempItem, "ControlPort", &obj->ascii.controlPort));
        kTest(kXml_Child32u(xml, tempItem, "DataPort", &obj->ascii.dataPort));
        kTest(kXml_Child32u(xml, tempItem, "HealthPort", &obj->ascii.healthPort));
        kTest(kXml_ChildString(xml, tempItem, "Delimiter", obj->ascii.delimiter));
        kTest(kXml_ChildString(xml, tempItem, "Terminator", obj->ascii.terminator));
        kTest(kXml_ChildString(xml, tempItem, "InvalidValue", obj->ascii.invalidValue));
        kTest(kXml_ChildString(xml, tempItem, "CustomDataFormat", obj->ascii.customFormat));
        kTest(kXml_ChildBool(xml, tempItem, "CustomFormatEnabled", &obj->ascii.customFormatEnabled));
    }
    kFinally
    {
        kObject_FreeMem(ethernet, text); 
        kEndFinally(); 
    }

    return kOK; 
}

GoFx(kStatus) GoEthernet_Write(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kChar* text = kNULL; 
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 
    kXmlItem tempItem = kNULL;

    kTry
    {
        kTest(kObject_GetMemZero(ethernet, sizeof(kChar) * textCapacity, &text)); 

        kTest(kXml_SetChild32s(xml, item, "Protocol", obj->protocol)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->videoSources), kArrayList_Count(obj->videoSources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Videos", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->rangeSources), kArrayList_Count(obj->rangeSources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Ranges", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->rangeIntensitySources), kArrayList_Count(obj->rangeIntensitySources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "RangeIntensities", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->profileSources), kArrayList_Count(obj->profileSources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Profiles", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->profileIntensitySources), kArrayList_Count(obj->profileIntensitySources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "ProfileIntensities", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->surfaceSources), kArrayList_Count(obj->surfaceSources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Surfaces", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->surfaceIntensitySources), kArrayList_Count(obj->surfaceIntensitySources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "SurfaceIntensities", text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->measurementSources), kArrayList_Count(obj->measurementSources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Measurements", text));

        //ASCII config
        kTest(kXml_AddItem(xml, item, "Ascii", &tempItem));
        kTest(kXml_SetChild32s(xml, tempItem, "Operation", obj->ascii.operation));
        kTest(kXml_SetChild32u(xml, tempItem, "ControlPort", obj->ascii.controlPort));
        kTest(kXml_SetChild32u(xml, tempItem, "DataPort", obj->ascii.dataPort));
        kTest(kXml_SetChild32u(xml, tempItem, "HealthPort", obj->ascii.healthPort));
        kTest(kXml_SetChildText(xml, tempItem, "Delimiter", kString_Chars(obj->ascii.delimiter)));
        kTest(kXml_SetChildText(xml, tempItem, "Terminator", kString_Chars(obj->ascii.terminator)));
        kTest(kXml_SetChildText(xml, tempItem, "InvalidValue", kString_Chars(obj->ascii.invalidValue)));
        kTest(kXml_SetChildText(xml, tempItem, "CustomDataFormat", kString_Chars(obj->ascii.customFormat)));
        kTest(kXml_SetChildBool(xml, tempItem, "CustomFormatEnabled", obj->ascii.customFormatEnabled));

        kTest(kXml_AddItem(xml, item, "EIP", &tempItem));
        kTest(kXml_SetChildBool(xml, tempItem, "BufferEnabled", obj->eip.bufferEnabled));
        kTest(kXml_SetChild32s(xml, tempItem, "EndianOutputType", obj->eip.endianOutputType));
        kTest(kXml_SetChildBool(xml, tempItem, "ImplicitOutputEnabled", obj->eip.implicitOutputEnabled));
        kTest(kXml_SetChild32s(xml, tempItem, "ImplicitTriggerOverride", obj->eip.implicitTriggerOverride));

        kTest(kXml_AddItem(xml, item, "Modbus", &tempItem));
        kTest(kXml_SetChildBool(xml, tempItem, "BufferEnabled", obj->modbus.bufferEnabled));
    }
    kFinally
    {
        kObject_FreeMem(ethernet, text); 
        kEndFinally(); 
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kArrayList) GoEthernet_OptionList(GoEthernet ethernet, GoOutputSource type)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list = kNULL;

    GoSensor_SyncConfig(obj->sensor);

    switch (type)
    {
    case GO_OUTPUT_SOURCE_RANGE:                           list = obj->rangeOptions;             break;
    case GO_OUTPUT_SOURCE_RANGE_INTENSITY:                 list = obj->rangeIntensityOptions;    break;
    case GO_OUTPUT_SOURCE_PROFILE:                         list = obj->profileOptions;             break;
    case GO_OUTPUT_SOURCE_PROFILE_INTENSITY:               list = obj->profileIntensityOptions;    break;
    case GO_OUTPUT_SOURCE_SURFACE:                         list = obj->surfaceOptions;                   break;
    case GO_OUTPUT_SOURCE_SURFACE_INTENSITY:               list = obj->surfaceIntensityOptions;          break;
    case GO_OUTPUT_SOURCE_VIDEO:                           list = obj->videoOptions;                  break;
    case GO_OUTPUT_SOURCE_MEASUREMENT:                     list = obj->measurementOptions;                  break;
    }

    return list;
}

GoFx(kArrayList) GoEthernet_SourceList(GoEthernet ethernet, GoOutputSource type)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list = kNULL;

    GoSensor_SyncConfig(obj->sensor);

    switch (type)
    {
        case GO_OUTPUT_SOURCE_RANGE:                         list = obj->rangeSources;             break;
        case GO_OUTPUT_SOURCE_RANGE_INTENSITY:               list = obj->rangeIntensitySources;    break;
        case GO_OUTPUT_SOURCE_PROFILE:                       list = obj->profileSources;             break;
        case GO_OUTPUT_SOURCE_PROFILE_INTENSITY:             list = obj->profileIntensitySources;    break;
        case GO_OUTPUT_SOURCE_SURFACE:                       list = obj->surfaceSources;                   break;
        case GO_OUTPUT_SOURCE_SURFACE_INTENSITY:             list = obj->surfaceIntensitySources;          break;
        case GO_OUTPUT_SOURCE_VIDEO:                         list = obj->videoSources;                  break;
        case GO_OUTPUT_SOURCE_MEASUREMENT:                   list = obj->measurementSources;                  break;
    }

    return list;
}

GoFx(kSize) GoEthernet_OptionCount(GoEthernet ethernet, GoOutputSource type)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_OptionList(ethernet, type);
    
    if (kIsNull(list))
    {
        return 0;
    }

    return kArrayList_Count(list);
}

GoFx(k32u) GoEthernet_OptionAt(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_OptionList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    kAssert(index < kArrayList_Count(list));

    return kArrayList_As_(list, index, k32u);
}

GoFx(kSize) GoEthernet_SourceCount(GoEthernet ethernet, GoOutputSource type)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);
    
    list = GoEthernet_SourceList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    return kArrayList_Count(list);
}

GoFx(k32u) GoEthernet_SourceAt(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_SourceList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    kAssert(index < kArrayList_Count(list));

    return kArrayList_As_(list, index, k32u);
}

GoFx(kStatus) GoEthernet_AddSource(GoEthernet ethernet, GoOutputSource type, k32u sourceId)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list = kNULL;
    kArrayList optionList = kNULL;

    kCheck(GoSensor_SyncConfig(obj->sensor));

    switch (type)
    {
        case GO_OUTPUT_SOURCE_RANGE:
            list = obj->rangeSources;
            optionList = obj->rangeOptions; 
            break;
        case GO_OUTPUT_SOURCE_RANGE_INTENSITY:
            list = obj->rangeIntensitySources;
            optionList = obj->rangeIntensityOptions; 
            break;
        case GO_OUTPUT_SOURCE_PROFILE:
            list = obj->profileSources;
            optionList = obj->profileOptions; 
            break;
        case GO_OUTPUT_SOURCE_PROFILE_INTENSITY:
            list = obj->profileIntensitySources;
            optionList = obj->profileIntensityOptions; 
            break;
        case GO_OUTPUT_SOURCE_SURFACE:
            list = obj->surfaceSources;
            optionList = obj->surfaceOptions; 
            break;
        case GO_OUTPUT_SOURCE_SURFACE_INTENSITY:
            list = obj->surfaceIntensitySources;
            optionList = obj->surfaceIntensityOptions; 
            break;
        case GO_OUTPUT_SOURCE_VIDEO:
            list = obj->videoSources;
            optionList = obj->videoOptions; 
            break;
        case GO_OUTPUT_SOURCE_MEASUREMENT:
            list = obj->measurementSources;
            optionList = obj->measurementOptions; 
            break;
        default:
            return kERROR_PARAMETER; 
    }

    kCheck(GoOptionList_Check32u(kArrayList_Data(optionList), kArrayList_Count(optionList), sourceId));
    kCheck(kArrayList_Add(list, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_RemoveSource(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list;
    
    list = GoEthernet_SourceList(ethernet, type);
    if (kIsNull(list))
    {
        return kERROR_PARAMETER;
    }

    kCheck(kArrayList_Remove(list, index, kNULL));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_ClearAllSources(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_VIDEO));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_RANGE));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_PROFILE));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_SURFACE));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_RANGE_INTENSITY));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_PROFILE_INTENSITY));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_SURFACE_INTENSITY));
    kCheck(GoEthernet_ClearSources(ethernet, GO_OUTPUT_SOURCE_MEASUREMENT));

    GoSensor_SyncConfig(obj->sensor);

    return kOK;
}

GoFx(kStatus) GoEthernet_ClearSources(GoEthernet ethernet, GoOutputSource type)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    kArrayList list;

    list = GoEthernet_SourceList(ethernet, type);
    if (kIsNull(list))
    {
        return kERROR_PARAMETER;
    }

    kCheck(kArrayList_Clear(list));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoEthernetProtocol) GoEthernet_Protocol(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->protocol; 
}

GoFx(kStatus) GoEthernet_SetProtocol(GoEthernet ethernet, GoEthernetProtocol protocol)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);
    
    obj->protocol = protocol; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(GoAsciiOperation) GoEthernet_AsciiOperation(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.operation; 
}

GoFx(kStatus) GoEthernet_SetAsciiOperation(GoEthernet ethernet, GoAsciiOperation mode)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->ascii.operation = mode; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(k32u) GoEthernet_AsciiControlPort(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.controlPort; 
}

GoFx(kStatus) GoEthernet_SetAsciiControlPort(GoEthernet ethernet, k32u port)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->ascii.controlPort = port; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(k32u) GoEthernet_AsciiHealthPort(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.healthPort; 
}

GoFx(kStatus) GoEthernet_SetAsciiHealthPort(GoEthernet ethernet, k32u port)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->ascii.healthPort = port; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(k32u) GoEthernet_AsciiDataPort(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.dataPort; 
}

GoFx(kStatus) GoEthernet_SetAsciiDataPort(GoEthernet ethernet, k32u port)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->ascii.dataPort = port; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kChar*) GoEthernet_AsciiDelimiter(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.delimiter); 
}

GoFx(kStatus) GoEthernet_SetAsciiDelimiter(GoEthernet ethernet, const kChar* string)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    kCheck(kString_Set(obj->ascii.delimiter, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kChar*) GoEthernet_AsciiTerminator(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.terminator); 
}

GoFx(kStatus) GoEthernet_SetAsciiTerminator(GoEthernet ethernet, const kChar* string)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    kCheck(kString_Set(obj->ascii.terminator, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kChar*) GoEthernet_AsciiInvalidValue(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.invalidValue); 
}

GoFx(kStatus) GoEthernet_SetAsciiInvalidValue(GoEthernet ethernet, const kChar* string)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    kCheck(kString_Set(obj->ascii.invalidValue, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kChar*) GoEthernet_AsciiCustomDataFormat(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.customFormat); 
}

GoFx(kStatus) GoEthernet_SetAsciiCustomDataFormat(GoEthernet ethernet, const kChar* string)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    kCheck(kString_Set(obj->ascii.customFormat, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kStatus) GoEthernet_EnableAsciiCustomFormat(GoEthernet ethernet, kBool enabled)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->ascii.customFormatEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoEthernet_AsciiCustomFormatEnabled(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.customFormatEnabled;
}

GoFx(kStatus) GoEthernet_SetEIPBufferingEnabled(GoEthernet ethernet, kBool enabled)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->eip.bufferEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoEthernet_EIPBufferingEnabled(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.bufferEnabled;
}

GoFx(kStatus) GoEthernet_SetEIPImplicitOutputEnabled(GoEthernet ethernet, kBool enabled)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->eip.implicitOutputEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoEthernet_EIPImplicitOutputEnabled(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.implicitOutputEnabled;
}

GoFx(kStatus) GoEthernet_SetEIPEndianOutputType(GoEthernet ethernet, GoEndianType type)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->eip.endianOutputType = type; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(GoEndianType) GoEthernet_EIPEndianOutputType(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.endianOutputType;
}

GoFx(kStatus) GoEthernet_SetEIPImplicitTriggerOverride(GoEthernet ethernet, GoImplicitTriggerOverride value)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->eip.implicitTriggerOverride = value; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(GoImplicitTriggerOverride) GoEthernet_EIPImplicitTriggerOverride(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.implicitTriggerOverride;
}


GoFx(kStatus) GoEthernet_SetModbusBufferingEnabled(GoEthernet ethernet, kBool enabled)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    obj->modbus.bufferEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(kBool) GoEthernet_ModbusBufferingEnabled(GoEthernet ethernet)
{
    GoEthernetClass* obj = GoEthernet_Cast_(ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->modbus.bufferEnabled; 
}
