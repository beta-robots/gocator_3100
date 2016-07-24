/** 
 * @file    GoSerial.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoSerial.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoSerial, kObject)
    kAddVMethod(GoSerial, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSerial_Construct(GoSerial* serial, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSerial), serial)); 

    if (!kSuccess(status = GoSerial_Init(*serial, kTypeOf(GoSerial), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, serial); 
    }

    return status; 
} 

GoFx(kStatus) GoSerial_Init(GoSerial serial, kType type, kObject sensor, kAlloc alloc)
{
    GoSerialClass* obj = serial; 
    kStatus exception;

    kCheck(kObject_Init(serial, type, alloc)); 
    kInitFields_(GoSerial, serial);

    obj->sensor = sensor; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->protocolOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->measurementSources, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->selcom.formatOptions, kTypeOf(k32s), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->selcom.rateOptions, kTypeOf(k32s), 0, alloc)); 

        kTest(kString_Construct(&obj->ascii.customFormat, "", alloc));
        kTest(kString_Construct(&obj->ascii.delimiter, "", alloc));
        kTest(kString_Construct(&obj->ascii.invalidValue, "", alloc));
        kTest(kString_Construct(&obj->ascii.terminator, "", alloc));
    }
    kCatch(&exception)
    {
        GoSerial_VRelease(serial);
        kEndCatch(exception);
    }
    

    return kOK; 
}

GoFx(kStatus) GoSerial_VRelease(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial); 

    kCheck(kDestroyRef(&obj->protocolOptions)); 
    kCheck(kDestroyRef(&obj->measurementOptions)); 
    kCheck(kDestroyRef(&obj->measurementSources)); 
    kCheck(kDisposeRef(&obj->selcom.formatOptions));
    kCheck(kDisposeRef(&obj->selcom.rateOptions));

    kCheck(kDestroyRef(&obj->ascii.customFormat));
    kCheck(kDestroyRef(&obj->ascii.delimiter));
    kCheck(kDestroyRef(&obj->ascii.invalidValue));
    kCheck(kDestroyRef(&obj->ascii.terminator));

    kCheck(kObject_VRelease(serial)); 

    return kOK; 
}

GoFx(kStatus) GoSerial_Read(GoSerial serial, kXml xml, kXmlItem item)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kXmlItem tempItem = kNULL;
    kChar* text = kNULL; 
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(kObject_GetMemZero(serial, sizeof(kChar) * textCapacity, &text)); 

        tempItem = kXml_Child(xml, item, "Protocol");
        kTest(kXml_Item32s(xml, tempItem, &obj->protocol)); 
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->protocolOptions));  

        tempItem = kXml_Child(xml, item, "Measurements");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementOptions));  

        kTest(kXml_ChildText(xml, item, "Measurements", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementSources));  

        //selcom
        tempItem = kXml_Child(xml, item, "Selcom/Rate");
        kTest(kXml_Item32u(xml, tempItem, &obj->selcom.rate));
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity));
        kTest(GoOptionList_ParseList32u(text, obj->selcom.rateOptions));

        tempItem = kXml_Child(xml, item, "Selcom/Format");
        kTest(kXml_Item32s(xml, tempItem, (k32u*)&obj->selcom.format));
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity));
        kTest(GoOptionList_ParseList32u(text, obj->selcom.formatOptions));

        tempItem = kXml_Child(xml, item, "Selcom");
        kTest(kXml_Child64f(xml, tempItem, "DataScaleMin", &obj->selcom.dataScaleMin));
        kTest(kXml_Child64f(xml, tempItem, "DataScaleMax", &obj->selcom.dataScaleMax));

        tempItem = kXml_Child(xml, item, "Ascii");
        kTest(kXml_ChildString(xml, tempItem, "Delimiter", obj->ascii.delimiter));
        kTest(kXml_ChildString(xml, tempItem, "Terminator", obj->ascii.terminator));
        kTest(kXml_ChildString(xml, tempItem, "InvalidValue",obj->ascii.invalidValue));
        kTest(kXml_ChildString(xml, tempItem, "CustomDataFormat", obj->ascii.customFormat));
        kTest(kXml_ChildBool(xml, tempItem, "CustomFormatEnabled", &obj->ascii.customFormatEnabled));
    }
    kFinally
    {
        kObject_FreeMem(serial, text); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSerial_Write(GoSerial serial, kXml xml, kXmlItem item)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kChar* text = kNULL; 
    kXmlItem tempItem = kNULL;
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    kTry
    {
        kTest(kObject_GetMemZero(serial, sizeof(kChar) * textCapacity, &text)); 

        kTest(kXml_SetChild32s(xml, item, "Protocol", obj->protocol)); 
        
        kTest(GoOptionList_Format32u((k32u*)kArrayList_Data(obj->measurementSources), kArrayList_Count(obj->measurementSources), text, textCapacity));        
        kTest(kXml_SetChildText(xml, item, "Measurements", text)); 

        //selcom
        kTest(kXml_AddItem(xml, item, "Selcom", &tempItem));
        kTest(kXml_SetChild32u(xml, tempItem, "Rate", obj->selcom.rate));
        kTest(GoOptionList_Format32u((k32u*)kArrayList_Data(obj->selcom.rateOptions), kArrayList_Count(obj->selcom.rateOptions), text, textCapacity));
        kTest(kXml_SetChild32s(xml, tempItem, "Format", obj->selcom.format));
        kTest(kXml_SetChild64f(xml, tempItem, "DataScaleMin", obj->selcom.dataScaleMin));
        kTest(kXml_SetChild64f(xml, tempItem, "DataScaleMax", obj->selcom.dataScaleMax));

        kTest(kXml_AddItem(xml, item, "Ascii", &tempItem));
        kTest(kXml_SetChildText(xml, tempItem, "Delimiter", kString_Chars(obj->ascii.delimiter)));
        kTest(kXml_SetChildText(xml, tempItem, "Terminator", kString_Chars(obj->ascii.terminator)));
        kTest(kXml_SetChildText(xml, tempItem, "InvalidValue", kString_Chars(obj->ascii.invalidValue)));
        kTest(kXml_SetChildText(xml, tempItem, "CustomDataFormat", kString_Chars(obj->ascii.customFormat)));
        kTest(kXml_SetChildBool(xml, tempItem, "CustomFormatEnabled", obj->ascii.customFormatEnabled));
    }
    kFinally
    {
        kObject_FreeMem(serial, text); 
        kEndFinally(); 
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}


GoFx(kArrayList) GoSerial_ProtocolOptionList(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->protocolOptions;
}

GoFx(kSize) GoSerial_ProtocolOptionCount(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = obj->protocolOptions;

    return kArrayList_Count(list);
}

GoFx(k32u) GoSerial_ProtocolOptionAt(GoSerial serial, kSize index)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = obj->protocolOptions;

    kAssert(index < GoSerial_ProtocolOptionCount(serial));

    return kArrayList_As_(list, index, k32u);
}

GoFx(kStatus) GoSerial_SetProtocol(GoSerial serial, k32u protocol)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList optionList = GoSerial_ProtocolOptionList(serial);

    kCheck(GoOptionList_Check32u(kArrayList_Data(optionList), kArrayList_Count(optionList), protocol));
    obj->protocol = protocol;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoSerial_Protocol(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->protocol;
}

GoFx(kArrayList) GoSerial_OptionList(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->measurementOptions;
}

GoFx(kSize) GoSerial_OptionCount(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = obj->measurementOptions;

    return kArrayList_Count(list);
}

GoFx(k32u) GoSerial_OptionAt(GoSerial serial, kSize index)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = obj->measurementOptions;

    kAssert(index < GoSerial_OptionCount(serial));

    return kArrayList_As_(list, index, k32u);
}

GoFx(kSize) GoSerial_SourceCount(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = obj->measurementSources;

    return kArrayList_Count(list);
}

GoFx(k32u) GoSerial_SourceAt(GoSerial serial, kSize index)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = obj->measurementSources;

    kAssert(index < GoSerial_SourceCount(serial));

    return kArrayList_As_(list, index, k32u);
}

GoFx(kStatus) GoSerial_AddSource(GoSerial serial, k32u sourceId)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;
    kArrayList optionList;

    list = obj->measurementSources;
    optionList = obj->measurementOptions;

    kCheck(GoOptionList_Check32u(kArrayList_Data(optionList), kArrayList_Count(optionList), sourceId));
    kCheck(kArrayList_Add(list, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSerial_RemoveSource(GoSerial serial, kSize index)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    list = obj->measurementSources;

    kCheck(kArrayList_Remove(list, index, kNULL));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSerial_ClearSources(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList list;

    list = obj->measurementSources;

    kCheck(kArrayList_Clear(list));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSerial_SetSelcomRate(GoSerial serial, k32u rate)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList optionList = GoSerial_SelcomRateOptionList(serial);

    kCheck(GoOptionList_Check32u(kArrayList_Data(optionList), kArrayList_Count(optionList), rate));
    obj->selcom.rate = rate;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoSerial_SelcomRate(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->selcom.rate;
}

GoFx(kArrayList) GoSerial_SelcomRateOptionList(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->selcom.rateOptions;
}

GoFx(kSize) GoSerial_SelcomRateOptionCount(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->selcom.rateOptions);
}

GoFx(k32u) GoSerial_SelcomRateOptionAt(GoSerial serial, kSize index)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSerial_SelcomRateOptionCount(serial));

    return kArrayList_As_(obj->selcom.rateOptions, index, k32u);
}

GoFx(kStatus) GoSerial_SetSelcomFormat(GoSerial serial, GoSelcomFormat format)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);
    kArrayList optionList = GoSerial_SelcomFormatOptionList(serial);

    kCheck(GoOptionList_Check32u(kArrayList_Data(optionList), kArrayList_Count(optionList), format));
    obj->selcom.format = format;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSelcomFormat) GoSerial_SelcomFormat(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->selcom.format;
}

GoFx(kArrayList) GoSerial_SelcomFormatOptionList(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->selcom.formatOptions;
}

GoFx(kSize) GoSerial_SelcomFormatOptionCount(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->selcom.formatOptions);
}

GoFx(GoSelcomFormat) GoSerial_SelcomFormatOptionAt(GoSerial serial, kSize index)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->selcom.formatOptions));
    
    return kArrayList_As_(obj->selcom.formatOptions, index, GoSelcomFormat);
}

GoFx(kStatus) GoSerial_SetSelcomDataScaleMax(GoSerial serial, k64f value)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    obj->selcom.dataScaleMax = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSerial_SelcomDataScaleMax(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->selcom.dataScaleMax;
}

GoFx(kStatus) GoSerial_SetSelcomDataScaleMin(GoSerial serial, k64f value)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    obj->selcom.dataScaleMin = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSerial_SelcomDataScaleMin(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->selcom.dataScaleMin;
}


GoFx(kChar*) GoSerial_AsciiDelimiter(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.delimiter);
}

GoFx(kStatus) GoSerial_SetAsciiDelimiter(GoSerial serial, const kChar* string)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    kCheck(kString_Set(obj->ascii.delimiter, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoSerial_AsciiTerminator(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.terminator);
}

GoFx(kStatus) GoSerial_SetAsciiTerminator(GoSerial serial, const kChar* string)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    kCheck(kString_Set(obj->ascii.terminator, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoSerial_AsciiInvalidValue(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.invalidValue);
}

GoFx(kStatus) GoSerial_SetAsciiInvalidValue(GoSerial serial, const kChar* string)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    kCheck(kString_Set(obj->ascii.invalidValue, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoSerial_AsciiCustomDataFormat(GoSerial serial)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.customFormat);
}

GoFx(kStatus) GoSerial_SetAsciiCustomDataFormat(GoSerial serial, const kChar* string)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    kCheck(kString_Set(obj->ascii.customFormat, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSerial_EnableAsciiCustomFormat(GoSerial serial, kBool enabled)
{
    GoSerialClass* obj = GoSerial_Cast_(serial);

    obj->ascii.customFormatEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSerial_AsciiCustomFormatEnabled(GoSerial serial)
{       
    GoSerialClass* obj = GoSerial_Cast_(serial);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.customFormatEnabled;
}

