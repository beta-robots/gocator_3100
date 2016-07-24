/** 
 * @file    GoDigital.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoDigital.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoDigital, kObject)
kAddVMethod(GoDigital, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoDigital_Construct(GoDigital* digital, k32u id, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDigital), digital)); 

    if (!kSuccess(status = GoDigital_Init(*digital, kTypeOf(GoDigital), id, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, digital); 
    }

    return status; 
} 

GoFx(kStatus) GoDigital_Init(GoDigital digital, kType type, k32u id, kObject sensor, kAlloc alloc)
{
    GoDigitalClass* obj = digital; 
    kStatus exception;

    kCheck(kObject_Init(digital, type, alloc)); 
    kInitFields_(GoDigital, digital);

    obj->sensor = sensor; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->decisionOptions, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->decisionSources, kTypeOf(k32u), 0, alloc)); 
    }
    kCatch(&exception)
    {
        GoDigital_VRelease(digital);
        kEndCatch(exception);
    }
    

    return kOK; 
}

GoFx(kStatus) GoDigital_VRelease(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    kCheck(kDestroyRef(&obj->decisionOptions)); 
    kCheck(kDestroyRef(&obj->decisionSources)); 

    kCheck(kObject_VRelease(digital)); 

    return kOK; 
}

GoFx(kStatus) GoDigital_Read(GoDigital digital, kXml xml, kXmlItem item)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);
    kChar* text = kNULL; 
    kXmlItem tempItem = kNULL;
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(kObject_GetMemZero(digital, sizeof(kChar) * textCapacity, &text)); 

        tempItem = kXml_Child(xml, item, "Measurements");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->decisionOptions));  
        kTest(kXml_ItemText(xml, tempItem, text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->decisionSources));  

        kTest(kXml_Child32u(xml, item, "Event", &obj->event)); 
        kTest(kXml_Child32u(xml, item, "SignalType", &obj->signalType)); 
        kTest(kXml_Child32s(xml, item, "ScheduleEnabled", &obj->scheduleEnabled)); 

        tempItem = kXml_Child(xml, item, "PulseWidth");
        kTest(kXml_Item32u(xml, tempItem, &obj->pulseWidth.value)); 
        kTest(kXml_Attr32u(xml, tempItem, "min", &obj->pulseWidth.min)); 
        kTest(kXml_Attr32u(xml, tempItem, "max", &obj->pulseWidth.max)); 

        kTest(kXml_Child32s(xml, item, "PassMode", &obj->passMode)); 
        kTest(kXml_Child64s(xml, item, "Delay", &obj->delay)); 
        kTest(kXml_Child32s(xml, item, "DelayDomain", &obj->delayDomain)); 
    }
    kFinally
    {
        kObject_FreeMem(digital, text); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoDigital_Write(GoDigital digital, kXml xml, kXmlItem item)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);
    kChar* text = kNULL; 
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    kTry
    {
        kTest(kObject_GetMemZero(digital, sizeof(kChar) * textCapacity, &text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->decisionSources), kArrayList_Count(obj->decisionSources), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Measurements", text)); 

        kTest(kXml_SetChild32u(xml, item, "Event", obj->event)); 
        kTest(kXml_SetChild32u(xml, item, "SignalType", obj->signalType)); 
        kTest(kXml_SetChild32u(xml, item, "ScheduleEnabled", obj->scheduleEnabled)); 
        kTest(kXml_SetChild32u(xml, item, "PulseWidth", obj->pulseWidth.value)); 
        kTest(kXml_SetChild32s(xml, item, "PassMode", obj->passMode)); 
        kTest(kXml_SetChild64s(xml, item, "Delay", obj->delay)); 
        kTest(kXml_SetChild32s(xml, item, "DelayDomain", obj->delayDomain));         
    }
    kFinally
    {
        kObject_FreeMem(digital, text); 
        kEndFinally(); 
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kStatus) GoDigital_SetEvent(GoDigital digital, GoDigitalEvent event)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    obj->event = event; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(GoDigitalEvent) GoDigital_Event(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->event;  
} 

GoFx(kArrayList) GoDigital_OptionList(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->decisionOptions;
}

GoFx(kSize) GoDigital_OptionCount(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->decisionOptions);  
}  

GoFx(k32u) GoDigital_OptionAt(GoDigital digital, kSize index)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoDigital_OptionCount(digital));

    return kArrayList_As_(obj->decisionOptions, index, k32u);
}  

GoFx(kSize) GoDigital_SourceCount(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->decisionSources);  
}  

GoFx(k32u) GoDigital_SourceAt(GoDigital digital, kSize index)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoDigital_SourceCount(digital));

    return kArrayList_As_(obj->decisionSources, index, k32u);
}  

GoFx(kStatus) GoDigital_AddSource(GoDigital digital, k32u sourceId)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);
    kArrayList options = GoDigital_OptionList(digital);

    kCheck(GoOptionList_Check32u(kArrayList_Data(options), kArrayList_Count(options), sourceId));
    kCheck(kArrayList_Add(obj->decisionSources, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;  
}   

GoFx(kStatus) GoDigital_RemoveSource(GoDigital digital, kSize index)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    kCheck(kArrayList_Remove(obj->decisionSources, index, kNULL));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}   

GoFx(kStatus) GoDigital_ClearSources(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);
    kCheck(kArrayList_Clear(obj->decisionSources));  
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}   

GoFx(kStatus) GoDigital_SetPassMode(GoDigital digital, GoDigitalPass pass)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    obj->passMode = pass; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(GoDigitalPass) GoDigital_PassMode(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->passMode;  
} 

GoFx(k32u) GoDigital_PulseWidthLimitMin(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->pulseWidth.min;  
}  

GoFx(k32u) GoDigital_PulseWidthLimitMax(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->pulseWidth.max;  
}  

GoFx(kStatus) GoDigital_SetPulseWidth(GoDigital digital, k32u width)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    kCheckArgs(GoUtils_MinMax_(width, GoDigital_PulseWidthLimitMin(digital), GoDigital_PulseWidthLimitMax(digital)));

    obj->pulseWidth.value = width; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}  

GoFx(k32u) GoDigital_PulseWidth(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->pulseWidth.value;  
}  

GoFx(kStatus) GoDigital_SetSignalType(GoDigital digital, GoDigitalSignal signal)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    obj->signalType = signal; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}  

GoFx(GoDigitalSignal) GoDigital_SignalType(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->signalType;  
} 

GoFx(kStatus) GoDigital_SetDelay(GoDigital digital, k64s delay)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    obj->delay = delay; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}  

GoFx(k64s) GoDigital_Delay(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delay;  
} 

GoFx(kStatus) GoDigital_SetDelayDomain(GoDigital digital, GoOutputDelayDomain delayDomain)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    obj->delayDomain = delayDomain; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(GoOutputDelayDomain) GoDigital_DelayDomain(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delayDomain;  
} 

GoFx(kStatus) GoDigital_EnableSchedule(GoDigital digital, kBool enabled )
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    obj->scheduleEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}  

GoFx(kBool) GoDigital_ScheduleEnabled(GoDigital digital)
{
    GoDigitalClass* obj = GoDigital_Cast_(digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->scheduleEnabled;  
}  
