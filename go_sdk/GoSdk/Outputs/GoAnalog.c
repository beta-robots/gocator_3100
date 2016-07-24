/** 
 * @file    GoAnalog.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoAnalog.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoAnalog, kObject)
kAddVMethod(GoAnalog, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoAnalog_Construct(GoAnalog* analog, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAnalog), analog)); 

    if (!kSuccess(status = GoAnalog_Init(*analog, kTypeOf(GoAnalog), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, analog); 
    }

    return status; 
} 

GoFx(kStatus) GoAnalog_Init(GoAnalog analog, kType type, kObject sensor, kAlloc alloc)
{
    GoAnalogClass* obj = analog; 
    kStatus exception;

    kCheck(kObject_Init(analog, type, alloc)); 
    kInitFields_(GoAnalog, analog);

    obj->sensor = sensor;
    
    kTry
    {
        kTest(kArrayList_Construct(&obj->measurementSource, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(k32u), 0, alloc)); 
    }
    kCatch(&exception)
    {
        GoAnalog_VRelease(analog);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoAnalog_VRelease(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog); 

    kCheck(kDestroyRef(&obj->measurementSource)); 
    kCheck(kDestroyRef(&obj->measurementOptions)); 

    kCheck(kObject_VRelease(analog)); 

    return kOK; 
}

GoFx(kStatus) GoAnalog_Read(GoAnalog analog, kXml xml, kXmlItem item)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);
    kChar* text = kNULL; 
    kXmlItem tempItem = kNULL;
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(kObject_GetMemZero(analog, sizeof(kChar) * textCapacity, &text)); 

        tempItem = kXml_Child(xml, item, "Measurement");
        kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementOptions));  

        kTest(kXml_ChildText(xml, item, "Measurement", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementSource));  
        if (kArrayList_Count(obj->measurementSource) > 1)
        {
            return kERROR_PARAMETER;
        }

        kTest(kXml_Child32u(xml, item, "Event", &obj->event)); 
        kTest(kXml_ChildBool(xml, item, "ScheduleEnabled", &obj->scheduleEnabled)); 

        tempItem = kXml_Child(xml, item, "CurrentMin");
        kTest(kXml_Item64f(xml, tempItem, &obj->currentMin.value)); 
        kTest(kXml_Attr64f(xml, tempItem, "min", &obj->currentMin.min));
        kTest(kXml_Attr64f(xml, tempItem, "max", &obj->currentMin.max));

        tempItem = kXml_Child(xml, item, "CurrentMax");
        kTest(kXml_Item64f(xml, tempItem, &obj->currentMax.value)); 
        kTest(kXml_Attr64f(xml, tempItem, "min", &obj->currentMax.min));
        kTest(kXml_Attr64f(xml, tempItem, "max", &obj->currentMax.max));

        kTest(kXml_ChildBool(xml, item, "CurrentInvalidEnabled", &obj->currentInvalid.enabled)); 
        tempItem = kXml_Child(xml, item, "CurrentInvalid");
        kTest(kXml_Item64f(xml, tempItem, &obj->currentInvalid.value)); 
        kTest(kXml_Attr64f(xml, tempItem, "min", &obj->currentInvalid.min));
        kTest(kXml_Attr64f(xml, tempItem, "max", &obj->currentInvalid.max));

        kTest(kXml_Child64f(xml, item, "DataScaleMin", &obj->dataScaleMin)); 
        kTest(kXml_Child64f(xml, item, "DataScaleMax", &obj->dataScaleMax)); 

        kTest(kXml_Child64s(xml, item, "Delay", &obj->delay));
        kTest(kXml_Child32s(xml, item, "DelayDomain", &obj->delayDomain));
    }
    kFinally
    {
        kObject_FreeMem(analog, text); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoAnalog_Write(GoAnalog analog, kXml xml, kXmlItem item)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);
    kChar* text = kNULL; 
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    kTry
    {
        kTest(kObject_GetMemZero(analog, sizeof(kChar) * textCapacity, &text)); 

        kTest(GoOptionList_Format32u(kArrayList_Data(obj->measurementSource), kArrayList_Count(obj->measurementSource), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Measurement", text)); 

        kTest(kXml_SetChild32s(xml, item, "Event", obj->event)); 
        kTest(kXml_SetChild32s(xml, item, "ScheduleEnabled", obj->scheduleEnabled)); 

        kTest(kXml_SetChild64f(xml, item, "CurrentMin", obj->currentMin.value)); 
        kTest(kXml_SetChild64f(xml, item, "CurrentMax", obj->currentMax.value)); 
        kTest(kXml_SetChild64f(xml, item, "CurrentInvalidEnabled", obj->currentInvalid.enabled)); 
        kTest(kXml_SetChild64f(xml, item, "CurrentInvalid", obj->currentInvalid.value)); 

        kTest(kXml_SetChild64f(xml, item, "DataScaleMin", obj->dataScaleMin)); 
        kTest(kXml_SetChild64f(xml, item, "DataScaleMax", obj->dataScaleMax)); 
        kTest(kXml_SetChild64s(xml, item, "Delay", obj->delay)); 
        kTest(kXml_SetChild32s(xml, item, "DelayDomain", obj->delayDomain)); 
    }
    kFinally
    {
        kObject_FreeMem(analog, text); 
        kEndFinally(); 
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kStatus) GoAnalog_SetEvent(GoAnalog analog, GoAnalogEvent event)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->event = event; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(GoAnalogEvent) GoAnalog_Event(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->event; 
} 

GoFx(kArrayList) GoAnalog_OptionList(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->measurementOptions;
}

GoFx(kSize) GoAnalog_OptionCount( GoAnalog analog )
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);
    list = obj->measurementOptions;

    return kArrayList_Count(list);
}

GoFx(k32u) GoAnalog_OptionAt(GoAnalog analog, kSize index)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);
    k32u* option = kNULL; 
    kArrayList list;
    
    GoSensor_SyncConfig(obj->sensor);

    list = obj->measurementOptions;
    kAssert(index < kArrayList_Count(list));

    return kArrayList_As_(list, index, k32u);
} 

GoFx(kStatus) GoAnalog_SetSource( GoAnalog analog, k32u sourceId )
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);
    kArrayList optionList = GoAnalog_OptionList(analog);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));

    kCheck(GoOptionList_Check32u(kArrayList_Data(optionList), kArrayList_Count(optionList), sourceId));
    kCheck(kArrayList_Clear(obj->measurementSource));
    kCheck(kArrayList_Add(obj->measurementSource, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAnalog_Source(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_As_(obj->measurementSource, 0, k32u);
}

GoFx(kStatus) GoAnalog_ClearSource(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);
    kCheck(kArrayList_Clear(obj->measurementSource));  
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}   

GoFx(k64f) GoAnalog_CurrentLimitMin(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.value; 
}  

GoFx(k64f) GoAnalog_CurrentLimitMax(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.value; 
} 

GoFx(kStatus) GoAnalog_SetCurrentMin(GoAnalog analog, k64f min)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    kCheckArgs(GoUtils_MinMax_(min, GoAnalog_CurrentMinLimitMin(analog), GoAnalog_CurrentMinLimitMax(analog)));

    obj->currentMin.value = min; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_CurrentMin(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.value;  
} 

GoFx(k64f) GoAnalog_CurrentMinLimitMin(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.min;  
}  

GoFx(k64f) GoAnalog_CurrentMinLimitMax(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.max;  
}  

GoFx(kStatus) GoAnalog_SetCurrentMax(GoAnalog analog, k64f max)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    kCheckArgs(GoUtils_MinMax_(max, GoAnalog_CurrentMaxLimitMin(analog), GoAnalog_CurrentMaxLimitMax(analog)));

    obj->currentMax.value = max; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_CurrentMax(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.value;  
}  

GoFx(k64f) GoAnalog_CurrentMaxLimitMin(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.min;  
}  

GoFx(k64f) GoAnalog_CurrentMaxLimitMax(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.max;  
}  

GoFx(kStatus) GoAnalog_EnableCurrentInvalid(GoAnalog analog, kBool enable)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->currentInvalid.enabled = enable; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(kBool) GoAnalog_CurrentInvalidEnabled(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.enabled;  
}  

GoFx(kStatus) GoAnalog_SetCurrentInvalid(GoAnalog analog, k64f invalid)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    kCheckArgs(GoUtils_MinMax_(invalid, GoAnalog_CurrentInvalidLimitMin(analog), GoAnalog_CurrentInvalidLimitMax(analog)));

    obj->currentInvalid.value = invalid; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_CurrentInvalid(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.value;  
}  

GoFx(k64f) GoAnalog_CurrentInvalidLimitMin(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.min;  
}  

GoFx(k64f) GoAnalog_CurrentInvalidLimitMax(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.max;  
}  

GoFx(kStatus) GoAnalog_SetDataScaleMin(GoAnalog analog, k64f min)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->dataScaleMin = min; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_DataScaleMin(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dataScaleMin;  
}  

GoFx(kStatus) GoAnalog_SetDataScaleMax(GoAnalog analog, k64f max)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->dataScaleMax = max; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_DataScaleMax(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dataScaleMax;  
}  

GoFx(kStatus) GoAnalog_SetDelay(GoAnalog analog, k64s delay)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->delay = delay; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64s) GoAnalog_Delay(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delay;  
} 

GoFx(kStatus) GoAnalog_SetDelayDomain(GoAnalog analog, GoOutputDelayDomain delayDomain)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->delayDomain = delayDomain; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(GoOutputDelayDomain) GoAnalog_DelayDomain(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delayDomain;  
} 

GoFx(kStatus) GoAnalog_EnableSchedule(GoAnalog analog, kBool enabled )
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    obj->scheduleEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(kBool) GoAnalog_ScheduleEnabled(GoAnalog analog)
{
    GoAnalogClass* obj = GoAnalog_Cast_(analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->scheduleEnabled;  
} 

