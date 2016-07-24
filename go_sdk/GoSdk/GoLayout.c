/** 
 * @file    GoLayout.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoLayout.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoLayout, kObject)
    kAddVMethod(GoLayout, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoLayout_Construct(GoLayout* layout, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoLayout), layout)); 

    if (!kSuccess(status = GoLayout_Init(*layout, kTypeOf(GoLayout), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, layout); 
    }

    return status; 
} 

GoFx(kStatus) GoLayout_Init(GoLayout layout, kType type, kObject sensor, kAlloc alloc)
{
    GoLayoutClass* obj = layout; 

    kCheck(kObject_Init(layout, type, alloc)); 
    kInitFields_(GoLayout, layout);

    obj->sensor = sensor; 
    obj->orientation = GO_ORIENTATION_WIDE;

    return kOK; 
}

GoFx(kStatus) GoLayout_VRelease(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout); 

    return  kObject_VRelease(layout);
}

GoFx(kStatus) GoLayout_Read(GoLayout layout, kXml xml, kXmlItem item)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout); 
    kXmlItem tempItem = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_Child32u(xml, item, "XSpacingCount", &obj->xSpacingCount));    
    kCheck(kXml_Child32u(xml, item, "YSpacingCount", &obj->ySpacingCount));    

    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/X", &obj->transformedDataRegion.x));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Y", &obj->transformedDataRegion.y));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Z", &obj->transformedDataRegion.z));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Width", &obj->transformedDataRegion.width));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Length", &obj->transformedDataRegion.length));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Height", &obj->transformedDataRegion.height));

    kCheck(kXml_Child32s(xml, item, "Orientation", (k32s*)&obj->orientation));
    kCheck(kXml_ChildBool(xml, item, "MultiplexBuddyEnabled", &obj->multiplexBuddyEnabled));
    kCheck(kXml_ChildBool(xml, item, "MultiplexSingleEnabled", &obj->multiplexSingleEnabled));
    kCheck(kXml_Child64f(xml, item, "MultiplexSingleDelay", &obj->multiplexSingleDelay));
    kCheck(kXml_Child64f(xml, item, "MultiplexSingleExposureDuration", &obj->multiplexSingleExposureDuration)); //read only

    tempItem = kXml_Child(xml, item, "MultiplexSinglePeriod");
    kCheck(kXml_Item64f(xml, tempItem, &obj->multiplexSinglePeriod)); 
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->multiplexSinglePeriodMin));
    
    return kOK; 
}

GoFx(kStatus) GoLayout_Write(GoLayout layout, kXml xml, kXmlItem item)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout); 

    kCheck(kXml_SetChild32s(xml, item, "Orientation", obj->orientation));
    kCheck(kXml_SetChildBool(xml, item, "MultiplexBuddyEnabled", obj->multiplexBuddyEnabled));
    kCheck(kXml_SetChildBool(xml, item, "MultiplexSingleEnabled", obj->multiplexSingleEnabled));
    kCheck(kXml_SetChild64f(xml, item, "MultiplexSingleDelay", obj->multiplexSingleDelay));
    kCheck(kXml_SetChild64f(xml, item, "MultiplexSinglePeriod", obj->multiplexSinglePeriod));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kBool) GoLayout_MultiplexBuddyEnabled(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->multiplexBuddyEnabled;
}

GoFx(kStatus) GoLayout_EnableMultiplexBuddy(GoLayout layout, kBool enable)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->multiplexBuddyEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(kBool) GoLayout_MultiplexSingleEnabled(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->multiplexSingleEnabled;
}

GoFx(kStatus) GoLayout_EnableMultiplexSingle(GoLayout layout, kBool enabled)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->multiplexSingleEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoLayout_MultiplexSingleDelay(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->multiplexSingleDelay;
}

GoFx(kStatus) GoLayout_SetMultiplexSingleDelay(GoLayout layout, k64f value)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->multiplexSingleDelay = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoLayout_MultiplexSinglePeriod(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->multiplexSinglePeriod;
}

GoFx(k64f) GoLayout_MultiplexSinglePeriodMin(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->multiplexSinglePeriodMin;
}

GoFx(kStatus) GoLayout_SetMultiplexSinglePeriod(GoLayout layout, k64f value)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->multiplexSinglePeriod = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}


GoFx(k64f) GoLayout_MultiplexSingleExposureDuration(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->multiplexSingleExposureDuration;
}

GoFx(kStatus) GoLayout_SetOrientation(GoLayout layout, GoOrientation orientation)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->orientation = orientation;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    
    return kOK; 
}

GoFx(GoOrientation) GoLayout_Orientation(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    GoSensor_SyncConfig(obj->sensor);

    return obj->orientation;
}

GoFx(k64f) GoLayout_TransformedDataRegionX(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->transformedDataRegion.x;
}

GoFx(k64f) GoLayout_TransformedDataRegionY(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->transformedDataRegion.y;
}

GoFx(k64f) GoLayout_TransformedDataRegionZ(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->transformedDataRegion.z;
}

GoFx(k64f) GoLayout_TransformedDataRegionWidth(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->transformedDataRegion.width;
}

GoFx(k64f) GoLayout_TransformedDataRegionLength(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->transformedDataRegion.length;
}

GoFx(k64f) GoLayout_TransformedDataRegionHeight(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->transformedDataRegion.height;
}

GoFx(k64f) GoLayout_XSpacingCount(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->xSpacingCount;
}

GoFx(k64f) GoLayout_YSpacingCount(GoLayout layout)
{
    GoLayoutClass* obj = GoLayout_Cast_(layout);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    GoSensor_SyncConfig(obj->sensor);

    return obj->ySpacingCount;
}
