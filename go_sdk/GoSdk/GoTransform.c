/** 
 * @file    GoTransform.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoTransform.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoTransform, kObject)
    kAddVMethod(GoTransform, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoTransform_Construct(GoTransform* transform, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoTransform), transform)); 

    if (!kSuccess(status = GoTransform_Init(*transform, kTypeOf(GoTransform), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, transform); 
    }

    return status; 
} 

GoFx(kStatus) GoTransform_Init(GoTransform transform, kType type, kObject sensor, kAlloc alloc)
{
    GoTransformClass* obj = transform; 

    kCheck(kObject_Init(transform, type, alloc)); 
    kInitFields_(GoTransform, transform);

    obj->sensor = sensor; 

    return kOK; 
}

GoFx(kStatus) GoTransform_VRelease(GoTransform transform)
{
    GoTransformClass* obj = GoTransform_Cast_(transform); 

    kDestroyRef(&obj->xml);

    return kObject_VRelease(transform);
}

GoFx(kStatus) GoTransform_Read(GoTransform transform, kXml xml, kXmlItem item)
{
    GoTransformClass* obj = GoTransform_Cast_(transform); 
    kXmlItem devicesItem = kNULL;
    kXmlItem tempItem = kNULL;
    kSize i;

    kDestroyRef(&obj->xml);
    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_Child64f(xml, item, "EncoderResolution", &obj->encoderResolution));
    kCheck(kXml_Child64f(xml, item, "Speed", &obj->encoderSpeed));

    kCheckArgs(!kIsNull(devicesItem = kXml_Child(xml, item, "Devices")));
    kCheckArgs(2 == kXml_ChildCount(xml, devicesItem));

    for (i = 0; i < 2; i++)
    {
        tempItem = kXml_ChildAt(xml, devicesItem, i);

        kCheck(kXml_Child64f(xml, tempItem, "X", &obj->transformation[i].x));
        kCheck(kXml_Child64f(xml, tempItem, "Y", &obj->transformation[i].y));
        kCheck(kXml_Child64f(xml, tempItem, "Z", &obj->transformation[i].z));
        kCheck(kXml_Child64f(xml, tempItem, "XAngle", &obj->transformation[i].xAngle));
        kCheck(kXml_Child64f(xml, tempItem, "YAngle", &obj->transformation[i].yAngle));
        kCheck(kXml_Child64f(xml, tempItem, "ZAngle", &obj->transformation[i].zAngle));
    }

    return kOK; 
}

GoFx(kStatus) GoTransform_Write(GoTransform transform, kXml xml, kXmlItem item)
{
    GoTransformClass* obj = GoTransform_Cast_(transform); 
    kXmlItem devicesItem = kNULL;
    kXmlItem tempItem = kNULL;
    kSize i;

    kCheck(kXml_SetChild64f(xml, item, "EncoderResolution", obj->encoderResolution));
    kCheck(kXml_SetChild64f(xml, item, "Speed", obj->encoderSpeed));

    kCheck(kXml_AddItem(xml, item, "Devices", &devicesItem));

    for (i = 0; i < 2; i++)
    {
        kCheck(kXml_AddItem(xml, devicesItem, "Device", &tempItem));
        kCheck(kXml_SetAttrSize(xml, tempItem, "role", i));
        kCheck(kXml_SetChild64f(xml, tempItem, "X", obj->transformation[i].x));
        kCheck(kXml_SetChild64f(xml, tempItem, "Y", obj->transformation[i].y));
        kCheck(kXml_SetChild64f(xml, tempItem, "Z", obj->transformation[i].z));
        kCheck(kXml_SetChild64f(xml, tempItem, "XAngle", obj->transformation[i].xAngle));
        kCheck(kXml_SetChild64f(xml, tempItem, "YAngle", obj->transformation[i].yAngle));
        kCheck(kXml_SetChild64f(xml, tempItem, "ZAngle", obj->transformation[i].zAngle));
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(k64f) GoTransform_EncoderResolution(GoTransform transform)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->encoderResolution;
}

GoFx(kStatus) GoTransform_SetEncoderResolution(GoTransform transform, k64f value)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->encoderResolution = value;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_Speed(GoTransform transform)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->encoderSpeed;
}

GoFx(kStatus) GoTransform_SetSpeed(GoTransform transform, k64f value)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->encoderSpeed = value;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_X(GoTransform transform, GoRole role)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->transformation[role].x;
}

GoFx(kStatus) GoTransform_SetX(GoTransform transform, GoRole role, k64f offset)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->transformation[role].x = offset;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_Y(GoTransform transform, GoRole role)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->transformation[role].y;
}

GoFx(kStatus) GoTransform_SetY(GoTransform transform, GoRole role, k64f offset)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->transformation[role].y = offset;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_Z(GoTransform transform, GoRole role)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->transformation[role].z;
}

GoFx(kStatus) GoTransform_SetZ(GoTransform transform, GoRole role, k64f offset)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->transformation[role].z = offset;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}


GoFx(k64f) GoTransform_XAngle(GoTransform transform, GoRole role)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->transformation[role].xAngle;
}

GoFx(kStatus) GoTransform_SetXAngle(GoTransform transform, GoRole role, k64f angle)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->transformation[role].xAngle = angle;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_YAngle(GoTransform transform, GoRole role)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->transformation[role].yAngle;
}

GoFx(kStatus) GoTransform_SetYAngle(GoTransform transform, GoRole role, k64f angle)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->transformation[role].yAngle = angle;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_ZAngle(GoTransform transform, GoRole role)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->transformation[role].zAngle;
}

GoFx(kStatus) GoTransform_SetZAngle(GoTransform transform, GoRole role, k64f angle)
{
    GoTransformClass* obj = GoTransform_Cast_(transform);

    obj->transformation[role].zAngle = angle;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}
