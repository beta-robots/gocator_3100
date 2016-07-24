/** 
 * @file    GoExtParam.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoExtParam.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginValue(Go, GoExtParamIntOption, kValue)
    kAddField(GoExtParamIntOption, k32s, value)
    kAddField(GoExtParamIntOption, kText64, description) 
kEndValue()

kBeginValue(Go, GoExtParamFloatOption, kValue)
    kAddField(GoExtParamFloatOption, k64f, value)
    kAddField(GoExtParamFloatOption, kText64, description) 
kEndValue()

kBeginVirtualClass(Go, GoExtParam, kObject)
    kAddFlags(GoExtParam, kTYPE_FLAGS_ABSTRACT)

    kAddVMethod(GoExtParam, kObject, VRelease)
    kAddVMethod(GoExtParam, GoExtParam, VInit)
    kAddVMethod(GoExtParam, GoExtParam, VRead)
    kAddVMethod(GoExtParam, GoExtParam, VWrite)
kEndVirtualClass()

GoFx(kStatus) GoExtParam_Construct(GoExtParam* param, kType type, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, type, param)); 

    if (!kSuccess(status = kCast(GoExtParamVTable*, kType_VTable_(type))->VInit(*param, type, sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParam_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED; //this function must be overriden by every tool
}

GoFx(kStatus) GoExtParam_Init(GoExtParam param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamClass* obj = param; 
    
    kCheck(kObject_Init(param, type, alloc)); 
    kInitFields_(GoExtParam, param);

    obj->sensor = sensor;

    return kOK; 
}

GoFx(kStatus) GoExtParam_VRelease(GoExtParam param)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);
    
    return kObject_VRelease(param);
}

GoFx(kStatus) GoExtParam_VRead(GoExtParam param, kXml xml, kXmlItem item)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);
    kXml paramXml = kNULL;
    kXmlItem paramItem = kNULL;
    
    kCheck(kStrCopy(obj->id, kCountOf(obj->id), kXml_ItemName(xml, item)));
    kCheck(kXml_AttrText(xml, item, "label", obj->label, 64));
    kCheck(kXml_Attr32s(xml, item, "type", &obj->type));
    
    //optional reads
    if (kXml_AttrExists(xml, item, "used"))
    {
        kCheck(kXml_AttrBool(xml, item, "used", &obj->used));
    }

    if (kXml_AttrExists(xml, item, "units"))
    {
        kCheck(kXml_Attr32s(xml, item, "units", &obj->units));
    }
    
    return kOK;
}

GoFx(kStatus) GoExtParam_VWrite(GoExtParam param, kXml xml, kXmlItem item)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);

    kCheck(kXml_SetAttrText(xml, item, "label", obj->label));
    
    return kOK;
}

GoFx(kStatus) GoExtParam_Read(GoExtParam param, kXml xml, kXmlItem item)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param); 
    return kCast(GoExtParamVTable*, kObject_VTable_(param))->VRead(param, xml, item);
}

GoFx(kStatus) GoExtParam_Write(GoExtParam param, kXml xml, kXmlItem item)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param); 
    return kCast(GoExtParamVTable*, kObject_VTable_(param))->VWrite(param, xml, item);
}

GoFx(const kChar*) GoExtParam_Label(GoExtParam param)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);

    return obj->label;
}

GoFx(const kChar*) GoExtParam_Id(GoExtParam param)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);

    return obj->id;
}

GoFx(GoExtParamType) GoExtParam_Type(GoExtParam param)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);

    return obj->type;
}

GoFx(kBool) GoExtParam_Used(GoExtParam param)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);

    return obj->used;
}

GoFx(GoUnitType) GoExtParam_UnitType(GoExtParam param)
{
    GoExtParamClass* obj = GoExtParam_Cast_(param);

    return obj->units;
}

kBeginClass(Go, GoExtParamInt, GoExtParam)
kAddVMethod(GoExtParamInt, kObject, VRelease)
kAddVMethod(GoExtParamInt, GoExtParam, VInit)
kAddVMethod(GoExtParamInt, GoExtParam, VRead)
kAddVMethod(GoExtParamInt, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamInt_Construct(GoExtParamInt* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamInt), param)); 

    if (!kSuccess(status = GoExtParamInt_VInit(*param, kTypeOf(GoExtParamInt), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamInt_VInit(GoExtParamInt param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamIntClass* obj = param; 
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamInt, param);

    kTry
    {
        kTest(kArrayList_Construct(&obj->options, kTypeOf(GoExtParamIntOption), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamInt_VRelease(param);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParamInt_VRelease(GoExtParamInt param)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    kDisposeRef(&obj->options);

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamInt_VRead(GoExtParamInt param, kXml xml, kXmlItem item)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);
    
    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_Item32s(xml, item, &obj->value));

    if (kXml_AttrExists(xml, item, "options"))
    {
        kCheck(GoExtParamInt_ParseList(xml, item, obj->options));
    }

    if (kXml_AttrExists(xml, item, "min"))
    {
        kCheck(kXml_Attr32s(xml, item, "min", &obj->valMin));
    }

    if (kXml_AttrExists(xml, item, "max"))
    {
        kCheck(kXml_Attr32s(xml, item, "max", &obj->valMax));
    }

    return kOK;
}

GoFx(kStatus) GoExtParamInt_VWrite(GoExtParamInt param, kXml xml, kXmlItem item)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItem32s(xml, item, obj->value));

    return kOK;
}

GoFx(k32s) GoExtParamInt_Value(GoExtParamInt param)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    return obj->value;
}

GoFx(kStatus) GoExtParamInt_SetValue(GoExtParamInt param, k32s newVal)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    //validate if min and max specified, or options list is present
    obj->value = newVal;

    return kOK;
}

GoFx(k32s) GoExtParamInt_ValueMin(GoExtParamInt param)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    return obj->valMin;
}

GoFx(k32s) GoExtParamInt_ValueMax(GoExtParamInt param)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    return obj->valMax;
}

GoFx(kSize) GoExtParamInt_OptionCount(GoExtParamInt param)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    return kArrayList_Count(obj->options);
}

GoFx(k32s) GoExtParamInt_OptionValueAt(GoExtParamInt param, kSize index)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    if (index >= kArrayList_Count(obj->options))
    {
        return kNULL;
    }

    return *(k32s*)kArrayList_At(obj->options, index);
}

GoFx(const kChar*) GoExtParamInt_OptionDescriptionAt(GoExtParamInt param, kSize index)
{
    GoExtParamIntClass* obj = GoExtParamInt_Cast_(param);

    if (index >= kArrayList_Count(obj->options))
    {
        return kNULL;
    }

    return ((GoExtParamIntOption*)kArrayList_At(obj->options, index))->description;
}

kBeginClass(Go, GoExtParamFloat, GoExtParam)
kAddVMethod(GoExtParamFloat, kObject, VRelease)
kAddVMethod(GoExtParamFloat, GoExtParam, VInit)
kAddVMethod(GoExtParamFloat, GoExtParam, VRead)
kAddVMethod(GoExtParamFloat, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamFloat_Construct(GoExtParamFloat* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamFloat), param)); 

    if (!kSuccess(status = GoExtParamFloat_VInit(*param, kTypeOf(GoExtParamFloat), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamFloat_VInit(GoExtParamFloat param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamFloatClass* obj = param; 
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamFloat, param);

    kTry
    {
        kTest(kArrayList_Construct(&obj->options, kTypeOf(GoExtParamFloatOption), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamFloat_VRelease(param);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParamFloat_VRelease(GoExtParamFloat param)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    kCheck(kDisposeRef(&obj->options));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamFloat_VRead(GoExtParamFloat param, kXml xml, kXmlItem item)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);
    
    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_Item64f(xml, item, &obj->value));

    if (kXml_AttrExists(xml, item, "options"))
    {
        kCheck(GoExtParamFloat_ParseList(xml, item, obj->options));
    }

    if (kXml_AttrExists(xml, item, "min"))
    {
        kCheck(kXml_Attr64f(xml, item, "min", &obj->valMin));
    }

    if (kXml_AttrExists(xml, item, "max"))
    {
        kCheck(kXml_Attr64f(xml, item, "max", &obj->valMax));
    }

    return kOK;
}

GoFx(kStatus) GoExtParamFloat_VWrite(GoExtParamFloat param, kXml xml, kXmlItem item)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItem64f(xml, item, obj->value));

    return kOK;
}

GoFx(k64f) GoExtParamFloat_Value(GoExtParamFloat param)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    return obj->value;
}

GoFx(kStatus) GoExtParamFloat_SetValue(GoExtParamFloat param, k64f newVal)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    obj->value = newVal;

    return kOK;
}

GoFx(k64f) GoExtParamFloat_ValueMin(GoExtParamFloat param)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    return obj->valMin;
}

GoFx(k64f) GoExtParamFloat_ValueMax(GoExtParamFloat param)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    return obj->valMax;
}

GoFx(kSize) GoExtParamFloat_OptionCount(GoExtParamFloat param)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    return kArrayList_Count(obj->options);
}

GoFx(k64f) GoExtParamFloat_OptionValueAt(GoExtParamFloat param, kSize index)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    if (index >= kArrayList_Count(obj->options))
    {
        return kNULL;
    }

    return *(k64f*)kArrayList_At(obj->options, index);
}

GoFx(const kChar*) GoExtParamFloat_OptionDescriptionAt(GoExtParamFloat param, kSize index)
{
    GoExtParamFloatClass* obj = GoExtParamFloat_Cast_(param);

    if (index >= kArrayList_Count(obj->options))
    {
        return kNULL;
    }

    return ((GoExtParamFloatOption*)kArrayList_At(obj->options, index))->description;
}

kBeginClass(Go, GoExtParamBool, GoExtParam)
kAddVMethod(GoExtParamBool, kObject, VRelease)
kAddVMethod(GoExtParamBool, GoExtParam, VInit)
kAddVMethod(GoExtParamBool, GoExtParam, VRead)
kAddVMethod(GoExtParamBool, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamBool_Construct(GoExtParamBool* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamBool), param)); 

    if (!kSuccess(status = GoExtParamBool_VInit(*param, kTypeOf(GoExtParamBool), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamBool_VInit(GoExtParamBool param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamBoolClass* obj = param; 

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamBool, param);

    return kOK; 
}

GoFx(kStatus) GoExtParamBool_VRelease(GoExtParamBool param)
{
    GoExtParamBoolClass* obj = GoExtParamBool_Cast_(param);

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamBool_VRead(GoExtParamBool param, kXml xml, kXmlItem item)
{
    GoExtParamBoolClass* obj = GoExtParamBool_Cast_(param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_ItemBool(xml, item, &obj->value));

    return kOK;
}

GoFx(kStatus) GoExtParamBool_VWrite(GoExtParamBool param, kXml xml, kXmlItem item)
{
    GoExtParamBoolClass* obj = GoExtParamBool_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItemBool(xml, item, obj->value));

    return kOK;
}

GoFx(kBool) GoExtParamBool_Value(GoExtParamBool param)
{
    GoExtParamBoolClass* obj = GoExtParamBool_Cast_(param);

    return obj->value;
}

GoFx(kStatus) GoExtParamBool_SetValue(GoExtParamBool param, kBool newVal)
{
    GoExtParamBoolClass* obj = GoExtParamBool_Cast_(param);

    obj->value = newVal;

    return kOK;
}

kBeginClass(Go, GoExtParamString, GoExtParam)
kAddVMethod(GoExtParamString, kObject, VRelease)
kAddVMethod(GoExtParamString, GoExtParam, VInit)
kAddVMethod(GoExtParamString, GoExtParam, VRead)
kAddVMethod(GoExtParamString, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamString_Construct(GoExtParamString* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamString), param)); 

    if (!kSuccess(status = GoExtParamString_VInit(*param, kTypeOf(GoExtParamString), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamString_VInit(GoExtParamString param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamStringClass* obj = param; 
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamString, param);

    kTry
    {
        kTest(kString_Construct(&obj->value, "", alloc));
    }
    kCatch(&exception)
    {
        GoExtParamString_VRelease(param);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParamString_VRelease(GoExtParamString param)
{
    GoExtParamStringClass* obj = GoExtParamString_Cast_(param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamString_VRead(GoExtParamString param, kXml xml, kXmlItem item)
{
    GoExtParamStringClass* obj = GoExtParamString_Cast_(param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_ItemString(xml, item, obj->value));

    return kOK;
}

GoFx(kStatus) GoExtParamString_VWrite(GoExtParamString param, kXml xml, kXmlItem item)
{
    GoExtParamStringClass* obj = GoExtParamString_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItemText(xml, item, kString_Chars(obj->value)));

    return kOK;
}

GoFx(kString) GoExtParamString_Value(GoExtParamString param)
{
    GoExtParamStringClass* obj = GoExtParamString_Cast_(param);

    return obj->value;
}

kBeginClass(Go, GoExtParamProfileRegion, GoExtParam)
kAddVMethod(GoExtParamProfileRegion, kObject, VRelease)
kAddVMethod(GoExtParamProfileRegion, GoExtParam, VInit)
kAddVMethod(GoExtParamProfileRegion, GoExtParam, VRead)
kAddVMethod(GoExtParamProfileRegion, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamProfileRegion_Construct(GoExtParamProfileRegion* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamProfileRegion), param)); 

    if (!kSuccess(status = GoExtParamProfileRegion_VInit(*param, kTypeOf(GoExtParamProfileRegion), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamProfileRegion_VInit(GoExtParamProfileRegion param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamProfileRegionClass* obj = param; 
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamProfileRegion, param);

    kTry
    {
        kTest(GoProfileRegion_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamProfileRegion_VRelease(param);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParamProfileRegion_VRelease(GoExtParamProfileRegion param)
{
    GoExtParamProfileRegionClass* obj = GoExtParamProfileRegion_Cast_(param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamProfileRegion_VRead(GoExtParamProfileRegion param, kXml xml, kXmlItem item)
{
    GoExtParamProfileRegionClass* obj = GoExtParamProfileRegion_Cast_(param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoProfileRegion_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamProfileRegion_VWrite(GoExtParamProfileRegion param, kXml xml, kXmlItem item)
{
    GoExtParamProfileRegionClass* obj = GoExtParamProfileRegion_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoProfileRegion_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoProfileRegion) GoExtParamProfileRegion_Value(GoExtParamProfileRegion param)
{
    GoExtParamProfileRegionClass* obj = GoExtParamProfileRegion_Cast_(param);

    return obj->value;
}

kBeginClass(Go, GoExtParamSurfaceRegion2d, GoExtParam)
kAddVMethod(GoExtParamSurfaceRegion2d, kObject, VRelease)
kAddVMethod(GoExtParamSurfaceRegion2d, GoExtParam, VInit)
kAddVMethod(GoExtParamSurfaceRegion2d, GoExtParam, VRead)
kAddVMethod(GoExtParamSurfaceRegion2d, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamSurfaceRegion2d_Construct(GoExtParamSurfaceRegion2d* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamSurfaceRegion2d), param)); 

    if (!kSuccess(status = GoExtParamSurfaceRegion2d_VInit(*param, kTypeOf(GoExtParamSurfaceRegion2d), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VInit(GoExtParamSurfaceRegion2d param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamSurfaceRegion2dClass* obj = param; 
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamSurfaceRegion2d, param);

    kTry
    {
        kTest(GoSurfaceRegion2d_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamSurfaceRegion2d_VRelease(param);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VRelease(GoExtParamSurfaceRegion2d param)
{
    GoExtParamSurfaceRegion2dClass* obj = GoExtParamSurfaceRegion2d_Cast_(param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VRead(GoExtParamSurfaceRegion2d param, kXml xml, kXmlItem item)
{
    GoExtParamSurfaceRegion2dClass* obj = GoExtParamSurfaceRegion2d_Cast_(param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoSurfaceRegion2d_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VWrite(GoExtParamSurfaceRegion2d param, kXml xml, kXmlItem item)
{
    GoExtParamSurfaceRegion2dClass* obj = GoExtParamSurfaceRegion2d_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoSurfaceRegion2d_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoExtParamSurfaceRegion2d_Value(GoExtParamSurfaceRegion2d param)
{
    GoExtParamSurfaceRegion2dClass* obj = GoExtParamSurfaceRegion2d_Cast_(param);

    return obj->value;
}

kBeginClass(Go, GoExtParamRegion3d, GoExtParam)
kAddVMethod(GoExtParamRegion3d, kObject, VRelease)
kAddVMethod(GoExtParamRegion3d, GoExtParam, VInit)
kAddVMethod(GoExtParamRegion3d, GoExtParam, VRead)
kAddVMethod(GoExtParamRegion3d, GoExtParam, VWrite)
kEndClass()

GoFx(kStatus) GoExtParamRegion3d_Construct(GoExtParamRegion3d* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamRegion3d), param)); 

    if (!kSuccess(status = GoExtParamRegion3d_VInit(*param, kTypeOf(GoExtParamRegion3d), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param); 
    }

    return status; 
}

GoFx(kStatus) GoExtParamRegion3d_VInit(GoExtParamRegion3d param, kType type, kObject sensor, kAlloc alloc)
{
    GoExtParamRegion3dClass* obj = param; 
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc)); 
    kInitFields_(GoExtParamRegion3d, param);

    kTry
    {
        kTest(GoRegion3d_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamRegion3d_VRelease(param);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParamRegion3d_VRelease(GoExtParamRegion3d param)
{
    GoExtParamRegion3dClass* obj = GoExtParamRegion3d_Cast_(param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamRegion3d_VRead(GoExtParamRegion3d param, kXml xml, kXmlItem item)
{
    GoExtParamRegion3dClass* obj = GoExtParamRegion3d_Cast_(param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoRegion3d_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamRegion3d_VWrite(GoExtParamRegion3d param, kXml xml, kXmlItem item)
{
    GoExtParamRegion3dClass* obj = GoExtParamRegion3d_Cast_(param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoRegion3d_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoRegion3d) GoExtParamRegion3d_Value(GoExtParamRegion3d param)
{
    GoExtParamRegion3dClass* obj = GoExtParamRegion3d_Cast_(param);

    return obj->value;
}

GoFx(kType) GoExtParam_GetKType(GoExtParamType type)
{
    switch(type)
    {
    case GO_EXT_PARAM_TYPE_BOOL: return kTypeOf(GoExtParamBool); break;
    case GO_EXT_PARAM_TYPE_INT: return kTypeOf(GoExtParamInt); break;
    case GO_EXT_PARAM_TYPE_FLOAT: return kTypeOf(GoExtParamFloat); break;
    case GO_EXT_PARAM_TYPE_STRING: return kTypeOf(GoExtParamString); break;
    case GO_EXT_PARAM_TYPE_PROFILE_REGION: return kTypeOf(GoExtParamProfileRegion); break;
    case GO_EXT_PARAM_TYPE_SURFACE_REGION_2D: return kTypeOf(GoExtParamSurfaceRegion2d); break;
    case GO_EXT_PARAM_TYPE_REGION_3D: return kTypeOf(GoExtParamRegion3d); break;        
    default: return kNULL;
    }
}

GoFx(kStatus) GoOptionList_ParseHelperText(const kChar* text, kSize length, kText64 value)
{
    if(length == 0)
    {
        return kERROR_PARAMETER;
    }
    kCheck(length < 64); 
    strncpy(value, text, length); 
    value[length] = 0; 

    return kOK; 
}

GoFx(kStatus) GoExtParamInt_ParseList(kXml xml, kXmlItem item, kArrayList list)
{
    kChar valuesText[2560];
    kChar descriptionsText[2560];
    const kChar* valReadIt = kNULL; 
    const kChar* descReadIt = kNULL; 
    const kChar* valSeparator = kNULL; 
    const kChar* descSeparator = kNULL; 
    GoExtParamIntOption option;

    kCheck(kXml_AttrText(xml, item, "options", valuesText, kCountOf(valuesText)));
    valReadIt = valuesText;

    kCheckArgs(kXml_AttrExists(xml, item, "optionNames"));
    kCheck(kXml_AttrText(xml, item, "optionNames", descriptionsText, kCountOf(descriptionsText)));
    descReadIt = descriptionsText;

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(GoExtParamIntOption), 0)); 

    while (!kIsNull(valSeparator = strchr(valReadIt, ','))
        && !kIsNull(descSeparator = strchr(descReadIt, ',')))
    {
        if (kSuccess(GoOptionList_ParseHelper32u(valReadIt, valSeparator - valReadIt, &option.value))
            && kSuccess(GoOptionList_ParseHelperText(descReadIt, descSeparator - descReadIt, option.description)))
        {
             kCheck(kArrayList_Add(list, &option)); 
        }
        valReadIt = valSeparator + 1; 
        descReadIt = descSeparator + 1;
    }

    if (kSuccess(GoOptionList_ParseHelper32u(valReadIt, strlen(valReadIt), &option.value))
        && kSuccess(GoOptionList_ParseHelperText(descReadIt, strlen(descReadIt), option.description)))
    {
        kCheck(kArrayList_Add(list, &option)); 
    }

    return kOK;    
}

GoFx(kStatus) GoExtParamFloat_ParseList(kXml xml, kXmlItem item, kArrayList list)
{
    kChar valuesText[2560];
    kChar descriptionsText[2560];
    const kChar* valReadIt = kNULL; 
    const kChar* descReadIt = kNULL; 
    const kChar* valSeparator = kNULL; 
    const kChar* descSeparator = kNULL; 
    GoExtParamFloatOption option;

    kCheck(kXml_AttrText(xml, item, "options", valuesText, kCountOf(valuesText)));
    valReadIt = valuesText;

    kCheckArgs(kXml_AttrExists(xml, item, "optionNames"));
    kCheck(kXml_AttrText(xml, item, "optionNames", descriptionsText, kCountOf(descriptionsText)));
    descReadIt = descriptionsText;

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(GoExtParamFloatOption), 0)); 

    while (!kIsNull(valSeparator = strchr(valReadIt, ','))
        && !kIsNull(descSeparator = strchr(descReadIt, ',')))
    {
        if (kSuccess(GoOptionList_ParseHelper64f(valReadIt, valSeparator - valReadIt, &option.value))
            && kSuccess(GoOptionList_ParseHelperText(descReadIt, descSeparator - descReadIt, option.description)))
        {
            kCheck(kArrayList_Add(list, &option)); 
        }
        valReadIt = valSeparator + 1; 
        descReadIt = descSeparator + 1;
    }

    if (kSuccess(GoOptionList_ParseHelper64f(valReadIt, strlen(valReadIt), &option.value))
        && kSuccess(GoOptionList_ParseHelperText(descReadIt, strlen(descReadIt), option.description)))
    {
        kCheck(kArrayList_Add(list, &option)); 
    }

    return kOK;    
}