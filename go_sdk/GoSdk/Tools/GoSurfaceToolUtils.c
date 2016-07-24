/** 
 * @file    GoSurfaceToolUtils.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <GoSdk/GoSensor.h>


kBeginClass(Go, GoCylinderRegion, kObject)
kEndClass()

GoFx(kStatus) GoCylinderRegion_Construct(GoCylinderRegion* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoCylinderRegion), region)); 

    if (!kSuccess(status = GoCylinderRegion_Init(*region, kTypeOf(GoCylinderRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region); 
    }

    return status; 
} 

GoFx(kStatus) GoCylinderRegion_Init(GoCylinderRegion region, kType type, kObject sensor, kAlloc alloc)
{
    GoCylinderRegionClass* obj = region;

    kCheck(kObject_Init(region, type, alloc)); 
    kInitFields_(GoCylinderRegion, region);

    obj->sensor = sensor; 

    obj->radius = 0.5;
    obj->height = 1.0;

    return kOK; 
}

GoFx(kStatus) GoCylinderRegion_Read(GoCylinderRegion region, kXml xml, kXmlItem item)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheck(kXml_Child64f(xml, item, "X", &obj->x)); 
    kCheck(kXml_Child64f(xml, item, "Y", &obj->y)); 
    kCheck(kXml_Child64f(xml, item, "Z", &obj->z)); 
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius)); 
    kCheck(kXml_Child64f(xml, item, "Height", &obj->height));

    return kOK; 
}

GoFx(kStatus) GoCylinderRegion_Write(GoCylinderRegion region, kXml xml, kXmlItem item)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x)); 
    kCheck(kXml_SetChild64f(xml, item, "Y", obj->y)); 
    kCheck(kXml_SetChild64f(xml, item, "Z", obj->z)); 
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius)); 
    kCheck(kXml_SetChild64f(xml, item, "Height", obj->height));

    return kOK; 
}


GoFx(kStatus) GoCylinderRegion_SetX(GoCylinderRegion region, k64f x)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoCylinderRegion_X(GoCylinderRegion region)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x; 
}

GoFx(kStatus) GoCylinderRegion_SetY(GoCylinderRegion region, k64f y)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->y = y;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoCylinderRegion_Y(GoCylinderRegion region)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->y; 
}

GoFx(kStatus) GoCylinderRegion_SetZ(GoCylinderRegion region, k64f z)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->z = z;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoCylinderRegion_Z(GoCylinderRegion region)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->z; 
}

GoFx(kStatus) GoCylinderRegion_SetRadius(GoCylinderRegion region, k64f radius)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->radius = radius;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;   
}

GoFx(k64f) GoCylinderRegion_Radius(GoCylinderRegion region)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->radius; 
}

GoFx(kStatus) GoCylinderRegion_SetHeight(GoCylinderRegion region, k64f height)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->height = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoCylinderRegion_Height(GoCylinderRegion region)
{
    GoCylinderRegionClass* obj = GoCylinderRegion_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->height; 
}


kBeginClass(Go, GoSurfaceRegion2d, kObject)
kEndClass()

GoFx(kStatus) GoSurfaceRegion2d_Construct(GoSurfaceRegion2d* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceRegion2d), region)); 

    if (!kSuccess(status = GoSurfaceRegion2d_Init(*region, kTypeOf(GoSurfaceRegion2d), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region); 
    }

    return status; 
} 

GoFx(kStatus) GoSurfaceRegion2d_Init(GoSurfaceRegion2d region, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceRegion2dClass* obj = region;

    kCheck(kObject_Init(region, type, alloc)); 
    kInitFields_(GoSurfaceRegion2d, region);

    obj->sensor = sensor; 

    obj->x = 0;
    obj->y = 0;
    obj->width = 1.0;
    obj->length = 1.0;

    return kOK; 
}

GoFx(kStatus) GoSurfaceRegion2d_Read(GoSurfaceRegion2d region, kXml xml, kXmlItem item)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    kCheck(kXml_Child64f(xml, item, "X", &obj->x)); 
    kCheck(kXml_Child64f(xml, item, "Y", &obj->y)); 
    kCheck(kXml_Child64f(xml, item, "Width", &obj->width)); 
    kCheck(kXml_Child64f(xml, item, "Length", &obj->length)); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceRegion2d_Write(GoSurfaceRegion2d region, kXml xml, kXmlItem item)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x)); 
    kCheck(kXml_SetChild64f(xml, item, "Y", obj->y)); 
    kCheck(kXml_SetChild64f(xml, item, "Width", obj->width)); 
    kCheck(kXml_SetChild64f(xml, item, "Length", obj->length)); 

    return kOK; 
}


GoFx(kStatus) GoSurfaceRegion2d_SetX(GoSurfaceRegion2d region, k64f x)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoSurfaceRegion2d_X(GoSurfaceRegion2d region)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x; 
}

GoFx(kStatus) GoSurfaceRegion2d_SetY(GoSurfaceRegion2d region, k64f y)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->y = y;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoSurfaceRegion2d_Y(GoSurfaceRegion2d region)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->y; 
}

GoFx(kStatus) GoSurfaceRegion2d_SetWidth(GoSurfaceRegion2d region, k64f width)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->width = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;   
}

GoFx(k64f) GoSurfaceRegion2d_Width(GoSurfaceRegion2d region)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->width; 
}

GoFx(kStatus) GoSurfaceRegion2d_SetLength(GoSurfaceRegion2d region, k64f length)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->length = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoSurfaceRegion2d_Length(GoSurfaceRegion2d region)
{
    GoSurfaceRegion2dClass* obj = GoSurfaceRegion2d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->length; 
}


kBeginClass(Go, GoRegion3d, kObject)
kEndClass()

GoFx(kStatus) GoRegion3d_Construct(GoRegion3d* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoRegion3d), region)); 

    if (!kSuccess(status = GoRegion3d_Init(*region, kTypeOf(GoRegion3d), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region); 
    }

    return status; 
} 

GoFx(kStatus) GoRegion3d_Init(GoRegion3d region, kType type, kObject sensor, kAlloc alloc)
{
    GoRegion3dClass* obj = region;

    kCheck(kObject_Init(region, type, alloc)); 
    kInitFields_(GoRegion3d, region);

    obj->sensor = sensor; 

    obj->width = 1.0;
    obj->height = 1.0;
    obj->length = 1.0;

    return kOK; 
}

GoFx(kStatus) GoRegion3d_Read(GoRegion3d region, kXml xml, kXmlItem item)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheck(kXml_Child64f(xml, item, "X", &obj->x)); 
    kCheck(kXml_Child64f(xml, item, "Y", &obj->y)); 
    kCheck(kXml_Child64f(xml, item, "Z", &obj->z)); 
    kCheck(kXml_Child64f(xml, item, "Width", &obj->width)); 
    kCheck(kXml_Child64f(xml, item, "Length", &obj->length)); 
    kCheck(kXml_Child64f(xml, item, "Height", &obj->height));

    return kOK; 
}

GoFx(kStatus) GoRegion3d_Write(GoRegion3d region, kXml xml, kXmlItem item)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x)); 
    kCheck(kXml_SetChild64f(xml, item, "Y", obj->y)); 
    kCheck(kXml_SetChild64f(xml, item, "Z", obj->z)); 
    kCheck(kXml_SetChild64f(xml, item, "Width", obj->width)); 
    kCheck(kXml_SetChild64f(xml, item, "Length", obj->length)); 
    kCheck(kXml_SetChild64f(xml, item, "Height", obj->height));

    return kOK; 
}


GoFx(kStatus) GoRegion3d_SetX(GoRegion3d region, k64f x)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoRegion3d_X(GoRegion3d region)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x; 
}

GoFx(kStatus) GoRegion3d_SetY(GoRegion3d region, k64f y)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->y = y;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoRegion3d_Y(GoRegion3d region)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->y; 
}

GoFx(kStatus) GoRegion3d_SetZ(GoRegion3d region, k64f z)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->z = z;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoRegion3d_Z(GoRegion3d region)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->z; 
}

GoFx(kStatus) GoRegion3d_SetWidth(GoRegion3d region, k64f width)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->width = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;   
}

GoFx(k64f) GoRegion3d_Width(GoRegion3d region)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->width; 
}

GoFx(kStatus) GoRegion3d_SetLength(GoRegion3d region, k64f length)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->length = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoRegion3d_Length(GoRegion3d region)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->length; 
}

GoFx(kStatus) GoRegion3d_SetHeight(GoRegion3d region, k64f height)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->height = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoRegion3d_Height(GoRegion3d region)
{
    GoRegion3dClass* obj = GoRegion3d_Cast_(region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->height; 
}



kBeginClass(Go, GoSurfaceFeature, kObject)
    kAddVMethod(GoSurfaceFeature, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSurfaceFeature_Construct(GoSurfaceFeature* feature, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceFeature), feature)); 

    if (!kSuccess(status = GoSurfaceFeature_Init(*feature, kTypeOf(GoSurfaceFeature), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, feature); 
    }

    return status; 
} 

GoFx(kStatus) GoSurfaceFeature_Init(GoSurfaceFeature feature, kType type, kObject sensor, kAlloc alloc)
{
    GoSurfaceFeatureClass* obj = feature;
    kStatus exception;

    kCheck(kObject_Init(feature, type, alloc)); 
    kInitFields_(GoSurfaceFeature, feature);

    obj->sensor = sensor; 
    obj->regionEnabled = kTRUE;

    kTry
    {
        kTest(GoRegion3d_Construct(&obj->region, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoSurfaceFeature_VRelease(feature);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoSurfaceFeature_VRelease(GoSurfaceFeature feature)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);

    kCheck(kDestroyRef(&obj->region));

    return kObject_VRelease(feature);
}

GoFx(kStatus) GoSurfaceFeature_Read(GoSurfaceFeature feature, kXml xml, kXmlItem item)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "Type", &obj->type)); 
    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled)); 
    
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    return kOK; 
}

GoFx(kStatus) GoSurfaceFeature_Write(GoSurfaceFeature feature, kXml xml, kXmlItem item)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type)); 
    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled)); 

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    return kOK; 
}

GoFx(GoSurfaceFeatureType) GoSurfaceFeature_Type(GoSurfaceFeature feature)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);

    return obj->type;
}

GoFx(kStatus) GoSurfaceFeature_SetType(GoSurfaceFeature feature, GoSurfaceFeatureType type)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);

    obj->type = type;

    return kOK;
}

GoFx(kBool) GoSurfaceFeature_RegionEnabled(GoSurfaceFeature feature)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceFeature_EnableRegion(GoSurfaceFeature feature, kBool enable)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);

    obj->regionEnabled = enable;

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceFeature_Region(GoSurfaceFeature feature)
{
    GoSurfaceFeatureClass* obj = GoSurfaceFeature_Cast_(feature);

    return obj->region;
}

