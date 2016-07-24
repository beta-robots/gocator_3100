/** 
 * @file    GoProfileToolUtils.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoProfileRegion, kObject)
kEndClass()

GoFx(kStatus) GoProfileRegion_Construct(GoProfileRegion* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileRegion), region)); 

    if (!kSuccess(status = GoProfileRegion_Init(*region, kTypeOf(GoProfileRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region); 
    }

    return status; 
} 

GoFx(kStatus) GoProfileRegion_Init(GoProfileRegion region, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileRegionClass* obj = region; 

    kCheck(kObject_Init(region, type, alloc)); 
    kInitFields_(GoProfileRegion, region);

    obj->sensor = sensor; 

    obj->x = 0;
    obj->z = 0;
    obj->width = 1.0;
    obj->height = 1.0;

    return kOK; 
}

GoFx(kStatus) GoProfileRegion_Read(GoProfileRegion region, kXml xml, kXmlItem item)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    kCheck(kXml_Child64f(xml, item, "X", &obj->x)); 
    kCheck(kXml_Child64f(xml, item, "Z", &obj->z)); 
    kCheck(kXml_Child64f(xml, item, "Width", &obj->width)); 
    kCheck(kXml_Child64f(xml, item, "Height", &obj->height));

    return kOK; 
}

GoFx(kStatus) GoProfileRegion_Write(GoProfileRegion region, kXml xml, kXmlItem item)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x)); 
    kCheck(kXml_SetChild64f(xml, item, "Z", obj->z)); 
    kCheck(kXml_SetChild64f(xml, item, "Width", obj->width)); 
    kCheck(kXml_SetChild64f(xml, item, "Height", obj->height));

    return kOK; 
}


GoFx(kStatus) GoProfileRegion_SetX(GoProfileRegion region, k64f x)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoProfileRegion_X(GoProfileRegion region)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->x; 
}

GoFx(kStatus) GoProfileRegion_SetZ(GoProfileRegion region, k64f z)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->z = z;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK; 
}

GoFx(k64f) GoProfileRegion_Z(GoProfileRegion region)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->z; 
}

GoFx(kStatus) GoProfileRegion_SetWidth(GoProfileRegion region, k64f width)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->width = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;   
}

GoFx(k64f) GoProfileRegion_Width(GoProfileRegion region)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->width; 
}

GoFx(kStatus) GoProfileRegion_SetHeight(GoProfileRegion region, k64f height)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->height = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(k64f) GoProfileRegion_Height(GoProfileRegion region)
{
    GoProfileRegionClass* obj = GoProfileRegion_Cast_(region); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->height; 
}


kBeginClass(Go, GoProfileFeature, kObject)
    kAddVMethod(GoProfileFeature, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoProfileFeature_Construct(GoProfileFeature* feature, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileFeature), feature)); 

    if (!kSuccess(status = GoProfileFeature_Init(*feature, kTypeOf(GoProfileFeature), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, feature); 
    }

    return status; 
} 

GoFx(kStatus) GoProfileFeature_Init(GoProfileFeature feature, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileFeatureClass* obj = feature; 
    kStatus exception;

    kCheck(kObject_Init(feature, type, alloc)); 
    kInitFields_(GoProfileFeature, feature);

    obj->sensor = sensor; 
    obj->type = GO_PROFILE_FEATURE_TYPE_MAX_Z;

    kTry
    {
        kTest(GoProfileRegion_Construct(&obj->region, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoProfileFeature_VRelease(feature);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoProfileFeature_VRelease(GoProfileFeature feature)
{
    GoProfileFeatureClass* obj = GoProfileFeature_Cast_(feature);

    kCheck(kDestroyRef(&obj->region));

    return kObject_VRelease(feature);
}

GoFx(kStatus) GoProfileFeature_Read(GoProfileFeature feature, kXml xml, kXmlItem item)
{
    GoProfileFeatureClass* obj = GoProfileFeature_Cast_(feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "Type", (k32s*) &obj->type)); 

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK; 
}

GoFx(kStatus) GoProfileFeature_Write(GoProfileFeature feature, kXml xml, kXmlItem item)
{
    GoProfileFeatureClass* obj = GoProfileFeature_Cast_(feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type)); 

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    return kOK; 
}


GoFx(kStatus) GoProfileFeature_SetType(GoProfileFeature feature, GoProfileFeatureType type)
{
    GoProfileFeatureClass* obj = GoProfileFeature_Cast_(feature);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;     
}

GoFx(GoProfileFeatureType) GoProfileFeature_Type(GoProfileFeature feature)
{
    GoProfileFeatureClass* obj = GoProfileFeature_Cast_(feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->type; 
}

GoFx(GoProfileRegion) GoProfileFeature_Region(GoProfileFeature feature)
{
    GoProfileFeatureClass* obj = GoProfileFeature_Cast_(feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->region; 
}


kBeginClass(Go, GoProfileLineRegion, kObject)
    kAddVMethod(GoProfileLineRegion, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoProfileLineRegion_Construct(GoProfileLineRegion* lineRegion, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileLineRegion), lineRegion)); 

    if (!kSuccess(status = GoProfileLineRegion_Init(*lineRegion, kTypeOf(GoProfileLineRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, lineRegion); 
    }

    return status; 
} 

GoFx(kStatus) GoProfileLineRegion_Init(GoProfileLineRegion lineRegion, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileLineRegionClass* obj = lineRegion;
    k32u i;
    kStatus exception;

    kCheck(kObject_Init(lineRegion, type, alloc)); 
    kInitFields_(GoProfileLineRegion, lineRegion);

    obj->sensor = sensor;     
    obj->regionCount = 1;

    kTry
    {
        for (i = 0; i < kCountOf(obj->regions); ++i)
        {
            kTest(GoProfileRegion_Construct(&obj->regions[i], sensor, alloc));     //max number of regions constructed regardless of region count
        }
    }
    kCatch(&exception)
    {
        GoProfileLineRegion_VRelease(lineRegion);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoProfileLineRegion_VRelease(GoProfileLineRegion lineRegion)
{
    GoProfileLineRegionClass* obj = GoProfileLineRegion_Cast_(lineRegion);
    k32u i;

    for (i = 0; i < kCountOf(obj->regions); ++i)
    {
        kCheck(kDestroyRef(&obj->regions[i])); 
    }
    
    return kObject_VRelease(lineRegion); 
}

GoFx(kStatus) GoProfileLineRegion_Read(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item)
{
    GoProfileLineRegionClass* obj = GoProfileLineRegion_Cast_(lineRegion);
    kXmlItem regionItem = kNULL;
    kXmlItem regionsItem = kNULL;

    kCheck(!kIsNull(regionsItem = kXml_Child(xml, item, "Regions")));
    
    kCheck(!kIsNull(regionItem = kXml_Child(xml, regionsItem, "Region")));
    kCheck(GoProfileRegion_Read(obj->regions[0], xml, regionItem));
    obj->regionCount = 1;

    
    if(!kIsNull(regionItem = kXml_NextSibling(xml, regionItem)) &&
        (strcmp(kXml_ItemName(xml, regionItem), "Region") == 0))
    {
        kCheck(GoProfileRegion_Read(obj->regions[1], xml, regionItem));
        obj->regionCount++;
    }

    return kOK; 
}

GoFx(kStatus) GoProfileLineRegion_Write(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item)
{
    GoProfileLineRegionClass* obj = GoProfileLineRegion_Cast_(lineRegion);
    kXmlItem regionItem = kNULL;
    kXmlItem regionsItem = kNULL;
    k32u i;

    kCheck(kXml_SetChild32u(xml, item, "RegionCount", obj->regionCount));
    kCheck(kXml_AddItem(xml, item, "Regions", &regionsItem));
    for (i = 0; i < kCountOf(obj->regions); ++i)
    {
        kCheck(kXml_AddItem(xml, regionsItem, "Region", &regionItem));
        kCheck(GoProfileRegion_Write(obj->regions[i], xml, regionItem));
    }

    return kOK; 
}

GoFx(k32u) GoProfileLineRegion_RegionCount(GoProfileLineRegion lineRegion)
{
    GoProfileLineRegionClass* obj = GoProfileLineRegion_Cast_(lineRegion);

    GoSensor_SyncConfig(obj->sensor);

    return obj->regionCount; 
}

GoFx(GoProfileRegion) GoProfileLineRegion_RegionAt(GoProfileLineRegion lineRegion, kSize index)
{
    GoProfileLineRegionClass* obj = GoProfileLineRegion_Cast_(lineRegion);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoProfileLineRegion_RegionCount(lineRegion));

    return obj->regions[index]; 
} 


kBeginClass(Go, GoProfileEdge, kObject)
    kAddVMethod(GoProfileEdge, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoProfileEdge_Construct(GoProfileEdge* edge, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileEdge), edge)); 

    if (!kSuccess(status = GoProfileEdge_Init(*edge, kTypeOf(GoProfileEdge), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, edge); 
    }

    return status; 
} 

GoFx(kStatus) GoProfileEdge_Init(GoProfileEdge edge, kType type, kObject sensor, kAlloc alloc)
{
    GoProfileEdgeClass* obj = edge; 
    kStatus exception;

    kCheck(kObject_Init(edge, type, alloc)); 
    kInitFields_(GoProfileEdge, edge);

    obj->sensor = sensor; 

    obj->surfaceWidth = 5.0;
    obj->surfaceOffset = 2.0;
    obj->nominalRadius = 2.0;
    obj->edgeAngle = 90.0;

    kTry
    {
        kTest(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 
    }
    kCatch(&exception)
    {
        GoProfileEdge_VRelease(edge);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoProfileEdge_VRelease(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge); 

    kCheck(kDestroyRef(&obj->region)); 

    return kObject_VRelease(edge);
}

GoFx(kStatus) GoProfileEdge_Read(GoProfileEdge edge, kXml xml, kXmlItem item)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge); 
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "MaxVoidWidth", &obj->maxVoidWidth)); 
    kCheck(kXml_Child64f(xml, item, "MinDepth", &obj->minDepth));
    kCheck(kXml_Child64f(xml, item, "SurfaceWidth", &obj->surfaceWidth)); 
    kCheck(kXml_Child64f(xml, item, "SurfaceOffset", &obj->surfaceOffset));
    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));
    kCheck(kXml_Child64f(xml, item, "EdgeAngle", &obj->edgeAngle));
    kCheck(kXml_Child32s(xml, item, "EdgeType", (k32s*) &obj->edgeType)); 

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK; 
}

GoFx(kStatus) GoProfileEdge_Write(GoProfileEdge edge, kXml xml, kXmlItem item)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge); 
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild64f(xml, item, "MaxVoidWidth", obj->maxVoidWidth)); 
    kCheck(kXml_SetChild64f(xml, item, "MinDepth", obj->minDepth)); 
    kCheck(kXml_SetChild64f(xml, item, "SurfaceWidth", obj->surfaceWidth)); 
    kCheck(kXml_SetChild64f(xml, item, "SurfaceOffset", obj->surfaceOffset)); 
    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius)); 
    kCheck(kXml_SetChild64f(xml, item, "EdgeAngle", obj->edgeAngle)); 
    kCheck(kXml_SetChild32s(xml, item, "EdgeType", obj->edgeType)); 

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    return kOK; 
}

GoFx(kStatus) GoProfileEdge_SetType(GoProfileEdge edge, GoProfileEdgeType type)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->edgeType = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileEdgeType) GoProfileEdge_Type(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeType; 
}

GoFx(GoProfileRegion) GoProfileEdge_Region(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->region;
}

GoFx(kStatus) GoProfileEdge_SetVoidWidthMax(GoProfileEdge edge, k64f width)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->maxVoidWidth = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_VoidWidthMax(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->maxVoidWidth; 
}

GoFx(kStatus) GoProfileEdge_SetDepthMin(GoProfileEdge edge, k64f depth)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->minDepth = depth;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_DepthMin(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minDepth; 
}

GoFx(kStatus) GoProfileEdge_SetSurfaceWidth(GoProfileEdge edge, k64f width)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->surfaceWidth = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_SurfaceWidth(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->surfaceWidth; 
}

GoFx(kStatus) GoProfileEdge_SetSurfaceOffset(GoProfileEdge edge, k64f offset)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->surfaceOffset = offset;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_SurfaceOffset(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->surfaceOffset; 
}

GoFx(kStatus) GoProfileEdge_SetNominalRadius(GoProfileEdge edge, k64f radius)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->nominalRadius = radius;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_NominalRadius(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->nominalRadius; 
}

GoFx(kStatus) GoProfileEdge_SetEdgeAngle(GoProfileEdge edge, k64f angle)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->edgeAngle = angle;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_EdgeAngle(GoProfileEdge edge)
{
    GoProfileEdgeClass* obj = GoProfileEdge_Cast_(edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeAngle; 
}
