/** 
 * @file    GoPartModel.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoPartModel.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoPartModelEdge, kObject)
    kAddVMethod(GoPartModelEdge, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoPartModelEdge_Construct(GoPartModelEdge* partModelEdge, kObject parentModel, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPartModelEdge), partModelEdge)); 

    if (!kSuccess(status = GoPartModelEdge_Init(*partModelEdge, kTypeOf(GoPartModelEdge), parentModel, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, partModelEdge); 
    }

    return status; 
} 

GoFx(kStatus) GoPartModelEdge_Init(GoPartModelEdge partModelEdge, kType type, kObject parentModel, kObject sensor, kAlloc alloc)
{
    GoPartModelEdgeClass* obj = partModelEdge; 
    kStatus exception = kOK;

    kCheck(kObject_Init(partModelEdge, type, alloc)); 
    kInitFields_(GoPartModelEdge, partModelEdge);

    obj->partModelParent = parentModel;
    obj->sensor = sensor; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->removedPoints, kTypeOf(k32u), 0, alloc));
    }
    kCatch(&exception)
    {
        GoPartModelEdge_VRelease(partModelEdge);
        kEndCatch(&exception);
    }

    return kOK; 
}

GoFx(kStatus) GoPartModelEdge_VRelease(GoPartModelEdge partModelEdge)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(partModelEdge); 

    kDisposeRef(&obj->removedPoints);

    return kObject_VRelease(partModelEdge); 
}

GoFx(kStatus) GoPartModelEdge_Read(GoPartModelEdge modelEdge, kXml xml, kXmlItem item)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(modelEdge);
    kXmlItem enabledItem = kNULL;
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kText256 tempText;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_Child32s(xml, item, "ImageType", &obj->imageType));
    kCheck(kXml_Child32s(xml, item, "ImageSource", &obj->imageSource));
    kCheck(kXml_ChildText(xml, item, "RemovedPoints", tempText, kCountOf(tempText)));

    kCheck(GoOptionList_ParseList32u(tempText, obj->removedPoints));

    return kOK;
}

GoFx(kStatus) GoPartModelEdge_Write(GoPartModelEdge partModelEdge, kXml xml, kXmlItem item)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(partModelEdge); 
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kText256 tempText;

    kCheck(GoOptionList_Format32u(kArrayList_Data(obj->removedPoints), kArrayList_Count(obj->removedPoints), tempText, kCountOf(tempText)));
    kCheck(kXml_SetChildText(xml, item, "RemovedPoints", tempText));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(GoImageType) GoPartModelEdge_ImageType(GoPartModelEdge edge)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(edge); 

    kCheck(GoSensor_SyncPartModels(obj->sensor));

    return obj->imageType;
}

GoFx(GoDataSource) GoPartModelEdge_ImageSource(GoPartModelEdge edge)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(edge); 

    kCheck(GoSensor_SyncPartModels(obj->sensor));

    return obj->imageSource;
}

GoFx(kStatus) GoPartModelEdge_SetRemovedPoints(GoPartModelEdge edge, const k32u* points, kSize length)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(edge);
    kSize i;

    kCheck(kArrayList_Purge(obj->removedPoints));

    for (i = 0; i < length; i++)
    {
        kCheck(kArrayList_Add(obj->removedPoints, &points[i]));
    }

    kCheck(GoPartModel_SetModified(obj->partModelParent));

    return kOK;
}

GoFx(kStatus) GoPartModelEdge_RemovedPoints(GoPartModelEdge edge, const k32u* points, kSize* length)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(edge); 

    GoSensor_SyncPartModels(obj->sensor);

    points = kArrayList_Data(obj->removedPoints);
    *length = kArrayList_Count(obj->removedPoints);

    return kOK;
}

GoFx(kSize) GoPartModelEdge_RemovedPointsLength(GoPartModelEdge edge)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(edge); 

    GoSensor_SyncPartModels(obj->sensor);

    return kArrayList_Count(obj->removedPoints);
}

GoFx(k32u) GoPartModelEdge_RemovedPointAt(GoPartModelEdge edge, kSize index)
{
    GoPartModelEdgeClass* obj = GoPartModelEdge_Cast_(edge); 

    GoSensor_SyncPartModels(obj->sensor);

    kAssert(index < GoPartModelEdge_RemovedPointsLength(edge));

    return *(k32u*)kArrayList_At(obj->removedPoints, index);
}


kBeginClass(Go, GoPartModel, kObject)
    kAddVMethod(GoPartModel, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoPartModel_Construct(GoPartModel* partModel, kObject sensor, const kChar* name, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPartModel), partModel)); 

    if (!kSuccess(status = GoPartModel_Init(*partModel, kTypeOf(GoPartModel), sensor, name, alloc)))
    {
        kAlloc_FreeRef(alloc, partModel); 
    }

    return status; 
} 

GoFx(kStatus) GoPartModel_Init(GoPartModel partModel, kType type, kObject sensor, const kChar* name, kAlloc alloc)
{
    GoPartModelClass* obj = partModel; 
    kStatus exception = kOK;

    kCheck(kObject_Init(partModel, type, alloc)); 
    kInitFields_(GoPartModel, partModel);

    kCheck(kStrCopy(obj->name, 64, name));
    obj->sensor = sensor; 
    
    kTry
    {
        kTest(kArrayList_Construct(&obj->edges, kTypeOf(GoPartModelEdge), 0, alloc));
        kTest(kArrayList_Construct(&obj->imageTypeOptions, kTypeOf(k32s), 0, alloc));
        kTest(GoRegion3d_Construct(&obj->transformedDataRegion, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoPartModel_VRelease(partModel);
        kEndCatch(&exception);
    }

    return kOK; 
}

GoFx(kStatus) GoPartModel_VRelease(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel); 

    kDestroyRef(&obj->xml);
    kDestroyRef(&obj->transformedDataRegion);
    kDisposeRef(&obj->edges);
    kDestroyRef(&obj->imageTypeOptions);

    return kObject_VRelease(partModel); 
}

GoFx(kStatus) GoPartModel_Read(GoPartModel model, kXml xml, kXmlItem item)
{
    GoPartModelClass* obj = GoPartModel_Cast_(model);
    kXmlItem enabledItem = kNULL;
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kSize i;
    kText256 tempText;
    GoRegion3dClass* region = GoRegion3d_Cast_(obj->transformedDataRegion);
    kStatus exception = kOK;
    GoPartModelEdge modelEdge;

    kCheck(kDestroyRef(&obj->xml));

    obj->xml = xml;
    obj->xmlItem = item;

    tempItem = kXml_Child(xml, item, "Edges");

    kTry
    {
        for (i = 0; i < kXml_ChildCount(xml, tempItem); i++)
        {
            tempItemInner = kXml_ChildAt(xml, tempItem, i);

            if (kArrayList_Count(obj->edges) <= i)
            {
                kTest(GoPartModelEdge_Construct(&modelEdge, model, obj->sensor, kObject_Alloc(model)));
                kTest(kArrayList_Add(obj->edges, &modelEdge));
            }
            else
            {
                modelEdge = *(GoPartModelEdge*)kArrayList_At(obj->edges, i);
            }

            kTestArgs(strcmp("Edge", kXml_ItemName(xml, tempItemInner)) == 0);            
            kTest(GoPartModelEdge_Read(modelEdge, xml, tempItemInner));
            
        }
    }
    kCatch(&exception)
    {
        kDestroyRef(&modelEdge);
        kEndCatch(exception);
    }

    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/X", &region->x));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Y", &region->y));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Z", &region->z));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Width", &region->width));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Length", &region->length));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Height", &region->height));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "ImageType")));
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_ParseList32u(tempText, obj->imageTypeOptions));
    kCheck(kXml_Item32s(xml, tempItem, &obj->imageType));

    kCheck(kXml_Child64f(xml, item, "TargetEdgeSensitivity", &obj->targetEdgeSensitivity));
    kCheck(kXml_Child64f(xml, item, "EdgeSensitivity", &obj->edgeSensitivity));
    kCheck(kXml_Child64f(xml, item, "ZAngle", &obj->zAngle));

    obj->isModified = kFALSE;
    obj->isValid = kTRUE; 

    return kOK;
}

GoFx(kStatus) GoPartModel_Write(GoPartModel partModel, kXml xml, kXmlItem item)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel); 
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kSize i;

    kCheck(kXml_AddItem(xml, item, "Edges", &tempItem));
    
    for (i = 0; i < GoPartModel_EdgeCount(partModel); i++)
    {
        kCheck(kXml_AddItem(xml, tempItem, "Edge", &tempItemInner));
        kCheck(GoPartModelEdge_Write(GoPartModel_EdgeAt(partModel, i), xml, tempItemInner));
    }

    kCheck(kXml_SetChild32s(xml, item, "ImageType", obj->imageType));
    kCheck(kXml_SetChild64f(xml, item, "TargetEdgeSensitivity", obj->targetEdgeSensitivity));
    kCheck(kXml_SetChild64f(xml, item, "EdgeSensitivity", obj->edgeSensitivity));
    kCheck(kXml_SetChild64f(xml, item, "ZAngle", obj->zAngle));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(const kChar*) GoPartModel_Name(GoPartModel model)
{
    GoPartModelClass* obj = GoPartModel_Cast_(model);

    return obj->name;
}

GoFx(kSize) GoPartModel_EdgeCount(GoPartModel model)
{
    GoPartModelClass* obj = GoPartModel_Cast_(model);

    GoSensor_SyncPartModels(obj->sensor);

    return kArrayList_Count(obj->edges);
}

GoFx(GoPartModelEdge) GoPartModel_EdgeAt(GoPartModel model, kSize index)
{
    GoPartModelClass* obj = GoPartModel_Cast_(model);

    GoSensor_SyncPartModels(obj->sensor);

    kAssert(index < GoPartModel_EdgeCount(model));

    return *(GoPartModelEdge*)kArrayList_At(obj->edges, index);
}

GoFx(k64f) GoPartModel_TransformedDataRegionX(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_X(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionY(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Y(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionZ(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Z(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionWidth(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Width(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionLength(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Length(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionHeight(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Height(obj->transformedDataRegion);
}

GoFx(GoImageType) GoPartModel_ImageType(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->imageType;
}

GoFx(kStatus) GoPartModel_SetImageType(GoPartModel partModel, GoImageType value)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    obj->imageType = value;
    kCheck(GoPartModel_SetModified(partModel));

    return kOK;
}

GoFx(GoImageType) GoPartModel_ImageTypeOptionAt(GoPartModel partModel, kSize index)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    kCheckArgs(index < kArrayList_Count(obj->imageTypeOptions));
    
    return *(GoImageType*)kArrayList_At(obj->imageTypeOptions, index);
}

GoFx(kSize) GoPartModel_ImageTypeOptionCount(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return kArrayList_Count(obj->imageTypeOptions);
}

GoFx(k64f) GoPartModel_TargetEdgeSensitivity(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->targetEdgeSensitivity;
}

GoFx(kStatus) GoPartModel_SetTargetEdgeSensitivity(GoPartModel partModel, k64f value)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    obj->targetEdgeSensitivity = value;
    kCheck(GoPartModel_SetModified(partModel));

    return kOK;
}

GoFx(k64f) GoPartModel_EdgeSensitivity(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->edgeSensitivity;
}

GoFx(k64f) GoPartModel_ZAngle(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->zAngle;
}

GoFx(kStatus) GoPartModel_SetZAngle(GoPartModel partModel, k64f value)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    obj->zAngle = value;
    kCheck(GoPartModel_SetModified(partModel));

    return kOK;
}

GoFx(kStatus) GoPartModel_SetModified(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    obj->isModified = kTRUE;

    return kOK;
}

GoFx(kBool) GoPartModel_IsModified(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    return obj->isModified;
}

GoFx(kBool) GoPartModel_IsValid(GoPartModel partModel)
{
    GoPartModelClass* obj = GoPartModel_Cast_(partModel);

    return obj->isValid;
}