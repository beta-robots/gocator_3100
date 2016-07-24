/** 
 * @file    GoPartModel.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PART_MODEL_X_H
#define GO_PART_MODEL_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoSurfaceTools.h>

kBeginHeader()

#define GO_PART_MODEL_CONFIG_VERSION    (1)

typedef struct GoPartModelEdgeClass
{
    kObjectClass base;
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    kObject partModelParent;

    k32s imageType;
    GoDataSource imageSource;
    kArrayList removedPoints;   //of type k64s
} GoPartModelEdgeClass;

kDeclareClass(Go, GoPartModelEdge, kObject)

#define GoPartModelEdge_Cast_(CONTEXT)    kCastClass_(GoPartModelEdge, CONTEXT)

GoFx(kStatus) GoPartModelEdge_Construct(GoPartModelEdge* part, kObject parentModel, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoPartModelEdge_Init(GoPartModelEdge modelEdge, kType type, kObject parentModel, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoPartModelEdge_VRelease(GoPartModelEdge modelEdge);

GoFx(kStatus) GoPartModelEdge_Read(GoPartModelEdge modelEdge, kXml xml, kXmlItem item);
GoFx(kStatus) GoPartModelEdge_Write(GoPartModelEdge matching, kXml xml, kXmlItem item); 


/**
* Sets the removed points for the given edge model.
*
* @public               @memberof GoPartModelEdge
* @param    edge        GoPartModelEdge object.
* @param    points      An array of edge model point indices to be removed.
* @param    length      The number of edge model point indices in the input array.
* @return               Operation status.
*/
GoFx(kStatus) GoPartModelEdge_SetRemovedPoints(GoPartModelEdge edge, const k32u* points, kSize length);

/**
* Gets the removed points for the given edge model.
*
* @public               @memberof GoPartModelEdge
* @param    edge        GoPartModelEdge object.
* @param    points      An array of edge model point indices to be returned.
* @param    length      The length of the removed points index array.
* @return               Operation status.
*/
GoFx(kStatus) GoPartModelEdge_RemovedPoints(GoPartModelEdge edge, const k32u* points, kSize* length);

/**
* Returns the edge model point index at the given removed point list index.
*
* @public             @memberof GoPartModelEdge
* @param   edge       GoPartModelEdge object.
* @param   index      The index of the removed point array to access.
* @return             Edge model point index.
*/
GoFx(k32u) GoPartModelEdge_RemovedPointAt(GoPartModelEdge edge, kSize index);


typedef struct GoPartModelClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    kText64 name;
    kArrayList edges;   //of type GoPartModelEdge
    GoRegion3d transformedDataRegion;
    k32s imageType;
    kArrayList imageTypeOptions;

    k64f targetEdgeSensitivity;
    k64f edgeSensitivity;
    k64f zAngle;

    kBool isValid;
    kBool isModified;
} GoPartModelClass;

kDeclareClass(Go, GoPartModel, kObject)

#define GoPartModel_Cast_(CONTEXT)    kCastClass_(GoPartModel, CONTEXT)

GoFx(kStatus) GoPartModel_Construct(GoPartModel* partModel, kObject sensor, const kChar* name, kAlloc allocator);;
GoFx(kStatus) GoPartModel_Init(GoPartModel partModel, kType type, kObject sensor, const kChar* name, kAlloc alloc);
GoFx(kStatus) GoPartModel_VRelease(GoPartModel partModel);

GoFx(kStatus) GoPartModel_Read(GoPartModel partModel, kXml xml, kXmlItem item);
GoFx(kStatus) GoPartModel_Write(GoPartModel partModel, kXml xml, kXmlItem item); 

GoFx(kStatus) GoPartModel_SetModified(GoPartModel partModel);

kEndHeader()

#endif
