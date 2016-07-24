/// @cond (Gocator_2x00 || Gocator_3x00)

/**
 * @file    GoPartModel.h
 * @brief   Declares the GoPartModel and GoPartModelEdge class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PART_MODEL_H
#define GO_PART_MODEL_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoPartModelEdge
 * @extends kObject
 * @ingroup GoSdk-Surface
 * @brief   Represents a part model edge configuration.
 */
typedef kObject GoPartModelEdge; 

/**
 * Returns the edge model image type used at the time of model creation.
 *
 * @public             @memberof GoPartModelEdge
 * @version             Introduced in firmware 4.2.4.7
 * @param   edge       GoPartModelEdge object.
 * @return             Edge model image type.
 */
GoFx(GoImageType) GoPartModelEdge_ImageType(GoPartModelEdge edge);  

/**
 * Returns the edge model image data source used at the time of model creation.
 *
 * @public             @memberof GoPartModelEdge
 * @version             Introduced in firmware 4.2.4.7
 * @param   edge       GoPartModelEdge object.
 * @return             Edge model image data source.
 */
GoFx(GoDataSource) GoPartModelEdge_ImageSource(GoPartModelEdge edge);   

/**
 * Returns the count of removed edge model points.
 *
 * @public             @memberof GoPartModelEdge
 * @version             Introduced in firmware 4.2.4.7
 * @param   edge       GoPartModelEdge object.
 * @return             The number of removed edge model points.
 */
GoFx(kSize) GoPartModelEdge_RemovedPointsLength(GoPartModelEdge edge);

/**
 * @class   GoPartModel
 * @extends kObject
 * @ingroup GoSdk-Surface
 * @brief   Represents a part model configuration.
 */
typedef kObject GoPartModel; 


/**
 * Returns the name of the part model.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   model      GoPartModel object.
 * @return             The name of the given part model.
 */
GoFx(const kChar*) GoPartModel_Name(GoPartModel model);

/**
 * Returns the number of edges for the given part model.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   model      GoPartModel object.
 * @return             The number of edges which make up the part model.
 */
GoFx(kSize) GoPartModel_EdgeCount(GoPartModel model);

/**
 * Returns the edge for the given index.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   model      GoPartModel object.
 * @param   index      The index of the part model edge list to access.
 * @return             The edge at the given index.
 */
GoFx(GoPartModelEdge) GoPartModel_EdgeAt(GoPartModel model, kSize index);

/**
 * Returns the edge sensitivity used at the time of model creation.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             The edge sensitivity used at the time of model creation.
 */
GoFx(k64f) GoPartModel_EdgeSensitivity(GoPartModel partModel);

/** 
 * Returns the transformed data region X-component value.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoPartModel_TransformedDataRegionX(GoPartModel partModel);

/** 
 * Returns the transformed data region Y-component value.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoPartModel_TransformedDataRegionY(GoPartModel partModel);

/** 
 * Returns the transformed data region Z-component value.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoPartModel_TransformedDataRegionZ(GoPartModel partModel);

/** 
 * Returns the transformed data region width value.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoPartModel_TransformedDataRegionWidth(GoPartModel partModel);

/** 
 * Returns the transformed data region length value.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoPartModel_TransformedDataRegionLength(GoPartModel partModel);

/** 
 * Returns the transformed data region height value.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoPartModel_TransformedDataRegionHeight(GoPartModel partModel);  //readonly

/**
 * Returns the current Z angle of the given part model.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             Z angle value.
 */
GoFx(k64f) GoPartModel_ZAngle(GoPartModel partModel);

/**
 * Sets the Z angle for the given part model.
 *
 * @public              @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param    partModel  GoPartModel object.
 * @param    value      The value to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoPartModel_SetZAngle(GoPartModel partModel, k64f value);


/**
 * Returns the image type for the given part model.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             The image type.
 */
GoFx(GoImageType) GoPartModel_ImageType(GoPartModel partModel);

/**
 * Sets the image type for the given part model.
 *
 * @public              @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param    partModel  GoPartModel object.
 * @param    value      The image type to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoPartModel_SetImageType(GoPartModel partModel, GoImageType value);

/**
 * Returns the image type option count.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             The image type option count.
 */
GoFx(kSize) GoPartModel_ImageTypeOptionCount(GoPartModel partModel);

/**
 * Returns the image type option at the given index.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @param   index      The image type option index to access.
 * @return             The image type option.
 */
GoFx(GoImageType) GoPartModel_ImageTypeOptionAt(GoPartModel partModel, kSize index);

/**
 * Returns the target edge sensitivity.
 *
 * @public             @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param   partModel  GoPartModel object.
 * @return             The target edge sensitivity.
 */
GoFx(k64f) GoPartModel_TargetEdgeSensitivity(GoPartModel partModel);

/**
 * Sets the target edge sensitivity for the given part model.
 *
 * @public              @memberof GoPartModel
 * @version             Introduced in firmware 4.2.4.7
 * @param    partModel  GoPartModel object.
 * @param    value      The value to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoPartModel_SetTargetEdgeSensitivity(GoPartModel partModel, k64f value);

kEndHeader()
#include <GoSdk/GoPartModel.x.h>

#endif

/// @endcond
