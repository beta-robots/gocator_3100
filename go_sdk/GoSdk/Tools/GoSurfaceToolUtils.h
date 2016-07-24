/// @cond (Gocator_2x00 || Gocator_3x00)
/** 
 * @file    GoSurfaceToolUtils.h
 * @brief   Declares all surface tools and their related classes. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_SURFACE_TOOL_UTILS_H
#define GO_SURFACE_TOOL_UTILS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoUtils.h>

kBeginHeader()

/**
* @class   GoSurfaceRegion2d
* @extends kObject
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a two dimensional surface tool region.
*/
typedef kObject GoSurfaceRegion2d; 

/** 
 * Sets the X position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    x          The X position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetX(GoSurfaceRegion2d region, k64f x);

/** 
 * Gets the X position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The X position of the region.            
 */
GoFx(k64f) GoSurfaceRegion2d_X(GoSurfaceRegion2d region);

/** 
 * Sets the Y position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    value      The Y position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetY(GoSurfaceRegion2d region, k64f value);

/** 
 * Gets the Y position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The Y position of the region.            
 */
GoFx(k64f) GoSurfaceRegion2d_Y(GoSurfaceRegion2d region);

/** 
 * Sets the width.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    width      The width to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetWidth(GoSurfaceRegion2d region, k64f width);

/** 
 * Gets the width.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The width of the region.            
 */
GoFx(k64f) GoSurfaceRegion2d_Width(GoSurfaceRegion2d region);

/** 
 * Sets the length.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    length     The length to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetLength(GoSurfaceRegion2d region, k64f length);

/** 
 * Gets the length.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The length of the region.            
 */
GoFx(k64f) GoSurfaceRegion2d_Length(GoSurfaceRegion2d region);


/**
* @class   GoRegion3d
* @extends kObject
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a three dimensional surface region.
*/
typedef kObject GoRegion3d; 

/** 
 * Sets the X position.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    x          The X position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetX(GoRegion3d region, k64f x);

/** 
 * Gets the X position.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The X position of the region.            
 */
GoFx(k64f) GoRegion3d_X(GoRegion3d region);

/** 
 * Sets the Y position.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    value      The Y position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetY(GoRegion3d region, k64f value);

/** 
 * Gets the Y position.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The Y position of the region.            
 */
GoFx(k64f) GoRegion3d_Y(GoRegion3d region);

/** 
 * Sets the Z-position.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    z          The Z-position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetZ(GoRegion3d region, k64f z);

/** 
 * Gets the Z-position.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The Z-position of the region.            
 */
GoFx(k64f) GoRegion3d_Z(GoRegion3d region);

/** 
 * Sets the width.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    width      The width to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetWidth(GoRegion3d region, k64f width);

/** 
 * Gets the width.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The width of the region.            
 */
GoFx(k64f) GoRegion3d_Width(GoRegion3d region);

/** 
 * Sets the length.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    length     The length to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetLength(GoRegion3d region, k64f length);

/** 
 * Gets the length.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The length of the region.            
 */
GoFx(k64f) GoRegion3d_Length(GoRegion3d region);

/** 
 * Sets the height.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    height     The height to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetHeight(GoRegion3d region, k64f height);

/** 
 * Gets the height.
 *
 * @public              @memberof GoRegion3d
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The height of the region.            
 */
GoFx(k64f) GoRegion3d_Height(GoRegion3d region);


/**
* @class   GoCylinderRegion
* @extends kObject
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a cylinder region for select surface tools.
*/
typedef kObject GoCylinderRegion; 

/** 
 * Sets the X position.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    x          The X position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetX(GoCylinderRegion region, k64f x);

/** 
 * Gets the X position.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The X position of the region.            
 */
GoFx(k64f) GoCylinderRegion_X(GoCylinderRegion region);

/** 
 * Sets the Y position.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    value      The Y position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetY(GoCylinderRegion region, k64f value);

/** 
 * Gets the Y position.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The Y position of the region.            
 */
GoFx(k64f) GoCylinderRegion_Y(GoCylinderRegion region);

/** 
 * Sets the Z-position.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    z          The Z-position to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetZ(GoCylinderRegion region, k64f z);

/** 
 * Gets the Z-position.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The Z-position of the region.            
 */
GoFx(k64f) GoCylinderRegion_Z(GoCylinderRegion region);

/** 
 * Sets the radius.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    value      The radius to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetRadius(GoCylinderRegion region, k64f value);

/** 
 * Gets the radius.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The radius of the region.            
 */
GoFx(k64f) GoCylinderRegion_Radius(GoCylinderRegion region);

/** 
 * Sets the Height.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    value      The height to set.
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetHeight(GoCylinderRegion region, k64f value);

/** 
 * Gets the Height.
 *
 * @public              @memberof GoCylinderRegion
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The height of the region.            
 */
GoFx(k64f) GoCylinderRegion_Height(GoCylinderRegion region);


/**
* @class   GoSurfaceFeature
* @extends kObject
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a surface feature for select surface tools.
*/
typedef kObject GoSurfaceFeature; 

/** 
 * Gets the surface feature type.
 *
 * @public                  @memberof GoSurfaceFeature
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @return                  The surface feature type.
 */
GoFx(GoSurfaceFeatureType) GoSurfaceFeature_Type(GoSurfaceFeature feature);

/** 
 * Sets the surface feature type.
 *
 * @public                  @memberof GoSurfaceFeature
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @param   type            The feature type value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceFeature_SetType(GoSurfaceFeature feature, GoSurfaceFeatureType type);

/** 
 * Gets the current state the surface feature region.
 *
 * @public                  @memberof GoSurfaceFeature
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceFeature_RegionEnabled(GoSurfaceFeature feature);

/** 
 * Enable or disable the surface feature region.
 *
 * @public                  @memberof GoSurfaceFeature
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @param   enable          kTRUE to enable, or kFALSE to disable.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceFeature_EnableRegion(GoSurfaceFeature feature, kBool enable);

/** 
 * Gets the 3d region for the feature.
 *
 * @public                  @memberof GoSurfaceFeature
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceFeature_Region(GoSurfaceFeature feature);


kEndHeader()
#include <GoSdk/Tools/GoSurfaceToolUtils.x.h>

#endif

/// @endcond
