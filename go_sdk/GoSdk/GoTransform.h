/** 
 * @file    GoTransform.h
 * @brief   Declares the GoTransform class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TRANSFORM_H
#define GO_TRANSFORM_H

#include <GoSdk/GoSdkDef.h>

kBeginHeader()

/**
 * @class   GoTransform
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a sensor transformation.
 */
typedef kObject GoTransform; 

/** 
 * Gets the encoder resolution.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @return                  The encoder resolution in mm/tick.
 */
GoFx(k64f) GoTransform_EncoderResolution(GoTransform transform);

/** 
 * Sets the encoder resolution.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    value       Encoder resolution in mm/tick.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetEncoderResolution(GoTransform transform, k64f value);

/** 
 * Gets the encoder speed.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @return                  The encoder speed in mm/sec.
 */
GoFx(k64f) GoTransform_Speed(GoTransform transform);

/** 
 * Sets the encoder speed.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    value       Encoder resolution in mm/sec.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetSpeed(GoTransform transform, k64f value);

/** 
 * Gets the X component of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           The sensor role transformation to retrieve the value from.
 * @return                  The transformation X component.
 */
GoFx(k64f) GoTransform_X(GoTransform transform, GoRole role);

/** 
 * Sets the transformation X component.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        The sensor role transformation entry to update.
 * @param    offset      The transformation X component to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetX(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Y component of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           The sensor role transformation to retrieve the value from.
 * @return                  The transformation Y component.
 */
GoFx(k64f) GoTransform_Y(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Y component.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        The sensor role transformation entry to update.
 * @param    offset      The transformation Y component to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetY(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Z component of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           The sensor role transformation to retrieve the value from.
 * @return                  The transformation Z component.
 */
GoFx(k64f) GoTransform_Z(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Z component.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        The sensor role transformation entry to update.
 * @param    offset      The transformation Z component to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetZ(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the X-angle of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           The sensor role transformation to retrieve the value from.
 * @return                  The transformation X-angle.
 */
GoFx(k64f) GoTransform_XAngle(GoTransform transform, GoRole role);

/** 
 * Sets the transformation X-angle.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        The sensor role transformation entry to update.
 * @param    offset      The transformation X-angle to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetXAngle(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Y-angle of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           The sensor role transformation to retrieve the value from.
 * @return                  The transformation Y-angle component.
 */
GoFx(k64f) GoTransform_YAngle(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Y-angle.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        The sensor role transformation entry to update.
 * @param    offset      The transformation Y-angle to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetYAngle(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Z-angle of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           The sensor role transformation to retrieve the value from.
 * @return                  The transformation Z-angle component.
 */
GoFx(k64f) GoTransform_ZAngle(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Z-angle.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        The sensor role transformation entry to update.
 * @param    offset      The transformation Z-angle to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetZAngle(GoTransform transform, GoRole role, k64f offset);


kEndHeader()
#include <GoSdk/GoTransform.x.h>

#endif
