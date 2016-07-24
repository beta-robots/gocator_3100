/// @cond (Gocator_2x00 || Gocator_3x00)

/** 
 * @file    GoSurfaceGeneration.h
 * @brief   Declares the GoSurfaceGeneration class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACEGENERATION_H
#define GO_SURFACEGENERATION_H

#include <GoSdk/GoSdkDef.h>

kBeginHeader()

/**
 * @class   GoSurfaceGeneration
 * @extends kObject
 * @ingroup GoSdk-Surface
 * @brief   Represents a surface generation configuration.
 */
typedef kObject GoSurfaceGeneration; 

/** 
 * Sets the surface generation type.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   type            The surface generation type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGeneration_SetGenerationType(GoSurfaceGeneration surface, GoSurfaceGenerationType type);

/** 
 * Gets the current surface generation type.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The surface generation type.
 */
GoFx(GoSurfaceGenerationType) GoSurfaceGeneration_GenerationType(GoSurfaceGeneration surface);

/** 
 * Sets the fixed length surface generation start trigger.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   trigger         The surface generation start trigger value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationFixedLength_SetStartTrigger(GoSurfaceGeneration surface, GoSurfaceGenerationStartTrigger trigger);

/** 
 * Gets the fixed length surface generation start trigger.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation start trigger.
 */
GoFx(GoSurfaceGenerationStartTrigger) GoSurfaceGenerationFixedLength_StartTrigger(GoSurfaceGeneration surface);

/** 
 * Sets the fixed length surface generation surface length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   length          The fixed length surface generation surface length to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationFixedLength_SetLength(GoSurfaceGeneration surface, k64f length);

/** 
 * Gets the fixed length surface generation surface length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation surface length.
 */
GoFx(k64f) GoSurfaceGenerationFixedLength_Length(GoSurfaceGeneration surface);

/** 
 * Gets the fixed length surface generation circumference limit maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation circumference limit maximum value.
 */
GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMax(GoSurfaceGeneration surface);

/** 
 * Gets the fixed length surface generation circumference limit minimum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation circumference limit minimum value.
 */
GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMin(GoSurfaceGeneration surface);

/** 
 * Sets the variable length surface generation maximum length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   length          The variable length surface generation maximum length to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationVariableLength_SetMaxLength(GoSurfaceGeneration surface, k64f length);

/** 
 * Gets the variable length surface generation maximum length.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The variable length surface generation maximum length.
 */
GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLength(GoSurfaceGeneration surface);

/** 
 * Gets the variable length surface generation circumference limit maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The variable length surface generation circumference limit maximum value.
 */
GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMax(GoSurfaceGeneration surface);

/** 
 * Gets the variable length surface generation circumference limit minimum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The variable length surface generation circumference limit minimum value.
 */
GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMin(GoSurfaceGeneration surface);

/** 
 * Sets the rotational surface generation surface circumference.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   value           The rotational surface generation circumference value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationRotational_SetCircumference(GoSurfaceGeneration surface, k64f value);

/** 
 * Gets the rotational surface generation circumference.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The rotational surface generation circumference.
 */
GoFx(k64f) GoSurfaceGenerationRotational_Circumference(GoSurfaceGeneration surface);

/** 
 * Gets the rotational surface generation circumference limit maximum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The rotational surface generation circumference limit maximum value.
 */
GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMax(GoSurfaceGeneration surface);

/** 
 * Gets the rotational surface generation circumference limit minimum value.
 *
 * @public                  @memberof GoPartDetection
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The rotational surface generation circumference limit minimum value.
 */
GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMin(GoSurfaceGeneration surface);

kEndHeader()
#include <GoSdk/GoSurfaceGeneration.x.h>

#endif

/// @endcond