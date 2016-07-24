/// @cond (Gocator_1x00 || Gocator_2x00)

/** 
 * @file    GoMaterial.h
 * @brief   Declares the GoMaterial class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_MATERIAL_H
#define GO_MATERIAL_H

#include <GoSdk/GoSdkDef.h>

kBeginHeader()

/**
 * @class   GoMaterial
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents configurable material acquisition settings.
 */
typedef kObject GoMaterial; 

/** 
 * Sets the material acquisition type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   type       The material type to set.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetType(GoMaterial material, GoMaterialType type);

/** 
 * Returns the user defined material acquisition type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The material type.
 */
GoFx(GoMaterialType) GoMaterial_Type(GoMaterial material);

/** 
 * Returns a boolean relating to whether the user defined material acquisition type value will be used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user defined material type will be used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsTypeUsed(GoMaterial material);

/** 
 * Returns the material acquisition type to be used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The system value material type.
 */
GoFx(GoMaterialType) GoMaterial_TypeSystemValue(GoMaterial material);

/** 
 * Sets the spot threshold.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Spot threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetSpotThreshold(GoMaterial material, k32u value);

/** 
 * Returns the user defined spot threshold.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThreshold(GoMaterial material);

/** 
 * Returns the minimum spot threshold limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The minimum spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThresholdLimitMin(GoMaterial material);

/** 
 * Returns the maximum spot threshold limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThresholdLimitMax(GoMaterial material);

/** 
 * Returns a boolean value representing whether the user specified spot threshold value is used.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSpotThresholdUsed(GoMaterial material);

/** 
 * Returns the system spot threshold value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The system spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThresholdSystemValue(GoMaterial material);

/** 
 * Sets the maximum spot width.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Maximum spot width.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetSpotWidthMax(GoMaterial material, k32u value);

/** 
 * Returns the user defined maximum spot width.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width.
 */
GoFx(k32u) GoMaterial_SpotWidthMax(GoMaterial material);

/** 
 * Returns the maximum spot width minimum limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width minimum limit.
 */
GoFx(k32u) GoMaterial_SpotWidthMaxLimitMin(GoMaterial material);

/** 
 * Returns the maximum spot width maximum limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width maximum limit.
 */
GoFx(k32u) GoMaterial_SpotWidthMaxLimitMax(GoMaterial material);

/** 
 * Returns a boolean relating to whether the user defined spot width max value will be used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user value will be used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSpotWidthMaxUsed(GoMaterial material);

/** 
 * Returns the maximum spot width system value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width system value.
 */
GoFx(k32u) GoMaterial_SpotWidthMaxSystemValue(GoMaterial material);

/** 
 * Sets the spot selection type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   type       Spot selection type.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetSpotSelectionType(GoMaterial material, GoSpotSelectionType type);

/** 
 * Returns the user defined spot selection type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width.
 */
GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionType(GoMaterial material);

/** 
 * Returns a boolean relating to whether the user defined spot selection type will be used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user value will be used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSpotSelectionTypeUsed(GoMaterial material);

/** 
 * Returns the system spot selection type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             System spot selection type.
 */
GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionTypeSystemValue(GoMaterial material);

/** 
 * Sets the analog camera gain.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Analog camera gain.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetCameraGainAnalog(GoMaterial material, k64f value);

/** 
 * Returns the user defined analog camera gain value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Analog camera gain value.
 */
GoFx(k64f) GoMaterial_CameraGainAnalog(GoMaterial material);

/** 
 * Returns the analog camera gain minimum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Analog camera gain minimum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainAnalogLimitMin(GoMaterial material);

/** 
 * Returns the analog camera gain maximum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Analog camera gain maximum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainAnalogLimitMax(GoMaterial material);

/** 
 * Returns a boolean value representing whether the user defined analog camera gain is used.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user defined analog camera gain is used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsCameraGainAnalogUsed(GoMaterial material);

/** 
 * Returns the analog camera gain system value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The analog camera gain system value.
 */
GoFx(k64f) GoMaterial_CameraGainAnalogSystemValue(GoMaterial material);

/** 
 * Sets the digital camera gain
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Digital camera gain.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetCameraGainDigital(GoMaterial material, k64f value);

/** 
 * Returns the user defined digital camera gain value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The digital camera gain system value.
 */
GoFx(k64f) GoMaterial_CameraGainDigital(GoMaterial material);

/** 
 * Returns the digital camera gain minimum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Digital camera gain minimum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainDigitalLimitMin(GoMaterial material);

/** 
 * Returns the digital camera gain maximum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Digital camera gain maximum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainDigitalLimitMax(GoMaterial material);

/** 
 * Returns a boolean value representing whether the user's digital camera gain value is used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsCameraGainDigitalUsed(GoMaterial material);

/** 
 * Returns the system's digital camera gain value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Digital camera gain system value.
 */
GoFx(k64f) GoMaterial_CameraGainDigitalSystemValue(GoMaterial material);

/** 
 * Sets the dynamic sensitivity.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Dynamic sensitivity.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetDynamicSensitivity(GoMaterial material, k64f value);

/** 
 * Returns the user defined dynamic sensitivity value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             User defined dynamic sensitivity value.
 */
GoFx(k64f) GoMaterial_DynamicSensitivity(GoMaterial material);

/** 
 * Returns the dynamic sensitivity minimum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic sensitivity minimum value limit.
 */
GoFx(k64f) GoMaterial_DynamicSensitivityLimitMin(GoMaterial material);

/** 
 * Returns the dynamic sensitivity maximum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic sensitivity maximum value limit.
 */
GoFx(k64f) GoMaterial_DynamicSensitivityLimitMax(GoMaterial material);

/** 
 * Returns a boolean representing whether the user defined dynamic sensitivity value is used.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsDynamicSensitivityUsed(GoMaterial material);

/** 
 * Returns the dynamic sensitivity system value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic sensitivity system value.
 */
GoFx(k64f) GoMaterial_DynamicSensitivitySystemValue(GoMaterial material);

/** 
 * Sets the dynamic threshold.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Dynamic threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetDynamicThreshold(GoMaterial material, k32u value);

/** 
 * Returns the dynamic threshold minimum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold minimum value limit.
 */
GoFx(k32u) GoMaterial_DynamicThresholdLimitMin(GoMaterial material);

/** 
 * Returns the dynamic threshold maximum value limit.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold maximum value limit.
 */
GoFx(k32u) GoMaterial_DynamicThresholdLimitMax(GoMaterial material);

/** 
 * Returns the user defined dynamic threshold value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The user defined dynamic threshold value.
 */
GoFx(k32u) GoMaterial_DynamicThreshold(GoMaterial material);

/** 
 * Returns a boolean representing whether or not the user defined dynamic threshold is used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold minimum value limit.
 */
GoFx(kBool) GoMaterial_IsDynamicThresholdUsed(GoMaterial material);

/** 
 * Returns the dynamic threshold system value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold system value.
 */
GoFx(k32u) GoMaterial_DynamicThresholdSystemValue(GoMaterial material);

/** 
 * Sets the gamma type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Gamma type.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetGammaType(GoMaterial material, GoGammaType value);

/** 
 * Returns the user defined gamma type.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             User defined gamma type.
 */
GoFx(GoGammaType) GoMaterial_GammaType(GoMaterial material);

/** 
 * Returns a boolean representing whether the user defined gamma type is used by the system.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsGammaTypeUsed(GoMaterial material);

/** 
 * Returns the system's gamma type value.
 *
 * @public             @memberof GoMaterial
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The system gamma type value.
 */
GoFx(GoGammaType) GoMaterial_GammaTypeSystemValue(GoMaterial material);

kEndHeader()
#include <GoSdk/GoMaterial.x.h>

#endif

/// @endcond