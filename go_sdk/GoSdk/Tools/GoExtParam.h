///@cond private

/** 
 * @file    GoExtParam.h
 * @brief   Declares the GoExtParam class.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_VALUE_H
#define GO_EXT_VALUE_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <kApi/Data/kXml.h>
kBeginHeader()

typedef struct GoExtParamIntOption
{
    k32s value;
    kText64 description;
} GoExtParamIntOption;

typedef struct GoExtParamFloatOption
{
    k64f value;
    kText64 description;
} GoExtParamFloatOption;

typedef kObject GoExtParam; 

GoFx(const kChar*) GoExtParam_Label(GoExtParam param);
GoFx(const kChar*) GoExtParam_Id(GoExtParam param);
GoFx(GoExtParamType) GoExtParam_Type(GoExtParam param);
GoFx(kBool) GoExtParam_Used(GoExtParam param);
GoFx(GoUnitType) GoExtParam_UnitType(GoExtParam param);

typedef GoExtParam GoExtParamBool; 

GoFx(kBool) GoExtParamBool_Value(GoExtParamBool param);
GoFx(kStatus) GoExtParamBool_SetValue(GoExtParamBool param, kBool newVal);

typedef GoExtParam GoExtParamInt; 

GoFx(k32s) GoExtParamInt_Value(GoExtParamInt param);
GoFx(kStatus) GoExtParamInt_SetValue(GoExtParamInt param, k32s newVal);
GoFx(k32s) GoExtParamInt_ValueMin(GoExtParam param);
GoFx(k32s) GoExtParamInt_ValueMax(GoExtParam param);
GoFx(kSize) GoExtParamInt_OptionCount(GoExtParamInt param);
GoFx(k32s) GoExtParamInt_OptionValueAt(GoExtParamInt param, kSize index);
GoFx(const kChar*) GoExtParamInt_OptionDescriptionAt(GoExtParamInt param, kSize index);

typedef GoExtParam GoExtParamFloat; 

GoFx(k64f) GoExtParamFloat_Value(GoExtParamFloat param);
GoFx(kStatus) GoExtParamFloat_SetValue(GoExtParamFloat param, k64f newVal);
GoFx(k64f) GoExtParamFloat_ValueMin(GoExtParam param);
GoFx(k64f) GoExtParamFloat_ValueMax(GoExtParam param);
GoFx(kSize) GoExtParamFloat_OptionCount(GoExtParamFloat param);
GoFx(k64f) GoExtParamFloat_OptionValueAt(GoExtParamFloat param, kSize index);
GoFx(const kChar*) GoExtParamFloat_OptionDescriptionAt(GoExtParamFloat param, kSize index);

typedef GoExtParam GoExtParamString; 
GoFx(kString) GoExtParamString_Value(GoExtParamString param);

typedef GoExtParam GoExtParamProfileRegion; 
GoFx(GoProfileRegion) GoExtParamProfileRegion_Value(GoExtParamProfileRegion param);

typedef GoExtParam GoExtParamSurfaceRegion2d; 
GoFx(GoSurfaceRegion2d) GoExtParamSurfaceRegion2d_Value(GoExtParamSurfaceRegion2d param);

typedef GoExtParam GoExtParamRegion3d; 
GoFx(GoRegion3d) GoExtParamRegion3d_Value(GoExtParamRegion3d param);

kEndHeader()
#include <GoSdk/Tools/GoExtParam.x.h>

#endif
///@endcond
