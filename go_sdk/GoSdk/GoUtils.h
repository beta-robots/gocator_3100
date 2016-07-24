/** 
 * @file    GoUtils.h
 * @brief   Contains various helper functions. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_UTILS_H
#define GO_SDK_UTILS_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Data/kString.h>
#include <math.h>
kBeginHeader()

/**
 * Returns true if input is within min and max. 
 */
#define GoUtils_MinMax_(in, min, max)                                \
    (((in) > (max) || (in) < (min)) ? (kFALSE) : (kTRUE))

/** 
 * Returns the result of a floating point number equivalence, based on a given degree of precision.
 *
 * @public                          @memberof GoUtils
 * @version                         Introduced in firmware 4.0.10.27
 * @param    first                  k64f object.
 * @param    second                 k64f object.
 * @param    decimalPrecision       k8u object.
 * @return                          Returns kTRUE if the numbers are equivalent within the given precision; kFALSE otherwise.            
 */
GoFx(kBool) GoUtils_FuzzyEquivalence(k64f first, k64f second, k8u decimalPrecision);

kEndHeader()
#include <GoSdk/GoUtils.x.h>

#endif
