/** 
 * @file    kMath.h
 * @brief   Math-related utilities. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MATH_H
#define K_API_MATH_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
* @class   kMath
* @extends kObject
* @ingroup kApi-Data
* @brief   Collection of mathematical utility functions.
*/
typedef kObject kMath;

/** @relates kMath @{ */

#define kMATH_PI                        (3.1415926535897932384626433832795)     ///< Mathematical constant pi. 
#define kMATH_E                         (2.7182818284590452353602874713527)     ///< Mathematical constant e. 
#define kMATH_SQRT2                     (1.4142135623730950488016887242097)     ///< Square root of 2.
#define kMATH_SQRT3                     (1.7320508075688772935274463415059)     ///< Square root of 3. 

#define kMath_RadToDeg_(RAD)            kxMath_RadToDeg_(RAD)                   ///< Converts radians to degrees. 
#define kMath_DegToRad_(DEG)            kxMath_DegToRad_(DEG)                   ///< Converts degrees to radians. 
#define kMath_Abs_(A)                   kxMath_Abs_(A)                          ///< Returns the absolute value of a number.
#define kMath_Min_(A, B)                kxMath_Min_((A), (B))                   ///< Returns the minimum of two numbers. 
#define kMath_Max_(A, B)                kxMath_Max_((A), (B))                   ///< Returns the maximum of two numbers. 
#define kMath_Sign_(A)                  kxMath_Sign_(A)                         ///< Returns the sign of a number (-1, 0, +1).
#define kMath_Common_Residue_(A,B)      kxMath_Common_Residue_((A), (B))        ///< Returns the common residue of A and B. 
#define kMath_Clamp_(V, VMIN, VMAX)     kxMath_Clamp_((V), (VMIN), (VMAX))      ///< Returns a value limited to the given range.
#define kMath_Round8s_(A)               kxMath_Round8s_(A)                      ///< Rounds a floating-point value and casts to an 8-bit signed integer. 
#define kMath_Round8u_(A)               kxMath_Round8u_(A)                      ///< Rounds a floating-point value and casts to an 8-bit unsigned integer.
#define kMath_Round16s_(A)              kxMath_Round16s_(A)                     ///< Rounds a floating-point value and casts to an 16-bit signed integer.
#define kMath_Round16u_(A)              kxMath_Round16u_(A)                     ///< Rounds a floating-point value and casts to an 16-bit unsigned integer.
#define kMath_Round32s_(A)              kxMath_Round32s_(A)                     ///< Rounds a floating-point value and casts to an 32-bit signed integer.
#define kMath_Round32u_(A)              kxMath_Round32u_(A)                     ///< Rounds a floating-point value and casts to an 32-bit unsigned integer.
#define kMath_Round64s_(A)              kxMath_Round64s_(A)                     ///< Rounds a floating-point value and casts to an 64-bit signed integer.
#define kMath_Round64u_(A)              kxMath_Round64u_(A)                     ///< Rounds a floating-point value and casts to an 64-bit unsigned integer.

/** @} */

/** 
 * Compares each value in a numerical array with a specified value and returns the index of the first match. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   comparison  Comparator that will be used to determine a match.  
 * @param   value       Value for comparison.  
 * @param   index       Returns the index of the first match (or count, if no match is found).   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindFirst32s(const k32s* v, kSize count, kComparison comparison, k32s value, kSize* index);

/** 
 * Compares each value in a numerical array with a specified value and returns the index of the first match. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   comparison  Comparator used to determine a match.  
 * @param   value       Value for comparison.  
 * @param   index       Returns the index of the first match (or count, if no match is found).   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindFirst64f(const k64f* v, kSize count, kComparison comparison, k64f value, kSize* index);

/** 
 * Compares each value in a numerical array with a specified value and returns the index of the last match. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   comparison  Comparator used to determine a match.  
 * @param   value       Value for comparison.  
 * @param   index       Returns the index of the last match (or count, if no match is found).   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindLast32s(const k32s* v, kSize count, kComparison comparison, k32s value, kSize* index);

/** 
 * Compares each value in a numerical array with a specified value and returns the index of the last match. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   comparison  Comparator that will be used to determine a match.  
 * @param   value       Value for comparison.  
 * @param   index       Returns the index of the last match (or count, if no match is found).   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindLast64f(const k64f* v, kSize count, kComparison comparison, k64f value, kSize* index);

/** 
 * Finds the index of the minimum value within a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   index       Returns the index of the minimum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindMin32s(const k32s* v, kSize count, kSize* index);

/** 
 * Finds the index of the minimum value within a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   index       Returns the index of the minimum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindMin64f(const k64f* v, kSize count, kSize* index);

/** 
 * Finds the index of the maximum value within a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   index       Returns the index of the maximum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindMax32s(const k32s* v, kSize count, kSize* index);

/** 
 * Finds the index of the maximum value within a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   index       Returns the index of the maximum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_FindMax64f(const k64f* v, kSize count, kSize* index);

/** 
 * Calculates the sum of a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   sum         Returns the sum.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Sum32s(const k32s* v, kSize count, k64s* sum);

/** 
 * Calculates the sum of a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   sum         Returns the sum.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Sum64f(const k64f* v, kSize count, k64f* sum);

/** 
 * Calculates the average value for a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   average     Returns the average.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Average32s(const k32s* v, kSize count, k64f* average);

/** 
 * Calculates the average value for a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   average     Returns the average.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Average64f(const k64f* v, kSize count, k64f* average);

/** 
 * Calculates the standard deviation for a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   stdev       Returns the standard deviation.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Stdev32s(const k32s* v, kSize count, k64f* stdev);

/** 
 * Calculates the standard deviation for a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   stdev       Returns the standard deviation.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Stdev64f(const k64f* v, kSize count, k64f* stdev);

/** 
 * Reports the minimum value in a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   minValue    Returns the minimum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Min32s(const k32s* v, kSize count, k32s* minValue);

/** 
 * Reports the minimum value in a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   minValue    Returns the minimum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Min64f(const k64f* v, kSize count, k64f* minValue);

/** 
 * Reports the maximum value in a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   maxValue    Returns the maximum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Max32s(const k32s* v, kSize count, k32s* maxValue);

/** 
 * Reports the maximum value in a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   maxValue    Returns the maximum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Max64f(const k64f* v, kSize count, k64f* maxValue);

/** 
 * Calculates the center of gravity for a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   centroid    Returns the centroid.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Centroid32s(const k32s* v, kSize count, k64f* centroid);

/** 
 * Calculates the center of gravity for a numerical array. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   centroid    Returns the centroid.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Centroid64f(const k64f* v, kSize count, k64f* centroid);

/** 
 * Sets all values in a numerical array to the given value.
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   value       Value to set.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Set32s(k32s* v, kSize count, k32s value);

/** 
 * Sets all values in a numerical array to the given value.
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   value       Value to set.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Set64f(k64f* v, kSize count, k64f value);

/** 
 * Sets values in a numerical array to increment from the specified starting value.
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   startValue  First value.   
 * @param   increment   Increment amount.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Step32s(k32s* v, kSize count, k32s startValue, k32s increment);

/** 
 * Sets values in a numerical array to increment from the specified starting value.
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   startValue  First value.   
 * @param   increment   Increment amount.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Step64f(k64f* v, kSize count, k64f startValue, k64f increment);

/** 
 * Sets values in a numerical array to step between the specified start and end values. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   startValue  First value.   
 * @param   endValue    Last value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Span32s(k32s* v, kSize count, k32s startValue, k32s endValue);

/** 
 * Sets values in a numerical array to step between the specified start and end values. 
 *
 * @public              @memberof kMath
 * @param   v           Array of values.  
 * @param   count       Count of values.  
 * @param   startValue  First value.   
 * @param   endValue    Last value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Span64f(k64f* v, kSize count, k64f startValue, k64f endValue);

/** 
 * Calculates the absolute value of each element in an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Abs32s(const k32s* vIn, k32s* vOut, kSize count);

/** 
 * Calculates the absolute value of each element in an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Abs64f(const k64f* vIn, k64f* vOut, kSize count);

/** 
 * Adds a constant to each element in an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_AddC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value);

/** 
 * Adds a constant to each element in an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_AddC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value);

/** 
 * Subtracts a constant from each element in an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_SubC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value);

/** 
 * Subtracts a constant from each element in an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_SubC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value);

/** 
 * Multiplies each element in an input array by a constant and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_MulC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value);

/** 
 * Multiplies each element in an input array by a constant and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_MulC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value);

/** 
 * Divides each element in an input array by a constant and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_DivC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value);

/** 
 * Divides each element in an input array by a constant and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   value       Constant value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_DivC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value);

/** 
 * Limits each element in an input array using a minimum and maximum value.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   minValue    Minimum value.   
 * @param   maxValue    Maximum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_ClampC32s(const k32s* vIn, k32s* vOut, kSize count, k32s minValue, k32s maxValue);

/** 
 * Limits each element in an input array using a minimum and maximum value.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   minValue    Minimum value.   
 * @param   maxValue    Maximum value.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_ClampC64f(const k64f* vIn, k64f* vOut, kSize count, k64f minValue, k64f maxValue);

/** 
 * Compares each element in an input array with a specified value, and replaces all matching values with another given value.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   comparison  Comparator used to determine a match.  
 * @param   value       Value for comparison.  
 * @param   replacement Value for replacement.  
 * @return              Operation status. 
 */
kFx(kStatus) kMath_ReplaceC32s(const k32s* vIn, k32s* vOut, kSize count, kComparison comparison, k32s value, k32s replacement);

/** 
 * Compares each element in an input array with a specified value, and replaces all matching values with another given value.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   comparison  Comparator used to determine a match.  
 * @param   value       Value for comparison.  
 * @param   replacement Value for replacement.  
 * @return              Operation status. 
 */
kFx(kStatus) kMath_ReplaceC64f(const k64f* vIn, k64f* vOut, kSize count, kComparison comparison, k64f value, k64f replacement);

/** 
 * Compares each element in an input array with a specified value and stores the results in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   comparison  Comparator used to determine a match.  
 * @param   value       Value for comparison.  
 * @return              Operation status. 
 */
kFx(kStatus) kMath_CompareC32s(const k32s* vIn, kBool* vOut, kSize count, kComparison comparison, k32s value);

/** 
 * Compares each element in an input array with a specified value and stores the results in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @param   comparison  Comparator used to determine a match.  
 * @param   value       Value for comparison.  
 * @return              Operation status. 
 */
kFx(kStatus) kMath_CompareC64f(const k64f* vIn, kBool* vOut, kSize count, kComparison comparison, k64f value);

/** 
 * Adds the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Add32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count);

/** 
 * Adds the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Add64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count);

/** 
 * Subtracts the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Sub32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count);

/** 
 * Subtracts the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Sub64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count);

/** 
 * Multiplies the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Mul32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count);

/** 
 * Multiplies the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Mul64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count);

/** 
 * Divides the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Div32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count);

/** 
 * Divides the values in two input arrays and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn1        First input array.  
 * @param   vIn2        Second input array.  
 * @param   vOut        Output array.  
 * @param   count       Count of values.   
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Div64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count);

/** 
 * Calculates the moving average over an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array. 
 * @param   vOut        Second input array.  
 * @param   count       Count of values. 
 * @param   window      Moving average window size  
 * @return              Operation status. 
 */
kFx(kStatus) kMath_MovingAvg32s(const k32s* vIn, k32s* vOut, kSize count, kSize window);

/** 
 * Calculates the moving average over an input array and stores the result in an output array.
 *
 * @public              @memberof kMath
 * @param   vIn         Input array. 
 * @param   vOut        Second input array.  
 * @param   count       Count of values. 
 * @param   window      Moving average window size  
 * @return              Operation status. 
 */
kFx(kStatus) kMath_MovingAvg64f(const k64f* vIn, k64f* vOut, kSize count, kSize window);

/** 
 * Returns the greatest common divisor of two integers.
 *
 * @public              @memberof kMath
 * @param   a           First integer.  
 * @param   b           Second integer.  
 * @param   result      Receives result.
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Gcd32s(k32s a, k32s b, k32s* result);

/** 
 * Returns the least common multiple of two integers.
 *
 * @public              @memberof kMath
 * @param   a           First Integer.  
 * @param   b           Second integer.  
 * @param   result      Receives result.
 * @return              Operation status. 
 */
kFx(kStatus) kMath_Lcm32s(k32s a, k32s b, k32s* result);

/** 
 * Returns the sign.
 *
 * @public              @memberof kMath
 * @param   a           Integer.  
 * @return              Returns the sign.
 */
kFx(k32s) kMath_Sign(k32s a);

/** 
 * Calculates the base-2 logarithm of the input, rounded up to the nearest integer. 
 *
 * @public              @memberof kMath
 * @param   a           Input value.
 * @return              Base-2 logarithm of input.
 */
kFx(k32u) kMath_Log2Ceil32u(k32u a); 

kEndHeader()

#include <kApi/Data/kMath.x.h>

#endif
