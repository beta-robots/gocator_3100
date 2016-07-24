/** 
 * @file    kMath.x.h
 * @brief   Utilities. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MATH_X_H
#define K_API_MATH_X_H

kBeginHeader()

typedef struct kMathStatic
{
    k32u placeholder;       //unused
} kMathStatic;

kDeclareStaticClass(k, kMath)

kFx(kStatus) kMath_InitStatic(); 
kFx(kStatus) kMath_ReleaseStatic();

#define kxMath_RadToDeg_(RAD)           ((RAD)*180.0/(kMATH_PI))
#define kxMath_DegToRad_(DEG)           ((DEG)*(kMATH_PI)/180.0)
#define kxMath_Abs_(A)                  (((A) >= 0) ? (A) : -(A))
#define kxMath_Min_(A, B)               (((A) < (B)) ? (A) : (B))
#define kxMath_Max_(A, B)               (((A) > (B)) ? (A) : (B))
#define kxMath_Sign_(A)                 (((A) > 0) ? 1 : ((A) == 0) ? 0 : -1)
#define kxMath_Common_Residue_(A,B)     (((A) >= 0) ? ((A) % (B)) : (((B) - (-(A) % (B))) % (B)))
#define kxMath_Clamp_(V, VMIN, VMAX)    (kMath_Min_(kMath_Max_((V), (VMIN)), (VMAX)))
#define kxMath_Round8s_(A)              ((k8s) ((A) > 0 ? ((A)+0.5) : ((A)-0.5)))
#define kxMath_Round8u_(A)              ((k8u) ((A)+0.5))
#define kxMath_Round16s_(A)             ((k16s)((A) > 0 ? ((A)+0.5) : ((A)-0.5)))
#define kxMath_Round16u_(A)             ((k16u)((A)+0.5))
#define kxMath_Round32s_(A)             ((k32s)((A) > 0 ? ((A)+0.5) : ((A)-0.5)))
#define kxMath_Round32u_(A)             ((k32u)((A)+0.5))
#define kxMath_Round64s_(A)             ((k64s)((A) > 0 ? ((A)+0.5) : ((A)-0.5)))
#define kxMath_Round64u_(A)             ((k64u)((A)+0.5))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kMATH_RAD_TO_DEG             kMath_RadToDeg_
#   define kMATH_DEG_TO_RAD             kMath_DegToRad_
#   define kMATH_SIGN                   kMath_Sign_
#   define kMATH_COMMON_RESIDUE         kMath_Common_Residue_
#   define kMATH_ROUND_8S               kMath_Round8s_
#   define kMATH_ROUND_8U               kMath_Round8u_
#   define kMATH_ROUND_16S              kMath_Round16s_
#   define kMATH_ROUND_16U              kMath_Round16u_
#   define kMATH_ROUND_32S              kMath_Round32s_
#   define kMATH_ROUND_32U              kMath_Round32u_
#   define kMATH_ROUND_64S              kMath_Round64s_
#   define kMATH_ROUND_64U              kMath_Round64u_

#endif

kEndHeader()

#endif
