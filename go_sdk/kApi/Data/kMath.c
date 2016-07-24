/** 
 * @file    kMath.c
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kMath.h>
#include <math.h>

kBeginStaticClass(k, kMath)
kEndStaticClass()

kFx(kStatus) kMath_InitStatic()
{
    return kOK;
}

kFx(kStatus) kMath_ReleaseStatic()
{
    return kOK;
}

#define kMath_IsEq_(V1, V2)              ((V1) == (V2))
#define kMath_IsNeq_(V1, V2)             ((V1) != (V2))
#define kMath_IsLt_(V1, V2)              ((V1) < (V2))
#define kMath_IsLte_(V1, V2)             ((V1) <= (V2))
#define kMath_IsGt_(V1, V2)              ((V1) > (V2))
#define kMath_IsGte_(V1, V2)             ((V1) >= (V2))

#define kMath_DefinePredicateFx_(F, T)              \
    kMath_Define##F##_(Eq, T)                       \
    kMath_Define##F##_(Neq, T)                      \
    kMath_Define##F##_(Lt, T)                       \
    kMath_Define##F##_(Lte, T)                      \
    kMath_Define##F##_(Gt, T)                       \
    kMath_Define##F##_(Gte, T)

#define kMath_Add_(V1, V2)               ((V1) + (V2))
#define kMath_Subtract_(V1, V2)          ((V1) - (V2))
#define kMath_Multiply_(V1, V2)          ((V1) * (V2))
#define kMath_Divide_(V1, V2)            ((V1) / (V2))

#define kMath_DefineOpFx_(F, T)                 \
    kMath_Define##F##_(Add, T)                  \
    kMath_Define##F##_(Subtract, T)             \
    kMath_Define##F##_(Multiply, T)             \
    kMath_Define##F##_(Divide, T)

#define kMath_DefineFindFirst_(P, T)                                                        \
kFx(kStatus) kMath_FindFirst##P##T(const k##T* v, kSize count, k##T value, kSize* index)    \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* end = it + count;                                                           \
                                                                                            \
    if ((!v && (count > 0)) || !index)                                                      \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    *index = count;                                                                         \
                                                                                            \
    if (count > 0)                                                                          \
    {                                                                                       \
        while (it != end)                                                                   \
        {                                                                                   \
            if (kMath_Is##P##_(*it, value))                                                 \
            {                                                                               \
                *index = (kSize) (it - v);                                                  \
                break;                                                                      \
            }                                                                               \
            it++;                                                                           \
        }                                                                                   \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineFindLast_(P, T)                                                         \
kFx(kStatus) kMath_FindLast##P##T(const k##T* v, kSize count, k##T value, kSize* index)     \
{                                                                                           \
    const k##T* it = v + count - 1;                                                         \
    const k##T* end = it - count;                                                           \
                                                                                            \
    if ((!v && (count > 0)) || !index)                                                      \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    *index = count;                                                                         \
                                                                                            \
    if (count > 0)                                                                          \
    {                                                                                       \
        while (it != end)                                                                   \
        {                                                                                   \
            if (kMath_Is##P##_(*it, value))                                                 \
            {                                                                               \
                *index = (kSize) (it - v);                                                  \
                break;                                                                      \
            }                                                                               \
            it--;                                                                           \
        }                                                                                   \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineFindMin_(T)                                                             \
kFx(kStatus) kMath_FindMin##T(const k##T* v, kSize count, kSize* index)                     \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* end = it + count;                                                           \
    const k##T* minIt = it;                                                                 \
                                                                                            \
    if (!it || (count == 0) || !index)                                                      \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != end)                                                                       \
    {                                                                                       \
        if (*it < *minIt)                                                                   \
        {                                                                                   \
            minIt = it;                                                                     \
        }                                                                                   \
        it++;                                                                               \
    }                                                                                       \
                                                                                            \
    *index = (kSize) (minIt - v);                                                           \
    return kOK;                                                                             \
}

#define kMath_DefineFindMax_(T)                                                             \
kFx(kStatus) kMath_FindMax##T(const k##T* v, kSize count, kSize* index)                     \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* end = it + count;                                                           \
    const k##T* maxIt = it;                                                                 \
                                                                                            \
    if (!it || (count == 0) || !index)                                                      \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != end)                                                                       \
    {                                                                                       \
        if (*it > *maxIt)                                                                   \
        {                                                                                   \
            maxIt = it;                                                                     \
        }                                                                                   \
        it++;                                                                               \
    }                                                                                       \
                                                                                            \
    *index = (kSize) (maxIt - v);                                                           \
    return kOK;                                                                             \
}

#define kMath_DefineSum_(T, V)                                                              \
kFx(kStatus) kMath_Sum##T##V(const k##T* v, kSize count, k##V* sum)                         \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* itEnd = it + count;                                                         \
    k##V accum = 0;                                                                         \
                                                                                            \
    if ((!v && (count > 0))  || !sum)                                                       \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != itEnd)                                                                     \
    {                                                                                       \
        accum += (k##V) (*it++);                                                            \
    }                                                                                       \
                                                                                            \
    *sum = accum;                                                                           \
    return kOK;                                                                             \
}

#define kMath_DefineAverage_(T)                                                             \
kFx(kStatus) kMath_Average##T(const k##T* v, kSize count, k64f* average)                    \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* itEnd = it + count;                                                         \
    k64f sum = 0;                                                                           \
                                                                                            \
    if (!v || (count < 1) || !average)                                                      \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != itEnd)                                                                     \
    {                                                                                       \
        sum += (k64f) (*it++);                                                              \
    }                                                                                       \
                                                                                            \
    *average = sum / count;                                                                 \
    return kOK;                                                                             \
}

#define kMath_DefineStdev_(T)                                                               \
kFx(kStatus) kMath_Stdev##T(const k##T* v, kSize count, k64f* stdev)                        \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* itEnd = it + count;                                                         \
    k64f sum = 0;                                                                           \
    k64f sumSq = 0;                                                                         \
    k64f num, den, quotient;                                                                \
                                                                                            \
    if (!v || (count < 2) || !stdev)                                                        \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != itEnd)                                                                     \
    {                                                                                       \
        sumSq += ((k64f) *it) * ((k64f) *it);                                               \
        sum   += *it++;                                                                     \
    }                                                                                       \
                                                                                            \
    num = count*sumSq - sum*sum;                                                            \
    den = count*(count - 1.0);                                                              \
    quotient = num/den;                                                                     \
                                                                                            \
    if (quotient <= 0)   *stdev = 0;                                                        \
    else                 *stdev = sqrt(quotient);                                           \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineMin_(T)                                                                 \
kFx(kStatus) kMath_Min##T(const k##T* v, kSize count, k##T* minValue)                       \
{                                                                                           \
    kStatus status;                                                                         \
    kSize index;                                                                            \
                                                                                            \
    if (!minValue)                                                                          \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    status = kMath_FindMin##T(v, count, &index);                                            \
    if (!kSuccess(status))                                                                  \
    {                                                                                       \
        return status;                                                                      \
    }                                                                                       \
                                                                                            \
    *minValue = v[index];                                                                   \
    return kOK;                                                                             \
}

#define kMath_DefineMax_(T)                                                                 \
kFx(kStatus) kMath_Max##T(const k##T* v, kSize count, k##T* maxValue)                       \
{                                                                                           \
    kStatus status;                                                                         \
    kSize index;                                                                            \
                                                                                            \
    if (!maxValue)                                                                          \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    status = kMath_FindMax##T(v, count, &index);                                            \
    if (!kSuccess(status))                                                                  \
    {                                                                                       \
        return status;                                                                      \
    }                                                                                       \
                                                                                            \
    *maxValue = v[index];                                                                   \
    return kOK;                                                                             \
}

#define kMath_DefineCentroid_(T)                                                            \
kFx(kStatus) kMath_Centroid##T(const k##T* v, kSize count, k64f* centroid)                  \
{                                                                                           \
    const k##T* it = v;                                                                     \
    const k##T* itEnd = it + count;                                                         \
    k64f sum = 0;                                                                           \
    k64f wSum = 0;                                                                          \
                                                                                            \
    if (!v || (count < 1) || !centroid)                                                     \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != itEnd)                                                                     \
    {                                                                                       \
        sum += *it++;                                                                       \
        wSum += sum;                                                                        \
    }                                                                                       \
                                                                                            \
    *centroid = (sum != 0) ? (count - wSum/sum) : (count/2.0);                              \
    return kOK;                                                                             \
}

#define kMath_DefineSet_(T)                                                                 \
kFx(kStatus) kMath_Set##T(k##T* v, kSize count, k##T value)                                 \
{                                                                                           \
    k##T* it = v;                                                                           \
    k##T* itEnd = it + count;                                                               \
                                                                                            \
    if (!v && (count > 0))                                                                  \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != itEnd)                                                                     \
    {                                                                                       \
        *it++ = value;                                                                      \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineStep_(T)                                                                \
kFx(kStatus) kMath_Step##T(k##T* v, kSize count, k##T startValue, k##T increment)           \
{                                                                                           \
    k##T* it = v;                                                                           \
    k##T* itEnd = it + count;                                                               \
    k##T value = startValue;                                                                \
                                                                                            \
    if (kIsNull(v) && (count > 0))                                                          \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (it != itEnd)                                                                     \
    {                                                                                       \
        *it++ = value;                                                                      \
        value += increment;                                                                 \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineSpan_(T)                                                                \
kFx(kStatus) kMath_Span##T(k##T* v, kSize count, k##T startValue, k##T endValue)            \
{                                                                                           \
    kSize i = 0;                                                                            \
                                                                                            \
    if (kIsNull(v) && (count > 0))                                                          \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
    else if (count == 0)                                                                    \
    {                                                                                       \
        return kOK;                                                                         \
    }                                                                                       \
    else if (count == 1)                                                                    \
    {                                                                                       \
        v[0] = endValue;                                                                    \
    }                                                                                       \
    else                                                                                    \
    {                                                                                       \
        for (i = 0; i < count; ++i)                                                         \
        {                                                                                   \
            v[i] = startValue + (k##T)i*(endValue - startValue)/((k##T)count-1);            \
        }                                                                                   \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineAbs_(T)                                                                 \
kFx(kStatus) kMath_Abs##T(const k##T* vIn, k##T* vOut, kSize count)                         \
{                                                                                           \
    const k##T* itIn = vIn;                                                                 \
    const k##T* itInEnd = itIn + count;                                                     \
    k##T* itOut = vOut;                                                                     \
                                                                                            \
    if ((!vIn || !vOut) && (count > 0))                                                     \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (itIn != itInEnd)                                                                 \
    {                                                                                       \
        *itOut++ = (*itIn >= 0) ? (*itIn) : -(*itIn);                                       \
        itIn++;                                                                             \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineApplyC_(P, T)                                                           \
kFx(kStatus) kMath_ApplyC##P##T(const k##T* vIn, k##T* vOut, kSize count, k##T value)       \
{                                                                                           \
    const k##T* itIn = vIn;                                                                 \
    const k##T* itInEnd = itIn + count;                                                     \
    k##T* itOut = vOut;                                                                     \
                                                                                            \
    if ((!vIn || !vOut) && (count > 0))                                                     \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (itIn != itInEnd)                                                                 \
    {                                                                                       \
        *itOut++ = kMath_##P##_(*itIn++, value);                                            \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineApply_(P, T)                                                            \
kFx(kStatus) kMath_Apply##P##T(const k##T* vIn1, const k##T* vIn2, k##T* vOut, kSize count) \
{                                                                                           \
    const k##T* itIn1 = vIn1;                                                               \
    const k##T* itIn1End = itIn1 + count;                                                   \
    const k##T* itIn2 = vIn2;                                                               \
    k##T* itOut = vOut;                                                                     \
                                                                                            \
    if ((!vIn1 || !vIn2 || !vOut) && (count > 0))                                           \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (itIn1 != itIn1End)                                                               \
    {                                                                                       \
        *itOut++ = kMath_##P##_(*itIn1++, *itIn2++);                                        \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineClampC_(T)                                                                          \
kFx(kStatus) kMath_ClampC##T(const k##T* vIn, k##T* vOut, kSize count, k##T minValue, k##T maxValue)    \
{                                                                                                       \
    const k##T* itIn = vIn;                                                                             \
    const k##T* itInEnd = itIn + count;                                                                 \
    k##T* itOut = vOut;                                                                                 \
                                                                                                        \
    if ((!vIn || !vOut) && (count > 0))                                                                 \
    {                                                                                                   \
        return kERROR_PARAMETER;                                                                        \
    }                                                                                                   \
                                                                                                        \
    while (itIn != itInEnd)                                                                             \
    {                                                                                                   \
        if      (*itIn < minValue)  *itOut = minValue;                                                  \
        else if (*itIn > maxValue)  *itOut = maxValue;                                                  \
        else                        *itOut = *itIn;                                                     \
        itIn++;                                                                                         \
        itOut++;                                                                                        \
    }                                                                                                   \
                                                                                                        \
    return kOK;                                                                                         \
}

#define kMath_DefineReplaceC_(P, T)                                                                         \
kFx(kStatus) kMath_ReplaceC##P##T(const k##T* vIn, k##T* vOut, kSize count, k##T value, k##T replacement)   \
{                                                                                                           \
    const k##T* itIn = vIn;                                                                                 \
    const k##T* itInEnd = itIn + count;                                                                     \
    k##T* itOut = vOut;                                                                                     \
                                                                                                            \
    if ((!vIn || !vOut) && (count > 0))                                                                     \
    {                                                                                                       \
        return kERROR_PARAMETER;                                                                            \
    }                                                                                                       \
                                                                                                            \
    while (itIn != itInEnd)                                                                                 \
    {                                                                                                       \
        if (kMath_Is##P##_(*itIn, value))   *itOut = replacement;                                           \
        else                                *itOut = *itIn;                                                 \
        itIn++;                                                                                             \
        itOut++;                                                                                            \
    }                                                                                                       \
                                                                                                            \
    return kOK;                                                                                             \
}

#define kMath_DefineCompareC_(P, T)                                                         \
kFx(kStatus) kMath_CompareC##P##T(const k##T* vIn, kBool* vOut, kSize count, k##T value)    \
{                                                                                           \
    const k##T* itIn = vIn;                                                                 \
    const k##T* itInEnd = itIn + count;                                                     \
    kBool* itOut = vOut;                                                                    \
                                                                                            \
    if ((!vIn || !vOut) && (count > 0))                                                     \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    while (itIn != itInEnd)                                                                 \
    {                                                                                       \
        *itOut++ = kMath_Is##P##_(*itIn++, value);                                          \
    }                                                                                       \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineMovingAvg_(T)                                                           \
kFx(kStatus) kMath_MovingAvg##T(const k##T* vIn, k##T* vOut, kSize count, kSize window)     \
{                                                                                           \
    const k##T* itIn = vIn;                                                                 \
    const k##T* itInEnd = itIn + count;                                                     \
    k##T* itOut = vOut;                                                                     \
    const k##T *windowBegin, *windowEnd, *windowReader, *windowBeginEnd;                    \
    k##T sum, average;                                                                      \
                                                                                            \
    if (!vIn || !vOut || (window < 1) || (count < window))                                  \
    {                                                                                       \
        return kERROR_PARAMETER;                                                            \
    }                                                                                       \
                                                                                            \
    sum = 0;                                                                                \
    windowBegin = itIn;                                                                     \
    windowEnd = itIn + window;                                                              \
    windowReader = windowBegin;                                                             \
    while (windowReader != windowEnd)                                                       \
    {                                                                                       \
        sum += *windowReader++;                                                             \
    }                                                                                       \
                                                                                            \
    windowBeginEnd = itInEnd - window;                                                      \
    while (windowBegin != windowBeginEnd)                                                   \
    {                                                                                       \
        average = sum/(k##T)window;                                                         \
        sum -= *windowBegin++;                                                              \
        sum += *windowEnd++;                                                                \
        *itOut++ = average;                                                                 \
    }                                                                                       \
                                                                                            \
    *itOut = sum/(k##T)window;                                                              \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineGcd_(T)                                                                 \
kFx(kStatus) kMath_Gcd##T(k##T a, k##T b, k##T* out)                                        \
{                                                                                           \
    k##T mod;                                                                               \
                                                                                            \
    /* Iterative version of */                                                              \
    /* f(a, b) {return b == 0 ? a : f(b, a % b);} */                                        \
    while (kTRUE)                                                                           \
    {                                                                                       \
        if (b == 0)                                                                         \
        {                                                                                   \
            break;                                                                          \
        }                                                                                   \
                                                                                            \
        mod = a % b;                                                                        \
        a = b;                                                                              \
        b = mod;                                                                            \
    }                                                                                       \
                                                                                            \
    *out = kMath_Abs_(a);                                                                   \
                                                                                            \
    return kOK;                                                                             \
}

#define kMath_DefineLcm_(T)                                                                 \
kFx(kStatus) kMath_Lcm##T(k##T a, k##T b, k##T* out)                                        \
{                                                                                           \
    k##T gcd;                                                                               \
                                                                                            \
    /* a = b = 0 gives 0 for GCD; handle this as a special case. */                         \
    if (a == 0 && b == 0)                                                                   \
    {                                                                                       \
        *out = 0;                                                                           \
                                                                                            \
        return kOK;                                                                         \
    }                                                                                       \
                                                                                            \
    kCheck(kMath_Gcd##T(a, b, &gcd));                                                       \
                                                                                            \
    *out = kMath_Abs_((a * b) / gcd);                                                       \
                                                                                            \
    return kOK;                                                                             \
}

/* Exports */

kMath_DefinePredicateFx_(FindFirst, 32s)

kFx(kStatus) kMath_FindFirst32s(const k32s* v, kSize count, kComparison comparison, k32s value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_FindFirstEq32s(v, count, value, index);
        case kCOMPARISON_NEQ:   return kMath_FindFirstNeq32s(v, count, value, index);
        case kCOMPARISON_LT:    return kMath_FindFirstLt32s(v, count, value, index);
        case kCOMPARISON_LTE:   return kMath_FindFirstLte32s(v, count, value, index);
        case kCOMPARISON_GT:    return kMath_FindFirstGt32s(v, count, value, index);
        case kCOMPARISON_GTE:   return kMath_FindFirstGte32s(v, count, value, index);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefinePredicateFx_(FindFirst, 64f)

kFx(kStatus) kMath_FindFirst64f(const k64f* v, kSize count, kComparison comparison, k64f value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_FindFirstEq64f(v, count, value, index);
        case kCOMPARISON_NEQ:   return kMath_FindFirstNeq64f(v, count, value, index);
        case kCOMPARISON_LT:    return kMath_FindFirstLt64f(v, count, value, index);
        case kCOMPARISON_LTE:   return kMath_FindFirstLte64f(v, count, value, index);
        case kCOMPARISON_GT:    return kMath_FindFirstGt64f(v, count, value, index);
        case kCOMPARISON_GTE:   return kMath_FindFirstGte64f(v, count, value, index);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefinePredicateFx_(FindLast, 32s)

kFx(kStatus) kMath_FindLast32s(const k32s* v, kSize count, kComparison comparison, k32s value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_FindLastEq32s(v, count, value, index);
        case kCOMPARISON_NEQ:   return kMath_FindLastNeq32s(v, count, value, index);
        case kCOMPARISON_LT:    return kMath_FindLastLt32s(v, count, value, index);
        case kCOMPARISON_LTE:   return kMath_FindLastLte32s(v, count, value, index);
        case kCOMPARISON_GT:    return kMath_FindLastGt32s(v, count, value, index);
        case kCOMPARISON_GTE:   return kMath_FindLastGte32s(v, count, value, index);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefinePredicateFx_(FindLast, 64f)

kFx(kStatus) kMath_FindLast64f(const k64f* v, kSize count, kComparison comparison, k64f value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_FindLastEq64f(v, count, value, index);
        case kCOMPARISON_NEQ:   return kMath_FindLastNeq64f(v, count, value, index);
        case kCOMPARISON_LT:    return kMath_FindLastLt64f(v, count, value, index);
        case kCOMPARISON_LTE:   return kMath_FindLastLte64f(v, count, value, index);
        case kCOMPARISON_GT:    return kMath_FindLastGt64f(v, count, value, index);
        case kCOMPARISON_GTE:   return kMath_FindLastGte64f(v, count, value, index);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefineFindMin_(32s)
kMath_DefineFindMin_(64f)

kMath_DefineFindMax_(32s)
kMath_DefineFindMax_(64f)

kMath_DefineSum_(32s, 64s)

kFx(kStatus) kMath_Sum32s(const k32s* v, kSize count, k64s* sum)
{
    return kMath_Sum32s64s(v, count, sum);
}

kMath_DefineSum_(64f, 64f)

kFx(kStatus) kMath_Sum64f(const k64f* v, kSize count, k64f* sum)
{
    return kMath_Sum64f64f(v, count, sum);
}

kMath_DefineAverage_(32s)
kMath_DefineAverage_(64f)

kMath_DefineStdev_(32s)
kMath_DefineStdev_(64f)

kMath_DefineMin_(32s)
kMath_DefineMin_(64f)

kMath_DefineMax_(32s)
kMath_DefineMax_(64f)

kMath_DefineCentroid_(32s)
kMath_DefineCentroid_(64f)

kMath_DefineSet_(32s)
kMath_DefineSet_(64f)

kMath_DefineStep_(32s)
kMath_DefineStep_(64f)

kMath_DefineSpan_(32s)
kMath_DefineSpan_(64f)

kMath_DefineAbs_(32s)
kMath_DefineAbs_(64f)

kMath_DefineOpFx_(ApplyC, 32s)
kMath_DefineOpFx_(ApplyC, 64f)

kFx(kStatus) kMath_AddC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMath_ApplyCAdd32s(vIn, vOut, count, value);
}

kFx(kStatus) kMath_AddC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMath_ApplyCAdd64f(vIn, vOut, count, value);
}

kFx(kStatus) kMath_SubC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMath_ApplyCSubtract32s(vIn, vOut, count, value);
}

kFx(kStatus) kMath_SubC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMath_ApplyCSubtract64f(vIn, vOut, count, value);
}

kFx(kStatus) kMath_MulC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMath_ApplyCMultiply32s(vIn, vOut, count, value);
}

kFx(kStatus) kMath_MulC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMath_ApplyCMultiply64f(vIn, vOut, count, value);
}

kFx(kStatus) kMath_DivC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMath_ApplyCDivide32s(vIn, vOut, count, value);
}

kFx(kStatus) kMath_DivC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMath_ApplyCDivide64f(vIn, vOut, count, value);
}

kMath_DefineClampC_(32s)
kMath_DefineClampC_(64f)

kMath_DefinePredicateFx_(ReplaceC, 32s)

kFx(kStatus) kMath_ReplaceC32s(const k32s* vIn, k32s* vOut, kSize count, kComparison comparison, k32s value, k32s replacement)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_ReplaceCEq32s(vIn, vOut, count, value, replacement);
        case kCOMPARISON_NEQ:   return kMath_ReplaceCNeq32s(vIn, vOut, count, value, replacement);
        case kCOMPARISON_LT:    return kMath_ReplaceCLt32s(vIn, vOut, count, value, replacement);
        case kCOMPARISON_LTE:   return kMath_ReplaceCLte32s(vIn, vOut, count, value, replacement);
        case kCOMPARISON_GT:    return kMath_ReplaceCGt32s(vIn, vOut, count, value, replacement);
        case kCOMPARISON_GTE:   return kMath_ReplaceCGte32s(vIn, vOut, count, value, replacement);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefinePredicateFx_(ReplaceC, 64f)

kFx(kStatus) kMath_ReplaceC64f(const k64f* vIn, k64f* vOut, kSize count, kComparison comparison, k64f value, k64f replacement)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_ReplaceCEq64f(vIn, vOut, count, value, replacement);
        case kCOMPARISON_NEQ:   return kMath_ReplaceCNeq64f(vIn, vOut, count, value, replacement);
        case kCOMPARISON_LT:    return kMath_ReplaceCLt64f(vIn, vOut, count, value, replacement);
        case kCOMPARISON_LTE:   return kMath_ReplaceCLte64f(vIn, vOut, count, value, replacement);
        case kCOMPARISON_GT:    return kMath_ReplaceCGt64f(vIn, vOut, count, value, replacement);
        case kCOMPARISON_GTE:   return kMath_ReplaceCGte64f(vIn, vOut, count, value, replacement);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefinePredicateFx_(CompareC, 32s)

kFx(kStatus) kMath_CompareC32s(const k32s* vIn, kBool* vOut, kSize count, kComparison comparison, k32s value)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_CompareCEq32s(vIn, vOut, count, value);
        case kCOMPARISON_NEQ:   return kMath_CompareCNeq32s(vIn, vOut, count, value);
        case kCOMPARISON_LT:    return kMath_CompareCLt32s(vIn, vOut, count, value);
        case kCOMPARISON_LTE:   return kMath_CompareCLte32s(vIn, vOut, count, value);
        case kCOMPARISON_GT:    return kMath_CompareCGt32s(vIn, vOut, count, value);
        case kCOMPARISON_GTE:   return kMath_CompareCGte32s(vIn, vOut, count, value);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefinePredicateFx_(CompareC, 64f)

kFx(kStatus) kMath_CompareC64f(const k64f* vIn, kBool* vOut, kSize count, kComparison comparison, k64f value)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMath_CompareCEq64f(vIn, vOut, count, value);
        case kCOMPARISON_NEQ:   return kMath_CompareCNeq64f(vIn, vOut, count, value);
        case kCOMPARISON_LT:    return kMath_CompareCLt64f(vIn, vOut, count, value);
        case kCOMPARISON_LTE:   return kMath_CompareCLte64f(vIn, vOut, count, value);
        case kCOMPARISON_GT:    return kMath_CompareCGt64f(vIn, vOut, count, value);
        case kCOMPARISON_GTE:   return kMath_CompareCGte64f(vIn, vOut, count, value);
        default:                return kERROR_PARAMETER;
    }
}

kMath_DefineOpFx_(Apply, 32s)
kMath_DefineOpFx_(Apply, 64f)


kFx(kStatus) kMath_Add32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMath_ApplyAdd32s(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Add64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMath_ApplyAdd64f(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Sub32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMath_ApplySubtract32s(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Sub64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMath_ApplySubtract64f(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Mul32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMath_ApplyMultiply32s(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Mul64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMath_ApplyMultiply64f(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Div32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMath_ApplyDivide32s(vIn1, vIn2, vOut, count);
}

kFx(kStatus) kMath_Div64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMath_ApplyDivide64f(vIn1, vIn2, vOut, count);
}

kMath_DefineMovingAvg_(32s)
kMath_DefineMovingAvg_(64f)

kMath_DefineGcd_(32s)

kMath_DefineLcm_(32s)

kFx(k32s) kMath_Sign(k32s a)
{
    return (a > 0) ? 1 : ((a == 0) ? 0 : -1); 
}

/* Adapted from: http://stackoverflow.com/questions/3272424/compute-fast-log-base-2-ceiling */
kFx(k32u) kMath_Log2Ceil32u(k32u a)
{
    static const k32u t[5] = { 0xFFFF0000u, 0x0000FF00u, 0x000000F0u, 0x0000000Cu, 0x00000002u }; 
    k32u y = ((a & (a-1)) == 0) ? 0 : 1; 
    k32u j = 16; 
    k32u i, k; 

    for (i = 0; i < kCountOf(t); ++i)
    {
        k = ((a & t[i]) == 0) ? 0 : j; 
        y += k; 
        a >>= k; 
        j >>= 1; 
    }

    return y; 
}
