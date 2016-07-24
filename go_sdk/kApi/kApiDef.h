/** 
 * @file    kApiDef.h
 * @brief   Core Zen type declarations. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_DEF_H
#define K_API_API_DEF_H

#include <kApi/kApiCfg.h>

kBeginHeader()

#define kCall           kxCall              ///< kApi standard function calling convention.
#define kDlCall         kxDlCall            ///< kApi dynamic load function calling convention.

#if defined(K_EMIT)
#   define kFx(TYPE)    kExportFx(TYPE)     ///< kApi function declaration helper. 
#   define kDx(TYPE)    kExportDx(TYPE)     ///< kApi data declaration helper. 
#else
#   define kFx(TYPE)    kImportFx(TYPE)     
#   define kDx(TYPE)    kImportDx(TYPE)     
#endif

typedef void (kCall* kFunction)();          ///< Generic pointer to function.

/**
 * @struct  k8u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an 8-bit unsigned integer.
 *
 * k8u supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx8u k8u;          

/** @relates k8u @{ */
#define k8U_MIN    (0)             ///< k8u minimum value. 
#define k8U_MAX    (255U)          ///< k8u maximum value.
#define k8U_NULL   (k8U_MAX)       ///< k8u invalid value.
/** @} */

/**
 * @struct  k16u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 16-bit unsigned integer.
 *
 * k16u supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx16u k16u;           

/** @relates k16u @{ */
#define k16U_MIN   (0)             ///< k16u minimum value. 
#define k16U_MAX   (65535U)        ///< k16u maximum value. 
#define k16U_NULL  (k16U_MAX)      ///< k16u invalid value. 
/** @} */

/**
 * @struct  k32u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-bit unsigned integer.
 *
 * k32u supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx32u k32u;   

/** @relates k32u @{ */
#define k32U_MIN   (0)             ///< k32u minimum value. 
#define k32U_MAX   (4294967295U)   ///< k32u maximum value. 
#define k32U_NULL  (k32U_MAX)      ///< k32u invalid value. 
/** @} */

/**
 * @struct  k64u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-bit unsigned integer.
 *
 * k64u supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx64u k64u;         

/** @relates k64u @{ */
#define k64U(CONST) kx64U(CONST)                    ///< Declares a 64-bit unsigned integer literal.
#define k64U_MIN    k64U(0)                         ///< k64u minimum value. 
#define k64U_MAX    k64U(18446744073709551615)      ///< k64u maximum value.
#define k64U_NULL  (k64U_MAX)                       ///< k64u invalid value.
/** @} */

#define kINFINITE   k64U_MAX                        ///< Infinity (used for k64u timeouts).

/**
 * @struct  k8s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an 8-bit signed integer.
 *
 * k8s supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx8s k8s;       

/** @relates k8s @{ */
#define k8S_MAX    (127)               ///< k8s maximum value.
#define k8S_MIN    (-k8S_MAX -1)       ///< k8s minimum value.
#define k8S_NULL   (k8S_MIN)           ///< k8s invalid value.
/** @} */

/**
 * @struct  k16s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 16-bit signed integer.
 *
 * k16s supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx16s k16s;      

/** @relates k16s @{ */
#define k16S_MAX   (32767)             ///< k16s maximum value. 
#define k16S_MIN   (-k16S_MAX -1)      ///< k16s minimum value.
#define k16S_NULL  (k16S_MIN)          ///< k16s invalid value. 
/** @} */

/**
 * @struct  k32s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-bit signed integer.
 *
 * k32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx32s k32s;      

/** @relates k32s @{ */
#define k32S_MAX   (2147483647)        ///< k32s maximum value. 
#define k32S_MIN   (-k32S_MAX -1)      ///< k32s minimum value.
#define k32S_NULL  (k32S_MIN)          ///< k32s invalid value. 
/** @} */

/**
 * @struct  k64s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-bit signed integer.
 *
 * k64s supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx64s k64s;      

/** @relates k64s @{ */
#define k64S(CONST) kx64S(CONST)                ///< Declares a 64-bit signed integer literal.
#define k64S_MAX    k64S(9223372036854775807)   ///< k64s maximum value. 
#define k64S_MIN    (-k64S_MAX -1)              ///< k64s minimum value.
#define k64S_NULL   (k64S_MIN)                  ///< k64s invalid value. 
/** @} */

/**
 * @struct  k32f
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-bit floating-point number.
 *
 * k32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx32f k32f;         

/** @relates k32f @{ */
#define k32F_MIN   (1.175494351e-38F)      ///< k32f smallest positive value.
#define k32F_MAX   (3.402823466e+38F)      ///< k32f largest positive value.
#define k32F_NULL  (-k32F_MAX)             ///< k32f invalid value. 
/** @} */

/**
 * @struct  k64f
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-bit floating-point number.
 *
 * k64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef kx64f k64f;        

/** @relates k64f @{ */
#define k64F_MIN   (2.2250738585072014e-308)   ///< k64f smallest positive value.
#define k64F_MAX   (1.7976931348623157e+308)   ///< k64f largest positive value.
#define k64F_NULL  (-k64F_MAX)                 ///< k64f invalid value. 
/** @} */

/**
 * @struct  kByte
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a byte on the current platform. 
 *
 * kByte supports the kdat5 and kdat6 serialization protocols.
 */
typedef kxByte kByte;                           

/**
 * @struct  kSize
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an unsigned integer that can store a pointer address.
 *
 * kSize supports the kdat5 and kdat6 serialization protocols.
 */
typedef kxSize kSize;   

/** @relates kSize @{ */
#define kSIZE_MAX           kxSIZE_MAX      ///< Maximum value contained by kSize. 
#define kSIZE_NULL          kSIZE_MAX       ///< Invalid size value. 
/** @} */

/**
 * @struct  kSSize
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a signed integer that can store a pointer address.
 */
typedef kxSSize kSSize;   

/** @relates kSSize @{ */
#define kSSIZE_MIN       kxSSIZE_MIN        ///< Minimum value contained by kSSize. 
#define kSSIZE_MAX       kxSSIZE_MAX        ///< Maximum value contained by kSSize. 
#define kSSIZE_NULL      kSSIZE_MIN         ///< Invalid kSSize value. 
/** @} */

/**
 * @struct  kPointer
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a void pointer.
 * @see     kNULL, kIsNull
 */
typedef void* kPointer;   

#define kNULL   (0)     ///< Null pointer. 

/** 
 * Tests for equality with null pointer.
 *
 * @param   POINTER     Pointer to be compared with null. 
 * @return              kTRUE if the argument is equal to null; kFALSE otherwise. 
 */
#define kIsNull(POINTER)    kIsNull_(POINTER)          

/**
 * @struct  kBool
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a boolean value. 
 *
 * kBool supports the kdat5 and kdat6 serialization protocols.
 */
typedef k32s kBool;

/** @relates kBool @{ */
#define kFALSE     (0)     ///< Boolean false. 
#define kTRUE      (1)     ///< Boolean true. 
/** @} */

/**
 * @struct  kChar
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a single unit (byte) in a UTF-8 character. 
 *
 * kChar supports the kdat5 and kdat6 serialization protocols.
 */
typedef kxChar kChar;                           

/**
 * @struct  kText16
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 16-unit, null-terminated, kChar sequence.
 *
 * kText16 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText16[16];    

/**
 * @struct  kText32
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-unit, null-terminated, kChar sequence.
 *
 * kText32 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText32[32];    

/**
 * @struct  kText64
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-unit, null-terminated, kChar sequence.
 *
 * kText64 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText64[64];    

/**
 * @struct  kText128
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 128-unit, null-terminated, kChar sequence.
 *
 * kText128 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText128[128];  

/**
 * @struct  kText256
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 256-unit, null-terminated, kChar sequence.
 *
 * kText256 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText256[256];  

/**
 * @struct  kStatus
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an error code. 
 *
 * kStatus supports the kdat6 serialization protocol. 
 */
typedef k32s kStatus; 

/** @relates kStatus @{ */
#define kERROR_STATE               (-1000)     ///< Invalid state.
#define kERROR_NOT_FOUND           (-999)      ///< Item is not found.
#define kERROR_COMMAND             (-998)      ///< Command not recognized.
#define kERROR_PARAMETER           (-997)      ///< Parameter is invalid.
#define kERROR_UNIMPLEMENTED       (-996)      ///< Feature not implemented.
#define kERROR_MEMORY              (-994)      ///< Out of memory.
#define kERROR_TIMEOUT             (-993)      ///< Action timed out.
#define kERROR_INCOMPLETE          (-992)      ///< Buffer insufficient for data.
#define kERROR_STREAM              (-991)      ///< Error in stream.
#define kERROR_CLOSED              (-990)      ///< Resource is no longer available. 
#define kERROR_VERSION             (-989)      ///< Incompatible version.
#define kERROR_ABORT               (-988)      ///< Operation aborted.
#define kERROR_ALREADY_EXISTS      (-987)      ///< Conflicts with existing item.
#define kERROR_NETWORK             (-986)      ///< Network setup/resource error.
#define kERROR_HEAP                (-985)      ///< Heap error (leak/double-free).
#define kERROR_FORMAT              (-984)      ///< Data parsing/formatting error. 
#define kERROR_READ_ONLY           (-983)      ///< Object is read-only (cannot be written).
#define kERROR_WRITE_ONLY          (-982)      ///< Object is write-only (cannot be read). 
#define kERROR_BUSY                (-981)      ///< Agent is busy (cannot service request).
#define kERROR_CONFLICT            (-980)      ///< State conflicts with another object.
#define kERROR_OS                  (-979)      ///< Generic error reported by underlying OS.
#define kERROR_DEVICE              (-978)      ///< Hardware device error. 
#define kERROR_FULL                (-977)      ///< Resource is already fully utilized. 
#define kERROR_IN_PROGRESS         (-976)      ///< Operation is in progress, but not yet complete.
#define kERROR                     (0)         ///< General error. 
#define kOK                        (1)         ///< Operation successful. 
/** @} */

/** 
 * Returns a text string representing the name of a status code (e.g. "kERROR_STATE"). 
 * 
 * This function returns a pointer to statically-allocated memory; do not attempt to free the memory.  
 *
 * @public              @memberof kStatus
 * @param   status      Status code. 
 * @return              Null-terminated character sequence.
 */
kFx(const kChar*) kStatus_Name(kStatus status); 

/** 
 * Returns kTRUE if the given status value is not kOK.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 *
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @return              kTRUE if the argument is not equal to kOK; kFALSE otherwise. 
 * @see                 @ref kApi-Error-Handling
 */
#define kIsError(EXPRESSION)        kIsError_(EXPRESSION)

/** 
 * Returns kTRUE if the given expression value is kOK.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 *
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @return              kTRUE if the argument is equal to kOK; kFALSE otherwise. 
 * @see                 @ref kApi-Error-Handling
 */
#define kSuccess(EXPRESSION)        kSuccess_(EXPRESSION)

/** 
 * Executes a <em>return</em> statement if the given expression is not kOK. 
 * 
 * If the expression result is not kOK, the current function will return the expression result.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 *
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kCheck(EXPRESSION)      kCheck_(EXPRESSION)

/** 
 * Executes a <em>return</em> statement if the given expression is not kTRUE. 
 * 
 * If the expression result is kFALSE, the current function will return kERROR_PARAMETER.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 *
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kCheckArgs(EXPRESSION)      kCheckArgs_(EXPRESSION)

/** 
 * Executes a <em>return</em> statement if the given expression is not kTRUE. 
 * 
 * If the expression result is kFALSE, the current function will return kERROR_STATE.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 *
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kCheckState(EXPRESSION)     kCheckState_(EXPRESSION)

/** 
 * Opens a kTry error-checking block.
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kTry        kTry_

/** 
 * Used within a kTry block to jump to the first error handling block (e.g. kCatch).
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kThrow(EXPRESSION)      kThrow_(EXPRESSION)

/** 
 * Used within a kTry block to conditionally jump to the first error handling block (e.g. kCatch). 
 * 
 * If the EXPRESSION argument does not evaluate to kOK, the result of the expression 
 * is passed to an error-handling block. Otherwise, execution continues at the next statement. 
 *
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kTest(EXPRESSION)       kTest_(EXPRESSION)

/** 
 * Within a kTry block, throws kERROR_PARAMETER if the expression result is kFALSE.
 * 
 * If the EXPRESSION argument does not evaluate to kTRUE, kERROR_PARAMETER is passed to an 
 * error-handling block. Otherwise, execution continues at the next statement. 
 *
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kTestArgs(EXPRESSION)       kTestArgs_(EXPRESSION)

/** 
 * Within a kTry block, throws kERROR_STATE if the expression result is kFALSE.
 * 
 * If the EXPRESSION argument does not evaluate to kTRUE, kERROR_STATE is passed to an 
 * error-handling block. Otherwise, execution continues at the next statement. 
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kTestState(EXPRESSION)      kTestState_(EXPRESSION)

/** 
 * Closes a kTry block and opens a kCatch error-handling block.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   STATUS_POINTER  Receives the exception code.
 * @see                     @ref kApi-Error-Handling
 */
#define kCatch(STATUS_POINTER)      kCatch_(STATUS_POINTER)

/** 
 * Closes a kCatch block. 
 * 
 * If the STATUS argument is not kOK, the current function returns STATUS.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   STATUS  Result of the kCatch block. 
 * @see             @ref kApi-Error-Handling
 */
#define kEndCatch(STATUS)       kEndCatch_(STATUS)

/** 
 * Closes a kTry block and opens a kFinally block.
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kFinally        kFinally_ 

/** 
 * Closes a kFinally block. 
 * 
 * If the kTry block produced an exception, the current function returns the exception code. 
 * Otherwise, execution continues after the kFinally block. 
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kEndFinally()       kEndFinally_()

/** 
 * Closes a kTry block and opens a kCatchEx error-handling block.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   STATUS_POINTER  Receives the exception code.
 * @see                     @ref kApi-Error-Handling
 */
#define kCatchEx(STATUS_POINTER)    kCatchEx_(STATUS_POINTER)

/** 
 * Closes a kCatchEx block. 
 * 
 * The exception state is set to the value of the STATUS argument. If the exception state is 
 * not kOK, the kFinallyEx block will return the exception code to the caller when kFinallyEndEx 
 * is reached.
 * 
 * This macro is guaranteed to evaluate its arguments only once.
 * 
 * @param   STATUS  Result of the kCatchEx block. 
 * @see             @ref kApi-Error-Handling
 */
#define kEndCatchEx(STATUS)     kEndCatchEx_(STATUS)

/** 
 * Opens a kFinallyEx block.
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kFinallyEx      kFinallyEx_ 

/** 
 * Closes a kFinallyEx block. 
 * 
 * If the exception state passed by kEndCatchEx is not kOK, the current function returns the 
 * exception code. Otherwise, execution continues after the kFinallyEx block. 
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kEndFinallyEx()             kEndFinallyEx_()

/** 
 * Aborts execution if EXPRESSION is kFALSE. 
 * 
 * kAssert statements are omitted if neither K_DEBUG nor K_ASSERT is defined.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 */
#define kAssert(EXPRESSION)         kAssert_(EXPRESSION)

/** 
 * Aborts execution if the type of the OBJECT argument is not equivalent to kTypeOf(SYMBOL).  
 * 
 * Type is equivalence is determined using kObject_Is. 
 * 
 * kAssertType statements are omitted if neither K_DEBUG nor K_ASSERT is defined.
 * 
 * @param   OBJECT      Expression that evaluates to a kType value.   
 * @param   SYMBOL      Type symbol, such as <em>kArrayList</em>. 
 */
#define kAssertType(OBJECT, SYMBOL)     kAssertType_(OBJECT, SYMBOL)

/** 
 * Generates a trace event using the given tag (string literals only).
 * 
 * Type is equivalence is determined using kObject_Is. 
 * 
 * kTrace statements are omitted if K_NO_TRACE is defined. 
 * 
 * @param   TAG         String literal passed to the trace handler. 
 */
#define kTrace(TAG)                 kTrace_(TAG)

/**
 * @struct  kVersion
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a version number.
 *
 * kVersion supports the kdat6 serialization protocol.
 */
typedef k32u kVersion;

/** 
 * Creates a version value from its constituent parts. 
 *
 * @public              @memberof kVersion
 * @param   major       Major version part. 
 * @param   minor       Minor version part.  
 * @param   release     Release version part. 
 * @param   build       Build version part. 
 * @return              Version value.  
 */
kFx(kVersion) kVersion_Create(k32u major, k32u minor, k32u release, k32u build);

/** 
 * Parses a version from a formatted string. 
 *
 * @public              @memberof kVersion
 * @param   version     Receives the parsed version. 
 * @param   buffer      Formatted string (e.g. "1.2.3.4"). 
 * @return              Operation status. 
 */
kFx(kStatus) kVersion_Parse(kVersion* version, const kChar* buffer);

/** 
 * Formats a version to a string buffer. 
 *
 * @public              @memberof kVersion
 * @param   version     Version. 
 * @param   buffer      Receives formatted string (e.g. "1.2.3.4"). 
 * @param   capacity    Buffer capacity. 
 * @return              Operation status. 
 */
kFx(kStatus) kVersion_Format(kVersion version, kChar* buffer, kSize capacity);

/** 
 * Returns an integral value indicating the relationship between the versions.
 * A zero value represents both versions are equal. A positive value indicates
 * that version2 is greater than version1; a negative value indicates the 
 * opposite. 
 *
 * @public              @memberof kVersion
 * @param   version1    Version1. 
 * @param   version2    Version2. 
 * @return              Comparison result value. 
 */
kFx(k32s) kVersion_Compare(kVersion version1, kVersion version2);

/** 
 * Returns the major part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Major version.  
 */
kFx(k8u) kVersion_Major(kVersion version);

/** 
 * Returns the minor part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Minor version.  
 */
kFx(k8u) kVersion_Minor(kVersion version);

/** 
 * Returns the release part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Release version.  
 */
kFx(k8u) kVersion_Release(kVersion version);

/** 
 * Returns the build part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Build version.  
 */
kFx(k8u) kVersion_Build(kVersion version);

/** 
 * Converts k8u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k8u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k8u_Format(k8u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k8u value. 
 *
 * @public              @memberof k8u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k8u_Parse(k8u* value, const kChar* str);

/** 
 * Converts k8s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k8s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k8s_Format(k8s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k8s value. 
 *
 * @public              @memberof k8s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k8s_Parse(k8s* value, const kChar* str);

/** 
 * Converts k16u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k16u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k16u_Format(k16u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k16u value. 
 *
 * @public              @memberof k16u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k16u_Parse(k16u* value, const kChar* str);

/** 
 * Converts k16s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k16s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k16s_Format(k16s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k16s value. 
 *
 * @public              @memberof k16s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k16s_Parse(k16s* value, const kChar* str);

/** 
 * Converts k32u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k32u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k32u_Format(k32u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k32u value. 
 *
 * @public              @memberof k32u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k32u_Parse(k32u* value, const kChar* str);

/** 
 * Converts k32s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k32s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k32s_Format(k32s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k32s value. 
 *
 * @public              @memberof k32s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k32s_Parse(k32s* value, const kChar* str);

/** 
 * Converts k64u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k64u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k64u_Format(k64u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k64u value. 
 *
 * @public              @memberof k64u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k64u_Parse(k64u* value, const kChar* str);

/** 
 * Converts k64s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k64s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k64s_Format(k64s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k64s value. 
 *
 * @public              @memberof k64s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k64s_Parse(k64s* value, const kChar* str);

/** 
 * Converts kBool value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kBool
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) kBool_Format(kBool value, kChar* buffer, kSize capacity);

/** 
 * Converts string to kBool value. 
 *
 * @public              @memberof kBool
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) kBool_Parse(kBool* value, const kChar* str);

/** 
 * Converts kSize value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kSize
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) kSize_Format(kSize value, kChar* buffer, kSize capacity);

/** 
 * Converts string to kSize value. 
 *
 * @public              @memberof kSize
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) kSize_Parse(kSize* value, const kChar* str);

/** 
 * Converts kSSize value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kSSize
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) kSSize_Format(kSSize value, kChar* buffer, kSize capacity);

/** 
 * Converts string to kSSize value. 
 *
 * @public              @memberof kSSize
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) kSSize_Parse(kSSize* value, const kChar* str);

/** 
 * Converts k32f value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k32f
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k32f_Format(k32f value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k32f value. 
 *
 * @public              @memberof k32f
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k32f_Parse(k32f* value, const kChar* str);

/** 
 * Converts k64f value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k64f
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k64f_Format(k64f value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k64f value. 
 *
 * @public              @memberof k64f
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k64f_Parse(k64f* value, const kChar* str);

/**
 * @struct  kPoint16s
 * @extends kValue
 * @ingroup kApi-Data
 * @brief   2D point structure with 16-bit signed integer fields.
 *
 * kPoint16s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint16s
{
    k16s x;     ///< X-coordinate value.
    k16s y;     ///< Y-coordinate value.
} kPoint16s;

/**
 * @struct  kPoint32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   2D point structure with 32-bit signed integer fields.
 *
 * kPoint32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint32s
{
    k32s x;     ///< X-coordinate value.
    k32s y;     ///< Y-coordinate value.
} kPoint32s; 

/**
 * @struct  kPoint32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   2D point structure with 32-bit floating-point fields.
 *
 * kPoint32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint32f
{
    k32f x;     ///< X-coordinate value.
    k32f y;     ///< Y-coordinate value.
} kPoint32f;  

/**
 * @struct  kPoint64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   2D point structure with 64-bit floating-point fields.
 *
 * kPoint64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint64f
{
    k64f x;     ///< X-coordinate value.
    k64f y;     ///< Y-coordinate value.
} kPoint64f;

/** 
 * Initializes a point structure. 
 * 
 * @param   POINT  Pointer to a point structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 */
#define kPoint_Init_(POINT, X, Y)     kxPoint_Init_(POINT, X, Y)  

/**
 * @struct  kPoint3d16s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 16-bit signed integer fields.
 *
 * kPoint3d16s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d16s
{
    k16s x;     ///< X-coordinate value.
    k16s y;     ///< Y-coordinate value.
    k16s z;     ///< Z-coordinate value.
} kPoint3d16s;

/**
 * @struct  kPoint3d32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 32-bit signed integer fields.
 *
 * kPoint3d32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d32s
{
    k32s x;     ///< X-coordinate value.
    k32s y;     ///< Y-coordinate value.
    k32s z;     ///< Z-coordinate value.
} kPoint3d32s;  

/**
 * @struct  kPoint3d32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 32-bit floating-point fields.
 *
 * kPoint3d32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d32f
{
    k32f x;     ///< X-coordinate value.
    k32f y;     ///< Y-coordinate value.
    k32f z;     ///< Z-coordinate value.
} kPoint3d32f; 

/**
 * @struct  kPoint3d64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 64-bit floating-point fields.
 *
 * kPoint3d64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d64f
{
    k64f x;     ///< X-coordinate value.
    k64f y;     ///< Y-coordinate value.
    k64f z;     ///< Z-coordinate value.
} kPoint3d64f;  

/** 
 * Initializes a 3d point structure. 
 * 
 * @param   POINT  Pointer to a 3d point structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   Z      z field value. 
 */
#define kPoint3d_Init_(POINT, X, Y, Z)   kxPoint3d_Init_(POINT, X, Y, Z)

/**
 * @struct  kRect16s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 16-bit signed integer fields.
 *
 * kRect16s supports the kdat6 serialization protocol.
 */
typedef struct kRect16s
{
    k16s x;         ///< X-coordinate of the origin.
    k16s y;         ///< Y-coordinate of the origin.
    k16s width;     ///< Width of the rectangle.
    k16s height;    ///< Height of the rectangle.
} kRect16s;        

/**
 * @struct  kRect32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 32-bit signed integer fields.
 *
 * kRect32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRect32s
{
    k32s x;         ///< X-coordinate of the origin.
    k32s y;         ///< Y-coordinate of the origin.
    k32s width;     ///< Width of the rectangle.
    k32s height;    ///< Height of the rectangle.
} kRect32s;        

/**
 * @struct  kRect32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 32-bit floating-point fields. 
 *
 * kRect32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRect32f
{
    k32f x;         ///< X-coordinate of the origin.
    k32f y;         ///< Y-coordinate of the origin.
    k32f width;     ///< Width of the rectangle.
    k32f height;    ///< Height of the rectangle.
} kRect32f;     

/**
 * @struct  kRect64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 64-bit floating-point fields. 
 *
 * kRect64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRect64f
{
    k64f x;         ///< X-coordinate of the origin.
    k64f y;         ///< Y-coordinate of the origin.
    k64f width;     ///< Width of the rectangle.
    k64f height;    ///< Height of the rectangle.
} kRect64f;         

/** 
 * Initializes a rectangle structure. 
 * 
 * @param   RECT   Pointer to a rectangle structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   W      width field value. 
 * @param   H      height field value. 
 */
#define kRect_Init_(RECT, X, Y, W, H)    kxRect_Init_(RECT, X, Y, W, H) 

/**
 * @struct  kRect3d64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangular cuboid structure with 64-bit floating-point fields. 
 *
 * kRect3d64f supports the kdat6 serialization protocol.
 */
typedef struct kRect3d64f
{
    k64f x;         ///< X-coordinate of the origin.
    k64f y;         ///< Y-coordinate of the origin.
    k64f z;         ///< Z-coordinate of the origin.
    k64f width;     ///< Width of the rectangular cuboid.
    k64f height;    ///< Height of the rectangular cuboid.
    k64f depth;     ///< Depth of the rectangular cuboid.
} kRect3d64f;

/** 
 * Initializes a rectangular cuboid structure. 
 * 
 * @param   RECT   Pointer to rectangular cuboid structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   Z      z field value.
 * @param   W      width field value. 
 * @param   H      height field value.
 * @param   D      depth field value.
 */
#define kRect3d_Init_(RECT, X, Y, Z, W, H, D)    kxRect3d_Init_(RECT, X, Y, Z, W, H, D) 

/**
 * @struct  kRotatedRect32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rotated rectangle structure with 32-bit signed integer fields. 
 *
 * kRotatedRect32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRotatedRect32s
{
    k32s xc;        ///< X-coordinate of the rectangle center.
    k32s yc;        ///< Y-coordinate of the rectangle center.
    k32s width;     ///< Width of the rectangle.
    k32s height;    ///< Height of the rectangle.
    k32s angle;     ///< Rotation angle of the rectangle.
} kRotatedRect32s;

/**
 * @struct  kRotatedRect32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rotated rectangle structure with 32-bit floating-point fields. 
 *
 * kRotatedRect32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRotatedRect32f
{
    k32f xc;        ///< X-coordinate of the rectangle center.
    k32f yc;        ///< Y-coordinate of the rectangle center.
    k32f width;     ///< Width of the rectangle.
    k32f height;    ///< Height of the rectangle.
    k32f angle;     ///< Rotation angle of the rectangle.
} kRotatedRect32f;  

/** 
 * Initializes a rotated rectangle structure. 
 * 
 * @param   RECT   Pointer to a rotated rectangle structure.    
 * @param   XC     xc field value. 
 * @param   YC     yc field value. 
 * @param   W      width field value. 
 * @param   H      height field value. 
 * @param   A      angle field value. 
 */
#define kRotatedRect_Init_(RECT, XC, YC, W, H, A)   kxRotatedRect_Init_(RECT, XC, YC, W, H, A) 

/**
 * @struct  kPixelFormat
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Pixel format descriptor.
 *
 * kPixelFormat supports the kdat6 serialization protocol.
 */
typedef k32s kPixelFormat; 

/** @relates kPixelFormat @{ */
#define kPIXEL_FORMAT_NULL              (0)      ///< Unknown pixel format.
#define kPIXEL_FORMAT_8BPP_GREYSCALE    (1)      ///< 8-bit greyscale (k8u)
#define kPIXEL_FORMAT_8BPP_CFA          (2)      ///< 8-bit color filter array (k8u)
#define kPIXEL_FORMAT_8BPC_BGRX         (3)      ///< 8-bits-per-channel color with 4 channels (blue/green/red/unused)(kRgb)
/** @} */

/**
 * @struct  kCfa
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Image color filter array type.
 *
 * kCfa supports the kdat6 serialization protocol.
 */
typedef k32s kCfa; 

/** @relates kCfa @{ */
#define kCFA_NONE            (0)      ///< No color filter.
#define kCFA_BAYER_BGGR      (1)      ///< Bayer filter: BG/GR.
#define kCFA_BAYER_GBRG      (2)      ///< Bayer filter: GB/RG.
#define kCFA_BAYER_RGGB      (3)      ///< Bayer filter: RG/GB.
#define kCFA_BAYER_GRBG      (4)      ///< Bayer filter: GR/BG.
/** @} */

/**
 * @struct  kRgb
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   32-bit color pixel structure (B/G/R/X). 
 *
 * kRgb supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRgb
{
    k8u b;      ///< Blue component value.
    k8u g;      ///< Green component value.
    k8u r;      ///< Red component value.
    k8u x;      ///< Undefined.
} kRgb;

/** 
 * Initializes a kRgb structure. 
 * 
 * @relates        kRgb
 * @param   RGB    Pointer to a kRgb structure.    
 * @param   R      r field value. 
 * @param   G      g field value. 
 * @param   B      b field value. 
 */
#define kRgb_Init_(RGB, R, G, B)  kxRgb_Init_(RGB, R, G, B) 

/**
 * @struct  kArgb
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   32-bit color pixel structure (B/G/R/A). 
 *
 * kArgb supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kArgb
{
    k8u b;      ///< Blue component value.
    k8u g;      ///< Green component value.
    k8u r;      ///< Red component value.
    k8u a;      ///< Alpha component value.
} kArgb;

/** 
 * Initializes a kArgb structure. 
 * 
 * @relates        kArgb
 * @param   ARGB   Pointer to a kArgb structure.
 * @param   A      a field value. 
 * @param   R      r field value. 
 * @param   G      g field value. 
 * @param   B      b field value. 
 */
#define kArgb_Init_(ARGB, A, R, G, B) kxArgb_Init_(ARGB, A, R, G, B)

/**
 * @struct  kComparison
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Represents a comparison type.
 */
typedef k32s kComparison; 

#define kCOMPARISON_EQ      (0)        ///< Is equal. @relates kComparison
#define kCOMPARISON_NEQ     (1)        ///< Is not equal. @relates kComparison
#define kCOMPARISON_LT      (2)        ///< Is less than. @relates kComparison
#define kCOMPARISON_LTE     (3)        ///< Is less than or equal. @relates kComparison
#define kCOMPARISON_GT      (4)        ///< Is greater than. @relates kComparison
#define kCOMPARISON_GTE     (5)        ///< Is greater than or equal. @relates kComparison

/** 
 * Callback signature to determine equality of two items.
 * 
 * @param   item1  Pointer to first item.    
 * @param   item2  Pointer to second item. 
 * @return         kTRUE if the arguments are equal; kFALSE otherwise. 
 */
typedef kBool (kCall* kEqualsFx)(const void* item1, const void* item2); 

/** 
 * Callback signature to determine hash code of an item.
 * 
 * @param   item   Pointer to item.    
 * @return         Item hash code.  
 */
typedef kSize (kCall* kHashFx)(const void* item); 

/** 
 * Callback signature for a generic event handler.
 * 
 * @param   receiver   Receiver context pointer.     
 * @param   sender     Sender context pointer. 
 * @param   args       Pointer to callback argument.     
 * @return             Operation status. 
 */
typedef kStatus (kCall* kCallbackFx)(kPointer receiver, kPointer sender, void* args); 

/**
 * @struct  kCallback
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a callback function and context pointer.
 */
typedef struct kCallback
{
    kCallbackFx function;       ///< Callback function.
    kPointer receiver;          ///< Callback receiver context pointer. 
} kCallback; 

/**
 * @struct  kFileMode
 * @extends kValue
 * @ingroup kApi-Io  
 * @brief   Flags that control how a file is opened. 
 */
typedef k32s kFileMode;         

/** @relates kFileMode @{ */
#define kFILE_MODE_READ         (0x1)     ///< Open the file with permission to read. 
#define kFILE_MODE_WRITE        (0x2)     ///< Open the file with permission to write.
#define kFILE_MODE_UPDATE       (0x4)     ///< Preserve contents when opened for writing.   
/** @} */

/**
 * @struct  kSeekOrigin
 * @extends kValue
 * @ingroup kApi-Io  
 * @brief   Represents a stream seek origin.
 */
typedef k32s kSeekOrigin; 

#define kSEEK_ORIGIN_BEGIN      (0)     ///< Seek relative to the start of stream. @relates kSeekOrigin
#define kSEEK_ORIGIN_CURRENT    (1)     ///< Seek relative to the current position. @relates kSeekOrigin
#define kSEEK_ORIGIN_END        (2)     ///< Seek relative to the end of stream. @relates kSeekOrigin

/**
* Returns the number of elements in a C array.
*
* Equivalent to sizeof(CARRAY)/sizeof(CARRAY[0]).
*
* @param   CARRAY     C array variable name.
* @return             Count of array elements.
*/
#define kCountOf(CARRAY)    kCountOf_(CARRAY)

/**
* Casts the ITEM argument to the specified TYPE.
*
* Equivalent (TYPE)(ITEM).
*
* This macro is guaranteed to evaluate its arguments only once.
*
* @param   TYPE     Type to which the item is cast.
* @param   ITEM     Value to be cast.
* @return           Result of the cast.
*/
#define kCast(TYPE, ITEM)       kCast_(TYPE, ITEM)     

/**
* Returns the minimum of two numbers.
*
* @param   A   First value.
* @param   B   Second value.
* @return      The lesser of A or B.
*/
#define kMin_(A, B)         kxMin_(A, B) 

/**
* Returns the maximum of two numbers.
*
* @param   A   First value.
* @param   B   Second value.
* @return      The greater of A or B.
*/
#define kMax_(A, B)         kxMax_(A, B)

/**
* Returns a value limited to the specified range.
*
* @param   V           Input value.
* @param   VMIN        Minimum output value.
* @param   VMAX        Maximum output value.
* @return              The input value, limited to [VMIN, VMAX].
*/
#define kClamp_(V, VMIN, VMAX)      kxClamp_((V), (VMIN), (VMAX))

/**
* Returns the absolute value of a number.
*
* @param   A   Input value.
* @return      Absolute value of input argument.
*/
#define kAbs_(A)            kxAbs_(A)

/**
* Sets all bits of a structure to zero.
*
* @param   VALUE   Structure instance.
*/
#define kZero_(VALUE)       kxZero_(VALUE)

/**
* Performs a small copy with minimal overhead.
*
* @param   DEST    Destination address.
* @param   SRC     Source address.
* @param   SIZE    Transfer size (bytes).
*/
#define kItemCopy_(DEST, SRC, SIZE)    kxItemCopy_(DEST, SRC, SIZE)       

/**
* Performs a small fill with minimal overhead.
*
* @param   DEST    Destination address.
* @param   FILL    Value to fill (byte).
* @param   SIZE    Fill size (bytes).
*/
#define kItemSet_(DEST, FILL, SIZE)    kxItemSet_(DEST, FILL, SIZE)

/**
* Zero-initializes a small amount of memory with minimal overhead.
*
* @param   DEST    Destination address.
* @param   SIZE    Transfer size (bytes).
*/
#define kItemZero_(DEST, SIZE)          kItemSet_(DEST, 0, SIZE)

/**
* Gets a pointer to the Nth element of an array.
*
* @param   BASE    Array pointer.
* @param   INDEX   Element index.
* @param   SIZE    Element size.
*/
#define kItemAt_(BASE, INDEX, SIZE)     kAt_((BASE), (INDEX)*(SIZE))


/** 
 * Returns the kType object associated with the specified class, interface, or value symbol. 
 * 
 * This macro is used to access type information by compile-time symbol name.  E.g. 
 * 
 * @code {.c}
 * 
 * #include <kApi/Data/kImage.h>
 * 
 * void PrintImageTypeInfo()
 * {
 *     kType type = kTypeOf(kImage); 
 *     
 *     printf("Type name: %s\n", kType_Name(type)); 
 *     printf("Base class: %s\n", kType_Name(kType_Base(type))); 
 * }
 * 
 * @endcode
 * 
 * Use of this macro requires that the header file defining the specified type symbol has 
 * been included.  For example, kTypeOf(kArrayList) requires inclusion of <kApi/Data/kArrayList.h>.
 * 
 * @param   SYMBOL     Type symbol, such as kArrayList or k32s.
 * @return             kType object representing metadata about the specified type.
 */
#define kTypeOf(SYMBOL)     kTypeOf_(SYMBOL)

/** 
 * Returns the kAssembly object associated with the specified assembly symbol. 
 * 
 * This macro is used to access assembly information by compile-time symbol name.  E.g. 
 * 
 * @code {.c}
 * 
 * #include <kApi/kApiLib.h>
 * 
 * void PrintCoreAssemblyInfo()
 * {
 *     kAssembly assembly = kAssemblyOf(kApiLib); 
 *     
 *     printf("Assembly name: %s\n", kAssembly_Name(assembly)); 
 *     printf("Type count: %u\n", (k32u) kAssembly_TypeCount(assembly));  
 * }
 * 
 * @endcode
 * 
 * Use of this macro requires that the header file defining the specified assembly symbol 
 * has been included. For example, kAssemblyOf(kApiLib) requires inclusion of <kApi/kApiLib.h>
 * 
 * @param   SYMBOL     Assembly symbol, such as <em>kApiLib</em>.
 * @return             kAssembly object representing metadata about the specified assembly. 
 */
#define kAssemblyOf(SYMBOL)     kAssemblyOf_(SYMBOL)

/** 
 * Returns static data associated with the specified class symbol.
 * 
 * This macro is used within class implementations to access static state. 
 * 
 * @param   SYMBOL     Class symbol, such as <em>kArrayList</em>.
 * @return             Pointer to static data for the specified class.  
 */
#define kStaticOf(SYMBOL)   kStaticOf_(SYMBOL)

/** 
 * Indicates whether static initialization has completed for the specified type symbol. 
 * 
 * @param   SYMBOL     Class symbol, such as <em>kArrayList</em>.
 * @return             kTRUE if initialization has completed; kFALSE otherwise.
 */
#define kStaticInitialized(SYMBOL)   kStaticInitialized_(SYMBOL)

/**
 * Declares a type assembly.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Assembly symbol name (e.g. <em>kApi</em>). 
 * @see                 @ref kApi-Extending, kBeginAssembly, kEndAssembly
 */
#define kDeclareAssembly(PREFIX, SYMBOL)        kDeclareAssembly_(PREFIX, SYMBOL)       

/**
 * Starts the definition of a type assembly.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Assembly symbol name (e.g. <em>kApi</em>). 
 * @param   VERSION     Assembly version string (e.g. "6.0.0.0"). 
 * @see                 @ref kApi-Extending, kEndAssembly
 */
#define kBeginAssembly(PREFIX, SYMBOL, VERSION)     kBeginAssembly_(PREFIX, SYMBOL, VERSION)  

/**
 * Ends the definition of a type assembly.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndAssembly()           kEndAssembly_()  

/**
 * Declares type information for a structure value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>k32s</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginValue, kEndValue
 */
#define kDeclareValue(PREFIX, SYMBOL, BASE)         kDeclareValue_(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of a structure value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>k32s</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndValue
 */
#define kBeginValue(PREFIX, SYMBOL, BASE)       kBeginValue_(PREFIX, SYMBOL, BASE)

/**
 * Ends the definition of a structure value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndValue()         kEndValue_() 

/**
 * Declares type information for an enumeration value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kStatus</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginEnum, kEndEnum
 */
#define kDeclareEnum(PREFIX, SYMBOL, BASE)      kDeclareEnum_(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an enumeration value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kStatus</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndEnum
 */
#define kBeginEnum(PREFIX, SYMBOL, BASE)         kBeginEnum_(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of an enumeration value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndEnum()           kEndEnum_()

/**
 * Declares type information for a bit-flag enumeration value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kFileMode</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginBitEnum, kEndBitEnum
 */
#define kDeclareBitEnum(PREFIX, SYMBOL, BASE)   kDeclareBitEnum_(PREFIX, SYMBOL, BASE)

/**
 * Starts the definition of a bit-flag enumeration value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kFileMode</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndBitEnum
 */
#define kBeginBitEnum(PREFIX, SYMBOL, BASE)         kBeginBitEnum_(PREFIX, SYMBOL, BASE)

/**
 * Ends the definition of a bit-flag enumeration value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndBitEnum()       kEndBitEnum_()  

/**
 * Declares type information for an array-based value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kText32</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginArrayValue, kEndArrayValue
 */
#define kDeclareArrayValue(PREFIX, SYMBOL, BASE)        kDeclareArrayValue_(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an array-based value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kText32</em>). 
 * @param   TYPE        Array element type symbol(e.g. <em>kChar</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndArrayValue
 */
#define kBeginArrayValue(PREFIX, SYMBOL, TYPE, BASE)        kBeginArrayValue_(PREFIX, SYMBOL, TYPE, BASE) 

/**
 * Ends the definition of an array-based value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndArrayValue()        kEndArrayValue_()

/**
 * Declares type information for an interface type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Interface symbol(e.g. <em>kCollection</em>). 
 * @param   BASE        Interface base symbol (typically <em>kNull</em>). 
 * @see                 @ref kApi-Extending, kBeginInterface, kEndInterface
 */
#define kDeclareInterface(PREFIX, SYMBOL, BASE)         kDeclareInterface_(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an interface type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Interface symbol(e.g. <em>kCollection</em>). 
 * @param   BASE        Interface base symbol (typically <em>kNull</em>). 
 * @see                 @ref kApi-Extending, kEndInterface
 */
#define kBeginInterface(PREFIX, SYMBOL, BASE)           kBeginInterface_(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of an interface type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndInterface()         kEndInterface_()  

/**
 * Declares type information for a class type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kFile</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginFullClass, kEndFullClass
 */
#define kDeclareFullClass(PREFIX, SYMBOL, BASE)         kDeclareFullClass_(PREFIX, SYMBOL, BASE)  

/**
 * Starts the definition of a class type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kFile</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kEndFullClass
 */
#define kBeginFullClass(PREFIX, SYMBOL, BASE)       kBeginFullClass_(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of a class type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndFullClass()         kEndFullClass_()

/**
 * Declares type information for a class type that requires an expanded vtable but does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kAlloc</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginVirtualClass, kEndVirtualClass
 */
#define kDeclareVirtualClass(PREFIX, SYMBOL, BASE)          kDeclareVirtualClass_(PREFIX, SYMBOL, BASE)

/**
 * Starts the definition of a class type that requires an expanded vtable but does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kAlloc</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kEndVirtualClass
 */
#define kBeginVirtualClass(PREFIX, SYMBOL, BASE)        kBeginVirtualClass_(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of a class type that requires an expanded vtable but does not have static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndVirtualClass()          kEndVirtualClass_()   

/**
 * Declares type information for a class type that has only static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kNetwork</em>). 
 * @see                 @ref kApi-Extending, kBeginStaticClass, kEndStaticClass
 */
#define kDeclareStaticClass(PREFIX, SYMBOL)         kDeclareStaticClass_(PREFIX, SYMBOL) 

/**
 * Starts the definition of a class type that has only static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kNetwork</em>). 
 * @see                 @ref kApi-Extending, kEndFullClass
 */
#define kBeginStaticClass(PREFIX, SYMBOL)       kBeginStaticClass_(PREFIX, SYMBOL)  

/**
 * Ends the definition of a class type that has only static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndStaticClass()           kEndStaticClass_()

/**
 * Declares type information for a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kArrayList</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginClass, kEndClass
 */
#define kDeclareClass(PREFIX, SYMBOL, BASE)         kDeclareClass_(PREFIX, SYMBOL, BASE)  

/**
 * Starts the definition of a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kArrayList</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kEndClass
 */
#define kBeginClass(PREFIX, SYMBOL, BASE)       kBeginClass_(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndClass()         kEndClass_()

/**
 * Within an assembly definition, specifies a dependency on another assembly. 
 * 
 * @param   SYMBOL      Dependency target (e.g. <em>kApi</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddDependency(SYMBOL)              kAddDependency_(SYMBOL)    

/**
 * Within an assembly definition, adds a type to the assembly.
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddType(SYMBOL)        kAddType_(SYMBOL)   

/**
 * Within an assembly definition, indicates a requirement on static initialization order. 
 * 
 * The order of kAddPriority statements within an assembly definition determines the order
 * in which types will be initialized. If initialization priorities are not given, types
 * can be initialized in any order. 
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddPriority(SYMBOL)        kAddPriority_(SYMBOL)

/**
 * Within a type definition, indicates that a type implements the specified interface.
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @param   IFACE       Interface symbol (e.g. <em>kCollection</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddInterface(SYMBOL, IFACE)        kAddInterface_(SYMBOL, IFACE) 

/**
 * Within a type definition, indicates that a type has the specified non-virtual method. 
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @param   METHOD      Method name (e.g. <em>Remove</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddMethod(SYMBOL, METHOD)           kAddMethod_(SYMBOL, METHOD)  

/**
 * Within a type definition, indicates that a type overrides the specified virtual method. 
 * 
 * @param   IN_TYPE     Overriding type (e.g. <em>kArrayList</em>). 
 * @param   FROM_TYPE   Overridden type (e.g. <em>kObject</em>). 
 * @param   METHOD      Method name (e.g. <em>VRelease</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddVMethod(IN_TYPE, FROM_TYPE, METHOD)           kAddVMethod_(IN_TYPE, FROM_TYPE, METHOD)   

/**
 * Within a type definition, indicates that a type implements the specified interface method. 
 * 
 * @param   IN_TYPE     Overriding type (e.g. <em>kArrayList</em>). 
 * @param   FROM_IFACE  Overridden interface (e.g. <em>kCollection</em>). 
 * @param   IMETHOD     Interface method name (e.g. <em>VGetIterator</em>). 
 * @param   CMETHOD     Type method name (e.g. <em>GetIterator</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddIVMethod(IN_TYPE, FROM_IFACE, IMETHOD, CMETHOD)        kAddIVMethod_(IN_TYPE, FROM_IFACE, IMETHOD, CMETHOD)

/**
 * Within a structure type definition, indicates that a structure has the specified field. 
 * 
 * @param   VALUE       Structure type (e.g. <em>kPoint32s</em>). 
 * @param   FIELD_TYPE  Field type (e.g. <em>k32s</em>). 
 * @param   FIELD       Field name (e.g. <em>x</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddField(VALUE, FIELD_TYPE, FIELD)         kAddField_(VALUE, FIELD_TYPE, FIELD)

/**
 * Within an enumeration type definition, indicates that an enumeration has the specified enumerator. 
 * 
 * @param   VALUE       Enumeration type (e.g. <em>kStatus</em>). 
 * @param   ENUMERATOR  Enumerator(e.g. <em>kERROR</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddEnumerator          kAddEnumerator_

/**
 * Within a type definition, indicates that a type has the specified serialization version. 
 * 
 * @param   TYPE            Type symbol (e.g. <em>kArrayList</em>). 
 * @param   FORMAT          Serialization format name string (e.g. "kdat6"). 
 * @param   FORMAT_VER      Serialization format version string (e.g. "5.7.1.0"). 
 * @param   GUID            Type identifier string within serialization format (e.g. "kArrayList-0"). 
 * @param   WRITE_METHOD    Serialization write method (e.g. <em>WriteDat6V0</em>). 
 * @param   READ_METHOD     Serialization read method (e.g. <em>ReadDat6V0</em>). 
 * @see                     @ref kApi-Extending, kSerializer
 */
#define kAddVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)          \
    kAddVersion_(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)   

/**
 * Adds specific flags to type metadata. 
 * 
 * @param   TYPE        Type (e.g. <em>kObject</em>). 
 * @param   FLAGS       Type flags (e.g. <em>kTYPE_FLAGS_ABSTRACT</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddFlags(TYPE, FLAGS)          kAddFlags_(TYPE, FLAGS)     


/**
 * The kDefineDebugHints macro can be helpful when attempting to use debug expressions to peek inside
 * kApi class implementations in Visual Studio. The underlying problem that this macro addresses
 * is that if a structure definition isn't <em>directly</em> used by the particular application or library
 * that is being debugged, then the debugger won't be able to locate it. E.g., if the debug expression 
 * "(kXmlClass*)myXmlObject" is used, but the kXmlClass structure isn't directly used by the library 
 * that is being debugged (instead, only the kXml handle definition is used), then the debug expression 
 * can't be evaluated.
 *
 * To make use of this feature, select <em>one</em> file in your library or program and include the 
 * following lines:
 *
 * @code {.c}
 *
 * #include <kApi/kApi.h>
 *
 * kDefineDebugHints()
 * 
 * @endcode {.c}
 *
 */
#define kDefineDebugHints()          kDefineDebugHints_()

kEndHeader()

#include <kApi/kApiDef.x.h>

#endif
