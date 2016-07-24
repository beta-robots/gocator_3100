/** 
 * @file    kUtils.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_UTILS_X_H
#define K_API_UTILS_X_H

kBeginHeader()

typedef struct kUtilsStatic
{
    k32u placeHolder;       //unused
} kUtilsStatic; 

kDeclareStaticClass(k, kUtils)

kFx(kStatus) kUtils_InitStatic(); 
kFx(kStatus) kUtils_ReleaseStatic(); 

kFx(kStatus) kOverrideFunctions(void* base, kSize baseSize, void* overrides); 

kFx(kStatus) kZeroContent(kType type, void* items, kSize count); 
kFx(kStatus) kCopyContent(kType type, void* dest, const void* src, kSize count); 
kFx(kStatus) kCloneContent(kType type, void* dest, const void* src, kSize count, kAlloc allocator); 
kFx(kStatus) kDisposeContent(kType type, void* items, kSize count); 
kFx(kStatus) kShareContent(kType type, void* items, kSize count); 
kFx(kSize) kSizeContent(kType type, const void* items, kSize count); 

kFx(k32s) kStrMeasuref(const kChar* format, kVarArgList argList); 

kFx(kChar) kStrAsciiToLower(kChar ch); 
kFx(kBool) kStrAsciiIsSpace(kChar ch);
kFx(kBool) kStrAsciiIsLetter(kChar ch);
kFx(kBool) kStrAsciiIsDigit(kChar ch);

kFx(kSize) kHashBytes(const void* key, kSize length); 
kFx(kSize) kHashStr(const kChar* str); 
kFx(kSize) kHashPointer(void* pointer); 

kFx(k32u) kReverseBits32(k32u input, k32u bitCount); 

kFx(kStatus) kBmpLoad(kObject *image, const char* fileName, kAlloc allocator);
kFx(kStatus) kBmpSave(kObject image, const char* fileName); 

kFx(kStatus) kObjectFromBytes6(kObject* object, const kByte* data, kSize size, kAlloc allocator); 
kFx(kStatus) kObjectToBytes6(kObject object, const kByte** data, kSize* size, kAlloc allocator); 

kFx(kStatus) kDefaultMemAlloc(kPointer receiver, kSize size, void* mem); 
kFx(kStatus) kDefaultMemFree(kPointer receiver, void* mem); 

kFx(kStatus) kStrCopyEx(kChar* dest, kSize capacity, const kChar* src, kChar** endPos);

/** 
 * Utility function, used internally by kApi for memory allocation. 
 *
 * This function calls the memory allocation callback provided via kApiLib_SetMemAllocHandlers. 
 * 
 * Most memory allocations in the kApi library are performed using the kAlloc_App allocator, 
 * including allocations performed via the kMemAlloc function.  But any allocations that are 
 * required before the kApi type system is initialized should use kSysMemAlloc instead.
 * 
 * This function ensures that allocated memory is initialized to zero.
 * 
 * @param   size    Size of memory to allocate, in bytes.
 * @param   mem     Receives a pointer to the memory block.
 * @return          Operation status. 
 */
kFx(kStatus) kSysMemAlloc(kSize size, void* mem);

/** 
 * Utility function, used internally by kApi for memory deallocation. 
 *
 * This function calls the memory deallocation callback provided via kApiLib_SetMemAllocHandlers.
 * 
 * This function should be used to free any memory allocated with the kSysMemAlloc function.
 *
 * @param   mem    Pointer to memory to free (or kNULL). 
 * @return         Operation status. 
 */
kFx(kStatus) kSysMemFree(void* mem);

/** 
 * Utility function, used internally by kApi to resize an array-based list of elements.
 *
 * Most logic in the kApi library that requires managing a dynamically-resizing list 
 * should use the kArrayList class. But if a dynamic list is required before the 
 * kApi type system is initialized, the kSysMemReallocList function can be used instead.
 * 
 * Use kSysMemFree to deallocate any memory allocated with this function.
 *
 * @param   list                Pointer to pointer to first list element (or pointer to kNULL). 
 * @param   count               Count of existing list elements that should be preserved.
 * @param   itemSize            Size of each list element, in bytes.
 * @param   capacity            On input, the current list capacity, in elements; on output, the new list capacity. 
 * @param   initialCapacity     If the list has not yet been allocated, the initial capacity that should be used, in elements.
 * @param   requiredCapacity    The minimum list capacity after reallocation, in elements.
 * @return                      Operation status. 
 */
kFx(kStatus) kSysMemReallocList(void* list, kSize count, kSize itemSize, kSize* capacity, kSize initialCapacity, kSize requiredCapacity); 

/** 
 * Default random number generator. 
 * 
 * @return      Returns a random 32-bit number. 
 */
kFx(k32u) kDefaultRandom(); 

/** 
 * Converts from microseconds to kernel time units. 
 * 
 * This function isn't used on POSIX-based systems. 
 * 
 * @param   time    Time, in microseconds (can be kINFINITE).  
 * @return          Time, in kernel time units. 
 */
kFx(k64u) kTimeToKernelTime(k64u time); 

/** 
 * Utility function to dipatch progress udpates at a limited time interval. 
 * 
 * Regardless of the frequency at which this function is called, updates are dispatched at no more 
 * than one second intervals.
 * 
 * @public                  @memberof kUtils
 * @param   progress        Optional progress callback function.
 * @param   receiver        Progress callback receiver. 
 * @param   sender          Progress callback sender. 
 * @param   updateTime      Time of most recent update (in/out); if null, update is not time-limited.
 * @param   progressValue   Current progress percentage.
 */
kFx(void) kUpdateProgress(kCallbackFx progress, kPointer receiver, kPointer sender, k64u* updateTime, k32u progressValue);

kFx(kStatus) kStrFormat8u(k8u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat8s(k8s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat16u(k16u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat16s(k16s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat32u(k32u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat32s(k32s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat64u(k64u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat64s(k64s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormatBool(kBool value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormatSize(kSize value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormatSSize(kSSize value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) kStrFormat32f(k32f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos);
kFx(kStatus) kStrFormat64f(k64f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos);

kFx(kStatus) kStrParse8u(k8u* value, const kChar* str);
kFx(kStatus) kStrParse8s(k8s* value, const kChar* str);
kFx(kStatus) kStrParse16u(k16u* value, const kChar* str);
kFx(kStatus) kStrParse16s(k16s* value, const kChar* str);
kFx(kStatus) kStrParse32u(k32u* value, const kChar* str);
kFx(kStatus) kStrParse32s(k32s* value, const kChar* str);
kFx(kStatus) kStrParse64u(k64u* value, const kChar* str);
kFx(kStatus) kStrParse64s(k64s* value, const kChar* str);
kFx(kStatus) kStrParseBool(kBool* value, const kChar* str);
kFx(kStatus) kStrParseSize(kSize* value, const kChar* str);
kFx(kStatus) kStrParseSSize(kSSize* value, const kChar* str);
kFx(kStatus) kStrParse64f(k64f* value, const kChar* str);
kFx(kStatus) kStrParse32f(k32f* value, const kChar* str);

kFx(kBool) kIsNanOrInf(k64f value);
kFx(kStatus) kStrEcvt(k64f value, kChar* buffer, kSize capacity, k32s digitCount, k32s* decPt, kBool* isNegative, kChar** endPos);

#if defined(K_POSIX)
kFx(kStatus) kFormatTimeout(k64u timeout, struct timespec* ts); 
#endif

#define kReverseBits8_(B)       \
    ((((B) & 0x80) >> 7) |      \
     (((B) & 0x40) >> 5) |      \
     (((B) & 0x20) >> 3) |      \
     (((B) & 0x10) >> 1) |      \
     (((B) & 0x08) << 1) |      \
     (((B) & 0x04) << 3) |      \
     (((B) & 0x02) << 5) |      \
     (((B) & 0x01) << 7))

#define kHashPointer_(P)                                \
    (((kSize)P >> kALIGN_ANY) |                         \
     ((kSize)P) << ((8*K_POINTER_SIZE)-kALIGN_ANY))
    
/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kAllocate            kMemAlloc
#   define kFree                kMemFree
#   define kDelete              kMemFreeRef
#   define kMemcpy              kMemCopy
#   define kMemset              kMemSet
#   define kDestroy             kDestroyRef
#   define kDispose             kDisposeRef
#   define kData_Save           kSave5
#   define kSnprintf            kStrPrintf
#   define kSprintf             kStrPrintf
#   define kStrlen              kStrLength
#   define kStrncmp             strncmp
#   define kStrcmp              strcmp
#   define kStricmp             kStrCompareLower
#   define kPrintf              kLogf

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kStrncpy(DST, SRC, SZ)       kStrCopy(DST, SZ, SRC)
#   define kData_Load(D, F)             kLoad5(D, F, kNULL)
    kFx(void*) kMalloc(kSize size); 

#endif

kEndHeader()

#endif
