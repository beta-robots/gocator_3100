/** 
 * @file    kMemory.x.h
 * @brief   Declares the kMemory class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MEMORY_X_H
#define K_API_MEMORY_X_H

#include <kApi/Io/kStream.h>

kBeginHeader()

#define kMEMORY_GROWTH_FACTOR               (2)
#define kMEMORY_MIN_CAPACITY                (256)

typedef k32s kMemoryMode; 

#define kMEMORY_MODE_NULL         (0x0)     // Read/write state unknown.
#define kMEMORY_MODE_READ         (0x1)     // Currently reading. 
#define kMEMORY_MODE_WRITE        (0x2)     // Currently writing.

typedef struct kMemoryClass
{   
    kStreamClass base; 
    kByte* buffer;
    kSize position;
    kSize length; 
    kSize capacity; 
    kBool owned; 
    kMemoryMode lastMode; 
} kMemoryClass;

kDeclareClass(k, kMemory, kStream)

kFx(kStatus) kMemory_Init(kMemory memory, kType type, kAlloc allocator); 
kFx(kStatus) kMemory_VRelease(kMemory memory); 

kFx(kStatus) kMemory_VReadSomeImpl(kMemory memory, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kMemory_VWriteImpl(kMemory memory, const void* buffer, kSize size); 
kFx(kStatus) kMemory_VSeek(kMemory memory, k64s offset, kSeekOrigin origin); 
kFx(kStatus) kMemory_VFlush(kMemory memory); 

kFx(kStatus) kMemory_Reserve(kMemory memory, kSize minimumCapacity); 

kFx(kBool) kMemory_IsAttached(kMemory memory); 

#define kMemory_Cast_(MEM)                (kCastClass_(kMemory, MEM))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_MEMORY             kTypeOf(kMemory)
#   define kMemory_Destroy          kObject_Destroy
#   define kMemory_Write            kStream_Write
#   define kMemory_Read             kStream_Read
#   define kMemory_Seek             kStream_Seek
#   define kMemory_Flush            kStream_Flush
#   define kMEMORY_CAPACITY         kMemory_Capacity

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kMemory_Data(M)          kMemory_At(M, 0)
#   define kMEMORY_DATA(M)          kMemory_At(M, 0)

    kFx(kStatus) kMemory_Construct5(kMemory* memory, void* buffer, kSize capacity); 

#endif

kEndHeader()

#endif

