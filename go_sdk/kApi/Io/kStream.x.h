/** 
 * @file    kStream.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STREAM_X_H
#define K_API_STREAM_X_H

kBeginHeader()

typedef struct kStreamClass
{
    kObjectClass base; 
    kByte* readBuffer;                          // Read buffer base pointer.
    kSize readCapacity;                         // Read buffer capacity, in bytes.
    kSize readBegin;                            // Current read location within read buffer. 
    kSize readEnd;                              // End of valid data within read buffer. 
    kByte* writeBuffer;                         // Write buffer base pointer.
    kSize writeCapacity;                        // Write buffer capacity, in bytes. 
    kSize writeBegin;                           // Current write location within write buffer.
    kSize writeEnd;                             // End of writeable region within write buffer.
    k64u bytesRead;                             // Total bytes read from underlying medium.
    k64u bytesWritten;                          // Total bytes written to underlying medium.
} kStreamClass;

typedef struct kStreamVTable
{
    kObjectVTable base; 
    kStatus (kCall* VReadSomeImpl)(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
    kStatus (kCall* VReadImpl)(kStream stream, void* buffer, kSize size);       //deprecated; use VReadSomeImpl instead
    kStatus (kCall* VWriteImpl)(kStream stream, const void* buffer, kSize size);
    kStatus (kCall* VSeek)(kStream stream, k64s offset, kSeekOrigin origin);
    kStatus (kCall* VFlush)(kStream stream);
} kStreamVTable; 

kDeclareVirtualClass(k, kStream, kObject)

kFx(kStatus) kStream_Init(kStream stream, kType type, kAlloc allocator);
kFx(kStatus) kStream_VRelease(kStream stream);

kFx(kStatus) kStream_VReadSomeImpl(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kStream_VReadImpl(kStream stream, void* buffer, kSize size);      //deprecated; use VReadSomeImpl instead
kFx(kStatus) kStream_VWriteImpl(kStream stream, const void* buffer, kSize size);
kFx(kStatus) kStream_VSeek(kStream stream, k64s offset, kSeekOrigin origin);
kFx(kStatus) kStream_VFlush(kStream stream); 

kFx(k64u) kStream_VBytesRead(kStream stream); 
kFx(k64u) kStream_VBytesWritten(kStream stream); 

#define kStream_(S)             (kCast(kStreamClass*, S))
#define kStream_Cast_(S)        (kCastClass_(kStream, S))
#define kStream_VTable_(S)      (kCastVTable_(kStream, S))

#define kStream_ReadSomeImpl_(S, BUF, MIN, MAX, READ)   (kStream_VTable_(S)->VReadSomeImpl(S, BUF, MIN, MAX, READ))
#define kStream_ReadImpl_(S, BUF, SIZE)                 (kStream_VTable_(S)->VReadImpl(S, BUF, SIZE))
#define kStream_WriteImpl_(S, BUF, SIZE)                (kStream_VTable_(S)->VWriteImpl(S, BUF, SIZE))
#define kStream_Seek_(S, OFFSET, ORIGIN)                (kStream_VTable_(S)->VSeek(S, OFFSET, ORIGIN))
#define kStream_Flush_(S)                               (kStream_VTable_(S)->VFlush(S))

#define kxStream_BytesRead_(S)                                                      \
    (kStream_(S)->bytesRead - (k64u)(kStream_(S)->readEnd - kStream_(S)->readBegin))

#define kxStream_BytesWritten_(S)                                                   \
    (kStream_(S)->bytesWritten + (k64u)kStream_(S)->writeBegin)

#define kStream_Read_(S, BUF, SIZE)                                                 \
    (((kStream_(S)->readEnd - kStream_(S)->readBegin) < (SIZE)) ?                   \
      (kStream_VTable_(S)->VReadSomeImpl(S, BUF, SIZE, SIZE, kNULL)) :              \
      (kMemCopy(BUF, &kStream_(S)->readBuffer[kStream_(S)->readBegin], (SIZE)),     \
       kStream_(S)->readBegin += (SIZE),                                            \
       kOK))

#define kStream_Read1_(S, BUF)                                                      \
    (((kStream_(S)->readEnd - kStream_(S)->readBegin) < 1) ?                        \
      (kStream_VTable_(S)->VReadSomeImpl(S, BUF, 1, 1, kNULL)) :                    \
      (kItemCopy1_(BUF, &kStream_(S)->readBuffer[kStream_(S)->readBegin]),          \
       kStream_(S)->readBegin += 1,                                                 \
       kOK))

#define kStream_Read2_(S, BUF)                                                      \
    (((kStream_(S)->readEnd - kStream_(S)->readBegin) < 2) ?                        \
      (kStream_VTable_(S)->VReadSomeImpl(S, BUF, 2, 2, kNULL)) :                    \
      (kItemCopy2_(BUF, &kStream_(S)->readBuffer[kStream_(S)->readBegin]),          \
       kStream_(S)->readBegin += 2,                                                 \
       kOK))

#define kStream_Read4_(S, BUF)                                                      \
    (((kStream_(S)->readEnd - kStream_(S)->readBegin) < 4) ?                        \
      (kStream_VTable_(S)->VReadSomeImpl(S, BUF, 4, 4, kNULL)) :                    \
      (kItemCopy4_(BUF, &kStream_(S)->readBuffer[kStream_(S)->readBegin]),          \
       kStream_(S)->readBegin += 4,                                                 \
       kOK))

#define kStream_Read8_(S, BUF)                                                      \
    (((kStream_(S)->readEnd - kStream_(S)->readBegin) < 8) ?                        \
      (kStream_VTable_(S)->VReadSomeImpl(S, BUF, 8, 8, kNULL)) :                    \
      (kItemCopy8_(BUF, &kStream_(S)->readBuffer[kStream_(S)->readBegin]),          \
       kStream_(S)->readBegin += 8,                                                 \
       kOK))

#define kStream_Write_(S, BUF, SIZE)                                                \
    (((kStream_(S)->writeEnd - kStream_(S)->writeBegin) < (SIZE)) ?                 \
      (kStream_VTable_(S)->VWriteImpl(S, BUF, SIZE)) :                              \
      (kMemCopy(&kStream_(S)->writeBuffer[kStream_(S)->writeBegin], BUF, SIZE),     \
       kStream_(S)->writeBegin += (SIZE),                                           \
       kOK))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_STREAM             kTypeOf(kStream)

#endif

kEndHeader()

#endif
