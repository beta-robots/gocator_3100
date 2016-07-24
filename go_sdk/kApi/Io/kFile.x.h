/** 
 * @file    kFile.x.h
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_FILE_X_H
#define K_API_FILE_X_H

#include <kApi/Io/kStream.h>

kBeginHeader()

#define kFILE_COPY_BUFFER_SIZE       (64*1024)         //size, in bytes, of buffer used for file copy/move operations

typedef struct kFileStatic
{
    k32u placeholder;       //unused
} kFileStatic; 

typedef struct kFileVTable
{
    kStreamVTable base; 
} kFileVTable; 

kDeclareFullClass(k, kFile, kStream)

#if defined(K_PLATFORM) 

#if defined(K_WINDOWS)

#   define kFILE_MAX_IO_SIZE    (0xFFFFFFFF)

#   define kFilePlatformFields()        \
        HANDLE handle;    
    
#elif defined(K_DARWIN) || defined(K_LINUX)

#   define kFilePlatformFields()        \
        int handle;         

#else

#   define kFilePlatformFields()        \
        kPointer handle;

#endif

#define kFILE_MODE_NULL     (0)


typedef struct kFileClass
{   
    kStreamClass base; 
    k64u streamPosition; 
    k64u streamLength; 
    kFileMode lastMode;
    kFilePlatformFields()
} kFileClass;

kFx(kStatus) kFile_InitStatic(); 
kFx(kStatus) kFile_ReleaseStatic(); 

kFx(kStatus) kFile_Init(kFile file, kType type, const kChar* path, kFileMode mode, kAlloc allocator); 
kFx(kStatus) kFile_VRelease(kFile file);

kFx(kStatus) kFile_VReadSomeImpl(kFile file, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kFile_VWriteImpl(kFile file, const void* buffer, kSize size);
kFx(kStatus) kFile_VSeek(kFile file, k64s offset, kSeekOrigin origin);
kFx(kStatus) kFile_VFlush(kFile file);

kFx(kStatus) kFile_ReadAtLeast(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kFile_WriteAll(kFile file, const kByte* buffer, kSize count);

kFx(kBool) kFile_ExistsImpl(const kChar* path); 
kFx(k64u) kFile_SizeImpl(const kChar* path); 
kFx(kStatus) kFile_CopyImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context);
kFx(kStatus) kFile_MoveImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context); 
kFx(kStatus) kFile_TempNameImpl(kChar* name, kSize capacity); 

kFx(void) kFile_InitPlatformFields(kFile file);
kFx(kBool) kFile_IsOpen(kFile file);

kFx(kStatus) kFile_OpenImpl(kFile file, const kChar* path, kFileMode mode);
kFx(kStatus) kFile_CloseImpl(kFile file);
kFx(kStatus) kFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kFile_WriteImpl(kFile file, const kByte* buffer, kSize count);
kFx(kStatus) kFile_FlushImpl(kFile file); 
kFx(k64u) kFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin); 
kFx(kStatus) kFile_DeleteImpl(const kChar* path); 

#define kFile_Cast_(FILE)                (kCastClass_(kFile, FILE))

#endif

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_FILE               kTypeOf(kFile)
#   define kFile_Destroy            kObject_Destroy
#   define kFile_Write              kStream_Write
#   define kFile_Read               kStream_Read
#   define kFile_Seek               kStream_Seek
#   define kFile_Flush              kStream_Flush
#   define kFile_CreateTemp         kFile_TempName

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kFile_Construct5(F, N, I)     kFile_Construct((F), (N), (I) ? kFILE_MODE_READ : kFILE_MODE_WRITE, kNULL)
#   define kFile_Load5(P, D, S)          kFile_Load(P, D, S, kNULL)

#endif

kEndHeader()

#endif
