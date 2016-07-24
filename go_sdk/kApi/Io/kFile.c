/** 
 * @file    kFile.c
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kFile.h>
#include <kApi/Io/kDirectory.h>
#include <kApi/Io/kPath.h>
#include <kApi/Data/kArray1.h>

kBeginFullClass(k, kFile, kStream)
    kAddVMethod(kFile, kObject, VRelease)
    kAddVMethod(kFile, kStream, VReadSomeImpl)
    kAddVMethod(kFile, kStream, VWriteImpl)
    kAddVMethod(kFile, kStream, VSeek)
    kAddVMethod(kFile, kStream, VFlush)
kEndFullClass()

kFx(kStatus) kFile_InitStatic()
{
    kApiFileFx handlers = { kNULL }; 

    handlers.open = kFile_OpenImpl; 
    handlers.close = kFile_CloseImpl; 
    handlers.read = kFile_ReadImpl; 
    handlers.write = kFile_WriteImpl; 
    handlers.flush = kFile_FlushImpl; 
    handlers.seek = kFile_SeekImpl; 
    handlers.size = kFile_SizeImpl; 
    handlers.exists = kFile_ExistsImpl; 
    handlers.copy = kFile_CopyImpl; 
    handlers.move = kFile_MoveImpl; 
    handlers.del = kFile_DeleteImpl; 
    handlers.tempName = kFile_TempNameImpl; 

    //respect handlers that have already been installed    
    if (kApiLib_HasFileHandlers_())
    {
        kCheck(kOverrideFunctions(&handlers, sizeof(handlers), kApiLib_FileHandlers_())); 
    }
        
    kCheck(kApiLib_SetFileHandlers(&handlers)); 
 
    return kOK; 
}

kFx(kStatus) kFile_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kFile_Load(const kChar* path, void* data, kSize* size, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kFile file = kNULL; 
    kByte* buffer = kNULL; 
    k64u fileSize; 
    kStatus exception; 

    kTry
    {
        kTest(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)); 

        if ((fileSize = kFile_Length(file)) >= kSIZE_MAX)
        {
            kThrow(kERROR_INCOMPLETE); 
        }

        kTest(kAlloc_Get(alloc, (kSize) fileSize, &buffer)); 
        kTest(kStream_Read(file, buffer, (kSize) fileSize)); 

        *(void**)data = buffer; 
        *size = (kSize) fileSize; 

        kTest(kFile_Close(file)); 
    }
    kCatchEx(&exception)     
    {
        kAlloc_Free(alloc, buffer); 
        kEndCatchEx(exception); 
    }
    kFinallyEx
    {
        kCheck(kDestroyRef(&file)); 
        kEndFinallyEx(); 
    }

    return kOK; 
}

kFx(kStatus) kFile_LoadTo(const kChar* path, void* data, kSize capacity)
{
    kFile file = kNULL; 

    kTry
    {
        kTest(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)); 
        kTestErr(kFile_Length(file) == capacity, INCOMPLETE); 

        kTest(kStream_Read(file, data, capacity)); 
        kTest(kFile_Close(file)); 
    }
    kFinally
    {
        kDestroyRef(&file); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kFile_Save(const kChar* path, const kByte* data, kSize size)
{
    kFile file = kNULL; 

    kTry
    {
        kTest(kFile_Construct(&file, path, kFILE_MODE_WRITE, kNULL)); 
        kTest(kStream_Write(file, data, size)); 
        kTest(kFile_Close(file)); 
    }
    kFinally
    {
        kDestroyRef(&file); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kFile_Construct(kFile* file, const kChar* path, kFileMode mode, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kFile); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, file)); 

    if (!kSuccess(status = kFile_Init(*file, type, path, mode, alloc)))
    {
        kAlloc_FreeRef(alloc, file); 
    }

    return status; 
} 

kFx(kStatus) kFile_Init(kFile file, kType type, const kChar* path, kFileMode mode, kAlloc allocator)
{
    kFileClass *obj = file; 
    kStatus status = kOK;  
    
    kCheck(kStream_Init(file, type, allocator)); 

    obj->streamPosition = 0; 
    obj->streamLength = 0; 
    obj->lastMode = kFILE_MODE_NULL; 

    kFile_InitPlatformFields(file); 

    kTry
    {         
        kTest(kApiLib_FileHandlers_()->open(file, path, mode)); 

        obj->streamLength = kApiLib_FileHandlers_()->seek(file, 0, kSEEK_ORIGIN_END); 

        kTest(kApiLib_FileHandlers_()->seek(file, 0, kSEEK_ORIGIN_BEGIN) == 0); 
    }
    kCatch(&status) 
    {
        kFile_VRelease(file); 
        kEndCatch(status); 
    }
       
    return kOK;
}

kFx(kStatus) kFile_VRelease(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    //ignore errors
    kFile_Close(file); 

    kCheck(kObject_FreeMemRef(file, &obj->base.readBuffer)); 
    kCheck(kObject_FreeMemRef(file, &obj->base.writeBuffer));

    kCheck(kStream_VRelease(file)); 

    return kOK; 
}

kFx(kStatus) kFile_Close(kFile file)
{
    kStatus flushStatus = kOK; 
    kStatus closeStatus = kOK; 

    if (kFile_IsOpen(file))
    {
        flushStatus = kStream_Flush(file); 
        closeStatus = kApiLib_FileHandlers_()->close(file); 
    }

    return kIsError(flushStatus) ? flushStatus : (kIsError(closeStatus) ? closeStatus : kOK); 
}

kFx(kStatus) kFile_SetWriteBuffer(kFile file, kSize size)
{
    kFileClass *obj = kFile_Cast_(file); 

    kCheckState(kFile_IsOpen(file)); 

    kCheck(kStream_Flush_(file)); 

    kCheck(kObject_FreeMemRef(file, &obj->base.writeBuffer)); 

    obj->base.writeCapacity = 0; 
    obj->base.writeBegin = 0; 
    obj->base.writeEnd = 0; 

    if (size > 0)
    {
        kCheck(kObject_GetMem(file, size, &obj->base.writeBuffer)); 
        obj->base.writeCapacity = size; 
    }

    return kOK;
}

kFx(kStatus) kFile_SetReadBuffer(kFile file, kSize size)
{
    kFileClass *obj = kFile_Cast_(file); 

    kCheckState(kFile_IsOpen(file)); 

    kCheck(kStream_Flush_(file)); 

    kCheck(kObject_FreeMemRef(file, &obj->base.readBuffer)); 

    obj->base.readCapacity = 0; 
    obj->base.readBegin = 0; 
    obj->base.readEnd = 0; 

    if (size > 0)
    {
        kCheck(kObject_GetMem(file, size, &obj->base.readBuffer));                 
        obj->base.readCapacity = size; 
    }
    
    return kOK;
}

kFx(kStatus) kFile_VReadSomeImpl(kFile file, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kFileClass *obj = kFile_Cast_(file); 
    kByte* dest = buffer;  
    kSize readCount = 0; 
    kSize copyCount, mediumCount; 

    kCheckState(kFile_IsOpen(file)); 
    kCheckState((kFile_Position(file) + minCount) <= kFile_Length(file)); 

    //configure the stream for reading, if necessary
    if (obj->lastMode != kFILE_MODE_READ)
    {
        kCheck(kStream_Flush_(file)); 

        obj->lastMode = kFILE_MODE_READ; 
    }
    
    //consume any bytes in the read buffer first
    if ((obj->base.readEnd - obj->base.readBegin) > 0)
    {
        copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin);

        kMemCopy(dest, &obj->base.readBuffer[obj->base.readBegin], copyCount);     
        
        obj->base.readBegin += copyCount; 
        readCount += copyCount; 
    }

    //if the request is not yet satisfied
    if (readCount < minCount)
    {
        //if the request is larger than the internal read buffer, read directly into the caller's buffer; else read into the internal buffer
        if ((maxCount - readCount) >= obj->base.readCapacity)
        {            
            kCheck(kFile_ReadAtLeast(file, &dest[readCount], minCount-readCount, maxCount-readCount, &mediumCount));

            readCount += mediumCount; 
        }
        else
        {
            kCheck(kFile_ReadAtLeast(file, &obj->base.readBuffer[0], minCount - readCount, obj->base.readCapacity, &mediumCount));    

            copyCount = kMin_(mediumCount, maxCount - readCount); 

            kCheck(kMemCopy(&dest[readCount], &obj->base.readBuffer[0], copyCount)); 

            obj->base.readBegin = copyCount; 
            obj->base.readEnd = mediumCount; 
            readCount += copyCount; 
        }
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = readCount; 
    }

    return kOK; 
}

kFx(kStatus) kFile_VWriteImpl(kFile file, const void* buffer, kSize size)
{
    kFileClass *obj = kFile_Cast_(file); 

    kCheckState(kFile_IsOpen(file)); 

    //configure the stream for writing, if necessary
    if (obj->lastMode != kFILE_MODE_WRITE)
    {
        kCheck(kStream_Flush_(file)); 

        obj->lastMode = kFILE_MODE_WRITE; 
        obj->base.writeEnd = obj->base.writeCapacity; 
    }

    //if the internal write buffer already has content, but not enough free space, flush
    if ((obj->base.writeBegin > 0) && ((obj->base.writeEnd - obj->base.writeBegin) < size))
    {
        kCheck(kStream_Flush_(file)); 
    }

    //if the write is larger than the internal write buffer, write directly to the medium; else write to the internal buffer
    if ((obj->base.writeEnd - obj->base.writeBegin) < size)
    {
        kCheck(kFile_WriteAll(file, buffer, size)); 
    }
    else
    {
        kCheck(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size)); 
        obj->base.writeBegin += size; 
    }

    return kOK; 
}

kFx(kStatus) kFile_VFlush(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 
        
    kCheckState(kFile_IsOpen(file)); 

    if ((obj->lastMode == kFILE_MODE_WRITE) && (obj->base.writeBegin > 0))
    {
        kCheck(kFile_WriteAll(file, obj->base.writeBuffer, obj->base.writeBegin));
    }
    else if ((obj->lastMode == kFILE_MODE_READ) && ((obj->base.readEnd - obj->base.readBegin) > 0))
    {
        obj->base.bytesRead -= (obj->base.readEnd - obj->base.readBegin); 
    }

    obj->base.readBegin = obj->base.readEnd = 0; 
    obj->base.writeBegin = obj->base.writeEnd = 0; 
            
    return kOK;
}

kFx(kStatus) kFile_VSeek(kFile file, k64s offset, kSeekOrigin origin)
{
    kFileClass *obj = kFile_Cast_(file); 
      
    kCheckState(kFile_IsOpen(file)); 

    kCheck(kStream_Flush_(file)); 
    
    obj->streamPosition = kApiLib_FileHandlers_()->seek(file, offset, origin); 

    return kOK; 
}

kFx(k64u) kFile_Length(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    kAssert(kFile_IsOpen(file)); 

    if (obj->lastMode != kFILE_MODE_WRITE)
    {
        return obj->streamLength; 
    }
    else
    {
        return kMax_(obj->streamLength, obj->streamPosition + obj->base.writeBegin); 
    }
}

kFx(k64u) kFile_Position(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    kAssert(kFile_IsOpen(file)); 

    if (obj->lastMode == kFILE_MODE_READ)
    {
        return obj->streamPosition - (obj->base.readEnd - obj->base.readBegin); 
    }
    else if (obj->lastMode == kFILE_MODE_WRITE)
    {
        return obj->streamPosition + obj->base.writeBegin; 
    }
    else
    {
        return obj->streamPosition; 
    }
}

kFx(kBool) kFile_Exists(const kChar* fileName)
{
    return kApiLib_FileHandlers_()->exists(fileName); 
}

kFx(k64u) kFile_Size(const kChar* fileName)
{
    return kApiLib_FileHandlers_()->size(fileName); 
}

kFx(kStatus) kFile_Copy(const kChar* source, const kChar* destination)
{
    return kApiLib_FileHandlers_()->copy(source, destination, kNULL, kNULL); 
} 

kFx(kStatus) kFile_CopyEx(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{    
    return kApiLib_FileHandlers_()->copy(source, destination, progress, context); 
}

kFx(kStatus) kFile_Move(const kChar* source, const kChar* destination)
{
    return kApiLib_FileHandlers_()->move(source, destination, kNULL, kNULL); 
}

kFx(kStatus) kFile_MoveEx(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{    
    return kApiLib_FileHandlers_()->move(source, destination, progress, context); 
}

kFx(kStatus) kFile_Delete(const kChar* path)
{
    return kApiLib_FileHandlers_()->del(path); 
}

kFx(kStatus) kFile_TempName(kChar* name, kSize capacity)
{
    return kApiLib_FileHandlers_()->tempName(name, capacity); 
}

kFx(kStatus) kFile_ReadAtLeast(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kFileClass *obj = kFile_Cast_(file); 

    kCheck(kApiLib_FileHandlers_()->read(file, buffer, minCount, maxCount, bytesRead)); 

    obj->streamPosition += *bytesRead;  
    obj->base.bytesRead += *bytesRead; 

    return kOK; 
}

kFx(kStatus) kFile_WriteAll(kFile file, const kByte* buffer, kSize count)
{
    kFileClass *obj = kFile_Cast_(file); 

    kCheck(kApiLib_FileHandlers_()->write(file, buffer, count)); 

    obj->streamPosition += count; 
    obj->streamLength = kMax_(obj->streamLength, obj->streamPosition); 
    obj->base.bytesWritten += count; 
    
    return kOK;    
}

kFx(kBool) kFile_ExistsImpl(const kChar* path)
{
    kFile file = kNULL; 
    kBool exists = kFALSE; 

    if (kSuccess(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)))
    {
        exists = kTRUE; 
        kObject_Destroy(file); 
    }

    return exists; 
}

kFx(k64u) kFile_SizeImpl(const kChar* path)
{
    kFile file = kNULL; 
    k64u size = 0; 

    if (kSuccess(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)))
    {
        size = kFile_Length(file); 
        kObject_Destroy(file); 
    }

    return size; 
}

kFx(kStatus) kFile_CopyImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{    
    kFile from = kNULL;
    kFile to = kNULL;
    kArray1 buffer = kNULL; 
    k64u size = 0; 
    k64u remaining = 0;
    kStatus status = kOK; 
    k64u updateTime = k64U_NULL; 
    
    if (kStrEquals(source, destination))
    {
        kUpdateProgress(progress, context, kNULL, kNULL, 100); 
        return kOK; 
    }

    kTry
    {
        kTest(kFile_Construct(&from, source, kFILE_MODE_READ, kNULL)); 
        kTest(kFile_Construct(&to, destination, kFILE_MODE_WRITE, kNULL));
        kTest(kArray1_Construct(&buffer, kTypeOf(kByte), kFILE_COPY_BUFFER_SIZE, kNULL)); 

        remaining = size = kFile_Length(from);

        while (remaining > 0)
        {
            kSize segment = (kSize) kMin_(remaining, kFILE_COPY_BUFFER_SIZE); 

            kTest(kStream_Read(from, kArray1_Data_(buffer), segment));
            kTest(kStream_Write(to, kArray1_Data_(buffer), segment));

            remaining -= segment;

            kUpdateProgress(progress, context, kNULL, &updateTime, (k32u)(100*(size-remaining)/size)); 
        }           

        kTest(kFile_Close(from)); 
        kTest(kFile_Close(to)); 

        kUpdateProgress(progress, context, kNULL, kNULL, 100); 
    } 
    kCatchEx(&status)
    {
        //eliminate any partially-copied remnant (not strictly required, but convenient)
        if (!kIsNull(to))
        {
            kDestroyRef(&to);
            kFile_Delete(destination); 
        }

        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        kCheck(kDestroyRef(&from));
        kCheck(kDestroyRef(&to));
        kCheck(kObject_Destroy(buffer));

        kEndFinallyEx(); 
    }

    return kOK;
}

kFx(kStatus) kFile_MoveImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{
    kCheck(kApiLib_FileHandlers_()->copy(source, destination, progress, context)); 
    kCheck(kApiLib_FileHandlers_()->del(source)); 

    return kOK;
}

kFx(kStatus) kFile_TempNameImpl(kChar* name, kSize capacity)
{
    return kStrPrintf(name, capacity, "%08X%08X%08X%08X", kRandom32u(),kRandom32u(), kRandom32u(), kRandom32u()); 
}

#if defined(K_WINDOWS)

kFx(void) kFile_InitPlatformFields(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    obj->handle = INVALID_HANDLE_VALUE; 
}

kFx(kBool) kFile_IsOpen(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    return obj->handle != INVALID_HANDLE_VALUE; 
}

kFx(kStatus) kFile_OpenImpl(kFile file, const kChar* path, kFileMode mode)
{
    kFileClass *obj = kFile_Cast_(file); 
    DWORD desiredAccess = 0; 
    DWORD shareMode = 0; 
    DWORD creationDisposition = 0; 
    DWORD flags = FILE_ATTRIBUTE_NORMAL;  
    kChar nativePath[kPATH_MAX]; 
    WCHAR wpath[MAX_PATH]; 

    kCheck(kPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    if (MultiByteToWideChar(CP_UTF8, 0, nativePath, -1, wpath, kCountOf(wpath)) == 0)
    {
        return kERROR_PARAMETER; 
    }

    if (mode == kFILE_MODE_READ)
    {
        desiredAccess = GENERIC_READ;        
        shareMode = FILE_SHARE_READ; 
        creationDisposition = OPEN_EXISTING;         
    }
    else if (mode == kFILE_MODE_WRITE)
    {
        desiredAccess = GENERIC_WRITE;        
        creationDisposition = CREATE_ALWAYS;         
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE))
    {
        desiredAccess = GENERIC_READ | GENERIC_WRITE;        
        creationDisposition = CREATE_ALWAYS;         
    }
    else if (mode == (kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        desiredAccess = GENERIC_WRITE;        
        creationDisposition = OPEN_ALWAYS;         
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        desiredAccess = GENERIC_READ | GENERIC_WRITE;        
        creationDisposition = OPEN_ALWAYS;         
    }
    else
    {
        return kERROR_PARAMETER; 
    }
  
    if ((obj->handle = CreateFile(wpath, desiredAccess, shareMode, kNULL, creationDisposition, flags, kNULL)) == INVALID_HANDLE_VALUE)
    {
        switch (GetLastError())
        {
        case ERROR_FILE_NOT_FOUND:
        case ERROR_PATH_NOT_FOUND:
            return kERROR_NOT_FOUND;
        default:
            return kERROR_STATE; 
        }
    }

    return kOK; 
}

kFx(kStatus) kFile_CloseImpl(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 
    BOOL result = TRUE; 
    
    if (kFile_IsOpen(file))
    {
        result = CloseHandle(obj->handle); 
        obj->handle = INVALID_HANDLE_VALUE; 
    }
    
    return (result) ? kOK : kERROR_STREAM; 
}

kFx(kStatus) kFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kFileClass *obj = kFile_Cast_(file); 
    DWORD read; 

    kCheckArgs(maxCount <= kFILE_MAX_IO_SIZE); 

    kCheck(ReadFile(obj->handle, buffer, (DWORD)maxCount, &read, kNULL)); 

    kCheck(read >= minCount); 

    *bytesRead = read;    

    return kOK; 
}

kFx(kStatus) kFile_WriteImpl(kFile file, const kByte* buffer, kSize count)
{
    kFileClass *obj = kFile_Cast_(file); 
    DWORD written; 

    kCheckArgs(count <= kFILE_MAX_IO_SIZE); 

    kCheck(WriteFile(obj->handle, buffer, (DWORD)count, &written, kNULL));

    kCheck(written == count); 

    return kOK; 
}

kFx(kStatus) kFile_FlushImpl(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    if (obj->lastMode == kFILE_MODE_WRITE)
    {
        kCheck(FlushFileBuffers(obj->handle)); 
    }

    return kOK; 
}

kFx(k64u) kFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin)
{
    kFileClass *obj = kFile_Cast_(file); 
    DWORD moveMethod; 
    LARGE_INTEGER offsetLarge; 
    LARGE_INTEGER positionLarge; 

    switch(origin)
    {
    case kSEEK_ORIGIN_BEGIN:        moveMethod = FILE_BEGIN;    break;
    case kSEEK_ORIGIN_CURRENT:      moveMethod = FILE_CURRENT;  break;
    case kSEEK_ORIGIN_END:          moveMethod = FILE_END;      break;
    default:                                                    return 0; 
    }

    offsetLarge.QuadPart = offset; 

    kCheck(SetFilePointerEx(obj->handle, offsetLarge, &positionLarge, moveMethod)); 

    return (k64u) positionLarge.QuadPart; 
}

kFx(kStatus) kFile_DeleteImpl(const kChar* path)
{
    kChar nativePath[kPATH_MAX]; 
    WCHAR wpath[MAX_PATH]; 

    kCheck(kPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    if (MultiByteToWideChar(CP_UTF8, 0, nativePath, -1, wpath, kCountOf(wpath)) == 0)
    {
        return kERROR_PARAMETER; 
    }

    return DeleteFile(wpath) ? kOK : kERROR_OS;
}

#elif defined(K_DARWIN) || defined (K_LINUX)

kFx(void) kFile_InitPlatformFields(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 
    obj->handle = -1; 
}

kFx(kBool) kFile_IsOpen(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 
    return obj->handle != -1; 
}

kFx(kStatus) kFile_OpenImpl(kFile file, const kChar* path, kFileMode mode)
{
    kFileClass *obj = kFile_Cast_(file); 
    kChar nativePath[kPATH_MAX]; 
    int oflag = 0; 

    kCheck(kPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    if (mode == kFILE_MODE_READ)
    {
        oflag = O_RDONLY;        
    }
    else if (mode == kFILE_MODE_WRITE)
    {
        oflag = O_WRONLY | O_CREAT | O_TRUNC;     
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE))
    {
        oflag = O_RDWR | O_CREAT | O_TRUNC; 
    }
    else if (mode == (kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        oflag = O_WRONLY | O_CREAT;     
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        oflag = O_RDWR | O_CREAT; 
    }
    else
    {
        return kERROR_PARAMETER; 
    }
      
    if ((obj->handle = open(nativePath, oflag)) == -1)
    {
        switch (errno)
        {
        case ENOENT:
        case ENOTDIR:
            return kERROR_NOT_FOUND;
        default:
            return kERROR_STATE; 
        }
    }

    if (chmod(path, S_IRWXU) == -1)
    {
        return kERROR_STATE;
    }
    
    return kOK; 
}

kFx(kStatus) kFile_CloseImpl(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 
    int result = 0; 
    
    if (kFile_IsOpen(file))
    {
        result = close(obj->handle); 
        obj->handle = -1; 
    }

    return (result != -1) ? kOK : kERROR_STREAM; 
}

kFx(kStatus) kFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kFileClass *obj = kFile_Cast_(file); 
    ssize_t readSize; 

    if ((readSize = read(obj->handle, buffer, maxCount)) < 0)
    {
        return kERROR_STREAM; 
    } 

    if ((kSize)readSize < minCount)
    {
        return kERROR_STREAM; 
    }

    *bytesRead = (kSize) readSize; 

    return kOK; 
}

kFx(kStatus) kFile_WriteImpl(kFile file, const kByte* buffer, kSize count)
{
    kFileClass *obj = kFile_Cast_(file); 
    ssize_t writeSize; 

    if ((writeSize = write(obj->handle, buffer, count)) < 0)
    {
        return kERROR_STREAM; 
    } 

    if ((kSize)writeSize != count)
    {
        return kERROR_STREAM; 
    }

    return kOK; 
}

kFx(kStatus) kFile_FlushImpl(kFile file)
{
    return kOK; 
}

kFx(k64u) kFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin)
{
    kFileClass *obj = kFile_Cast_(file); 
    off_t position; 
    int whence; 

    switch(origin)
    {
    case kSEEK_ORIGIN_BEGIN:        whence = SEEK_SET;    break;
    case kSEEK_ORIGIN_CURRENT:      whence = SEEK_CUR;    break;
    case kSEEK_ORIGIN_END:          whence = SEEK_END;    break;
    default:                                              return 0; 
    }

    if ((position = lseek(obj->handle, (off_t)offset, whence)) == -1)
    {
        return 0; 
    }

    return (k64u) position; 
}

kFx(kStatus) kFile_DeleteImpl(const kChar* path)
{
    kChar nativePath[kPATH_MAX]; 

    kCheck(kPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    return (unlink(nativePath) == 0) ? kOK : kERROR_OS; 
}

#else 

kFx(void) kFile_InitPlatformFields(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 
    obj->handle = kNULL; 
}

kFx(kBool) kFile_IsOpen(kFile file)
{
    kFileClass *obj = kFile_Cast_(file); 

    return !kIsNull(obj->handle); 
}

kFx(kStatus) kFile_OpenImpl(kFile file, const kChar* path, kFileMode mode)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kFile_CloseImpl(kFile file)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kFile_WriteImpl(kFile file, const kByte* buffer, kSize count)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kFile_FlushImpl(kFile file)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(k64u) kFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin)
{
    kAssert(kFALSE); 
    return 0;
}

kFx(kStatus) kFile_DeleteImpl(const kChar* path)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

#endif
