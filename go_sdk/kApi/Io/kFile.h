/** 
 * @file    kFile.h
 * @brief   Declares the kFile class. 
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_FILE_H
#define K_API_FILE_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kFile
 * @extends kStream
 * @ingroup kApi-Io
 * @brief   Represents a file stream.
 */
//typedef kStream kFile;      --forward-declared in kApiDef.x.h 

/** 
 * Reads the specified file and provides the file contents in an array.
 *
 * @public              @memberof kFile
 * @param   path        Path to file.
 * @param   data        Receives a pointer to the file contents. 
 * @param   size        Receives the size of the file contents.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Load(const kChar* path, void* data, kSize* size, kAlloc allocator);

/** 
 * Reads the specified file into the provided array.
 *
 * @public              @memberof kFile
 * @param   path        Path to file.
 * @param   data        Receives file contents. 
 * @param   capacity    Amount of file to read, in bytes.
 * @return              Operation status (kERROR_INCOMPLETE if capacity is not equal to file size). 
 */
kFx(kStatus) kFile_LoadTo(const kChar* path, void* data, kSize capacity);

/** 
 * Saves the specified data to the specified file.
 *
 * @public              @memberof kFile
 * @param   path        Path to file.
 * @param   data        Pointer to the file contents. 
 * @param   size        Size of the file contents.
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Save(const kChar* path, const kByte* data, kSize size); 

/** 
 * Constructs a kFile object.
 *
 * @public              @memberof kFile
 * @param   file        Destination for the constructed object handle. 
 * @param   path        Path to the file.
 * @param   mode        Specifies how to open the file. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Construct(kFile* file, const kChar* path, kFileMode mode, kAlloc allocator); 

/** 
 * Performs any outstanding I/O operations and closes the underlying file. 
 * 
 * The purpose of the Close method is to provide an opportunity to finalize I/O and
 * report any errors before destroying a file object. If the Destroy method is called without 
 * having closed the file, the Destroy method will close the file and ignore any errors.
 * 
 * The Close method should only be called once per file object. After calling the Close method, 
 * any operations other than Destroy will produce an undefined result. 
 * 
 *
 * @public              @memberof kFile
 * @param   file        File object. 
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Close(kFile file); 

/** 
 * Sets the size of the buffer used for writing.
 *
 * Buffering can improve efficiency when performing several small write operations. 
 * The default buffer size is 0. 
 *
 * @public              @memberof kFile
 * @param   file        File object. 
 * @param   size        Size of the buffer. 
 * @return              Operation status. 
 */
kFx(kStatus) kFile_SetWriteBuffer(kFile file, kSize size); 

/** 
 * Sets the size of the buffer used for reading.
 *
 * Buffering can improve efficiency when performing several small read operations. 
 * The default buffer size is 0. 
 *
 * @public              @memberof kFile
 * @param   file        File object. 
 * @param   size        Size of the buffer. 
 * @return              Operation status. 
 */
kFx(kStatus) kFile_SetReadBuffer(kFile file, kSize size); 

/** 
 * Returns the current length of the file. 
 *
 * @public              @memberof kFile
 * @param   file        File object. 
 * @return              Length of the file, in bytes.
 */
kFx(k64u) kFile_Length(kFile file); 

/** 
 * Returns the current position of the read/write pointer, relative to the beginning of the file. 
 *
 * @public              @memberof kFile
 * @param   file        File object. 
 * @return              Offset of read/write pointer from beginning of file. 
 */
kFx(k64u) kFile_Position(kFile file); 

/** 
 * Reports whether the specified file exists. 
 *
 * @public              @memberof kFile
 * @param   fileName    Path to the file.
 * @return              kTRUE if the file exists; kFALSE otherwise.
 */
kFx(kBool) kFile_Exists(const kChar* fileName); 

/** 
 * Reports the size the specified file, in bytes. 
 *
 * @public              @memberof kFile
 * @param   fileName    Path to the file.
 * @return              Size of the file, in bytes.
 */
kFx(k64u) kFile_Size(const kChar* fileName); 

/** 
 * Copies a file to the specified destination.
 *
 * @public              @memberof kFile
 * @param   source      Source file path.
 * @param   destination Destination file path.
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Copy(const kChar* source, const kChar* destination); 

/** 
 * Copies a file to the specified destination with progress feedback.
 * 
 * The specified callback will be invoked to provide feedback on the progress of the operation. The callback 'args' 
 * parameter will receive a k32u value representing the percentage completed. The callback is guaranteed to be 
 * called at least once if the operation is successful, with a progress value of 100%. 
 *
 * @public              @memberof kFile
 * @param   source      Source file path.
 * @param   destination Destination file path.
 * @param   progress    Optional progress callback (can be kNULL). 
 * @param   context     Callback context.
 * @return              Operation status. 
 */
kFx(kStatus) kFile_CopyEx(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context); 

/** 
 * Moves a file to the specified destination.
 *
 * @public              @memberof kFile
 * @param   source      Source file path.
 * @param   destination Destination file path.
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Move(const kChar* source, const kChar* destination); 

/** 
 * Moves a file to the specified destination with progress feedback.
 * 
 * The specified callback will be invoked to provide feedback on the progress of the operation. The callback 'args' 
 * parameter will receive a k32u value representing the percentage completed. The callback is guaranteed to be 
 * called at least once if the operation is successful, with a progress value of 100%. 
 *
 * @public              @memberof kFile
 * @param   source      Source file path.
 * @param   destination Destination file path.
 * @param   progress    Optional progress callback (can be kNULL). 
 * @param   context     Callback context.
 * @return              Operation status. 
 */
kFx(kStatus) kFile_MoveEx(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context); 

/** 
 * Deletes the specified file.
 *
 * @public              @memberof kFile
 * @param   path        File path.
 * @return              Operation status. 
 */
kFx(kStatus) kFile_Delete(const kChar* path); 

/** 
 * Gets a temporary file name. 
 * 
 * @public              @memberof kFile
 * @param   name        Receives temporary file name.
 * @param   capacity    Maximum number of characters (including null terminator).
 * @return              Operation status. 
 */
kFx(kStatus) kFile_TempName(kChar* name, kSize capacity); 

kEndHeader()

#include <kApi/Io/kFile.x.h>

#endif
