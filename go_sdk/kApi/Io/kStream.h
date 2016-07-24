/** 
 * @file    kStream.h
 * @brief   Declares the kStream class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STREAM_H
#define K_API_STREAM_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kStream
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents an I/O stream. 
 */
//typedef kObject kStream;   --forward-declared in kApiDef.x.h

/** 
 * Reads the specified number of bytes from the stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Destination for bytes that are read.
 * @param   size        Count of bytes to read.
 * @return              Operation status. 
 */
kFx(kStatus) kStream_Read(kStream stream, void* buffer, kSize size); 

/** 
 * Reads up to the specified number of bytes from the stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Destination for bytes that are read.
 * @param   minCount    Minimum count of bytes to read.
 * @param   maxCount    Maximum count of bytes to read.
 * @param   bytesRead   Receives count of bytes read.
 * @return              Operation status. 
 */
kFx(kStatus) kStream_ReadSome(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead); 

/** 
 * Writes the specified number of bytes to the stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Bytes to be written to the stream.
 * @param   size        Count of bytes to be written.
 * @return              Operation status. 
 */
kFx(kStatus) kStream_Write(kStream stream, const void* buffer, kSize size); 

/** 
 * Copies the specified number of bytes from one stream to another. 
 *
 * @public              @memberof kStream
 * @param   stream      Destination stream. 
 * @param   source      Source stream. 
 * @param   size        Count of bytes to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kStream_Copy(kStream stream, kStream source, kSize size); 

/** 
 * Copies the specified number of bytes from one stream to another, with progress feedback.
 *
 * The specified callback will be invoked to provide feedback on the progress of the operation. The callback 'args' 
 * parameter will receive a k32u value representing the percentage completed. The callback is guaranteed to be 
 * called at least once if the operation is successful, with a progress value of 100%. 
 *
 * @public              @memberof kStream
 * @param   stream      Destination stream. 
 * @param   source      Source stream. 
 * @param   size        Count of bytes to be copied.
 * @param   progress    Optional progress callback (can be kNULL). 
 * @param   context     Callback context.
 * @return              Operation status. 
 */
kFx(kStatus) kStream_CopyEx(kStream stream, kStream source, kSize size, kCallbackFx progress, kPointer context); 

/** 
 * Moves the read/write pointer to the specified location, if supported by the underlying stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   offset      Offset by which to adjust the read/write pointer.
 * @param   origin      Origin to which movement is relative (i.e. begin, current, end). 
 * @return              Operation status. 
 */
kFx(kStatus) kStream_Seek(kStream stream, k64s offset, kSeekOrigin origin); 

/** 
 * Flushes buffered writes to the underlying medium.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status. 
 */
kFx(kStatus) kStream_Flush(kStream stream); 

/** 
 * Reports the number of bytes read from this stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Count of bytes read.
 */
kFx(k64u) kStream_BytesRead(kStream stream); 

/** 
 * Reports the number of bytes written to this stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Count of bytes written.
 */
kFx(k64u) kStream_BytesWritten(kStream stream); 

/** 
 * Clears stream statistics (e.g. BytesRead, BytesWritten). 
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation Status
 */
kFx(kStatus) kStream_ClearStats(kStream stream); 

#define kStream_BytesRead_(STREAM)                  kxStream_BytesRead_(STREAM)         ///< Macro version of kStream_BytesRead.   
#define kStream_BytesWritten_(STREAM)               kxStream_BytesWritten_(STREAM)      ///< Macro version of kStream_BytesWritten.

kEndHeader()

#include <kApi/Io/kStream.x.h>

#endif
