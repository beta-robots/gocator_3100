/** 
 * @file    kMemory.h
 * @brief   Declares the kMemory class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MEMORY_H
#define K_API_MEMORY_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kMemory
 * @extends kStream
 * @ingroup kApi-Io
 * @brief   Represents an in-memory stream.
 */
//typedef kStream kMemory;            --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kMemory object.
 *
 * By default, an auto-sizing, internally-managed buffer is used. 
 *
 * @public              @memberof kMemory
 * @param   memory      Destination for the constructed object handle. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kMemory_Construct(kMemory* memory, kAlloc allocator);

/** 
 * Attaches the memory stream to an external, fixed-capacity buffer. 
 *
 * @public              @memberof kMemory
 * @param   memory      kMemory object.
 * @param   buffer      Data buffer.
 * @param   position    Current read/write position, relative to start of buffer.
 * @param   length      Length of valid data contained in the buffer, in bytes.
 * @param   capacity    Total size of the buffer, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kMemory_Attach(kMemory memory, void* buffer, kSize position, kSize length, kSize capacity); 

/** 
 * Allocates an auto-sizing, internally-managed buffer for the memory stream. 
 *
 * @public                  @memberof kMemory
 * @param   memory          kMemory object.
 * @param   initialCapacity Initial capacity of the buffer, in bytes.
 * @return                  Operation status. 
 */
kFx(kStatus) kMemory_Allocate(kMemory memory, kSize initialCapacity); 

/** 
 * Returns the current length of the memory buffer. 
 *
 * @public              @memberof kMemory
 * @param   memory      kMemory object.
 * @return              Length of the memory buffer. 
 */
kFx(k64u) kMemory_Length(kMemory memory); 

/** 
 * Returns the current position of the read/write pointer, relative to the beginning of the buffer. 
 *
 * @public              @memberof kMemory
 * @param   memory      kMemory object.
 * @return              Position of the write/write pointer.
 */
kFx(k64u) kMemory_Position(kMemory memory); 

/** 
 * Returns the current capacity of the memory buffer. 
 *
 * @public              @memberof kMemory
 * @param   memory      kMemory object.
 * @return              Capacity of the memory buffer. 
 */
kFx(kSize) kMemory_Capacity(kMemory memory); 

/** 
 * Sets the reported length of the memory buffer. 
 *
 * @public              @memberof kMemory
 * @param   memory      kMemory object.
 * @param   length      Buffer length
 * @return              Operation status. 
 */
kFx(kStatus) kMemory_SetLength(kMemory memory, kSize length);

/** 
 * Returns a pointer to the memory buffer at the specified position. 
 *
 * @public              @memberof kMemory
 * @param   memory      kMemory object.
 * @param   offset      Position of interest, in bytes.
 * @return              Operation status. 
 */
kFx(void*) kMemory_At(kMemory memory, kSize offset); 

kEndHeader()

#include <kApi/Io/kMemory.x.h>

#endif
