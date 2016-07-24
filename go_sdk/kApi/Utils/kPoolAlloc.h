/** 
 * @file    kPoolAlloc.h
 * @brief   Declares the kPoolAlloc class and related types. 
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_POOL_ALLOC_H
#define K_API_POOL_ALLOC_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kPoolAlloc
 * @ingroup kApi-Utils
 * @brief   Allocates small buffers from larger blocks and/or caches deallocated buffers for later reuse.
 * 
 * This memory allocator can be used to improve performance in some circumstances. It can reduce the number 
 * of individual requests to an underlying allocator by allocating multiple small memory buffers from larger 
 * blocks. It can also cache deallocated buffers for later reuse, reducing the frequency of allocation requests
 * made to the underlying allocator. 
 * 
 * For each memory request, a 'rank' is determined by calculating the base-2 logarithm of the requested size and then 
 * rounding up. The rank determines the true size of the buffer that will be allocated (requests are rounded up to 
 * the nearest power of two). Rank-based buffer management provides simple organization and fast reallocation; the 
 * cost is increased memory space. 
 * 
 * Parameters are provided to control which ranks should be allocated from larger blocks, which ranks should 
 * be cached upon deallocation, memory capacities, etc. Memory can be reserved using the kPoolAlloc_Reserve
 * and kPoolAlloc_ReserveAt functions, and/or dynamically allocated from the underlying allocator as needed. 
 * 
 * The operations provided in this class should be used in the following order: 
 * - Construct a kPoolAlloc instance.
 * - Perform configuration.
 * - Use the kPoolAlloc_Start method to prepare the allocator for use.
 * - Use the kPoolAlloc_Reserve or kPoolAlloc_ReserveAt methods to make pre-emptive allocations.
 * - Use the kAlloc_Get/kAlloc_Free methods to perform allocations/deallocations.
 * - Destroy the kPoolAlloc instance when no longer needed. 
 * 
 * All outstanding memory allocations must be freed before destroying the allocator.
 */
//typedef kAlloc kPoolAlloc;            --forward-declared in kApiDef.x.h  

/** 
 * Constructs a kPoolAlloc object.
 * 
 * The 'allocator' argument specifies the underlying memory allocator used by this kPoolAlloc 
 * instance to satisfy memory requests. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      Destination for the constructed object handle. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_Construct(kPoolAlloc* object, kAlloc allocator);

/** 
 * Sets the approximate size of large memory blocks used to satisfy small memory requests. 
 * 
 * Blocks are assigned to one rank at a time. Multiple small allocations are typically performed 
 * from each larger block. The number of allocations that can be performed from a single block 
 * depends on the rank.
 * 
 * A common block size is used for all ranks to allow free blocks to be transferred between ranks 
 * (if reuse is enabled). 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Block size, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_SetBlockSize(kPoolAlloc object, kSize size);

/** 
 * Returns the approximate size of large memory blocks used to satisfy small memory requests. 
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Block size, in bytes.
 */
kFx(kSize) kPoolAlloc_BlockSize(kPoolAlloc object);

/** 
 * Sets the size limit for memory requests that can be allocated from larger blocks.
 * 
 * This property must be smaller than the BlockSize property. By default, this property 
 * is zero (block-based allocation is disabled).
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Maximum block-based allocation size, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_SetMaxBlockBufferSize(kPoolAlloc object, kSize size);

/** 
 * Returns the size limit for memory requests that can be allocated from larger blocks.
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Maximum size for a block-based allocation, in bytes.
 */
kFx(kSize) kPoolAlloc_MaxBlockBufferSize(kPoolAlloc object);

/** 
 * Sets the maximum total amount of memory that can be used for block-based allocations.
 * 
 * Blocks are dynamically allocated when needed; this property controls the maximum amount
 * of memory that can be used for blocks. This property is kSIZE_MAX by default. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Block memory capacity, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_SetBlockCapacity(kPoolAlloc object, kSize size);

/** 
 * Returns the maximum amount of memory that can be used for block-based allocations.
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Block memory capacity, in bytes.
 */
kFx(kSize) kPoolAlloc_BlockCapacity(kPoolAlloc object);

/** 
 * Determines whether blocks can be reused between ranks.
 * 
 * When a memory request qualifies for block-based allocation, and no free buffers 
 * are available at the required rank, a new block must be provided. If block 
 * reuse is enabled, then free blocks from other ranks can be reassigned as needed. 
 * If block reuse is disabled, then blocks remain at the rank to which they are first 
 * assigned. 
 * 
 * Block reuse can increase allocation time slightly, due to the need to search 
 * through all ranks for a free block. Reuse is enabled by default.
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   enabled     kTRUE to enable reuse; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_EnableBlockReuse(kPoolAlloc object, kBool enabled);

/** 
 * Reports whether blocks can be reused between ranks.
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              kTRUE if block reuse is enabled; kFALSE otherwise.
 */
kFx(kBool) kPoolAlloc_BlockReuseEnabled(kPoolAlloc object);

/** 
 * Sets the size limit for memory requests that can be cached upon deallocation.
 * 
 * Small allocations are typically configured to be provided by block-based allocation, which 
 * automatically caches deallocated buffers for reuse. But larger (individually allocated) 
 * buffers can also be cached upon deallocation for later use. This property controls the 
 * maximum buffer size than can be cached upon deallocation. 
 * 
 * This property is zero by default (caching of individual allocations is disabled).
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Maximum cached allocation size, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_SetMaxCachedBufferSize(kPoolAlloc object, kSize size);

/** 
 * Returns the size limit for memory requests that can be cached upon deallocation.
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Maximum cached allocation size, in bytes.
 */
kFx(kSize) kPoolAlloc_MaxCachedBufferSize(kPoolAlloc object);

/** 
 * Sets the maximum total amount of memory that can be used to cache buffers upon deallocation.
 * 
 * This property does not affect block-based allocations, which can be limited using the 
 * kPoolAlloc_SetBlockCapacity function.
 * 
 * This property is kSIZE_MAX by default. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Cached memory capacity, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_SetCacheCapacity(kPoolAlloc object, kSize size);

/** 
 * Returns the maximum total amount of memory that can be used to cache buffers upon deallocation.
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Cached memory capacity, in bytes.
 */
kFx(kSize) kPoolAlloc_CacheCapacity(kPoolAlloc object);

/** 
 * Sets the total amount of memory that can be requested from the underlying allocator.
 * 
 * This property is kSIZE_MAX by default. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Amount of memory that can be requested from underlying allocator, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_SetTotalCapacity(kPoolAlloc object, kSize size);

/** 
 * Returns the total amount of memory that can be requested from the underlying allocator.
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Amount of memory that can be requested from underlying allocator, in bytes.
 */
kFx(kSize) kPoolAlloc_TotalCapacity(kPoolAlloc object);

/** 
 * Prepares the allocator for first use.
 * 
 * This function should be called after configuration properties have been provided, and before using
 * memory reservation or allocation functions.
 * 
 * Configuration functions cannot be used after calling kPoolAlloc_Start. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_Start(kPoolAlloc object); 

/** 
 * Specifies the minimum amount of memory that should be set aside for blocks. 
 * 
 * This function can be used to ensure that the specified amount of memory is set aside 
 * for block-based allocations. The amount of memory specified is inclusive of any 
 * existing block memory (e.g., individual rank reservations). 
 * 
 * This function cannot be used before calling kPoolAlloc_Start. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   size        Amount of memory to reserve, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_Reserve(kPoolAlloc object, kSize size);

/** 
 * Specifies the minimum amount of memory that should be set aside at a particular rank.
 * 
 * This function can be used to ensure that the specified amount of memory is set aside 
 * for block-based allocations at a specific rank.  
 * 
 * This function can also be used to pre-cache individual buffers at a specific rank. 
 * However, these buffers may later be deallocated if the CacheCapacity limit is reached. 
 * 
 * This function cannot be used before calling kPoolAlloc_Start. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   rank        Memory rank (base-2 logarithm of memory size). 
 * @param   size        Amount of memory to reserve, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_ReserveAt(kPoolAlloc object, kSize rank, kSize size);

/** 
 * Returns surplus memory to the underlying allocator. 
 * 
 * For block-based ranks, any unused blocks will be returned to the underlying allocator. 
 * Reservations made using kPoolAlloc_Reserve/kPoolAlloc_ReserveAt are honored; those
 * blocks will be preserved for later use.
 * 
 * For ranks configured to cache buffers upon deallocation, any unused buffers will be 
 * returned to the underlying allocator. Reservations made using kPoolAlloc_ReserveAt are 
 * honored; those buffers will be preserved for later use.
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_Clear(kPoolAlloc object);

/** 
 * Removes any existing memory reservations and returns surplus memory to the underlying allocator. 
 * 
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Operation status. 
 */
kFx(kStatus) kPoolAlloc_ClearAll(kPoolAlloc object);

/** 
 * Reports the total number of memory buffers at the given rank.
 * 
 * This function reports the total number of buffers, including buffers currently in use 
 * (allocated via kAlloc_Get and not yet freed) and buffers cached for later use. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @param   rank        Memory rank (base-2 logarithm of memory size). 
 * @return              Buffer count.
 */
kFx(kSize) kPoolAlloc_BufferCountAt(kPoolAlloc object, kSize rank);

/** 
 * Reports the current amount of memory drawn from the underlying allocator. 
 * 
 * Total does not include the kPoolAlloc object header; all other memory is included. 
 *
 * @public              @memberof kPoolAlloc
 * @param   object      kPoolAlloc object.
 * @return              Total memory, in bytes.
 */
kFx(kSize) kPoolAlloc_TotalSize(kPoolAlloc object);

kEndHeader()

#include <kApi/Utils/kPoolAlloc.x.h>

#endif
