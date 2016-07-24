/** 
 * @file    kPoolAlloc.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_POOL_ALLOC_X_H
#define K_API_POOL_ALLOC_X_H

kBeginHeader()

#define kPOOL_ALLOC_DEFAULT_BLOCK_SIZE                  (1 << 14)               ///< Default value for BlockSize property, in bytes.
#define kPOOL_ALLOC_DEFAULT_MAX_BLOCK_BUFFER_SIZE       (0)                     ///< Default value for MaxBlockBufferSize property, in bytes.
#define kPOOL_ALLOC_DEFAULT_BLOCK_CAPACITY              (kSIZE_MAX)             ///< Default value for BlockCapacity property, in bytes.
#define kPOOL_ALLOC_DEFAULT_BLOCK_REUSE_ENABLED         (kTRUE)                 ///< Default value for BlockReuseEnabled property.
#define kPOOL_ALLOC_DEFAULT_MAX_CACHED_BUFFER_SIZE      (0)                     ///< Default value for MaxCachedBufferSize property, in bytes.
#define kPOOL_ALLOC_DEFAULT_CACHE_CAPACITY              (kSIZE_MAX)             ///< Default value for CacheCapacity property, in bytes.
#define kPOOL_ALLOC_DEFAULT_TOTAL_CAPACITY              (kSIZE_MAX)             ///< Default value for TotalCapacity property, in bytes.

#define kPOOL_ALLOC_RANK_CAPACITY                       (32)                    ///< kPoolAlloc can optimize 32-bit or smaller allocations (limited by use of kMath_Log2Ceil32u to compute rank).
#define kPOOL_ALLOC_MAX_MANAGED_BUFFER_SIZE             (0x80000000)            ///< Based on kPOOL_ALLOC_RANK_CAPACITY, hard interal limit on optimized buffer size.

#define kPOOL_ALLOC_MIN_HEADERS_PER_BLOCK               (8)                     ///< Used in block allocation to ensure at least N buffer headers will fit (helps reduce fragementation).

/**
 * @internal
 * @struct  kPoolAllocAllocType
 * @ingroup kApi-Utils
 * @brief   Represents a kPoolAlloc allocation type GUID.
 * 
 * Allocation type identifiers are dynamically-generated GUIDs (use of GUIDs improves ability to detect 
 * invalid deallocations). 
 */
typedef kSize kPoolAllocAllocType; 

/**
 * @internal
 * @struct  kPoolAllocBlockHeader
 * @ingroup kApi-Utils
 * @brief   Header for a kPoolAlloc buffer allocation block. 
 */
typedef struct kPoolAllocBlockHeader
{
    kSize allocCount;                               ///< Count of allocations made from this block.
    kSize refCount;                                 ///< Count of allocations from this block that are currently in use (not free).
    kSize rank;                                     ///< Base-2 logarithm of block buffer size (index into kPoolAlloc rank array; kSIZE_NULL if block unassigned).
    struct kPoolAllocBlockHeader* next;             ///< Pointer to next block (used in kPoolAllocRank.newBlockList, kPoolAllocRank.freeBlockList, kPoolAlloc.unassignedBlockList).
    struct kPoolAllocBlockHeader* previous;         ///< Pointer to previous block (used in kPoolAllocRank.newBlockList, kPoolAllocRank.freeBlockList, kPoolAlloc.unassignedBlockList).
    kSize unused;                                   ///< Structure allocation padding. 
} kPoolAllocBlockHeader; 

/**
 * @internal
 * @struct  kPoolAllocBufferHeader
 * @ingroup kApi-Utils
 * @brief   Header for a kPoolAlloc memory buffer. 
 */
typedef struct kPoolAllocBufferHeader
{
    kPoolAllocAllocType type;                       ///< Type of kPoolAlloc allocation. 
    union
    {
        kPoolAllocBlockHeader* block;               ///< For block-based allocations, the block from which this allocation was created.
        kSize rank;                                 ///< For potentially-cached allocations, the rank associated with the allocation.
        kSize size;                                 ///< For unmanaged allocations, the raw size of the allocation (excluding header).         
    } info;                                         ///< Allocation-type-specific information.
    struct kPoolAllocBufferHeader* next;            ///< Pointer to next buffer (used in kPoolAllocRank.freeBufferList).
    struct kPoolAllocBufferHeader* previous;        ///< Pointer to previous buffer (used in kPoolAllocRank.freeBufferList).
} kPoolAllocBufferHeader; 

/**
 * @internal
 * @struct  kPoolAllocBlockList
 * @ingroup kApi-Utils
 * @brief   List of kPoolAlloc memory blocks.
 */
typedef struct kPoolAllocBlockList
{
    kPoolAllocBlockHeader* first;                   ///< Head of block list. 
    kPoolAllocBlockHeader* last;                    ///< Tail of block list. 
    kSize count;                                    ///< Count of items in block list. 
} kPoolAllocBlockList; 

/**
 * @internal
 * @struct  kPoolAllocBufferList
 * @ingroup kApi-Utils
 * @brief   List of kPoolAlloc memory buffers.
 */
typedef struct kPoolAllocBufferList
{
    kPoolAllocBufferHeader* first;                  ///< Head of buffer list. 
    kPoolAllocBufferHeader* last;                   ///< Tail of buffer list. 
    kSize count;                                    ///< Count of items in buffer list. 
} kPoolAllocBufferList; 

/**
 * @internal
 * @struct  kPoolAllocRank
 * @ingroup kApi-Utils
 * @brief   Represents kPoolAlloc information related to a specific memory rank.
 */
typedef struct kPoolAllocRank
{
    kSize rank;                                     ///< Rank (base-2 logarithm of buffer size).  
    kSize size;                                     ///< Size of buffer associate with this rank. 
    kSize bufferStride;                             ///< Memory footprint of one block-based buffer, including buffer header and alignment padding.
    kBool isBlockBased;                             ///< Use blocks to allocate buffers?
    kBool isCached;                                 ///< Cache deallocated buffers for later use?
    kSize buffersPerBlock;                          ///< Count of buffers that can be allocated per block.
    kSize bufferCount;                              ///< Count of buffers in use for this rank (allocated or cached). 
    kSize minBufferCount;                           ///< Count of buffers reserved for this rank.
    kPoolAllocBufferList freeBufferList;            ///< List of free buffers. 
    kPoolAllocBlockList newBlockList;               ///< List of blocks whose allocCount is less than buffersPerBlock.
    kPoolAllocBlockList freeBlockList;              ///< List of blocks with a reference count of zero (not currently in use).
} kPoolAllocRank; 

typedef struct kPoolAllocClass
{
    kAllocClass base;     

    kLock lock;                                             //Provides mutual exclusion.

    kSize nominalBlockSize;                                 //Nominal block size for block-based allocations (exluding header), in bytes.
    kSize maxBlockBufferSize;                               //Maximum buffer size for an individual block-based allocation, in bytes.
    kSize blockCapacity;                                    //Maximum memory used to allocate blocks, in bytes.
    kBool blockReuseEnabled;                                //Can blocks be automatically reassigned between ranks as needed?
    kSize maxCachedBufferSize;                              //Maximum buffer size for an individual cached allocation, in bytes.
    kSize cacheCapacity;                                    //Maximum amount of non-block memory to retain upon deallocation, in bytes.
    kSize totalCapacity;                                    //Maximum amount of memory that can be requested from underlying allocator.

    kSize blockBasedRankCount;                              //Count of ranks potentially eligible for block-based allocation.
    kSize cacheBasedRankCount;                              //Count of ranks potentially eligible for cache-based allocation.
    kSize managedRankCount;                                 //Maximum of blockBasedRankCount and cacheBasedRankCount. 
    kSize maxManagedBufferSize;                             //Maximum size of managed buffer.

    kSize blockAllocId;                                     //GUID for block allocations from this allocator. 
    kSize cacheAllocId;                                     //GUID for cached allocations from this allocator. 
    kSize unmanagedAllocId;                                 //GUID for unmanaged allocations from this allocator. 

    kBool isStarted;                                        //Has pool-alloc been started?

    kPoolAllocRank rankInfo[kPOOL_ALLOC_RANK_CAPACITY];     //Information associated with each rank. 

    kPoolAllocBlockList unassignedBlockList;                //List of blocks blocks not yet assigned to any rank.

    kSize blockCount;                                       //Total number of blocks currently allocated from underlying allocator.
    kSize minBlockCount;                                    //Minimum number of blocks that should be allocated from underlying allocator (reservation).
    kSize cacheSize;                                        //Total size of free, cached (non-block) allocations, excluding headers, in bytes.
    kSize totalSize;                                        //Total amount of memory currently drawn from underlying allocator.
   
} kPoolAllocClass; 

kDeclareClass(k, kPoolAlloc, kAlloc)
        
kFx(kStatus) kPoolAlloc_Init(kPoolAlloc object, kType type, kAlloc alloc);
kFx(kStatus) kPoolAlloc_VRelease(kPoolAlloc object);

kFx(kStatus) kPoolAlloc_VGet(kPoolAlloc object, kSize size, void* mem); 
kFx(kStatus) kPoolAlloc_VFree(kPoolAlloc object, void* mem); 

kFx(kStatus) kPoolAlloc_ReserveBlocksAt(kPoolAlloc object, kSize rank, kSize size);
kFx(kStatus) kPoolAlloc_ReserveCachedBuffersAt(kPoolAlloc object, kSize rank, kSize size);

kFx(void) kPoolAlloc_RemoveFromBlockList(kPoolAlloc object, kPoolAllocBlockList* list, kPoolAllocBlockHeader* item); 
kFx(void) kPoolAlloc_InsertIntoBlockList(kPoolAlloc object, kPoolAllocBlockList* list, kPoolAllocBlockHeader* before, kPoolAllocBlockHeader* item); 

kFx(void) kPoolAlloc_RemoveFromBufferList(kPoolAlloc object, kPoolAllocBufferList* list, kPoolAllocBufferHeader* item); 
kFx(void) kPoolAlloc_InsertIntoBufferList(kPoolAlloc object, kPoolAllocBufferList* list, kPoolAllocBufferHeader* before, kPoolAllocBufferHeader* item); 

kFx(kStatus) kPoolAlloc_AddBlockRef(kPoolAlloc object, kPoolAllocBlockHeader* block); 
kFx(kStatus) kPoolAlloc_RemoveBlockRef(kPoolAlloc object, kPoolAllocBlockHeader* block); 

kFx(kStatus) kPoolAlloc_AllocBlock(kPoolAlloc object); 
kFx(kStatus) kPoolAlloc_AllocCachedBuffers(kPoolAlloc object, kSize rank, kSize count); 
kFx(kStatus) kPoolAlloc_AllocUnmanagedBuffer(kPoolAlloc object, kSize size, void* mem); 

kFx(kStatus) kPoolAlloc_PrepareFreeBlockBasedBuffer(kPoolAlloc object, kSize rank); 
kFx(kStatus) kPoolAlloc_RecycleBlocks(kPoolAlloc object, kSize maxCount); 
kFx(kStatus) kPoolAlloc_ReturnBlockToUnassigned(kPoolAlloc object, kPoolAllocBlockList* list, kPoolAllocBlockHeader* block);

kFx(kStatus) kPoolAlloc_ClearEx(kPoolAlloc object, kBool all);
kFx(kStatus) kPoolAlloc_FreeUnassignedBlocks(kPoolAlloc object); 
kFx(kStatus) kPoolAlloc_FreeCachedBuffers(kPoolAlloc object); 

#define kPoolAlloc_Cast_(P)                 kCastClass_(kPoolAlloc, P)
#define kPoolAlloc_(P)                      kCast_(kPoolAllocClass*, P)

/** Reports size of a single block, excluding block header, in bytes. */
#define kPoolAlloc_BlockContentSize_(P)     (kPoolAlloc_(P)->nominalBlockSize + kPOOL_ALLOC_MIN_HEADERS_PER_BLOCK*sizeof(kPoolAllocBufferHeader) + kALIGN_ANY_SIZE)

/** Reports size of a single block, including block header, in bytes. */
#define kPoolAlloc_BlockSize_(P)            (sizeof(kPoolAllocBlockHeader) + kPoolAlloc_BlockContentSize_(P))

kEndHeader()

#endif
