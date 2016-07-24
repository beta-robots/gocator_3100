/** 
 * @file    kPoolAlloc.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.  All rights reserved.
 */
#include <kApi/Utils/kPoolAlloc.h>
#include <kApi/Data/kMath.h>
#include <kApi/Threads/kLock.h>

kBeginClass(k, kPoolAlloc, kAlloc)
    kAddVMethod(kPoolAlloc, kObject, VRelease)
    kAddVMethod(kPoolAlloc, kAlloc, VGet)
    kAddVMethod(kPoolAlloc, kAlloc, VFree)
kEndClass()

kFx(kStatus) kPoolAlloc_Construct(kPoolAlloc* object, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kAssert((sizeof(kPoolAllocBlockHeader) % kALIGN_ANY_SIZE) == 0); 
    kAssert((sizeof(kPoolAllocBufferHeader) % kALIGN_ANY_SIZE) == 0); 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kPoolAlloc), object)); 

    if (!kSuccess(status = kPoolAlloc_Init(*object, kTypeOf(kPoolAlloc), alloc)))
    {
        kAlloc_FreeRef(alloc, object); 
    }

    return status; 
} 

kFx(kStatus) kPoolAlloc_Init(kPoolAlloc object, kType type, kAlloc alloc)
{
    kPoolAllocClass* obj = object; 
    kStatus status = kOK; 

    kCheck(kAlloc_Init(object, type, alloc)); 

    kInitFields_(kPoolAlloc, object); 

    obj->nominalBlockSize = kPOOL_ALLOC_DEFAULT_BLOCK_SIZE; 
    obj->maxBlockBufferSize = kPOOL_ALLOC_DEFAULT_MAX_BLOCK_BUFFER_SIZE; 
    obj->blockCapacity = kPOOL_ALLOC_DEFAULT_BLOCK_CAPACITY; 
    obj->blockReuseEnabled = kPOOL_ALLOC_DEFAULT_BLOCK_REUSE_ENABLED; 
    obj->maxCachedBufferSize = kPOOL_ALLOC_DEFAULT_MAX_CACHED_BUFFER_SIZE; 
    obj->cacheCapacity = kPOOL_ALLOC_DEFAULT_CACHE_CAPACITY; 
    obj->totalCapacity = kPOOL_ALLOC_DEFAULT_TOTAL_CAPACITY; 

    kTry
    {
        kTest(kLock_Construct(&obj->lock, alloc)); 
    }
    kCatch(&status)
    {
        kPoolAlloc_VRelease(object); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_VRelease(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    if (obj->isStarted)
    {
        kCheck(kPoolAlloc_ClearAll(object)); 
    }

    kCheck(kObject_Destroy(obj->lock)); 

    kCheck(kAlloc_VRelease(object)); 

    return kOK; 
}

kFx(kStatus) kPoolAlloc_SetBlockSize(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->nominalBlockSize = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_BlockSize(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->nominalBlockSize; 
}

kFx(kStatus) kPoolAlloc_SetMaxBlockBufferSize(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->maxBlockBufferSize = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_MaxBlockBufferSize(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->maxBlockBufferSize; 
}

kFx(kStatus) kPoolAlloc_SetBlockCapacity(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->blockCapacity = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_BlockCapacity(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->blockCapacity; 
}

kFx(kStatus) kPoolAlloc_EnableBlockReuse(kPoolAlloc object, kBool enabled)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->blockReuseEnabled = enabled; 

    return kOK; 
}

kFx(kBool) kPoolAlloc_BlockReuseEnabled(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->blockReuseEnabled; 
}

kFx(kStatus) kPoolAlloc_SetMaxCachedBufferSize(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->maxCachedBufferSize = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_MaxCachedBufferSize(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->maxCachedBufferSize; 
}

kFx(kStatus) kPoolAlloc_SetCacheCapacity(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->cacheCapacity = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_CacheCapacity(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->cacheCapacity; 
}

kFx(kStatus) kPoolAlloc_SetTotalCapacity(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kCheckState(!obj->isStarted); 

    obj->totalCapacity = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_TotalCapacity(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
   
    return obj->totalCapacity; 
}

kFx(kStatus) kPoolAlloc_Start(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kSize hardMaxBufferSize = kPOOL_ALLOC_MAX_MANAGED_BUFFER_SIZE; 
    kSize i; 

    kCheckState(!obj->isStarted); 
    kCheckState(obj->maxBlockBufferSize <= obj->nominalBlockSize); 

    //create randomized GUIDs for allocation type idenifiers
    do 
    {
        obj->blockAllocId = kRandomSize(); 
        obj->cacheAllocId = kRandomSize(); 
        obj->unmanagedAllocId = kRandomSize(); 
    } 
    while ((obj->blockAllocId == obj->cacheAllocId) || (obj->cacheAllocId == obj->unmanagedAllocId)); 

    //pre-calculate various parameters
    if (obj->maxBlockBufferSize == 0)                       obj->blockBasedRankCount = 0; 
    else if (obj->maxBlockBufferSize >= hardMaxBufferSize)  obj->blockBasedRankCount = kPOOL_ALLOC_RANK_CAPACITY; 
    else                                                    obj->blockBasedRankCount = (kSize) kMath_Log2Ceil32u((k32u)obj->maxBlockBufferSize) + 1; 

    if (obj->maxCachedBufferSize == 0)                      obj->cacheBasedRankCount = 0; 
    else if (obj->maxCachedBufferSize >= hardMaxBufferSize) obj->cacheBasedRankCount = kPOOL_ALLOC_RANK_CAPACITY; 
    else                                                    obj->cacheBasedRankCount = (kSize) kMath_Log2Ceil32u((k32u)obj->maxCachedBufferSize) + 1; 

    obj->managedRankCount = kMax_(obj->blockBasedRankCount, obj->cacheBasedRankCount); 
    obj->maxManagedBufferSize = (obj->managedRankCount > 0) ? ((kSize)1 << (obj->managedRankCount-1)) : 0; 

    for (i = 0; i < kPOOL_ALLOC_RANK_CAPACITY; ++i)
    {
        kPoolAllocRank* rankInfo = &obj->rankInfo[i]; 

        rankInfo->rank = i; 
        rankInfo->size = (kSize)1 << i;

        rankInfo->bufferStride = sizeof(kPoolAllocBufferHeader) + rankInfo->size; 
        rankInfo->bufferStride = kAlign_(rankInfo->bufferStride, kALIGN_ANY); 

        if (i < obj->blockBasedRankCount)
        {
            rankInfo->isBlockBased = kTRUE;  
            rankInfo->buffersPerBlock = kPoolAlloc_BlockContentSize_(object) / rankInfo->bufferStride;  
        }
        else if (i < obj->cacheBasedRankCount)
        {
            rankInfo->isCached = kTRUE;  
        }
    }

    obj->isStarted = kTRUE; 

    return kOK; 
}

kFx(kStatus) kPoolAlloc_VGet(kPoolAlloc object, kSize size, void* mem)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kBool isAllocated = kFALSE; 

    kLock_Enter(obj->lock); 

    kTry
    {   
        if (size == 0)
        {
            *(void**)mem = kNULL; 
            isAllocated = kTRUE; 
        }
        else if (size <= obj->maxManagedBufferSize)
        {
            kSize rank = (kSize) kMath_Log2Ceil32u((k32u)size);       
            kPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

            if (rankInfo->isBlockBased)
            {
                if (rankInfo->freeBufferList.count == 0)
                {                 
                    kPoolAlloc_PrepareFreeBlockBasedBuffer(object, rank); 
                }

                if (rankInfo->freeBufferList.count > 0)
                {
                    kPoolAllocBufferHeader* buffer = rankInfo->freeBufferList.first; 

                    kAssert(buffer->info.block->rank == rank); 

                    kPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 
                    kPoolAlloc_AddBlockRef(object, buffer->info.block); 

                    *(void**)mem = kAt_(buffer, sizeof(kPoolAllocBufferHeader));     
                    isAllocated = kTRUE; 
                }
            }
            else if (rankInfo->isCached)
            {
                if (rankInfo->freeBufferList.count == 0)
                {                 
                    kPoolAlloc_AllocCachedBuffers(object, rank, 1); 
                }

                if (rankInfo->freeBufferList.count > 0)
                {
                    kPoolAllocBufferHeader* buffer = rankInfo->freeBufferList.first; 

                    kPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 
                    obj->cacheSize -= rankInfo->size; 

                    *(void**)mem = kAt_(buffer, sizeof(kPoolAllocBufferHeader));                         
                    isAllocated = kTRUE; 
                }
            }
        }

        if (!isAllocated)
        {
            kTest(kPoolAlloc_AllocUnmanagedBuffer(object, size, mem));
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_VFree(kPoolAlloc object, void* mem)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kAlloc innerAlloc = kObject_Alloc_(object); 

    kLock_Enter(obj->lock); 

    kTry
    {   
        if (!kIsNull(mem))
        {
            kPoolAllocBufferHeader* buffer = kAt_(mem, -1*(kSSize)sizeof(kPoolAllocBufferHeader)); 
               
            if (buffer->type == obj->blockAllocId)
            {
                kPoolAllocBlockHeader* block = buffer->info.block; 
                kPoolAllocRank* rankInfo = &obj->rankInfo[block->rank]; 

                //CPU cache optimization: insert free buffer at front of list, so that most-recently-used buffer will be reused first
                kPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, rankInfo->freeBufferList.first, buffer); 
                kPoolAlloc_RemoveBlockRef(object, block); 
            }
            else if (buffer->type == obj->cacheAllocId)
            {
                kPoolAllocRank* rankInfo = &obj->rankInfo[buffer->info.rank]; 

                if ((obj->cacheSize + rankInfo->size) <= obj->cacheCapacity)
                {
                    //CPU cache optimization: insert free buffer at front of list, so that most-recently-used buffer will be reused first
                    kPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, rankInfo->freeBufferList.first, buffer); 
                    obj->cacheSize += rankInfo->size; 
                }
                else
                {
                    obj->totalSize -= sizeof(kPoolAllocBufferHeader) + rankInfo->size; 
                    rankInfo->bufferCount--; 

                    kTest(kAlloc_Free_(innerAlloc, buffer));             
                }
            }
            else if (buffer->type == obj->unmanagedAllocId)
            {
                obj->totalSize -= sizeof(kPoolAllocBufferHeader) + buffer->info.size; 

                kTest(kAlloc_Free_(innerAlloc, buffer));             
            }
            else
            {
                //if this assertion is reached, an invalid buffer was passed to this function (corrupted, or not from this allocator)
                kAssert(kFALSE); 
                kThrow(kERROR_PARAMETER); 
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_Reserve(kPoolAlloc object, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kLock_Enter(obj->lock); 

    kTry
    {   
        obj->minBlockCount = kCeilUInt_(size, obj->nominalBlockSize); 

        while (obj->blockCount < obj->minBlockCount)
        {
            kTest(kPoolAlloc_AllocBlock(object)); 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_ReserveAt(kPoolAlloc object, kSize rank, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    kLock_Enter(obj->lock); 

    kTry
    {   
        if (rank < obj->blockBasedRankCount)
        {
            kTest(kPoolAlloc_ReserveBlocksAt(object, rank, size)); 
        }
        else if (rank < obj->cacheBasedRankCount)
        {
            kTest(kPoolAlloc_ReserveCachedBuffersAt(object, rank, size)); 
        }
        else
        {
            kThrow(kERROR_PARAMETER); 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_ReserveBlocksAt(kPoolAlloc object, kSize rank, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

    rankInfo->minBufferCount = kCeilUInt_(size, rankInfo->size); 

    //reuse existing blocks, if possible
    if (obj->blockReuseEnabled && (rankInfo->minBufferCount > rankInfo->bufferCount))
    {
        kPoolAlloc_RecycleBlocks(object, kCeilUInt_(rankInfo->minBufferCount - rankInfo->bufferCount, rankInfo->buffersPerBlock)); 
    }

    //allocate the rest
    while ((rankInfo->bufferCount + obj->unassignedBlockList.count*rankInfo->buffersPerBlock) < rankInfo->minBufferCount)
    {
        kCheck(kPoolAlloc_AllocBlock(object)); 
    }

    //assign allocated buffers to the requested rank
    while (rankInfo->bufferCount < rankInfo->minBufferCount)
    {       
        kPoolAllocBlockHeader* block = obj->unassignedBlockList.first; 

        kAssert(obj->unassignedBlockList.count > 0); 

        kPoolAlloc_RemoveFromBlockList(object, &obj->unassignedBlockList, block); 
        kPoolAlloc_InsertIntoBlockList(object, &rankInfo->newBlockList, kNULL, block);

        block->rank = rankInfo->rank; 
        rankInfo->bufferCount += rankInfo->buffersPerBlock;            
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_ReserveCachedBuffersAt(kPoolAlloc object, kSize rank, kSize size)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

    rankInfo->minBufferCount = kCeilUInt_(size, rankInfo->size); 

    if (rankInfo->bufferCount < rankInfo->minBufferCount)
    {
        kCheck(kPoolAlloc_AllocCachedBuffers(object, rank, rankInfo->minBufferCount - rankInfo->bufferCount)); 
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_Clear(kPoolAlloc object)
{
    return kPoolAlloc_ClearEx(object, kFALSE); 
}

kFx(kStatus) kPoolAlloc_ClearAll(kPoolAlloc object)
{
    return kPoolAlloc_ClearEx(object, kTRUE); 
}

kFx(kStatus) kPoolAlloc_PrepareFreeBlockBasedBuffer(kPoolAlloc object, kSize rank)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

    if (rankInfo->freeBufferList.count == 0)
    {
        //if we don't have a "new" block to draw from, attempt to reuse or allocate one
        if (rankInfo->newBlockList.count == 0)
        {
            if ((obj->unassignedBlockList.count == 0) && obj->blockReuseEnabled)
            {
                kPoolAlloc_RecycleBlocks(object, 1); 
            }
           
            if ((obj->unassignedBlockList.count == 0))
            {
                kPoolAlloc_AllocBlock(object); 
            }

            if (obj->unassignedBlockList.count > 0)
            {
                kPoolAllocBlockHeader* block = obj->unassignedBlockList.first; 

                kPoolAlloc_RemoveFromBlockList(object, &obj->unassignedBlockList, block); 
                kPoolAlloc_InsertIntoBlockList(object, &rankInfo->newBlockList, kNULL, block); 

                block->rank = rank; 
                rankInfo->bufferCount += rankInfo->buffersPerBlock; 
            }
        }

        //if we now have a "new" block to draw from, get a free buffer from the block
        if (rankInfo->newBlockList.count > 0)
        {
            kPoolAllocBlockHeader* block = rankInfo->newBlockList.first;
            kPoolAllocBufferHeader* buffer = kAt_(block, sizeof(kPoolAllocBlockHeader) + rankInfo->bufferStride*block->allocCount); 

            kAssert(block->rank == rank); 

            buffer->type = obj->blockAllocId; 
            buffer->info.block = block; 
            buffer->next = kNULL; 
            buffer->previous = kNULL; 

            kPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, kNULL, buffer); 

            block->allocCount++; 

            if (block->allocCount == rankInfo->buffersPerBlock)
            {
                kPoolAlloc_RemoveFromBlockList(object, &rankInfo->newBlockList, block); 
                kPoolAlloc_RemoveBlockRef(object, block); 
            }
        }
    }

    return (rankInfo->freeBufferList.count > 0) ? kOK : kERROR_NOT_FOUND; 
}

kFx(void) kPoolAlloc_RemoveFromBlockList(kPoolAlloc object, kPoolAllocBlockList* list, kPoolAllocBlockHeader* item)
{
    if (!kIsNull(item->previous))
    {
        item->previous->next = item->next; 
    }
    
    if (!kIsNull(item->next))
    {
        item->next->previous = item->previous; 
    }

    if (list->first == item)
    {
        list->first = item->next; 
    }

    if (list->last == item)
    {
        list->last = item->previous; 
    }

    item->next = kNULL; 
    item->previous = kNULL;

    list->count--; 
} 

kFx(void) kPoolAlloc_InsertIntoBlockList(kPoolAlloc object, kPoolAllocBlockList* list, kPoolAllocBlockHeader* before, kPoolAllocBlockHeader* item)
{
    item->previous = kNULL; 
    item->next = kNULL; 

    if (kIsNull(list->first))
    {
        list->first = item; 
        list->last = item; 
    }
    else if (kIsNull(before))
    {
        item->previous = list->last; 
        list->last->next = item; 
        list->last = item;         
    }
    else if (before == list->first)
    {
        item->next = list->first; 
        list->first->previous = item; 
        list->first = item;         
    }
    else
    {
        item->next = before; 
        item->previous = before->previous; 
        before->previous->next = item; 
        before->previous = item;         
    }    

    list->count++; 
} 

kFx(void) kPoolAlloc_RemoveFromBufferList(kPoolAlloc object, kPoolAllocBufferList* list, kPoolAllocBufferHeader* item)
{
    if (!kIsNull(item->previous))
    {
        item->previous->next = item->next; 
    }
    
    if (!kIsNull(item->next))
    {
        item->next->previous = item->previous; 
    }

    if (list->first == item)
    {
        list->first = item->next; 
    }

    if (list->last == item)
    {
        list->last = item->previous; 
    }

    item->next = kNULL; 
    item->previous = kNULL;

    list->count--; 
} 

kFx(void) kPoolAlloc_InsertIntoBufferList(kPoolAlloc object, kPoolAllocBufferList* list, kPoolAllocBufferHeader* before, kPoolAllocBufferHeader* item)
{
    item->previous = kNULL; 
    item->next = kNULL; 

    if (kIsNull(list->first))
    {
        list->first = item; 
        list->last = item; 
    }
    else if (kIsNull(before))
    {
        item->previous = list->last; 
        list->last->next = item; 
        list->last = item;         
    }
    else if (before == list->first)
    {
        item->next = list->first; 
        list->first->previous = item; 
        list->first = item;         
    }
    else
    {
        item->next = before; 
        item->previous = before->previous; 
        before->previous->next = item; 
        before->previous = item;         
    }    

    list->count++; 
} 

kFx(kStatus) kPoolAlloc_AddBlockRef(kPoolAlloc object, kPoolAllocBlockHeader* block)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    if (block->refCount == 0)
    {
        kPoolAlloc_RemoveFromBlockList(object, &obj->rankInfo[block->rank].freeBlockList, block); 
    }

    block->refCount++; 

    return kOK; 
}

kFx(kStatus) kPoolAlloc_RemoveBlockRef(kPoolAlloc object, kPoolAllocBlockHeader* block)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 

    block->refCount--; 

    if (block->refCount == 0)
    {
        kPoolAlloc_InsertIntoBlockList(object, &obj->rankInfo[block->rank].freeBlockList, kNULL, block); 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_RecycleBlocks(kPoolAlloc object, kSize maxCount)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kSize count = 0; 
    kSize i; 
    
    for (i = 0; i < obj->blockBasedRankCount; ++i)
    {
        kSize rankIndex = obj->blockBasedRankCount - i - 1;     //largest rank first (fewer buffers to unhook)
        kPoolAllocRank* rankInfo = &obj->rankInfo[rankIndex]; 
            
        //try to reuse surplus "free" blocks
        while ((rankInfo->freeBlockList.count > 0) && 
               (rankInfo->bufferCount >= (rankInfo->minBufferCount + rankInfo->buffersPerBlock)))
        {
            kCheck(kPoolAlloc_ReturnBlockToUnassigned(object, &rankInfo->freeBlockList, rankInfo->freeBlockList.last)); 

            if (++count == maxCount)
            {
                return kOK;
            }
        }

        //try to reuse surplus "new" blocks 
        while ((rankInfo->newBlockList.count > 0) && (rankInfo->newBlockList.last->refCount == 1) && 
               (rankInfo->bufferCount >= (rankInfo->minBufferCount + rankInfo->buffersPerBlock)))
        {
            kCheck(kPoolAlloc_ReturnBlockToUnassigned(object, &rankInfo->newBlockList, rankInfo->newBlockList.last)); 

            if (++count == maxCount)
            {
                return kOK;
            }
        }      
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_ReturnBlockToUnassigned(kPoolAlloc object, kPoolAllocBlockList* list, kPoolAllocBlockHeader* block)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kPoolAllocRank* rankInfo = &obj->rankInfo[block->rank]; 
    kSize i; 

    for (i = 0; i < block->allocCount; ++i)
    {
        kPoolAllocBufferHeader* buffer = kAt_(block, sizeof(kPoolAllocBlockHeader) + i*rankInfo->bufferStride); 

        kPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 
    }

    rankInfo->bufferCount -= rankInfo->buffersPerBlock; 

    block->allocCount = 0; 
    block->rank = kSIZE_NULL; 
    block->refCount = 1; 

    kPoolAlloc_RemoveFromBlockList(object, list, block); 
    kPoolAlloc_InsertIntoBlockList(object, &obj->unassignedBlockList, kNULL, block);

    return kOK; 
}

kFx(kStatus) kPoolAlloc_AllocBlock(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kAlloc innerAlloc = kObject_Alloc_(object); 
    kSize allocSize = kPoolAlloc_BlockSize_(object); 
    kPoolAllocBlockHeader* block = kNULL; 

    if (((obj->blockCount + 1)*obj->nominalBlockSize > obj->blockCapacity) || 
        ((obj->totalSize + allocSize) > obj->totalCapacity))
    {
        return kERROR_MEMORY; 
    }

    kCheck(kAlloc_Get_(innerAlloc, allocSize, &block)); 

    block->allocCount = 0; 
    block->refCount = 1; 
    block->rank = kSIZE_NULL; 
    block->next = kNULL; 
    block->previous = kNULL; 

    obj->blockCount++; 
    obj->totalSize += allocSize; 

    kPoolAlloc_InsertIntoBlockList(object, &obj->unassignedBlockList, kNULL, block); 

    return kOK; 
}

kFx(kStatus) kPoolAlloc_ClearEx(kPoolAlloc object, kBool all)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kSize i; 

    kLock_Enter(obj->lock); 

    kTry
    {
        //clear reservations, if requested
        if (all)
        {            
            kPoolAlloc_Reserve(object, 0); 

            for (i = 0; i < obj->managedRankCount; ++i)
            {
                kPoolAlloc_ReserveAt(object, i, 0); 
            }
        }

        //blocks
        kTest(kPoolAlloc_RecycleBlocks(object, kSIZE_MAX)); 
        kTest(kPoolAlloc_FreeUnassignedBlocks(object)); 

        //cached buffers
        kTest(kPoolAlloc_FreeCachedBuffers(object)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_AllocCachedBuffers(kPoolAlloc object, kSize rank, kSize count)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 
    kAlloc innerAlloc = kObject_Alloc_(object); 
    kSize allocSize = sizeof(kPoolAllocBufferHeader) + rankInfo->size; 
    kSize i; 

    for (i = 0; i < count; ++i)
    {
        kPoolAllocBufferHeader* buffer = kNULL; 

        if ((obj->totalSize + allocSize) > obj->totalCapacity)
        {
            return kERROR_MEMORY; 
        }

        kCheck(kAlloc_Get_(innerAlloc, allocSize, &buffer)); 

        buffer->type = obj->cacheAllocId; 
        buffer->info.rank = rank; 
        buffer->next = kNULL; 
        buffer->previous = kNULL; 

        kPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, kNULL, buffer); 

        rankInfo->bufferCount++; 
        obj->cacheSize += rankInfo->size;          
        obj->totalSize += allocSize; 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_AllocUnmanagedBuffer(kPoolAlloc object, kSize size, void* mem)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kAlloc innerAlloc = kObject_Alloc_(object); 
    kSize allocSize = sizeof(kPoolAllocBufferHeader) + size; 
    kPoolAllocBufferHeader* buffer = kNULL; 

    if ((obj->totalSize + allocSize) > obj->totalCapacity)
    {
        return kERROR_MEMORY; 
    }

    kCheck(kAlloc_Get_(innerAlloc, allocSize, &buffer)); 

    buffer->type = obj->unmanagedAllocId; 
    buffer->info.size = size; 
    buffer->next = kNULL; 
    buffer->previous = kNULL; 

    obj->totalSize += allocSize; 

    *(void**)mem = kAt_(buffer, sizeof(kPoolAllocBufferHeader));   

    return kOK; 
}

kFx(kStatus) kPoolAlloc_FreeUnassignedBlocks(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kAlloc internalAlloc = kObject_Alloc_(object); 

    while ((obj->blockCount > obj->minBlockCount) && (obj->unassignedBlockList.count > 0))
    {
        kPoolAllocBlockHeader* block = obj->unassignedBlockList.first; 

        kPoolAlloc_RemoveFromBlockList(object, &obj->unassignedBlockList, block); 
        
        kCheck(kAlloc_Free_(internalAlloc, block)); 

        obj->totalSize -= kPoolAlloc_BlockSize_(object); 
        obj->blockCount--; 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_FreeCachedBuffers(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kAlloc internalAlloc = kObject_Alloc_(object); 
    kSize i;

    for (i = obj->blockBasedRankCount; i < obj->cacheBasedRankCount; ++i)
    {
        kPoolAllocRank* rankInfo = &obj->rankInfo[i]; 

        kAssert(rankInfo->isCached); 

        while ((rankInfo->bufferCount > rankInfo->minBufferCount) && (rankInfo->freeBufferList.count > 0))
        {
            kPoolAllocBufferHeader* buffer = rankInfo->freeBufferList.first;

            kPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 

            kCheck(kAlloc_Free_(internalAlloc, buffer)); 

            obj->cacheSize -= rankInfo->size; 
            obj->totalSize -= sizeof(kPoolAllocBufferHeader) + rankInfo->size; 

            rankInfo->bufferCount--; 
        }
    }

    return kOK; 
}

kFx(kSize) kPoolAlloc_TotalSize(kPoolAlloc object)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kSize size; 

    kLock_Enter(obj->lock); 
    {
        size = obj->totalSize; 
    }
    kLock_Exit(obj->lock); 

    return size; 
}

kFx(kSize) kPoolAlloc_BufferCountAt(kPoolAlloc object, kSize rank)
{
    kPoolAllocClass* obj = kPoolAlloc_Cast_(object); 
    kSize count = 0; 

    kLock_Enter(obj->lock); 
    {
        if (rank < obj->managedRankCount)
        {
            count += obj->rankInfo[rank].bufferCount; 
        }
    }
    kLock_Exit(obj->lock); 

    return count; 
}
