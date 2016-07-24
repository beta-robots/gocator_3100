/** 
 * @file    kDebugAlloc.h
 * @brief   Declares the kDebugAlloc class. 
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DEBUG_ALLOC_H
#define K_API_DEBUG_ALLOC_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @struct  kDebugAllocation
 * @extends kValue
 * @ingroup kApi-Utils
 * @brief   Allocation record used by kDebugAlloc.
 */
typedef struct kDebugAllocation
{
    kByte* data;            ///< User data pointer.
    kSize size;             ///< Size of memory allocation.
    k64u index;             ///< Incremented with each allocation.
    kArrayList trace;       ///< Backtrace at point of allocation -- kArrayList<kString>.
} kDebugAllocation; 

/**
 * @class   kDebugAlloc
 * @extends kAlloc
 * @ingroup kApi-Utils
 * @brief   Debug memory allocator that can track allocations and report leaks.
 */
//typedef kAlloc kDebugAlloc;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a new kDebugAlloc allocator. 
 *
 * @public               @memberof kDebugAlloc
 * @param   object       Receives the constructed kDebugAlloc object. 
 * @param   name         Descriptive name for this memory allocator.
 * @param   allocator    Memory allocator to use for internal allocations (or kNULL for default). 
 * @return               Operation status. 
 */
kFx(kStatus) kDebugAlloc_Construct(kDebugAlloc* object, const kChar* name, kAlloc allocator); 

/** 
 * Clears all outstanding allocations (resets allocator). 
 *
 * @public          @memberof kDebugAlloc
 * @param   object  Memory allocator.  
 * @return          Operation status. 
 */
kFx(kStatus) kDebugAlloc_Clear(kDebugAlloc object); 

/** 
 * Returns the total amount of memory that has been allocated and not yet freed.
 *
 * @public          @memberof kDebugAlloc
 * @param   object  Memory allocator.  
 * @return          Amount of allocated memory.
 */
kFx(kSize) kDebugAlloc_Allocated(kDebugAlloc object); 

/** 
 * Returns a memory checkpoint value, used in conjunction with allocation logging functions.
 *
 * @public          @memberof kDebugAlloc
 * @param   object  Memory allocator.  
 * @return          Memory checkpoint value.
 * @see             kDebugAlloc_Allocations, kDebugAlloc_LogAllocations
 */
kFx(k64u) kDebugAlloc_Checkpoint(kDebugAlloc object);

/** 
 * Gets a list of all outstanding memory allocations performed after the given checkpoint. 
 *
 * @public              @memberof kDebugAlloc
 * @param   object      Memory allocator.  
 * @param   since       Memory checkpoint (or zero for beginning). 
 * @param   history     Receives list of outstanding allocations. 
 * @param   alloc       Allocator for history list (or kNULL for default). 
 * @return              Operation status.
 * @see                 kDebugAlloc_Checkpoint
 */
kFx(kStatus) kDebugAlloc_Allocations(kDebugAlloc object, k64u since, kArrayList* history, kAlloc alloc);

/** 
 * Logs all outstanding memory allocations performed after the given checkpoint (using kLogf). 
 *
 * @public                  @memberof kDebugAlloc
 * @param   object          Memory allocator.  
 * @param   since           Memory checkpoint (or zero for beginning). 
 * @return                  Operation status.
 * @see                     kDebugAlloc_Checkpoint
 */
kFx(kStatus) kDebugAlloc_LogAllocations(kDebugAlloc object, k64u since); 

/** 
 * Makes note of outstanding allocations that appear to be objects from any currently-loaded assembly. 
 * 
 * This function records information about outstanding allocations that appear to be objects, for later use
 * when logging leak information.
 * 
 * @public                  @memberof kDebugAlloc
 * @param   object          Memory allocator.  
 * @param   since           Memory checkpoint (or zero for beginning). 
 * @return                  Operation status.
 * @see                     kDebugAlloc_LogAllocations, kDebugAlloc_DetectLeakedAssemblyObjects
 */
kFx(kStatus) kDebugAlloc_DetectLeakedObjects(kDebugAlloc object, k64u since); 

/** 
 * Makes note of outstanding allocations that appear to be objects associated with the specified assembly. 
 * 
 * This function records information about outstanding allocations that appear to be objects, for later use
 * when logging leak information.
 * 
 * @public                  @memberof kDebugAlloc
 * @param   object          Memory allocator.  
 * @param   since           Memory checkpoint (or zero for beginning). 
 * @param   assembly        Assembly associated with objects. 
 * @return                  Operation status.
 * @see                     kDebugAlloc_LogAllocations, kDebugAlloc_DetectLeakedObjects
 */
kFx(kStatus) kDebugAlloc_DetectLeakedAssemblyObjects(kDebugAlloc object, k64u since, kAssembly assembly); 

/** 
 * Registers a user-defined function that is called whenever a memory allocation occurs. 
 * 
 * The args parameter of the callback will receive a pointer to a kDebugAllocation structure 
 * representing the memory allocation.
 *
 * @public              @memberof kDebugAlloc
 * @param   object      Memory allocator.  
 * @param   function    Callback function. 
 * @param   receiver    Callback receiver. 
 * @return              Operation status.
 */
kFx(kStatus) kDebugAlloc_SetAllocListener(kDebugAlloc object, kCallbackFx function, kPointer receiver); 

kEndHeader()

#include <kApi/Utils/kDebugAlloc.x.h>

#endif
