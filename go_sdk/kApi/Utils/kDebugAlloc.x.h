/** 
 * @file    kDebugAlloc.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DEBUG_ALLOC_X_H
#define K_API_DEBUG_ALLOC_X_H

#include <kApi/kAlloc.h>
kBeginHeader()

#define K_DEBUG_ALLOC_PAD_SIZE          (8)             //size of memory before and after each allocation to fill with test pattern

kDeclareValue(k, kDebugAllocation, kValue)

typedef struct kDebugAllocClass
{
    kAllocClass base; 
    kLock lock;                     //provides mutual exclusion
    kText32 name;                   //descriptive name for this allocator
    kCallback allocListener;        //notified on allocations
    kSize allocated;                //count of allocated memory, in bytes
    k64u counter;                   //incremented with each allocation
    kMap history;                   //records all outstanding allocations -- kMap<kPointer, kDebugAllocation>
    kArray1 padPattern;             //known pattern before and after each allocation (used to check for corruption)
    kMap candidateObjectLeaks;      //map of potential object leaks (allocation index to type name) -- kMap<k64u, kTypeName>
    kBackTrace backTrace;           //temporary variable used to store back trace information.
    kMap backTraceDescriptions;     //map of allocation back trace information -- kMap<kBackTrace, kArrayList<kString>>
} kDebugAllocClass;

kDeclareClass(k, kDebugAlloc, kAlloc)

kFx(kStatus) kDebugAlloc_Init(kDebugAlloc object, kType type, const kChar* name, kAlloc alloc); 
kFx(kStatus) kDebugAlloc_VRelease(kDebugAlloc object); 

kFx(kStatus) kDebugAlloc_CandidateObjectAllocations(kDebugAlloc object, k64u since, kArrayList* history, kAlloc alloc);
kFx(kStatus) kDebugAlloc_InitPadPattern(kDebugAlloc object); 

kFx(kStatus) kDebugAlloc_VGet(kDebugAlloc object, kSize size, void* mem); 
kFx(kStatus) kDebugAlloc_VFree(kDebugAlloc object, void* mem); 

#define kDebugAlloc_Cast_(A)                    kCastClass_(kDebugAlloc, A)
#define kDebugAlloc_(A)                         kCast_(kDebugAllocClass*, A)

kEndHeader()

#endif
