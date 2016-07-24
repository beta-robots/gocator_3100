/** 
 * @file    kAlloc.h
 * @brief   Declares the kAlloc class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_ALLOC_H
#define K_API_ALLOC_H

kBeginHeader()

/**
 * @class   kAlloc
 * @extends kObject
 * @ingroup kApi
 * @brief   Abstract base class for memory allocator types. 
 */

/** 
 * Allocates a block of memory. 
 *
 * @public              @memberof kAlloc
 * @param   alloc       Allocator object. 
 * @param   size        Size of memory to be allocated.
 * @param   mem         Receives pointer to allocated memory. 
 * @return              Operation status. 
 */
kFx(kStatus) kAlloc_Get(kAlloc alloc, kSize size, void* mem);

/** 
 * Allocates a block of memory and zero-initializes the block. 
 *
 * @public              @memberof kAlloc
 * @param   alloc       Allocator object. 
 * @param   size        Size of memory to be allocated.
 * @param   mem         Receives pointer to allocated memory. 
 * @return              Operation status. 
 */
kFx(kStatus) kAlloc_GetZero(kAlloc alloc, kSize size, void* mem); 

/** 
 * Frees a block of memory. 
 *
 * @public              @memberof kAlloc
 * @param   alloc       Allocator object. 
 * @param   mem         Pointer to memory to be freed. 
 * @return              Operation status. 
 */
kFx(kStatus) kAlloc_Free(kAlloc alloc, void* mem);

/** 
 * Frees a block of memory and sets the memory pointer to kNULL. 
 *
 * @public              @memberof kAlloc
 * @param   alloc       Allocator object. 
 * @param   mem         Pointer to pointer to memory to be freed. 
 * @return              Operation status. 
 */
kFx(kStatus) kAlloc_FreeRef(kAlloc alloc, void* mem);

/** 
 * Gets the allocator that should normally be used by applications to request memory.
 *
 * The App alloctor is typically implemented by layering additional allocators on the 
 * System allocator (e.g., kPoolAlloc, kDebugAlloc). 
 *
 * @public      @memberof kAlloc
 * @return      Application memory allocator. 
 */
kFx(kAlloc) kAlloc_App();

/** 
 * Gets the system allocator. 
 * 
 * The System allocator is a kUserAlloc instance that allocates directly from main system memory. 
 * 
 * For most purposes, the App allocator should be prefered to the System allocator. Use of the System 
 * allocator may be appropriate when the additional layers provided by the App allocator are undesirable.
 *
 * @public      @memberof kAlloc
 * @return      System memory allocator. 
 */
kFx(kAlloc) kAlloc_System();

/** 
 * Returns the passed allocator, or if null, the App allocator. 
 *
 * @public              @memberof kAlloc
 * @param   alloc       Allocator. 
 * @return              Allocator. 
 */
kFx(kAlloc) kAlloc_Fallback(kAlloc alloc);

/** @relates kAlloc @{ */
#define kAlloc_Get_(ALLOC, SIZE, MEM)           kxAlloc_Get_(ALLOC, SIZE, MEM)             ///< Macro version of kAlloc_Get
#define kAlloc_GetZero_(ALLOC, SIZE, MEM)       kxAlloc_GetZero_(ALLOC, SIZE, MEM)         ///< Macro version of kAlloc_GetZero
#define kAlloc_Free_(ALLOC, MEM)                kxAlloc_Free_(ALLOC, MEM)                  ///< Macro version of kAlloc_Free
#define kAlloc_FreeRef_(ALLOC, MEM)             kxAlloc_FreeRef_(ALLOC, MEM)               ///< Macro version of kAlloc_FreeRef
#define kAlloc_App_()                           kxAlloc_App_()                             ///< Macro version of kAlloc_App
#define kAlloc_System_()                        kxAlloc_System_()                          ///< Macro version of kAlloc_System
#define kAlloc_Fallback_(ALLOC)                 kxAlloc_Fallback_(ALLOC)                   ///< Macro version of kAlloc_Fallback
/** @} */

kEndHeader()

#include <kApi/kAlloc.x.h>

#endif
