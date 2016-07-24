/** 
 * @file    kCollection.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_COLLECTION_X_H
#define K_API_COLLECTION_X_H

kBeginHeader()

typedef struct kCollectionVTable
{   
    kIterator (kCall* VGetIterator)(kCollection collection);
    kType (kCall* VItemType)(kCollection collection);
    kSize (kCall* VCount)(kCollection collection); 
    kBool (kCall* VHasNext)(kCollection collection, kIterator iterator); 
    void* (kCall* VNext)(kCollection collection, kIterator* iterator);  
} kCollectionVTable;

kDeclareInterface(k, kCollection, kNull) 

kFx(kIterator) kCollection_VGetIterator(kCollection collection);
kFx(kType) kCollection_VItemType(kCollection collection);
kFx(kSize) kCollection_VCount(kCollection collection); 
kFx(kBool) kCollection_VHasNext(kCollection collection, kIterator iterator); 
kFx(void*) kCollection_VNext(kCollection collection, kIterator* iterator); 

#define kCollection_VTable_(C)              kCastIVTable_(kCollection, C)

#define kCollection_GetIterator_(C)         (kCollection_VTable_(C)->VGetIterator(C))
#define kCollection_ItemType_(C)            (kCollection_VTable_(C)->VItemType(C))
#define kCollection_Count_(C)               (kCollection_VTable_(C)->VCount(C))
#define kCollection_HasNext_(C, I)          (kCollection_VTable_(C)->VHasNext(C, I))
#define kCollection_Next_(C, I)             (kCollection_VTable_(C)->VNext(C, I))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_COLLECTION                 kTypeOf(kCollection)

#endif

kEndHeader()

#endif
