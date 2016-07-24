/** 
 * @file    kList.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LIST_X_H
#define K_API_LIST_X_H

kBeginHeader()

#define kLIST_MIN_ITEMS_PER_BLOCK            (16)            //minimum number of list items per allocated memory block

//represents a single list entry
typedef struct kListItemStruct //<T>
{
    struct kListItemStruct* next;               //linked list pointer to next item
    struct kListItemStruct* previous;           //linked list pointer to previous item (ignored/unused on free list)
    //T content;                                //content; size dynamically calculated
} kListItemStruct;

//represents a block of allocated list items; multiple items
//are allocated at the same time for efficiency
typedef struct kListItemBlock //<T, N>
{
    struct kListItemBlock* next;             //linked list pointer to next memory block 
    kSize itemCount;                         //count of items in block (N)
    kListItemStruct* items;                  //pointer to first item in block
    //kListItemStruct<T> item[N];            //array of list items
} kListItemBlock; 

typedef struct kListClass
{
    kObjectClass base;                          // base fields
    kListItemBlock* blocks;                     // allocated memory blocks, where block contains N items
    kListItemStruct* free;                      // singly-linked list of free items
    kListItemStruct* first;                     // pointer to first list item
    kListItemStruct* last;                      // pointer to last list item
    kSize count;                                // current count of items
    kSize capacity;                             // maximum number of items before allocation required
    kStructField contentField;                  // content field info
    kSize itemSize;                             // size of list item, including content fields
} kListClass;

kDeclareClass(k, kList, kObject) 

kFx(kStatus) kList_Init(kList list, kType classType, kType itemType, kSize initialCapacity, kAlloc allocator); 

kFx(kStatus) kList_VInitClone(kList list, kList source, kAlloc allocator); 

kFx(kStatus) kList_VRelease(kList list); 
kFx(kStatus) kList_VDisposeItems(kList list); 

kFx(kSize) kList_VSize(kObject object); 

kFx(kStatus) kList_WriteDat6V0(kList list, kSerializer serializer); 
kFx(kStatus) kList_ReadDat6V0(kList list, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kList_Deallocate(kList list); 

kFx(kStatus) kList_Layout(kList list, kType itemType); 

kFx(kIterator) kList_CollectionGetIterator(kList list);
kFx(kType) kList_CollectionItemType(kList list);
kFx(kBool) kList_CollectionHasNext(kList list, kIterator iterator); 
kFx(void*) kList_CollectionNext(kList list, kIterator* iterator); 

#define kList_(LIST)                            kCast_(kListClass*, LIST)
#define kList_Cast_(LIST)                       kCastClass_(kList, LIST)

#define kList_ItemIsRef_(LIST)                  (kType_IsReference_(kList_ItemType_(LIST)))

#define kxList_ItemType_(LIST)                  (kList_(LIST)->contentField.type)
#define kxList_Count_(LIST)                     (kList_(LIST)->count)
#define kxList_Capacity_(LIST)                  (kList_(LIST)->capacity)
#define kxList_First_(LIST)                     (kList_(LIST)->first)
#define kxList_Last_(LIST)                      (kList_(LIST)->last)
#define kxList_Next_(LIST, ITEM)                (kListItem_(ITEM)->next)
#define kxList_Previous_(LIST, ITEM)            (kListItem_(ITEM)->previous)

#define kxList_At_(LIST, ITEM)                  (kListItem_Content_(LIST, ITEM))
#define kxList_As_(LIST, ITEM, TYPE)            (*(TYPE*)kListItem_Content_(LIST, ITEM))

#define kListItem_(ITEM)                        kCast_(kListItemStruct*, ITEM)
#define kListItem_Content_(LIST, ITEM)          kAt_((ITEM), kList_(LIST)->contentField.offset)

kEndHeader()

#endif
