/** 
 * @file    kBox.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BOX_X_H
#define K_API_BOX_X_H

kBeginHeader()

typedef struct kBoxClass
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* item;                 //array memory 
} kBoxClass;
    
kDeclareClass(k, kBox, kObject) 

kFx(kStatus) kBox_Init(kBox box, kType classType, kType itemType, kAlloc allocator);

kFx(kStatus) kBox_VInitClone(kBox box, kBox source, kAlloc allocator); 
kFx(kStatus) kBox_VRelease(kBox box); 
kFx(kSize) kBox_VSize(kBox box); 

kFx(kStatus) kBox_WriteDat5V1(kBox box, kSerializer serializer); 
kFx(kStatus) kBox_ReadDat5V1(kBox box, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kBox_WriteDat6V0(kBox box, kSerializer serializer); 
kFx(kStatus) kBox_ReadDat6V0(kBox box, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kBox_Realloc(kBox box); 

#define kBox_(BOX)                    kCast(kBoxClass*, BOX)
#define kBox_Cast_(BOX)               kCastClass_(kBox, BOX)

#define kxBox_Data_(BOX)              (kBox_(BOX)->item)
#define kxBox_As_(BOX, TYPE)          (kAs_(kBox_Data_(BOX), TYPE))

#define kxBox_ItemType_(BOX)          (kBox_(BOX)->itemType)
#define kxBox_ItemSize_(BOX)          (kBox_(BOX)->itemSize)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_BOX                    kTypeOf(kBox)
#   define kBox_Destroy                 kObject_Destroy
#   define kBox_Alloc                   kBox_Allocate
#   define kBOX_DATA                    kBox_Data_
#   define kBOX_ITEM_TYPE               kBox_ItemType_
#   define kBOX_ITEM_SIZE               kBox_ItemSize_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kBox_Construct5(B, T)        kBox_Construct(B, T, kNULL)

#endif

kEndHeader()

#endif
