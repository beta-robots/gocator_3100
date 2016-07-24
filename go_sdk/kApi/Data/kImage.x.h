/** 
 * @file    kImage.x.h
 *
 * @internal
 * Copyright (C) 2003-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_IMAGE_X_H
#define K_API_IMAGE_X_H

kBeginHeader()

#define kIMAGE_ALIGNMENT       (3)          //align rows to 8-byte boundaries by default

typedef struct kImageClass
{
    kObjectClass base; 
    kType pixelType;            //item type
    kSize pixelSize;            //item size, in bytes
    kSize allocSize;            //size of allocated image memory, in bytes
    void* pixels;               //image memory 
    kSize stride;               //image stride (row size), in pixels
    kSize width;                //image width, in pixels
    kSize height;               //image height, in pixels
    kBool isAttached;           //is image memory externally owned?
    kPixelFormat format;        //optional pixel format descriptor
    kCfa cfa;                   //optional color filter array type
} kImageClass;
    
kDeclareClass(k, kImage, kObject) 

kFx(kStatus) kImage_Init(kImage image, kType classType, kType pixelType, kSize width, kSize height, kAlloc allocator);

kFx(kStatus) kImage_VInitClone(kImage image, kImage source, kAlloc allocator); 

kFx(kStatus) kImage_VRelease(kImage image); 

kFx(kSize) kImage_VSize(kImage image); 

kFx(kStatus) kImage_WriteDat5V3(kImage image, kSerializer serializer); 
kFx(kStatus) kImage_ReadDat5V3(kImage image, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kImage_WriteDat6V0(kImage image, kSerializer serializer); 
kFx(kStatus) kImage_ReadDat6V0(kImage image, kSerializer serializer, kAlloc allocator); 

kFx(kStatus) kImage_Realloc(kImage image, kSize width, kSize height); 

#define kImage_(IMAGE)                     kCast(kImageClass*, IMAGE)
#define kImage_Cast_(IMAGE)                kCastClass_(kImage, IMAGE)

#define kImage_Pitch_(IMAGE)               kxImage_Pitch_(IMAGE)

#define kxImage_Data_(IMAGE)               (kImage_(IMAGE)->pixels)
#define kxImage_DataSize_(IMAGE)           (kImage_Height_(IMAGE) * kImage_Stride_(IMAGE))
#define kxImage_At_(IMAGE, X, Y)           ((void*)((kByte*)kImage_Data_(IMAGE) + (Y)*kImage_Stride_(IMAGE) + (X)*kImage_PixelSize_(IMAGE)))
#define kxImage_As_(IMAGE, X, Y, TYPE)     (*(TYPE*) kImage_At_(IMAGE, X, Y))
#define kxImage_RowAt_(IMAGE, Y)           ((void*)((kByte*)kImage_Data_(IMAGE) + (Y)*kImage_Stride_(IMAGE)))
#define kxImage_PixelType_(IMAGE)          (kImage_(IMAGE)->pixelType)
#define kxImage_PixelSize_(IMAGE)          (kImage_(IMAGE)->pixelSize)
#define kxImage_Width_(IMAGE)              (kImage_(IMAGE)->width)
#define kxImage_Height_(IMAGE)             (kImage_(IMAGE)->height)
#define kxImage_Pitch_(IMAGE)              (kImage_(IMAGE)->stride / kImage_(IMAGE)->pixelSize)
#define kxImage_Area_(IMAGE)               (kImage_Width_(IMAGE) * kImage_Height_(IMAGE))
#define kxImage_Stride_(IMAGE)             (kImage_(IMAGE)->stride)
#define kxImage_PixelFormat_(IMAGE)        (kImage_(IMAGE)->format)
#define kxImage_Cfa_(IMAGE)                (kImage_(IMAGE)->cfa)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_IMAGE                  kTypeOf(kImage)
#   define kImage_Destroy               kObject_Destroy
#   define kImage_Save                  kImage_Export
#   define kImage_Alloc                 kImage_Allocate
#   define kIMAGE_DATA                  kImage_Data_
#   define kIMAGE_AT                    kImage_At_
#   define kIMAGE_ROW_AT                kImage_RowAt_
#   define kIMAGE_PIXEL_TYPE            kImage_PixelType_
#   define kIMAGE_PIXEL_SIZE            kImage_PixelSize_
#   define kIMAGE_WIDTH                 kImage_Width_
#   define kIMAGE_HEIGHT                kImage_Height_
#   define kIMAGE_STRIDE                kImage_Stride_
#   define kIMAGE_CFA                   kImage_Cfa_
#   define kIMAGE_PITCH                 kImage_Pitch_
#   define kIMAGE_PITCH_BYTES           kImage_Stride_
#   define kIMAGE_AREA                  kImage_Area_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kImage_Construct5(I, T, W, H)        kImage_Construct(I, T, W, H, kNULL)
#   define kImage_Import5(I, F)                 kImage_Import(I, F, kNULL)
#   define kIMAGE_WIDTH_BYTES(IMAGE)            (kImage_Width_(IMAGE) * kImage_PixelSize_(IMAGE))
#   define kIMAGE_AREA_BYTES(IMAGE)             (kImage_Area_(IMAGE) * kImage_PixelSize_(IMAGE))
#   define kIMAGE_IS_WHOLE(IMAGE)               (kIMAGE_WIDTH_BYTES(IMAGE) == kImage_Stride_(IMAGE))

#endif

kEndHeader()

#endif
