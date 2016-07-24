/** 
 * @file    kImage.c
 *
 * @internal
 * Copyright (C) 2003-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kImage.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kDat5Serializer.h>

kBeginClass(k, kImage, kObject) 
    
    //serialization versions
    kAddVersion(kImage, "kdat5", "5.0.0.0", "31-3", WriteDat5V3, ReadDat5V3)
    kAddVersion(kImage, "kdat6", "5.7.1.0", "kImage-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(kImage, kObject, VRelease)
    kAddVMethod(kImage, kObject, VInitClone)
    kAddVMethod(kImage, kObject, VSize)

kEndClass() 

kFx(kStatus) kImage_Construct(kImage* image, kType pixelType, kSize width, kSize height, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject_(alloc, kTypeOf(kImage), image)); 

    if (!kSuccess(status = kImage_Init(*image, kTypeOf(kImage), pixelType, width, height, alloc)))
    {
        kAlloc_FreeRef(alloc, image); 
    }

    return status; 
} 

kFx(kStatus) kImage_Import(kImage *image, const char* fileName, kAlloc allocator)
{
    return kBmpLoad(image, fileName, allocator); 
}

kFx(kStatus) kImage_Export(kImage image, const char* fileName)
{
    return kBmpSave(image, fileName); 
}

kFx(kStatus) kImage_Init(kImage image, kType classType, kType pixelType, kSize width, kSize height, kAlloc allocator)
{
    kImageClass* obj = image; 
    kStatus status = kOK; 

    kCheckArgs(kIsNull(pixelType) || kType_IsValue_(pixelType)); 

    kCheck(kObject_Init_(image, classType, allocator)); 

    obj->pixelType = kIsNull(pixelType) ? kTypeOf(kVoid) : pixelType; 
    obj->pixelSize = kType_Size_(obj->pixelType); 
    obj->allocSize = 0; 
    obj->pixels = kNULL;
    obj->stride = 0; 
    obj->width = 0; 
    obj->height = 0; 
    obj->isAttached = kFALSE; 
    obj->format = kPIXEL_FORMAT_NULL; 
    obj->cfa = kCFA_NONE; 

    kTry
    {
        kTest(kImage_Realloc(image, width, height));  
    }
    kCatch(&status)
    {
        kImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kImage_VInitClone(kImage image, kImage source, kAlloc allocator)
{
    kImageClass* obj = image; 
    kImageClass* srcObj = source; 
    kStatus status = kOK; 

    kCheck(kObject_Init_(image, kObject_Type_(source), allocator));     

    obj->pixelType = srcObj->pixelType; 
    obj->pixelSize = srcObj->pixelSize; 
    obj->allocSize = 0; 
    obj->pixels = kNULL;
    obj->stride = 0;
    obj->width = 0; 
    obj->height = 0; 
    obj->isAttached = kFALSE; 
    obj->format = srcObj->format; 
    obj->cfa = srcObj->cfa; 

    kTry
    {
        kTest(kImage_Realloc(image, srcObj->width, srcObj->height));  
        kTest(kCopyContent(kTypeOf(kByte), obj->pixels, srcObj->pixels, obj->stride*obj->height)); 
    }
    kCatch(&status)
    {
        kImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kImage_VRelease(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMem_(image, obj->pixels)); 
    }

    kCheck(kObject_VRelease_(image)); 

    return kOK; 
}

kFx(kSize) kImage_VSize(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kImage_DataSize_(image); 
    kSize size = sizeof(kImageClass) + dataSize; 

    return size; 
}

kFx(kStatus) kImage_WriteDat5V3(kImage image, kSerializer serializer)
{
    kImageClass* obj = kImage_Cast_(image); 
    kTypeVersion itemVersion; 
    kSize i; 
    
    kCheck(kSerializer_WriteSize_(serializer, obj->width)); 
    kCheck(kSerializer_WriteSize_(serializer, obj->height)); 
    kCheck(kSerializer_Write32s_(serializer, obj->cfa)); 

    //works around a bug/limitation in serialization version 5.3 -- pixel type version not written
    kCheck(kObject_Is(serializer, kTypeOf(kDat5Serializer))); 
    kCheck(kDat5Serializer_WriteTypeWithoutVersion(serializer, obj->pixelType, &itemVersion));

    for (i = 0; i < obj->height; ++i)
    {
        void* row = kImage_RowAt_(image, i); 

        kCheck(kSerializer_WriteSize_(serializer, obj->width)); 
        kCheck(kSerializer_WriteType_(serializer, obj->pixelType, &itemVersion));
        kCheck(kSerializer_WriteItems_(serializer, obj->pixelType, itemVersion, row, obj->width)); 
    }

    return kOK; 
}

kFx(kStatus) kImage_ReadDat5V3(kImage image, kSerializer serializer, kAlloc allocator)
{
    kImageClass *obj = image; 
    kSize width = 0, height = 0; 
    k32s cfa; 
    kTypeVersion itemVersion;
    kType pixelType = kNULL;            
    kStatus status; 
    kSize i; 

    kCheck(kSerializer_ReadSize_(serializer, &width)); 
    kCheck(kSerializer_ReadSize_(serializer, &height)); 
    kCheck(kSerializer_Read32s_(serializer, &cfa)); 

    //works around a bug/limitation in serialization version 5.3 -- pixel type version not written
    kCheck(kObject_Is(serializer, kTypeOf(kDat5Serializer))); 
    kCheck(kDat5Serializer_ReadTypeExplicitVersion(serializer, 0, &pixelType, &itemVersion)); 

    kCheck(kImage_Init(image, kTypeOf(kImage), pixelType, width, height, allocator)); 

    kTry
    {         
        obj->cfa = cfa; 

        for (i = 0; i < obj->height; ++i)
        {
            void* row = kImage_RowAt_(image, i); 

            kTest(kSerializer_ReadSize_(serializer, &width)); 
            kTest(kSerializer_ReadType_(serializer, &pixelType, &itemVersion));
            kTest(kSerializer_ReadItems_(serializer, pixelType, itemVersion, row, obj->width)); 
        }
    }
    kCatch(&status)
    {
        kImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kImage_WriteDat6V0(kImage image, kSerializer serializer)
{
    kImageClass* obj = kImage_Cast_(image); 
    kTypeVersion itemVersion; 
    kSize i; 

    kCheck(kSerializer_WriteType_(serializer, obj->pixelType, &itemVersion));
    kCheck(kSerializer_WriteSize_(serializer, obj->width)); 
    kCheck(kSerializer_WriteSize_(serializer, obj->height));
    kCheck(kSerializer_Write32s_(serializer, obj->cfa)); 
    kCheck(kSerializer_Write32s_(serializer, obj->format)); 

    //reserve space for optional fields
    kCheck(kSerializer_Write16u_(serializer, 0)); 

    if ((obj->width*obj->pixelSize) == obj->stride)
    {
		kCheck(kSerializer_WriteItems(serializer, obj->pixelType, itemVersion, obj->pixels, obj->width*obj->height));     	
    }
    else
    {
    	for (i = 0; i < obj->height; ++i)
    	{
    		void* row = kImage_RowAt_(image, i); 

    		kCheck(kSerializer_WriteItems_(serializer, obj->pixelType, itemVersion, row, obj->width)); 
    	}
    }

    return kOK; 
}

kFx(kStatus) kImage_ReadDat6V0(kImage image, kSerializer serializer, kAlloc allocator)
{
    kImageClass *obj = image; 
    kType pixelType = kNULL;            
    kTypeVersion itemVersion; 
    kSize width = 0, height = 0; 
    k16u optional = 0; 
    kStatus status; 
    kSize i; 

    kCheck(kSerializer_ReadType_(serializer, &pixelType, &itemVersion));
    kCheck(kSerializer_ReadSize_(serializer, &width)); 
    kCheck(kSerializer_ReadSize_(serializer, &height)); 

    kCheck(kImage_Init(image, kTypeOf(kImage), pixelType, width, height, allocator)); 

    kTry
    {
        kTest(kSerializer_Read32s_(serializer, &obj->cfa)); 
        kTest(kSerializer_Read32s_(serializer, &obj->format)); 

        //consume optional fields
        kTest(kSerializer_Read16u_(serializer, &optional)); 
        kTest(kSerializer_AdvanceRead_(serializer, optional)); 

        if ((obj->width*obj->pixelSize) == obj->stride)
        {
            kTest(kSerializer_ReadItems(serializer, pixelType, itemVersion, obj->pixels, obj->width*obj->height));
        }
        else
        {
            for (i = 0; i < obj->height; ++i)
            {
                void* row = kImage_RowAt_(image, i);

                kTest(kSerializer_ReadItems_(serializer, pixelType, itemVersion, row, obj->width));
            }
        }
    }
    kCatch(&status)
    {
        kImage_VRelease(image); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kImage_Realloc(kImage image, kSize width, kSize height)
{
    kImageClass* obj = kImage_Cast_(image); 
    kSize stride = kAlign_(width * obj->pixelSize, kIMAGE_ALIGNMENT); 
    kSize newSize = kMax_(obj->allocSize, height*stride); 

    if (newSize > obj->allocSize)
    {
        void* oldPixels = obj->pixels; 
        void* newPixels = kNULL; 

        kCheck(kObject_GetMem_(image, newSize, &newPixels));         
        
        obj->pixels = newPixels; 
        obj->allocSize = newSize; 

        kCheck(kObject_FreeMem_(image, oldPixels)); 
    }
        
    obj->width = width; 
    obj->height = height; 
    obj->stride = stride; 
    
    return kOK; 
}

kFx(kStatus) kImage_Allocate(kImage image, kType pixelType, kSize width, kSize height)
{
    kImageClass* obj = kImage_Cast_(image); 

    if (obj->isAttached)
    {
        obj->pixels = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->pixelType = kIsNull(pixelType) ? kTypeOf(kVoid) : pixelType; 
    obj->pixelSize = kType_Size_(obj->pixelType); 
    obj->stride = 0; 
    obj->width = 0; 
    obj->height = 0; 

    kCheck(kImage_Realloc(image, width, height));  

    return kOK; 
}

kFx(kStatus) kImage_Attach(kImage image, void* pixels, kType pixelType, kSize width, kSize height, kSize stride)
{
    kImageClass* obj = kImage_Cast_(image); 

    if (!obj->isAttached)
    {
        kCheck(kObject_FreeMemRef_(image, &obj->pixels)); 
    }
    
    obj->pixelType = kIsNull(pixelType) ? kTypeOf(kVoid) : pixelType; 
    obj->pixelSize = kType_Size_(obj->pixelType); 
    obj->allocSize = 0; 
    obj->pixels = pixels; 
    obj->stride = stride; 
    obj->width = width; 
    obj->height = height; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) kImage_Assign(kImage image, kImage source)
{
    kImageClass* obj = kImage_Cast_(image); 
    kImageClass* srcObj = kImage_Cast_(source); 
    kSize i; 

    kCheck(kImage_Allocate(image, srcObj->pixelType, srcObj->width, srcObj->height)); 

    for (i = 0; i < obj->height; ++i)
    {
        void* row = kImage_RowAt_(image, i); 
        void* srcRow = kImage_RowAt_(source, i); 

        kCheck(kCopyContent(obj->pixelType, row, srcRow, obj->width)); 
    }

    return kOK;   
}

kFx(kStatus) kImage_Zero(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    kCheck(kZeroContent(kTypeOf(kByte), obj->pixels, kImage_DataSize_(image)));

    return kOK; 
}

kFx(kStatus) kImage_SetPixelFormat(kImage image, kPixelFormat format)
{
    kImageClass* obj = kImage_Cast_(image); 
    
    obj->format = format; 

    return kOK; 
}

kFx(kPixelFormat) kImage_PixelFormat(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_PixelFormat_(obj); 
}

kFx(kStatus) kImage_SetCfa(kImage image, kCfa cfa)
{
    kImageClass* obj = kImage_Cast_(image); 

    obj->cfa = cfa; 

    return kOK; 
}

kFx(kCfa) kImage_Cfa(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_Cfa_(obj); 
}

kFx(kStatus) kImage_SetPixel(kImage image, kSize x, kSize y, const void* pixel)
{
    kImageClass* obj = kImage_Cast_(image); 

    kCheckArgs((x < obj->width) && (y < obj->height)); 

    kItemImport_(kImage_At_(image, x, y), pixel, obj->pixelType); 

    return kOK; 
}

kFx(kStatus) kImage_Pixel(kImage image, kSize x, kSize y, void* pixel)
{
    kImageClass* obj = kImage_Cast_(image); 

    kCheckArgs((x < obj->width) && (y < obj->height)); 

    kItemCopy_(pixel, kImage_At_(image, x, y), obj->pixelSize); 

    return kOK; 
}

kFx(void*) kImage_Data(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_Data_(obj); 
}

kFx(kSize) kImage_DataSize(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_DataSize_(obj); 
}

kFx(void*) kImage_At(kImage image, kSize x, kSize y)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_At_(obj, x, y); 
}

kFx(void*) kImage_RowAt(kImage image, kSize y)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_RowAt_(obj, y); 
}

kFx(kType) kImage_PixelType(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_PixelType_(obj); 
}

kFx(kSize) kImage_PixelSize(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_PixelSize_(obj); 
}

kFx(kSize) kImage_Width(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_Width_(obj); 
}

kFx(kSize) kImage_Height(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_Height_(obj); 
}

kFx(kSize) kImage_Area(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_Area_(obj); 
}

kFx(kSize) kImage_Stride(kImage image)
{
    kImageClass* obj = kImage_Cast_(image); 

    return kImage_Stride_(obj); 
}
