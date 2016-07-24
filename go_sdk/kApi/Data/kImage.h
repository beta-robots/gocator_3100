/** 
 * @file    kImage.h
 * @brief   Declares the kImage class. 
 *
 * @internal
 * Copyright (C) 2003-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_IMAGE_H
#define K_API_IMAGE_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kImage
 * @extends kObject
 * @ingroup kApi-Data
 * @brief   Represents a 2D collection of pixels.
 * 
 * The kImage class is similar to the kArray2 class, with some important differences:
 * - By default, image memory is allocated such that rows are aligned to 8-byte boundaries. 
 * - Pixels are limited to value types. 
 * - Image indices/dimensions are given in (column, row) rather than (row, column) order.  
 * - kImage contains additional image-specific attributes (e.g. color filter array type). 
 * 
 * In particular, the row-alignment behaviour means that image memory should typically be accessed
 * by getting a pointer to the beginning of a row, and then iterating over pixels. E.g. 
 * 
 * @code {.c}
 * kSize height = kImage_Height(image); 
 * kSize width = kImage_Width(image); 
 * kSize i, j; 
 * k32u sum = 0; 
 * 
 * kAssert(kImage_PixelType(image, kTypeOf(k8u))); 
 * 
 * for (i = 0; i < height; ++i)
 * {
 *     k8u* row = kImage_RowAt(image, i); 
 * 
 *     for (j = 0; j < width; ++j)
 *     {
 *         sum += row[j]; 
 *     }
 * }
 *
 * @endcode
 *
 * kImage supports the kObject_Clone and kObject_Size methods.
 *
 * kImage supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kImage; --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kImage object.
 *
 * @public              @memberof kImage
 * @param   image       Receives the constructed image object. 
 * @param   pixelType   Pixel type (must be a value type). 
 * @param   width       Image width. 
 * @param   height      Image height. 
 * @param   allocator   Memory allocator. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Construct(kImage* image, kType pixelType, kSize width, kSize height, kAlloc allocator); 

/** 
 * Loads an image from file.
 * 
 * The file extension is used to determine to image format. Currently, only the BMP (.bmp) format is supported.
 *
 * @public              @memberof kImage
 * @param   image       Receives the constructed image object. 
 * @param   fileName    File path. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Import(kImage *image, const char* fileName, kAlloc allocator); 

/** 
 * Saves an image to file.
 * 
 * The file extension is used to determine to image format. Currently, only the BMP (.bmp) format is supported.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   fileName    File path. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Export(kImage image, const char* fileName); 

/** 
 * Reallocates the internal pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   pixelType   Type of pixel (values types only). 
 * @param   width       Image width, in pixels. 
 * @param   height      Image height, in pixels. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Allocate(kImage image, kType pixelType, kSize width, kSize height); 

/** 
 * Attaches the image to an external pixel buffer. 
 *
 * Attached pixel buffers are not freed when the image is destroyed. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   pixels      Pointer to external pixel buffer. 
 * @param   pixelType   Type of pixel (values types only). 
 * @param   width       Image width, in pixels. 
 * @param   height      Image height, in pixels. 
 * @param   stride      Image stride (row size), in bytes. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Attach(kImage image, void* pixels, kType pixelType, kSize width, kSize height, kSize stride); 

/** 
 * Copies a given source image into this image. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   source      Source image to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Assign(kImage image, kImage source); 

/** 
 * Sets all pixel bits to zero.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Zero(kImage image); 

/** 
 * Sets the optional pixel format descriptor associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   format      Pixel format.  
 * @return              Operation status. 
 */
kFx(kStatus) kImage_SetPixelFormat(kImage image, kPixelFormat format); 

/** 
 * Gets the optional pixel format descriptor associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pixel format. 
 */
kFx(kPixelFormat) kImage_PixelFormat(kImage image);

/** 
 * Sets the color filter array type associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   cfa         Color filter array type.  
 * @return              Operation status. 
 */
kFx(kStatus) kImage_SetCfa(kImage image, kCfa cfa); 

/** 
 * Gets the color filter array type associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Color filter array type.  
 */
kFx(kCfa) kImage_Cfa(kImage image);

/** 
 * Sets the value of a pixel. 
 * 
 * This method is convenient in some contexts, but is not an efficient method to manipulate
 * many pixels.  In performance-critical code, use kImage_RowAt to directly access the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @param   pixel       Pointer to pixel value that will be copied into the image.
 * @return              Operation status. 
 */
kFx(kStatus) kImage_SetPixel(kImage image, kSize x, kSize y, const void* pixel); 

/** 
 * Gets the value of a pixel. 
 * 
 * This method is convenient in some contexts, but is not an efficient method to access
 * many pixels.  In performance-critical code, use kImage_RowAt to directly access the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @param   pixel       Destination for pixel copied from the image. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Pixel(kImage image, kSize x, kSize y, void* pixel); 

/** 
 * Returns a pointer to the first row in the pixel buffer.  
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pointer to pixel buffer. 
 */
kFx(void*) kImage_Data(kImage image); 

/** 
 * Reports the size, in bytes, of the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Size of pixel buffer (bytes). 
 */
kFx(kSize) kImage_DataSize(kImage image); 

/** 
 * Returns a pointer to the specified pixel in the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @return              Pointer to pixel. 
 */
kFx(void*) kImage_At(kImage image, kSize x, kSize y); 

/** 
 * Returns a pointer to the specified row in the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   y           Row index.  
 * @return              Pointer to first pixel in row. 
 */
kFx(void*) kImage_RowAt(kImage image, kSize y); 

/** 
 * Returns the pixel type. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pixel type. 
 */
kFx(kType) kImage_PixelType(kImage image); 

/** 
 * Returns the pixel size. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pixel size, in bytes. 
 */
kFx(kSize) kImage_PixelSize(kImage image); 

/** 
 * Returns the width of the image, in pixels. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image width (pixels). 
 */
kFx(kSize) kImage_Width(kImage image); 

/** 
 * Returns the height of the image, in pixels. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image height (pixels). 
 */
kFx(kSize) kImage_Height(kImage image); 

/** 
 * Returns the area of the image, in pixels. 
 * 
 * Image area is the product of image width and image height. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image area (pixels). 
 */
kFx(kSize) kImage_Area(kImage image); 

/** 
 * Returns the size of an image row, including alignment padding bytes, in bytes. 
 * 
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image stride (bytes). 
 */
kFx(kSize) kImage_Stride(kImage image); 

/** @relates kImage @{ */

#define kImage_Data_(IMAGE)                     kxImage_Data_(IMAGE)                ///< Macro version of kImage_Data.
#define kImage_DataSize_(IMAGE)                 kxImage_DataSize_(IMAGE)            ///< Macro version of kImage_DataSize.            
#define kImage_At_(IMAGE, X, Y)                 kxImage_At_(IMAGE, X, Y)            ///< Macro version of kImage_At.
#define kImage_RowAt_(IMAGE, Y)                 kxImage_RowAt_(IMAGE, Y)            ///< Macro version of kImage_RowAt.   
#define kImage_PixelType_(IMAGE)                kxImage_PixelType_(IMAGE)           ///< Macro version of kImage_PixelType.        
#define kImage_PixelSize_(IMAGE)                kxImage_PixelSize_(IMAGE)           ///< Macro version of kImage_PixelSize.        
#define kImage_Width_(IMAGE)                    kxImage_Width_(IMAGE)               ///< Macro version of kImage_Width.        
#define kImage_Height_(IMAGE)                   kxImage_Height_(IMAGE)              ///< Macro version of kImage_Height.        
#define kImage_Area_(IMAGE)                     kxImage_Area_(IMAGE)                ///< Macro version of kImage_Area.      
#define kImage_Stride_(IMAGE)                   kxImage_Stride_(IMAGE)              ///< Macro version of kImage_Stride.        
#define kImage_PixelFormat_(IMAGE)              kxImage_PixelFormat_(IMAGE)         ///< Macro version of kImage_PixelFormat.        
#define kImage_Cfa_(IMAGE)                      kxImage_Cfa_(IMAGE)                 ///< Macro version of kImage_Cfa.        

/** Accesses a pixel at the specified location, and casts the value to the specified type.  */
#define kImage_As_(IMAGE, X, Y, TYPE)           kxImage_As_(IMAGE, X, Y, TYPE)  

/** @} */

kEndHeader()

#include <kApi/Data/kImage.x.h>

#endif
