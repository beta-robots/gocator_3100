/** 
 * @file    GoDataTypes.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoDataTypes.h>
#include <GoSdk/Messages/GoHealth.h>

kBeginClass(Go, GoDataMsg, kObject)
    kAddVMethod(GoDataMsg, GoDataMsg, VInit)
    kAddVMethod(GoDataMsg, kObject, VInitClone)
    kAddVMethod(GoDataMsg, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoDataMsg_VInit(GoDataMsg msg, kType type, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED;
}

GoFx(kStatus) GoDataMsg_Init(GoDataMsg msg, kType type, GoDataMessageType typeId, kAlloc alloc)
{
    GoDataMsgClass* obj = msg; 

    kCheck(kObject_Init_(msg, type, alloc)); 
    kInitFields_(GoDataMsg, msg);

    obj->typeId = typeId; 

    return kOK; 
}

GoFx(kStatus) GoDataMsg_VInitClone(GoDataMsg msg, GoDataMsg source, kAlloc alloc)
{
    GoDataMsgClass* obj = GoDataMsg_Cast_(msg); 
    GoDataMsgClass* src = GoDataMsg_Cast_(source); 
    
    kCheck(GoDataMsg_Init(msg, kObject_Type_(source), src->typeId, alloc));     

    return kOK;
}

GoFx(kStatus) GoDataMsg_VRelease(GoDataMsg messsage)
{
    GoDataMsgClass * obj = GoDataMsg_Cast_(messsage); 

    return kObject_VRelease(messsage);
}

GoFx(GoDataMessageType) GoDataMsg_Type(GoDataMsg message)
{
    GoDataMsgClass* obj = GoDataMsg_Cast_(message); 

    return obj->typeId;
}


/* 
 * GoStamp
 */
kBeginValue(Go, GoStamp, kValue) 
    kAddField(GoStamp, k64u, frameIndex)
    kAddField(GoStamp, k64u, timestamp)
    kAddField(GoStamp, k64s, encoder)
    kAddField(GoStamp, k64s, encoderAtZ)
    kAddField(GoStamp, k64u, status)
    kAddField(GoStamp, k32u, id)
    kAddField(GoStamp, k32u, reserved32u)
    kAddField(GoStamp, k64u, reserved64u)
kEndValue() 

/* 
 * GoStampMsg
 */

kBeginClass(Go, GoStampMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoStampMsg, "godat", "6.0.0.0", "1", WriteV1, ReadV1)
    kAddVersion(GoStampMsg, "kdat6", "4.0.0.0", "GoStampMsg-1", WriteV1, ReadV1)

    //virtual methods
    kAddVMethod(GoStampMsg, GoDataMsg, VInit)
    kAddVMethod(GoStampMsg, kObject, VInitClone)
    kAddVMethod(GoStampMsg, kObject, VRelease)
    kAddVMethod(GoStampMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoStampMsg_Construct(GoStampMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoStampMsg), msg)); 

    if (!kSuccess(status = GoStampMsg_VInit(*msg, kTypeOf(GoStampMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoStampMsg_VInit(GoStampMsg msg, kType type, kAlloc alloc)
{
    GoStampMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_STAMP, alloc)); 

    obj->stamps = kNULL; 
    obj->source = GO_DATA_SOURCE_TOP; 
    
    return kOK; 
}

GoFx(kStatus) GoStampMsg_VInitClone(GoStampMsg msg, GoStampMsg source, kAlloc alloc)
{
    GoStampMsgClass* obj = msg; 
    GoStampMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoStampMsg_VInit(msg, kObject_Type_(source), alloc));     

    kTry
    {
        obj->source = srcObj->source;
        
        kTest(kObject_Clone(&obj->stamps, srcObj->stamps, alloc)); 
    }
    kCatch(&status)
    {
        GoStampMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoStampMsg_Allocate(GoStampMsg msg, kSize count)
{
    GoStampMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->stamps)); 
    kCheck(kArray1_Construct(&obj->stamps, kTypeOf(GoStamp), count, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kStatus) GoStampMsg_VRelease(GoStampMsg msg)
{
    GoStampMsgClass* obj = msg; 
    
    kCheck(kObject_Destroy(obj->stamps)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoStampMsg_VSize(GoStampMsg msg)
{
    GoStampMsgClass* obj = msg; 

    return sizeof(GoStampMsgClass) + (kIsNull(obj->stamps) ? 0 : kObject_Size_(obj->stamps)); 
}

GoFx(kStatus) GoStampMsg_WriteV1(GoStampMsg msg, kSerializer serializer)
{
    GoStampMsgClass* obj = msg; 
    k32u count = (k32u) kArray1_Count_(obj->stamps); 
    k32u i; 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write16u_(serializer, GO_STAMP_MSG_STAMP_SIZE_1_56)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source));  
    kCheck(kSerializer_Write8u_(serializer, 0)); 

    for (i = 0; i < count; ++i)
    {
        const GoStamp* stamp = kArray1_At_(obj->stamps, i); 

        kCheck(kSerializer_Write64u_(serializer, stamp->frameIndex)); 
        kCheck(kSerializer_Write64u_(serializer, stamp->timestamp)); 
        kCheck(kSerializer_Write64s_(serializer, stamp->encoder)); 
        kCheck(kSerializer_Write64s_(serializer, stamp->encoderAtZ));
        kCheck(kSerializer_Write64u_(serializer, stamp->status)); 
        kCheck(kSerializer_Write32u_(serializer, stamp->id)); 
        kCheck(kSerializer_Write32u_(serializer, stamp->reserved32u)); 
        kCheck(kSerializer_Write64u_(serializer, stamp->reserved64u)); 
    }

    return kOK; 
}

GoFx(kStatus) GoStampMsg_ReadV1(GoStampMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoStampMsgClass* obj = msg; 
    kStatus status;
    k16u stampSize = 0; 
    k8u source = 0; 
    k8u reserved; 
    k32u count = 0; 
    k32u i; 

    kCheck(GoStampMsg_VInit(msg, kTypeOf(GoStampMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read16u_(serializer, &stampSize)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
            
        obj->source = (GoDataSource)source; 

        kTest(kArray1_Construct(&obj->stamps, kTypeOf(GoStamp), count, alloc)); 

        for (i = 0; i < count; ++i)
        {
            const GoStamp* stamp = kArray1_At_(obj->stamps, i); 

            kTest(kSerializer_Read64u_(serializer, &stamp->frameIndex)); 
            kTest(kSerializer_Read64u_(serializer, &stamp->timestamp)); 
            kTest(kSerializer_Read64s_(serializer, &stamp->encoder)); 
            kTest(kSerializer_Read64s_(serializer, &stamp->encoderAtZ)); 
            kTest(kSerializer_Read64u_(serializer, &stamp->status)); 
            kTest(kSerializer_Read32u_(serializer, &stamp->id)); 
            kTest(kSerializer_Read32u_(serializer, &stamp->reserved32u)); 
            kTest(kSerializer_Read64u_(serializer, &stamp->reserved64u)); 

            kTest(kSerializer_AdvanceRead_(serializer, stampSize - GO_STAMP_MSG_STAMP_SIZE_1_56)); 
        } 
    }
    kCatch(&status)
    {
        GoStampMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoStampMsg_Source(GoStampMsg msg)
{
    return GoStampMsg_(msg)->source; 
}

GoFx(kSize) GoStampMsg_Count(GoStampMsg msg)
{
    return kArray1_Count_(GoStampMsg_Content_(msg)); 
}

GoFx(GoStamp*) GoStampMsg_At(GoStampMsg msg, kSize index)
{
    kAssert(index < kArray1_Count(GoStampMsg_Content_(msg)));

    return kArrayList_At_(GoStampMsg_Content_(msg), index); 
}


/* 
 * GoVideoMsg
 */

kBeginClass(Go, GoVideoMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoVideoMsg, "godat", "6.0.0.0", "2", WriteV2, ReadV2)
    kAddVersion(GoVideoMsg, "kdat6", "4.0.0.0", "GoVideoMsg-2", WriteV2, ReadV2)

    //virtual methods
    kAddVMethod(GoVideoMsg, GoDataMsg, VInit)
    kAddVMethod(GoVideoMsg, kObject, VInitClone)
    kAddVMethod(GoVideoMsg, kObject, VRelease)
    kAddVMethod(GoVideoMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoVideoMsg_Construct(GoVideoMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoVideoMsg), msg)); 

    if (!kSuccess(status = GoVideoMsg_VInit(*msg, kTypeOf(GoVideoMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoVideoMsg_VInit(GoVideoMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_VIDEO, alloc)); 

    kZeroDerivedFields_(GoVideoMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoVideoMsg_VInitClone(GoVideoMsg msg, GoVideoMsg source, kAlloc alloc)
{
    GoVideoMsgClass* obj = msg; 
    GoVideoMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoVideoMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->exposureIndex = srcObj->exposureIndex;
        obj->exposure = srcObj->exposure;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoVideoMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoVideoMsg_Allocate(GoVideoMsg msg, kType pixelType, kSize width, kSize height)
{
    GoVideoMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kImage_Construct(&obj->content, pixelType, width, height, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoVideoMsg_VSize(GoVideoMsg msg)
{
    GoVideoMsgClass* obj = msg; 

    return sizeof(GoVideoMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoVideoMsg_VRelease(GoVideoMsg msg)
{
    GoVideoMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoVideoMsg_WriteV2(GoVideoMsg msg, kSerializer serializer)
{
    GoVideoMsgClass* obj = msg; 
    k32u width = (k32u) kImage_Width_(obj->content); 
    k32u height = (k32u) kImage_Height_(obj->content); 
    k32u i; 
    kSize rowSize;

    kCheck(kSerializer_Write16u_(serializer, GO_VIDEO_MSG_ATTR_2_20)); 

    kCheck(kSerializer_Write32u_(serializer, height)); 
    kCheck(kSerializer_Write32u_(serializer, width)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u)kImage_PixelSize_(obj->content))); 
    kCheck(kSerializer_Write8u_(serializer, (k8u)kImage_PixelFormat_(obj->content))); 
    kCheck(kSerializer_Write8u_(serializer, (k8u)kImage_Cfa_(obj->content))); 
    kCheck(kSerializer_Write8u_(serializer, (k8u)obj->source)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u)obj->cameraIndex)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u)obj->exposureIndex)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 

    if (kImage_PixelType_(obj->content) == kTypeOf(k8u))
    {
        rowSize = width * kImage_PixelSize_(obj->content);

        for (i = 0; i < height; ++i)
        {
            void* pixels = kImage_RowAt_(obj->content, i); 
            kCheck(kSerializer_WriteByteArray_(serializer, pixels, rowSize)); 
        }
    }
    else if (kImage_PixelType_(obj->content) == kTypeOf(kRgb))
    {
        for (i = 0; i < height; ++i)
        {
            void* pixels = kImage_RowAt_(obj->content, i); 
            kCheck(kSerializer_Write8uArray_(serializer, pixels, 4*width)); 
        }
    }
    else
    {
        return kERROR_UNIMPLEMENTED; 
    }
    
    return kOK; 
}

GoFx(kStatus) GoVideoMsg_ReadV2(GoVideoMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoVideoMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u width, height; 
    k8u pixelSize, pixelFormat, colorMask, source, cameraIndex, exposureIndex, reserved; 
    k32u i, exposure;
    kSize rowSize;

    kCheck(GoVideoMsg_VInit(msg, kTypeOf(GoVideoMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &height)); 
        kTest(kSerializer_Read32u_(serializer, &width)); 
        kTest(kSerializer_Read8u_(serializer, &pixelSize)); 
        kTest(kSerializer_Read8u_(serializer, &pixelFormat)); 
        kTest(kSerializer_Read8u_(serializer, &colorMask)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read8u_(serializer, &cameraIndex)); 
        kTest(kSerializer_Read8u_(serializer, &exposureIndex)); 
        kTest(kSerializer_Read32u_(serializer, &exposure)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_VIDEO_MSG_ATTR_2_20)); 

        if ((pixelFormat == kPIXEL_FORMAT_8BPP_GREYSCALE) || (pixelFormat == kPIXEL_FORMAT_8BPP_CFA))
        {
            kTest(kImage_Construct(&obj->content, kTypeOf(k8u), width, height, alloc)); 

            rowSize = pixelSize * width;

            for (i = 0; i < height; ++i)
            {
                void* pixels = kImage_RowAt_(obj->content, i); 
                kTest(kSerializer_ReadByteArray_(serializer, pixels, rowSize)); 
            }
        }
        else if (pixelFormat == kPIXEL_FORMAT_8BPC_BGRX)
        {
            kTest(kImage_Construct(&obj->content, kTypeOf(kRgb), width, height, alloc)); 

            for (i = 0; i < height; ++i)
            {
                void* pixels = kImage_RowAt_(obj->content, i); 
                kTest(kSerializer_Read8uArray_(serializer, pixels, 4*width)); 
            }
        }
        else
        {
            kThrow(kERROR_UNIMPLEMENTED); 
        }

        kTest(kImage_SetPixelFormat(obj->content, (kPixelFormat)pixelFormat)); 
        kTest(kImage_SetCfa(obj->content, (kCfa)colorMask));         
        obj->cameraIndex = cameraIndex;
        obj->source = (GoDataSource)source; 
        obj->exposureIndex = exposureIndex;
        obj->exposure = exposure;
    } 
    kCatch(&status)
    {
        GoVideoMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoVideoMsg_Source(GoVideoMsg msg)
{
    return GoVideoMsg_(msg)->source; 
}

GoFx(kSize) GoVideoMsg_CameraIndex(GoVideoMsg msg)
{
    return GoVideoMsg_(msg)->cameraIndex; 
}

GoFx(kSize) GoVideoMsg_Width(GoVideoMsg msg)
{
    return kImage_Width_(GoVideoMsg_Content_(msg)); 
}

GoFx(kSize) GoVideoMsg_Height(GoVideoMsg msg)
{
    return kImage_Height_(GoVideoMsg_Content_(msg)); 
}

GoFx(GoPixelType) GoVideoMsg_PixelType(GoVideoMsg msg)
{
    kType pixelType = kImage_PixelType_(GoVideoMsg_Content_(msg)); 
    GoPixelType typeToReturn;

    if (pixelType == kTypeOf(k8u))
    {
        typeToReturn = GO_PIXEL_TYPE_8U;    
    }
    else if (pixelType == kTypeOf(kRgb))
    {
        typeToReturn = GO_PIXEL_TYPE_RGB;
    }

    return typeToReturn;
}

GoFx(kSize) GoVideoMsg_PixelSize(GoVideoMsg msg)
{
    return kImage_PixelSize_(GoVideoMsg_Content_(msg)); 
}

GoFx(kPixelFormat) GoVideoMsg_PixelFormat(GoVideoMsg msg)
{
    return kImage_PixelFormat_(GoVideoMsg_Content_(msg)); 
}

GoFx(kCfa) GoVideoMsg_Cfa(GoVideoMsg msg)
{
    return kImage_Cfa_(GoVideoMsg_Content_(msg)); 
}

GoFx(void*) GoVideoMsg_RowAt(GoVideoMsg msg, kSize rowIndex)
{
    kAssert(rowIndex < GoVideoMsg_Height(msg));

    return kImage_RowAt_(GoVideoMsg_Content_(msg), rowIndex); 
}

GoFx(kSize) GoVideoMsg_ExposureIndex(GoVideoMsg msg)
{
    return GoVideoMsg_(msg)->exposureIndex; 
}

GoFx(k32u) GoVideoMsg_Exposure(GoVideoMsg msg)
{
    return GoVideoMsg_(msg)->exposure; 
}



/* 
 * GoRangeMsg
 */

kBeginClass(Go, GoRangeMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoRangeMsg, "godat", "6.0.0.0", "3", WriteV3, ReadV3)
    kAddVersion(GoRangeMsg, "kdat6", "4.0.0.0", "GoRangeMsg-3", WriteV3, ReadV3)

    //virtual methods
    kAddVMethod(GoRangeMsg, GoDataMsg, VInit)
    kAddVMethod(GoRangeMsg, kObject, VInitClone)
    kAddVMethod(GoRangeMsg, kObject, VRelease)
    kAddVMethod(GoRangeMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoRangeMsg_Construct(GoRangeMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoRangeMsg), msg)); 

    if (!kSuccess(status = GoRangeMsg_VInit(*msg, kTypeOf(GoRangeMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoRangeMsg_VInit(GoRangeMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_RANGE, alloc)); 

    kZeroDerivedFields_(GoRangeMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoRangeMsg_VInitClone(GoRangeMsg msg, GoRangeMsg source, kAlloc alloc)
{
    GoRangeMsgClass* obj = msg; 
    GoRangeMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoRangeMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->zResolution = srcObj->zResolution; 
        obj->zOffset = srcObj->zOffset; 
        obj->exposure = srcObj->exposure;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoRangeMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoRangeMsg_Allocate(GoRangeMsg msg, kSize count, kSize width)
{
    GoRangeMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray2_Construct(&obj->content, kTypeOf(k16s), count, width, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoRangeMsg_VSize(GoRangeMsg msg)
{
    GoRangeMsgClass* obj = msg; 

    return sizeof(GoRangeMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoRangeMsg_VRelease(GoRangeMsg msg)
{
    GoRangeMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoRangeMsg_WriteV3(GoRangeMsg msg, kSerializer serializer)
{
    GoRangeMsgClass* obj = msg; 
    k32u count = (k32u) kArray1_Length_(obj->content); 

    kCheck(kSerializer_Write16u_(serializer, GO_RANGE_MSG_ATTR_3_20)); 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write32u_(serializer, obj->zResolution)); 
    kCheck(kSerializer_Write32s_(serializer, obj->zOffset)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 

    kCheck(kSerializer_Write16sArray_(serializer, kArray1_Data_(obj->content), kArray1_Count_(obj->content))); 
    
    return kOK; 
}

GoFx(kStatus) GoRangeMsg_ReadV3(GoRangeMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoRangeMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u count; 
    k8u source, reserved;

    kCheck(GoRangeMsg_VInit(msg, kTypeOf(GoRangeMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read32u_(serializer, &obj->zResolution)); 
        kTest(kSerializer_Read32s_(serializer, &obj->zOffset)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_RANGE_MSG_ATTR_3_20)); 
 
        kTest(kArray1_Construct(&obj->content, kTypeOf(k16s), count, alloc)); 
        kTest(kSerializer_Read16sArray_(serializer, kArray1_Data_(obj->content), kArray1_Count_(obj->content))); 
    
        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoRangeMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoRangeMsg_Source(GoRangeMsg msg)
{
    return GoRangeMsg_(msg)->source; 
}

GoFx(kSize) GoRangeMsg_Count(GoRangeMsg msg)
{
    return kArray1_Length_(GoRangeMsg_Content_(msg)); 
}

GoFx(k32u) GoRangeMsg_ZResolution(GoRangeMsg msg)
{
    return GoRangeMsg_(msg)->zResolution; 
}

GoFx(k32s) GoRangeMsg_ZOffset(GoRangeMsg msg)
{
    return GoRangeMsg_(msg)->zOffset; 
}

GoFx(k16s*) GoRangeMsg_At(GoRangeMsg msg, kSize index)
{
    kAssert(index < GoRangeMsg_Count(msg));

    return kArray1_At_(GoRangeMsg_Content_(msg), index); 
}

GoFx(k32u) GoRangeMsg_Exposure(GoRangeMsg msg)
{
    return GoRangeMsg_(msg)->exposure; 
}

/* 
 * GoRangeIntensityMsg
 */

kBeginClass(Go, GoRangeIntensityMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoRangeIntensityMsg, "godat", "6.0.0.0", "4", WriteV4, ReadV4)
    kAddVersion(GoRangeIntensityMsg, "kdat6", "4.0.0.0", "GoRangeIntensityMsg-4", WriteV4, ReadV4)

    //virtual methods
    kAddVMethod(GoRangeIntensityMsg, GoDataMsg, VInit)
    kAddVMethod(GoRangeIntensityMsg, kObject, VInitClone)
    kAddVMethod(GoRangeIntensityMsg, kObject, VRelease)
    kAddVMethod(GoRangeIntensityMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoRangeIntensityMsg_Construct(GoRangeIntensityMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoRangeIntensityMsg), msg)); 

    if (!kSuccess(status = GoRangeIntensityMsg_VInit(*msg, kTypeOf(GoRangeIntensityMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoRangeIntensityMsg_VInit(GoRangeIntensityMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY, alloc)); 

    kZeroDerivedFields_(GoRangeIntensityMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoRangeIntensityMsg_VInitClone(GoRangeIntensityMsg msg, GoRangeIntensityMsg source, kAlloc alloc)
{
    GoRangeIntensityMsgClass* obj = msg; 
    GoRangeIntensityMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoRangeIntensityMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->exposure = srcObj->exposure;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoRangeIntensityMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoRangeIntensityMsg_Allocate(GoRangeIntensityMsg msg, kSize count)
{
    GoRangeIntensityMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray1_Construct(&obj->content, kTypeOf(k8u), count, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoRangeIntensityMsg_VSize(GoRangeIntensityMsg msg)
{
    GoRangeIntensityMsgClass* obj = msg; 

    return sizeof(GoRangeIntensityMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoRangeIntensityMsg_VRelease(GoRangeIntensityMsg msg)
{
    GoRangeIntensityMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoRangeIntensityMsg_WriteV4(GoRangeIntensityMsg msg, kSerializer serializer)
{
    GoRangeIntensityMsgClass* obj = msg; 
    k32u count = (k32u) kArray2_Length_(obj->content, 0); 
    k32u width = (k32u) kArray2_Length_(obj->content, 1); 

    kCheck(kSerializer_Write16u_(serializer, GO_RANGE_INTENSITY_MSG_ATTR_4_12)); 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 

    kCheck(kSerializer_Write8uArray_(serializer, kArray1_Data_(obj->content), kArray1_Count_(obj->content))); 
    
    return kOK; 
}

GoFx(kStatus) GoRangeIntensityMsg_ReadV4(GoRangeIntensityMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoRangeIntensityMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u count; 
    k8u source, reserved;

    kCheck(GoRangeIntensityMsg_VInit(msg, kTypeOf(GoRangeIntensityMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_RANGE_INTENSITY_MSG_ATTR_4_12)); 
 
        kTest(kArray1_Construct(&obj->content, kTypeOf(k8u), count, alloc)); 
        kTest(kSerializer_Read8uArray_(serializer, kArray1_Data_(obj->content), kArray1_Count_(obj->content))); 
    
        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoRangeIntensityMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoRangeIntensityMsg_Source(GoRangeIntensityMsg msg)
{
    return GoRangeIntensityMsg_(msg)->source; 
}

GoFx(kSize) GoRangeIntensityMsg_Count(GoRangeIntensityMsg msg)
{
    return kArray1_Length_(GoRangeIntensityMsg_Content_(msg)); 
}

GoFx(k8u*) GoRangeIntensityMsg_At(GoRangeIntensityMsg msg, kSize index)
{
    kAssert(index < GoRangeIntensityMsg_Count(msg));
    
    return kArray1_At_(GoRangeIntensityMsg_Content_(msg), index); 
}

GoFx(k32u) GoRangeIntensityMsg_Exposure(GoRangeIntensityMsg msg)
{
    return GoRangeIntensityMsg_(msg)->exposure; 
}

/* 
 * GoProfileMsg
 */

kBeginClass(Go, GoProfileMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoProfileMsg, "godat", "6.0.0.0", "5", WriteV5, ReadV5)
    kAddVersion(GoProfileMsg, "kdat6", "4.0.0.0", "GoProfileMsg-5", WriteV5, ReadV5)

    //virtual methods
    kAddVMethod(GoProfileMsg, GoDataMsg, VInit)
    kAddVMethod(GoProfileMsg, kObject, VInitClone)
    kAddVMethod(GoProfileMsg, kObject, VRelease)
    kAddVMethod(GoProfileMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoProfileMsg_Construct(GoProfileMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileMsg), msg)); 

    if (!kSuccess(status = GoProfileMsg_VInit(*msg, kTypeOf(GoProfileMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoProfileMsg_VInit(GoProfileMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_PROFILE, alloc)); 

    kZeroDerivedFields_(GoProfileMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoProfileMsg_VInitClone(GoProfileMsg msg, GoProfileMsg source, kAlloc alloc)
{
    GoProfileMsgClass* obj = msg; 
    GoProfileMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoProfileMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->xResolution = srcObj->xResolution; 
        obj->zResolution = srcObj->zResolution; 
        obj->xOffset = srcObj->xOffset; 
        obj->zOffset = srcObj->zOffset; 
        obj->exposure = srcObj->exposure;
        obj->cameraIndex = srcObj->cameraIndex;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoProfileMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoProfileMsg_Allocate(GoProfileMsg msg, kSize count, kSize width)
{
    GoProfileMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray2_Construct(&obj->content, kTypeOf(kPoint16s), count, width, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoProfileMsg_VSize(GoProfileMsg msg)
{
    GoProfileMsgClass* obj = msg; 

    return sizeof(GoProfileMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoProfileMsg_VRelease(GoProfileMsg msg)
{
    GoProfileMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoProfileMsg_WriteV5(GoProfileMsg msg, kSerializer serializer)
{
    GoProfileMsgClass* obj = msg; 
    k32u count = (k32u) kArray2_Length_(obj->content, 0); 
    k32u width = (k32u) kArray2_Length_(obj->content, 1); 

    kCheck(kSerializer_Write16u_(serializer, GO_PROFILE_MSG_ATTR_5_32)); 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write32u_(serializer, width)); 
    kCheck(kSerializer_Write32u_(serializer, obj->xResolution)); 
    kCheck(kSerializer_Write32u_(serializer, obj->zResolution)); 
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset)); 
    kCheck(kSerializer_Write32s_(serializer, obj->zOffset)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, obj->cameraIndex)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 

    kCheck(kSerializer_Write16sArray_(serializer, kArray2_Data_(obj->content), count * width * 2)); 
    
    return kOK; 
}

GoFx(kStatus) GoProfileMsg_ReadV5(GoProfileMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoProfileMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u count, width; 
    k8u source, reserved;

    kCheck(GoProfileMsg_VInit(msg, kTypeOf(GoProfileMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read32u_(serializer, &width)); 
        kTest(kSerializer_Read32u_(serializer, &obj->xResolution)); 
        kTest(kSerializer_Read32u_(serializer, &obj->zResolution)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        kTest(kSerializer_Read32s_(serializer, &obj->zOffset)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
        kTest(kSerializer_Read8u_(serializer, &obj->cameraIndex)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_PROFILE_MSG_ATTR_5_32)); 
 
        kTest(kArray2_Construct(&obj->content, kTypeOf(kPoint16s), count, width, alloc)); 
        kTest(kSerializer_Read16sArray_(serializer, kArray2_Data_(obj->content), count * width * 2)); 
    
        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoProfileMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoProfileMsg_Source(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->source; 
}

GoFx(kSize) GoProfileMsg_Count(GoProfileMsg msg)
{
    return kArray2_Length_(GoProfileMsg_Content_(msg), 0); 
}

GoFx(kSize) GoProfileMsg_Width(GoProfileMsg msg)
{
    return kArray2_Length_(GoProfileMsg_Content_(msg), 1); 
}

GoFx(k32u) GoProfileMsg_XResolution(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->xResolution; 
}

GoFx(k32u) GoProfileMsg_ZResolution(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->zResolution; 
}

GoFx(k32s) GoProfileMsg_XOffset(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->xOffset; 
}

GoFx(k32s) GoProfileMsg_ZOffset(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->zOffset; 
}

GoFx(kPoint16s*) GoProfileMsg_At(GoProfileMsg msg, kSize index)
{
    kAssert(index < GoProfileMsg_Count(msg));

    return kArray2_At_(GoProfileMsg_Content_(msg), index, 0); 
}

GoFx(k32u) GoProfileMsg_Exposure(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->exposure; 
}

GoFx(k8u) GoProfileMsg_CameraIndex(GoProfileMsg msg)
{
    return GoProfileMsg_(msg)->cameraIndex; 
}

/* 
 * GoResampledProfileMsg
 */

kBeginClass(Go, GoResampledProfileMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoResampledProfileMsg, "godat", "6.0.0.0", "6", WriteV6, ReadV6)
    kAddVersion(GoResampledProfileMsg, "kdat6", "4.0.0.0", "GoResampledProfileMsg-6", WriteV6, ReadV6)

    //virtual methods
    kAddVMethod(GoResampledProfileMsg, GoDataMsg, VInit)
    kAddVMethod(GoResampledProfileMsg, kObject, VInitClone)
    kAddVMethod(GoResampledProfileMsg, kObject, VRelease)
    kAddVMethod(GoResampledProfileMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoResampledProfileMsg_Construct(GoResampledProfileMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoResampledProfileMsg), msg)); 

    if (!kSuccess(status = GoResampledProfileMsg_VInit(*msg, kTypeOf(GoResampledProfileMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoResampledProfileMsg_VInit(GoResampledProfileMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE, alloc)); 

    kZeroDerivedFields_(GoResampledProfileMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoResampledProfileMsg_VInitClone(GoResampledProfileMsg msg, GoResampledProfileMsg source, kAlloc alloc)
{
    GoResampledProfileMsgClass* obj = msg; 
    GoResampledProfileMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoResampledProfileMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->xResolution = srcObj->xResolution; 
        obj->zResolution = srcObj->zResolution; 
        obj->xOffset = srcObj->xOffset; 
        obj->zOffset = srcObj->zOffset; 
        obj->exposure = srcObj->exposure;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoResampledProfileMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoResampledProfileMsg_Allocate(GoResampledProfileMsg msg, kSize count, kSize width)
{
    GoResampledProfileMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray2_Construct(&obj->content, kTypeOf(k16s), count, width, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoResampledProfileMsg_VSize(GoResampledProfileMsg msg)
{
    GoResampledProfileMsgClass* obj = msg; 

    return sizeof(GoResampledProfileMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoResampledProfileMsg_VRelease(GoResampledProfileMsg msg)
{
    GoResampledProfileMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoResampledProfileMsg_WriteV6(GoResampledProfileMsg msg, kSerializer serializer)
{
    GoResampledProfileMsgClass* obj = msg; 
    k32u count = (k32u) kArray2_Length_(obj->content, 0); 
    k32u width = (k32u) kArray2_Length_(obj->content, 1); 

    kCheck(kSerializer_Write16u_(serializer, GO_RESAMPLED_PROFILE_MSG_ATTR_6_32)); 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write32u_(serializer, width)); 
    kCheck(kSerializer_Write32u_(serializer, obj->xResolution)); 
    kCheck(kSerializer_Write32u_(serializer, obj->zResolution)); 
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset)); 
    kCheck(kSerializer_Write32s_(serializer, obj->zOffset)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 

    kCheck(kSerializer_Write16sArray_(serializer, kArray2_Data_(obj->content), count * width)); 
    
    return kOK; 
}

GoFx(kStatus) GoResampledProfileMsg_ReadV6(GoResampledProfileMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoResampledProfileMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u count, width; 
    k8u source, reserved; 

    kCheck(GoResampledProfileMsg_VInit(msg, kTypeOf(GoResampledProfileMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read32u_(serializer, &width)); 
        kTest(kSerializer_Read32u_(serializer, &obj->xResolution)); 
        kTest(kSerializer_Read32u_(serializer, &obj->zResolution)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        kTest(kSerializer_Read32s_(serializer, &obj->zOffset)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_RESAMPLED_PROFILE_MSG_ATTR_6_32)); 
 
        kTest(kArray2_Construct(&obj->content, kTypeOf(k16s), count, width, alloc)); 
        kTest(kSerializer_Read16sArray_(serializer, kArray2_Data_(obj->content), count * width)); 
    
        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoResampledProfileMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoResampledProfileMsg_Source(GoResampledProfileMsg msg)
{
    return GoResampledProfileMsg_(msg)->source; 
}

GoFx(kSize) GoResampledProfileMsg_Count(GoResampledProfileMsg msg)
{
    return kArray2_Length_(GoResampledProfileMsg_Content_(msg), 0); 
}

GoFx(kSize) GoResampledProfileMsg_Width(GoResampledProfileMsg msg)
{
    return kArray2_Length_(GoResampledProfileMsg_Content_(msg), 1); 
}

GoFx(k32u) GoResampledProfileMsg_XResolution(GoResampledProfileMsg msg)
{
    return GoResampledProfileMsg_(msg)->xResolution; 
}

GoFx(k32u) GoResampledProfileMsg_ZResolution(GoResampledProfileMsg msg)
{
    return GoResampledProfileMsg_(msg)->zResolution; 
}

GoFx(k32s) GoResampledProfileMsg_XOffset(GoResampledProfileMsg msg)
{
    return GoResampledProfileMsg_(msg)->xOffset; 
}

GoFx(k32s) GoResampledProfileMsg_ZOffset(GoResampledProfileMsg msg)
{
    return GoResampledProfileMsg_(msg)->zOffset; 
}

GoFx(k16s*) GoResampledProfileMsg_At(GoResampledProfileMsg msg, kSize index)
{
    kAssert(index < GoResampledProfileMsg_Count(msg));

    return kArray2_At_(GoResampledProfileMsg_Content_(msg), index, 0); 
}

GoFx(k32u) GoResampledProfileMsg_Exposure(GoResampledProfileMsg msg)
{
    return GoResampledProfileMsg_(msg)->exposure; 
}


/* 
 * GoProfileIntensityMsg
 */

kBeginClass(Go, GoProfileIntensityMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoProfileIntensityMsg, "godat", "6.0.0.0", "7", WriteV7, ReadV7)
    kAddVersion(GoProfileIntensityMsg, "kdat6", "4.0.0.0", "GoProfileIntensityMsg-7", WriteV7, ReadV7)

    //virtual methods
    kAddVMethod(GoProfileIntensityMsg, GoDataMsg, VInit)
    kAddVMethod(GoProfileIntensityMsg, kObject, VInitClone)
    kAddVMethod(GoProfileIntensityMsg, kObject, VRelease)
    kAddVMethod(GoProfileIntensityMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoProfileIntensityMsg_Construct(GoProfileIntensityMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileIntensityMsg), msg)); 

    if (!kSuccess(status = GoProfileIntensityMsg_VInit(*msg, kTypeOf(GoProfileIntensityMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoProfileIntensityMsg_VInit(GoProfileIntensityMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY, alloc)); 

    kZeroDerivedFields_(GoProfileIntensityMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoProfileIntensityMsg_VInitClone(GoProfileIntensityMsg msg, GoProfileIntensityMsg source, kAlloc alloc)
{
    GoProfileIntensityMsgClass* obj = msg; 
    GoProfileIntensityMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoProfileIntensityMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->xResolution = srcObj->xResolution; 
        obj->xOffset = srcObj->xOffset; 
        obj->exposure = srcObj->exposure;
        obj->cameraIndex = srcObj->cameraIndex;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoProfileIntensityMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoProfileIntensityMsg_Allocate(GoProfileIntensityMsg msg, kSize count, kSize width)
{
    GoProfileIntensityMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray2_Construct(&obj->content, kTypeOf(k8u), count, width, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoProfileIntensityMsg_VSize(GoProfileIntensityMsg msg)
{
    GoProfileIntensityMsgClass* obj = msg; 

    return sizeof(GoProfileIntensityMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoProfileIntensityMsg_VRelease(GoProfileIntensityMsg msg)
{
    GoProfileIntensityMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoProfileIntensityMsg_WriteV7(GoProfileIntensityMsg msg, kSerializer serializer)
{
    GoProfileIntensityMsgClass* obj = msg; 
    k32u count = (k32u) kArray2_Length_(obj->content, 0); 
    k32u width = (k32u) kArray2_Length_(obj->content, 1); 

    kCheck(kSerializer_Write16u_(serializer, GO_PROFILE_INTENSITY_MSG_ATTR_7_24)); 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write32u_(serializer, width)); 
    kCheck(kSerializer_Write32u_(serializer, obj->xResolution)); 
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, obj->cameraIndex)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 

    kCheck(kSerializer_Write8uArray_(serializer, kArray2_Data_(obj->content), kArray2_Count_(obj->content))); 
    
    return kOK; 
}

GoFx(kStatus) GoProfileIntensityMsg_ReadV7(GoProfileIntensityMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoProfileIntensityMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u count, width; 
    k8u source, reserved;

    kCheck(GoProfileIntensityMsg_VInit(msg, kTypeOf(GoProfileIntensityMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read32u_(serializer, &width)); 
        kTest(kSerializer_Read32u_(serializer, &obj->xResolution)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
        kTest(kSerializer_Read8u_(serializer, &obj->cameraIndex)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_PROFILE_INTENSITY_MSG_ATTR_7_24)); 
 
        kTest(kArray2_Construct(&obj->content, kTypeOf(k8u), count, width, alloc)); 
        kTest(kSerializer_Read8uArray_(serializer, kArray2_Data_(obj->content), kArray2_Count_(obj->content))); 
    
        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoProfileIntensityMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoProfileIntensityMsg_Source(GoProfileIntensityMsg msg)
{
    return GoProfileIntensityMsg_(msg)->source; 
}

GoFx(kSize) GoProfileIntensityMsg_Count(GoProfileIntensityMsg msg)
{
    return kArray2_Length_(GoProfileIntensityMsg_Content_(msg), 0); 
}

GoFx(kSize) GoProfileIntensityMsg_Width(GoProfileIntensityMsg msg)
{
    return kArray2_Length_(GoProfileIntensityMsg_Content_(msg), 1); 
}

GoFx(k32u) GoProfileIntensityMsg_XResolution(GoProfileIntensityMsg msg)
{
    return GoProfileIntensityMsg_(msg)->xResolution; 
}

GoFx(k32s) GoProfileIntensityMsg_XOffset(GoProfileIntensityMsg msg)
{
    return GoProfileIntensityMsg_(msg)->xOffset; 
}

GoFx(k8u*) GoProfileIntensityMsg_At(GoProfileIntensityMsg msg, kSize index)
{
    kAssert(index < GoProfileIntensityMsg_Count(msg));
    
    return kArray2_At_(GoProfileIntensityMsg_Content_(msg), index, 0); 
}

GoFx(k32u) GoProfileIntensityMsg_Exposure(GoProfileIntensityMsg msg)
{
    return GoProfileIntensityMsg_(msg)->exposure; 
}

GoFx(k8u) GoProfileIntensityMsg_CameraIndex(GoProfileMsg msg)
{
    return GoProfileIntensityMsg_(msg)->cameraIndex; 
}


/* 
 * GoSurfaceMsg
 */

kBeginClass(Go, GoSurfaceMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoSurfaceMsg, "godat", "6.0.0.0", "8", WriteV8, ReadV8)
    kAddVersion(GoSurfaceMsg, "kdat6", "4.0.0.0", "GoSurfaceMsg-8", WriteV8, ReadV8)

    //virtual methods
    kAddVMethod(GoSurfaceMsg, GoDataMsg, VInit)
    kAddVMethod(GoSurfaceMsg, kObject, VInitClone)
    kAddVMethod(GoSurfaceMsg, kObject, VRelease)
    kAddVMethod(GoSurfaceMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoSurfaceMsg_Construct(GoSurfaceMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceMsg), msg)); 

    if (!kSuccess(status = GoSurfaceMsg_VInit(*msg, kTypeOf(GoSurfaceMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoSurfaceMsg_VInit(GoSurfaceMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_SURFACE, alloc)); 

    kZeroDerivedFields_(GoSurfaceMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceMsg_VInitClone(GoSurfaceMsg msg, GoSurfaceMsg source, kAlloc alloc)
{
    GoSurfaceMsgClass* obj = msg; 
    GoSurfaceMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoSurfaceMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->xResolution = srcObj->xResolution; 
        obj->yResolution = srcObj->yResolution; 
        obj->zResolution = srcObj->zResolution; 
        obj->xOffset = srcObj->xOffset; 
        obj->yOffset = srcObj->yOffset; 
        obj->zOffset = srcObj->zOffset; 
        obj->exposure = srcObj->exposure;
        obj->zAngle = srcObj->zAngle;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoSurfaceMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSurfaceMsg_Allocate(GoSurfaceMsg msg, kSize length, kSize width)
{
    GoSurfaceMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray2_Construct(&obj->content, kTypeOf(k16s), length, width, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoSurfaceMsg_VSize(GoSurfaceMsg msg)
{
    GoSurfaceMsgClass* obj = msg; 

    return sizeof(GoSurfaceMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoSurfaceMsg_VRelease(GoSurfaceMsg msg)
{
    GoSurfaceMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceMsg_WriteV8(GoSurfaceMsg msg, kSerializer serializer)
{
    GoSurfaceMsgClass* obj = msg; 
    k32u length = (k32u) kArray2_Length_(obj->content, 0); 
    k32u width = (k32u) kArray2_Length_(obj->content, 1); 

    kCheck(kSerializer_Write16u_(serializer, GO_SURFACE_MSG_ATTR_8_44)); 

    kCheck(kSerializer_Write32u_(serializer, length)); 
    kCheck(kSerializer_Write32u_(serializer, width)); 
    kCheck(kSerializer_Write32u_(serializer, obj->xResolution)); 
    kCheck(kSerializer_Write32u_(serializer, obj->yResolution)); 
    kCheck(kSerializer_Write32u_(serializer, obj->zResolution)); 
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset)); 
    kCheck(kSerializer_Write32s_(serializer, obj->yOffset)); 
    kCheck(kSerializer_Write32s_(serializer, obj->zOffset)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write32s_(serializer, obj->zAngle)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) 0)); 

    kCheck(kSerializer_Write16sArray_(serializer, kArray2_Data_(obj->content), kArray2_Count_(obj->content))); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceMsg_ReadV8(GoSurfaceMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoSurfaceMsgClass* obj = msg; 
    kSerializerClass* srl = serializer;
    kStatus status; 
    k16u attrSize; 
    k32u length, width; 
    k8u source, reserved;
    kSize readLength;

    kCheck(GoSurfaceMsg_VInit(msg, kTypeOf(GoSurfaceMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &length)); 
        kTest(kSerializer_Read32u_(serializer, &width)); 
        kTest(kSerializer_Read32u_(serializer, &obj->xResolution)); 
        kTest(kSerializer_Read32u_(serializer, &obj->yResolution)); 
        kTest(kSerializer_Read32u_(serializer, &obj->zResolution)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        kTest(kSerializer_Read32s_(serializer, &obj->yOffset)); 
        kTest(kSerializer_Read32s_(serializer, &obj->zOffset)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure));

        if (attrSize >= GO_SURFACE_MSG_ATTR_8_44) 
        {
            kTest(kSerializer_Read32s_(serializer, &obj->zAngle)); 
            readLength = GO_SURFACE_MSG_ATTR_8_44;
        }
        else
        {
            readLength = GO_SURFACE_MSG_ATTR_8_44 - 4;  //subtracted by the length of the z-angle field
        }

        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_AdvanceRead_(serializer, attrSize - readLength)); 

        kTest(kArray2_Construct(&obj->content, kTypeOf(k16s), length, width, alloc)); 
        kTest(kSerializer_Read16sArray_(serializer, kArray2_Data_(obj->content), kArray2_Count_(obj->content))); 

        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoSurfaceMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoSurfaceMsg_Source(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->source; 
}

GoFx(kSize) GoSurfaceMsg_Length(GoSurfaceMsg msg)
{
    return kArray2_Length_(GoSurfaceMsg_Content_(msg), 0); 
}

GoFx(kSize) GoSurfaceMsg_Width(GoSurfaceMsg msg)
{
    return kArray2_Length_(GoSurfaceMsg_Content_(msg), 1); 
}

GoFx(k32u) GoSurfaceMsg_XResolution(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->xResolution; 
}

GoFx(k32u) GoSurfaceMsg_YResolution(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->yResolution; 
}

GoFx(k32u) GoSurfaceMsg_ZResolution(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->zResolution; 
}

GoFx(k32s) GoSurfaceMsg_XOffset(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->xOffset; 
}

GoFx(k32s) GoSurfaceMsg_YOffset(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->yOffset; 
}

GoFx(k32s) GoSurfaceMsg_ZOffset(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->zOffset; 
}

GoFx(k32s) GoSurfaceMsg_ZAngle(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->zAngle; 
}

GoFx(k16s*) GoSurfaceMsg_RowAt(GoSurfaceMsg msg, kSize index)
{
    kAssert(index < GoSurfaceMsg_Length(msg));

    return kArray2_At_(GoSurfaceMsg_Content_(msg), index, 0); 
}

GoFx(k32u) GoSurfaceMsg_Exposure(GoSurfaceMsg msg)
{
    return GoSurfaceMsg_(msg)->exposure; 
}


/* 
 * GoSurfaceIntensityMsg
 */

kBeginClass(Go, GoSurfaceIntensityMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoSurfaceIntensityMsg, "godat", "6.0.0.0", "9", WriteV9, ReadV9)
    kAddVersion(GoSurfaceIntensityMsg, "kdat6", "4.0.0.0", "GoSurfaceIntensityMsg-9", WriteV9, ReadV9)

    //virtual methods
    kAddVMethod(GoSurfaceIntensityMsg, GoDataMsg, VInit)
    kAddVMethod(GoSurfaceIntensityMsg, kObject, VInitClone)
    kAddVMethod(GoSurfaceIntensityMsg, kObject, VRelease)
    kAddVMethod(GoSurfaceIntensityMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoSurfaceIntensityMsg_Construct(GoSurfaceIntensityMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceIntensityMsg), msg)); 

    if (!kSuccess(status = GoSurfaceIntensityMsg_VInit(*msg, kTypeOf(GoSurfaceIntensityMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoSurfaceIntensityMsg_VInit(GoSurfaceIntensityMsg msg, kType type, kAlloc alloc)
{
    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY, alloc)); 

    kZeroDerivedFields_(GoSurfaceIntensityMsg, msg); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceIntensityMsg_VInitClone(GoSurfaceIntensityMsg msg, GoSurfaceIntensityMsg source, kAlloc alloc)
{
    GoSurfaceIntensityMsgClass* obj = msg; 
    GoSurfaceIntensityMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoSurfaceIntensityMsg_VInit(msg, kObject_Type_(source), alloc));     
    
    kTry
    {
        obj->source = srcObj->source; 
        obj->xResolution = srcObj->xResolution; 
        obj->yResolution = srcObj->yResolution; 
        obj->xOffset = srcObj->xOffset; 
        obj->yOffset = srcObj->yOffset; 
        obj->exposure = srcObj->exposure;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoSurfaceIntensityMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSurfaceIntensityMsg_Allocate(GoSurfaceIntensityMsg msg, kSize length, kSize width)
{
    GoSurfaceIntensityMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->content)); 
    kCheck(kArray2_Construct(&obj->content, kTypeOf(k8u), length, width, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kSize) GoSurfaceIntensityMsg_VSize(GoSurfaceIntensityMsg msg)
{
    GoSurfaceIntensityMsgClass* obj = msg; 

    return sizeof(GoSurfaceIntensityMsgClass) + (kIsNull(obj->content) ? 0 : kObject_Size_(obj->content)); 
}

GoFx(kStatus) GoSurfaceIntensityMsg_VRelease(GoSurfaceIntensityMsg msg)
{
    GoSurfaceIntensityMsgClass* obj = msg; 

    kCheck(kObject_Destroy(obj->content)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceIntensityMsg_WriteV9(GoSurfaceIntensityMsg msg, kSerializer serializer)
{
    GoSurfaceIntensityMsgClass* obj = msg; 
    k32u length = (k32u) kArray2_Length_(obj->content, 0); 
    k32u width = (k32u) kArray2_Length_(obj->content, 1); 

    kCheck(kSerializer_Write16u_(serializer, GO_SURFACE_INTENSITY_MSG_ATTR_9_32)); 

    kCheck(kSerializer_Write32u_(serializer, length)); 
    kCheck(kSerializer_Write32u_(serializer, width)); 
    kCheck(kSerializer_Write32u_(serializer, obj->xResolution)); 
    kCheck(kSerializer_Write32u_(serializer, obj->yResolution)); 
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset)); 
    kCheck(kSerializer_Write32s_(serializer, obj->yOffset)); 
    kCheck(kSerializer_Write8u_(serializer, (k8u) obj->source)); 
    kCheck(kSerializer_Write32u_(serializer, obj->exposure)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 

    kCheck(kSerializer_Write8uArray_(serializer, kArray2_Data_(obj->content), kArray2_Count_(obj->content))); 
    
    return kOK; 
}

GoFx(kStatus) GoSurfaceIntensityMsg_ReadV9(GoSurfaceIntensityMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoSurfaceIntensityMsgClass* obj = msg; 
    kStatus status; 
    k16u attrSize; 
    k32u length, width; 
    k8u source, reserved;

    kCheck(GoSurfaceIntensityMsg_VInit(msg, kTypeOf(GoSurfaceIntensityMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 

        kTest(kSerializer_Read32u_(serializer, &length)); 
        kTest(kSerializer_Read32u_(serializer, &width)); 
        kTest(kSerializer_Read32u_(serializer, &obj->xResolution)); 
        kTest(kSerializer_Read32u_(serializer, &obj->yResolution)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        kTest(kSerializer_Read32s_(serializer, &obj->yOffset)); 
        kTest(kSerializer_Read8u_(serializer, &source)); 
        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_SURFACE_INTENSITY_MSG_ATTR_9_32)); 
 
        kTest(kArray2_Construct(&obj->content, kTypeOf(k8u), length, width, alloc)); 
        kTest(kSerializer_Read8uArray_(serializer, kArray2_Data_(obj->content), kArray2_Count_(obj->content))); 
    
        obj->source = (GoDataSource)source; 
    } 
    kCatch(&status)
    {
        GoSurfaceIntensityMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoDataSource) GoSurfaceIntensityMsg_Source(GoSurfaceIntensityMsg msg)
{
    return GoSurfaceIntensityMsg_(msg)->source; 
}

GoFx(kSize) GoSurfaceIntensityMsg_Length(GoSurfaceIntensityMsg msg)
{
    return kArray2_Length_(GoSurfaceIntensityMsg_Content_(msg), 0); 
}

GoFx(kSize) GoSurfaceIntensityMsg_Width(GoSurfaceIntensityMsg msg)
{
    return kArray2_Length_(GoSurfaceIntensityMsg_Content_(msg), 1); 
}

GoFx(k32u) GoSurfaceIntensityMsg_XResolution(GoSurfaceIntensityMsg msg)
{
    return GoSurfaceIntensityMsg_(msg)->xResolution; 
}

GoFx(k32u) GoSurfaceIntensityMsg_YResolution(GoSurfaceIntensityMsg msg)
{
    return GoSurfaceIntensityMsg_(msg)->yResolution; 
}

GoFx(k32s) GoSurfaceIntensityMsg_XOffset(GoSurfaceIntensityMsg msg)
{
    return GoSurfaceIntensityMsg_(msg)->xOffset; 
}

GoFx(k32s) GoSurfaceIntensityMsg_YOffset(GoSurfaceIntensityMsg msg)
{
    return GoSurfaceIntensityMsg_(msg)->yOffset; 
}

GoFx(k8u*) GoSurfaceIntensityMsg_RowAt(GoSurfaceIntensityMsg msg, kSize index)
{
    kAssert(index < GoSurfaceIntensityMsg_Length(msg));

    return kArray2_At_(GoSurfaceIntensityMsg_Content_(msg), index, 0); 
}

GoFx(k32u) GoSurfaceIntensityMsg_Exposure(GoSurfaceIntensityMsg msg)
{
    return GoSurfaceIntensityMsg_(msg)->exposure; 
}


/* 
 * GoMeasurementData
 */
kBeginValue(Go, GoMeasurementData, kValue) 
    kAddField(GoMeasurementData, k64f, value)
    kAddField(GoMeasurementData, k8u, decision)
    kAddField(GoMeasurementData, k8u, decisionCode)
kEndValue() 

GoFx(k64f) GoMeasurementData_Value(GoMeasurementData data)
{
    return data.value;
}

GoFx(GoDecision) GoMeasurementData_Decision(GoMeasurementData data)
{
    return data.decision;
}

GoFx(GoDecisionCode) GoMeasurementData_DecisionCode(GoMeasurementData data)
{
    return data.decisionCode;
}

/* 
 * GoMeasurementMsg
 */

kBeginClass(Go, GoMeasurementMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoMeasurementMsg, "godat", "6.0.0.0", "10", WriteV10, ReadV10)
    kAddVersion(GoMeasurementMsg, "kdat6", "4.0.0.0", "GoMeasurementMsg-10", WriteV10, ReadV10)

    //virtual methods
    kAddVMethod(GoMeasurementMsg, GoDataMsg, VInit)
    kAddVMethod(GoMeasurementMsg, kObject, VInitClone)
    kAddVMethod(GoMeasurementMsg, kObject, VRelease)
    kAddVMethod(GoMeasurementMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoMeasurementMsg_Construct(GoMeasurementMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoMeasurementMsg), msg)); 

    if (!kSuccess(status = GoMeasurementMsg_VInit(*msg, kTypeOf(GoMeasurementMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoMeasurementMsg_VInit(GoMeasurementMsg msg, kType type, kAlloc alloc)
{
    GoMeasurementMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_MEASUREMENT, alloc)); 

    obj->measurements = kNULL; 
    obj->id = 0; 
    
    return kOK; 
}

GoFx(kStatus) GoMeasurementMsg_VInitClone(GoMeasurementMsg msg, GoMeasurementMsg source, kAlloc alloc)
{
    GoMeasurementMsgClass* obj = msg; 
    GoMeasurementMsgClass* srcObj = source; 
    kStatus status; 

    kCheck(GoMeasurementMsg_VInit(msg, kObject_Type_(source), alloc));     

    kTry
    {
        obj->id = srcObj->id; 

        kTest(kObject_Clone(&obj->measurements, srcObj->measurements, alloc)); 
    }
    kCatch(&status)
    {
        GoMeasurementMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoMeasurementMsg_Allocate(GoMeasurementMsg msg, kSize count)
{
    GoMeasurementMsgClass* obj = msg; 

    kCheck(kDestroyRef(&obj->measurements)); 
    kCheck(kArray1_Construct(&obj->measurements, kTypeOf(GoMeasurementData), count, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kStatus) GoMeasurementMsg_VRelease(GoMeasurementMsg msg)
{
    GoMeasurementMsgClass* obj = msg; 
    
    kCheck(kObject_Destroy(obj->measurements)); 

    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoMeasurementMsg_VSize(GoMeasurementMsg msg)
{
    GoMeasurementMsgClass* obj = msg; 

    return sizeof(GoMeasurementMsgClass) + (kIsNull(obj->measurements) ? 0 : kObject_Size_(obj->measurements)); 
}

GoFx(kStatus) GoMeasurementMsg_WriteV10(GoMeasurementMsg msg, kSerializer serializer)
{
    GoMeasurementMsgClass* obj = msg; 
    k32u count = (k32u) kArray1_Count_(obj->measurements); 
    k32u i; 

    kCheck(kSerializer_Write32u_(serializer, count)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write8u_(serializer, 0)); 
    kCheck(kSerializer_Write16u_(serializer, obj->id));  

    for (i = 0; i < count; ++i)
    {
        const GoMeasurementData* measurement = kArray1_At_(obj->measurements, i); 

        if (measurement->value != k64F_NULL)
        {
            kCheck(kSerializer_Write32s_(serializer, (k32s)(measurement->value * 1000.0)));
        }
        else
        {
            kCheck(kSerializer_Write32s_(serializer, k32S_NULL));
        }

        kCheck(kSerializer_Write8u_(serializer, (measurement->decision | (measurement->decisionCode << 1)))); 
        kCheck(kSerializer_Write8u_(serializer, 0)); 
        kCheck(kSerializer_Write8u_(serializer, 0)); 
        kCheck(kSerializer_Write8u_(serializer, 0)); 
    }

    return kOK; 
}

GoFx(kStatus) GoMeasurementMsg_ReadV10(GoMeasurementMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoMeasurementMsgClass* obj = msg; 
    kStatus status;
    k32u count = 0; 
    k32u i; 
    k32s intVal;
    k8u decision, reserved;

    kCheck(GoMeasurementMsg_VInit(msg, kTypeOf(GoMeasurementMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read32u_(serializer, &count)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read16u_(serializer, &obj->id)); 
            
        kTest(kArray1_Construct(&obj->measurements, kTypeOf(GoMeasurementData), count, alloc)); 

        for (i = 0; i < count; ++i)
        {
            GoMeasurementData* measurement = (GoMeasurementData*)kArray1_At_(obj->measurements, i); 

            kTest(kSerializer_Read32s_(serializer, &intVal)); 
            if (intVal != k32S_NULL)
            {
                measurement->value = intVal / 1000.0;
            }
            else
            {
                measurement->value = k64F_NULL;
            }

            kTest(kSerializer_Read8u_(serializer, &decision)); 
            kTest(kSerializer_Read8u_(serializer, &reserved)); 
            kTest(kSerializer_Read8u_(serializer, &reserved)); 
            kTest(kSerializer_Read8u_(serializer, &reserved)); 

            measurement->decision = (decision & GO_DECISION_PASS);
            measurement->decisionCode = (decision >> 1);
        } 
    }
    kCatch(&status)
    {
        GoMeasurementMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(k16u) GoMeasurementMsg_Id(GoMeasurementMsg msg)
{
    return GoMeasurementMsg_(msg)->id; 
}

GoFx(kSize) GoMeasurementMsg_Count(GoMeasurementMsg msg)
{
    return kArrayList_Count_(GoMeasurementMsg_Content_(msg)); 
}

GoFx(GoMeasurementData*) GoMeasurementMsg_At(GoMeasurementMsg msg, kSize index)
{
    kAssert(index < GoMeasurementMsg_Count(msg));

    return kArrayList_At_(GoMeasurementMsg_Content_(msg), index); 
}


/* 
 * GoCalMsg
 */

kBeginClass(Go, GoAlignMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoAlignMsg, "godat", "6.0.0.0", "11", WriteV11, ReadV11)
    kAddVersion(GoAlignMsg, "kdat6", "4.0.0.0", "GoAlignMsg-11", WriteV11, ReadV11)

    //virtual methods
    kAddVMethod(GoAlignMsg, GoDataMsg, VInit)
    kAddVMethod(GoAlignMsg, kObject, VInitClone)
    kAddVMethod(GoAlignMsg, kObject, VRelease)
    kAddVMethod(GoAlignMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoAlignMsg_Construct(GoAlignMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAlignMsg), msg)); 

    if (!kSuccess(status = GoAlignMsg_VInit(*msg, kTypeOf(GoAlignMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoAlignMsg_VInit(GoAlignMsg msg, kType type, kAlloc alloc)
{
    GoAlignMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_ALIGNMENT, alloc)); 

    obj->status = kOK; 
        
    return kOK; 
}

GoFx(kStatus) GoAlignMsg_VInitClone(GoAlignMsg msg, GoAlignMsg source, kAlloc alloc)
{
    GoAlignMsgClass* obj = msg; 
    GoAlignMsgClass* srcObj = source; 

    kCheck(GoAlignMsg_VInit(msg, kObject_Type_(source), alloc));     

    obj->status = srcObj->status;

    return kOK; 
}

GoFx(kStatus) GoAlignMsg_VRelease(GoAlignMsg msg)
{    
    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoAlignMsg_VSize(GoAlignMsg msg)
{
    return sizeof(GoAlignMsgClass); 
}

GoFx(kStatus) GoAlignMsg_WriteV11(GoAlignMsg msg, kSerializer serializer)
{
    GoAlignMsgClass* obj = msg; 

    kCheck(kSerializer_Write16u_(serializer, GO_CAL_MSG_ATTR_SIZE_11_8)); 
    kCheck(kSerializer_Write32u_(serializer, obj->opId));  
    kCheck(kSerializer_Write32s_(serializer, obj->status));  

    return kOK; 
}

GoFx(kStatus) GoAlignMsg_ReadV11(GoAlignMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoAlignMsgClass* obj = msg; 
    k16u attrSize = 0; 
    kStatus status; 

    kCheck(GoAlignMsg_VInit(msg, kTypeOf(GoAlignMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 
        kTest(kSerializer_Read32u_(serializer, &obj->opId));
        kTest(kSerializer_Read32s_(serializer, &obj->status)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_CAL_MSG_ATTR_SIZE_11_8)); 
    }
    kCatch(&status)
    {
        GoAlignMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(GoAlignmentStatus) GoAlignMsg_Status(GoAlignMsg msg)
{
    return GoAlignMsg_(msg)->status; 
}



/* 
 * GoExposureCalMsg
 */

kBeginClass(Go, GoExposureCalMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoExposureCalMsg, "godat", "6.0.0.0", "12", WriteV12, ReadV12)
    kAddVersion(GoExposureCalMsg, "kdat6", "4.0.0.0", "GoExposureCalMsg-12", WriteV12, ReadV12)

    //virtual methods
    kAddVMethod(GoExposureCalMsg, GoDataMsg, VInit)
    kAddVMethod(GoExposureCalMsg, kObject, VInitClone)
    kAddVMethod(GoExposureCalMsg, kObject, VRelease)
    kAddVMethod(GoExposureCalMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoExposureCalMsg_Construct(GoExposureCalMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoExposureCalMsg), msg)); 

    if (!kSuccess(status = GoExposureCalMsg_VInit(*msg, kTypeOf(GoExposureCalMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoExposureCalMsg_VInit(GoExposureCalMsg msg, kType type, kAlloc alloc)
{
    GoExposureCalMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL, alloc)); 

    obj->status = kOK; 
    obj->exposure = kINFINITE;
        
    return kOK; 
}

GoFx(kStatus) GoExposureCalMsg_VInitClone(GoExposureCalMsg msg, GoExposureCalMsg source, kAlloc alloc)
{
    GoExposureCalMsgClass* obj = msg; 
    GoExposureCalMsgClass* srcObj = source; 

    kCheck(GoExposureCalMsg_VInit(msg, kObject_Type_(source), alloc));     

    obj->status = srcObj->status;
    obj->exposure = srcObj->exposure;

    return kOK; 
}

GoFx(kStatus) GoExposureCalMsg_VRelease(GoExposureCalMsg msg)
{    
    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoExposureCalMsg_VSize(GoExposureCalMsg msg)
{
    return sizeof(GoExposureCalMsgClass); 
}

GoFx(kStatus) GoExposureCalMsg_WriteV12(GoExposureCalMsg msg, kSerializer serializer)
{
    GoExposureCalMsgClass* obj = msg; 

    kCheck(kSerializer_Write16u_(serializer, GO_EXPOSURE_CAL_MSG_ATTR_SIZE_12_8)); 
    kCheck(kSerializer_Write32u_(serializer, obj->opId));  
    kCheck(kSerializer_Write32s_(serializer, obj->status));  
    kCheck(kSerializer_Write32s_(serializer, obj->exposure));  

    return kOK; 
}

GoFx(kStatus) GoExposureCalMsg_ReadV12(GoExposureCalMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoExposureCalMsgClass* obj = msg; 
    k16u attrSize = 0; 
    kStatus status; 

    kCheck(GoExposureCalMsg_VInit(msg, kTypeOf(GoExposureCalMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 
        kTest(kSerializer_Read32u_(serializer, &obj->opId));
        kTest(kSerializer_Read32s_(serializer, &obj->status)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_EXPOSURE_CAL_MSG_ATTR_SIZE_12_8)); 

        kTest(kSerializer_Read32u_(serializer, &obj->exposure)); 
    }
    kCatch(&status)
    {
        GoExposureCalMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoExposureCalMsg_Status(GoExposureCalMsg msg)
{
    return GoExposureCalMsg_(msg)->status; 
}

GoFx(k32u) GoExposureCalMsg_Exposure(GoExposureCalMsg msg)
{
    return GoExposureCalMsg_(msg)->exposure; 
}


/* 
 * GoEdgeMatchMsg
 */

kBeginClass(Go, GoEdgeMatchMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoEdgeMatchMsg, "godat", "6.0.0.0", "16", WriteV16, ReadV16)
    kAddVersion(GoEdgeMatchMsg, "kdat6", "4.0.0.0", "GoEdgeMatchMsg-16", WriteV16, ReadV16)

    //virtual methods
    kAddVMethod(GoEdgeMatchMsg, GoDataMsg, VInit)
    kAddVMethod(GoEdgeMatchMsg, kObject, VInitClone)
    kAddVMethod(GoEdgeMatchMsg, kObject, VRelease)
    kAddVMethod(GoEdgeMatchMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoEdgeMatchMsg_Construct(GoEdgeMatchMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoEdgeMatchMsg), msg)); 

    if (!kSuccess(status = GoEdgeMatchMsg_VInit(*msg, kTypeOf(GoEdgeMatchMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoEdgeMatchMsg_VInit(GoEdgeMatchMsg msg, kType type, kAlloc alloc)
{
    GoEdgeMatchMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_EDGE_MATCH, alloc)); 
    kInitFields_(GoEdgeMatchMsg, msg);

    return kOK; 
}

GoFx(kStatus) GoEdgeMatchMsg_VInitClone(GoEdgeMatchMsg msg, GoEdgeMatchMsg source, kAlloc alloc)
{
    GoEdgeMatchMsgClass* obj = msg; 
    GoEdgeMatchMsgClass* srcObj = source; 

    kCheck(GoEdgeMatchMsg_VInit(msg, kObject_Type_(source), alloc));     

    obj->decision = srcObj->decision;
    obj->xOffset = srcObj->xOffset;
    obj->yOffset = srcObj->yOffset;
    obj->zAngle = srcObj->zAngle;
    obj->qualityValue = srcObj->qualityValue;
    obj->qualityDecision = srcObj->qualityDecision;
    
    return kOK; 
}

GoFx(kStatus) GoEdgeMatchMsg_VRelease(GoEdgeMatchMsg msg)
{    
    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoEdgeMatchMsg_VSize(GoEdgeMatchMsg msg)
{
    return sizeof(GoEdgeMatchMsgClass); 
}

GoFx(kStatus) GoEdgeMatchMsg_WriteV16(GoEdgeMatchMsg msg, kSerializer serializer)
{
    GoEdgeMatchMsgClass* obj = msg; 

    kCheck(kSerializer_Write16u_(serializer, GO_EDGE_MATCH_MSG_ATTR_SIZE_16_36)); 
    kCheck(kSerializer_Write8u_(serializer, obj->decision));  
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset));  
    kCheck(kSerializer_Write32s_(serializer, obj->yOffset));  
    kCheck(kSerializer_Write32s_(serializer, obj->zAngle));  
    kCheck(kSerializer_Write32s_(serializer, obj->qualityValue));  
    kCheck(kSerializer_Write8u_(serializer, obj->qualityDecision));  
    kCheck(kSerializer_Write8u_(serializer, 0));  
    kCheck(kSerializer_Write8u_(serializer, 0));  

    return kOK; 
}

GoFx(kStatus) GoEdgeMatchMsg_ReadV16(GoEdgeMatchMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoEdgeMatchMsgClass* obj = msg; 
    k16u attrSize = 0; 
    kStatus status; 
    k8u reserved;

    kCheck(GoEdgeMatchMsg_VInit(msg, kTypeOf(GoEdgeMatchMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 
        kTest(kSerializer_Read8u_(serializer, &obj->decision)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        obj->xOffset /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->yOffset)); 
        obj->yOffset /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->zAngle)); 
        obj->zAngle /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->qualityValue)); 
        obj->qualityValue /= 1000.0;
        kTest(kSerializer_Read8u_(serializer, &obj->qualityDecision)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_EDGE_MATCH_MSG_ATTR_SIZE_16_36)); 
    }
    kCatch(&status)
    {
        GoEdgeMatchMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(k8u) GoEdgeMatchMsg_Decision(GoEdgeMatchMsg msg)
{
    return GoEdgeMatchMsg_(msg)->decision;
}

GoFx(k64f) GoEdgeMatchMsg_XOffset(GoEdgeMatchMsg msg)
{
    return GoEdgeMatchMsg_(msg)->xOffset;
}

GoFx(k64f) GoEdgeMatchMsg_YOffset(GoEdgeMatchMsg msg)
{
    return GoEdgeMatchMsg_(msg)->yOffset;
}

GoFx(k64f) GoEdgeMatchMsg_ZAngle(GoEdgeMatchMsg msg)
{
    return GoEdgeMatchMsg_(msg)->zAngle;
}

GoFx(k64f) GoEdgeMatchMsg_QualityValue(GoEdgeMatchMsg msg)
{
    return GoEdgeMatchMsg_(msg)->qualityValue;
}

GoFx(k8u) GoEdgeMatchMsg_QualityDecision(GoEdgeMatchMsg msg)
{
    return GoEdgeMatchMsg_(msg)->qualityDecision;
}



/* 
 * GoBoundingBoxMatchMsg
 */

kBeginClass(Go, GoBoundingBoxMatchMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoBoundingBoxMatchMsg, "godat", "6.0.0.0", "17", WriteV17, ReadV17)
    kAddVersion(GoBoundingBoxMatchMsg, "kdat6", "4.0.0.0", "GoBoundingBoxMatchMsg-17", WriteV17, ReadV17)

    //virtual methods
    kAddVMethod(GoBoundingBoxMatchMsg, GoDataMsg, VInit)
    kAddVMethod(GoBoundingBoxMatchMsg, kObject, VInitClone)
    kAddVMethod(GoBoundingBoxMatchMsg, kObject, VRelease)
    kAddVMethod(GoBoundingBoxMatchMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoBoundingBoxMatchMsg_Construct(GoBoundingBoxMatchMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoBoundingBoxMatchMsg), msg)); 

    if (!kSuccess(status = GoBoundingBoxMatchMsg_VInit(*msg, kTypeOf(GoBoundingBoxMatchMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoBoundingBoxMatchMsg_VInit(GoBoundingBoxMatchMsg msg, kType type, kAlloc alloc)
{
    GoBoundingBoxMatchMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_BOUNDING_BOX_MATCH, alloc)); 
    kInitFields_(GoBoundingBoxMatchMsg, msg);

    return kOK; 
}

GoFx(kStatus) GoBoundingBoxMatchMsg_VInitClone(GoBoundingBoxMatchMsg msg, GoBoundingBoxMatchMsg source, kAlloc alloc)
{
    GoBoundingBoxMatchMsgClass* obj = msg; 
    GoBoundingBoxMatchMsgClass* srcObj = source; 

    kCheck(GoBoundingBoxMatchMsg_VInit(msg, kObject_Type_(source), alloc));     

    obj->decision = srcObj->decision;
    obj->xOffset = srcObj->xOffset;
    obj->yOffset = srcObj->yOffset;
    obj->zAngle = srcObj->zAngle;
    obj->lengthValue = srcObj->lengthValue;
    obj->lengthDecision = srcObj->lengthDecision;
    obj->widthValue = srcObj->widthValue;
    obj->widthDecision = srcObj->widthDecision;

    return kOK; 
}

GoFx(kStatus) GoBoundingBoxMatchMsg_VRelease(GoBoundingBoxMatchMsg msg)
{    
    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoBoundingBoxMatchMsg_VSize(GoBoundingBoxMatchMsg msg)
{
    return sizeof(GoBoundingBoxMatchMsgClass); 
}

GoFx(kStatus) GoBoundingBoxMatchMsg_WriteV17(GoBoundingBoxMatchMsg msg, kSerializer serializer)
{
    GoBoundingBoxMatchMsgClass* obj = msg; 

    kCheck(kSerializer_Write16u_(serializer, GO_BOUNDING_BOX_MATCH_MSG_ATTR_SIZE_17_30)); 
    kCheck(kSerializer_Write8u_(serializer, obj->decision));  
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset));  
    kCheck(kSerializer_Write32s_(serializer, obj->yOffset));  
    kCheck(kSerializer_Write32s_(serializer, obj->zAngle));  
    kCheck(kSerializer_Write32s_(serializer, obj->widthValue));  
    kCheck(kSerializer_Write8u_(serializer, obj->widthDecision));  
    kCheck(kSerializer_Write32s_(serializer, obj->lengthValue));  
    kCheck(kSerializer_Write8u_(serializer, obj->lengthDecision));  
    kCheck(kSerializer_Write8u_(serializer, 0));  

    return kOK; 
}

GoFx(kStatus) GoBoundingBoxMatchMsg_ReadV17(GoBoundingBoxMatchMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoBoundingBoxMatchMsgClass* obj = msg; 
    k16u attrSize = 0; 
    kStatus status; 
    k8u reserved;

    kCheck(GoBoundingBoxMatchMsg_VInit(msg, kTypeOf(GoBoundingBoxMatchMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 
        kTest(kSerializer_Read8u_(serializer, &obj->decision)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        obj->xOffset /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->yOffset)); 
        obj->yOffset /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->zAngle)); 
        obj->zAngle /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->widthValue)); 
        obj->widthValue /= 1000.0;
        kTest(kSerializer_Read8u_(serializer, &obj->widthDecision)); 
        kTest(kSerializer_Read32s_(serializer, &obj->lengthValue)); 
        obj->lengthValue /= 1000.0;
        kTest(kSerializer_Read8u_(serializer, &obj->lengthDecision)); 
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_BOUNDING_BOX_MATCH_MSG_ATTR_SIZE_17_30)); 
    }
    kCatch(&status)
    {
        GoBoundingBoxMatchMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(k8u) GoBoundingBoxMatchMsg_Decision(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->decision;
}

GoFx(k64f) GoBoundingBoxMatchMsg_XOffset(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->xOffset;
}

GoFx(k64f) GoBoundingBoxMatchMsg_YOffset(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->yOffset;
}

GoFx(k64f) GoBoundingBoxMatchMsg_ZAngle(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->zAngle;
}

GoFx(k64f) GoBoundingBoxMatchMsg_LengthValue(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->lengthValue;
}

GoFx(k8u) GoBoundingBoxMatchMsg_LengthDecision(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->lengthDecision;
}

GoFx(k64f) GoBoundingBoxMatchMsg_WidthValue(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->widthValue;
}

GoFx(k8u) GoBoundingBoxMatchMsg_WidthDecision(GoBoundingBoxMatchMsg msg)
{
    return GoBoundingBoxMatchMsg_(msg)->widthDecision;
}


/* 
 * GoEllipseMatchMsg
 */

kBeginClass(Go, GoEllipseMatchMsg, GoDataMsg) 
    
    //serialization versions
    kAddVersion(GoEllipseMatchMsg, "godat", "6.0.0.0", "18", WriteV18, ReadV18)
    kAddVersion(GoEllipseMatchMsg, "kdat6", "4.0.0.0", "GoEllipseMatchMsg-18", WriteV18, ReadV18)

    //virtual methods
    kAddVMethod(GoEllipseMatchMsg, GoDataMsg, VInit)
    kAddVMethod(GoEllipseMatchMsg, kObject, VInitClone)
    kAddVMethod(GoEllipseMatchMsg, kObject, VRelease)
    kAddVMethod(GoEllipseMatchMsg, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoEllipseMatchMsg_Construct(GoEllipseMatchMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoEllipseMatchMsg), msg)); 

    if (!kSuccess(status = GoEllipseMatchMsg_VInit(*msg, kTypeOf(GoEllipseMatchMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoEllipseMatchMsg_VInit(GoEllipseMatchMsg msg, kType type, kAlloc alloc)
{
    GoEllipseMatchMsgClass* obj = msg; 

    kCheck(GoDataMsg_Init(msg, type, GO_DATA_MESSAGE_TYPE_ELLIPSE_MATCH, alloc)); 
    kInitFields_(GoEllipseMatchMsg, msg);

    return kOK; 
}

GoFx(kStatus) GoEllipseMatchMsg_VInitClone(GoEllipseMatchMsg msg, GoEllipseMatchMsg source, kAlloc alloc)
{
    GoEllipseMatchMsgClass* obj = msg; 
    GoEllipseMatchMsgClass* srcObj = source; 

    kCheck(GoEllipseMatchMsg_VInit(msg, kObject_Type_(source), alloc));     

    obj->decision = srcObj->decision;
    obj->xOffset = srcObj->xOffset;
    obj->yOffset = srcObj->yOffset;
    obj->zAngle = srcObj->zAngle;
    obj->majorValue = srcObj->majorValue;
    obj->majorDecision = srcObj->majorDecision;
    obj->minorValue = srcObj->minorValue;
    obj->minorDecision = srcObj->minorDecision;

    return kOK; 
}

GoFx(kStatus) GoEllipseMatchMsg_VRelease(GoEllipseMatchMsg msg)
{    
    kCheck(kObject_VRelease_(msg)); 

    return kOK; 
}

GoFx(kSize) GoEllipseMatchMsg_VSize(GoEllipseMatchMsg msg)
{
    return sizeof(GoEllipseMatchMsgClass); 
}

GoFx(kStatus) GoEllipseMatchMsg_WriteV18(GoEllipseMatchMsg msg, kSerializer serializer)
{
    GoEllipseMatchMsgClass* obj = msg; 

    kCheck(kSerializer_Write16u_(serializer, GO_ELLIPSE_MATCH_MSG_ATTR_SIZE_18_30)); 
    kCheck(kSerializer_Write8u_(serializer, obj->decision));  
    kCheck(kSerializer_Write32s_(serializer, obj->xOffset));  
    kCheck(kSerializer_Write32s_(serializer, obj->yOffset));  
    kCheck(kSerializer_Write32s_(serializer, obj->zAngle));  
    kCheck(kSerializer_Write32s_(serializer, obj->minorValue));  
    kCheck(kSerializer_Write8u_(serializer, obj->minorDecision));  
    kCheck(kSerializer_Write32s_(serializer, obj->majorValue));
    kCheck(kSerializer_Write8u_(serializer, obj->majorDecision));
    kCheck(kSerializer_Write8u_(serializer, 0));  

    return kOK; 
}

GoFx(kStatus) GoEllipseMatchMsg_ReadV18(GoEllipseMatchMsg msg, kSerializer serializer, kAlloc alloc)
{
    GoEllipseMatchMsgClass* obj = msg; 
    k16u attrSize = 0; 
    kStatus status; 
    k8u reserved;

    kCheck(GoEllipseMatchMsg_VInit(msg, kTypeOf(GoEllipseMatchMsg), alloc));  

    kTry
    {
        kTest(kSerializer_Read16u_(serializer, &attrSize)); 
        kTest(kSerializer_Read8u_(serializer, &obj->decision)); 
        kTest(kSerializer_Read32s_(serializer, &obj->xOffset)); 
        obj->xOffset /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->yOffset)); 
        obj->yOffset /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->zAngle)); 
        obj->zAngle /= 1000.0;
        kTest(kSerializer_Read32s_(serializer, &obj->minorValue));
        obj->minorValue /= 1000.0;
        kTest(kSerializer_Read8u_(serializer, &obj->minorDecision));
        kTest(kSerializer_Read32s_(serializer, &obj->majorValue)); 
        obj->majorValue /= 1000.0;
        kTest(kSerializer_Read8u_(serializer, &obj->majorDecision));        
        kTest(kSerializer_Read8u_(serializer, &reserved)); 

        kTest(kSerializer_AdvanceRead_(serializer, attrSize - GO_ELLIPSE_MATCH_MSG_ATTR_SIZE_18_30)); 
    }
    kCatch(&status)
    {
        GoEllipseMatchMsg_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(k8u) GoEllipseMatchMsg_Decision(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->decision;
}

GoFx(k64f) GoEllipseMatchMsg_XOffset(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->xOffset;
}

GoFx(k64f) GoEllipseMatchMsg_YOffset(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->yOffset;
}

GoFx(k64f) GoEllipseMatchMsg_ZAngle(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->zAngle;
}

GoFx(k64f) GoEllipseMatchMsg_MajorValue(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->majorValue;
}

GoFx(k8u) GoEllipseMatchMsg_MajorDecision(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->majorDecision;
}

GoFx(k64f) GoEllipseMatchMsg_MinorValue(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->minorValue;
}

GoFx(k8u) GoEllipseMatchMsg_MinorDecision(GoEllipseMatchMsg msg)
{
    return GoEllipseMatchMsg_(msg)->minorDecision;
}
