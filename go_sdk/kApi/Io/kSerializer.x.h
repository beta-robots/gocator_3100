/** 
 * @file    kSerializer.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SERIALIZER_X_H
#define K_API_SERIALIZER_X_H

#include <kApi/Io/kStream.h> 

kBeginHeader()

#define kSERIALIZER_DEFAULT_BUFFER_SIZE             (16384)
#define kSERIALIZER_MAX_BUFFERED_WRITE_SIZE         (2048)

typedef struct kSerializerVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(kSerializer serializer, kType type, kStream stream, kAlloc allocator); 
    kBool (kCall* VCanAutoFlush)(kSerializer serializer); 
    kBool (kCall* VReset)(kSerializer serializer); 
    kStatus (kCall* VWriteObject)(kSerializer serializer, kObject object); 
    kStatus (kCall* VReadObject)(kSerializer serializer, kObject* object, kAlloc allocator); 
    kStatus (kCall* VWriteType)(kSerializer serializer, kType type, kTypeVersion* version); 
    kStatus (kCall* VReadType)(kSerializer serializer, kType* type, kTypeVersion* version); 
} kSerializerVTable; 

typedef struct kSerializerBuffer
{
    struct kSerializerBuffer* next;               //next buffer in linked list
    void* data;                                   //buffer data
    kSize capacity;                               //buffer capacity, in bytes
    kSize written;                                //valid bytes written to buffer
} kSerializerBuffer; 

typedef struct kSerializerWriteSection
{
    kSerializerBuffer* buffer;                    //buffer containing optional write size
    kSize start;                                  //buffer offset to optional write size 
    kType type;                                   //field type
    kBool includeSize;                            //include size field in recorded size?
} kSerializerWriteSection; 

kDeclareValue(k, kSerializerWriteSection, kValue)

typedef struct kSerializerClass
{    
    kObjectClass base; 

    kStream stream;                                 //stream for reading/writing
    kAlloc readAlloc;                               //read allocator, used during kSerializer_ReadObject

    kBool swap;                                     //should the serializer reverse endianness?
    k32u sizeEncoding;                              //kSize and kSSize encoding (4 or 8 bytes). 
    kText16 format;                                 //format name (e.g. Dat5, Dat6)
    kVersion formatVersion;                         //global serialization format version                          
    kMap assemblyVersions;                          //maps assemblies to versions (kMap<kAssembly, kVersion>)

    kSerializerBuffer* activeBuffers;               //head of active write buffer linked list
    kSerializerBuffer* currentBuffer;               //tail of active write buffer linked list
    kSerializerBuffer* freeBuffers;                 //head of free write buffer linked list
    kSize bufferSize;                               //size of each write buffer, excluding header

    kArrayList writeSections;                       //stack of optional write sections (kArrayList<kSerializerWriteSection>)
    kArrayList readSections;                        //stack of optional read sections (kArrayList<k64u>)

    k8u temp8u;                                     //temporary variable, used in macros
    k16u temp16u;                                   //temporary variable, used in macros
    k32u temp32u;                                   //temporary variable, used in macros
    k64u temp64u;                                   //temporary variable, used in macros
    k64s temp32s;                                   //temporary variable, used in macros
    k64s temp64s;                                   //temporary variable, used in macros
    k32f temp32f;                                   //temporary variable, used in macros
    k64f temp64f;                                   //temporary variable, used in macros
    kStatus tempStatus;                             //temporary variable, used in macros
} kSerializerClass;

kDeclareVirtualClass(k, kSerializer, kObject)

kFx(kStatus) kSerializer_VInit(kSerializer serializer, kType type, kStream stream, kAlloc allocator); 
kFx(kStatus) kSerializer_VRelease(kSerializer serializer); 

kFx(kBool) kSerializer_SetBigEndian(kSerializer serializer, kBool isBigEndian); 
kFx(kBool) kSerializer_IsBigEndian(kSerializer serializer); 

kFx(kBool) kSerializer_CanAutoFlush(kSerializer serializer); 
kFx(kStatus) kSerializer_VCanAutoFlush(kSerializer serializer); 
kFx(kStatus) kSerializer_VReset(kSerializer serializer); 

kFx(kStatus) kSerializer_VWriteObject(kSerializer serializer, kObject object); 
kFx(kStatus) kSerializer_VReadObject(kSerializer serializer, kObject* object, kAlloc allocator); 
kFx(kStatus) kSerializer_VWriteType(kSerializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) kSerializer_VReadType(kSerializer serializer, kType* type, kTypeVersion* version); 

kFx(kStatus) kSerializer_AllocateBuffer(kSerializer serializer); 
kFx(kStatus) kSerializer_InsertHeader(kSerializer serializer); 
kFx(kStatus) kSerializer_AddBuffer(kSerializer serializer); 
kFx(kStatus) kSerializer_ClearBuffers(kSerializer serializer);
kFx(kStatus) kSerializer_FlushBuffers(kSerializer serializer); 

kFx(kStatus) kSerializer_FindCompatibleVersion(kSerializer serializer, kType type, kTypeVersion* version); 

kFx(kStatus) kSerializer_Write1N(kSerializer serializer, const void* items, kSize count); 
kFx(kStatus) kSerializer_Write2N(kSerializer serializer, const void* items, kSize count); 
kFx(kStatus) kSerializer_Write4N(kSerializer serializer, const void* items, kSize count); 
kFx(kStatus) kSerializer_Write8N(kSerializer serializer, const void* items, kSize count); 

kFx(kStatus) kSerializer_Read1N(kSerializer serializer, void* items, kSize count); 
kFx(kStatus) kSerializer_Read2N(kSerializer serializer, void* items, kSize count); 
kFx(kStatus) kSerializer_Read4N(kSerializer serializer, void* items, kSize count); 
kFx(kStatus) kSerializer_Read8N(kSerializer serializer, void* items, kSize count); 

#define kSerializer_(S)                                     (kCast(kSerializerClass*, S))
#define kSerializer_Cast_(S)                                (kCastClass_(kSerializer, S))
#define kSerializer_VTable_(S)                              (kCastVTable_(kSerializer, S))

#define kxSerializer_Stream_(S)                             (kSerializer_(S)->stream)

#define kxSerializer_Flush_(S)                              (kSerializer_Flush(S))
#define kxSerializer_Reset_(S)                              (kSerializer_VTable_(S)->VReset(S))

#define kSerializer_CanAutoFlush_(S)                        (kSerializer_VTable_(S)->VCanAutoFlush(S))

#define kxSerializer_WriteObject_(S, D)                     (kSerializer_VTable_(S)->VWriteObject(S, D))
#define kxSerializer_ReadObject_(S, D, A)                   (kSerializer_VTable_(S)->VReadObject(S, D, A))

#define kxSerializer_WriteType_(S, T, V)                    (kSerializer_VTable_(S)->VWriteType(S, T, V)) 
#define kxSerializer_ReadType_(S, T, V)                     (kSerializer_VTable_(S)->VReadType(S, T, V))                  
                                                            
#define kxSerializer_WriteItems_(S, T, V, I, C)             (kSerializer_WriteItems(S, T, V, I, C))
#define kxSerializer_ReadItems_(S, T, V, I, C)              (kSerializer_ReadItems(S, T, V, I, C))                                                             

#define kxSerializer_BeginWrite_(S, T, I)                   (kSerializer_BeginWrite(S, T, I))
#define kxSerializer_EndWrite_(S)                           (kSerializer_EndWrite(S))                                                            

#define kxSerializer_BeginRead_(S, T, I)                    (kSerializer_BeginRead(S, T, I))
#define kxSerializer_EndRead_(S)                            (kSerializer_EndRead(S))

#define kxSerializer_ReadCompleted_(S)                                                          \
    (kStream_BytesRead_(kSerializer_Stream_(S)) >=                                              \
        kArrayList_As_(kSerializer_(S)->readSections,                                           \
            kArrayList_Count_(kSerializer_(S)->readSections)-1,                                 \
            k64u))

#define kxSerializer_AdvanceRead_(S, O)                                                         \
    (((O) == 0) ? kOK : kSerializer_AdvanceRead(S, O))

#define kSerializer_Buffer_(S)                                                                  \
    ((kByte*)kSerializer_(S)->currentBuffer->data +                                             \
        kSerializer_(S)->currentBuffer->written)

#define kSerializer_CanBuffer_(S, C)                                                            \
    ((C) <= (kSerializer_(S)->currentBuffer->capacity -                                         \
             kSerializer_(S)->currentBuffer->written))

#define kSerializer_Write1_(S, D)                                                               \
    (kSerializer_CanBuffer_(S, 1) ?                                                             \
     ((kSerializer_Buffer_(S)[0] = *(kByte*)(D),                                                \
      kSerializer_(S)->currentBuffer->written += 1, kOK)) :                                     \
     (kSerializer_Write1N(S, D, 1)))

#define kSerializer_Write2_(S, D)                                                               \
    (kSerializer_CanBuffer_(S, 2) ?                                                             \
     ((kSerializer_Reorder2_(S, kSerializer_Buffer_(S), D),                                     \
      kSerializer_(S)->currentBuffer->written += 2, kOK)) :                                     \
     (kSerializer_Write2N(S, D, 1)))

#define kSerializer_Write4_(S, D)                                                               \
    (kSerializer_CanBuffer_(S, 4) ?                                                             \
     ((kSerializer_Reorder4_(S, kSerializer_Buffer_(S), D),                                     \
      kSerializer_(S)->currentBuffer->written += 4, kOK)) :                                     \
     (kSerializer_Write4N(S, D, 1)))

#define kSerializer_Write8_(S, D)                                                               \
    (kSerializer_CanBuffer_(S, 8) ?                                                             \
     ((kSerializer_Reorder8_(S, kSerializer_Buffer_(S), D),                                     \
      kSerializer_(S)->currentBuffer->written += 8, kOK)) :                                     \
     (kSerializer_Write8N(S, D, 1)))

#define kSerializer_Reorder2_(S, DEST, SRC)                                                     \
    (kSerializer_(S)->swap ?                                                                    \
     kItemSwap2_(DEST, SRC) :                                                                   \
     kItemCopy2_(DEST, SRC))

#define kSerializer_Reorder4_(S, DEST, SRC)                                                     \
    (kSerializer_(S)->swap ?                                                                    \
     kItemSwap4_(DEST, SRC) :                                                                   \
     kItemCopy4_(DEST, SRC))

#define kSerializer_Reorder8_(S, DEST, SRC)                                                     \
    (kSerializer_(S)->swap ?                                                                    \
     kItemSwap8_(DEST, SRC) :                                                                   \
     kItemCopy8_(DEST, SRC))

#define kxSerializer_WriteByte_(S, D)                               kSerializer_Write8u_(S, D)
#define kxSerializer_WriteByteArray_(S, D, C)                       kSerializer_Write8uArray_(S, D, C)
#define kxSerializer_ReadByte_(S, D)                                kSerializer_Read8u_(S, D)
#define kxSerializer_ReadByteArray_(S, D, C)                        kSerializer_Read8uArray_(S, D, C)

#define kxSerializer_WriteChar_(S, D)                               kSerializer_Write8u_(S, D)
#define kxSerializer_WriteCharArray_(S, D, C)                       kSerializer_Write8uArray_(S, D, C)
#define kxSerializer_ReadChar_(S, D)                                kSerializer_Read8u_(S, D)
#define kxSerializer_ReadCharArray_(S, D, C)                        kSerializer_Read8uArray_(S, D, C)

#define kxSerializer_Write8u_(S, D)                                                             \
    (kSerializer_(S)->temp8u = (D),                                                             \
     kSerializer_Write1_(S, (kByte*)&kSerializer_(S)->temp8u))

#define kxSerializer_Write8uArray_(S, D, C)                                                     \
    kSerializer_Write1N(S, D, C)

#define kxSerializer_Read8u_(S, D)                                                              \
    (kStream_Read1_(kSerializer_Stream_(S), (D)))

#define kxSerializer_Read8uArray_(S, D, C)                                                      \
    kStream_Read_(kSerializer_Stream_(S), (D), (C))

#define kxSerializer_Write8s_(S, D)                                 kSerializer_Write8u_(S, D)
#define kxSerializer_Write8sArray_(S, D, C)                         kSerializer_Write8uArray_(S, D, C)
#define kxSerializer_Read8s_(S, D)                                  kSerializer_Read8u_(S, D)
#define kxSerializer_Read8sArray_(S, D, C)                          kSerializer_Read8uArray_(S, D, C)
                                                                    
#define kxSerializer_Write16u_(S, D)                                                            \
    (kSerializer_(S)->temp16u = (k16u)(D),                                                      \
     kSerializer_Write2_(S, &kSerializer_(S)->temp16u))

#define kxSerializer_Write16uArray_(S, D, C)                                                    \
    kSerializer_Write2N(S, D, C)

#define kxSerializer_Read16u_(S, D)                                                             \
    (kSerializer_(S)->tempStatus =                                                              \
       kStream_Read2_(kSerializer_Stream_(S), &kSerializer_(S)->temp16u),                       \
     kSerializer_Reorder2_((S), (D), &kSerializer_(S)->temp16u),                                \
     kSerializer_(S)->tempStatus)

#define kxSerializer_Read16uArray_(S, D, C)                                                     \
    kSerializer_Read2N(S, D, C)

#define kxSerializer_Write16s_(S, D)                                kSerializer_Write16u_(S, D)
#define kxSerializer_Write16sArray_(S, D, C)                        kSerializer_Write16uArray_(S, D, C)
#define kxSerializer_Read16s_(S, D)                                 kSerializer_Read16u_(S, D)
#define kxSerializer_Read16sArray_(S, D, C)                         kSerializer_Read16uArray_(S, D, C)
                                                                    
#define kxSerializer_Write32u_(S, D)                                                            \
    (kSerializer_(S)->temp32u = (k32u)(D),                                                      \
     kSerializer_Write4_(S, &kSerializer_(S)->temp32u))

#define kxSerializer_Write32uArray_(S, D, C)                                                    \
    kSerializer_Write4N(S, D, C)

#define kxSerializer_Read32u_(S, D)                                                             \
    (kSerializer_(S)->tempStatus =                                                              \
       kStream_Read4_(kSerializer_Stream_(S), &kSerializer_(S)->temp32u),                       \
     kSerializer_Reorder4_((S), (D), &kSerializer_(S)->temp32u),                                \
     kSerializer_(S)->tempStatus)

#define kxSerializer_Read32uArray_(S, D, C)                                                     \
    kSerializer_Read4N(S, D, C)

#define kxSerializer_Write32s_(S, D)                                kSerializer_Write32u_(S, D)
#define kxSerializer_Write32sArray_(S, D, C)                        kSerializer_Write32uArray_(S, D, C)
#define kxSerializer_Read32s_(S, D)                                 kSerializer_Read32u_(S, D)
#define kxSerializer_Read32sArray_(S, D, C)                         kSerializer_Read32uArray_(S, D, C)
                                                                    
#define kxSerializer_Write64u_(S, D)                                                            \
    (kSerializer_(S)->temp64u = (k64u)(D),                                                      \
     kSerializer_Write8_(S, &kSerializer_(S)->temp64u))

#define kxSerializer_Write64uArray_(S, D, C)                                                    \
    kSerializer_Write8N(S, D, C)

#define kxSerializer_Read64u_(S, D)                                                             \
    (kSerializer_(S)->tempStatus =                                                              \
       kStream_Read8_(kSerializer_Stream_(S), &kSerializer_(S)->temp64u),                       \
     kSerializer_Reorder8_((S), (D), &kSerializer_(S)->temp64u),                                \
     kSerializer_(S)->tempStatus)

#define kxSerializer_Read64uArray_(S, D, C)                                                     \
    kSerializer_Read8N(S, D, C)

#define kxSerializer_Write64s_(S, D)                                kSerializer_Write64u_(S, D)
#define kxSerializer_Write64sArray_(S, D, C)                        kSerializer_Write64uArray_(S, D, C)
#define kxSerializer_Read64s_(S, D)                                 kSerializer_Read64u_(S, D)
#define kxSerializer_Read64sArray_(S, D, C)                         kSerializer_Read64uArray_(S, D, C)
                                      
#define kxSerializer_Write32f_(S, D)                                                            \
    (kSerializer_(S)->temp32f = (k32f)(D),                                                      \
     kSerializer_Write4_(S, &kSerializer_(S)->temp32f))

#define kxSerializer_Write32fArray_(S, D, C)                                                    \
    kSerializer_Write4N(S, D, C)

#define kxSerializer_Read32f_(S, D)                                                             \
    (kSerializer_(S)->tempStatus =                                                              \
       kStream_Read4_(kSerializer_Stream_(S), &kSerializer_(S)->temp32f),                       \
     kSerializer_Reorder4_((S), (D), &kSerializer_(S)->temp32f),                                \
     kSerializer_(S)->tempStatus)

#define kxSerializer_Read32fArray_(S, D, C)                                                     \
    kSerializer_Read4N(S, D, C)
          
#define kxSerializer_Write64f_(S, D)                                                            \
    (kSerializer_(S)->temp64f = (k64f)(D),                                                      \
     kSerializer_Write8_(S, &kSerializer_(S)->temp64f))

#define kxSerializer_Write64fArray_(S, D, C)                                                    \
    kSerializer_Write8N(S, D, C)

#define kxSerializer_Read64f_(S, D)                                                             \
    (kSerializer_(S)->tempStatus =                                                              \
       kStream_Read8_(kSerializer_Stream_(S), &kSerializer_(S)->temp64f),                       \
     kSerializer_Reorder8_((S), (D), &kSerializer_(S)->temp64f),                                \
     kSerializer_(S)->tempStatus)

#define kxSerializer_Read64fArray_(S, D, C)                                                     \
    kSerializer_Read8N(S, D, C)

#if (K_POINTER_SIZE == 4)

#define kxSerializer_WriteSize_(S, D)                                                           \
    ((kSerializer_(S)->sizeEncoding == 4) ?                                                     \
       kSerializer_Write32u_(S, D) :                                                            \
       kSerializer_Write64u_(S, D))

#elif (K_POINTER_SIZE == 8)

#define kxSerializer_WriteSize_(S, D)                                                           \
    ((kSerializer_(S)->sizeEncoding == 8) ? kSerializer_Write64u_(S, D) :                       \
        (((D) > k32U_MAX) ? kERROR_FORMAT : kSerializer_Write32u_(S, D)))

#endif

#define kxSerializer_WriteSizeArray_(S, D, C)                                                   \
    kSerializer_WriteSizeArray(S, D, C)


#if (K_POINTER_SIZE == 4)

#define kxSerializer_ReadSize_(S, D)                                                            \
    ((kSerializer_(S)->sizeEncoding == 4) ? kSerializer_Read32u_(S, D) :                        \
       (kSerializer_(S)->tempStatus = kSerializer_Read64u_(S, &kSerializer_(S)->temp64u),       \
         (kIsError(kSerializer_(S)->tempStatus) ?  kSerializer_(S)->tempStatus :                \
           ((kSerializer_(S)->temp64u > k32U_MAX) ? kERROR_FORMAT :                             \
              (kSetAs_(D, kSerializer_(S)->temp64u, kSize), kOK)))))
            
#elif (K_POINTER_SIZE == 8)

#define kxSerializer_ReadSize_(S, D)                                                            \
    ((kSerializer_(S)->sizeEncoding == 8) ? kSerializer_Read64u_(S, D) :                        \
       (kSerializer_(S)->tempStatus = kSerializer_Read32u_(S, &kSerializer_(S)->temp32u),       \
         (kIsError(kSerializer_(S)->tempStatus) ?  kSerializer_(S)->tempStatus :                \
           (kSetAs_(D, kSerializer_(S)->temp32u, kSize), kOK))))

#endif

#define kxSerializer_ReadSizeArray_(S, D, C)                                                    \
    kSerializer_ReadSizeArray(S, D, C)

#if (K_POINTER_SIZE == 4)

#define kxSerializer_WriteSSize_(S, D)                                                          \
    ((kSerializer_(S)->sizeEncoding == 4) ?                                                     \
       kSerializer_Write32s_(S, D) :                                                            \
       kSerializer_Write64s_(S, D))

#elif (K_POINTER_SIZE == 8)

#define kxSerializer_WriteSSize_(S, D)                                                          \
    ((kSerializer_(S)->sizeEncoding == 8) ? kSerializer_Write64s_(S, D) :                       \
        ((((D) > k32S_MAX) || ((D) < k32S_MIN)) ? kERROR_FORMAT :                               \
            kSerializer_Write32s_(S, D)))

#endif

#define kxSerializer_WriteSSizeArray_(S, D, C)                                                   \
    kSerializer_WriteSSizeArray(S, D, C)

#if (K_POINTER_SIZE == 4)

#define kxSerializer_ReadSSize_(S, D)                                                           \
    ((kSerializer_(S)->sizeEncoding == 4) ? kSerializer_Read32s_(S, D) :                        \
       (kSerializer_(S)->tempStatus = kSerializer_Read64s_(S, &kSerializer_(S)->temp64s),       \
         (kIsError(kSerializer_(S)->tempStatus) ?  kSerializer_(S)->tempStatus :                \
           (((kSerializer_(S)->temp64s > k32S_MAX) || (kSerializer_(S)->temp64s < k32S_MIN)) ?  \
              kERROR_FORMAT : (kSetAs_(D, kSerializer_(S)->temp64s, kSSize), kOK)))))
            
#elif (K_POINTER_SIZE == 8)

#define kxSerializer_ReadSSize_(S, D)                                                           \
    ((kSerializer_(S)->sizeEncoding == 8) ? kSerializer_Read64s_(S, D) :                        \
       (kSerializer_(S)->tempStatus = kSerializer_Read32s_(S, &kSerializer_(S)->temp32s),       \
         (kIsError(kSerializer_(S)->tempStatus) ?  kSerializer_(S)->tempStatus :                \
           (kSetAs_(D, kSerializer_(S)->temp32s, kSSize), kOK))))

#endif

#define kxSerializer_ReadSSizeArray_(S, D, C)                                                    \
    kSerializer_ReadSSizeArray(S, D, C)


/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kSerializer_Destroy              kObject_Destroy
#   define kSerializer_Write                kSerializer_WriteObject
#   define kSerializer_WriteBytes           kSerializer_WriteByteArray
#   define kSerializer_ReadBytes            kSerializer_ReadByteArray

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kSerializer_ReadObject5(S, O)    kSerializer_ReadObject(S, O, kNULL)

#endif

kEndHeader()

#endif
