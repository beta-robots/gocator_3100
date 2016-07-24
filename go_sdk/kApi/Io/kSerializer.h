/** 
 * @file    kSerializer.h
 * @brief   Declares the kSerializer class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SERIALIZER_H
#define K_API_SERIALIZER_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kSerializer
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Base class for serialization/deserialization classes. 
 * 
 * kSerializer is a base class for serialization/deserialization classes, where each derived 
 * class typically implements a specific object serialization format. 
 * 
 * kSerializer itself is also an instantiable class. The kSerializer base class does not provide
 * the ability to serialize/deserialize objects or type information, but can be used to 
 * read/write primitive data and arrays. 
 *
 * kSerializer does not automatically flush its internal write buffer when the serializer is destroyed.
 * The Flush method must be used to ensure that all bytes are written to the underlying stream.
 *
 */
//typedef kObject kSerializer;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a serializer object using the specified serialization format. 
 *
 * This method is a factory constructor; the type of serializer to be instantiated is provided 
 * as an argument. If no serializer type is provided, the default serializer will be used 
 * (currently kSerializer). 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   stream          Stream for reading/writing. 
 * @param   serializerType  Serializer type (or kNULL for default). 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_Construct(kSerializer* serializer, kStream stream, kType serializerType, kAlloc allocator); 

/** 
 * Loads an object from file using the specified serializer type.
 *
 * @public                  @memberof kSerializer
 * @param   object          Receives deserialized object.
 * @param   serializerType  Type of serializer to use (e.g. kDat6Serializer). 
 * @param   filePath        Path of the file to load. 
 * @param   readAlloc       Memory allocator to use for loaded object (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_LoadObject(kObject* object, kType serializerType, const kChar* filePath, kAlloc readAlloc);

/** 
 * Saves an object to file using the specified serializer type.
 *
 * @public                  @memberof kSerializer
 * @param   object          Object to be serialized.
 * @param   serializerType  Type of serializer to use (e.g. kDat6Serializer). 
 * @param   filePath        Path of the file to save. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_SaveObject(kObject object, kType serializerType, const kChar* filePath); 

/** 
 * Sets the version to use when serializing types. 
 * 
 * Some serialization formats use a versioning scheme in which a separate version is given 
 * for each assembly, while others use a single version that is applied to types from all 
 * assemblies. If a single version is used for assemblies, set the assembly argument to null. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   assembly        Assembly object (or kNULL). 
 * @param   version         Desired version. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_SetVersion(kSerializer serializer, kAssembly assembly, kVersion version); 

/** 
 * Explicitly sets the number of bytes used to encode/decode kSize and kSSize values. 
 * 
 * The default size encoding for kSerializer is 4 bytes. This value can be overridden by derived classes. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   byteCount       Number of bytes to use when encoding size values (accepts 4 or 8). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_SetSizeEncoding(kSerializer serializer, k32u byteCount); 

/** 
 * Writes an object to the underlying stream. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   object          Object to be serialized. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_WriteObject(kSerializer serializer, kObject object); 

/** 
 * Reads an object from the underlying stream. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   object          Receives deserialized object. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_ReadObject(kSerializer serializer, kObject* object, kAlloc allocator); 

/** 
 * Writes a null-terminated kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        String to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteText(kSerializer serializer, const kChar* data); 

/** 
 * Reads a null-terminated kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the string.
 * @param   capacity    Capacity of data string (including null terminator; must be greater than zero). 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadText(kSerializer serializer, kChar* data, kSize capacity); 

/** 
 * Writes a kByte value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteByte(kSerializer serializer, kByte data); 

/** 
 * Writes a kByte array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteByteArray(kSerializer serializer, const kByte* data, kSize count); 

/** 
 * Reads a kByte value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadByte(kSerializer serializer, kByte* data);

/** 
 * Reads a kByte array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadByteArray(kSerializer serializer, kByte* data, kSize count); 

/** 
 * Writes a kChar value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteChar(kSerializer serializer, kChar data); 

/** 
 * Writes a kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteCharArray(kSerializer serializer, const kChar* data, kSize count); 

/** 
 * Reads a kChar value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadChar(kSerializer serializer, kChar* data);

/** 
 * Reads a kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadCharArray(kSerializer serializer, kChar* data, kSize count); 

/** 
 * Writes a k8u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write8u(kSerializer serializer, k8u data); 

/** 
 * Writes a k8u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write8uArray(kSerializer serializer, const k8u* data, kSize count); 

/** 
 * Reads a k8u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read8u(kSerializer serializer, k8u* data);

/** 
 * Reads a k8u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read8uArray(kSerializer serializer, k8u* data, kSize count); 

/** 
 * Writes a k8s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write8s(kSerializer serializer, k8s data); 

/** 
 * Writes a k8s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write8sArray(kSerializer serializer, const k8s* data, kSize count); 

/** 
 * Reads a k8s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read8s(kSerializer serializer, k8s* data); 

/** 
 * Reads a k8s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read8sArray(kSerializer serializer, k8s* data, kSize count); 

/** 
 * Writes a k16u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write16u(kSerializer serializer, k16u data); 

/** 
 * Writes a k16u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write16uArray(kSerializer serializer, const k16u* data, kSize count); 

/** 
 * Reads a k16u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read16u(kSerializer serializer, k16u* data);

/** 
 * Reads a k16u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read16uArray(kSerializer serializer, k16u* data, kSize count);

/** 
 * Writes a k16s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write16s(kSerializer serializer, k16s data); 

/** 
 * Writes a k16s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write16sArray(kSerializer serializer, const k16s* data, kSize count); 


/** 
 * Reads a k16s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read16s(kSerializer serializer, k16s* data); 

/** 
 * Reads a k16s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read16sArray(kSerializer serializer, k16s* data, kSize count); 


/** 
 * Writes a k32u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write32u(kSerializer serializer, k32u data); 

/** 
 * Writes a k32u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write32uArray(kSerializer serializer, const k32u* data, kSize count); 

/** 
 * Reads a k32u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read32u(kSerializer serializer, k32u* data); 

/** 
 * Reads a k32u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read32uArray(kSerializer serializer, k32u* data, kSize count); 

/** 
 * Writes a k32s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write32s(kSerializer serializer, k32s data); 

/** 
 * Writes a k32s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write32sArray(kSerializer serializer, const k32s* data, kSize count); 

/** 
 * Reads a k32s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read32s(kSerializer serializer, k32s* data);

/** 
 * Reads a k32s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read32sArray(kSerializer serializer, k32s* data, kSize count);

/** 
 * Writes a k64u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write64u(kSerializer serializer, k64u data); 

/** 
 * Writes a k64u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write64uArray(kSerializer serializer, const k64u* data, kSize count); 

/** 
 * Reads a k64u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read64u(kSerializer serializer, k64u* data); 

/** 
 * Reads a k64u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read64uArray(kSerializer serializer, k64u* data, kSize count); 

/** 
 * Writes a k64s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write64s(kSerializer serializer, k64s data); 

/** 
 * Writes a k64s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write64sArray(kSerializer serializer, const k64s* data, kSize count); 

/** 
 * Reads a k64s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read64s(kSerializer serializer, k64s* data); 

/** 
 * Reads a k64s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read64sArray(kSerializer serializer, k64s* data, kSize count); 

/** 
 * Writes a k32f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write32f(kSerializer serializer, k32f data); 

/** 
 * Writes a k32f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write32fArray(kSerializer serializer, const k32f* data, kSize count); 

/** 
 * Reads a k32f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read32f(kSerializer serializer, k32f* data); 

/** 
 * Reads a k32f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read32fArray(kSerializer serializer, k32f* data, kSize count); 

/** 
 * Writes a k64f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write64f(kSerializer serializer, k64f data);

/** 
 * Writes a k64f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Write64fArray(kSerializer serializer, const k64f* data, kSize count);

/** 
 * Reads a k64f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read64f(kSerializer serializer, k64f* data); 

/** 
 * Reads a k64f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Read64fArray(kSerializer serializer, k64f* data, kSize count); 

/** 
 * Writes a kSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size value. 
 * 
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Size value. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteSize(kSerializer serializer, kSize data); 

/** 
 * Writes a kSize array. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteSizeArray(kSerializer serializer, const kSize* data, kSize count);

/** 
 * Reads a kSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives size value. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadSize(kSerializer serializer, kSize* data); 

/** 
 * Reads a kSize array. 
 * 
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadSizeArray(kSerializer serializer, kSize* data, kSize count); 

/** 
 * Writes a kSSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size value. 
 * 
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        SSize value. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteSSize(kSerializer serializer, kSSize data); 

/** 
 * Writes a kSSize array. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteSSizeArray(kSerializer serializer, const kSSize* data, kSize count);

/** 
 * Reads a kSSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives size value. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadSSize(kSerializer serializer, kSSize* data); 

/** 
 * Reads a kSSize array. 
 * 
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadSSizeArray(kSerializer serializer, kSSize* data, kSize count); 

/** 
 * Writes a type code. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Type instance. 
 * @param   version     Receives a reference to the type version that was written. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteType(kSerializer serializer, kType type, kTypeVersion* version); 

/** 
 * Reads a type code. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Receives type instance.
 * @param   version     Receives a reference to the type version that was read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadType(kSerializer serializer, kType* type, kTypeVersion* version); 

/** 
 * Writes an array of values or objects. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Type of items. 
 * @param   version     Type version.  
 * @param   items       Items to serialize. 
 * @param   count       Count of items. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteItems(kSerializer serializer, kType type, kTypeVersion version, const void* items, kSize count); 

/** 
 * Reads an array of values or objects. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Type of items. 
 * @param   version     Type version.  
 * @param   items       Receives deserialized items.
 * @param   count       Count of items. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadItems(kSerializer serializer, kType type, kTypeVersion version, void* items, kSize count); 

/** 
 * Begins writing a measured section of data, using an 8, 16, 32, or 64-bit integer
 * to record the size. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   sizeType        Type of size field (k8u, k16u, k32u, or k64u)
 * @param   includeSize     Include the size of the size field in the written size?
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_BeginWrite(kSerializer serializer, kType sizeType, kBool includeSize); 

/** 
 * Ends writing a measured data section. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_EndWrite(kSerializer serializer); 

/** 
 * Begins reading a measured data section. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   sizeType        Type of size field (k8u, k16u, k32u, or k64u)
 * @param   includeSize     Size of the size field was included in the recorded size?
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_BeginRead(kSerializer serializer, kType sizeType, kBool includeSize); 

/** 
 * Ends reading a measured data section. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_EndRead(kSerializer serializer); 

/** 
 * Determines whether the current measured read section has more bytes.
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              kTRUE if no more bytes available; otherwise, kFALSE.
 */
kFx(kBool) kSerializer_ReadCompleted(kSerializer serializer); 

/** 
 * Reads and discards a specified number of bytes. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   offset      Number of bytes to read and discard. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_AdvanceRead(kSerializer serializer, kSize offset); 

/** 
 * Flushes the serializer write buffer to the underlying stream.
 * 
 * kSerializer does not automatically flush its internal write buffer when the serializer is destroyed. 
 * The Flush method must be used to ensure that all bytes are written to the underlying stream.
 * 
 * The kSerializer_Flush method calls kStream_Flush on the underlying stream.
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Flush(kSerializer serializer); 

/** 
 * Discards any streaming context accumulated by the serializer. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_Reset(kSerializer serializer); 

/** 
 * Gets the underlying stream. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Stream object. 
 */
kFx(kStream) kSerializer_Stream(kSerializer serializer); 

/** @relates kSerializer @{ */

#define kSerializer_WriteObject_(S, D)             kxSerializer_WriteObject_(S, D)              ///< Macro version of kSerializer_WriteObject.
#define kSerializer_ReadObject_(S, D, A)           kxSerializer_ReadObject_(S, D, A)            ///< Macro version of kSerializer_ReadObject.

#define kSerializer_WriteByte_(S, D)               kxSerializer_WriteByte_(S, D)                ///< Macro version of kSerializer_WriteByte.
#define kSerializer_WriteByteArray_(S, D, C)       kxSerializer_WriteByteArray_(S, D, C)        ///< Macro version of kSerializer_WriteByteArray.    
#define kSerializer_ReadByte_(S, D)                kxSerializer_ReadByte_(S, D)                 ///< Macro version of kSerializer_ReadByte.
#define kSerializer_ReadByteArray_(S, D, C)        kxSerializer_ReadByteArray_(S, D, C)         ///< Macro version of kSerializer_ReadByteArray.    

#define kSerializer_WriteChar_(S, D)               kxSerializer_WriteChar_(S, D)                ///< Macro version of kSerializer_WriteChar.
#define kSerializer_WriteCharArray_(S, D, C)       kxSerializer_WriteCharArray_(S, D, C)        ///< Macro version of kSerializer_WriteCharArray.
#define kSerializer_ReadChar_(S, D)                kxSerializer_ReadChar_(S, D)                 ///< Macro version of kSerializer_ReadChar.
#define kSerializer_ReadCharArray_(S, D, C)        kxSerializer_ReadCharArray_(S, D, C)         ///< Macro version of kSerializer_ReadCharArray.

#define kSerializer_Write8u_(S, D)                 kxSerializer_Write8u_(S, D)                  ///< Macro version of kSerializer_Write8u.
#define kSerializer_Write8uArray_(S, D, C)         kxSerializer_Write8uArray_(S, D, C)          ///< Macro version of kSerializer_Write8uArray.
#define kSerializer_Read8u_(S, D)                  kxSerializer_Read8u_(S, D)                   ///< Macro version of kSerializer_Read8u.
#define kSerializer_Read8uArray_(S, D, C)          kxSerializer_Read8uArray_(S, D, C)           ///< Macro version of kSerializer_Read8uArray.
                                                   
#define kSerializer_Write8s_(S, D)                 kxSerializer_Write8s_(S, D)                  ///< Macro version of kSerializer_Write8s.
#define kSerializer_Write8sArray_(S, D, C)         kxSerializer_Write8sArray_(S, D, C)          ///< Macro version of kSerializer_Write8sArray.
#define kSerializer_Read8s_(S, D)                  kxSerializer_Read8s_(S, D)                   ///< Macro version of kSerializer_Read8s.
#define kSerializer_Read8sArray_(S, D, C)          kxSerializer_Read8sArray_(S, D, C)           ///< Macro version of kSerializer_Read8sArray.

#define kSerializer_Write16u_(S, D)                kxSerializer_Write16u_(S, D)                 ///< Macro version of kSerializer_Write16u.
#define kSerializer_Write16uArray_(S, D, C)        kxSerializer_Write16uArray_(S, D, C)         ///< Macro version of kSerializer_Write16uArray.
#define kSerializer_Read16u_(S, D)                 kxSerializer_Read16u_(S, D)                  ///< Macro version of kSerializer_Read16u.
#define kSerializer_Read16uArray_(S, D, C)         kxSerializer_Read16uArray_(S, D, C)          ///< Macro version of kSerializer_Read16uArray.

#define kSerializer_Write16s_(S, D)                kxSerializer_Write16s_(S, D)                 ///< Macro version of kSerializer_Write16s.
#define kSerializer_Write16sArray_(S, D, C)        kxSerializer_Write16sArray_(S, D, C)         ///< Macro version of kSerializer_Write16sArray.
#define kSerializer_Read16s_(S, D)                 kxSerializer_Read16s_(S, D)                  ///< Macro version of kSerializer_Read16s.
#define kSerializer_Read16sArray_(S, D, C)         kxSerializer_Read16sArray_(S, D, C)          ///< Macro version of kSerializer_Read16sArray.    

#define kSerializer_Write32u_(S, D)                kxSerializer_Write32u_(S, D)                 ///< Macro version of kSerializer_Write32u.
#define kSerializer_Write32uArray_(S, D, C)        kxSerializer_Write32uArray_(S, D, C)         ///< Macro version of kSerializer_Write32uArray.
#define kSerializer_Read32u_(S, D)                 kxSerializer_Read32u_(S, D)                  ///< Macro version of kSerializer_Read32u.
#define kSerializer_Read32uArray_(S, D, C)         kxSerializer_Read32uArray_(S, D, C)          ///< Macro version of kSerializer_Read32uArray.

#define kSerializer_Write32s_(S, D)                kxSerializer_Write32s_(S, D)                 ///< Macro version of kSerializer_Write32s.
#define kSerializer_Write32sArray_(S, D, C)        kxSerializer_Write32sArray_(S, D, C)         ///< Macro version of kSerializer_Write32sArray.
#define kSerializer_Read32s_(S, D)                 kxSerializer_Read32s_(S, D)                  ///< Macro version of kSerializer_Read32s.
#define kSerializer_Read32sArray_(S, D, C)         kxSerializer_Read32sArray_(S, D, C)          ///< Macro version of kSerializer_Read32sArray.

#define kSerializer_Write64u_(S, D)                kxSerializer_Write64u_(S, D)                 ///< Macro version of kSerializer_Write64u.
#define kSerializer_Write64uArray_(S, D, C)        kxSerializer_Write64uArray_(S, D, C)         ///< Macro version of kSerializer_Write64uArray.
#define kSerializer_Read64u_(S, D)                 kxSerializer_Read64u_(S, D)                  ///< Macro version of kSerializer_Read64u.
#define kSerializer_Read64uArray_(S, D, C)         kxSerializer_Read64uArray_(S, D, C)          ///< Macro version of kSerializer_Read64uArray.

#define kSerializer_Write64s_(S, D)                kxSerializer_Write64s_(S, D)                 ///< Macro version of kSerializer_Write64s.
#define kSerializer_Write64sArray_(S, D, C)        kxSerializer_Write64sArray_(S, D, C)         ///< Macro version of kSerializer_Write64sArray.
#define kSerializer_Read64s_(S, D)                 kxSerializer_Read64s_(S, D)                  ///< Macro version of kSerializer_Read64s.    
#define kSerializer_Read64sArray_(S, D, C)         kxSerializer_Read64sArray_(S, D, C)          ///< Macro version of kSerializer_Read64sArray.

#define kSerializer_Write32f_(S, D)                kxSerializer_Write32f_(S, D)                 ///< Macro version of kSerializer_Write32f.
#define kSerializer_Write32fArray_(S, D, C)        kxSerializer_Write32fArray_(S, D, C)         ///< Macro version of kSerializer_Write32fArray.
#define kSerializer_Read32f_(S, D)                 kxSerializer_Read32f_(S, D)                  ///< Macro version of kSerializer_Read32f.
#define kSerializer_Read32fArray_(S, D, C)         kxSerializer_Read32fArray_(S, D, C)          ///< Macro version of kSerializer_Read32fArray.

#define kSerializer_Write64f_(S, D)                kxSerializer_Write64f_(S, D)                 ///< Macro version of kSerializer_Write64f.
#define kSerializer_Write64fArray_(S, D, C)        kxSerializer_Write64fArray_(S, D, C)         ///< Macro version of kSerializer_Write64fArray.
#define kSerializer_Read64f_(S, D)                 kxSerializer_Read64f_(S, D)                  ///< Macro version of kSerializer_Read64f.
#define kSerializer_Read64fArray_(S, D, C)         kxSerializer_Read64fArray_(S, D, C)          ///< Macro version of kSerializer_Read64fArray.

#define kSerializer_WriteSize_(S, D)               kxSerializer_WriteSize_(S, D)                ///< Macro version of kSerializer_WriteSize.
#define kSerializer_WriteSizeArray_(S, D, C)       kxSerializer_WriteSizeArray_(S, D, C)        ///< Macro version of kSerializer_WriteSizeArray.
#define kSerializer_ReadSize_(S, D)                kxSerializer_ReadSize_(S, D)                 ///< Macro version of kSerializer_ReadSize.
#define kSerializer_ReadSizeArray_(S, D, C)        kxSerializer_ReadSizeArray_(S, D, C)         ///< Macro version of kSerializer_ReadSizeArray.

#define kSerializer_WriteSSize_(S, D)              kxSerializer_WriteSSize_(S, D)               ///< Macro version of kSerializer_WriteSSize.
#define kSerializer_WriteSSizeArray_(S, D, C)      kxSerializer_WriteSSizeArray_(S, D, C)       ///< Macro version of kSerializer_WriteSSizeArray.
#define kSerializer_ReadSSize_(S, D)               kxSerializer_ReadSSize_(S, D)                ///< Macro version of kSerializer_ReadSSize.
#define kSerializer_ReadSSizeArray_(S, D, C)       kxSerializer_ReadSSizeArray_(S, D, C)        ///< Macro version of kSerializer_ReadSSizeArray.

#define kSerializer_WriteType_(S, T, V)            kxSerializer_WriteType_(S, T, V)             ///< Macro version of kSerializer_WriteType.
#define kSerializer_ReadType_(S, T, V)             kxSerializer_ReadType_(S, T, V)              ///< Macro version of kSerializer_ReadType.                                                           

#define kSerializer_WriteItems_(S, T, V, I, C)     kxSerializer_WriteItems_(S, T, V, I, C)      ///< Macro version of kSerializer_WriteItems.
#define kSerializer_ReadItems_(S, T, V, I, C)      kxSerializer_ReadItems_(S, T, V, I, C)       ///< Macro version of kSerializer_ReadItems.                                                         

#define kSerializer_BeginWrite_(S, T, I)           kxSerializer_BeginWrite_(S, T, I)            ///< Macro version of kSerializer_BeginWrite.
#define kSerializer_EndWrite_(S)                   kxSerializer_EndWrite_(S)                    ///< Macro version of kSerializer_EndWrite.                                                       
#define kSerializer_BeginRead_(S, T, I)            kxSerializer_BeginRead_(S, T, I)             ///< Macro version of kSerializer_BeginRead.
#define kSerializer_EndRead_(S)                    kxSerializer_EndRead_(S)                     ///< Macro version of kSerializer_EndRead.    
#define kSerializer_ReadCompleted_(S)              kxSerializer_ReadCompleted_(S)               ///< Macro version of kSerializer_ReadCompleted.    

#define kSerializer_AdvanceRead_(S, O)             kxSerializer_AdvanceRead_(S, O)              ///< Macro version of kSerializer_AdvanceRead.
#define kSerializer_Flush_(S)                      kxSerializer_Flush_(S)                       ///< Macro version of kSerializer_Flush.       
#define kSerializer_Reset_(S)                      kxSerializer_Reset_(S)                       ///< Macro version of kSerializer_Reset.
#define kSerializer_Stream_(S)                     kxSerializer_Stream_(S)                      ///< Macro version of kSerializer_Stream.

/** @} */

kEndHeader()

#include <kApi/Io/kSerializer.x.h>

#endif
