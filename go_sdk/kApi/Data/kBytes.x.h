/** 
 * @file    kBytes.x.h
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BYTES_X_H
#define K_API_BYTES_X_H

kBeginHeader()

#define kBYTES_CAPACITY          (1024)

kFx(kStatus) kBytes_AddTypes(kAssembly assembly); 
kFx(kStatus) kBytes_Register(kAssembly assembly, const kChar* name); 

kFx(kType) kBytes_GetType(kSize size); 
kFx(kStatus) kBytes_Write(kType type, void* values, kSize count, kSerializer serializer);      
kFx(kStatus) kBytes_Read(kType type, void* values, kSize count, kSerializer serializer);     
kFx(kBool) kBytes_VEquals(kType type, const void* value, const void* other);
kFx(kSize) kBytes_VHashCode(kType type, const void* value);

#define kAddBytes()     kBytes_AddTypes(output); 

#if defined(K_COMPAT_5)

#   define kTYPE_BYTES(N)           kBytes_GetType(N)
#   define kType_Bytes(N)           kBytes_GetType(N)

#endif

kEndHeader()

#endif
