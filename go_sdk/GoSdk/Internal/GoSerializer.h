/** 
 * @file    GoSerializer.h
 * @brief   Declares the GoSerializer class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SERIALIZER_H
#define GO_SDK_SERIALIZER_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Io/kSerializer.h>
kBeginHeader()

/**
 * @class   GoSerializer
 * @extends kSerializer
 * @ingroup GoSdk-Internal
 * @brief   Serializes/deserializes objects to/from Gocator Data Protocol. 
 */
typedef kSerializer GoSerializer; 

/** 
 * Constructs a GoSerializer object.
 *
 * @public              @memberof GoSerializer
 * @version             Introduced in firmware 4.0.10.27
 * @param   serializer  Receives the constructed object. 
 * @param   stream      Stream for reading or writing.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
GoFx(kStatus) GoSerializer_Construct(GoSerializer* serializer, kStream stream, kAlloc allocator); 

kEndHeader()
#include <GoSdk/Internal/GoSerializer.x.h>

#endif