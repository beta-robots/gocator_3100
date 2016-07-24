/** 
 * @file    kDat5Serializer.h
 * @brief   Declares the kDat5Serializer class. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT5_SERIALIZER_H
#define K_API_DAT5_SERIALIZER_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kDat5Serializer
 * @extends kSerializer
 * @ingroup kApi-Io
 * @brief   Serializes/deserializes objects using kDat5 format. 
 */
//typedef kSerializer kDat5Serializer;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kDat5Serializer object.
 *
 * @public              @memberof kDat5Serializer
 * @param   serializer  Receives the constructed object. 
 * @param   stream      Stream for reading or writing.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kDat5Serializer_Construct(kDat5Serializer* serializer, kStream stream, kAlloc allocator); 

kEndHeader()

#include <kApi/Io/kDat5Serializer.x.h>

#endif
