/** 
 * @file    kDat6Serializer.h
 * @brief   Declares the kDat6Serializer class. 
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT6_SERIALIZER_H
#define K_API_DAT6_SERIALIZER_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kDat6Serializer
 * @extends kSerializer
 * @ingroup kApi-Io
 * @brief   Serializes/deserializes objects using kDat6 format. 
 */
//typedef kSerializer kDat6Serializer;        --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kDat6Serializer object.
 *
 * @public              @memberof kDat6Serializer
 * @param   serializer  Receives the constructed object. 
 * @param   stream      Stream for reading or writing.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kDat6Serializer_Construct(kDat6Serializer* serializer, kStream stream, kAlloc allocator); 

kEndHeader()

#include <kApi/Io/kDat6Serializer.x.h>

#endif
