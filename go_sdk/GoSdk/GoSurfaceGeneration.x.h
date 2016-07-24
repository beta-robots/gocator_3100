/** 
 * @file    GoSurfaceGeneration.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACEGENERATION_X_H
#define GO_SURFACEGENERATION_X_H

#include <kApi/Data/kXml.h>

kBeginHeader()

typedef struct GoSurfaceGenerationFixedLength
{
    GoSurfaceGenerationStartTrigger startTrigger;
    GoElement64f length;
} GoSurfaceGenerationFixedLength;

typedef struct GoSurfaceGenerationVariableLength
{
    GoElement64f maxLength;
} GoSurfaceGenerationVariableLength;

typedef struct GoSurfaceGenerationRotational
{
    GoElement64f circumference;
} GoSurfaceGenerationRotational;


typedef struct GoSurfaceGenerationClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoSurfaceGenerationType type;
    GoSurfaceGenerationFixedLength fixedLength;
    GoSurfaceGenerationVariableLength variableLength;
    GoSurfaceGenerationRotational rotational;
} GoSurfaceGenerationClass; 

kDeclareClass(Go, GoSurfaceGeneration, kObject)

#define GoSurfaceGeneration_Cast_(CONTEXT)    kCastClass_(GoSurfaceGeneration, CONTEXT)

GoFx(kStatus) GoSurfaceGeneration_Construct(GoSurfaceGeneration* surface, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoSurfaceGeneration_Init(GoSurfaceGeneration surface, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceGeneration_VRelease(GoSurfaceGeneration surface);

GoFx(kStatus) GoSurfaceGeneration_Read(GoSurfaceGeneration surface, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceGeneration_Write(GoSurfaceGeneration surface, kXml xml, kXmlItem item); 

kEndHeader()

#endif
