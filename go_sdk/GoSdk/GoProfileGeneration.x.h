/** 
 * @file    GoProfileGeneration.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILEGENERATION_X_H
#define GO_PROFILEGENERATION_X_H

#include <kApi/Data/kXml.h>

kBeginHeader()

typedef struct GoProfileGenerationFixedLength
{
    GoProfileGenerationStartTrigger startTrigger;
    GoElement64f length;
} GoProfileGenerationFixedLength;

typedef struct GoProfileGenerationVariableLength
{
    GoElement64f maxLength;
} GoProfileGenerationVariableLength;

typedef struct GoProfileGenerationRotational
{
    GoElement64f circumference;
} GoProfileGenerationRotational;


typedef struct GoProfileGenerationClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoProfileGenerationType type;
    GoProfileGenerationFixedLength fixedLength;
    GoProfileGenerationVariableLength variableLength;
    GoProfileGenerationRotational rotational;
} GoProfileGenerationClass; 

kDeclareClass(Go, GoProfileGeneration, kObject)

#define GoProfileGeneration_Cast_(CONTEXT)    kCastClass_(GoProfileGeneration, CONTEXT)

GoFx(kStatus) GoProfileGeneration_Construct(GoProfileGeneration* surface, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoProfileGeneration_Init(GoProfileGeneration surface, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileGeneration_VRelease(GoProfileGeneration surface);

GoFx(kStatus) GoProfileGeneration_Read(GoProfileGeneration surface, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGeneration_Write(GoProfileGeneration surface, kXml xml, kXmlItem item); 

kEndHeader()

#endif
