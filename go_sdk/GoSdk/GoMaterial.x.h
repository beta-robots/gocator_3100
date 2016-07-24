/** 
 * @file    GoMaterial.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_MATERIAL_X_H
#define GO_MATERIAL_X_H

#include <kApi/Data/kXml.h>

kBeginHeader()

typedef struct GoMaterialClass
{
    kObjectClass base;
    
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    GoMaterialType type;
    GoMaterialType typeSystemValue;
    kBool typeUsed;
    
    GoElement32u spotThreshold;
    GoElement32u spotWidthMax;

    GoSpotSelectionType spotSelectionType;
    GoSpotSelectionType systemSpotSelectionType;
    kBool spotSelectionTypeUsed;

    GoElement64f cameraGainAnalog;
    GoElement64f cameraGainDigital;
    GoElement64f dynamicSensitivity;
    GoElement32u dynamicThreshold;
    GoElement32u gammaType;
} GoMaterialClass;

kDeclareClass(Go, GoMaterial, kObject)

#define GoMaterial_Cast_(CONTEXT)    kCastClass_(GoMaterial, CONTEXT)

GoFx(kStatus) GoMaterial_Construct(GoMaterial* layout, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoMaterial_Init(GoMaterial layout, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoMaterial_VRelease(GoMaterial layout);

GoFx(kStatus) GoMaterial_Read(GoMaterial layout, kXml xml, kXmlItem item);
GoFx(kStatus) GoMaterial_Write(GoMaterial layout, kXml xml, kXmlItem item); 

kEndHeader()

#endif
