/** 
 * @file    GoTools.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOLS_X_H
#define GO_TOOLS_X_H

#include <kApi/Data/kXml.h>
kBeginHeader()

typedef struct GoToolOptionClass
{
    kObjectClass base;

    kText64 name;
    kBool isExtensibleTool;
    kArrayList measurementOptions;  //of type GoMeasurementOption
} GoToolOptionClass;

kDeclareClass(Go, GoToolOption, kObject)

#define GoToolOption_Cast_(CONTEXT)    kCastClass_(GoToolOption, CONTEXT)

GoFx(kStatus) GoToolOption_Construct(GoToolOption* option, kAlloc allocator);
GoFx(kStatus) GoToolOption_Init(GoToolOption option, kType type, kAlloc alloc);
GoFx(kStatus) GoToolOption_VRelease(GoToolOption option);

GoFx(kBool) GoToolOption_IsCustom(GoToolOption option);


typedef struct GoToolsClass
{
    kObjectClass base;

    kObject sensor;

    kXml xml;
    //kXmlItem xmlItem; //there will be no forwards compatibility for collections

    kArrayList tools; //of type GoTool
    kArrayList toolOptions; //of type GoExtToolOption

    kArrayList nodesToMerge;    //of type kXmlItem
} GoToolsClass; 

kDeclareClass(Go, GoTools, kObject)

#define GoTools_Cast_(CONTEXT)    kCastClass_(GoTools, CONTEXT)

GoFx(kStatus) GoTools_Construct(GoTools* tools, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoTools_Init(GoTools tools, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTools_VRelease(GoTools tools);
GoFx(kStatus) GoTools_Read(GoTools tools, kXml xml, kXmlItem item, kXmlItem toolOptionsItem);
GoFx(kStatus) GoTools_Write(GoTools tools, kXml xml, kXmlItem item);
GoFx(kStatus) GoUtils_ParseToolType(const kChar* toolName, kType* type);
GoFx(kStatus) GoUtils_FormatToolType(GoTool tool, kChar* toolName, kSize capacity);


typedef struct GoScriptClass
{
    GoToolClass base; 
    kString code;
} GoScriptClass; 

kDeclareClass(Go, GoScript, GoTool)

#define GoScript_Cast_(CONTEXT)    kCastClass_(GoScript, CONTEXT)

GoFx(kStatus) GoScript_Construct(GoScript* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoScript_VInit(GoScript tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoScript_VRelease(GoScript tool);
GoFx(kStatus) GoScript_VRead(GoScript tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoScript_VWrite(GoScript tool, kXml xml, kXmlItem item); 

GoFx(const kChar*) GoTools_BuiltInToolDefineToString(GoToolType type);
GoFx(GoToolType) GoTools_StringToBuiltInDefine(const kChar* name);

GoFx(kStatus) GoTools_ReadOptions(GoTools tools, kXml xml, kXmlItem optionsItem);
GoFx(kSize) GoTools_ToolOptionCount(GoScript tool);
GoFx(GoToolOption) GoTools_ToolOptionAt(GoScript tool, kSize index);
GoFx(kStatus) GoTools_AddToolByName(GoTools tools, const kChar* optionName, GoTool* tool);
GoFx(kStatus) GoTools_AddMeasurementByName(GoTools tools, GoTool tool, const kChar* type, GoMeasurement* measurement);

GoFx(const kChar*) GoToolOption_Name(GoToolOption option);
GoFx(kSize) GoToolOption_MeasurementOptionCount(GoToolOption option);
GoFx(GoMeasurementOption) GoToolOption_MeasurementOptionAt(GoToolOption option, kSize index);


kEndHeader()

#endif
