/** 
 * @file    GoUtils.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_UTILS_X_H
#define GO_SDK_UTILS_X_H

#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>
kBeginHeader()

typedef kStatus (kCall* GoReadFunction)(kObject object, kXml xml, kXmlItem item);

GoFx(kStatus) GoOptionList_ParseList32u(const kChar* text, kArrayList list);  
GoFx(kStatus) GoOptionList_Parse32u(const kChar* text, k32u* optionList, kSize capacity, kSize* count);  
GoFx(kStatus) GoOptionList_ParseHelper32u(const kChar* text, kSize length, k32u* value); 
GoFx(kStatus) GoOptionList_Format32u(const k32u* optionList, kSize count, kChar* text, kSize capacity);  
GoFx(kBool) GoOptionList_Check32u(const k32u* optionList, kSize count, k32u value);  

GoFx(kStatus) GoOptionList_ParseList64f(const kChar* text, kArrayList list);  
GoFx(kStatus) GoOptionList_Parse64f(const kChar* text, k64f* optionList, kSize capacity, kSize* count);  
GoFx(kStatus) GoOptionList_ParseHelper64f(const kChar* text, kSize length, k64f* value); 
GoFx(kStatus) GoOptionList_Format64f(const k64f* optionList, kSize count, kChar* text, kSize capacity);  
GoFx(kBool) GoOptionList_Check64f(const k64f* optionList, kSize count, k64f value);  

GoFx(kBool) GoOptionList_CheckSize(const k32u* optionList, kSize count, kSize value);

GoFx(kStatus) GoConfig_WriteRangeElement64f(kXml xml, kXmlItem item, const kChar* elementName, GoElement64f element);
GoFx(kStatus) GoConfig_WriteRangeElement32u(kXml xml, kXmlItem item, const kChar* elementName, GoElement32u element);
GoFx(kStatus) GoConfig_ReadRangeElement64f(kXml xml, kXmlItem item, const kChar* elementName, GoElement64f* element);
GoFx(kStatus) GoConfig_ReadRangeElement32u(kXml xml, kXmlItem item, const kChar* elementName, GoElement32u* element);

GoFx(kStatus) GoConfig_ReadChildOptional(kXml xml, kXmlItem item, const kChar* name, kObject object, GoReadFunction func);
GoFx(kStatus) GoConfig_Read16sOptional(kXml xml, kXmlItem item, const kChar* name, k16s defaultVal, k16s* value);
GoFx(kStatus) GoConfig_Read16uOptional(kXml xml, kXmlItem item, const kChar* name, k16u defaultVal, k16u* value);

GoFx(kStatus) GoConfig_Read32sOptional(kXml xml, kXmlItem item, const kChar* name, k32s defaultVal, k32s* value);
GoFx(kStatus) GoConfig_Read32uOptional(kXml xml, kXmlItem item, const kChar* name, k32u defaultVal, k32u* value);
GoFx(kStatus) GoConfig_Read32fOptional(kXml xml, kXmlItem item, const kChar* name, k32f defaultVal, k32f* value);
GoFx(kStatus) GoConfig_ReadRangeElement32uOptional(kXml xml, kXmlItem item, const kChar* elementName, GoElement32u* element);

GoFx(kStatus) GoConfig_Read64sOptional(kXml xml, kXmlItem item, const kChar* name, k64s defaultVal, k64s* value);
GoFx(kStatus) GoConfig_Read64uOptional(kXml xml, kXmlItem item, const kChar* name, k64u defaultVal, k64u* value);
GoFx(kStatus) GoConfig_Read64fOptional(kXml xml, kXmlItem item, const kChar* name, k64f defaultVal, k64f* value);
GoFx(kStatus) GoConfig_ReadRangeElement64fOptional(kXml xml, kXmlItem item, const kChar* elementName, GoElement64f* element);

GoFx(kStatus) GoConfig_ReadSizeOptional(kXml xml, kXmlItem item, const kChar* name, kSize defaultVal, kSize* value);

GoFx(kStatus) GoConfig_ReadAttrBoolOptional(kXml xml, kXmlItem item, const kChar* name, kBool defaultVal, kBool* value);
GoFx(kStatus) GoConfig_ReadBoolOptional(kXml xml, kXmlItem item, const kChar* name, kBool defaultVal, kBool* value);

GoFx(kStatus) GoConfig_ReadStringOptional(kXml xml, kXmlItem item, const kChar* name, const kChar* defaultVal, kString value);



/** 
 * Merges srcItem onto dstItem. Shallow copies child nodes from srcItem to dstItem and 
 * copies attributes. Only copies from srcItem if dstItem doesn't have a matching child or 
 * attribute name.
 * If srcXml or srcItem equal kNULL, the function returns successfully with no effect.
 *
 * @public                  @memberof GoUtils
 * @param    srcXml         Source XML.
 * @param    srcItem        Source XML node.
 * @param    dstXml         Destination XML.
 * @param    dstItem        Destination XML node.
 * @return   Operation status.            
 */
GoFx(kStatus) GoUtils_XmlMerge(kXml srcXml, kXmlItem srcItem, kXml dstXml, kXmlItem dstItem);  

kEndHeader()

#endif