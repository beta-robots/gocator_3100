/** 
 * @file    kXml.x.h
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_XML_X_H
#define K_API_XML_X_H

kBeginHeader()

#define kXML_FILE_READ_BUFFER_SIZE      (16384)
#define kXML_FILE_WRITE_BUFFER_SIZE     (16384)
#define kXML_FORMAT_BUFFER_SIZE         (256)
#define kXML_STRING_INIT_SIZE           (100*1024)
#define kXML_DEFAULT_TEXT_SIZE          (64)
#define kXML_DEFAULT_ITEM_BLOCK_COUNT   (128)
#define kXML_DEFAULT_ATTR_BLOCK_COUNT   (128)

#define kXML_PATH_DELIMITERS            "\\/"
#define kXML_DEFAULT_HEADER             "?xml version=\"1.0\" encoding=\"UTF-8\"?"
#define kXML_NEW_LINE                   "\r\n"
#define kXML_INDENTATION                "  "

typedef struct
{
    kChar text[kXML_DEFAULT_TEXT_SIZE];
    kChar* buffer;
    kSize capacity;
} kXmlTextField;

typedef struct kXmlAttrClassTag kXmlAttrClass;
typedef struct kXmlItemClassTag kXmlItemClass;

struct kXmlAttrClassTag
{
    kXmlTextField name;
    kXmlTextField value;
    kXmlAttrClass* next;
};

struct kXmlItemClassTag
{
    kXmlTextField name;
    kXmlTextField value;

    kXmlAttrClass* firstAttr;
    kXmlAttrClass* lastAttr;

    kXmlItemClass* firstChild;
    kXmlItemClass* lastChild;

    kXmlItemClass* parent;
    kXmlItemClass* prev;
    kXmlItemClass* next;
};

typedef struct
{
    kXmlItemClass* items;
} kXmlItemBlock;

kDeclareValue(k, kXmlItemBlock, kValue)

typedef struct
{
    kXmlAttrClass* attrs;
} kXmlAttrBlock;

kDeclareValue(k, kXmlAttrBlock, kValue)

typedef enum
{
    TAG_OPEN,
    TAG_CLOSE,
    TAG_EMPTY
} kXmlTagType;

typedef struct
{
    kXml xml;
    kXmlItemClass* currentItem;
    const kChar* position;
    kBool isStream;
    kChar* buffer;

    // stream info
    kStream stream;
    kSize capacity;
    kSize size;
    kBool eof;
} kXmlParseContext;

typedef struct
{
    kObjectClass base; 

    kString header;
    
    kXmlItemClass* root;

    kSize itemBlockCount;
    kArrayList itemBlocks;
    kXmlItemClass* freeItems;

    kSize attrBlockCount;
    kArrayList attrBlocks;
    kXmlAttrClass* freeAttrs;

    kChar formatBuffer[kXML_FORMAT_BUFFER_SIZE];
    kArrayList pathParseBuffer;
} kXmlClass; 

kDeclareClass(k, kXml, kObject)

kFx(kStatus) kXml_VRelease(kXml xml); 

kFx(kStatus) kXml_WriteDat6V0(kXml xml, kSerializer serializer); 
kFx(kStatus) kXml_ReadDat6V0(kXml xml, kSerializer serializer, kAlloc allocator); 

kStatus kXml_AllocItemBlock(kXml xml);
kStatus kXml_AllocAttrBlock(kXml xml);
kStatus kXml_FreeItemBlocks(kXml xml);
kStatus kXml_FreeAttrBlocks(kXml xml);

kStatus kXml_AllocItem(kXml xml, kXmlItemClass** item);
kStatus kXml_FreeItem(kXml xml, kXmlItemClass* item);
kStatus kXml_AllocAttr(kXml xml, kXmlAttrClass** attr);
kStatus kXml_FreeAttr(kXml xml, kXmlAttrClass* attr);

kStatus kXml_LinkItemUnder(kXml xml, kXmlItemClass* parent, kXmlItemClass* item);
kStatus kXml_LinkItemBefore(kXml xml, kXmlItemClass* next, kXmlItemClass* item);
kStatus kXml_LinkItemAfter(kXml xml, kXmlItemClass* prev, kXmlItemClass* item);
kStatus kXml_UnlinkItem(kXml xml, kXmlItemClass* item);
kStatus kXml_LinkAttr(kXml xml, kXmlItemClass* item, kXmlAttrClass* attr);

kXmlAttrClass* kXml_FindItemAttributeObject(kXml xml, kXmlItem item, const kChar *name);
kXmlItemClass* kXml_FindChildByName(kXml xml, kXmlItemClass* parent, const kChar* name, kSize index);
kXmlAttrClass* kXml_FindAttrByName(kXml xml, kXmlItemClass* item, const kChar* name);

kStatus kXml_ItemToString(kXml xml, kString str, kXmlItemClass* item, kSize level);
const kChar* kXml_ItemValue(kXml xml, kXmlItem item);
const kChar* kXml_AttrValue(kXml xml, kXmlItem item, const kChar* name);

kStatus kXml_TextFieldInit(kXml xml, kXmlTextField* field);
kStatus kXml_TextFieldRelease(kXml xml, kXmlTextField* field);
kStatus kXml_TextFieldReserve(kXml xml, kXmlTextField* field, kSize size);
kStatus kXml_TextFieldSetString(kXml xml, kXmlTextField* field, const kChar* str);
kChar* kXml_TextFieldString(kXml xml, kXmlTextField* field);

const kChar* kXml_Format16u(kXml xml, k16u val);
const kChar* kXml_Format16s(kXml xml, k16s val);
const kChar* kXml_Format32u(kXml xml, k32u val);
const kChar* kXml_Format32s(kXml xml, k32s val);
const kChar* kXml_FormatBool(kXml xml, kBool val);
const kChar* kXml_Format64u(kXml xml, k64u val);
const kChar* kXml_Format64s(kXml xml, k64s val);
const kChar* kXml_FormatSize(kXml xml, kSize val);
const kChar* kXml_Format32f(kXml xml, k32f val);
const kChar* kXml_Format64f(kXml xml, k64f val);
kStatus kXml_Parse16u(kXml xml, const kChar *string, k16u *val);
kStatus kXml_Parse16s(kXml xml, const kChar *string, k16s *val);
kStatus kXml_Parse32u(kXml xml, const kChar *string, k32u *val);
kStatus kXml_Parse32s(kXml xml, const kChar *string, k32s *val);
kStatus kXml_ParseBool(kXml xml, const kChar *string, kBool *val);
kStatus kXml_Parse64u(kXml xml, const kChar *string, k64u *val);
kStatus kXml_Parse64s(kXml xml, const kChar *string, k64s *val);
kStatus kXml_ParseSize(kXml xml, const kChar *string, kSize *val);
kStatus kXml_Parse32f(kXml xml, const kChar *string, k32f *val);
kStatus kXml_Parse64f(kXml xml, const kChar *string, k64f *val);

kStatus kXml_Substr(kXml xml, const kChar* input, kSize begin, kSize end, kXmlTextField* str);
kStatus kXml_UnescapeSubstr(kXml xml, const kChar* input, kSize begin, kSize end, kXmlTextField* str);
const kChar* kXml_FindDelimiter(const kChar* str, const kChar* delims);
kSize kXml_ParseNameSubstrIndex(const kChar* str, kSize size, kSize* adjustedSize);

kStatus kXml_SetPathParseBuffer(kXml xml, const kChar* text, kSize size);
const kChar* kXml_PathParseBuffer(kXml xml);

kBool kXml_ValidateName(const kChar *name);
kBool kXml_IsNameChar(kChar ch);
kStatus kXml_AddEscapedString(kXml xml, const kChar* in, kString out);

// Parser code

kStatus kXml_ParseAttributes(kXml xml, kXmlParseContext* context);
kStatus kXml_ParseTag(kXml xml, kXmlParseContext* context, kXmlTagType* tagType);
kStatus kXml_ParseComment(kXml xml, kXmlParseContext* context);
kStatus kXml_ParseDeclaration(kXml xml, kXmlParseContext* context);
kStatus kXml_Parse(kXml xml, kXmlParseContext* context);
kStatus kXml_AdvanceParser(kXmlParseContext* context);
kStatus kXml_SkipWhitespace(kXmlParseContext* context);
kStatus kXml_SkipToken(kXmlParseContext* context);

kFx(kStatus) kXml_Init(kXml xml, kAlloc allocator);
kFx(kStatus) kXml_VInitClone(kXml xml, kXml source, kAlloc allocator);
kFx(kStatus) kXml_Copy(kXml xml, kXml source);

kFx(kStatus) kXml_Read(kXml xml, kStream stream);
kFx(kStatus) kXml_Write(kXml xml, kStream stream);

#define kXml_(XML)                      kCast(kXmlClass*, (XML))
#define kXml_Cast_(XML)                kCastClass_(kXml, (XML))
#define kXmlItem_(ITEM)                 kCast(kXmlItemClass*, (ITEM))

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_XML                       kTypeOf(kXml)
#   define kXml_Destroy                    kObject_Destroy

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kXml_Construct5(X)              kXml_Construct(X, kNULL)
#   define kXml_Copy5(X, S)                kXml_Copy(X, S, kNULL)
#   define kXml_Load5(X, P)                kXml_Load(X, P, kNULL)
#   define kXml_FromText5(X, S)            kXml_FromText(X, S, kNULL)
#   define kXml_Assign5(X, S)              kXml_Assign(X, S, kNULL)

#endif

kEndHeader()

#endif
