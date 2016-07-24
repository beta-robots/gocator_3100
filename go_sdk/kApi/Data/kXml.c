/** 
 * @file    kXml.c
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMath.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>

struct EscSeq
{
    char character;
    char szSeq[25];
};

static struct EscSeq escSeqs[] = {
    { '&', "amp" },
    { '<', "lt" },
    { '>', "gt" },
    { '\"', "quot" },
    { '\'', "apos" },
    { 0xA, "#x0A" },
    { 0xD, "#x0D" }
};

kBeginClass(k, kXml, kObject)

    kAddVersion(kXml, "kdat6", "6.0.0.0", "kXml-0", WriteDat6V0, ReadDat6V0)
    
    kAddVMethod(kXml, kObject, VRelease)
    kAddVMethod(kXml, kObject, VInitClone)
kEndClass() 

kBeginValue(k, kXmlItemBlock, kValue)
    kAddField(kXmlItemBlock, kPointer, items)
kEndValue()

kBeginValue(k, kXmlAttrBlock, kValue)
    kAddField(kXmlAttrBlock, kPointer, attrs)
kEndValue()

kFx(kStatus) kXml_Construct(kXml* xml, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 
    
    kCheck(kAlloc_GetObject(alloc, kTypeOf(kXml), xml)); 

    if (!kSuccess(status = kXml_Init(*xml, alloc)))
    {
        kAlloc_FreeRef(alloc, xml); 
    }

    return status; 
} 

kFx(kStatus) kXml_Init(kXml xml, kAlloc allocator)
{
    kXmlClass* obj = kXml_(xml);

    kCheck(kObject_Init(xml, kTypeOf(kXml), allocator));

    kCheck(kString_Construct(&obj->header, kNULL, allocator));
    kCheck(kArrayList_Construct(&obj->pathParseBuffer, kTypeOf(kByte), 128, allocator));
    kCheck(kArrayList_Construct(&obj->itemBlocks, kTypeOf(kXmlItemBlock), 1, allocator));
    kCheck(kArrayList_Construct(&obj->attrBlocks, kTypeOf(kXmlAttrBlock), 1, allocator));

    obj->itemBlockCount = kXML_DEFAULT_ITEM_BLOCK_COUNT;
    obj->attrBlockCount = kXML_DEFAULT_ATTR_BLOCK_COUNT;
    obj->root = kNULL;
    obj->freeItems = kNULL;
    obj->freeAttrs = kNULL;

    return kOK;
}

kFx(kStatus) kXml_VInitClone(kXml xml, kXml source, kAlloc allocator)
{
    kStatus status; 

    kCheck(kXml_Init(xml, allocator));

    if (!kSuccess(status = kXml_Copy(xml, source)))
    {
        kXml_VRelease(xml); 
        return status;
    }

    return kOK; 
}

kFx(kStatus) kXml_VRelease(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);

    kXml_Clear(xml);       
    kXml_FreeItemBlocks(xml);
    kXml_FreeAttrBlocks(xml);

    kDestroyRef(&obj->itemBlocks);
    kDestroyRef(&obj->attrBlocks);
    kDestroyRef(&obj->header);
    kDestroyRef(&obj->pathParseBuffer);

    kCheck(kObject_VRelease_(xml));

    return kOK;
}

kFx(kStatus) kXml_WriteDat6V0(kXml xml, kSerializer serializer)
{
    kString string = kNULL; 

    kTry
    {
        kTest(kString_Construct(&string, kNULL, kObject_Alloc_(xml))); 

        kTest(kXml_ToString(xml, string)); 

        kTest(kSerializer_WriteObject(serializer, string)); 
    }
    kFinally
    {
        kObject_Destroy(string); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kXml_ReadDat6V0(kXml xml, kSerializer serializer, kAlloc allocator)
{
    kString string = kNULL; 
    kStatus status; 
    
    kCheck(kXml_Init(xml, allocator)); 

    kTry
    {
        kTest(kSerializer_ReadObject(serializer, &string, allocator)); 

        kTest(kXml_FromString(xml, string)); 
    }
    kCatchEx(&status)
    {
        kXml_VRelease(xml); 
        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        kCheck(kObject_Destroy(string)); 
        kEndFinallyEx(); 
    }

    return kOK; 
}

kFx(kStatus) kXml_Compact(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXml newXml;
    kXmlClass* newObj;

    // Make a clean copy, which is compact.
    kCheck(kXml_Construct(&newXml, obj->base.alloc));
    kCheck(kXml_Assign(newXml, xml));
    newObj = kXml_(newXml);

    // Copy over the guts
    kCheck(kXml_Clear(xml));
    kCheck(kArrayList_Assign(obj->attrBlocks, newObj->attrBlocks));
    kCheck(kArrayList_Assign(obj->itemBlocks, newObj->itemBlocks));
    obj->root = newObj->root;
    obj->freeAttrs = newObj->freeAttrs;
    obj->freeItems = newObj->freeItems;

    // Remove references to copied data
    kCheck(kArrayList_Clear(newObj->attrBlocks));
    kCheck(kArrayList_Clear(newObj->itemBlocks));
    newObj->root = kNULL;
    newObj->freeAttrs = kNULL;
    newObj->freeItems = kNULL;

    kCheck(kObject_Destroy(newXml));

    return kOK;
}

kFx(kStatus) kXml_Copy(kXml xml, kXml source)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlClass* srcObj = kXml_Cast_(source);
    kAlloc alloc = kObject_Alloc_(xml); 

    kCheck(kObject_Release(xml));
    kCheck(kXml_Init(xml, alloc));

    kCheck(kXml_CopyItem(xml, kNULL, kNULL, source, kXml_Root(source), kNULL));
    kCheck(kString_Assign(obj->header, srcObj->header));

    return kOK;
}

kFx(kStatus) kXml_Assign(kXml xml, kXml source)
{
    return kXml_Copy(xml, source);
}

kFx(kStatus) kXml_Read(kXml xml, kStream stream)
{
    kXmlClass* obj = xml;
    kXmlParseContext context;

    context.isStream = kTRUE;

    context.xml = xml;
    context.buffer = kNULL;
    context.currentItem = kNULL;
    context.position = kNULL;
    context.capacity = 0;
    context.size = 0;
    context.eof = kFALSE;
    context.stream = stream;

    kTry 
    {
        context.capacity = 1024*10;
        kTest(kAlloc_Get(obj->base.alloc, context.capacity, &context.buffer));
        context.position = context.buffer;

        kTest(kXml_AdvanceParser(&context));
        context.position = context.buffer;
        kTest(kXml_Parse(xml, &context));
    }
    kFinally
    {
        kAlloc_Free(obj->base.alloc, context.buffer);
        context.buffer = kNULL;
        kEndFinally();
    }
    return kOK;
}

kFx(kStatus) kXml_Write(kXml xml, kStream stream)
{
    kString str = kNULL;
    
    kTry
    {
        kTest(kString_Construct(&str, kNULL, kNULL));
        kTest(kXml_ToString(xml, str));
        kTest(kStream_Write(stream, kString_Chars(str), kString_Length(str)));
    }
    kFinally
    {
        kObject_Destroy(str);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kXml_Load(kXml* xml, const kChar* fileName, kAlloc allocator)
{
    kFile file = kNULL;
    kXml result = kNULL; 

    kTry 
    {
        kTest(kXml_Construct(&result, allocator));
        
        kTest(kFile_Construct(&file, fileName, kFILE_MODE_READ, allocator));
        kTest(kFile_SetReadBuffer(file, kXML_FILE_READ_BUFFER_SIZE)); 

        kTest(kXml_Read(result, file));

        *xml = result; 
        result = kNULL; 
    } 
    kFinally
    {
        kObject_Destroy(result);
        kObject_Destroy(file);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kXml_Save(kXml xml, const kChar* fileName)
{
    kFile file = kNULL;

    kTry
    {
        kTest(kFile_Construct(&file, fileName, kFILE_MODE_WRITE, kNULL));
        kTest(kFile_SetWriteBuffer(file, kXML_FILE_WRITE_BUFFER_SIZE)); 

        kTest(kXml_Write(xml, file));
    }
    kFinally
    {
        kObject_Destroy(file);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kXml_FromString(kXml xml, kString str)
{
    kXmlParseContext context;

    context.isStream = kFALSE;
    context.buffer = (kChar*) kString_Chars(str); // isn't modified for non-stream
    context.currentItem = kNULL;
    context.position = kString_Chars(str);
    context.xml = xml;

    // unused
    context.capacity = 0;
    context.size = 0;
    context.eof = kFALSE;
    context.stream = kNULL;

    kCheck(kXml_Clear(xml));  
    kCheck(kXml_Parse(xml, &context));

    return kOK;
}

kFx(kStatus) kXml_ToString(kXml xml, kString str)
{
    kXmlClass* obj = kXml_Cast_(xml);

    kCheck(kString_Import(str, "<", kFALSE));

    if (kString_Length(obj->header) != 0)
    {
        kCheck(kString_Import(str, kString_Chars(obj->header), kTRUE));
    }
    else
    {
        kCheck(kString_Import(str, kXML_DEFAULT_HEADER, kTRUE));
    }

    kCheck(kString_Import(str, ">", kTRUE));
    kCheck(kString_Import(str, kXML_NEW_LINE, kTRUE));

    if (!kIsNull(obj->root))
    {
        kCheck(kXml_ItemToString(xml, str, obj->root, 0));
    }

    return kOK;
}

kFx(kStatus) kXml_FromText(kXml* xml, const kChar* str, kAlloc allocator)
{
    kXml result = kNULL; 
    kStatus status;
    kXmlParseContext context;

    context.isStream = kFALSE;
    context.buffer = (kChar*)str; // isn't modified for non-stream
    context.currentItem = kNULL;
    context.position = str;

    // unused
    context.capacity = 0;
    context.size = 0;
    context.eof = kFALSE;
    context.stream = kNULL;

    kTry 
    {
        kTest(kXml_Construct(&result, allocator));
        context.xml = result;
        kTest(kXml_Parse(result, &context));

        *xml = result; 
    } 
    kCatch(&status) 
    {
        kObject_Destroy(result);
        kEndCatch(status); 
    }

    return kOK;
}

kFx(kStatus) kXml_SaveBytes(kXml xml, kByte** data, kSize* size, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kMemory stream = kNULL; 
    kByte* output = kNULL; 

    kTry
    {
        kTest(kMemory_Construct(&stream, kNULL)); 
        kTest(kXml_Write(xml, stream)); 

        kTest(kAlloc_Get(alloc, (kSize)kMemory_Length(stream), &output)); 
        kTest(kMemCopy(output, kMemory_At(stream, 0), (kSize) kMemory_Length(stream)));  

        *size = (kSize) kMemory_Length(stream); 
        *data = output; 
        output = kNULL; 
    }
    kFinally
    {
        kCheck(kObject_Destroy(stream)); 
        kCheck(kAlloc_Free(alloc, output)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kXml_LoadBytes(kXml* xml, const kByte* data, kSize size, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator); 
    kMemory stream = kNULL; 
    kXml output = kNULL; 

    kTry
    {
        kTest(kMemory_Construct(&stream, kNULL)); 
        kTest(kMemory_Attach(stream, (void*)data, 0, size, size)); 

        kTest(kXml_Construct(&output, alloc)); 
        kTest(kXml_Read(output, stream)); 

        *xml = output; 
        output = kNULL; 
    }
    kFinally
    {
        kCheck(kObject_Destroy(stream)); 
        kCheck(kObject_Destroy(output)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kXml_Clear(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);

    if (!kIsNull(obj->root))
    {
        kCheck(kXml_DeleteItem(xml, kXml_Root(xml)));
    }

    kCheck(kXml_FreeItemBlocks(xml));
    kCheck(kXml_FreeAttrBlocks(xml));

    return kOK;
}

kFx(kXmlItem) kXml_Root(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);

    return obj->root;
}

kFx(kStatus) kXml_AddItem(kXml xml, kXmlItem parent, const kChar* name, kXmlItem* item)
{
    kXmlItemClass* itemObj = kNULL; 

    kCheckArgs(kXml_ValidateName(name)); 

    kCheck(kXml_AllocItem(xml, &itemObj));
    kCheck(kXml_LinkItemUnder(xml, kIsNull(parent) ? kNULL : kXmlItem_(parent), itemObj));

    kCheck(kXml_TextFieldSetString(xml, &itemObj->name, name));
    kCheck(kXml_TextFieldSetString(xml, &itemObj->value, ""));

    if (!kIsNull(item))
    {
        *item = itemObj;
    }

    return kOK;
}

kFx(kStatus) kXml_InsertItem(kXml xml, kXmlItem next, const kChar* name, kXmlItem* item)
{
    kXmlItemClass* itemObj = kNULL;

    kCheckArgs(!kIsNull(next) && kXml_ValidateName(name)); 

    kCheck(kXml_AllocItem(xml, &itemObj));
    kCheck(kXml_LinkItemBefore(xml, kXmlItem_(next), itemObj));

    kCheck(kXml_TextFieldSetString(xml, &itemObj->name, name));
    kCheck(kXml_TextFieldSetString(xml, &itemObj->value, ""));

    if (!kIsNull(item))
    {
        *item = itemObj;
    }

    return kOK;
}

kFx(kStatus) kXml_DeleteItem(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);

    kCheckArgs(!kIsNull(item));  
    
    kCheck(kXml_UnlinkItem(xml, itemObj));
    kCheck(kXml_FreeItem(xml, itemObj));

    return kOK;
}

kFx(kStatus) kXml_EnsureChildExists(kXml xml, kXmlItem parent, const kChar* path, kXmlItem* item)
{
    const kChar* delim = kNULL;
    kXmlItemClass* foundItem = kXmlItem_(parent);

    // for non-existent path items: if index is unspecified or 0, then the
    // item is created. if index is greater than 0 then it is not created,
    // an error is returned instead

    do
    {
        kSize index;
        kSize tokenSize;
        kXmlItemClass* tempItem;

        delim = kXml_FindDelimiter(path, kXML_PATH_DELIMITERS);
        tokenSize = delim - path;
        index = kXml_ParseNameSubstrIndex(path, tokenSize, &tokenSize);

        if (!kSuccess(kXml_SetPathParseBuffer(xml, path, tokenSize)))
        {
            return kNULL;
        }

        tempItem = kXml_FindChildByName(xml, foundItem, kXml_PathParseBuffer(xml), index);
        if (kIsNull(tempItem) && index == 0)
        {
            kXmlItem newItem;

            kCheck(kXml_AddItem(xml, foundItem, kXml_PathParseBuffer(xml), &newItem));
            tempItem = kXmlItem_(newItem);
        }
        foundItem = tempItem;

        path = delim + 1;
    } 
    while (*delim != '\0' && !kIsNull(foundItem));

    if (kIsNull(foundItem))
    {
        return kERROR;
    }

    if (!kIsNull(item))
    {
        *item = foundItem; 
    }

    return kOK; 
}

kFx(kStatus) kXml_DeleteChildren(kXml xml, kXmlItem item)
{
    kXmlItem child = kNULL; 

    kCheckArgs(!kIsNull(item)); 

    child = kXml_FirstChild(xml, item); 

    while (!kIsNull(child))
    {
        kCheck(kXml_DeleteItem(xml, child)); 

        child = kXml_FirstChild(xml, item); 
    }

    return kOK; 
}

kFx(kStatus) kXml_SetItemName(kXml xml, kXmlItem item, const kChar* name)
{
    kXmlItemClass* itemObj = kXmlItem_(item);

    kCheckArgs(!kIsNull(item));  
 
    return kXml_TextFieldSetString(xml, &itemObj->name, name);
}

kFx(const kChar*) kXml_ItemName(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);

    kAssert(!kIsNull(item));  

    return kXml_TextFieldString(xml, &itemObj->name);
}

kFx(kStatus) kXml_SetItemText(kXml xml, kXmlItem item, const kChar* str)
{
    kXmlItemClass* itemObj = kXmlItem_(item);

    kCheckArgs(!kIsNull(item));  

    return kXml_TextFieldSetString(xml, &itemObj->value, str);
}

kFx(kStatus) kXml_ItemText(kXml xml, kXmlItem item, kChar* str, kSize capacity)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    const kChar* value;

    kCheckArgs(!kIsNull(item));  

    value = kXml_TextFieldString(xml, &itemObj->value);

    if (capacity <= kStrLength(value))
    {
        return kERROR_INCOMPLETE;
    }

    kCheck(kStrCopy(str, capacity, value)); 
    
    return kOK;
}

kFx(kStatus) kXml_ItemString(kXml xml, kXmlItem item, kString str)
{
    kXmlItemClass* itemObj = kXmlItem_(item);

    kCheckArgs(!kIsNull(item) && !kIsNull(str)); 

    kCheck(kString_Import(str, kXml_TextFieldString(xml, &itemObj->value), kFALSE));

    return kOK;
}

kFx(kStatus) kXml_SetAttrText(kXml xml, kXmlItem item, const kChar* name, const kChar* str)
{
    kXmlAttrClass* attrObj;

    kCheckArgs(!kIsNull(item));  

    attrObj = kXml_FindAttrByName(xml, kXmlItem_(item), name);

    if (kIsNull(attrObj))
    {
        kCheck(kXml_AllocAttr(xml, &attrObj));
        kCheck(kXml_LinkAttr(xml, kXmlItem_(item), attrObj));
    }

    kCheck(kXml_TextFieldSetString(xml, &attrObj->name, name));
    kCheck(kXml_TextFieldSetString(xml, &attrObj->value, str));

    return kOK;
}

kFx(kBool) kXml_AttrExists(kXml xml, kXmlItem item, const kChar* name)
{
    kXmlAttrClass* attrObj;

    kAssert(!kIsNull(item));

    attrObj = kXml_FindAttrByName(xml, kXmlItem_(item), name);
    return (!kIsNull(attrObj)) ? kTRUE : kFALSE;
}

kFx(kSize) kXml_AttrCount(kXml xml, kXmlItem item)
{
    kXmlAttrClass* attrObj;
    kSize count = 0;

    kAssert(!kIsNull(item));

    attrObj = kXmlItem_(item)->firstAttr;
    while (attrObj != NULL)
    {
        ++count;
        attrObj = attrObj->next;
    }

    return count;
}

kFx(const kChar*) kXml_AttrNameAt(kXml xml, kXmlItem item, kSize index)
{
    kXmlAttrClass* attrObj;
    kSize i;

    kAssert(!kIsNull(item));

    attrObj = kXmlItem_(item)->firstAttr;
    for (i = 0; (i < index) && (!kIsNull(attrObj)); ++i)
    {
        attrObj = attrObj->next;
    }

    return kIsNull(attrObj) ? kNULL : kXml_TextFieldString(xml, &attrObj->name);
}

kFx(kStatus) kXml_DeleteAttr(kXml xml, kXmlItem item, const kChar* name)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    kXmlAttrClass* attrObj;
    kXmlAttrClass* prevAttrObj;

    kCheckArgs(!kIsNull(item));

    attrObj = itemObj->firstAttr;
    prevAttrObj = kNULL;

    while (!kIsNull(attrObj))
    {
        if (kStrEquals(kXml_TextFieldString(xml, &attrObj->name), name))
        {
            if (!kIsNull(prevAttrObj))
                prevAttrObj->next = attrObj->next;

            if (itemObj->firstAttr == attrObj)
                itemObj->firstAttr = attrObj->next;
            if (itemObj->lastAttr == attrObj)
                itemObj->lastAttr = prevAttrObj;

            return kXml_FreeAttr(xml, attrObj);
        }
        prevAttrObj = attrObj;
        attrObj = attrObj->next;
    }

    return kERROR_NOT_FOUND;
}

kFx(kStatus) kXml_DeleteAttrs(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    kXmlAttrClass* attrObj = kNULL;
    kXmlAttrClass* nextAttrObj = kNULL;

    kCheckArgs(!kIsNull(item));

    attrObj = itemObj->firstAttr;
    nextAttrObj = kNULL;

    while (!kIsNull(attrObj))
    {
        nextAttrObj = attrObj->next; 

        kCheck(kXml_FreeAttr(xml, attrObj)); 

        attrObj = nextAttrObj; 
    }
        
    itemObj->firstAttr = kNULL; 
    itemObj->lastAttr = kNULL; 

    return kOK;
}

kFx(kStatus) kXml_AttrString(kXml xml, kXmlItem item, const kChar* name, kString string)
{
    kXmlAttrClass* attrObj;

    kCheckArgs(!kIsNull(item));

    if (kIsNull(attrObj = kXml_FindAttrByName(xml, kXmlItem_(item), name)))
    {
        return kERROR_NOT_FOUND; 
    }
      
    return kString_Set(string, kXml_TextFieldString(xml, &attrObj->value));
}

kFx(kStatus) kXml_AttrText(kXml xml, kXmlItem item, const kChar* name, kChar* str, kSize capacity)
{
    kXmlAttrClass* attrObj;
    const kChar* value;

    kCheckArgs(!kIsNull(item));

    if (kIsNull(attrObj = kXml_FindAttrByName(xml, kXmlItem_(item), name)))
    {
        return kERROR_NOT_FOUND;
    }

    value = kXml_TextFieldString(xml, &attrObj->value);
    
    if (capacity <= kStrLength(value))
    {
        return kERROR_MEMORY;
    }

    kCheck(kStrCopy(str, capacity, value)); 

    return kOK;
}

kFx(kStatus) kXml_FindRoot(kXml xml, kXmlItem* root)
{
    return kIsNull(*root = kXml_Root(xml)) ? kERROR_FORMAT : kOK; 
}

kFx(kStatus) kXml_FindChild(kXml xml, kXmlItem parent, const kChar* path, kXmlItem* child)
{
    return kIsNull(*child = kXml_Child(xml, parent, path)) ? kERROR_FORMAT : kOK; 
}

kFx(kBool) kXml_ChildExists(kXml xml, kXmlItem parent, const kChar* path)
{
    return kIsNull(kXml_Child(xml, parent, path)) ? kFALSE : kTRUE;
}

kFx(kSize) kXml_ChildCount(kXml xml, kXmlItem parent)
{
    kXmlItemClass* parentObj = kXmlItem_(parent);
    kXmlItemClass* childObj;
    kSize count = 0;

    kAssert(!kIsNull(parent));

    childObj = parentObj->firstChild;

    while (!kIsNull(childObj))
    {
        ++count;
        childObj = childObj->next;
    }

    return count;
}

kFx(kXmlItem) kXml_ChildAt(kXml xml, kXmlItem parent, kSize index)
{
    kXmlItemClass* parentObj = kXmlItem_(parent);
    kXmlItemClass* childObj;
    kSize i;

    kAssert(!kIsNull(parent));

    childObj = parentObj->firstChild;

    for (i = 0; (i < index) && !kIsNull(childObj); ++i)
    {
        childObj = childObj->next;
    }

    return childObj;
}

kFx(kStatus) kXml_SetChildText(kXml xml, kXmlItem parent, const kChar* path, const kChar* str)
{
    kXmlItem item = kNULL; 

    kCheck(kXml_EnsureChildExists(xml, parent, path, &item)); 

    kCheck(kXml_SetItemText(xml, item, str)); 

    return kOK; 
}

kFx(kXmlItem) kXml_Child(kXml xml, kXmlItem parent, const kChar* path)
{
    const kChar* delim = kNULL;
    kXmlItemClass* foundItem = kXmlItem_(parent);

    do
    {
        kSize index;
        kSize tokenSize;

        delim = kXml_FindDelimiter(path, kXML_PATH_DELIMITERS);
        tokenSize = delim - path;
        index = kXml_ParseNameSubstrIndex(path, tokenSize, &tokenSize);

        if (!kSuccess(kXml_SetPathParseBuffer(xml, path, tokenSize)))
        {
            return kNULL;
        }

        foundItem = kXml_FindChildByName(xml, foundItem, kXml_PathParseBuffer(xml), index);
        if (kIsNull(foundItem))
        {
            break;
        }
        path = delim + 1;
    } 
    while (*delim != '\0');

    return foundItem;
}

kFx(kXmlItem) kXml_FirstChild(kXml xml, kXmlItem parent)
{
    kXmlItemClass* parentObj = kXmlItem_(parent);

    kAssert(!kIsNull(parent));
    
    return parentObj->firstChild;
}

kFx(kXmlItem) kXml_LastChild(kXml xml, kXmlItem parent)
{
    kXmlItemClass* parentObj = kXmlItem_(parent);

    kAssert(!kIsNull(parent));
    
    return parentObj->lastChild;
}

kFx(kXmlItem) kXml_Parent(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    
    kAssert(!kIsNull(item));
    
    return itemObj->parent;
}

kFx(kXmlItem) kXml_NextSibling(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    
    kAssert(!kIsNull(item));
    
    return itemObj->next;
}

kFx(kXmlItem) kXml_PreviousSibling(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    
    kAssert(!kIsNull(item));
    
    return itemObj->prev;
}

kFx(kStatus) kXml_Item16u(kXml xml, kXmlItem item, k16u* value)
{
    return kXml_Parse16u(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item16s(kXml xml, kXmlItem item, k16s* value)
{
    return kXml_Parse16s(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item32u(kXml xml, kXmlItem item, k32u* value)
{
    return kXml_Parse32u(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item32s(kXml xml, kXmlItem item, k32s* value)
{
    return kXml_Parse32s(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_ItemBool(kXml xml, kXmlItem item, kBool* value)
{
    return kXml_ParseBool(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item64u(kXml xml, kXmlItem item, k64u* value)
{
    return kXml_Parse64u(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item64s(kXml xml, kXmlItem item, k64s* value)
{
    return kXml_Parse64s(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_ItemSize(kXml xml, kXmlItem item, kSize* value)
{
    return kXml_ParseSize(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item32f(kXml xml, kXmlItem item, k32f* value)
{
    return kXml_Parse32f(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_Item64f(kXml xml, kXmlItem item, k64f* value)
{
    return kXml_Parse64f(xml, kXml_ItemValue(xml, item), value);
}

kFx(kStatus) kXml_SetItem16u(kXml xml, kXmlItem item, k16u value)
{
    return kXml_SetItemText(xml, item, kXml_Format16u(xml, value));
}

kFx(kStatus) kXml_SetItem16s(kXml xml, kXmlItem item, k16s value)
{
    return kXml_SetItemText(xml, item, kXml_Format16s(xml, value));
}

kFx(kStatus) kXml_SetItem32u(kXml xml, kXmlItem item, k32u value)
{
    return kXml_SetItemText(xml, item, kXml_Format32u(xml, value));
}

kFx(kStatus) kXml_SetItem32s(kXml xml, kXmlItem item, k32s value)
{
    return kXml_SetItemText(xml, item, kXml_Format32s(xml, value));
}

kFx(kStatus) kXml_SetItemBool(kXml xml, kXmlItem item, kBool value)
{
    return kXml_SetItemText(xml, item, kXml_FormatBool(xml, value));
}

kFx(kStatus) kXml_SetItem64u(kXml xml, kXmlItem item, k64u value)
{
    return kXml_SetItemText(xml, item, kXml_Format64u(xml, value));
}

kFx(kStatus) kXml_SetItem64s(kXml xml, kXmlItem item, k64s value)
{
    return kXml_SetItemText(xml, item, kXml_Format64s(xml, value));
}

kFx(kStatus) kXml_SetItemSize(kXml xml, kXmlItem item, kSize value)
{
    return kXml_SetItemText(xml, item, kXml_FormatSize(xml, value));
}

kFx(kStatus) kXml_SetItem32f(kXml xml, kXmlItem item, k32f value)
{
    return kXml_SetItemText(xml, item, kXml_Format32f(xml, value));
}

kFx(kStatus) kXml_SetItem64f(kXml xml, kXmlItem item, k64f value)
{
    return kXml_SetItemText(xml, item, kXml_Format64f(xml, value));
}

kFx(kStatus) kXml_Attr16u(kXml xml, kXmlItem item, const kChar* name, k16u* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }

    return kXml_Parse16u(xml, str, value);
}

kFx(kStatus) kXml_Attr16s(kXml xml, kXmlItem item, const kChar* name, k16s* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_Parse16s(xml, str, value);
}

kFx(kStatus) kXml_Attr32u(kXml xml, kXmlItem item, const kChar* name, k32u* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }

    return kXml_Parse32u(xml, str, value);
}

kFx(kStatus) kXml_Attr32s(kXml xml, kXmlItem item, const kChar* name, k32s* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_Parse32s(xml, str, value);
}

kFx(kStatus) kXml_AttrBool(kXml xml, kXmlItem item, const kChar* name, kBool* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_ParseBool(xml, str, value);
}

kFx(kStatus) kXml_Attr64u(kXml xml, kXmlItem item, const kChar* name, k64u* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_Parse64u(xml, str, value);
}

kFx(kStatus) kXml_Attr64s(kXml xml, kXmlItem item, const kChar* name, k64s* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_Parse64s(xml, str, value);
}

kFx(kStatus) kXml_AttrSize(kXml xml, kXmlItem item, const kChar* name, kSize* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_ParseSize(xml, str, value);
} 

kFx(kStatus) kXml_Attr32f(kXml xml, kXmlItem item, const kChar* name, k32f* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }
    
    return kXml_Parse32f(xml, str, value);
}

kFx(kStatus) kXml_Attr64f(kXml xml, kXmlItem item, const kChar* name, k64f* value)
{
    const kChar* str = kXml_AttrValue(xml, item, name);

    if (kIsNull(str))
    {
        return kERROR_NOT_FOUND;
    }

    return kXml_Parse64f(xml, str, value);
}

kFx(kStatus) kXml_SetAttr16u(kXml xml, kXmlItem item, const kChar* name, k16u value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format16u(xml, value));
}

kFx(kStatus) kXml_SetAttr16s(kXml xml, kXmlItem item, const kChar* name, k16s value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format16s(xml, value));
}

kFx(kStatus) kXml_SetAttr32u(kXml xml, kXmlItem item, const kChar* name, k32u value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format32u(xml, value));
}

kFx(kStatus) kXml_SetAttr32s(kXml xml, kXmlItem item, const kChar* name, k32s value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format32s(xml, value));
}

kFx(kStatus) kXml_SetAttrBool(kXml xml, kXmlItem item, const kChar* name, kBool value)
{
    return kXml_SetAttrText(xml, item, name, kXml_FormatBool(xml, value));
}

kFx(kStatus) kXml_SetAttr64u(kXml xml, kXmlItem item, const kChar* name, k64u value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format64u(xml, value));
}

kFx(kStatus) kXml_SetAttr64s(kXml xml, kXmlItem item, const kChar* name, k64s value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format64s(xml, value));
}

kFx(kStatus) kXml_SetAttrSize(kXml xml, kXmlItem item, const kChar* name, kSize value)
{
    return kXml_SetAttrText(xml, item, name, kXml_FormatSize(xml, value));
}

kFx(kStatus) kXml_SetAttr32f(kXml xml, kXmlItem item, const kChar* name, k32f value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format32f(xml, value));
}

kFx(kStatus) kXml_SetAttr64f(kXml xml, kXmlItem item, const kChar* name, k64f value)
{
    return kXml_SetAttrText(xml, item, name, kXml_Format64f(xml, value));
}

kFx(kStatus) kXml_SetChild16u(kXml xml, kXmlItem parent, const kChar* path, k16u value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format16u(xml, value));
}

kFx(kStatus) kXml_SetChild16s(kXml xml, kXmlItem parent, const kChar* path, k16s value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format16s(xml, value));
}

kFx(kStatus) kXml_SetChild32u(kXml xml, kXmlItem parent, const kChar* path, k32u value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format32u(xml, value));
}

kFx(kStatus) kXml_SetChild32s(kXml xml, kXmlItem parent, const kChar* path, k32s value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format32s(xml, value));
}

kFx(kStatus) kXml_SetChildBool(kXml xml, kXmlItem parent, const kChar* path, kBool value)
{
    return kXml_SetChildText(xml, parent, path, kXml_FormatBool(xml, value));
}

kFx(kStatus) kXml_SetChild64u(kXml xml, kXmlItem parent, const kChar* path, k64u value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format64u(xml, value));
}

kFx(kStatus) kXml_SetChild64s(kXml xml, kXmlItem parent, const kChar* path, k64s value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format64s(xml, value));
}

kFx(kStatus) kXml_SetChildSize(kXml xml, kXmlItem parent, const kChar* path, kSize value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format64s(xml, value));
}

kFx(kStatus) kXml_SetChild32f(kXml xml, kXmlItem parent, const kChar* path, k32f value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format32f(xml, value));
}

kFx(kStatus) kXml_SetChild64f(kXml xml, kXmlItem parent, const kChar* path, k64f value)
{
    return kXml_SetChildText(xml, parent, path, kXml_Format64f(xml, value));
}

kFx(kStatus) kXml_ChildString(kXml xml, kXmlItem parent, const kChar* path, kString str)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_ItemString(xml, child, str);
}

kFx(kStatus) kXml_ChildText(kXml xml, kXmlItem parent, const kChar* path, kChar* str, kSize capacity)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_ItemText(xml, child, str, capacity);
}

kFx(kStatus) kXml_Child16u(kXml xml, kXmlItem parent, const kChar* path, k16u* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_Item16u(xml, child, value);
}

kFx(kStatus) kXml_Child16s(kXml xml, kXmlItem parent, const kChar* path, k16s* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_Item16s(xml, child, value);
}

kFx(kStatus) kXml_Child32u(kXml xml, kXmlItem parent, const kChar* path, k32u* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_Item32u(xml, child, value);
}

kFx(kStatus) kXml_Child32s(kXml xml, kXmlItem parent, const kChar* path, k32s* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_Item32s(xml, child, value);
}

kFx(kStatus) kXml_ChildBool(kXml xml, kXmlItem parent, const kChar* path, kBool* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);
    
    kCheckArgs(!kIsNull(child)); 

    return kXml_ItemBool(xml, child, value);
}

kFx(kStatus) kXml_Child64u(kXml xml, kXmlItem parent, const kChar* path, k64u* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 

    return kXml_Item64u(xml, child, value);
}

kFx(kStatus) kXml_Child64s(kXml xml, kXmlItem parent, const kChar* path, k64s* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_Item64s(xml, child, value);
}

kFx(kStatus) kXml_ChildSize(kXml xml, kXmlItem parent, const kChar* path, kSize* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 
    
    return kXml_ItemSize(xml, child, value);
}

kFx(kStatus) kXml_Child32f(kXml xml, kXmlItem parent, const kChar* path, k32f* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);
    
    kCheckArgs(!kIsNull(child)); 
    
    return kXml_Item32f(xml, child, value);
}

kFx(kStatus) kXml_Child64f(kXml xml, kXmlItem parent, const kChar* path, k64f* value)
{
    kXmlItem child = kXml_Child(xml, parent, path);

    kCheckArgs(!kIsNull(child)); 

    return kXml_Item64f(xml, child, value);
}

// Private functions

kStatus kXml_TextFieldInit(kXml xml, kXmlTextField* field)
{
    field->text[0] = '\0';
    field->buffer = kNULL;
    field->capacity = 0;

    return kOK;
}

kStatus kXml_TextFieldRelease(kXml xml, kXmlTextField* field)
{
    kXmlClass* obj = kXml_Cast_(xml);

    kAlloc_Free(obj->base.alloc, field->buffer);
    field->buffer = kNULL;
    field->capacity = 0;
    return kOK;
}

kStatus kXml_TextFieldReserve(kXml xml, kXmlTextField* field, kSize size)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kSize bufferSize = size + 1;

    if (bufferSize <= kCountOf(field->text))
    {
        if (!kIsNull(field->buffer))
        {
            kAlloc_Free(obj->base.alloc, field->buffer);
            field->buffer = kNULL;
            field->capacity = 0;
        }
    }
    else
    {
        if (!kIsNull(field->buffer) && field->capacity < bufferSize)
        {
            kAlloc_Free(obj->base.alloc, field->buffer);
            field->buffer = kNULL;
            field->capacity = 0;
        }

        if (kIsNull(field->buffer))
        {
            kCheck(kAlloc_Get(obj->base.alloc, bufferSize, &field->buffer));
            field->capacity = bufferSize;
        }
    }

    return kOK;
}

kStatus kXml_TextFieldSetString(kXml xml, kXmlTextField* field, const kChar* str)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kSize bufferSize = kStrLength(str) + 1;

    if (bufferSize <= kCountOf(field->text))
    {
        if (!kIsNull(field->buffer))
        {
            kAlloc_Free(obj->base.alloc, field->buffer);
            field->buffer = kNULL;
            field->capacity = 0;
        }
        kStrCopy(field->text, kCountOf(field->text), str);
    }
    else
    {
        kAlloc_Free(obj->base.alloc, field->buffer);
        field->buffer = kNULL;

        kCheck(kAlloc_Get(obj->base.alloc, bufferSize, &field->buffer));
        field->capacity = bufferSize;

        kStrCopy(field->buffer, field->capacity, str);
    }

    return kOK;
}

kChar* kXml_TextFieldString(kXml xml, kXmlTextField* field)
{
    return (!kIsNull(field->buffer)) ? field->buffer : field->text;
}

kStatus kXml_AllocItemBlock(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlItemBlock block;
    kSize i;

    kCheck(kAlloc_Get(obj->base.alloc, sizeof(kXmlItemClass) * obj->itemBlockCount, &block.items));

    for (i = 0; i < (obj->itemBlockCount - 1); ++i)
    {
        block.items[i].next = &block.items[i+1];
    }

    block.items[obj->itemBlockCount - 1].next = obj->freeItems;
    obj->freeItems = &block.items[0];

    kCheck(kArrayList_Add(obj->itemBlocks, &block));

    return kOK;
}

kStatus kXml_AllocAttrBlock(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlAttrBlock block;
    kSize i;

    kCheck(kAlloc_Get(obj->base.alloc, sizeof(kXmlAttrClass) * obj->attrBlockCount, &block.attrs));

    for (i = 0; i < (obj->attrBlockCount - 1); ++i)
    {
        block.attrs[i].next = &block.attrs[i+1];
    }

    block.attrs[obj->attrBlockCount - 1].next = obj->freeAttrs;
    obj->freeAttrs = &block.attrs[0];

    kCheck(kArrayList_Add(obj->attrBlocks, &block));

    return kOK;
}

kStatus kXml_FreeItemBlocks(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->itemBlocks); ++i)
    {
        kXmlItemBlock* block = (kXmlItemBlock*)kArrayList_At(obj->itemBlocks, i);
        kCheck(kAlloc_Free(obj->base.alloc, block->items));
    }

    kCheck(kArrayList_Clear(obj->itemBlocks));
    obj->freeItems = kNULL;

    return kOK;
}

kStatus kXml_FreeAttrBlocks(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->attrBlocks); ++i)
    {
        kXmlAttrBlock* block = (kXmlAttrBlock*)kArrayList_At(obj->attrBlocks, i);
        kCheck(kAlloc_Free(obj->base.alloc, block->attrs));
    }

    kCheck(kArrayList_Clear(obj->attrBlocks));
    obj->freeAttrs = kNULL;

    return kOK;
}

kStatus kXml_AllocItem(kXml xml, kXmlItemClass** item)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlItemClass* newItem;

    if (kIsNull(obj->freeItems))
    {
        kCheck(kXml_AllocItemBlock(xml));
        kCheck(!kIsNull(obj->freeItems));
    }

    newItem = obj->freeItems;
    obj->freeItems = obj->freeItems->next;
    
    kCheck(kXml_TextFieldInit(xml, &newItem->name));
    kCheck(kXml_TextFieldInit(xml, &newItem->value));

    newItem->firstAttr = newItem->lastAttr = kNULL;
    newItem->parent = kNULL;
    newItem->firstChild = newItem->lastChild = kNULL;
    newItem->prev = newItem->next = kNULL;

    *item = newItem;

    return kOK;
}

kStatus kXml_FreeItem(kXml xml, kXmlItemClass* item)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlItemClass* child;
    kXmlAttrClass* attr;

    for (attr = item->firstAttr; !kIsNull(attr); )
    {
        kXmlAttrClass* next = attr->next;
        kCheck(kXml_FreeAttr(xml, attr));
        attr = next;
    }

    for (child = item->firstChild; !kIsNull(child); )
    {
        kXmlItemClass* next = child->next;
        kCheck(kXml_FreeItem(xml, child));
        child = next;
    }

    kCheck(kXml_TextFieldRelease(xml, &item->name));
    kCheck(kXml_TextFieldRelease(xml, &item->value));

    item->next = obj->freeItems;
    obj->freeItems = item;

    return kOK;
}

kStatus kXml_AllocAttr(kXml xml, kXmlAttrClass** attr)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlAttrClass* newAttr;
    
    if (kIsNull(obj->freeAttrs))
    {
        kCheck(kXml_AllocAttrBlock(xml));
        kCheck(!kIsNull(obj->freeAttrs));
    }

    newAttr = obj->freeAttrs;
    obj->freeAttrs = newAttr->next;

    kCheck(kXml_TextFieldInit(xml, &newAttr->name));
    kCheck(kXml_TextFieldInit(xml, &newAttr->value));
    newAttr->next = kNULL;

    *attr = newAttr;

    return kOK;
}

kStatus kXml_FreeAttr(kXml xml, kXmlAttrClass* attr)
{
    kXmlClass* obj = kXml_Cast_(xml);

    kCheck(kXml_TextFieldRelease(xml, &attr->name));
    kCheck(kXml_TextFieldRelease(xml, &attr->value));

    attr->next = obj->freeAttrs;
    obj->freeAttrs = attr;

    return kOK;
}

kStatus kXml_LinkItemUnder(kXml xml, kXmlItemClass* parent, kXmlItemClass* item)
{
    kXmlClass* obj = kXml_Cast_(xml);

    if (kIsNull(parent))
    {
        if (!kIsNull(obj->root))
            return kERROR_PARAMETER;
        obj->root = item;
    }
    else
    {
        item->parent = parent;
        if (parent->lastChild)
        {
            kXmlItemClass* lastChild = parent->lastChild;

            lastChild->next = item;
            item->prev = parent->lastChild;
            parent->lastChild = item;
        }
        else
        {
            parent->firstChild = item;
            parent->lastChild = item;
        }
    }

    return kOK;
}

kStatus kXml_LinkItemBefore(kXml xml, kXmlItemClass* next, kXmlItemClass* item)
{
    item->parent = next->parent;
    item->prev = next->prev;
    item->next = next;

    if (!kIsNull(next->prev))
    {
        next->prev->next = item;
    }

    next->prev = item;

    if (!kIsNull(item->parent) && kIsNull(item->prev))
    {
        item->parent->firstChild = item;
    }

    return kOK;
}

kStatus kXml_LinkItemAfter(kXml xml, kXmlItemClass* prev, kXmlItemClass* item)
{
    item->parent = prev->parent;
    item->prev = prev;
    item->next = prev->next;

    prev->next = item;

    if (!kIsNull(prev->next))
    {
        prev->next->prev = item;
    }

    if (!kIsNull(item->parent) && kIsNull(item->next))
    {
        item->parent->lastChild = item;
    }

    return kOK;
}

kStatus kXml_UnlinkItem(kXml xml, kXmlItemClass* item)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlItemClass* itemObj = kXmlItem_(item);
    kXmlItemClass* parentObj = itemObj->parent;

    if (!kIsNull(parentObj))
    {
        if (parentObj->firstChild == itemObj)
        {
            parentObj->firstChild = itemObj->next;
        }
        if (parentObj->lastChild == itemObj)
        {
            parentObj->lastChild = itemObj->prev;
        }
    }
    else
    {
        obj->root = kNULL;
    }

    if (!kIsNull(itemObj->prev))
    {
        itemObj->prev->next = itemObj->next;
    }

    if (!kIsNull(itemObj->next))
    {
        itemObj->next->prev = itemObj->prev;
    }

    return kOK;
}

kStatus kXml_LinkAttr(kXml xml, kXmlItemClass* item, kXmlAttrClass* attr)
{
    if (kIsNull(item->lastAttr))
    {
        item->firstAttr = attr;
        item->lastAttr = attr;
    }
    else
    {
        item->lastAttr->next = attr;
        item->lastAttr = attr;
    }

    return kOK;
}

kXmlAttrClass* kXml_FindAttrByName(kXml xml, kXmlItemClass* item, const kChar* name)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kXmlAttrClass* attr = item->firstAttr;

    while (!kIsNull(attr))
    {
        if (kStrEquals(kXml_TextFieldString(obj, &attr->name), name))
        {
            return attr;
        }
        attr = attr->next;
    }

    return kNULL;
}

kXmlItemClass* kXml_FindChildByName(kXml xml, kXmlItemClass* parent, const kChar* name, kSize index)
{
    kXmlClass* obj = kXml_Cast_(xml);

    if (kIsNull(parent))
    {
        // special case, item to find must be root and indexed 0
        if (kIsNull(obj->root) || index != 0)
        {
            return kNULL;
        }
        if (!kStrEquals(kXml_TextFieldString(xml, &obj->root->name), name))
        {
            return kNULL;
        }

        return obj->root;
    }
    else
    {
        kXmlItemClass* child = parent->firstChild;
        kSize i = 0;

        while (!kIsNull(child))
        {
            if (kStrEquals(kXml_TextFieldString(xml, &child->name), name))
            {
                if (i >= index)
                {
                    break;
                }
                i++;
            }
            child = child->next;
        }

        return child;
    }
}

kStatus kXml_Substr(kXml xml, const kChar* input, kSize begin, kSize end, kXmlTextField* str)
{
    kSize size = end - begin;
    kChar* cstr;

    kCheck(kXml_TextFieldReserve(xml, str, size));
    cstr = kXml_TextFieldString(xml, str);

    kCheck(kMemCopy(cstr, &input[begin], size));
    cstr[size] = '\0';
    
    return kOK;
}

kStatus kXml_UnescapeSubstr(kXml xml, const kChar* input, kSize begin, kSize end, kXmlTextField* str)
{
    kChar* writer;
    kSize i;
    kBool found;

    // a buffer that can hold the escaped string can also hold the unescaped string.
    kCheck(kXml_TextFieldReserve(xml, str, end - begin));
    writer = kXml_TextFieldString(xml, str);

    while (begin != end)
    {
        if (input[begin] == '&')
        {
            const kChar* escBegin;
            const kChar* escEnd;
            const kChar* escPos;

            ++begin;
            escBegin = &input[begin];

            while (begin != end && input[begin] != ';')
            {
                ++begin;
            }
            if (begin == end)
            {
                return kERROR;
            }
            escEnd = &input[begin];
            ++begin;

            found = kFALSE;

            for (i = 0; i < kCountOf(escSeqs); ++i)
            {
                if (strncmp(escSeqs[i].szSeq, escBegin, escEnd - escBegin) == 0)
                {
                    *(writer++) = escSeqs[i].character;
                    found = kTRUE;
                    break;
                }
            }

            if (!found)
            {
                *(writer++) = '&';

                for (escPos = escBegin; escPos <= escEnd; ++escPos)
                {
                    *(writer++) = *escPos;
                }
            }
        }
        else
        {
            *(writer++) = input[begin++];
        }
    }

    *writer = '\0';

    return kOK;
}

// This function returns a pointer to '\0' if a delimiter is not found
const kChar* kXml_FindDelimiter(const kChar* str, const kChar* delims)
{
    while (1)
    {
        const kChar* delimPtr;

        if (*str == '\0')
        {
            return str;
        }

        for (delimPtr = delims; *delimPtr != '\0'; ++delimPtr)
        {
            if (*delimPtr == *str)
            {
                return str;
            }
        }
        ++str;
    }
}

kSize kXml_ParseNameSubstrIndex(const kChar* str, kSize size, kSize* adjustedSize)
{
    const kChar* openBracket = kNULL;
    const kChar* closeBracket = kNULL;
    const kChar* ptr;
    kChar buffer[128];

    for (ptr = str; ptr < (str + size); ++ptr)
    {
        if (*ptr == '[')
        {
            openBracket = ptr;
        }
        else if (*ptr == ']')
        {
            closeBracket = ptr;
        }

        if (openBracket && closeBracket)
        {
            break;
        }
    }

    if (openBracket && closeBracket && (openBracket < closeBracket))
    {
        kSize length = closeBracket - openBracket - 1;

        if (adjustedSize)
        {
            *adjustedSize = openBracket - str;
        }

        if (length > (kCountOf(buffer) - 1))
        {
            return 0;
        }

        kMemCopy(buffer, openBracket + 1, length);
        buffer[length] = '\0';
        
        return atoi(buffer);
    }

    return 0;
}

kStatus kXml_SetPathParseBuffer(kXml xml, const kChar* text, kSize size)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kSize bufferSize = size + 1;

    if (bufferSize > kArrayList_Capacity_(obj->pathParseBuffer))
    {
        kCheck(kArrayList_Reserve(obj->pathParseBuffer, bufferSize));
    }

    kMemCopy(kArrayList_At(obj->pathParseBuffer, 0), text, size);
    *(kChar*)kArrayList_At(obj->pathParseBuffer, size) = '\0';

    return kOK;
}

const kChar* kXml_PathParseBuffer(kXml xml)
{
    kXmlClass* obj = kXml_Cast_(xml);
    return (kChar*)kArrayList_At(obj->pathParseBuffer, 0);
}

kStatus kXml_AddEscapedString(kXml xml, const kChar* in, kString out)
{
    kSize i, j;
    kSize len = kStrLength(in);
    const kChar* start;

    start = in;
    for (i = 0; i < len; ++i)
    {
        kChar c = in[i];          

        for (j = 0; j < kCountOf(escSeqs); ++j)
        {
            if (escSeqs[j].character == c)
            {
                break;
            }
        }

        // found
        if (j < kCountOf(escSeqs))
        {
            kCheck(kString_AddSubstring(out, start, 0, &in[i] - start));
            start = &in[i] + 1;
            kCheck(kString_Add(out, "&"));
            kCheck(kString_Add(out, escSeqs[j].szSeq));
            kCheck(kString_Add(out, ";"));
        }
    }

    kCheck(kString_AddSubstring(out, start, 0, in + len - start));

    return kOK;
}

kBool kXml_ValidateName(const kChar *name)
{
    kSize nameLen = kStrLength(name);
    kSize i;

    if (nameLen < 1) 
    {
        return kFALSE;
    }

    if (!kStrAsciiIsLetter(name[0]) && name[0] != '_' && name[0] != ':')
    {
        return kFALSE;
    }

    for (i = 1; i < nameLen; ++i)
    {
        if (!kXml_IsNameChar(name[i]))
        {
            return kFALSE;
        }
    }

    return kTRUE;
}

kBool kXml_IsNameChar(kChar ch)
{
    if (kStrAsciiIsLetter(ch)) 
    {
        return kTRUE;
    }

    if (kStrAsciiIsDigit(ch)) 
    {
        return kTRUE;
    }

    switch(ch)
    {
    case '.':
    case '-':
    case '_':
    case ':':
        return kTRUE;

    default:
        return kFALSE; 
    }
}

const kChar* kXml_ItemValue(kXml xml, kXmlItem item)
{
    kXmlItemClass* itemObj = kXmlItem_(item);
    
    kAssert(!kIsNull(item));

    return kXml_TextFieldString(xml, &itemObj->value);
}

const kChar* kXml_AttrValue(kXml xml, kXmlItem item, const kChar* name)
{
    kXmlAttrClass* attr = kXml_FindAttrByName(xml, kXmlItem_(item), name);

    kAssert(!kIsNull(item));
    
    if (kIsNull(attr))
    {
        return kNULL;
    }

    return kXml_TextFieldString(xml, &attr->value);
}

kStatus kXml_ItemToString(kXml xml, kString str, kXmlItemClass* item, kSize level)
{
    kXmlAttrClass* attr;
    const kChar* value;
    const kChar* name;
    kSize freeSpace;
    kSize i;

    // Some rudimentary geometric buffer growth to improve performance - should be built into kString.
    // This is very important for larger XML objects.
    freeSpace = kString_Capacity(str) - kString_Length(str);
    if (freeSpace < kXML_STRING_INIT_SIZE)
    {
        kCheck(kString_Reserve(str, kMath_Round32u_(kString_Capacity(str) * 1.5)));
    }

    value = kXml_TextFieldString(xml, &item->value);
    name = kXml_TextFieldString(xml, &item->name);

    for (i = 0; i < level; ++i)
    {
        kCheck(kString_Import(str, kXML_INDENTATION, kTRUE));
    }

    kCheck(kString_Import(str, "<", kTRUE));
    kCheck(kString_Import(str, name, kTRUE));

    attr = item->firstAttr;
    while (!kIsNull(attr))
    {
        kCheck(kString_Import(str, "\x20", kTRUE));
        kCheck(kString_Import(str, kXml_TextFieldString(xml, &attr->name), kTRUE));
        kCheck(kString_Import(str, "=\"", kTRUE));
        kCheck(kXml_AddEscapedString(xml, kXml_TextFieldString(xml, &attr->value), str));
        kCheck(kString_Import(str, "\"", kTRUE));

        attr = attr->next;
    }

    if ((kStrLength(value) > 0) || !kIsNull(item->firstChild))
    {
        kXmlItemClass* child = item->firstChild;

        kCheck(kString_Import(str, ">", kTRUE));
        if (!kIsNull(child))
        {
            kCheck(kString_Import(str, kXML_NEW_LINE, kTRUE));
        }

        kCheck(kXml_AddEscapedString(xml, value, str));

        while (!kIsNull(child))
        {
            kXml_ItemToString(xml, str, child, level + 1);
            child = child->next;
        }

        if (!kIsNull(item->firstChild))
        {
            for (i = 0; i < level; ++i)
            {
                kCheck(kString_Import(str, kXML_INDENTATION, kTRUE));
            }
        }

        kCheck(kString_Import(str, "</", kTRUE));
        kCheck(kString_Import(str, name, kTRUE));
    }
    else
    {
        kCheck(kString_Import(str, " /", kTRUE));
    }
    
    kCheck(kString_Import(str, ">", kTRUE));
    kCheck(kString_Import(str, kXML_NEW_LINE, kTRUE));

    return kOK;
}

kFx(kStatus) kXml_CopyItem(kXml xml, kXmlItem parent, kXmlItem before, kXml srcXml, kXmlItem srcItem, kXmlItem* item)
{
    kXmlItem newItem = kNULL;
    kXmlItemClass* srcItemObj = kXmlItem_(srcItem);
    kXmlAttrClass* attr;
    kXmlItem child;

    if (kIsNull(srcItem))
    {
        return kOK;
    }

    // Add item and set value
    if (kIsNull(before))
    {
        kCheck(kXml_AddItem(xml, parent, kXml_ItemName(srcXml, srcItem), &newItem));
    }
    else
    {
        if (!kIsNull(parent) && kXml_Parent(xml, before) != parent)
        {
            return kERROR_PARAMETER;
        }

        kCheck(kXml_InsertItem(xml, before, kXml_ItemName(srcXml, srcItem), &newItem));
    }
    kCheck(kXml_SetItemText(xml, newItem, kXml_ItemValue(srcXml, srcItem)));

    // Copy attributes
    attr = srcItemObj->firstAttr;
    while (!kIsNull(attr))
    {
        kCheck(kXml_SetAttrText(xml, newItem,
            kXml_TextFieldString(srcXml, &attr->name),
            kXml_TextFieldString(srcXml, &attr->value)));
        attr = attr->next;
    }

    // Copy children
    child = kXml_FirstChild(srcXml, srcItem);
    while (!kIsNull(child))
    {
        kCheck(kXml_CopyItem(xml, newItem, kNULL, srcXml, child, kNULL));
        child = kXml_NextSibling(srcXml, child);
    }

    if (!kIsNull(item))
    {
        *item = newItem;
    }

    return kOK;
}

kFx(kStatus) kXml_OverwriteItem(kXml xml, kXmlItem destItem, kXml srcXml, kXmlItem srcItem)
{
    kXmlItemClass* srcItemObj = kXmlItem_(srcItem);
    kXmlAttrClass* attr;
    kXmlItem child;

    kCheckArgs(!kIsNull(destItem) && !kIsNull(srcXml) && !kIsNull(srcItem)); 

    kCheck(kXml_ClearItem(xml, destItem)); 

    //copy name
    kCheck(kXml_SetItemName(xml, destItem, kXml_ItemName(srcXml, srcItem))); 

    //copy attributes
    attr = srcItemObj->firstAttr;
    while (!kIsNull(attr))
    {
        kCheck(kXml_SetAttrText(xml, destItem, kXml_TextFieldString(srcXml, &attr->name), kXml_TextFieldString(srcXml, &attr->value)));
        attr = attr->next;
    }

    //copy children
    child = kXml_FirstChild(srcXml, srcItem);
    while (!kIsNull(child))
    {
        kCheck(kXml_CopyItem(xml, destItem, kNULL, srcXml, child, kNULL));
        child = kXml_NextSibling(srcXml, child);
    }

    return kOK;       
}


kFx(kStatus) kXml_ClearItem(kXml xml, kXmlItem item)
{
    kCheckArgs(!kIsNull(item)); 

    kCheck(kXml_DeleteChildren(xml, item)); 
    kCheck(kXml_DeleteAttrs(xml, item)); 

    return kOK; 
}

const kChar* kXml_Format16u(kXml xml, k16u val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(k16u_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format16s(kXml xml, k16s val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(k16s_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format32u(kXml xml, k32u val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(k32u_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format32s(kXml xml, k32s val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(k32s_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_FormatBool(kXml xml, kBool val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(kBool_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format64u(kXml xml, k64u val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(k64u_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format64s(kXml xml, k64s val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(k64s_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_FormatSize(kXml xml, kSize val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(kSize_Format(val, obj->formatBuffer, kCountOf(obj->formatBuffer))))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format32f(kXml xml, k32f val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(kStrFormat32f(val, obj->formatBuffer, kCountOf(obj->formatBuffer), 10, kNULL)))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

const kChar* kXml_Format64f(kXml xml, k64f val)
{
    kXmlClass* obj = kXml_Cast_(xml); 

    if (!kSuccess(kStrFormat64f(val, obj->formatBuffer, kCountOf(obj->formatBuffer), 10, kNULL)))
    {
        return kNULL;
    }

    return obj->formatBuffer;
}

kStatus kXml_Parse16u(kXml xml, const kChar *string, k16u *val)
{
    return k16u_Parse(val, string);
}

kStatus kXml_Parse16s(kXml xml, const kChar *string, k16s *val)
{
    return k16s_Parse(val, string);
}

kStatus kXml_Parse32u(kXml xml, const kChar *string, k32u *val)
{
    return k32u_Parse(val, string);
}

kStatus kXml_Parse32s(kXml xml, const kChar *string, k32s *val)
{
    return k32s_Parse(val, string);
}

kStatus kXml_ParseBool(kXml xml, const kChar *string, kBool *val)
{
    return kBool_Parse(val, string);
}

kStatus kXml_Parse64u(kXml xml, const kChar *string, k64u *val)
{
    return k64u_Parse(val, string);
}

kStatus kXml_Parse64s(kXml xml, const kChar *string, k64s *val)
{
    return k64s_Parse(val, string);
}

kStatus kXml_ParseSize(kXml xml, const kChar *string, kSize *val)
{
    return kSize_Parse(val, string);
}

kStatus kXml_Parse32f(kXml xml, const kChar *string, k32f *val)
{
    return k32f_Parse(val, string);
}

kStatus kXml_Parse64f(kXml xml, const kChar *string, k64f *val)
{
    return k64f_Parse(val, string);
}

kStatus kXml_AdvanceParser(kXmlParseContext* context)
{
    kXmlClass* obj = kXml_Cast_(context->xml);

    if (context->isStream)
    {
        kChar ch;
        kStatus status;

        if (context->eof)
        {
            return kERROR;
        }

        status = kStream_Read(context->stream, &ch, sizeof(kChar));
        if (!kSuccess(status))
        {
            context->eof = kTRUE;
            return kERROR;
        }

        if ((context->size + 1) > context->capacity)
        {
            kChar* newBuffer;
            kSize position = context->position - context->buffer;

            kCheck(kAlloc_Get(obj->base.alloc, context->capacity * 2, &newBuffer));
            kCheck(kMemCopy(newBuffer, context->buffer, context->size));
            kCheck(kAlloc_Free(obj->base.alloc, context->buffer));
            context->buffer = kNULL;
            context->buffer = newBuffer;
            context->capacity = context->capacity * 2;

            context->position = context->buffer + position;
        }
        context->buffer[context->size] = ch;
        ++context->size;
        ++context->position;
    }
    else
    {
        if (*context->position == '\0')
        {
            return kERROR;
        }
        ++context->position;
    }
    return kOK;
}

kStatus kXml_SkipWhitespace(kXmlParseContext* context)
{
    while (kStrAsciiIsSpace(*context->position))
    {
        kCheck(kXml_AdvanceParser(context));
    }
    return kOK;
}

kStatus kXml_SkipToken(kXmlParseContext* context)
{
    while (!kStrAsciiIsSpace(*context->position) &&
        *context->position != '<' &&
        *context->position != '/' &&
        *context->position != '>' &&
        *context->position != '=')
    {
        kCheck(kXml_AdvanceParser(context));
    }

    return kOK;
}

kStatus kXml_Parse(kXml xml, kXmlParseContext* context)
{
    kXmlClass* obj = kXml_Cast_(xml);
    kSize valueBegin;
    kSize valueEnd;
    kXmlTagType tagType = TAG_CLOSE;

    kXml_Clear(xml);

    while (*context->position != '\0')
    {
        if (*context->position == '<')
        {
            valueEnd = context->position - context->buffer;

            kCheck(kXml_AdvanceParser(context));
            kCheck(kXml_SkipWhitespace(context));

            // Comment/declaration parsing is a bit sloppy and probably not entirely spec-conforming but should do for our purposes.
            // TODO: make this more rigorous
            if (*context->position == '!')
            {
                kCheck(kXml_ParseComment(xml, context));
            }
            else if (*context->position == '?')
            {
                kCheck(kXml_ParseDeclaration(xml, context));
            }
            else
            {
                kXmlTagType prevTagType = tagType;
                kXmlItemClass* prevNode = context->currentItem;
                kBool done = kFALSE;
                
                kCheck(kXml_ParseTag(xml, context, &tagType));

                // logic in this part needs some clean up
                if (kIsNull(context->currentItem) && (tagType == TAG_CLOSE || tagType == TAG_EMPTY))
                {
                    // root element is found (but don't stop yet, not necessarily completed)
                    done = kTRUE;
                }

                if (!done)
                {
                    kCheck(kXml_AdvanceParser(context));
                }
 
                if (tagType == TAG_OPEN)
                {
                    valueBegin = context->position - context->buffer;
                }
                else if (tagType == TAG_CLOSE)
                {
                    // this detects a non-nested open/close pair. content within is considered as value.
                    // note: this doesn't support text nodes along side other nodes, but this isn't something
                    // we use. In fact there's no way on the API level to access this (only a single "value"
                    // per node, which isn't enough to represent text nodes).
                    // TODO: add this?
                    if (prevTagType == TAG_OPEN)
                    {
                        kCheck(kXml_UnescapeSubstr(xml, context->buffer, valueBegin, valueEnd, &prevNode->value));
                    }
                    else
                    {
                        kCheck(kXml_TextFieldSetString(xml, &prevNode->value, ""));
                    }
                }

                if (done)
                {
                    break;
                }
            }
        }
        else
        {
            if (*context->position != '\0')
            {
                kCheck(kXml_AdvanceParser(context));
            }
        }
    }

    if (!kIsNull(context->currentItem) || kIsNull(obj->root))
    {
        return kERROR;
    }

    return kOK;
}

kStatus kXml_ParseTag(kXml xml, kXmlParseContext* context, kXmlTagType* tagType)
{
    kSize tokenBegin;
    kSize tokenEnd;

    // read name
    kCheck(kXml_SkipWhitespace(context));

    tokenBegin = context->position - context->buffer;
    
    kCheck(kXml_AdvanceParser(context));
    kCheck(kXml_SkipToken(context));
    
    if (!kStrAsciiIsSpace(*context->position) && *context->position != '/' && *context->position != '>')
    {
        return kERROR;
    }

    tokenEnd = context->position - context->buffer;

    if (tokenBegin >= tokenEnd)
    {
        return kERROR; // zero length token is bad
    }

    if (context->buffer[tokenBegin] == '/') // closing item
    {       
        if (strncmp(kXml_TextFieldString(xml, &context->currentItem->name), &context->buffer[tokenBegin + 1], tokenEnd - tokenBegin - 1) != 0)
        {
            return kERROR; // mismatched tags
        }

        context->currentItem = context->currentItem->parent;
        *tagType = TAG_CLOSE;
    }
    else // beginning item
    {
        kXmlItemClass* parentItem;

        parentItem = context->currentItem;
        kCheck(kXml_AllocItem(xml, &context->currentItem));
        kCheck(kXml_LinkItemUnder(xml, parentItem, context->currentItem));

        kCheck(kXml_Substr(xml, context->buffer, tokenBegin, tokenEnd, &context->currentItem->name));
        kCheck(kXml_ParseAttributes(xml, context));

        // parse until end
        kCheck(kXml_SkipWhitespace(context));

        if (*context->position == '/') // empty item
        {
            kCheck(kXml_AdvanceParser(context));
            kCheck(kXml_TextFieldSetString(xml, &context->currentItem->value, ""));
            context->currentItem = parentItem;
            *tagType = TAG_EMPTY;
        }
        else
        {
            *tagType = TAG_OPEN;
        }
    }

    kCheck(kXml_SkipWhitespace(context));

    if (*context->position != '>')
    {
        return kERROR;
    }

    return kOK;
}

kStatus kXml_ParseAttributes(kXml xml, kXmlParseContext* context)
{
    kSize tokenBegin; // use index because pointer can change during parsing
    kSize tokenEnd;
    kXmlAttrClass* attr = kNULL;

    while (kTRUE)
    {
        kCheck(kXml_SkipWhitespace(context));

        if (*context->position == '/' || *context->position == '>')
        {
            break;
        }
        tokenBegin = context->position - context->buffer;
        kCheck(kXml_SkipToken(context));
        tokenEnd = context->position - context->buffer;
        
        if (!kStrAsciiIsSpace(context->buffer[tokenEnd]) &&
            context->buffer[tokenEnd] != '/' &&
            context->buffer[tokenEnd] != '>' &&
            context->buffer[tokenEnd] != '=')
        {
            return kERROR;
        }

        // allocate and insert
        kCheck(kXml_AllocAttr(xml, &attr));
        kCheck(kXml_LinkAttr(xml, context->currentItem, attr));
        kCheck(kXml_Substr(xml, context->buffer, tokenBegin, tokenEnd, &attr->name));

        // parse value
        kCheck(kXml_SkipWhitespace(context));

        if (*context->position == '=')
        {
            kChar quote;

            kCheck(kXml_AdvanceParser(context));
            kCheck(kXml_SkipWhitespace(context));

            quote = *context->position;

            if (quote != '\'' && quote != '\"')
            {
                return kERROR;
            }

            kCheck(kXml_AdvanceParser(context));
            tokenBegin = context->position - context->buffer;
            
            while (*context->position != '\0' && *context->position != quote)
            {
                kCheck(kXml_AdvanceParser(context));
            }

            tokenEnd = context->position - context->buffer;
            
            if (context->buffer[tokenEnd] != quote)
            {
                return kERROR;
            }

            kCheck(kXml_Substr(xml, context->buffer, tokenBegin, tokenEnd, &attr->value));
            kCheck(kXml_AdvanceParser(context));
        }
        else
        {
            kCheck(kXml_TextFieldSetString(xml, &attr->value, ""));
        }
    }

    return kOK;
}

kStatus kXml_ParseComment(kXml xml, kXmlParseContext* context)
{
    k32u state = 0;

    kCheck(kXml_AdvanceParser(context));

    if (*context->position != '-')
    {
        return kERROR;
    }

    kCheck(kXml_AdvanceParser(context));
    
    if (*context->position != '-')
    {
        return kERROR;
    }

    kCheck(kXml_AdvanceParser(context));

    while (*context->position != '\0')
    {
        kChar c = *context->position;

        switch(state)
        {
        case 0:
            if (c == '-')
                state = 1;
            break;
        case 1:
            if (c == '-')
                state = 2;
            else
                state = 0;
            break;
        case 2:
            if (c == '>')
            {
                kCheck(kXml_AdvanceParser(context));
                return kOK;
            }
            else if (c != '-')
            {
                state = 0;
            }
            break;
        }

        kCheck(kXml_AdvanceParser(context));
    }
    return kERROR;
}

kStatus kXml_ParseDeclaration(kXml xml, kXmlParseContext* context)
{
    k32u state = 0;

    kCheck(kXml_AdvanceParser(context));

    while (*context->position != '\0')
    {
        switch(state)
        {
        case 0:
            if (*context->position == '?')
            {
                state = 1;
            }
            break;
        case 1:
            if (*context->position == '>')
            {
                kCheck(kXml_AdvanceParser(context));
                return kOK;
            }
            else if (*context->position != '?')
            {
                state = 0;
            }
            break;
        }

        kCheck(kXml_AdvanceParser(context));
    }

    return kERROR;
}
