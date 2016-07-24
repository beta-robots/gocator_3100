/** 
 * @file    kXml.h
 * @brief   Declares the kXml class. 
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_XML_H
#define K_API_XML_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kXml
 * @extends kObject
 * @ingroup kApi-Data
 * @brief   Represents an XML document.
 *
 * kXml supports the kObject_Clone method. 
 *
 * kXml supports the kdat6 serialization protocol. 
 */
//typedef kObject kXml;       --forward-declared in kApiDef.x.h 

/**
 * Represents an XML element within an XML document.
 *
 * @typedef kPointer kXmlItem 
 * @relates kXml
 */
//typedef kPointer kXmlItem;       --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kXml object. 
 *
 * The kXml object is initially empty; use kXml_AddItem with parent=kNULL to create a root node.
 *
 * @public                  @memberof kXml
 * @param   xml             Destination for the constructed object handle.
 * @param   allocator       Memory allocator.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Construct(kXml* xml, kAlloc allocator);

/** 
 * Loads an XML document from a string object.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   str             String object. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_FromString(kXml xml, kString str); 

/** 
 * Exports the XML document to a string object.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   str             String object to receive the XML content.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ToString(kXml xml, kString str);

/** 
 * Saves an XML document to an in-memory file.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   data            Receives the file data. 
 * @param   size            Receives the file size.
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SaveBytes(kXml xml, kByte** data, kSize* size, kAlloc allocator); 

/** 
 * Loads an XML document from an in-memory file.
 *
 * @public                  @memberof kXml
 * @param   xml             Destination for the constructed object handle.
 * @param   data            File data.
 * @param   size            File size. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_LoadBytes(kXml* xml, const kByte* data, kSize size, kAlloc allocator); 

/** 
 * Loads an XML document from file.
 *
 * @public                  @memberof kXml
 * @param   xml             Destination for the constructed object handle.
 * @param   fileName        Name of the file.
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Load(kXml* xml, const kChar* fileName, kAlloc allocator);

/** 
 * Loads an XML document from a stream.
 *
 * @public                  @memberof kXml
 * @param   xml             XML object. 
 * @param   stream          Stream for reading.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Read(kXml xml, kStream stream);

/** 
 * Writes an XML document to a stream.
 *
 * @public                  @memberof kXml
 * @param   xml             XML object. 
 * @param   stream          Stream for writing.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Write(kXml xml, kStream stream);

/** 
 * Loads an XML document from a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             Destination for the constructed object handle.
 * @param   str             Null-terminated character array to parse.
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_FromText(kXml* xml, const kChar* str, kAlloc allocator);

/** 
 * Compacts the XML object for minimum memory usage.
 *
 * Compacting a document renders existing kXmlItem handles invalid.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Compact(kXml xml);

/** 
 * Saves the XML document object to file.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   fileName        Name of the file.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Save(kXml xml, const kChar* fileName); 

/** 
 * Removes all elements from the XML document. 
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Clear(kXml xml);

/** 
 * Copies the source document. 
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   source          Source document to be copied. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Assign(kXml xml, kXml source);

/** 
 * Returns the root element of the XML document.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @return                  XML root node. 
 */
kFx(kXmlItem) kXml_Root(kXml xml); 

/** 
 * Returns the child node at the given relative path.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @return                  Requested child node, or kNULL if not found.
 */
kFx(kXmlItem) kXml_Child(kXml xml, kXmlItem parent, const kChar* path);

/** 
 * Returns the parent node of the given element.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @return                  Parent node, or kNULL if none.
 */
kFx(kXmlItem) kXml_Parent(kXml xml, kXmlItem item); 

/** 
 * Returns the first child element of the given parent node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node.
 * @return                  First child node, or kNULL if none.
 */
kFx(kXmlItem) kXml_FirstChild(kXml xml, kXmlItem parent);

/** 
 * Returns the last child element of the given parent node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @return                  Last child node, or kNULL if none.
 */
kFx(kXmlItem) kXml_LastChild(kXml xml, kXmlItem parent); 

/** 
 * Returns the next sibling element of the given element node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @return                  Next sibling node, or kNULL if none.
 */
kFx(kXmlItem) kXml_NextSibling(kXml xml, kXmlItem item); 

/** 
 * Returns the previous sibling element of the given element node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @return                  Previous sibling node, or kNULL if none.
 */
kFx(kXmlItem) kXml_PreviousSibling(kXml xml, kXmlItem item);

/** 
 * Finds the root node, if it exists. 
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   root            Receives root node, if it exists. 
 * @return                  kOperation status (kERROR_FORMAT if not found). 
 */
kFx(kStatus) kXml_FindRoot(kXml xml, kXmlItem* root);

/** 
 * Finds the child node at the given relative path, if it exists. 
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   child           Receives child node, if it exists. 
 * @return                  kOperation status (kERROR_FORMAT if not found). 
 */
kFx(kStatus) kXml_FindChild(kXml xml, kXmlItem parent, const kChar* path, kXmlItem* child);

/** 
 * Reports whether a child node exists at the specified relative path.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @return                  kTRUE if child exists. kFALSE otherwise. 
 */
kFx(kBool) kXml_ChildExists(kXml xml, kXmlItem parent, const kChar* path);

/** 
 * Returns the number of child nodes for the given parent node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @return                  Count of child nodes.
 */
kFx(kSize) kXml_ChildCount(kXml xml, kXmlItem parent); 

/** 
 * Returns a child node at a specific index within the list of child nodes 
 * for the given parent node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node.  
 * @param   index           Child node index.
 * @return                  Requested child node, or kNULL if not found.
 */
kFx(kXmlItem) kXml_ChildAt(kXml xml, kXmlItem parent, kSize index);

/** 
 * Ensures that a child node exists at the specified path. 
 * 
 * This method creates any missing nodes that are necessary for the path to be complete. 
 * If a child already exists at the specified path, the existing child is returned.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   item            Destination for the item handle.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_EnsureChildExists(kXml xml, kXmlItem parent, const kChar* path, kXmlItem* item);  

/** 
 * Deletes all children in the specified parent item
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Parent node. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_DeleteChildren(kXml xml, kXmlItem item);  

/** 
 * Inserts a new child node at the end of the specified parent node's child list.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   name            Name of the new node.
 * @param   item            Destination for the item handle.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_AddItem(kXml xml, kXmlItem parent, const kChar* name, kXmlItem* item);  

/** 
 * Inserts a new node before the specified sibling node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   before          Sibling node. 
 * @param   name            Name of the new node.
 * @param   item            Destination for the item handle.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_InsertItem(kXml xml, kXmlItem before, const kChar* name, kXmlItem* item);
 
/** 
 * Copies a node from another XML document to this document, inserting a new node at the specified location.
 *
 * @public                  @memberof kXml
 * @param   xml             Destination XML document. 
 * @param   parent          Destination parent element.
 * @param   before          Destination sibling element (kNULL appends to end of parent's child list). 
 * @param   srcXml          Source XML document.
 * @param   srcItem         Source XML element.
 * @param   item            Destination for the copied item handle.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_CopyItem(kXml xml, kXmlItem parent, kXmlItem before, kXml srcXml, kXmlItem srcItem, kXmlItem* item); 

/** 
 * Copies a node from another XML document to this document, overwriting an existing element. 
 *
 * @public                  @memberof kXml
 * @param   xml             XML destination document. 
 * @param   destItem        XML destination element. 
 * @param   srcXml          Source XML document.
 * @param   srcItem         Source XML element.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_OverwriteItem(kXml xml, kXmlItem destItem, kXml srcXml, kXmlItem srcItem); 

/** 
 * Removes all children, attributes, and value from the XML element node. 
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ClearItem(kXml xml, kXmlItem item); 

/** 
 * Returns the name of an XML element node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node.
 * @return                  Element name.
 */
kFx(const kChar*) kXml_ItemName(kXml xml, kXmlItem item); 

/** 
 * Sets the name of an XML element node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Element name. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItemName(kXml xml, kXmlItem item, const kChar* name);  

/** 
 * Deletes an XML element node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @param   item            Element node. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_DeleteItem(kXml xml, kXmlItem item);

/** 
 * Gets XML element content as a character array.
 *
 * Use the kMemFree function to deallocate the received character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   str             Receives null-terminated character array. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ItemStringZen(kXml xml, kXmlItem item, kChar** str); 

/** 
 * Gets XML element node content as a string object.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   str             String object.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ItemString(kXml xml, kXmlItem item, kString str);

/** 
 * Gets XML element content as a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   str             Character array. 
 * @param   capacity        Character array capacity, in bytes (includes null-terminator). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ItemText(kXml xml, kXmlItem item, kChar* str, kSize capacity); 

/** 
 * Gets XML element node content as a k16u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item16u(kXml xml, kXmlItem item, k16u* value);

/** 
 * Gets XML element node content as a k16s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item16s(kXml xml, kXmlItem item, k16s* value); 

/** 
 * Gets XML element node content as a k32u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item32u(kXml xml, kXmlItem item, k32u* value);

/** 
 * Gets XML element node content as a k32s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item32s(kXml xml, kXmlItem item, k32s* value);  

/** 
 * Gets XML element node content as a kBool value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Returns XML content as a boolean value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ItemBool(kXml xml, kXmlItem item, kBool* value);

/** 
 * Gets XML element node content as a k64u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item64u(kXml xml, kXmlItem item, k64u* value); 

/** 
 * Gets XML element node content as a k64s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item64s(kXml xml, kXmlItem item, k64s* value); 

/** 
 * Gets XML element node content as a kSize value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value            Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ItemSize(kXml xml, kXmlItem item, kSize* value);

/** 
 * Gets XML element node content as a k32f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @param   item            Element node. 
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item32f(kXml xml, kXmlItem item, k32f* value);  

/** 
 * Gets XML element node content as a k64f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Item64f(kXml xml, kXmlItem item, k64f* value); 

/** 
 * Sets XML element node content from a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   str             Character array. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItemText(kXml xml, kXmlItem item, const kChar* str); 

/** 
 * Sets XML element node content from a k16u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem16u(kXml xml, kXmlItem item, k16u value);  

/** 
 * Sets XML element node content from a k16s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem16s(kXml xml, kXmlItem item, k16s value); 

/** 
 * Sets XML element node content from a k32u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem32u(kXml xml, kXmlItem item, k32u value);  

/** 
 * Sets XML element node content from a k32s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem32s(kXml xml, kXmlItem item, k32s value); 

/** 
 * Sets XML element node content from a kBool value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Boolean value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItemBool(kXml xml, kXmlItem item, kBool value);

/** 
 * Sets XML element node content from a k64u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem64u(kXml xml, kXmlItem item, k64u value); 

/** 
 * Sets XML element node content from a k64s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem64s(kXml xml, kXmlItem item, k64s value); 

/** 
 * Sets XML element node content from a kSize value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItemSize(kXml xml, kXmlItem item, kSize value); 

/** 
 * Sets XML element node content from a k32f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem32f(kXml xml, kXmlItem item, k32f value); 

/** 
 * Sets XML element node content from a k64f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @param   item            Element node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetItem64f(kXml xml, kXmlItem item, k64f value); 

/** 
 * Gets XML element node content as a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   str             Receives null-terminated character array. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ChildStringZen(kXml xml, kXmlItem parent, const kChar* path, kChar** str);

/** 
 * Gets XML child node content as a string object.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   str             String object.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ChildString(kXml xml, kXmlItem parent, const kChar* path, kString str);

/** 
 * Gets XML child node content as a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   str             Character array. 
 * @param   capacity        Character array capacity, in bytes (includes null-terminator). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ChildText(kXml xml, kXmlItem parent, const kChar* path, kChar* str, kSize capacity);  

/** 
 * Gets XML child node content as a k16u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child16u(kXml xml, kXmlItem parent, const kChar* path, k16u* value); 

/** 
 * Gets XML child node content as a k16s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child16s(kXml xml, kXmlItem parent, const kChar* path, k16s* value); 

/** 
 * Gets XML child node content as a k32u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child32u(kXml xml, kXmlItem parent, const kChar* path, k32u* value); 

/** 
 * Gets XML child node content as a k32s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child32s(kXml xml, kXmlItem parent, const kChar* path, k32s* value); 

/** 
 * Gets XML child node content as a kBool value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a boolean value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ChildBool(kXml xml, kXmlItem parent, const kChar* path, kBool* value);

/** 
 * Gets XML child node content as a k64u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child64u(kXml xml, kXmlItem parent, const kChar* path, k64u* value); 

/** 
 * Gets XML child node content as a k64s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child64s(kXml xml, kXmlItem parent, const kChar* path, k64s* value); 

/** 
 * Gets XML child node content as a kSize value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_ChildSize(kXml xml, kXmlItem parent, const kChar* path, kSize* value); 

/** 
 * Gets XML child node content as a k32f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          XML document. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child32f(kXml xml, kXmlItem parent, const kChar* path, k32f* value); 

/** 
 * Gets XML child node content as a k64f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          XML document. 
 * @param   path            Path relative to the parent node.
 * @param   value           Returns XML content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Child64f(kXml xml, kXmlItem parent, const kChar* path, k64f* value); 
/** 
 * Sets XML child node content from a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   str             Character array. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChildText(kXml xml, kXmlItem parent, const kChar* path, const kChar* str); 

/** 
 * Sets XML child node content from a k16u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild16u(kXml xml, kXmlItem parent, const kChar* path, k16u value);

/** 
 * Sets XML child node content from a k16s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild16s(kXml xml, kXmlItem parent, const kChar* path, k16s value); 

/** 
 * Sets XML child node content from a k32u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document object.
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node. 
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild32u(kXml xml, kXmlItem parent, const kChar* path, k32u value);

/** 
 * Sets XML child node content from a k32s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild32s(kXml xml, kXmlItem parent, const kChar* path, k32s value); 

/** 
 * Sets XML child node content from a kBool value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Boolean value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChildBool(kXml xml, kXmlItem parent, const kChar* path, kBool value);

/** 
 * Sets XML child node content from a k64u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild64u(kXml xml, kXmlItem parent, const kChar* path, k64u value);  

/** 
 * Sets XML child node content from a k64s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild64s(kXml xml, kXmlItem parent, const kChar* path, k64s value); 

/** 
 * Sets XML child node content from a kSize value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChildSize(kXml xml, kXmlItem parent, const kChar* path, kSize value); 

/** 
 * Sets XML child node content from a k32f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild32f(kXml xml, kXmlItem parent, const kChar* path, k32f value); 
 
/** 
 * Sets XML child node content from a k64f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   parent          Parent node. 
 * @param   path            Path relative to the parent node.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetChild64f(kXml xml, kXmlItem parent, const kChar* path, k64f value); 

/** 
 * Reports whether a specific attribute exists for the given XML element.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @return                  kTRUE if attribute exists. kFALSE otherwise. 
 */
kFx(kBool) kXml_AttrExists(kXml xml, kXmlItem item, const kChar* name); 

/** 
 * Returns the number of attributes for the given XML element.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @return                  Number of attributes. 
 */
kFx(kSize) kXml_AttrCount(kXml xml, kXmlItem item);  

/** 
 * Returns an attribute at a specific index within the list of attributes for the given element node.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   index           Attribute index.
 * @return                  Attribute name. 
 */
kFx(const kChar*) kXml_AttrNameAt(kXml xml, kXmlItem item, kSize index); 

/** 
 * Deletes an XML attribute.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_DeleteAttr(kXml xml, kXmlItem item, const kChar* name); 

/** 
 * Deletes all XML attributes.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_DeleteAttrs(kXml xml, kXmlItem item); 

/** 
 * Gets XML attribute content as a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   str             Character array.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_AttrStringZen(kXml xml, kXmlItem item, const kChar* name, kChar** str);

/** 
 * Gets XML attribute content as a string object.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   string          String object.
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_AttrString(kXml xml, kXmlItem item, const kChar* name, kString string);

/** 
 * Gets XML attribute content as a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   str             Character array.
 * @param   capacity        Character array capacity, in bytes (includes null-terminator). 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_AttrText(kXml xml, kXmlItem item, const kChar* name, kChar* str, kSize capacity);

/** 
 * Gets XML attribute content as a k16u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr16u(kXml xml, kXmlItem item, const kChar* name, k16u* value); 

/** 
 * Gets XML attribute content as a k16s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr16s(kXml xml, kXmlItem item, const kChar* name, k16s* value); 

/** 
 * Gets XML attribute content as a k32u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr32u(kXml xml, kXmlItem item, const kChar* name, k32u* value); 

/** 
 * Gets XML attribute content as a k32s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr32s(kXml xml, kXmlItem item, const kChar* name, k32s* value); 

/** 
 * Gets XML attribute content as a kBool value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a boolean value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_AttrBool(kXml xml, kXmlItem item, const kChar* name, kBool* value);

/** 
 * Gets XML attribute content as a k64u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr64u(kXml xml, kXmlItem item, const kChar* name, k64u* value); 

/** 
 * Gets XML attribute content as a k64s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr64s(kXml xml, kXmlItem item, const kChar* name, k64s* value); 

/** 
 * Gets XML attribute content as a kSize value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_AttrSize(kXml xml, kXmlItem item, const kChar* name, kSize* value); 

/** 
 * Gets XML attribute content as a k32f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr32f(kXml xml, kXmlItem item, const kChar* name, k32f* value); 

/** 
 * Gets XML attribute content as a k64f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Returns attribute content as a numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_Attr64f(kXml xml, kXmlItem item, const kChar* name, k64f* value);

/** 
 * Sets XML attribute content from a character array.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   str             Character array. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttrText(kXml xml, kXmlItem item, const kChar* name, const kChar* str); 

/** 
 * Sets XML attribute content from a k16u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr16u(kXml xml, kXmlItem item, const kChar* name, k16u value);

/** 
 * Sets XML attribute content from a k16s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr16s(kXml xml, kXmlItem item, const kChar* name, k16s value);

/** 
 * Sets XML attribute content from a k32u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr32u(kXml xml, kXmlItem item, const kChar* name, k32u value);

/** 
 * Sets XML attribute content from a k32s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr32s(kXml xml, kXmlItem item, const kChar* name, k32s value);

/** 
 * Sets XML attribute content from a kBool value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Boolean value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttrBool(kXml xml, kXmlItem item, const kChar* name, kBool value);

/** 
 * Sets XML attribute content from a k64u value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr64u(kXml xml, kXmlItem item, const kChar* name, k64u value);

/** 
 * Sets XML attribute content from a k64s value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr64s(kXml xml, kXmlItem item, const kChar* name, k64s value);

/** 
 * Sets XML attribute content from a kSize value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttrSize(kXml xml, kXmlItem item, const kChar* name, kSize value);

/** 
 * Sets XML attribute content from a k32f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr32f(kXml xml, kXmlItem item, const kChar* name, k32f value);

/** 
 * Sets XML attribute content from a k64f value.
 *
 * @public                  @memberof kXml
 * @param   xml             XML document. 
 * @param   item            Element node. 
 * @param   name            Attribute name.
 * @param   value           Numeric value. 
 * @return                  Operation status. 
 */
kFx(kStatus) kXml_SetAttr64f(kXml xml, kXmlItem item, const kChar* name, k64f value);

kEndHeader()

#include <kApi/Data/kXml.x.h>

#endif

