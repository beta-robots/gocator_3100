/** 
 * @file    kType.h
 * @brief   Declares the kType class. 
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_TYPE_H
#define K_API_TYPE_H

kBeginHeader()

/** @relates kType @{ */
#define kTypeName kText64       ///< Alias for type used to store a kType text name. 
/** @} */

/**
 * @struct  kTypeFlags
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a set of type flags. 
 */
typedef k32u kTypeFlags; 

/** @relates kTypeFlags @{ */
#define kTYPE_FLAGS_CLASS          (0x01)       ///< Type is a class.
#define kTYPE_FLAGS_INTERFACE      (0x02)       ///< Type is an interface. 
#define kTYPE_FLAGS_VALUE          (0x04)       ///< Type is a value. 
#define kTYPE_FLAGS_ENUM           (0x08)       ///< Type is an enumeration.
#define kTYPE_FLAGS_BIT_ENUM       (0x10)       ///< Type is a bitset enumeration. 
#define kTYPE_FLAGS_ABSTRACT       (0x20)       ///< Type is an abstract class. 
#define kTYPE_FLAGS_ARRAY_VALUE    (0x40)       ///< Type is an array-based value (e.g. kText32).
/** @} */

/**
 * @struct  kTypeVersion
 * @typedef kPointer kTypeVersion
 * @relates kType
 * @brief   Represents an opaque reference to type version information (used in object serialization). 
 */
typedef kPointer kTypeVersion; 

/**
 * @struct  kMethodInfo
 * @ingroup kApi  
 * @brief   Represents type method information. 
 */
typedef struct kMethodInfo
{
    kTypeName methodName;               ///< Method name (e.g. "Clone"). 
    kTypeName functionName;             ///< Full function name (e.g. "kObject_Clone"). 
    kFunction function;                 ///< Pointer to function.
} kMethodInfo; 

/**
 * @struct  kFieldInfo
 * @ingroup kApi  
 * @brief   Represents type field information. 
 */
typedef struct kFieldInfo
{
    kTypeName name;                     ///< Field name. 
    kType type;                         ///< Field type.
    kSize offset;                       ///< Offset of field within structure (bytes).
    kSize count;                        ///< Count of values in this field (typically 1; can be higher for "array value" fields, e.g. kText32). 
} kFieldInfo; 

/**
 * @struct  kEnumeratorInfo
 * @ingroup kApi  
 * @brief   Represents enumerator information. 
 */
typedef struct kEnumeratorInfo
{
    k32s value;                         ///< Enumerator numeric value.
    kTypeName name;                     ///< Enumerator name (e.g. "kPIXEL_FORMAT_8BPP_GREYSCALE"). 
    kTypeName displayName;              ///< Formatted display name (e.g. "8bpp Greyscale"); 
} kEnumeratorInfo; 

/**
* @struct  kTypeVersionInfo
* @ingroup kApi
* @brief   Represents serialization version information.
*/
typedef struct kTypeVersionInfo
{
    kText16 format;                             ///< Serialization format name (e.g. "kdat6").
    kVersion formatVersion;                     ///< Serialization format version (e.g. "6.0.0.0").
    kText64 guid;                               ///< Unique id (e.g. "kArrayList-0").
    kFunction serialize;                        ///< Serialization method.
    kFunction deserialize;                      ///< Deserialization method.
} kTypeVersionInfo;

/**
 * @class   kType
 * @extends kObject
 * @ingroup kApi
 * @brief   Represents metadata about a type (class, interface, or value). 
 * 
 * When an assembly is constructed, one kType instance is created for every type defined in the assembly. 
 * kAssembly methods can be used to discover the types in the assembly, and kType methods can be used to 
 * learn about individual types (e.g., type name, base class). 
 * 
 * The kObject_Type method can be used to obtain type information for any object derived from kObject. 
 * The kTypeOf macro can be used to obtain type information for any class, interface, or value by 
 * compile-time type symbol. 
 * 
 * @code {.c}
 * 
 * void DescribeObjectType(kObject object)
 * {
 *     kType type = kObject_Type(object);
 *     kType base = kType_Base(type);
 *
 *     printf("Type name: %s\n", kType_Name(type));
 *     printf("Base type: %s\n", (base == 0) ? "(None)" : kType_Name(base));
 *     printf("Type assembly: %s\n", kAssembly_Name(kType_Assembly(type))); 
 *     printf("Virtual method count: %u\n", (k32u) kType_VMethodCount(type));
 *     printf("Extends kStream?: %s\n", kType_Extends(type, kTypeOf(kStream)) ? "Yes" : "No"); 
 *     printf("Implements kCollection?: %s\n", kType_Implements(type, kTypeOf(kCollection)) ? "Yes" : "No"); 
 *
 * }
 * 
 * @endcode
 * 
 */
//typedef kObject kType;   --forward-declared in kApiDef.x.h

/** 
 * Gets the assembly to which the type belongs. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              Assembly. 
 */
kFx(kAssembly) kType_Assembly(kType type); 

/** 
 * Gets the name of the type. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              Type name. 
 */
kFx(const kChar*) kType_Name(kType type); 

/** 
 * Determines whether a type is equivalent to another type. 
 *
 * Checks type equality, inheritance, and interfaces. 
 *
 * @public              @memberof kType
 * @param   type        Type to be compared. 
 * @param   other       Type to which type is compared. 
 * @return              kTRUE if type is equivalent. 
 */
kFx(kBool) kType_Is(kType type, kType other);

/** 
 * Determines whether a type represents a value (primitive, struct, enum). 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents a value.  
 */
kFx(kBool) kType_IsValue(kType type);

/** 
 * Determines whether a type represents a class. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents a class.  
 */
kFx(kBool) kType_IsClass(kType type);

/** 
 * Determines whether a type represents an interface.  
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents an interface. 
 */
kFx(kBool) kType_IsInterface(kType type); 

/** 
 * Determines whether a type represents a class or interface. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents a reference. 
 */
kFx(kBool) kType_IsReference(kType type);

/** 
 * Determines whether a type represents an abstract class. 
 *
 * @public              @memberof kType
 * @param   type        Class type. 
 * @return              kTRUE if type is abstract. 
 */
kFx(kBool) kType_IsAbstract(kType type);

/** 
 * Reports whether the type is an enumeration. 
 *
 * @public          @memberof kType
 * @return          kTRUE if the type is an enumeration; otherwise, kFALSE. 
 */
kFx(kBool) kType_IsEnum(kType type);

/** 
 * Reports whether the type is an enumeration bitset. 
 *
 * @public          @memberof kType
 * @return          kTRUE if the type is an enumeration bitset; otherwise, kFALSE. 
 */
kFx(kBool) kType_IsBitEnum(kType type);

/** 
 * Reports whether the type is an 'array-value' type (e.g., kText32)
 *
 * @public          @memberof kType
 * @return          kTRUE if the type is an 'array-value' type; otherwise, kFALSE. 
 */
kFx(kBool) kType_IsArrayValue(kType type);

/** 
 * Determines whether a type implements a specific interface.  
 *
 * @public                  @memberof kType
 * @param   type            Type. 
 * @param   interfaceType   Interface type. 
 * @return                  kTRUE if type implements interface. 
 */
kFx(kBool) kType_Implements(kType type, kType interfaceType);

/** 
 * Determines whether a type extends another type. 
 *
 * @public                  @memberof kType
 * @param   type            Type. 
 * @param   baseType        Base type. 
 * @return                  kTRUE if type extends base type.
 */
kFx(kBool) kType_Extends(kType type, kType baseType);

/** 
 * Gets the base of a class or interface. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Base type (or kNULL if none). 
 */
kFx(kType) kType_Base(kType type); 

/** 
 * Reports count of implemented interfaces. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Count of implemented interface. 
 */
kFx(kSize) kType_InterfaceCount(kType type);

/** 
 * Gets the implemented interface at the specified index. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   index   Interface index.
 * @return          Interface type.
 */
kFx(kType) kType_InterfaceAt(kType type, kSize index);

/** 
 * Gets the external size of a type. 
 *
 * The external size of a value type reflects the size of the struct, value or enum. The external size of a reference 
 * type is equal to the size of a pointer. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Type size. 
 */
kFx(kSize) kType_Size(kType type);

/** 
 * Gets the internal size of a type. 
 *
 * The internal size of a value type reflects the size of the struct, value or enum. The internal size of a reference 
 * type is equal to the size of its class structure. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Type size. 
 */
kFx(kSize) kType_InnerSize(kType type);

/** 
 * Gets the size of a type's static data. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Static data size. 
 */
kFx(kSize) kType_StaticSize(kType type);

/** 
 * Gets a pointer to the type's primary virtual method table. 
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Virtual table pointer.
 */
kFx(kFunction*) kType_VTable(kType type);

/** 
 * Gets a pointer to the type's virtual method table corresponding to the specified interface type. 
 *
 * @public                   @memberof kType
 * @param   type             Type.
 * @param   interfaceType    Interface type.
 * @return  Interface virtual table pointer.
 */
kFx(kFunction*) kType_IVTable(kType type, kType interfaceType);

/** 
 * Gets a pointer to the type's static data structure. 
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Pointer to static data. 
 */
kFx(void*) kType_Static(kType type);

/** 
 * Reports whether the type's static data structure has been successfully initialized. 
 * 
 * This function can be helpful during startup, to determine whether the static data for a particular type 
 * has been initialized. This function is not thread-safe.
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          kTRUE if static data has been initialized; otherwise, kFALSE. 
 */
kFx(kBool) kType_StaticInitialized(kType type); 

/** 
 * Reports count of non-virtual methods. 
 * 
 * Most types do not register non-virtual methods. However, non-virtual methods can optionally be registered 
 * using the kAddMethod macro. This can be useful in specific scenarios requiring non-virtual method reflection.  
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Count of non-virtual methods. 
 */
kFx(kSize) kType_MethodCount(kType type);

/** 
 * Gets metadata for the non-virtual method at the specified index.
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   index   Method index.
 * @return          Method metadata.
 */
kFx(const kMethodInfo*) kType_MethodInfoAt(kType type, kSize index);

/** 
 * Finds metadata for the non-virtual method with the specified name.
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   name    Method name. 
 * @param   info    Receives pointer to metadata. 
 * @return          Operation status (kERROR_NOT_FOUND if method info not located).
 */
kFx(kStatus) kType_FindMethodInfo(kType type, const kChar* name, const kMethodInfo** info);

/** 
 * Reports count of virtual methods. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Count of virtual methods. 
 */
kFx(kSize) kType_VMethodCount(kType type);

/** 
 * Gets metadata for the virtual method at the specified index.
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   index   Method index.
 * @return          Method metadata.
 */
kFx(const kMethodInfo*) kType_VMethodInfoAt(kType type, kSize index);

/** 
 * Reports count of interface methods for the given interface.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   interfaceType   Interface type.
 * @return                  Count of interface methods. 
 */
kFx(kSize) kType_IMethodCount(kType type, kType interfaceType);

/** 
 * Gets metadata for the interface method at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   interfaceType   Interface type.
 * @param   index           Method index.
 * @return                  Method metadata.
 */
kFx(const kMethodInfo*) kType_IMethodInfoAt(kType type, kType interfaceType, kSize index);

/** 
 * Reports count of registered fields for the given type.
 * 
 * Value fields can optionally be registered using the kAddField macro. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Count of fields. 
 */
kFx(kSize) kType_FieldCount(kType type);

/** 
 * Gets metadata for the field at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   index           Field index.
 * @return                  Field metadata.
 */
kFx(const kFieldInfo*) kType_FieldInfoAt(kType type, kSize index);

/** 
 * Reports count of registered enumerators for the given enumeration type.
 * 
 * Enumerators can optionally be registered using the kAddEnumerator macro. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Count of enumerators.
 */
kFx(kSize) kType_EnumeratorCount(kType type);

/** 
 * Gets metadata for the enumerator at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   index           Enumerator index.
 * @return                  Enumerator metadata.
 */
kFx(const kEnumeratorInfo*) kType_EnumeratorInfoAt(kType type, kSize index);

/** 
 * Finds enumerator metadata for the enumerator with the specified value. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   value   Enumerator value. 
 * @param   info    Receives pointer to enumerator metadata.
 * @return          Operation status (kERROR_NOT_FOUND if metadata not located). 
 */
kFx(kStatus) kType_FindEnumeratorInfo(kType type, k32s value, const kEnumeratorInfo** info);

/** 
 * Formats an enumerator value to a text buffer using the enumerator display name.
 *
 * If the enumerator value isn't known, the buffer will receive [Unknown] and the return 
 * value will be kERROR_NOT_FOUND.
 * 
 * Exercise caution when using formatted enumerator names in file formats or communication protocols; 
 * enumerator text names may change if the enumerator source code is modified. Consider using explicit and 
 * stable format/parse functions, or using numeric values instead.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   value       Enumerator value. 
 * @param   displayName Receives formatted enumerator name. 
 * @param   capacity    Buffer capacity. 
 * @return              Operation status (see notes). 
 */
kFx(kStatus) kType_FormatEnumerator(kType type, k32s value, kChar* displayName, kSize capacity);

/** 
 * Parses an enumerator value from a text buffer using the enumerator display name.
 *
 * If a matching enumerator display name cannot be found, the value field will receive 0 and the return 
 * value will be kERROR_NOT_FOUND. 
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   value       Receives parsed enumerator value. 
 * @param   displayName Enumerator display name. 
 * @return              Operation status (see notes). 
 */
kFx(kStatus) kType_ParseEnumerator(kType type, k32s* value, const kChar* displayName);

/**
 * Reports count of registered serialization versions.
 *
 * Serialization versions can optionally be registered using the kAddVersion macro.
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Count of serialization versions.
 */
kFx(kSize) kType_VersionCount(kType type);

/** 
 * Gets metadata for the serialization version at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   index           Serialization version index.
 * @return                  Serialization version metadata.
 */
kFx(const kTypeVersionInfo*) kType_VersionInfoAt(kType type, kSize index);

/** @relates  kType @{ */
#define kType_Assembly_(TYPE)           kxType_Assembly_(TYPE)                  ///< Macro version of kType_Assembly. 
#define kType_Is_(TYPE, OTHER)          kxType_Is_(TYPE, OTHER)                 ///< Macro version of kType_Is. 
#define kType_IsValue_(TYPE)            kxType_IsValue_(TYPE)                   ///< Macro version of kType_IsValue. 
#define kType_IsClass_(TYPE)            kxType_IsClass_(TYPE)                   ///< Macro version of kType_IsClass. 
#define kType_IsInterface_(TYPE)        kxType_IsInterface_(TYPE)               ///< Macro version of kType_IsInterface. 
#define kType_IsAbstract_(TYPE)         kxType_IsAbstract_(TYPE)                ///< Macro version of kType_IsAbstract. 
#define kType_IsReference_(TYPE)        kxType_IsReference_(TYPE)               ///< Macro version of kType_IsReference.
#define kType_IsEnum_(TYPE)             kxType_IsEnum_(TYPE)                    ///< Macro version of kType_IsEnum. 
#define kType_IsBitEnum_(TYPE)          kxType_IsBitEnum_(TYPE)                 ///< Macro version of kType_IsBitEnum. 
#define kType_IsArrayValue_(TYPE)       kxType_IsArrayValue_(TYPE)              ///< Macro version of kType_IsArrayValue. 
#define kType_Implements_(TYPE, IFACE)  kxType_Implements_(TYPE, IFACE)         ///< Macro version of kType_Implements. 
#define kType_Extends_(TYPE, BASE)      kxType_Extends_(TYPE, BASE)             ///< Macro version of kType_Extends. 
#define kType_Base_(TYPE)               kxType_Base_(TYPE)                      ///< Macro version of kType_Base. 
#define kType_Size_(TYPE)               kxType_Size_(TYPE)                      ///< Macro version of kType_Size. 
#define kType_InnerSize_(TYPE)          kxType_InnerSize_(TYPE)                 ///< Macro version of kType_InnerSize. 
#define kType_VTable_(TYPE)             kxType_VTable_(TYPE)                    ///< Macro version of kType_VTable. 
#define kType_Static_(TYPE)             kxType_Static_(TYPE)                    ///< Macro version of kType_Static. 
/** @} */

kEndHeader()

#include <kApi/kType.x.h>

#endif
