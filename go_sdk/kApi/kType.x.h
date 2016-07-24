/** 
 * @file  kType.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TYPE_X_H
#define K_API_TYPE_X_H

kBeginHeader()

#define kTYPE_MAX_BASES                     (4)
#define kTYPE_MAX_INTERFACES                (4)

#define kTYPE_GROWTH_FACTOR                 (2)
#define kTYPE_INITIAL_CMETHOD_CAPACITY      (16)
#define kTYPE_INITIAL_FIELD_CAPACITY        (16)
#define kTYPE_INITIAL_ENUMERATOR_CAPACITY   (16)
#define kTYPE_INITIAL_VERSION_CAPACITY      (16)

typedef kStatus (kCall* kStaticInitFx)(); 
typedef kStatus (kCall* kStaticReleaseFx)(); 

//used by individual container types, not the type system itself. 
typedef struct kStructField
{
    kType type;             //field type
    kSize offset;           //field offset, in bytes
    kSize count;            //field element count, in items
    kSize typeSize;         //field element size, in bytes
    kSize fieldSize;        //field size, in bytes
} kStructField;

typedef struct kInterfaceInfo
{
    kType type; 
    kFunction* iTable; 
    kMethodInfo* iMethodInfo; 
    kSize iMethodCount; 
} kInterfaceInfo; 

typedef struct kTypeClass
{
    kObjectClass base; 
    kAssembly assembly; 
    kTypeName name; 
    kType* selfReference; 
    kTypeFlags flags; 
    kSize size;
    kSize innerSize;
    kType bases[kTYPE_MAX_BASES]; 
    kSize baseCount;
    kFunction* vTable;
    kMethodInfo* vMethodInfo; 
    kSize vMethodCount;
    kMethodInfo* cMethodInfo; 
    kSize cMethodCount;
    kSize cMethodCapacity;
    kFieldInfo* fieldInfo; 
    kSize fieldCount; 
    kSize fieldCapacity; 
    kEnumeratorInfo* enumeratorInfo; 
    kSize enumeratorCount; 
    kSize enumeratorCapacity; 
    kPointer staticData; 
    kSize staticSize; 
    kStaticInitFx staticInit; 
    kStaticReleaseFx staticRelease; 
    volatile kBool* staticInitialized; 
    k32s staticPriority; 
    kInterfaceInfo interfaces[kTYPE_MAX_INTERFACES]; 
    kSize interfaceCount;
    kTypeVersionInfo* versionInfo;
    kSize versionCount; 
    kSize versionCapacity; 
} kTypeClass;

kDeclareClass(k, kType, kObject)

kFx(kStatus) kType_Construct(kType* type, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize); 
kFx(kStatus) kType_Init(kType type, kType typeType, kAlloc allocator, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize); 
kFx(kStatus) kType_VRelease(kType type);

kFx(kStatus) kType_SetBase(kType type, kType base);
kFx(kStatus) kType_InitMethods(kType type);
kFx(kStatus) kType_InitVTable(kType type, kSize vTableSize);
kFx(kStatus) kType_InitInterfaces(kType type);
kFx(kStatus) kType_SetStatic(kType type, void* staticReference, kSize staticSize, kStaticInitFx init, kStaticReleaseFx release, volatile kBool* staticInitialized); 

kFx(kStatus) kType_ImplementInterface(kType type, kType interfaceType, const kChar* interfaceName, kSize iTableSize); 
kFx(kStatus) kType_AddVersion(kType type, const kChar* format, const kChar* formatVersion, const kChar* guid, kFunction serialize, kFunction deserialize); 
kFx(kStatus) kType_AddFlags(kType type, kTypeFlags flags); 
kFx(kStatus) kType_AddMethod(kType type, kFunction function, const kChar* methodName); 
kFx(kStatus) kType_AddVMethod(kType type, kSize index, kFunction function, const kChar* methodName); 
kFx(kStatus) kType_AddIVMethod(kType type, kType interfaceType, kSize index, kFunction function, const kChar* iMethodName, const kChar* cMethodName); 
kFx(kStatus) kType_AddField(kType type, kType fieldType, const kChar* fieldTypeName, kSize size, kSize offset, kSize count, const kChar* fieldName); 
kFx(kStatus) kType_AddEnumerator(kType type, k32s value, const kChar* name); 
kFx(kStatus) kType_FormatEnumeratorDisplayName(kType type, const kChar* enumeratorName, kChar* buffer, kSize capacity); 
kFx(kStatus) kType_SetInitPriority(kType type, k32u priority);

kFx(kStatus) kType_ZeroStaticData(kType type);
kFx(kStatus) kType_RunStaticInit(kType type);
kFx(kStatus) kType_RunStaticRelease(kType type); 

kFx(kTypeVersion) kType_VersionAt(kType type, kSize index);

kFx(kStatus) kType_FindEnumeratorInfoByName(kType type, const kChar* displayName, const kEnumeratorInfo** info); 

kFx(const kChar*) kType_VersionFormat(kType type, kTypeVersion version); 
kFx(kVersion) kType_VersionFormatVersion(kType type, kTypeVersion version); 
kFx(const kChar*) kType_VersionGuid(kType type, kTypeVersion version); 
kFx(kFunction) kType_VersionSerializeFx(kType type, kTypeVersion version); 
kFx(kFunction) kType_VersionDeserializeFx(kType type, kTypeVersion version); 

kFx(k32u) kType_Priority(kType type); 

kFx(kStatus) kType_LayoutStruct(kStructField** fields, kSize fieldCount, kSize* structureSize); 
k32s kType_LayoutStructOffsetComparator(const void* field1, const void* field2); 
k32s kType_LayoutStructSizeComparator(const void* field1, const void* field2);
kFx(kSize) kType_MaxPrimitiveSize(kType type); 

#define kType_Overrides_(TYPE, CLASS, METHOD)                           \
    (kType_Extends_(TYPE, kTypeOf(CLASS)) &&                            \
     (((CLASS##VTable*)kType_VTable_(TYPE))->METHOD !=                  \
      ((CLASS##VTable*)kType_VTable_(kType_Base_(TYPE)))->METHOD))

#define kType_(TYPE)                            kCast(kTypeClass*, TYPE)
#define kType_Class_(TYPE)                      ((!kIsNull(TYPE) && kObject_VerifyTag_(TYPE)) ? kType_(TYPE) : (abort(), (void*)kNULL))
#define kType_Exists_(TYPE)                     (!kIsNull(TYPE))

#define kxType_Assembly_(TYPE)                  (kType_(TYPE)->assembly)
#define kxType_IsValue_(TYPE)                   ((kType_(TYPE)->flags & kTYPE_FLAGS_VALUE) != 0)
#define kxType_IsClass_(TYPE)                   ((kType_(TYPE)->flags & kTYPE_FLAGS_CLASS) != 0)
#define kxType_IsInterface_(TYPE)               ((kType_(TYPE)->flags & kTYPE_FLAGS_INTERFACE) != 0)
#define kxType_IsAbstract_(TYPE)                ((kType_(TYPE)->flags & kTYPE_FLAGS_ABSTRACT) != 0)
#define kxType_IsReference_(TYPE)               ((kType_(TYPE)->flags & (kTYPE_FLAGS_INTERFACE | kTYPE_FLAGS_CLASS)) != 0)
#define kxType_IsEnum_(TYPE)                    ((kType_(TYPE)->flags & kTYPE_FLAGS_ENUM) != 0)
#define kxType_IsBitEnum_(TYPE)                 ((kType_(TYPE)->flags & kTYPE_FLAGS_BIT_ENUM) != 0)
#define kxType_IsArrayValue_(TYPE)              ((kType_(TYPE)->flags & kTYPE_FLAGS_ARRAY_VALUE) != 0)
#define kxType_Implements_(TYPE, IFACE)         (kType_IVTable_(TYPE, IFACE) != kNULL)
#define kxType_Is_(TYPE, OTHER)                 (((TYPE) == (OTHER)) || kType_Extends_(TYPE, OTHER) || kType_Implements_(TYPE, OTHER))
#define kxType_Base_(TYPE)                      (kType_(TYPE)->bases[0])
#define kxType_Size_(TYPE)                      (kType_(TYPE)->size)
#define kxType_InnerSize_(TYPE)                 (kType_(TYPE)->innerSize)
#define kxType_VTable_(TYPE)                    ((void*)kType_(TYPE)->vTable)
#define kxType_Static_(TYPE)                    (kType_(TYPE)->staticData)

#define kType_VersionGuid_(TYPE, VER)           (kCast(kTypeVersionInfo*, VER)->guid)
#define kType_VersionId_(TYPE, VER)             (kCast(kTypeVersionInfo*, VER)->versionId)
#define kType_SerializeFx_(TYPE, VER, SYMBOL)   (kCast(SYMBOL##SerializeFx, kCast(kTypeVersionInfo*, VER)->serialize))
#define kType_DeserializeFx_(TYPE, VER, SYMBOL) (kCast(SYMBOL##DeserializeFx, kCast(kTypeVersionInfo*, VER)->deserialize))

#define kxType_Extends_(TYPE, BASE)                    \
    ((kType_(TYPE)->bases[0] == (BASE))   ||           \
     (kType_(TYPE)->bases[1] == (BASE))   ||           \
     (kType_(TYPE)->bases[2] == (BASE))   ||           \
     (kType_(TYPE)->bases[3] == (BASE)))

#define kType_InterfaceInfo_(TYPE, IFACE)                                                       \
    ((kType_(TYPE)->interfaces[0].type == (IFACE)) ? &kType_(TYPE)->interfaces[0] :             \
     (kType_(TYPE)->interfaces[1].type == (IFACE)) ? &kType_(TYPE)->interfaces[1] :             \
     (kType_(TYPE)->interfaces[2].type == (IFACE)) ? &kType_(TYPE)->interfaces[2] :             \
     (kType_(TYPE)->interfaces[3].type == (IFACE)) ? &kType_(TYPE)->interfaces[3] : kNULL)

#define kType_IVTable_(TYPE, IFACE)                                                                 \
    ((kType_(TYPE)->interfaces[0].type == (IFACE)) ? kType_(TYPE)->interfaces[0].iTable :           \
     (kType_(TYPE)->interfaces[1].type == (IFACE)) ? kType_(TYPE)->interfaces[1].iTable :           \
     (kType_(TYPE)->interfaces[2].type == (IFACE)) ? kType_(TYPE)->interfaces[2].iTable :           \
     (kType_(TYPE)->interfaces[3].type == (IFACE)) ? kType_(TYPE)->interfaces[3].iTable : kNULL)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_UNKNOWN           kTypeOf(kVoid)

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kType_Name5(T, B, C)    kStrCopy(B, C, kType_Name(T))

#endif

kEndHeader()

#endif
