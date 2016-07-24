/** 
 * @file    kApiDef.x.h
 *
 * @internal
 * Copyright (C) 2005-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_DEF_X_H
#define K_API_API_DEF_X_H

kBeginHeader()

/* 
 * Core macros
 */

#define kNull_type                                  \
    (0)

#define kCast_(TYPE, ITEM)                          \
    ((TYPE)(ITEM))

#define kStringify_(STR)                            \
    #STR

#define kAt_(BASE, OFFSET)                          \
    ((void*)((kByte*)(BASE) + (OFFSET)))

#define kAs_(ITEM, TYPE)                            \
    (*(TYPE*)(ITEM))

#define kSetAs_(ITEM, VALUE, TYPE)                  \
    (*(TYPE*)(ITEM) = (TYPE)(VALUE))

#define kAssemblyOf_(SYMBOL)                        \
    (SYMBOL##_assembly)

#define kStaticOf_(SYMBOL)                          \
    (&SYMBOL##_static)

#define kStaticInitialized_(SYMBOL)                 \
    (SYMBOL##_staticInitialized)

#define kCountOf_(CARRAY)                           \
    (sizeof(CARRAY)/sizeof(CARRAY[0]))  

#define kxMin_(A, B)                                \
    (((A) < (B)) ? (A) : (B))

#define kxMax_(A, B)                                \
    (((A) > (B)) ? (A) : (B))

#define kxAbs_(A)                                   \
    (((A) >= 0) ? (A) : -(A))

#define kxZero_(VALUE)                              \
    memset(&VALUE, 0, sizeof(VALUE))

#define kxItemCopy_(DEST, SRC, SIZE)                \
    memcpy(DEST, SRC, SIZE)

#define kxItemSet_(DEST, FILL, SIZE)                \
    memset(DEST, FILL, SIZE)

#define kItemCopy1_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[0]))

#define kItemCopy2_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[0]),        \
     (((kByte*)(DEST))[1] = ((kByte*)(SRC))[1]))

#define kItemCopy4_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[0]),        \
     (((kByte*)(DEST))[1] = ((kByte*)(SRC))[1]),        \
     (((kByte*)(DEST))[2] = ((kByte*)(SRC))[2]),        \
     (((kByte*)(DEST))[3] = ((kByte*)(SRC))[3]))

#define kItemCopy8_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[0]),        \
     (((kByte*)(DEST))[1] = ((kByte*)(SRC))[1]),        \
     (((kByte*)(DEST))[2] = ((kByte*)(SRC))[2]),        \
     (((kByte*)(DEST))[3] = ((kByte*)(SRC))[3]),        \
     (((kByte*)(DEST))[4] = ((kByte*)(SRC))[4]),        \
     (((kByte*)(DEST))[5] = ((kByte*)(SRC))[5]),        \
     (((kByte*)(DEST))[6] = ((kByte*)(SRC))[6]),        \
     (((kByte*)(DEST))[7] = ((kByte*)(SRC))[7]))

#define kItemSwap2_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[1]),        \
     (((kByte*)(DEST))[1] = ((kByte*)(SRC))[0]))

#define kItemSwap4_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[3]),        \
     (((kByte*)(DEST))[1] = ((kByte*)(SRC))[2]),        \
     (((kByte*)(DEST))[2] = ((kByte*)(SRC))[1]),        \
     (((kByte*)(DEST))[3] = ((kByte*)(SRC))[0]))

#define kItemSwap8_(DEST, SRC)                          \
    ((((kByte*)(DEST))[0] = ((kByte*)(SRC))[7]),        \
     (((kByte*)(DEST))[1] = ((kByte*)(SRC))[6]),        \
     (((kByte*)(DEST))[2] = ((kByte*)(SRC))[5]),        \
     (((kByte*)(DEST))[3] = ((kByte*)(SRC))[4]),        \
     (((kByte*)(DEST))[4] = ((kByte*)(SRC))[3]),        \
     (((kByte*)(DEST))[5] = ((kByte*)(SRC))[2]),        \
     (((kByte*)(DEST))[6] = ((kByte*)(SRC))[1]),        \
     (((kByte*)(DEST))[7] = ((kByte*)(SRC))[0]))

#define kItemImport_(DEST, SRC, TYPE)         \
    (kType_IsArrayValue_(TYPE) ? (kValue_Import_(TYPE, DEST, SRC)) : (void)(kItemCopy_(DEST, SRC, kType_Size_(TYPE))))

#define kxClamp_(V, VMIN, VMAX)                     \
    (kMin_(kMax_((V), (VMIN)), (VMAX)))

#define kAlign_(VAL, TO)                            \
    ((((VAL) >> (TO)) + !!((VAL) & ((1 << (TO)) - 1))) << (TO))
    
#define kFloorInt_(A, B)                            \
    (((A) >= 0) ? (A) / (B) : ((A) - (B) + 1) / (B))

#define kCeilInt_(A, B)                             \
    (((A) >= 0) ? ((A) + (B) - 1) / (B) : (A) / (B))

#define kFloorUInt_(A, B)                           \
    ((A) / (B))

#define kCeilUInt_(A, B)                            \
    (((A) + (B) - 1) / (B))

#define kZeroDerivedFields_                         \
    kInitFields_

#define kInitFields_(SYMBOL, OBJECT)                                                        \
    (memset(kAt_(OBJECT, sizeof(kCast(SYMBOL##Class*, OBJECT)->base)), 0,                   \
        sizeof(SYMBOL##Class) - sizeof(kCast(SYMBOL##Class*, OBJECT)->base)), kOK)

#define kDeclareAssembly_(PREFIX, SYMBOL)                                                   \
    extern PREFIX##Dx(kAssembly) SYMBOL##_assembly;                                         \
    PREFIX##Fx(kStatus) SYMBOL##_Construct(kAssembly* assembly);                            \
    PREFIX##Fx(kAssembly) SYMBOL##_Instance();

#define kBeginAssembly_(PREFIX, SYMBOL, VERSION)                                            \
    PREFIX##Dx(kAssembly) SYMBOL##_assembly = kNULL;                                        \
    PREFIX##Fx(kStatus) SYMBOL##_Construct(kAssembly* assembly);                            \
    kDefinePlugin(SYMBOL)                                                                   \
    PREFIX##Fx(kStatus) SYMBOL##_Construct(kAssembly* assembly)                             \
    {                                                                                       \
        kAssembly output = kNULL;                                                           \
        kStatus status;                                                                     \
        kVersion version;                                                                   \
        k32s priority = 0;                                                                  \
        kItemZero_(&priority, sizeof(priority));  /* silence Xcode */                       \
        if (!kIsNull(SYMBOL##_assembly))                                                    \
        {                                                                                   \
            kCheck(kObject_Share(SYMBOL##_assembly));                                       \
            *assembly = SYMBOL##_assembly;                                                  \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
            kTry                                                                            \
            {                                                                               \
                kTest(kVersion_Parse(&version, VERSION));                                   \
                kTest(kAssembly_ConstructInternal(&output,                                  \
                    &SYMBOL##_assembly,  #SYMBOL, version)); 

#if defined(K_PLUGIN)

#define kDefinePlugin(SYMBOL)                                                               \
    kExportCx(kStatus) kPlugin_ConstructAssembly(kAssembly* assembly)                       \
    {                                                                                       \
        return SYMBOL##_Construct(assembly);                                                \
    }
#else

#define kDefinePlugin(SYMBOL) 

#endif

#define kEndAssembly_()                                                                     \
                kTest(kAssembly_Finalize(output));                                          \
                *assembly = output;                                                         \
            }                                                                               \
            kCatch(&status)                                                                 \
            {                                                                               \
                kAssembly_VRelease(output);                                                 \
                kEndCatch(status);                                                          \
            }                                                                               \
        }                                                                                   \
        return kOK;                                                                         \
    }

#define kDeclareVirtualValue(PREFIX, SYMBOL, BASE)                                          \
    kDeclareValueType(PREFIX, SYMBOL)

#define kBeginVirtualValue(PREFIX, SYMBOL, BASE)                                            \
    kDefineValueType(PREFIX, SYMBOL)                                                        \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name)            \
    {                                                                                       \
        kCheck(kAssembly_AddValue(assembly, &SYMBOL##_type,                                 \
            #SYMBOL, BASE##_type, #BASE, 0,                                                 \
            sizeof(SYMBOL##VTable), kTYPE_FLAGS_VALUE));

#define kEndVirtualValue()                                                                  \
    kEndValueRegister()

#define kDeclareVoidValue(PREFIX, SYMBOL, BASE)                                             \
    kDeclareValueVTable(PREFIX, SYMBOL, BASE)                                               \
    kDeclareValueType(PREFIX, SYMBOL)

#define kBeginVoidValue(PREFIX, SYMBOL, BASE)                                               \
    kDefineValueType(PREFIX, SYMBOL)                                                        \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name)            \
    {                                                                                       \
        kCheck(kAssembly_AddValue(assembly, &SYMBOL##_type,                                 \
            #SYMBOL, BASE##_type, #BASE, 0,                                                 \
            sizeof(SYMBOL##VTable), kTYPE_FLAGS_VALUE));

#define kEndVoidValue()                                                                     \
    kEndValueRegister()

#define kDeclareValue_(PREFIX, SYMBOL, BASE)                                                \
    kDeclareValueVTable(PREFIX, SYMBOL, BASE)                                               \
    kDeclareValueType(PREFIX, SYMBOL)

#define kBeginValue_(PREFIX, SYMBOL, BASE)                                                  \
    kDefineValueType(PREFIX, SYMBOL)                                                        \
    kBeginValueRegister(PREFIX, SYMBOL, BASE, kTYPE_FLAGS_VALUE)

#define kEndValue_()                                                                        \
    kEndValueRegister()

#define kDeclareEnum_(PREFIX, SYMBOL, BASE)                                                 \
    kDeclareValueVTable(PREFIX, SYMBOL, BASE)                                               \
    kDeclareValueType(PREFIX, SYMBOL)

#define kBeginEnum_(PREFIX, SYMBOL, BASE)                                                   \
    kDefineValueType(PREFIX, SYMBOL)                                                        \
    kBeginValueRegister(PREFIX, SYMBOL, BASE,                                               \
        kTYPE_FLAGS_VALUE | kTYPE_FLAGS_ENUM)                                               \
    kCheck(kType_AddField(SYMBOL##_type, k32s##_type, "k32s", sizeof(k32s),                 \
        0, 1, "value")); 

#define kEndEnum_()                                                                         \
    kEndValueRegister()

#define kDeclareBitEnum_(PREFIX, SYMBOL, BASE)                                              \
    kDeclareValueVTable(PREFIX, SYMBOL, BASE)                                               \
    kDeclareValueType(PREFIX, SYMBOL)

#define kBeginBitEnum_(PREFIX, SYMBOL, BASE)                                                \
    kDefineValueType(PREFIX, SYMBOL)                                                        \
    kBeginValueRegister(PREFIX, SYMBOL, BASE,                                               \
        kTYPE_FLAGS_VALUE | kTYPE_FLAGS_ENUM | kTYPE_FLAGS_BIT_ENUM)                        \
    kCheck(kType_AddField(SYMBOL##_type, k32s##_type, "k32s", sizeof(k32s),                 \
        0, 1, "value")); 

#define kEndBitEnum_()                                                                      \
    kEndValueRegister()

#define kDeclareArrayValue_(PREFIX, SYMBOL, BASE)                                           \
    kDeclareValueVTable(PREFIX, SYMBOL, BASE)                                               \
    kDeclareValueType(PREFIX, SYMBOL)

#define kBeginArrayValue_(PREFIX, SYMBOL, TYPE, BASE)                                       \
    kDefineValueType(PREFIX, SYMBOL)                                                        \
    kBeginValueRegister(PREFIX, SYMBOL, BASE, kTYPE_FLAGS_VALUE | kTYPE_FLAGS_ARRAY_VALUE)  \
    kCheck(kType_AddField(SYMBOL##_type, TYPE##_type, #TYPE,                                \
        sizeof(SYMBOL), 0, sizeof(SYMBOL)/sizeof(TYPE), "elements")); 

#define kEndArrayValue_()                                                                   \
    kEndValueRegister()

#define kDeclareValueVTable(PREFIX, SYMBOL, BASE)                                           \
    typedef struct SYMBOL##VTable { BASE##VTable base; } SYMBOL##VTable;

#define kDeclareValueType(PREFIX, SYMBOL)                                                   \
    extern PREFIX##Dx(kType) SYMBOL##_type;                                                 \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name);
    
#define kDefineValueType(PREFIX, SYMBOL)                                                    \
    PREFIX##Dx(kType) SYMBOL##_type = kNULL;

#define kBeginValueRegister(PREFIX, SYMBOL, BASE, FLAGS)                                    \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name)            \
    {                                                                                       \
        kCheck(kAssembly_AddValue(assembly, &SYMBOL##_type,                                 \
            #SYMBOL, BASE##_type, #BASE, sizeof(SYMBOL),                                    \
            sizeof(SYMBOL##VTable), FLAGS));

#define kEndValueRegister()                                                                 \
        return kOK;                                                                         \
    }

#define kDeclareInterface_(PREFIX, SYMBOL, BASE)                                            \
    extern PREFIX##Dx(kType) SYMBOL##_type;                                                 \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name);

#define kBeginInterface_(PREFIX, SYMBOL, BASE)                                              \
    kDefineInterfaceType(PREFIX, SYMBOL)                                                    \
    kBeginInterfaceRegister(PREFIX, SYMBOL, BASE)

#define kEndInterface_()                                                                    \
    kEndInterfaceRegister()

#define kDefineInterfaceType(PREFIX, SYMBOL)                                                \
    PREFIX##Dx(kType) SYMBOL##_type = kNULL;

#define kBeginInterfaceRegister(PREFIX, SYMBOL, BASE)                                       \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name)            \
    {                                                                                       \
        kCheck(kAssembly_AddInterface(assembly, &SYMBOL##_type,                             \
            #SYMBOL, BASE##_type, #BASE, sizeof(SYMBOL##VTable))); 

#define kEndInterfaceRegister()                                                             \
        return kOK;                                                                         \
    }

#define kDeclareFullClass_(PREFIX, SYMBOL, BASE)                                            \
    kDeclareClassType(PREFIX, SYMBOL) 

#define kBeginFullClass_(PREFIX, SYMBOL, BASE)                                              \
    kDefineClassType(PREFIX, SYMBOL)                                                        \
    kBeginClassRegister(PREFIX, SYMBOL, BASE)

#define kEndFullClass_()                                                                    \
    kEndClassRegister()

#define kDeclareVirtualClass_(PREFIX, SYMBOL, BASE)                                         \
    kDeclareClassStatic(PREFIX, SYMBOL)                                                     \
    kDeclareClassType(PREFIX, SYMBOL)

#define kBeginVirtualClass_(PREFIX, SYMBOL, BASE)                                           \
    kDefineClassType(PREFIX, SYMBOL)                                                        \
    kDefineClassStatic(PREFIX, SYMBOL)                                                      \
    kBeginClassRegister(PREFIX, SYMBOL, BASE)

#define kEndVirtualClass_()                                                                 \
    kEndClassRegister()

#define kDeclareStaticClass_(PREFIX, SYMBOL)                                                \
    kDeclareClassInstance(PREFIX, SYMBOL, kObject)                                          \
    kDeclareClassVTable(PREFIX, SYMBOL, kObject)                                            \
    kDeclareClassType(PREFIX, SYMBOL) 

#define kBeginStaticClass_(PREFIX, SYMBOL)                                                  \
    kDefineClassType(PREFIX, SYMBOL)                                                        \
    kBeginClassRegister(PREFIX, SYMBOL, kObject)

#define kEndStaticClass_()                                                                  \
    kEndClassRegister()

#define kDeclareClass_(PREFIX, SYMBOL, BASE)                                                \
    kDeclareClassVTable(PREFIX, SYMBOL, BASE)                                               \
    kDeclareClassStatic(PREFIX, SYMBOL)                                                     \
    kDeclareClassType(PREFIX, SYMBOL) 

#define kBeginClass_(PREFIX, SYMBOL, BASE)                                                  \
    kDefineClassType(PREFIX, SYMBOL)                                                        \
    kDefineClassStatic(PREFIX, SYMBOL)                                                      \
    kBeginClassRegister(PREFIX, SYMBOL, BASE)

#define kEndClass_()                                                                        \
    kEndClassRegister()

#define kDeclareClassType(PREFIX, SYMBOL)                                                   \
    extern PREFIX##Dx(kType) SYMBOL##_type;                                                 \
    extern PREFIX##Dx(SYMBOL##Static) SYMBOL##_static;                                      \
    extern PREFIX##Dx(volatile kBool) SYMBOL##_staticInitialized;                           \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name);

#define kDeclareClassInstance(PREFIX, SYMBOL, BASE)                                         \
    typedef struct SYMBOL##Class { BASE##Class base; } SYMBOL##Class;

#define kDeclareClassVTable(PREFIX, SYMBOL, BASE)                                           \
    typedef struct SYMBOL##VTable { BASE##VTable base; } SYMBOL##VTable;

#define kDeclareClassStatic(PREFIX, SYMBOL)                                                 \
    typedef struct SYMBOL##Static { kByte placeholder; } SYMBOL##Static;                    \
    PREFIX##Fx(kStatus) SYMBOL##_InitStatic();                                              \
    PREFIX##Fx(kStatus) SYMBOL##_ReleaseStatic();

#define kDefineClassType(PREFIX, SYMBOL)                                                    \
    PREFIX##Dx(kType) SYMBOL##_type = kNULL;                                                \
    PREFIX##Dx(SYMBOL##Static) SYMBOL##_static;                                             \
    PREFIX##Dx(volatile kBool) SYMBOL##_staticInitialized = kFALSE;

#define kDefineClassStatic(PREFIX, SYMBOL)                                                  \
    PREFIX##Fx(kStatus) SYMBOL##_InitStatic() { return kOK; }                               \
    PREFIX##Fx(kStatus) SYMBOL##_ReleaseStatic() { return kOK; }

#define kBeginClassRegister(PREFIX, SYMBOL, BASE)                                           \
    PREFIX##Fx(kStatus) SYMBOL##_Register(kAssembly assembly, const kChar* name)            \
    {                                                                                       \
        kCheck(kAssembly_AddClass(assembly, &SYMBOL##_type,                                 \
            #SYMBOL, BASE##_type, #BASE, sizeof(SYMBOL##Class),                             \
            sizeof(SYMBOL##VTable), &SYMBOL##_static, sizeof(SYMBOL##Static),               \
            SYMBOL##_InitStatic, SYMBOL##_ReleaseStatic, &SYMBOL##_staticInitialized)); 

#define kEndClassRegister()                                                                 \
        return kOK;                                                                         \
    }

#define kAddDependency_(SYMBOL)                                                             \
    kTest(kAssembly_AddDependency(output, SYMBOL##_Construct)); 

#define kAddType_(SYMBOL)                                                                   \
    kTest(kAssembly_AddType(output, SYMBOL##_Register, #SYMBOL)); 

#define kAddPriority_(SYMBOL)                                                               \
    kTest(kAssembly_AddPriority(output, SYMBOL##_Register, priority++)); 

#define kAddInterface_(SYMBOL, IFACE)                                                       \
    kCheck(kType_ImplementInterface(SYMBOL##_type, IFACE##_type,                            \
        #IFACE, sizeof(IFACE##VTable))); 

#define kAddMethod_(CLASS, METHOD)                                                          \
    kCheck(kType_AddMethod(CLASS##_type,                                                    \
        (kFunction)CLASS##_##METHOD, #METHOD));

#define kAddVMethod_(IN_CLASS, FROM_CLASS, METHOD)                                          \
    if (0) (((FROM_CLASS##VTable*)0)->METHOD) = IN_CLASS##_##METHOD;  /* type check */      \
    kCheck(kType_AddVMethod(IN_CLASS##_type,                                                \
        offsetof(FROM_CLASS##VTable, METHOD)/sizeof(kPointer),                              \
        (kFunction)IN_CLASS##_##METHOD, #METHOD));

#define kAddIVMethod_(IN_CLASS, FROM_IFACE, IMETHOD, CMETHOD)                               \
    if (0) (((FROM_IFACE##VTable*)0)->IMETHOD) = IN_CLASS##_##CMETHOD;  /* type check */    \
    kCheck(kType_AddIVMethod(IN_CLASS##_type, FROM_IFACE##_type,                            \
        offsetof(FROM_IFACE##VTable, IMETHOD)/sizeof(kPointer),                             \
        (kFunction) IN_CLASS##_##CMETHOD, #IMETHOD, #CMETHOD)); 

#define kAddField_(VALUE, FIELD_TYPE, FIELD)                                                \
    kCheck(kType_AddField(VALUE##_type, FIELD_TYPE##_type, #FIELD_TYPE,                     \
        sizeof(((VALUE*)0)->FIELD), offsetof(VALUE, FIELD),                                 \
        sizeof(((VALUE*)0)->FIELD)/sizeof(FIELD_TYPE),                                      \
        #FIELD)); 

#define kAddEnumerator_(SYMBOL, ENUMERATOR)                                                 \
    kCheck(kType_AddEnumerator(SYMBOL##_type, ENUMERATOR, #ENUMERATOR)); 

#define kAddVersion_(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)             \
    kCheck(kType_AddVersion(TYPE##_type, FORMAT, FORMAT_VER, GUID,                          \
    (kFunction)TYPE##_##WRITE_METHOD, (kFunction)TYPE##_##READ_METHOD));

#define kAddFlags_(TYPE, FLAGS)                                                             \
    kCheck(kType_AddFlags(TYPE##_type, FLAGS)); 

#define kAddClassDebugHint(TYPE)                                \
    static TYPE##Class* TYPE##_debugHint = kNULL;

#define kAddValueDebugHint(TYPE)                                \
    static TYPE* TYPE##_debugHint = kNULL;

#if defined(K_DEBUG) && defined(K_MSVC)

//types that require K_PLATFORM (e.g., kThread) should be excluded from this list
#define kDefineDebugHints_()                                    \
    kAddClassDebugHint(kAlloc)                                  \
    kAddClassDebugHint(kAssembly)                               \
    kAddClassDebugHint(kObject)                                 \
    kAddClassDebugHint(kType)                                   \
                                                                \
    kAddClassDebugHint(kArray1)                                 \
    kAddClassDebugHint(kArray2)                                 \
    kAddClassDebugHint(kArray3)                                 \
    kAddClassDebugHint(kArrayList)                              \
    kAddClassDebugHint(kBox)                                    \
    kAddClassDebugHint(kImage)                                  \
    kAddClassDebugHint(kList)                                   \
    kAddClassDebugHint(kMap)                                    \
    kAddClassDebugHint(kQueue)                                  \
    kAddClassDebugHint(kString)                                 \
    kAddClassDebugHint(kXml)                                    \
                                                                \
    kAddClassDebugHint(kDat5Serializer)                         \
    kAddClassDebugHint(kDat6Serializer)                         \
    kAddClassDebugHint(kHttpServer)                             \
    kAddClassDebugHint(kHttpServerChannel)                      \
    kAddClassDebugHint(kHttpServerRequest)                      \
    kAddClassDebugHint(kHttpServerResponse)                     \
    kAddClassDebugHint(kMemory)                                 \
    kAddClassDebugHint(kSerializer)                             \
    kAddClassDebugHint(kStream)                                 \
    kAddClassDebugHint(kTcpClient)                              \
    kAddClassDebugHint(kTcpServer)                              \
    kAddClassDebugHint(kUdpClient)                              \
                                                                \
    kAddClassDebugHint(kMsgQueue)                               \
    kAddClassDebugHint(kPeriodic)                               \
    kAddClassDebugHint(kTimer)                                  \
                                                                \
    kAddClassDebugHint(kBackTrace)                              \
    kAddClassDebugHint(kDebugAlloc)                             \
    kAddClassDebugHint(kDynamicLib)                             \
    kAddClassDebugHint(kEvent)                                  \
    kAddClassDebugHint(kPoolAlloc)                              \
    kAddClassDebugHint(kUserAlloc)                              \
                                                                \
    kAddValueDebugHint(kEnumeratorInfo)                         \
    kAddValueDebugHint(kFieldInfo)                              \
    kAddValueDebugHint(kInterfaceInfo)                          \
    kAddValueDebugHint(kMethodInfo)                             \
    kAddValueDebugHint(kStructField)                            \
    kAddValueDebugHint(kTypeVersionInfo)                        \
                                                                \
    kAddValueDebugHint(kListItemStruct)                         \
    kAddValueDebugHint(kListItemBlock)                          \
                                                                \
    kAddValueDebugHint(kArgb)                                   \
    kAddValueDebugHint(kPoint16s)                               \
    kAddValueDebugHint(kPoint32s)                               \
    kAddValueDebugHint(kPoint32f)                               \
    kAddValueDebugHint(kPoint64f)                               \
    kAddValueDebugHint(kPoint3d16s)                             \
    kAddValueDebugHint(kPoint3d32s)                             \
    kAddValueDebugHint(kPoint3d32f)                             \
    kAddValueDebugHint(kPoint3d64f)                             \
    kAddValueDebugHint(kRect16s)                                \
    kAddValueDebugHint(kRect32s)                                \
    kAddValueDebugHint(kRect32f)                                \
    kAddValueDebugHint(kRect64f)                                \
    kAddValueDebugHint(kRect3d64f)                              \
    kAddValueDebugHint(kRgb)                                    \
    kAddValueDebugHint(kRotatedRect32s)                         \
    kAddValueDebugHint(kRotatedRect32f)

#else

#define kDefineDebugHints_()

#endif

/* 
 * Forward declarations
 */

typedef void* kObject;
typedef void* kIterator; 

typedef kObject kAlloc; 
typedef kObject kArray1;
typedef kObject kArray2;
typedef kObject kArray3;
typedef kObject kArrayList;
typedef kObject kAssembly;
typedef kObject kBackTrace; 
typedef kObject kBox;
typedef kObject kCollection; 
typedef kObject kDynamicLib;
typedef kObject kEvent; 
typedef kObject kHttpServer;
typedef kObject kHttpServerChannel;
typedef kObject kHttpServerRequest;
typedef kObject kHttpServerResponse;
typedef kObject kImage;
typedef kObject kList; 
typedef kObject kLock; 
typedef kObject kMap;
typedef kObject kMsgQueue;
typedef kObject kPeriodic;
typedef kObject kPlugin;
typedef kObject kObjectPool;
typedef kObject kQueue;
typedef kObject kSemaphore;
typedef kObject kSerializer;
typedef kObject kSocket;
typedef kObject kStream; 
typedef kObject kString;
typedef kObject kTcpServer;
typedef kObject kThread;
typedef kObject kTimer;
typedef kObject kType; 
typedef kObject kXml;

typedef kAlloc kDebugAlloc;
typedef kAlloc kUserAlloc; 
typedef kAlloc kPoolAlloc;

typedef kSerializer kDat5Serializer;
typedef kSerializer kDat6Serializer;

typedef kStream kFile;
typedef kStream kMemory;
typedef kStream kTcpClient;
typedef kStream kUdpClient;

typedef kPointer kXmlItem;

typedef kxAtomic32s kAtomic32s;
typedef kxAtomicPointer kAtomicPointer;

/* 
 * Some core headers are included here, both for convenience, and to enforce a 
 * specific include order. For these headers, #include <kApi/kApiDef.h> should 
 * appear *before* the normal include guard, which ensures that kApiDef has 
 * complete control over include order.
 */

kEndHeader()

#include <kApi/kValue.h>
#include <kApi/kObject.h>
#include <kApi/kType.h>
#include <kApi/kAssembly.h>
#include <kApi/kAlloc.h>
#include <kApi/kApiLib.h>
#include <kApi/Threads/kAtomic.h>
#include <kApi/Utils/kUtils.h>

kBeginHeader()

/* 
 * Core Types
 */

kDeclareVoidValue(k, kVoid, kValue) 

kFx(kStatus) kVoid_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kVoid_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k8u, kValue) 

kFx(kBool) k8u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k8u_VHashCode(kType type, const void* value); 
kFx(kStatus) k8u_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k8u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k16u, kValue) 

kFx(kBool) k16u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k16u_VHashCode(kType type, const void* value); 
kFx(kStatus) k16u_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k16u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k32u, kValue)

kFx(kBool) k32u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k32u_VHashCode(kType type, const void* value); 
kFx(kStatus) k32u_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k32u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k64u, kValue)

kFx(kBool) k64u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k64u_VHashCode(kType type, const void* value); 
kFx(kStatus) k64u_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k64u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k8s, kValue)

kFx(kBool) k8s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k8s_VHashCode(kType type, const void* value); 
kFx(kStatus) k8s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k8s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k16s, kValue)

kFx(kBool) k16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k16s_VHashCode(kType type, const void* value); 
kFx(kStatus) k16s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k32s, kValue)

kFx(kBool) k32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k32s_VHashCode(kType type, const void* value); 
kFx(kStatus) k32s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k64s, kValue) 

kFx(kBool) k64s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k64s_VHashCode(kType type, const void* value); 
kFx(kStatus) k64s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k64s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k32f, kValue)

kFx(kBool) k32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k32f_VHashCode(kType type, const void* value); 
kFx(kStatus) k32f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, k64f, kValue)

kFx(kBool) k64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) k64f_VHashCode(kType type, const void* value); 
kFx(kStatus) k64f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) k64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kByte, kValue)

kFx(kBool) kByte_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kByte_VHashCode(kType type, const void* value); 
kFx(kStatus) kByte_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kByte_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kChar, kValue)

kFx(kBool) kChar_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kChar_VHashCode(kType type, const void* value); 
kFx(kStatus) kChar_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kChar_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kBool, kValue)

kFx(kBool) kBool_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kBool_VHashCode(kType type, const void* value); 
kFx(kStatus) kBool_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kBool_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kSize, kValue)

kFx(kBool) kSize_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kSize_VHashCode(kType type, const void* value); 
kFx(kStatus) kSize_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kSize_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kSSize, kValue)

kFx(kBool) kSSize_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kSSize_VHashCode(kType type, const void* value); 
kFx(kStatus) kSSize_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kSSize_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPointer, kValue)

kFx(kBool) kPointer_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPointer_VHashCode(kType type, const void* value); 

#define kIsNull_(POINTER)   ((POINTER) == kNULL)

kDeclareValue(k, kFunction, kValue) 

kFx(kBool) kFunction_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kFunction_VHashCode(kType type, const void* value); 

kDeclareArrayValue(k, kText16, kValue)

kFx(kBool) kText16_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kText16_VHashCode(kType type, const void* value); 
kFx(void) kText16_VImport(kType type, void* value, const void* source); 
kFx(kStatus) kText16_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kText16_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValue(k, kText32, kValue)

kFx(kBool) kText32_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kText32_VHashCode(kType type, const void* value); 
kFx(void) kText32_VImport(kType type, void* value, const void* source); 
kFx(kStatus) kText32_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kText32_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValue(k, kText64, kValue)

kFx(kBool) kText64_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kText64_VHashCode(kType type, const void* value); 
kFx(void) kText64_VImport(kType type, void* value, const void* source); 
kFx(kStatus) kText64_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kText64_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValue(k, kText128, kValue)

kFx(kBool) kText128_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kText128_VHashCode(kType type, const void* value); 
kFx(void) kText128_VImport(kType type, void* value, const void* source); 
kFx(kStatus) kText128_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kText128_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValue(k, kText256, kValue)

kFx(kBool) kText256_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kText256_VHashCode(kType type, const void* value); 
kFx(void) kText256_VImport(kType type, void* value, const void* source); 
kFx(kStatus) kText256_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kText256_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareEnum(k, kStatus, kValue)

kFx(kBool) kStatus_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kStatus_VHashCode(kType type, const void* value); 
kFx(kStatus) kStatus_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kStatus_Read(kType type, void* values, kSize count, kSerializer serializer); 

#if defined(K_DEBUG) && defined(K_MSVC)
#   define kDebugBreak()        __debugbreak()
#else 
#   define kDebugBreak()
#endif

#define kIsError_(STATUS)                                               \
    ((STATUS) != kOK) 

#define kSuccess_(EXPRESSION)                                           \
    ((EXPRESSION) == kOK)

#define kCheckErr(EXPRESSION, ERROR)                                    \
    do                                                                  \
    {                                                                   \
        if (!(EXPRESSION))                                              \
        {                                                               \
            kCheckTrace("kCheck failed");                               \
            return kERROR_##ERROR;                                      \
        }                                                               \
    } while(0)    

#define kCheck_(EXPRESSION)                                             \
    do                                                                  \
    {                                                                   \
        kStatus kStatus_var = (kStatus)(EXPRESSION);                    \
        if (kIsError(kStatus_var))                                      \
        {                                                               \
            kCheckTrace("kCheck failed");                               \
            return kStatus_var;                                         \
        }                                                               \
    } while(0)

#define kCheckArgs_(EXPRESSION)                                         \
    kCheckErr(EXPRESSION, PARAMETER)

#define kCheckState_(EXPRESSION)                                        \
    kCheckErr(EXPRESSION, STATE)

#define kTry_                                                           \
    {                                                                   \
        kStatus kExcept_value = kOK;

#define kThrow_(EXPRESSION)                                             \
        do                                                              \
        {                                                               \
            kExcept_value = (EXPRESSION);                               \
            kCheckTrace("kThrow exception");                            \
            goto kEXCEPT_CATCH_LABEL;                                   \
        } while (kFALSE)

#define kTest_(EXPRESSION)                                              \
        do                                                              \
        {                                                               \
            kStatus kExcept_temp = (kStatus)(EXPRESSION);               \
            if (!kSuccess(kExcept_temp))                                \
            {                                                           \
                kThrow(kExcept_temp);                                   \
            }                                                           \
        } while (kFALSE)

#define kTestErr(EXPRESSION, ERROR)                                     \
        do                                                              \
        {                                                               \
            if (!(EXPRESSION))                                          \
            {                                                           \
                kThrow(kERROR##_##ERROR);                               \
            }                                                           \
        } while (kFALSE)

#define kTestArgs_(EXPRESSION)                                          \
        kTestErr(EXPRESSION, PARAMETER)

#define kTestState_(EXPRESSION)                                         \
        kTestErr(EXPRESSION, STATE)

#define kCatch_(STATUS_POINTER)                                         \
kEXCEPT_CATCH_LABEL:                                                    \
        {                                                               \
            *(STATUS_POINTER) = kExcept_value;                          \
            if (kSuccess(kExcept_value))                                \
            {                                                           \
                goto kEXCEPT_CATCH_END_LABEL;                           \
            }                                                           \

#define kEndCatch_(status)                                              \
            kCheck(status);                                             \
        }                                                               \
    }                                                                   \
kEXCEPT_CATCH_END_LABEL:                                                \
    (void)0


#define kFinally_                                                       \
kEXCEPT_CATCH_LABEL:

#define kEndFinally_()                                                  \
        kCheck(kExcept_value);                                          \
    } (void)0

#define kCatchEx_(STATUS_POINTER)                                       \
kEXCEPT_CATCH_LABEL:                                                    \
        {                                                               \
            *(STATUS_POINTER) = kExcept_value;                          \
            if (kSuccess(kExcept_value))                                \
            {                                                           \
                goto kEXCEPT_FINALLY_LABEL;                             \
            }

#define kEndCatchEx_(STATUS)                                            \
    kExcept_value = (STATUS)

#define kFinallyEx_                                                     \
        }                                                               \
kEXCEPT_FINALLY_LABEL:

#define kEndFinallyEx_()                                                \
        kCheck(kExcept_value);                                          \
    } (void)0

#if (defined(K_DEBUG) || defined(K_ASSERT)) && !defined(K_NO_ASSERT)

#   define kAssert_(EXPRESSION)                                         \
        do                                                              \
        {                                                               \
            kBool kAssert_result = (EXPRESSION);                        \
            if (!kAssert_result)                                        \
            {                                                           \
                if (kApiLib_AssertHandler_())                           \
                {                                                       \
                    kApiLib_AssertHandler_()(__FILE__, __LINE__);       \
                }                                                       \
                else                                                    \
                {                                                       \
                    assert(kAssert_result);                             \
                }                                                       \
            }                                                           \
        } while(0)

#   define kAssertOk_(EXPRESSION)                                       \
        kAssert(kSuccess(EXPRESSION))

#   define kAssertType_(EXPRESSION, SYMBOL)                             \
        kAssert(kObject_Is(EXPRESSION, kTypeOf(SYMBOL)))

#   define kCastClass_(SYMBOL, OBJ)                                                 \
        (kObject_Is(OBJ, kTypeOf(SYMBOL)) ?                                         \
            kCast(SYMBOL##Class*, OBJ) :                                            \
            kCast(SYMBOL##Class*, kApiLib_CastFailHandler(__FILE__, __LINE__)))

#   define kTypeOf_(SYMBOL)                                                         \
        (!kIsNull_(SYMBOL##_type) ?                                                 \
            (SYMBOL##_type) :                                                       \
            kApiLib_CastFailHandler(__FILE__, __LINE__))

#else

#   define kAssert_(EXPRESSION)                     ((void)0)
#   define kAssertOk_(EXPRESSION)                   ((void)0)
#   define kAssertType_(EXPRESSION, SYMBOL)         ((void)0)

#   define kCastClass_(SYMBOL, OBJ)             \
        kCast(SYMBOL##Class*, OBJ)

#   define kCastInterface_(SYMBOL, OBJ)         \
        kCast(kObjectClass*, OBJ)

#   define kTypeOf_(SYMBOL)                     \
        (SYMBOL##_type)


#endif

#define kCastVTable_(SYMBOL, OBJ)               \
    kCast_(SYMBOL##VTable*, kType_VTable_(kObject_Type_(OBJ)))

#define kCastIVTable_(SYMBOL, OBJ)               \
    kCast_(SYMBOL##VTable*, kType_IVTable_(kObject_Type_(OBJ), kTypeOf(SYMBOL)))

#if !defined(K_NO_TRACE)

#   define kTrace_(TAG)                                                             \
        if (!kIsNull(kApiLib_TraceHandler_()))                                      \
        {                                                                           \
            kApiLib_TraceHandler_()(TAG, __FILE__, __LINE__);                       \
        }

#else

#   define kTrace_(TAG)

#endif

#if defined(K_CHECK_TRACE)

#   define kCheckTrace(TAG)                                                         \
        do                                                                          \
        {                                                                           \
            if (!kIsNull(kApiLib_TraceHandler_()) && kApiLib_CheckTraceEnabled_())  \
            {                                                                       \
                kApiLib_TraceHandler_()(TAG, __FILE__, __LINE__);                   \
            }                                                                       \
        } while (0)

#else

#   define kCheckTrace(TAG)                     \
        (void)0

#endif


kDeclareValue(k, kVersion, kValue)

kFx(kBool) kVersion_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kVersion_VHashCode(kType type, const void* value); 
kFx(kStatus) kVersion_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kVersion_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kVersion_Create_(MAJOR, MINOR, RELEASE, BUILD)                  \
    ((MAJOR) & 0xFF) << 24 | ((MINOR) & 0xFF) << 16 |                   \
    ((RELEASE) & 0xFF) << 8 | ((BUILD) & 0xFF)

#define kVersion_Stringify_(MAJOR, MINOR, RELEASE, BUILD)               \
    kStringify_(MAJOR)"."kStringify_(MINOR)"."kStringify_(RELEASE)"."kStringify_(BUILD)

kDeclareValue(k, kPoint16s, kValue)

kFx(kBool) kPoint16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint16s_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint16s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPoint32s, kValue)

kFx(kBool) kPoint32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint32s_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint32s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPoint32f, kValue)

kFx(kBool) kPoint32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint32f_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint32f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPoint64f, kValue)

kFx(kBool) kPoint64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint64f_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint64f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kxPoint_Init_(POINT, X, Y)          \
    ((POINT)->x = X, (POINT)->y = (Y))

kDeclareValue(k, kPoint3d16s, kValue)

kFx(kBool) kPoint3d16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint3d16s_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint3d16s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint3d16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPoint3d32s, kValue)

kFx(kBool) kPoint3d32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint3d32s_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint3d32s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint3d32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPoint3d32f, kValue)

kFx(kBool) kPoint3d32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint3d32f_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint3d32f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint3d32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kPoint3d64f, kValue)

kFx(kBool) kPoint3d64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPoint3d64f_VHashCode(kType type, const void* value); 
kFx(kStatus) kPoint3d64f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPoint3d64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kxPoint3d_Init_(POINT, X, Y, Z)                         \
    ((POINT)->x = X, (POINT)->y = (Y), (POINT)->z = (Z))

kDeclareValue(k, kRect16s, kValue)

kFx(kBool) kRect16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRect16s_VHashCode(kType type, const void* value); 
kFx(kStatus) kRect16s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRect16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kRect32s, kValue)

kFx(kBool) kRect32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRect32s_VHashCode(kType type, const void* value); 
kFx(kStatus) kRect32s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRect32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kRect32f, kValue)

kFx(kBool) kRect32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRect32f_VHashCode(kType type, const void* value); 
kFx(kStatus) kRect32f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRect32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kRect64f, kValue)

kFx(kBool) kRect64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRect64f_VHashCode(kType type, const void* value); 
kFx(kStatus) kRect64f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRect64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kRect3d64f, kValue)

kFx(kBool) kRect3d64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRect3d64f_VHashCode(kType type, const void* value); 
kFx(kStatus) kRect3d64f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRect3d64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kxRect_Init_(RECT, X, Y, W, H)                          \
    ((RECT)->x = X, (RECT)->y = (Y),                            \
     (RECT)->width = (W), (RECT)->height = (H))

#define kxRect3d_Init_(RECT, X, Y, Z, W, H, D)                   \
    ((RECT)->x = X, (RECT)->y = (Y), (RECT)->z = (Z),            \
     (RECT)->width = (W), (RECT)->height = (H),                  \
     (RECT)->depth = (D))

kDeclareValue(k, kRotatedRect32s, kValue)

kFx(kBool) kRotatedRect32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRotatedRect32s_VHashCode(kType type, const void* value); 
kFx(kStatus) kRotatedRect32s_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRotatedRect32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kRotatedRect32f, kValue)

kFx(kBool) kRotatedRect32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRotatedRect32f_VHashCode(kType type, const void* value); 
kFx(kStatus) kRotatedRect32f_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRotatedRect32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kxRotatedRect_Init_(RECT, XC, YC, W, H, A)              \
    ((RECT)->xc = XC, (RECT)->yc = (YC),                        \
     (RECT)->width = (W), (RECT)->height = (H),                 \
     (RECT)->angle = (A))

kDeclareEnum(k, kPixelFormat, kValue)

kFx(kBool) kPixelFormat_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kPixelFormat_VHashCode(kType type, const void* value); 
kFx(kStatus) kPixelFormat_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kPixelFormat_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareEnum(k, kCfa, kValue)

kFx(kBool) kCfa_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kCfa_VHashCode(kType type, const void* value); 
kFx(kStatus) kCfa_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kCfa_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kRgb, kValue)

kFx(kBool) kRgb_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kRgb_VHashCode(kType type, const void* value); 
kFx(kStatus) kRgb_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kRgb_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kxRgb_Init_(RGB, R, G, B)                               \
    ((RGB)->b = (B), (RGB)->g = (G), (RGB)->r = (R)) 

kDeclareValue(k, kArgb, kValue)

kFx(kBool) kArgb_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kArgb_VHashCode(kType type, const void* value); 
kFx(kStatus) kArgb_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kArgb_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define kxArgb_Init_(ARGB, A, R, G, B)                          \
    ((ARGB)->b = (B), (ARGB)->g = (G),                          \
     (ARGB)->r = (R), (ARGB)->a = (A))

kDeclareEnum(k, kComparison, kValue)

kFx(kBool) kComparison_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kComparison_VHashCode(kType type, const void* value); 
kFx(kStatus) kComparison_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) kComparison_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValue(k, kCallbackFx, kValue) 

kFx(kBool) kCallbackFx_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kCallbackFx_VHashCode(kType type, const void* value); 

kDeclareValue(k, kCallback, kValue)

kFx(kBool) kCallback_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) kCallback_VHashCode(kType type, const void* value); 

kDeclareBitEnum(k, kFileMode, kValue)

kDeclareEnum(k, kSeekOrigin, kValue)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

//simple renaming (handled by porting script)
#   define KAPI_CALL                       kCall
#   define KAPI_EXPORT                     kFx
#   define kUIntPtr                        kSize
#   define kUINTPTR_MAX                    kSIZE_MAX
#   define kIntPtr                         kSSize
#   define kINTPTR_MAX                     kSSIZE_MAX
#   define kINTPTR_MIN                     kSSIZE_MIN
#   define kNIL                            kNULL
#   define kIS_NULL                        kIsNull
#   define kIS_NIL                         kIsNull
#   define kHandle                         kPointer
#   define kNULL_HANDLE                    (kNIL)
#   define kHandle_FromPointer(P)          ((kPointer)P)
#   define kHandle_Pointer(H)              ((kPointer)H)
#   define kERROR_TYPE                     kERROR_PARAMETER
#   define kERROR_NOTIMPL                  kERROR_UNIMPLEMENTED
#   define kERROR_DRIVER                   kERROR
#   define kERROR_INVALID_GENERATION       kERROR_STATE
#   define kERROR_INVALID_STATE            kERROR_STATE
#   define kERROR_NETWORK_SETUP            kERROR_STATE
#   define kERROR_COMMUNICATION_RESPONSE   kERROR_STATE
#   define kERROR_PACKET_LOST              kERROR_INCOMPLETE
#   define kSUCCESS                        kSuccess
#   define kCHECK                          kCheck
#   define kEXCEPT_TRY                     kTry
#   define kEXCEPT_THROW                   kThrow
#   define kEXCEPT_CHECK                   kTest
#   define kPOINT                          kPoint_Init_
#   define kPOINT3D                        kPoint3d_Init_
#   define kRECT                           kRect_Init_
#   define kRECT3D                         kRect3d_Init_
#   define kROTATED_RECT                   kRotatedRect_Init_
#   define kRGB                            kRgb_Init_
#   define kARGB                           kArgb_Init_
#   define kASSERT                         kAssert
#   define kCOUNT                          kCountOf
#   define kMATH_MIN                       kMin_
#   define kMATH_MAX                       kMax_
#   define kMATH_ABS                       kAbs_
#   define kMATH_CLAMP                     kClamp_
#   define kTYPE_NONE                      kTypeOf(kVoid)
#   define kTYPE_8U                        kTypeOf(k8u)
#   define kTYPE_16U                       kTypeOf(k16u)
#   define kTYPE_32U                       kTypeOf(k32u)
#   define kTYPE_64U                       kTypeOf(k64u)
#   define kTYPE_8S                        kTypeOf(k8s)
#   define kTYPE_16S                       kTypeOf(k16s)
#   define kTYPE_32S                       kTypeOf(k32s)
#   define kTYPE_64S                       kTypeOf(k64s)
#   define kTYPE_32F                       kTypeOf(k32f)
#   define kTYPE_64F                       kTypeOf(k64f)
#   define kTYPE_BYTE                      kTypeOf(kByte)
#   define kTYPE_CHAR                      kTypeOf(kChar)
#   define kTYPE_BOOL                      kTypeOf(kBool)
#   define kTYPE_UINTPTR                   kTypeOf(kSize)
#   define kTYPE_INTPTR                    kTypeOf(kSSize)
#   define kTYPE_POINTER                   kTypeOf(kPointer)
#   define kTYPE_TEXT_16                   kTypeOf(kText16)
#   define kTYPE_TEXT_32                   kTypeOf(kText32)
#   define kTYPE_TEXT_64                   kTypeOf(kText64)
#   define kTYPE_TEXT_128                  kTypeOf(kText128)
#   define kTYPE_TEXT_256                  kTypeOf(kText256)
#   define kTYPE_VERSION                   kTypeOf(kVersion)
#   define kTYPE_POINT_16S                 kTypeOf(kPoint16s)
#   define kTYPE_POINT_32S                 kTypeOf(kPoint32s)
#   define kTYPE_POINT_32F                 kTypeOf(kPoint32f)
#   define kTYPE_POINT_64F                 kTypeOf(kPoint64f)
#   define kTYPE_POINT3D_16S               kTypeOf(kPoint3d16s)
#   define kTYPE_POINT3D_32S               kTypeOf(kPoint3d32s)
#   define kTYPE_POINT3D_32F               kTypeOf(kPoint3d32f)
#   define kTYPE_POINT3D_64F               kTypeOf(kPoint3d64f)
#   define kTYPE_RECT_16S                  kTypeOf(kRect16s)
#   define kTYPE_RECT_32S                  kTypeOf(kRect32s)
#   define kTYPE_RECT_32F                  kTypeOf(kRect32f)
#   define kTYPE_RECT_64F                  kTypeOf(kRect64f)
#   define kTYPE_RECT3D_64F                kTypeOf(kRect3d64f)
#   define kTYPE_ROTATED_RECT_32S          kTypeOf(kRotatedRect32s)
#   define kTYPE_ROTATED_RECT_32F          kTypeOf(kRotatedRect32f)
#   define kTYPE_CFA                       kTypeOf(kCfa)
#   define kTYPE_RGB                       kTypeOf(kRgb)
#   define kTYPE_ARGB                      kTypeOf(kArgb)
#   define kTYPE_SIZE                      kType_Size_

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kHANDLE(HANDLE, POINTER)                 (*(HANDLE) = (POINTER))
#   define kHANDLE_POINTER(HANDLE)                  (HANDLE)
#   define kHANDLE_IS_NIL(HANDLE)                   ((HANDLE) == kNULL)
#   define kHANDLE_IS_NULL(HANDLE)                  (kHANDLE_IS_NIL(HANDLE))
#   define kHANDLE_EQUALS(HANDLE1, HANDLE2)         ((HANDLE1) == (HANDLE2))
#   define kASSERT_CREATED(OBJECT, TYPE)            kAssert(kObject_Is(OBJECT, TYPE))
#   define kASSERT_INITIALIZED(OBJECT, TYPE)        kAssert(kObject_Is(OBJECT, TYPE) && kOBJECT_IS_INITIALIZED(OBJECT))

#   define kEXCEPT_SET(EXPRESSION)                                      \
        do                                                              \
        {                                                               \
            kStatus kExcept_temp = (kStatus)(EXPRESSION);               \
            if (kSUCCESS(kExcept_value))                                \
            {                                                           \
                kExcept_value = kExcept_temp;                           \
            }                                                           \
        } while (kFALSE)

#   define kEXCEPT_CLEAR()                                              \
        kExcept_value = kOK

#   define kEXCEPT_CATCH(STATUS_POINTER)                                \
kEXCEPT_CATCH_LABEL:                                                    \
        {                                                               \
            kStatus* kExcept_userValue = (STATUS_POINTER);              \
            if (kIS_NULL(kExcept_userValue) || kSUCCESS(kExcept_value)) \
            {                                                           \
                goto kEXCEPT_FINALLY_LABEL;                             \
            }                                                           \
            else                                                        \
            {                                                           \
                *kExcept_userValue = kExcept_value;                     \
                kEXCEPT_CLEAR();                                        \
            }

#   define kEXCEPT_FINALLY                                              \
        }                                                               \
kEXCEPT_FINALLY_LABEL:

#   define kEXCEPT_CLOSE()                                              \
        if (!kSUCCESS(kExcept_value))                                   \
        {                                                               \
            return kExcept_value;                                       \
        }                                                               \
    } do {} while(kFALSE)

#   define kCHECK_ASSERT(EXPRESSION)                                    \
        do                                                              \
        {                                                               \
            kStatus kCheckAssert_result = (EXPRESSION);                 \
            kAssert(kCheckAssert_result == kOK);                        \
        } while(0)

#   define kVersion_FromString(B, V)       kVersion_Parse(V, B)
#   define kVersion_ToString(V, B, C)      kVersion_Format(V, B, C)

    kFx(kStatus) kVersion_Fields(kVersion version, k32u* major, k32u* minor, k32u* release, k32u* build);

#   define kApi_Version()                   kAssembly_Version(kAssemblyOf(kApiLib))

#endif      /* #if defined(K_COMPAT_5) */

kEndHeader()

#endif
