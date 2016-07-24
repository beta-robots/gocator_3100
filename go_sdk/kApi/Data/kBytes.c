/** 
 * @file    kBytes.c
 *
 * @internal
 * Copyright (C) 2012-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kBytes.h>
#include <kApi/kApiLib.h>
#include <kApi/Io/kSerializer.h>
#include <stdio.h>

static kType kBytes_types[kBYTES_CAPACITY] = { 0 }; 

kFx(kStatus) kBytes_AddTypes(kAssembly assembly)
{
    kChar name[64];
    kSize i; 
    
    kCheck(kMemSet(&kBytes_types[0], 0, sizeof(kBytes_types))); 
    
    //add each kByteN type
    for (i = 1; i < kBYTES_CAPACITY; ++i)
    {
        kCheck(kStrPrintf(name, kCountOf(name), "kByte%u", i)); 
        kCheck(kAssembly_AddType(assembly, kBytes_Register, name)); 
    }

    return kOK; 
}

kFx(kStatus) kBytes_Register(kAssembly assembly, const kChar* name)
{    
    kSize guid5Base = 0x40000000; 
    kChar guid5[64];
    kChar guid6[64];
    k32u typeIndex; 

    kCheckArgs(sscanf(name, "kByte%u", &typeIndex) == 1); 

    kCheck(kStrPrintf(guid5, kCountOf(guid5), "%u-0", guid5Base + typeIndex)); 
    kCheck(kStrPrintf(guid6, kCountOf(guid6), "kBytes%u-0", typeIndex)); 

    kCheck(kAssembly_AddValue(assembly, &kBytes_types[typeIndex], name, kTypeOf(kValue), "kValue", typeIndex, sizeof(kValueVTable), kTYPE_FLAGS_VALUE)); 

    kCheck(kType_AddVMethod(kBytes_types[typeIndex], offsetof(kValueVTable, VEquals)/sizeof(kPointer), (kFunction)kBytes_VEquals, "VEquals")); 
    kCheck(kType_AddVMethod(kBytes_types[typeIndex], offsetof(kValueVTable, VHashCode)/sizeof(kPointer), (kFunction)kBytes_VHashCode, "VHashCode")); 

    kCheck(kType_AddVersion(kBytes_types[typeIndex], "kdat5", "5.0.0.0", guid5, (kFunction)kBytes_Write, (kFunction)kBytes_Read));  
    kCheck(kType_AddVersion(kBytes_types[typeIndex], "kdat6", "5.7.1.0", guid6, (kFunction)kBytes_Write, (kFunction)kBytes_Read));  

    return kOK; 
}

kFx(kBool) kBytes_VEquals(kType type, const void* value, const void* other)                    
{                                                                                              
    return kMemEquals(value, other, kType_Size_(type));                                                      
}           

kFx(kSize) kBytes_VHashCode(kType type, const void* value)                                     
{                                                                                              
    return kHashBytes(value, kType_Size_(type));                                                             
}

kFx(kStatus) kBytes_Write(kType type, void* values, kSize count, kSerializer serializer)       
{                                                                                              
    return kSerializer_WriteByteArray_(serializer, values, kType_Size_(type)*count);                         
}

kFx(kStatus) kBytes_Read(kType type, void* values, kSize count, kSerializer serializer)        
{                                                                                              
    return kSerializer_ReadByteArray_(serializer, values, kType_Size_(type)*count);                          
}

kFx(kType) kBytes_GetType(kSize size)
{
    if (size < kBYTES_CAPACITY)
    {
        return kBytes_types[size]; 
    }
    else
    {
        //If this assertion fails, the requested size is larger than the maximum size 
        //supported by the kTYPE_BYTES macro. Consider defining a custom value type instead.
        kAssert(size < kBYTES_CAPACITY); 

        return kNULL; 
    }
}
