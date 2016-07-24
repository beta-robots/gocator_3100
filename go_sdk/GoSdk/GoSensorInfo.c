/** 
 * @file    GoSensorInfo.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSensorInfo.h>
#include <kApi/Utils/kUtils.h>

kBeginClass(Go, GoSensorInfo, kObject)
kEndClass()

GoFx(kStatus) GoSensorInfo_Construct(GoSensorInfo* info, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSensorInfo), info)); 

    if (!kSuccess(status = GoSensorInfo_Init(*info, kTypeOf(GoSensorInfo), alloc)))
    {
        kAlloc_FreeRef(alloc, info); 
    }

    return status; 
} 

GoFx(kStatus) GoSensorInfo_Init(GoSensorInfo info, kType type, kAlloc alloc)
{
    GoSensorInfoClass* obj = info; 

    kCheck(kObject_Init(info, type, alloc)); 
    kInitFields_(GoSensorInfo, info);

    obj->role = GO_ROLE_MAIN; 
    obj->state = GO_STATE_OFFLINE; 
    
    return kOK; 
}


GoFx(kStatus) GoSensorInfo_Read(GoSensorInfo info, kSerializer serializer)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info); 
    k32s temp; 
    kByte tempArray[4];
    kSize i;

    kCheck(kSerializer_Read32u(serializer, &obj->id)); 

    kCheck(kSerializer_ReadByteArray(serializer, tempArray, kCountOf(tempArray)));
    for (i = 0; i < 16; i++)
    {
        if (i < 4)
        {
            obj->address.address[i] = tempArray[i];
        }
        else
        {
            obj->address.address[i] = 0;
        }
    }

    kCheck(kSerializer_ReadCharArray(serializer, obj->modelName, kCountOf(obj->modelName))); 

    kCheck(kSerializer_ReadByteArray(serializer, tempArray, kCountOf(tempArray)));

    obj->firmwareVersion = kVersion_Create(tempArray[0], tempArray[1], tempArray[2], tempArray[3]);

    kCheck(kSerializer_Read32s(serializer, &temp)); 
    switch(temp)
    {
    case -1:     obj->state = GO_STATE_INCOMPLETE;       break;
    case 0:     obj->state = GO_STATE_READY;            break;
    case 1:     obj->state = GO_STATE_RUNNING;          break;
    default:    obj->state = GO_STATE_CONNECTED;        break;
    }

    kCheck(kSerializer_Read32s(serializer, &obj->role)); 

    kCheck(kSerializer_Read32s(serializer, &temp));
    obj->buddyId = (k32u)temp;

    return kOK; 
}

GoFx(k32u) GoSensorInfo_Id(GoSensorInfo info)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return obj->id; 
}

GoFx(kVersion) GoSensorInfo_Firmware(GoSensorInfo info)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return obj->firmwareVersion; 
}

GoFx(kStatus) GoSensorInfo_Model(GoSensorInfo info, kChar* model, k32u capacity)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return kStrCopy(model, capacity, obj->modelName); 
}

GoFx(GoRole) GoSensorInfo_Role(GoSensorInfo info)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return obj->role; 
}

GoFx(kBool) GoSensorInfo_HasBuddy(GoSensorInfo info)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return (obj->buddyId != 0); 
}

GoFx(k32u) GoSensorInfo_BuddyId(GoSensorInfo info)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return obj->buddyId; 
}

GoFx(GoState) GoSensorInfo_State(GoSensorInfo info)
{
    GoSensorInfoClass* obj = GoSensorInfo_Cast_(info);  
    return obj->state; 
}
