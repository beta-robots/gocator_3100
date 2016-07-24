/** 
 * @file    GoMultiplexBank.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoMultiplexBank.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClass(Go, GoMultiplexBank, kObject)
    kAddVMethod(GoMultiplexBank, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoMultiplexBank_Construct(GoMultiplexBank* bank, k32u id, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoMultiplexBank), bank)); 

    if (!kSuccess(status = GoMultiplexBank_Init(*bank, kTypeOf(GoMultiplexBank), id, alloc)))
    {
        kAlloc_FreeRef(alloc, bank); 
    }

    return status; 
} 

GoFx(kStatus) GoMultiplexBank_Init(GoMultiplexBank bank, kType type,  k32u id, kAlloc alloc)
{
    GoMultiplexBankClass* obj = bank; 
    kStatus exception;

    kCheck(kObject_Init(bank, type, alloc)); 
    kInitFields_(GoMultiplexBank, bank);

    obj->id = id; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->sensorList, kTypeOf(GoSensor), 0, alloc));
    }
    kCatch(&exception)
    {
        GoMultiplexBank_VRelease(bank);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoMultiplexBank_VRelease(GoMultiplexBank bank)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 

    kCheck(kDestroyRef(&obj->sensorList));  //dispose not called - do not want to destroy all sensor handles, as they may still be used elsewhere

    return kObject_VRelease(bank);
}

GoFx(kStatus) GoMultiplexBank_AddSensor(GoMultiplexBank bank, GoSensor sensor)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 
    kSize i;   

    for (i = 0; i < kArrayList_Count(obj->sensorList); i++)
    {
        if (GoSensor_Id(sensor) == GoSensor_Id(kArrayList_As_(obj->sensorList, i, GoSensor)))
        {
            return kERROR_ALREADY_EXISTS;
        }
    }

    kCheck(kArrayList_Add(obj->sensorList, &sensor));

    return kOK;
}

GoFx(kStatus) GoMultiplexBank_RemoveSensor(GoMultiplexBank bank, k32u id)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->sensorList); i++)
    {
        if (id == GoSensor_Id(kArrayList_As_(obj->sensorList, i, GoSensor)))
        {
            kCheck(kArrayList_Remove(obj->sensorList, i, kNULL));

            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoMultiplexBank_SensorCount(GoMultiplexBank bank)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 

    return kArrayList_Count(obj->sensorList);
}

GoFx(GoSensor) GoMultiplexBank_SensorAt(GoMultiplexBank bank, kSize index)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 

    kAssert(index < kArrayList_Count(obj->sensorList));

    return kArrayList_As_(obj->sensorList, index, GoSensor);
}

GoFx(kBool) GoMultiplexBank_HasSensor(GoMultiplexBank bank, k32u id)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->sensorList); i++)
    {
        if (id == GoSensor_Id(kArrayList_As_(obj->sensorList, i, GoSensor)))
        {
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(k32u) GoMultiplexBank_Id(GoMultiplexBank bank)
{
    GoMultiplexBankClass* obj = GoMultiplexBank_Cast_(bank); 

    return obj->id;
}
