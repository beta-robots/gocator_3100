/** 
 * @file    GoSensor.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoSystem.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/Messages/GoHealth.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kPath.h>
#include <kApi/Utils/kUtils.h>

kBeginClass(Go, GoSensor, kObject)
    kAddVMethod(GoSensor, kObject, VRelease)
kEndClass()

GoFx(kStatus) GoSensor_Construct(GoSensor* sensor, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSensor), sensor)); 

    if (!kSuccess(status = GoSensor_Init(*sensor, kTypeOf(GoSensor), system, discoveryInfo, alloc)))
    {
        kAlloc_FreeRef(alloc, sensor); 
    }

    return status; 
} 

GoFx(kStatus) GoSensor_Init(GoSensor sensor, kType type, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc alloc)
{
    GoSensorClass* obj = sensor;
    kStatus status; 

    kCheck(kObject_Init(sensor, type, alloc));
    kInitFields_(GoSensor, sensor); 

    obj->system = system; 
    obj->deviceId = discoveryInfo->id; 
    obj->address = discoveryInfo->address; 
    obj->role = GO_ROLE_INVALID; 
    obj->alignmentState = GO_ALIGNMENT_STATE_NOT_ALIGNED; 
    obj->user = GO_USER_NONE; 
    obj->sensorInfoTime = k64U_NULL; 

    kTry
    {
        kTest(GoSensorInfo_Construct(&obj->localSensorInfo, alloc)); 
        kTest(kArrayList_Construct(&obj->fileList, kTypeOf(kText64), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->directoryList, kTypeOf(kText64), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->partModelList, kTypeOf(GoPartModel), 0, alloc)); 
        kTest(kTimer_Construct(&obj->timer, alloc)); 
        kTest(kPeriodic_Construct(&obj->resetTimer, alloc)); 

        kTest(kArrayList_Construct(&obj->remoteSensorInfo, kTypeOf(GoSensorInfo), 0, alloc));
    }
    kCatch(&status)
    {
        GoSensor_VRelease(sensor); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_VRelease(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_Disconnect(sensor)); 

    kCheck(kPeriodic_Stop(obj->resetTimer));
    kCheck(kDestroyRef(&obj->resetTimer)); 
    kCheck(kDisposeRef(&obj->remoteSensorInfo));
    kCheck(kDestroyRef(&obj->localSensorInfo)); 
    kCheck(kDisposeRef(&obj->partModelList)); 
    kCheck(kDisposeRef(&obj->directoryList)); 
    kCheck(kDisposeRef(&obj->fileList)); 
    kCheck(kDestroyRef(&obj->timer)); 

    kCheck(kDestroyRef(&obj->configXml)); 
    kCheck(kDestroyRef(&obj->transformXml)); 

    kCheck(kObject_VRelease(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_SetAddress(GoSensor sensor, const GoAddressInfo* info, kBool wait)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoDiscovery discovery = kNULL;     
    kBool wasConnected = GoSensor_IsConnected(sensor); 

    kTry
    {
        if (wasConnected)
        {
            kTest(GoSensor_Flush(sensor)); 
        }

        kTest(GoSensor_BeginReset(sensor)); 
        kTest(GoSensor_Disconnect(sensor)); 

        kTest(GoDiscovery_Construct(&discovery, kObject_Alloc_(sensor))); 
        kTest(GoDiscovery_SetAddress(discovery, obj->deviceId, info));         

        if (wait)
        {
            if (wasConnected)
            {
                kTest(GoSensor_WaitForReconnect(sensor, GO_SYSTEM_RESET_TIMEOUT)); 
            }
            else
            {
                kTest(GoSensor_WaitForReboot(sensor, GO_SYSTEM_RESET_TIMEOUT)); 
            }
        }
    }
    kFinally
    {
        kDestroyRef(&discovery); 
        kEndFinally();
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_Address(GoSensor sensor, GoAddressInfo* info)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(GoSensor_LockState(sensor)); 
    {  
        *info = obj->address; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 
   
    return kOK; 
}

GoFx(kStatus) GoSensor_Connect(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor);
    kStatus status; 
    kBool attemptingToOpenControl = kFALSE;

    kCheck(GoSensor_Disconnect(sensor));  

    kCheckState(GoSensor_State(sensor) == GO_STATE_ONLINE); 

    kTry
    {
        kTest(GoControl_Construct(&obj->control, alloc)); 
        kTest(GoControl_SetCancelHandler(obj->control, GoSensor_OnCancelQuery, sensor)); 

        attemptingToOpenControl = kTRUE;
        kTest(GoControl_Open(obj->control, obj->address.address)); 
        attemptingToOpenControl = kFALSE;

        if (GoControl_IsCompatible(obj->control))
        {
            kTest(GoSensor_EnableHealth(sensor, kTRUE)); 
        }

        kTest(GoSetup_Construct(&obj->setup, sensor, alloc)); 
        kTest(GoTools_Construct(&obj->tools, sensor, alloc)); 
        kTest(GoOutput_Construct(&obj->output, sensor, alloc)); 
        kTest(GoTransform_Construct(&obj->transform, sensor, alloc)); 
        
        kTest(GoSensor_SetConnected(sensor, kTRUE, GoControl_IsCompatible(obj->control))); 
        kTest(GoSensor_ReadInfo(sensor));
        kTest(GoSensor_ReadConfig(sensor));
        kTest(GoSensor_ReadPartModels(sensor));
        kTest(GoSensor_ReadTransform(sensor));
    }
    kCatch(&status)
    {
        //if kERROR_COMMAND is returned
        if (status != kERROR_COMMAND || attemptingToOpenControl == kFALSE)
        {
            GoSensor_Disconnect(sensor); 
        }

        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_Disconnect(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (GoSensor_IsConfigurable(sensor))
    {
        GoSensor_Flush(sensor);
    }
    
    kTry
    {
        kTest(GoSensor_Cancel(sensor)); 
    }
    kFinally
    {
        kCheck(kDisposeRef(&obj->transform)); 
        kCheck(kDisposeRef(&obj->output)); 
        kCheck(kDisposeRef(&obj->tools)); 
        kCheck(kDisposeRef(&obj->setup)); 
        kCheck(kDisposeRef(&obj->control)); 
        kCheck(kDisposeRef(&obj->data)); 
        kCheck(kDisposeRef(&obj->health)); 

        kCheck(GoSensor_SetConnected(sensor, kFALSE, kFALSE)); 
        kCheck(GoSensor_Invalidate(sensor)); 

        kEndFinally();
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_SetConnected(GoSensor sensor, kBool isConnected, kBool isCompatible)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isConnected = isConnected; 
        obj->isCompatible = isCompatible; 

        if (!isConnected)
        {
            obj->isCancelled = kFALSE; 
        }
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Flush(GoSensor sensor)
{    
    kCheck(GoSensor_FlushConfig(sensor)); 
    kCheck(GoSensor_FlushTransform(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Invalidate(GoSensor sensor)
{    
    kCheck(GoSensor_InvalidateRole(sensor)); 
    kCheck(GoSensor_InvalidateInfo(sensor)); 
    kCheck(GoSensor_InvalidateConfig(sensor)); 
    kCheck(GoSensor_InvalidateTransform(sensor)); 
    kCheck(GoSensor_InvalidateFileList(sensor)); 
    kCheck(GoSensor_InvalidateDirectoryList(sensor));

    return kOK; 
}

GoFx(kStatus) GoSensor_SyncConfig(GoSensor sensor)
{    
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->isSyncConfig)
    {
        kTry
        {
            obj->isSyncConfig = kTRUE;
            kTest(GoSensor_FlushConfig(sensor)); 
            kTest(GoSensor_CacheConfig(sensor));             
        }
        kFinally
        {
            obj->isSyncConfig = kFALSE;
            kEndFinally();
        }        
    }
    
    return kOK; 
}

GoFx(kStatus) GoSensor_SyncPartModels(GoSensor sensor)
{    
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->isSyncPartModels)
    {
        kTry
        {
            obj->isSyncPartModels = kTRUE;
            kTest(GoSensor_FlushPartModels(sensor)); 
            kTest(GoSensor_CachePartModels(sensor));             
        }
        kFinally
        {
            obj->isSyncPartModels = kFALSE;
            kEndFinally();
        }        
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_FlushPartModels(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
    {
        GoPartModelClass* modelObj = GoPartModel_Cast_(*(GoPartModel*)kArrayList_At(obj->partModelList, i));

        if (modelObj->isModified)
        {
            kCheck(GoSensor_WritePartModel(sensor, modelObj)); 
        }
    }    

    return kOK; 
}

GoFx(kStatus) GoSensor_CachePartModels(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize i, j;
    kText64 tempText = "";
    kStatus exception = kOK;

    // Remove part models that are no longer present in the live job
    for (i = 0; i < kArrayList_Count(obj->partModelList); )
    {
        GoPartModelClass* modelObj = GoPartModel_Cast_(*(GoPartModel*)kArrayList_At(obj->partModelList, i));
        kBool foundFile = kFALSE;
        GoPartModel partModel = kNULL;

        for (j = 0; j < GoSensor_DirectoryFileCount(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE); j++)
        {
            kCheck(GoSensor_DirectoryFileNameAt(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE, j, tempText, 64));
            strtok(tempText, ".");

            if (strcmp(tempText, modelObj->name) == 0)
            {
                foundFile = kTRUE;
            }
        }

        if (!foundFile)
        {
            kCheck(kArrayList_Remove(obj->partModelList, i, &partModel));
            kDestroyRef(&partModel);
        }
        else
        {
            i++;
        }
    }

    // Add or update part models in the live job
    for (i = 0; i < GoSensor_DirectoryFileCount(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE); i++)
    {
        kBool foundModel = kFALSE;

        kCheck(GoSensor_DirectoryFileNameAt(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE, i, tempText, 64));
        strtok(tempText, "."); //stripping out .mdl extension

        for (j = 0; j < kArrayList_Count(obj->partModelList); j++)
        {
            GoPartModelClass* modelObj = GoPartModel_Cast_(*(GoPartModel*)kArrayList_At(obj->partModelList, j));
            
            if (strcmp(tempText, modelObj->name) == 0)
            {
                foundModel = kTRUE;

                if (!modelObj->isValid)
                {
                    kCheck(GoSensor_ReadPartModel(sensor, modelObj)); 
                    break;
                }
            }
        }

        if (!foundModel)
        {
            GoPartModel modelToAdd = kNULL;

            kTry
            {
                kTest(GoPartModel_Construct(&modelToAdd, sensor, tempText, kObject_Alloc(sensor)));
                kTest(GoSensor_ReadPartModel(sensor, modelToAdd)); 
                kTest(kArrayList_Add(obj->partModelList, &modelToAdd));
            }
            kCatch(&exception)
            {
                kDestroyRef(&modelToAdd);
                kEndCatch(exception);
            }
        }
    }    

    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidatePartModels(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
    {
        GoPartModelClass* modelObj = *(GoPartModel*)kArrayList_At(obj->partModelList, i);

        modelObj->isValid = kFALSE;
    }
    

    return kOK; 
}

GoFx(kStatus) GoSensor_GetLivePartModel(GoSensor sensor, const kChar* name, kXml* xml, kAlloc allocator)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc tempAlloc = kObject_Alloc_(sensor); 
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kText256 configurationPath = "";

    kCheckState(GoSensor_IsReadable(sensor)); 

    kTry
    {
        kStrCopy(configurationPath, 256, "_live.job/");
        kStrCat(configurationPath, 256, name);
        kStrCat(configurationPath, 256, ".mdl/config.xml");
        kTest(GoControl_ReadFile(obj->control, configurationPath, &fileData, &fileSize, tempAlloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_GetLivePartModel-Debug.cfg", fileData, fileSize));

        kTest(kXml_LoadBytes(xml, fileData, fileSize, allocator));
    }
    kFinally
    {
        kAlloc_Free(tempAlloc, fileData); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetLivePartModel(GoSensor sensor, const kChar* name, kXml xml)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kText256 configurationPath = "";

    kCheckState(GoSensor_IsConfigurable(sensor)); 

    kTry
    {
        kTest(kXml_SaveBytes(xml, &fileData, &fileSize, alloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_SetLivePartModel-Debug.xml", fileData, fileSize));

        kStrCopy(configurationPath, 256, "_live.job/");
        kStrCat(configurationPath, 256, name);
        kStrCat(configurationPath, 256, ".mdl/config.xml");
        kTest(GoControl_WriteFile(obj->control, configurationPath, fileData, fileSize));
    }
    kFinally
    {
        kAlloc_Free_(alloc, fileData); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ReadPartModels(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize i;
    
    for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
    {
        GoPartModel model = *(GoPartModel*)kArrayList_At(obj->partModelList, i);

        kCheck(GoSensor_ReadPartModel(sensor, model));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ReadPartModel(GoSensor sensor, GoPartModel model)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kXml xml = kNULL;
    kXml root = kNULL;

    kCheck(GoSensor_GetLivePartModel(sensor, GoPartModel_Name(model), &xml, kObject_Alloc_(sensor)));
    kCheck(!kIsNull(root = kXml_Root(xml))); 

    kCheck(GoPartModel_Read(model, xml, root));

    return kOK;
}


GoFx(kStatus) GoSensor_WritePartModel(GoSensor sensor, GoPartModel model)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize i;
    kText64 tempName = "";
    kXml xml = kNULL;
    kXmlItem root = kNULL;
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kText128 modelConfigString = "";

    kCheckState(GoSensor_IsConfigurable(sensor)); 

    for (i = 0; i < GoSensor_DirectoryFileCount(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE); i++)
    {
        kCheck(GoSensor_DirectoryFileNameAt(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE, i, tempName, 64));
        strtok(tempName, ".");

        if (strcmp(tempName, GoPartModel_Name(model)) == 0)
        {
            kTry
            {
                GoPartModelClass* modelObj = GoPartModel_Cast_(model);

                kTest(kXml_Construct(&xml, kObject_Alloc(sensor)));
                kTest(kXml_AddItem(xml, kXml_Root(xml), "Configuration", &root));
                kTest(kXml_SetAttr32u(xml, root, "version", GO_PART_MODEL_CONFIG_VERSION));
                kTest(GoPartModel_Write(model, xml, root));
                kTest(kXml_SaveBytes(xml, &fileData, &fileSize, kObject_Alloc(sensor)));

                // Uncomment to save config to file (useful for debugging config problems)
                //kTest(kFile_Save("GoSensor_SetModel-Debug.xml", fileData, fileSize));

                kTest(kStrCat(modelConfigString, 256, "_live.job/"));
                kTest(kStrCat(modelConfigString, 256, GoPartModel_Name(model)));
                kTest(kStrCat(modelConfigString, 256, ".mdl/config.xml"));
                kTest(GoControl_WriteFile(obj->control, modelConfigString, fileData, fileSize));

                modelObj->isModified = kFALSE;
                modelObj->isValid = kFALSE; 
            }
            kFinally
            {
                kDestroyRef(&xml);
                kAlloc_Free_(kObject_Alloc(sensor), fileData); 
                kEndFinally(); 
            }

            return kOK;
        }
    }
        
    //see if the model exists
    //if exists, set the live model

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoSensor_PartMatchModelCount(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReadable(sensor));
    kCheck(GoSensor_SyncPartModels(sensor));

    return kArrayList_Count(obj->partModelList);
}

GoFx(GoPartModel) GoSensor_PartMatchModelAt(GoSensor sensor, kSize index)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoPartModel model = kNULL;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoSensor_SyncPartModels(sensor))
        && index < kArrayList_Count(obj->partModelList))
    {
        model = *(GoPartModel*)kArrayList_At(obj->partModelList, index);
    }   

    return model;
}

GoFx(GoPartModel) GoSensor_PartMatchModel(GoSensor sensor, const kChar* name)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoPartModel model = kNULL;
    kSize i;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoSensor_SyncPartModels(sensor)))
    {
        for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
        {
            GoPartModel partModel = *(GoPartModel*)kArrayList_At(obj->partModelList, i);

            if (strcmp(name, GoPartModel_Name(partModel)) == 0)
            {
                model = partModel;
            }
        }
    }

    return model;
}

GoFx(kStatus) GoSensor_FlushConfig(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (obj->configModified)
    {
        kCheck(GoSensor_WriteConfig(sensor)); 
        obj->configModified = kFALSE;
        obj->configValid = kFALSE; 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_CacheConfig(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->configValid)
    {
        kCheck(GoSensor_ReadConfig(sensor)); 
        obj->configModified = kFALSE;
        obj->configValid = kTRUE; 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidateConfig(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    obj->configValid = kFALSE; 
    
    return kOK; 
}

GoFx(kBool) GoSensor_ConfigValid(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    return obj->configValid; 
}

GoFx(kStatus) GoSensor_SetConfigModified(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    obj->configModified = kTRUE; 
    
    return kOK; 
}

GoFx(kBool) GoSensor_ConfigModified(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    return obj->configModified; 
}

GoFx(kStatus) GoSensor_ReadConfig(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kXml xml = kNULL;
    kXml root = kNULL;
    k32u version;

    kCheck(GoSensor_GetLiveConfig(sensor, &xml, kObject_Alloc_(sensor)));
    kCheck(!kIsNull(root = kXml_Root(xml))); 

    kCheck(kDestroyRef(&obj->configXml));
    obj->configXml = xml;
    obj->configXmlItem = root;

    kCheck(kXml_Attr32u(xml, root, "version", &version));
    if (version != GO_SENSOR_CONFIG_SCHEMA_VERSION)
    {
        return kERROR_VERSION;
    }
   
    kCheck(GoSetup_ReadConfig(obj->setup, xml, kXml_Child(xml, root, "Setup")));
    kCheck(GoTools_Read(obj->tools, xml, kXml_Child(xml, root, "Tools"), kXml_Child(xml, root, "ToolOptions"))); 
    kCheck(GoOutput_Read(obj->output, xml, kXml_Child(xml, root, "Output")));

    return kOK;
}

GoFx(kStatus) GoSensor_WriteConfig(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kXml xml = kNULL;
    kXml root = kNULL;
    kXml item = kNULL;

    kCheckState(GoSensor_IsConfigurable(sensor)); 

    kTry
    {
        kTest(kXml_Construct(&xml, kObject_Alloc_(sensor))); 

        kTest(kXml_AddItem(xml, kNULL, "Configuration", &root));
        kTest(kXml_SetAttr32u(xml, root, "version", GO_SENSOR_CONFIG_SCHEMA_VERSION));

        kTest(kXml_AddItem(xml, root, "Setup", &item));
        kTest(GoSetup_WriteConfig(obj->setup, xml, item));

        kTest(kXml_AddItem(xml, root, "Tools", &item));
        kTest(GoTools_Write(obj->tools, xml, item));

        kTest(kXml_AddItem(xml, root, "Output", &item));
        kTest(GoOutput_Write(obj->output, xml, item));

        kTest(GoUtils_XmlMerge(obj->configXml, obj->configXmlItem, xml, root));

        kTest(GoSensor_SetLiveConfig(sensor, xml));
    }
    kFinally
    {
        kCheck(kDestroyRef(&xml));
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_GetLiveConfig(GoSensor sensor, kXml* xml, kAlloc allocator)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc tempAlloc = kObject_Alloc_(sensor); 
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsReadable(sensor)); 

    kTry
    {
        kTest(GoControl_ReadFile(obj->control, GO_SENSOR_LIVE_CONFIG_NAME, &fileData, &fileSize, tempAlloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_GetLiveConfig-Debug.cfg", fileData, fileSize));

        kTest(kXml_LoadBytes(xml, fileData, fileSize, allocator));
    }
    kFinally
    {
        kAlloc_Free(tempAlloc, fileData); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetLiveConfig(GoSensor sensor, kXml xml)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsConfigurable(sensor)); 

    kTry
    {
        kTest(kXml_SaveBytes(xml, &fileData, &fileSize, alloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_SetLiveConfig-Debug.cfg", fileData, fileSize));

        kTest(GoControl_WriteFile(obj->control, GO_SENSOR_LIVE_CONFIG_NAME, fileData, fileSize));
    }
    kFinally
    {
        kAlloc_Free_(alloc, fileData); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SyncTransform(GoSensor sensor)
{    
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->isSyncTransform)
    {
        kTry
        {
            obj->isSyncTransform = kTRUE;
            kTest(GoSensor_FlushTransform(sensor)); 
            kTest(GoSensor_CacheTransform(sensor)); 
        }
        kFinally
        {
            obj->isSyncTransform = kFALSE;
            kEndFinally();
        }                
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_FlushTransform(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (obj->transformModified)
    {
        kCheck(GoSensor_WriteTransform(sensor)); 
        obj->transformModified = kFALSE;
        obj->transformValid = kFALSE; 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_CacheTransform(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->transformValid)
    {
        kCheck(GoSensor_ReadTransform(sensor)); 
        obj->transformModified = kFALSE;
        obj->transformValid = kTRUE; 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidateTransform(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    obj->transformValid = kFALSE; 
    
    return kOK; 
}

GoFx(kBool) GoSensor_TransformValid(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->transformValid; 
}

GoFx(kStatus) GoSensor_SetTransformModified(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    obj->transformModified = kTRUE; 

    return kOK; 
}

GoFx(kBool) GoSensor_TransformModified(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->transformModified; 
}

GoFx(kStatus) GoSensor_ReadTransform(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kXml xml = kNULL;
    kXml root = kNULL;
    k32u version;

    kCheck(GoSensor_GetLiveTransform(sensor, &xml, kObject_Alloc_(sensor)));
    kCheck(!kIsNull(root = kXml_Root(xml))); 

    kCheck(kXml_Attr32u(xml, root, "version", &version));
    if (version != GO_SENSOR_TRANSFORM_SCHEMA_VERSION)
    {
        return kERROR_VERSION;
    }
   
    kCheck(GoTransform_Read(obj->transform, xml, root));

    return kOK;
}

GoFx(kStatus) GoSensor_WriteTransform(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kXml xml = kNULL;
    kXml root = kNULL;

    kCheckState(GoSensor_IsConfigurable(sensor)); 

    kTry
    {
        kTest(kXml_Construct(&xml, kObject_Alloc(sensor))); 

        kTest(kXml_AddItem(xml, kNULL, "Transform", &root));
        kTest(kXml_SetAttr32u(xml, root, "version", GO_SENSOR_TRANSFORM_SCHEMA_VERSION));

        kTest(GoTransform_Write(obj->transform, xml, root));

        kTest(GoSensor_SetLiveTransform(sensor, xml));
    }
    kFinally
    {
        kDestroyRef(&xml);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_GetLiveTransform(GoSensor sensor, kXml* xml, kAlloc allocator)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc tempAlloc = kObject_Alloc_(sensor); 
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsReadable(sensor)); 

    kTry
    {
        kTest(GoControl_ReadFile(obj->control, GO_SENSOR_LIVE_TRANSFORM_NAME, &fileData, &fileSize, tempAlloc));

        // Uncomment to save transform to file (useful for debugging transform problems)
        //kTest(kFile_Save("GoSensor_GetLiveTransform-Debug.tfm", fileData, fileSize));

        kTest(kXml_LoadBytes(xml, fileData, fileSize, allocator));
    }
    kFinally
    {
        kAlloc_Free(tempAlloc, fileData); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetLiveTransform(GoSensor sensor, kXml xml)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsConfigurable(sensor)); 

    kTry
    {
        kTest(kXml_SaveBytes(xml, &fileData, &fileSize, alloc));

        // Uncomment to save transform to file (useful for debugging transform problems)
        //kTest(kFile_Save("GoSensor_SetLiveTransform-Debug.cfg", fileData, fileSize));

        kTest(GoControl_WriteFile(obj->control, GO_SENSOR_LIVE_TRANSFORM_NAME, fileData, fileSize));
    }
    kFinally
    {
        kAlloc_Free(alloc, fileData);  
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_CacheFileList(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->fileListValid)
    {
        kCheck(GoSensor_ReadFileList(sensor)); 
        obj->fileListValid = kTRUE; 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidateFileList(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    obj->fileListValid = kFALSE; 
    
    return kOK; 
}

GoFx(kBool) GoSensor_FileListValid(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->fileListValid; 
}

GoFx(kStatus) GoSensor_ReadFileList(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReadable(sensor)); 

    kCheck(GoControl_ReadFileList(obj->control, obj->fileList, kNULL)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ClearLog(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReadable(sensor)); 

    kCheck(GoControl_ClearLog(obj->control)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_CacheInfo(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (!obj->infoValid)
    {
        kCheck(GoSensor_ReadInfo(sensor)); 
        obj->infoValid = kTRUE; 
    }
    
    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidateInfo(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    obj->infoValid = kFALSE; 

    kCheck(GoSensor_LockState(sensor));
    {
        obj->sensorInfoTime = k64U_NULL; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(kBool) GoSensor_InfoValid(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->infoValid; 
}

GoFx(kStatus) GoSensor_ReadInfo(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoState state = GoSensor_KnownState(sensor);        //avoids communication
    k64u now = kTimer_Now(); 
    GoStates states;

    kCheckState(GoState_IsResponsive(state)); 
   
    kCheck(GoControl_GetSensorInfo(obj->control, obj->localSensorInfo, obj->remoteSensorInfo));
    kCheck(GoControl_GetStates(obj->control, &states));

    kCheck(GoSensor_LockState(sensor));

    kTry
    {
        kTest(GoSensorInfo_Model(obj->localSensorInfo, obj->model, kCountOf(obj->model))); 

        obj->firmwareVersion = GoSensorInfo_Firmware(obj->localSensorInfo);
        obj->user = states.loginType;
        obj->runState = GoSensorInfo_State(obj->localSensorInfo); 
        obj->alignmentState = states.alignmentState;

        if (obj->role == GO_ROLE_INVALID)
        {
            obj->role = GoSensorInfo_Role(obj->localSensorInfo);
            obj->buddyId = GoSensorInfo_HasBuddy(obj->localSensorInfo) ? GoSensorInfo_BuddyId(obj->localSensorInfo) : k32U_NULL; 
        }

        obj->sensorInfoTime = now; 
    }
    kFinally
    {
        kCheck(GoSensor_UnlockState(sensor)); 
        kEndFinally(); 
    }
    
    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidateRole(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_LockState(sensor));
    {
        obj->role = GO_ROLE_INVALID; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Refresh(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor); 

    if (state == GO_STATE_CANCELLED)
    {    
        kCheck(GoSensor_Connect(sensor)); 
    }
    else if (state == GO_STATE_UNRESPONSIVE)
    {
        kCheck(GoSensor_Disconnect(sensor)); 
    }
    else
    {
        kCheck(GoSensor_Invalidate(sensor)); 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_CheckHealth(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
                
    if (obj->healthCheckEnabled)
    {
        kSize historyMask = kCountOf(obj->healthHistory) - 1; 
        k64u diff = obj->healthMsgCount - obj->previousHealthMsgCount; 

        obj->healthHistory[obj->healthCheckCount & historyMask] = (diff > 0);        
        obj->healthCheckCount++; 
        
        obj->previousHealthMsgCount = obj->healthMsgCount; 
    }

    return kOK; 
}

GoFx(GoState) GoSensor_KnownState(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoState state; 
       
    kCheck(GoSensor_LockState(sensor)); 
    {
        if (obj->isCancelled)
        {
            state = GO_STATE_CANCELLED; 
        }
        else if (obj->isResetting)
        {
            state = GO_STATE_RESETTING; 
        }
        else if (!obj->isConnected)
        {
            state = GoSensor_IsDiscoveryOnline(sensor) ? GO_STATE_ONLINE : GO_STATE_OFFLINE; 
        }
        else if (!GoSensor_IsDiscoveryOnline(sensor) && !GoSensor_IsHealthOnline(sensor))
        {
            state = GO_STATE_UNRESPONSIVE; 
        }
        else if (!obj->isCompatible)
        {
            state = GO_STATE_INCOMPATIBLE; 
        }
        else if ((obj->sensorInfoTime != k64U_NULL) && (obj->role == GO_ROLE_BUDDY))
        {
            state = GO_STATE_BUSY; 
        }
        else if (obj->sensorInfoTime != k64U_NULL)
        {
            state = obj->runState; 
        }
        else
        {
            state = GO_STATE_CONNECTED; 
        }
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return state; 
}

GoFx(kBool) GoSensor_IsDiscoveryOnline(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize discoveryMask = kCountOf(obj->discoveryHistory) - 1;            
    kSize discoveryTotal = (kSize) obj->discoveryCount; 
    kSize discoveryCount = (kSize) kMin_(obj->discoveryCount, GO_SENSOR_DISCOVERY_CHECK_COUNT); 
    kBool discoveryOnline = (discoveryCount < GO_SENSOR_DISCOVERY_CHECK_COUNT);
    kSize i; 

    for (i = 0; i < discoveryCount; ++i)
    {
        discoveryOnline = discoveryOnline || obj->discoveryHistory[(discoveryTotal-i-1) & discoveryMask]; 
    }

    return discoveryOnline; 
}

GoFx(kBool) GoSensor_IsHealthOnline(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize healthMask = kCountOf(obj->healthHistory) - 1;            
    kSize healthTotal = (kSize) obj->healthCheckCount; 
    kSize healthCount = (kSize) kMin_(obj->healthCheckCount, GO_SENSOR_HEALTH_CHECK_COUNT); 
    kBool healthOnline = (healthCount < GO_SENSOR_HEALTH_CHECK_COUNT);
    kSize i; 

    for (i = 0; i < healthCount; ++i)
    {
        healthOnline = healthOnline || obj->healthHistory[(healthTotal-i-1) & healthMask]; 
    }

    return healthOnline; 
}

GoFx(kBool) GoSensor_IsConnected(GoSensor sensor)
{
    GoState state = GoSensor_KnownState(sensor); 

    return GoState_IsConnected(state); 
}

GoFx(kBool) GoSensor_IsResponsive(GoSensor sensor)
{
    GoState state = GoSensor_KnownState(sensor); 

    return GoState_IsResponsive(state); 
}

GoFx(kBool) GoSensor_IsReadable(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor); 

    return GoState_IsReadable(state); 
}

GoFx(kBool) GoSensor_IsConfigurable(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor); 

    return GoState_IsConfigurable(state); 
}

GoFx(kBool) GoSensor_IsReady(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor); 

    return (state == GO_STATE_READY); 
}

GoFx(kBool) GoSensor_IsRunning(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor); 

    return (state == GO_STATE_RUNNING); 
}

GoFx(kBool) GoSensor_IsNormal(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor); 

    return GoState_IsNormal(state); 
}

GoFx(kBool) GoSensor_IsCancelled(GoSensor sensor)
{
    GoState state = GoSensor_KnownState(sensor); 

    return (state == GO_STATE_CANCELLED); 
}

GoFx(kBool) GoSensor_ConsistencyIntervalElapsed(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    k64u now = kTimer_Now(); 
    
    return (obj->sensorInfoTime != k64U_NULL) && ((now - obj->sensorInfoTime) > GO_SENSOR_CONSISTENCY_INTERVAL); 
}

GoFx(kStatus) GoSensor_Cancel(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isCancelled = kTRUE; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_OnCancelQuery(GoSensor sensor, kObject sender, kPointer args)
{
    GoState state = GoSensor_KnownState(sensor);        //use known state; avoids communication 
  
    return GoState_IsResponsive(state) ? kOK : kERROR_ABORT; 
}

GoFx(kStatus) GoSensor_LockState(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    return GoSystem_LockState(obj->system); 
}

GoFx(kStatus) GoSensor_UnlockState(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    return GoSystem_UnlockState(obj->system); 
}

GoFx(kStatus) GoSensor_BeginReset(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isResetting = kTRUE; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    kCheck(kPeriodic_Start(obj->resetTimer, GO_SYSTEM_RESET_HOLD_INTERVAL, GoSensor_OnResetHoldComplete, sensor));

    return kOK; 
}

GoFx(kStatus) GoSensor_OnResetHoldComplete(GoSensor sensor, kPeriodic timer)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_LockState(sensor)); 
    {
        obj->isResetting = kFALSE; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_WaitForReboot(GoSensor sensor, k64u timeout)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(kTimer_Start(obj->timer, timeout)); 

    while ((GoSensor_State(sensor) != GO_STATE_ONLINE) && !kTimer_IsExpired(obj->timer))
    {
        kCheck(kThread_Sleep(GO_SYSTEM_RESET_QUERY_INTERVAL)); 
    }

    return (GoSensor_State(sensor) == GO_STATE_ONLINE) ? kOK : kERROR_TIMEOUT; 
}

GoFx(kStatus) GoSensor_WaitForReconnect(GoSensor sensor, k64u timeout)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoState state; 

    kCheck(kTimer_Start(obj->timer, timeout)); 

    //wait for reconnection
    while (!kSuccess(GoSensor_Connect(sensor)) && !kTimer_IsExpired(obj->timer))
    {
        kCheck(kThread_Sleep(GO_SYSTEM_RESET_QUERY_INTERVAL)); 
    }

    kCheck(kTimer_Start(obj->timer, kMin_(kTimer_Remaining(obj->timer), GO_SYSTEM_RESET_INCOMPLETE_TIMEOUT))); 

    //wait for incomplete status to resolve (main booted, but buddy not yet detected)
    state = GoSensor_State(sensor); 

    while (((state == GO_STATE_INCOMPLETE) || (state == GO_STATE_INCONSISTENT)) && !kTimer_IsExpired(obj->timer))
    {
        kCheck(kThread_Sleep(GO_SYSTEM_RESET_QUERY_INTERVAL)); 
        
        kCheck(GoSensor_Refresh(sensor)); 
        
        state = GoSensor_State(sensor); 
    }

    return GoSensor_IsConnected(sensor) ? kOK : kERROR_TIMEOUT; 
}

GoFx(kStatus) GoSensor_AddBuddy(GoSensor sensor, GoSensor buddy)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsConfigurable(sensor)); 
    kCheckState(GoSensor_IsConfigurable(buddy)); 
    kCheckState(!GoSensor_HasBuddy(sensor)); 
    kCheckState(!GoSensor_HasBuddy(buddy)); 

    kCheck(GoSensor_Flush(sensor)); 
    kCheck(GoSensor_Flush(buddy)); 

    kCheck(GoControl_ChangeBuddy(obj->control, kTRUE, GoSensor_Id(buddy))); 

    kCheck(GoSensor_Invalidate(sensor)); 
    kCheck(GoSensor_Invalidate(buddy)); 
    kCheck(GoSensor_SetConnected(buddy, kFALSE, kTRUE));

    return kOK; 
}

GoFx(kStatus) GoSensor_RemoveBuddy(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoSensor buddy = GoSensor_Buddy(sensor); 
 
    kCheckState(GoSensor_IsConfigurable(sensor)); 
    kCheckState(GoSensor_Role(sensor) == GO_ROLE_MAIN); 
    
    if (GoSensor_HasBuddy(sensor))
    {
        kCheck(GoSensor_Flush(sensor)); 

        kCheck(GoControl_ChangeBuddy(obj->control, kFALSE, GoSensor_BuddyId(sensor))); 

        kCheck(GoSensor_Invalidate(sensor)); 

        if (!kIsNull(buddy))
        {
            kCheck(GoSensor_Invalidate(buddy)); 
        }
    }

    return kOK; 
}

GoFx(kBool) GoSensor_HasBuddy(GoSensor sensor)
{
    if (GoSensor_Role(sensor) == GO_ROLE_MAIN)
    {
        return (GoSensor_BuddyId(sensor) != k32U_NULL); 
    }
    else
    {
        return kFALSE;  
    }
}

GoFx(GoSensor) GoSensor_Buddy(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoSensor buddy = kNULL; 

    if (GoSensor_HasBuddy(sensor))
    {
        k32u buddyId = GoSensor_BuddyId(sensor); 

        if (kSuccess(GoSystem_FindSensorById(obj->system, buddyId, &buddy)))
        {
            return buddy; 
        }
    }
            
    return kNULL; 
}

GoFx(k32u) GoSensor_BuddyId(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    k32u id; 

    kCheck(GoSensor_CacheInfo(sensor)); 

    kCheck(GoSensor_LockState(sensor)); 
    {
        id = obj->buddyId; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return id; 
}

GoFx(kStatus) GoSensor_EndAlign(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoControl_EndAlignment(obj->control)); 

    kCheck(GoSensor_Invalidate(sensor));

    return kOK; 
}

GoFx(kStatus) GoSensor_BeginAlign(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor));  

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginAlignment(obj->control)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Align(GoSensor sensor)
{
    kCheck(GoSensor_BeginAlign(sensor)); 
    kCheck(GoSensor_EndAlign(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ClearAlignment(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoControl_ClearAlignment(obj->control)); 

    return kOK;
}

GoFx(kStatus) GoSensor_SetAlignmentReference(GoSensor sensor, GoAlignmentRef reference)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_SetAlignmentReference(obj->control, reference)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_AlignmentReference(GoSensor sensor, GoAlignmentRef* reference)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReadable(sensor));
    kCheck(GoControl_GetAlignmentReference(obj->control, reference));

    return kOK;
}

GoFx(kStatus) GoSensor_EndExposureAutoSet(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoControl_EndExposureAutoSet(obj->control)); 

    kCheck(GoSensor_InvalidateInfo(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_BeginExposureAutoSet(GoSensor sensor, GoRole role)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(GoSensor_Refresh(sensor));
    kCheckState(GoSensor_IsReady(sensor));  

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginExposureAutoSet(obj->control, role)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ExposureAutoSet(GoSensor sensor, GoRole role)
{
    kCheck(GoSensor_BeginExposureAutoSet(sensor, role)); 
    kCheck(GoSensor_EndExposureAutoSet(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_BeginStart(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor));  

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginStart(obj->control)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_EndStart(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoControl_EndStart(obj->control)); 

    kCheck(GoSensor_InvalidateInfo(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Start(GoSensor sensor)
{
    kCheck(GoSensor_BeginStart(sensor)); 
    kCheck(GoSensor_EndStart(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_BeginScheduledStart(GoSensor sensor, k64s value)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor));  

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginScheduledStart(obj->control, value)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_EndScheduledStart(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoControl_EndScheduledStart(obj->control)); 

    kCheck(GoSensor_InvalidateInfo(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ScheduledStart(GoSensor sensor, k64s value)
{
    kCheck(GoSensor_BeginScheduledStart(sensor, value)); 
    kCheck(GoSensor_EndScheduledStart(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_BeginStop(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_BeginStop(obj->control)); 

    return kOK; 
} 

GoFx(kStatus) GoSensor_EndStop(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(GoControl_EndStop(obj->control)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Stop(GoSensor sensor)
{
    kCheck(GoSensor_BeginStop(sensor)); 
    kCheck(GoSensor_EndStop(sensor)); 
   
    return kOK; 
}

GoFx(GoState) GoSensor_State(GoSensor sensor)
{    
    //return value intentionally not checked; success not required
    GoSensor_CacheInfo(sensor); 
   
    return GoSensor_KnownState(sensor); 
}

GoFx(kStatus) GoSensor_States(GoSensor sensor, GoStates* states)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoMode mode = GO_MODE_UNKNOWN; 

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetStates(obj->control, states)))
    {
        return kOK; 
    }

    return kERROR;
}

GoFx(GoRole) GoSensor_Role(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(GoSensor_CacheInfo(sensor)); 

    return obj->role; 
}

GoFx(GoMode) GoSensor_ScanMode(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoMode mode = GO_MODE_UNKNOWN; 

    kCheck(GoSensor_Flush(sensor));

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetScanMode(obj->control, &mode)))
    {
        return mode; 
    }
    
    return GO_MODE_UNKNOWN; 
}

GoFx(kStatus) GoSensor_EmitDigital(GoSensor sensor, k16u index, k64s target, k8u value)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsRunning(sensor)); 

    kCheck(GoControl_ScheduleDigital(obj->control, index, target, value)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_EmitAnalog(GoSensor sensor, k16u index, k64s target, k32s value)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsRunning(sensor)); 

    kCheck(GoControl_ScheduleAnalog(obj->control, index, target, value)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Reset(GoSensor sensor, kBool wait)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 
    kCheck(GoSensor_BeginReset(sensor)); 
    kCheck(GoControl_Reset(obj->control)); 
    kCheck(GoSensor_Disconnect(sensor)); 

    if (wait)
    {
        kCheck(GoSensor_WaitForReconnect(sensor, GO_SYSTEM_RESET_TIMEOUT)); 
    }
   
    return kOK; 
}

GoFx(k32u) GoSensor_Id(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    return obj->deviceId; 
}

GoFx(kStatus) GoSensor_Model(GoSensor sensor, kChar* model, kSize capacity)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(GoSensor_CacheInfo(sensor)); 

    return kStrCopy(model, capacity, obj->model); 
}

GoFx(GoUser) GoSensor_User(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    if (!kSuccess(GoSensor_CacheInfo(sensor)))
    {
        return GO_USER_NONE; 
    }

    return obj->user; 
}

GoFx(kVersion) GoSensor_ProtocolVersion(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
       
    if (!GoSensor_IsConnected(sensor))
    {
        return 0; 
    }

    return GoControl_ProtocolVersion(obj->control); 
}

GoFx(kVersion) GoSensor_FirmwareVersion(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    if (!kSuccess(GoSensor_CacheInfo(sensor)))
    {
        return 0; 
    }

    return obj->firmwareVersion; 
}

GoFx(GoAlignmentState) GoSensor_AlignmentState(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    if (!kSuccess(GoSensor_CacheInfo(sensor)))
    {
        return GO_ALIGNMENT_STATE_NOT_ALIGNED; 
    }

    return obj->alignmentState; 
}

GoFx(kStatus) GoSensor_Timestamp(GoSensor sensor, k64u* time)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsNormal(sensor)); 

    kCheck(GoControl_GetTimestamp(obj->control, time)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Encoder(GoSensor sensor, k64s* encoder)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsNormal(sensor)); 

    kCheck(GoControl_GetEncoder(obj->control, encoder)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Trigger(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsRunning(sensor)); 

    kCheck(GoControl_Trigger(obj->control));

    return kOK; 
}

GoFx(kSize) GoSensor_FileCount(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    if (!kSuccess(GoSensor_CacheFileList(sensor)))
    {
        return 0; 
    }

    return kArrayList_Count(obj->fileList); 
}

GoFx(kStatus) GoSensor_FileNameAt(GoSensor sensor, kSize index, kChar* name, kSize capacity)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(GoSensor_CacheFileList(sensor)); 

    kCheckArgs(index < kArrayList_Count(obj->fileList)); 

    kCheck(kStrCopy(name, capacity, kArrayList_At(obj->fileList, index))); 

    return kOK; 
}

GoFx(kStatus) GoSensor_UploadFile(GoSensor sensor, const kChar* sourcePath, const kChar* destName)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* data = kNULL; 
    kSize size = 0; 

    kCheckState(GoSensor_IsNormal(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kTry
    {
        kTest(kFile_Load(sourcePath, &data, &size, alloc)); 

        kTest(GoControl_WriteFile(obj->control, destName, data, size));  

        kTest(GoSensor_Invalidate(sensor)); 
    }
    kFinally
    {
        kAlloc_Free_(alloc, data); 
        kEndFinally(); 
    }    

    return kOK; 
}

GoFx(kStatus) GoSensor_DownloadFile(GoSensor sensor, const kChar* sourceName, const kChar* destPath)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* data = kNULL; 
    kSize size = 0; 
    kText32 extension;

    kCheckState(GoSensor_IsNormal(sensor)); 

    kTry
    {
        kTest(kPath_Extension(sourceName, extension, kCountOf(extension)));
        
        if (kStrCompare(extension, "rec") == 0)
        {
            kTest(GoControl_ReadFileStreamed(obj->control, sourceName, destPath));  
        }
        else
        {
            kTest(GoControl_ReadFile(obj->control, sourceName, &data, &size, alloc));  
            kTest(kFile_Save(destPath, data, size)); 
        }
    }
    kFinally
    {
        kAlloc_Free_(alloc, data); 
        kEndFinally(); 
    }    

    return kOK; 
} 

GoFx(kStatus) GoSensor_CopyFile(GoSensor sensor, const kChar* sourceName, const kChar* destName)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsNormal(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 
    kCheck(GoControl_CopyFile(obj->control, sourceName, destName));  
    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_DeleteFile(GoSensor sensor, const kChar* name)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsNormal(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 
    kCheck(GoControl_DeleteFile(obj->control, name));  
    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kBool) GoSensor_FileExists(GoSensor sensor, const kChar* name)
{
    kSize fileCount = GoSensor_FileCount(sensor); 
    kText128 fileName; 
    kSize i; 
    
    for (i = 0; i < fileCount; ++i)
    {
        kCheck(GoSensor_FileNameAt(sensor, i, fileName, kCountOf(fileName))); 
       
        if (kStrEquals(fileName, name))
        {
            return kTRUE; 
        }
    }

    return kFALSE; 
}

GoFx(kStatus) GoSensor_SetDefaultJob(GoSensor sensor, const kChar* fileName)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_SetDefaultJob(obj->control, fileName)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_DefaultJob(GoSensor sensor, kChar* fileName, kSize capacity)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_GetDefaultJob(obj->control, fileName, capacity)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_LoadedJob(GoSensor sensor, kChar* fileName, kSize capacity, kBool* changed)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_GetLoadedJob(obj->control, fileName, capacity, changed)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_LogIn(GoSensor sensor, GoUser user, const kChar* password)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_Login(obj->control, user, password)); 

    kCheck(GoSensor_InvalidateInfo(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ChangePassword(GoSensor sensor, GoUser user, const kChar* password)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsResponsive(sensor)); 

    kCheck(GoControl_ChangePassword(obj->control, user, password)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_Upgrade(GoSensor sensor, const kChar* sourcePath, GoUpgradeFx callback, kPointer receiver)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* data = kNULL; 
    kSize size = 0; 
    kBool complete = kFALSE; 
    kBool succeeded = kFALSE; 
    k32s progress; 
    GoUpgradeFxArgs args; 

    kCheckState(GoSensor_IsResponsive(sensor) || GoControl_IsConnected(obj->control));
    
    //success optional
    GoSensor_Flush(sensor);

    //disable health; health service may not be reliable during upgrade
    kCheck(GoSensor_EnableHealth(sensor, kFALSE)); 

    kTry
    {
        kTest(kTimer_Start(obj->timer, GO_SENSOR_UPGRADE_TIMEOUT)); 

        kTest(kFile_Load(sourcePath, &data, &size, alloc)); 

        kTest(GoControl_BeginUpgrade(obj->control, data, size)); 

        do
        {
            kTest(kThread_Sleep(GO_SYSTEM_QUIT_QUERY_INTERVAL)); 

            kTest(GoControl_GetUpgradeStatus(obj->control, &complete, &succeeded, &progress)); 
            
            if (callback)
            {
                args.progress = (k64f) progress;  
                kTest(callback(receiver, sensor, &args)); 
            }

            kTestState(GoSensor_KnownState(sensor) != GO_STATE_CANCELLED); 
        }
        while (!complete && !kTimer_IsExpired(obj->timer));

        if (!complete)  kThrow(kERROR_TIMEOUT); 
        if (!succeeded) kThrow(kERROR);       
    }
    kFinally
    {
        kAlloc_Free_(alloc, data); 
        kEndFinally(); 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_Backup(GoSensor sensor, const kChar* destPath)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* data = kNULL; 
    kSize size = 0; 
    
    kCheckState(GoSensor_IsReady(sensor));

    kTry
    {
        kTest(GoControl_Backup(obj->control, &data, &size, alloc));  
        kTest(kFile_Save(destPath, data, size)); 
    }
    kFinally
    {
        kAlloc_Free_(alloc, data); 
        kEndFinally(); 
    }    

    return kOK; 
}

GoFx(kStatus) GoSensor_Restore(GoSensor sensor, const kChar* sourcePath)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc alloc = kObject_Alloc_(sensor); 
    kByte* data = kNULL; 
    kSize size = 0; 

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Invalidate(sensor));

    kTry
    {
        kTest(kFile_Load(sourcePath, &data, &size, alloc)); 
        kTest(GoControl_Restore(obj->control, data, size)); 
    }
    kFinally
    {
        kAlloc_Free_(alloc, data); 
        kEndFinally(); 
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_RestoreDefaults(GoSensor sensor, kBool restoreAddress)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheckState(GoSensor_IsConnected(sensor)); 

    kCheck(GoSensor_Invalidate(sensor));

    kCheck(GoControl_RestoreFactory(obj->control, restoreAddress)); 

    kCheck(GoSensor_CacheInfo(sensor));
    kCheck(GoSensor_ReadConfig(sensor));
    kCheck(GoSensor_ReadTransform(sensor));

    return kOK; 
}

GoFx(kStatus) GoSensor_EnableData(GoSensor sensor, kBool enable)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsNormal(sensor)); 
    
    kCheck(kDestroyRef(&obj->data)); 
    
    if (enable)
    {
        kCheck(GoReceiver_Construct(&obj->data, kObject_Alloc_(sensor))); 
        kCheck(GoReceiver_SetBuffers(obj->data, GO_SENSOR_DATA_SOCKET_BUFFER, GO_SENSOR_DATA_STREAM_BUFFER)); 
        kCheck(GoReceiver_SetCancelHandler(obj->data, GoSensor_OnCancelQuery, sensor)); 
        kCheck(GoReceiver_SetMessageHandler(obj->data, GoSensor_OnData, sensor)); 

        kCheck(GoReceiver_Open(obj->data, obj->address.address, GO_SENSOR_DATA_PORT)); 
    }

    return kOK; 
}

GoFx(kBool) GoSensor_DataEnabled(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);    
    return !kIsNull(obj->data); 
}

GoFx(kStatus) GoSensor_OnData(GoSensor sensor, GoReceiver receiver, kSerializer reader)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc msgAlloc = kAlloc_Default_();    //use the default system allocator for now; revisit later (performance)    
    GoDataSet dataSet = kNULL; 
    kStatus status = kNULL; 

    kTry
    {        
        kTest(kSerializer_ReadObject_(reader, &dataSet, msgAlloc));         
        kTest(GoDataSet_SetSenderId_(dataSet, obj->deviceId)); 
    }
    kCatch(&status)
    {
        kObject_Dispose(dataSet); 
        kEndCatch(status); 
    }

    if (obj->onDataSet == kNULL)
    {
        kCheck(GoSystem_OnData(obj->system, sensor, dataSet)); 
    }
    else
    {
        kCheck(obj->onDataSet(obj->onDataSetContext, sensor, dataSet));
    }

    return kOK; 
}

GoFx(kStatus) GoSensor_EnableHealth(GoSensor sensor, kBool enable)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    
    kCheck(kDestroyRef(&obj->health)); 
    
    if (enable)
    {
        kCheck(GoReceiver_Construct(&obj->health, kObject_Alloc_(sensor))); 
        kCheck(GoReceiver_SetBuffers(obj->health, GO_SENSOR_HEALTH_SOCKET_BUFFER, GO_SENSOR_HEALTH_STREAM_BUFFER)); 
        kCheck(GoReceiver_SetCancelHandler(obj->health, GoSensor_OnCancelQuery, sensor)); 
        kCheck(GoReceiver_SetMessageHandler(obj->health, GoSensor_OnHealth, sensor)); 

        kCheck(GoReceiver_Open(obj->health, obj->address.address, GO_SENSOR_HEALTH_PORT)); 
    }

    kCheck(GoSensor_LockState(sensor)); 
    {
        obj->healthCheckEnabled = enable; 
        obj->previousHealthMsgCount = 0; 
        obj->healthCheckCount = 0; 
        obj->healthMsgCount = 0;
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_OnHealth(GoSensor sensor, GoReceiver receiver, kSerializer reader)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kAlloc msgAlloc = kAlloc_Default_();
    GoDataSet healthSet = kNULL; 
    kStatus status = kNULL; 

    kTry
    {        
        kTest(kSerializer_ReadObject(reader, &healthSet, msgAlloc));         
        kTest(GoDataSet_SetSenderId_(healthSet, obj->deviceId)); 
        
        kTest(GoSensor_UpdateHealthInfo(sensor, healthSet)); 
    }
    kCatch(&status)
    {
        kObject_Dispose(healthSet); 
        kEndCatch(status); 
    }

    kCheck(GoSystem_OnHealth(obj->system, sensor, healthSet)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_UpdateHealthInfo(GoSensor sensor, GoDataSet healthSet)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kSize i, j; 

    kCheck(GoSensor_LockState(sensor)); 
    {
        kBool consistencyIntervalElapsed = GoSensor_ConsistencyIntervalElapsed(sensor);
        kSize deviceCount = GoDataSet_Count(healthSet); 

        for (i = 0; i < deviceCount; ++i)
        {
            GoHealthMsg health = GoDataSet_At(healthSet, i); 
            kSize indicatorCount = GoHealthMsg_Count(health); 
            k32u source = GoDataSet_SenderId(healthSet); 

            if (source == GoSensor_Id(sensor))
            {
                for (j = 0; j < indicatorCount; ++j)
                {
                    const GoIndicator* indicator = GoHealthMsg_At(health, j); 

                    switch (indicator->id)
                    {
                    case GO_HEALTH_STATE:
                        {             
                            if (consistencyIntervalElapsed)
                            {
                                switch (indicator->value)
                                {
                                case -1:    obj->runState = GO_STATE_INCOMPLETE;       break;
                                case 0:     obj->runState = GO_STATE_READY;            break;
                                case 1:     obj->runState = GO_STATE_RUNNING;          break;
                                default:    obj->runState = GO_STATE_CONNECTED;        break;
                                }
                            }
                            break;
                        }

                    default:
                        break;
                    }
                    
                }              
            }
        }

        obj->healthMsgCount++; 
    }
    kCheck(GoSensor_UnlockState(sensor)); 

    return kOK; 
}

GoFx(GoSetup) GoSensor_Setup(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->setup; 
}

GoFx(GoTool) GoSensor_Tools(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->tools; 
}

GoFx(GoOutput) GoSensor_Output(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->output; 
}

GoFx(kStatus) GoSensor_EnableRecording(GoSensor sensor, kBool enable)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_SetRecordingEnabled(obj->control, enable)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kBool) GoSensor_RecordingEnabled(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kBool enabled = kFALSE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetRecordingEnabled(obj->control, &enabled)))
    {
        return enabled; 
    }

    return enabled; 
}

GoFx(kStatus) GoSensor_SetInputSource(GoSensor sensor, GoInputSource source)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_SetInputSource(obj->control, source)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(GoInputSource) GoSensor_InputSource(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    GoInputSource source = GO_INPUT_SOURCE_LIVE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetInputSource(obj->control, &source)))
    {
        return source; 
    }

    return source; 
}

GoFx(kStatus) GoSensor_Simulate(GoSensor sensor, kBool* isBufferValid)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_Simulate(obj->control, isBufferValid)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_PlaybackStep(GoSensor sensor, GoSeekDirection direction)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_PlaybackStep(obj->control, direction)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_PlaybackSeek(GoSensor sensor, kSize position)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_PlaybackSeek(obj->control, position)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_PlaybackPosition(GoSensor sensor, kSize* position, kSize* count)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_PlaybackPosition(obj->control, position, count)))
    {
        return kOK; 
    }

    return kERROR; 
}

GoFx(kStatus) GoSensor_ClearReplayData(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_ClearReplayData(obj->control)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ClearMeasurementStats(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_ClearMeasurementStats(obj->control)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_ExportBitmap(GoSensor sensor, 
                                    GoReplayExportSourceType type, 
                                    GoDataSource source,
                                    const kChar* dstFileName)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_ExportBitmap(obj->control, 
                                 type,
                                 source,
                                 dstFileName)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}


GoFx(kStatus) GoSensor_ExportCsv(GoSensor sensor, const kChar* dstFileName)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_ExportCsv(obj->control, dstFileName)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(GoFamily) GoSensor_Family(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);
    const char* pos = kNULL;
    kString tempString = kNULL;
    kArrayList tokens = kNULL;
    GoFamily returnType;

    kTry
    {
        kTest(GoSensor_CacheInfo(sensor)); 
        kTest(kString_Construct(&tempString, obj->model, kObject_Alloc(sensor)));

        kTest(kString_Split(tempString, "-", &tokens, kObject_Alloc(sensor)));

        if(kArrayList_Count(tokens) == 0)
            returnType = GO_FAMILY_2000;

        switch(atoi(kString_Chars(kArrayList_As_(tokens, 0, kString)) + 2) / 1000)
        {
        case 1:     returnType = GO_FAMILY_1000; break;
        case 2:     returnType = GO_FAMILY_2000; break;
        case 3:     returnType = GO_FAMILY_3000; break;
        default:    returnType = GO_FAMILY_2000; break;
        }
    }
    kFinally
    {
        kDestroyRef(&tempString);
        kDisposeRef(&tokens);
        kEndFinally();
    }

    return returnType;
}

GoFx(GoTransform) GoSensor_Transform(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);

    return obj->transform;
}


GoFx(kStatus) GoSensor_EnableAutoStart(GoSensor sensor, kBool enable)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_SetAutoStartEnabled(obj->control, enable)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kBool) GoSensor_AutoStartEnabled(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 
    kBool enabled = kFALSE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetAutoStartEnabled(obj->control, &enabled)))
    {
        return enabled; 
    }

    return enabled; 
}

GoFx(kSize) GoSensor_RemoteInfoCount(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);

    return kArrayList_Count(obj->remoteSensorInfo);
}

GoFx(GoSensorInfo) GoSensor_RemoteInfoAt(GoSensor sensor, kSize index)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);

    kAssert(index < kArrayList_Count(obj->remoteSensorInfo));

    return kArrayList_As_(obj->remoteSensorInfo, index, GoSensorInfo);
}

GoFx(kStatus) GoSensor_PartMatchCreateModel(GoSensor sensor, const kChar* name)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_CreateModel(obj->control, name)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_PartMatchDetectModelEdges(GoSensor sensor, const kChar* name, k16u sensitivity)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_DetectModelEdges(obj->control, name, sensitivity)); 

    kCheck(GoSensor_Invalidate(sensor)); 

    return kOK; 
}

GoFx(kStatus) GoSensor_CacheDirectoryList(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);

    if (!obj->isDirectoryListValid)
    {
        kCheck(GoSensor_ListDirectory(sensor, extensionFilter, path, isRecursive, obj->directoryList)); 
        obj->isDirectoryListValid = kTRUE; 
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ListDirectory(GoSensor sensor, const kChar* extension, const kChar* root, kBool isRecursive, kArrayList fileList)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReadable(sensor));

    kCheck(GoControl_ListDirectory(obj->control, extension, root, isRecursive, fileList));

    return kOK;
}

GoFx(kSize) GoSensor_DirectoryFileCount(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_CacheDirectoryList(sensor, extensionFilter, path, isRecursive));

    return kArrayList_Count(obj->directoryList); 
}

GoFx(kStatus) GoSensor_DirectoryFileNameAt(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive, kSize index, kChar* fileName, kSize capacity)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheck(GoSensor_CacheDirectoryList(sensor, extensionFilter, path, isRecursive));

    kCheckArgs(index < kArrayList_Count(obj->directoryList)); 

    kCheck(kStrCopy(fileName, capacity, kArrayList_At(obj->directoryList, index))); 

    return kOK; 
}

GoFx(kStatus) GoSensor_InvalidateDirectoryList(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    obj->isDirectoryListValid = kFALSE; 

    return kOK; 
}

GoFx(kBool) GoSensor_DirectoryListValid(GoSensor sensor)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    return obj->isDirectoryListValid; 
}


GoFx(kStatus) GoSensor_SetDataHandler(GoSensor sensor, GoSensorDataSetFx function, kPointer context)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor);

    obj->onDataSet = function;
    obj->onDataSetContext = context;

    return kOK;
}

GoFx(kStatus) GoSensor_AddTool(GoSensor sensor, const kChar* type, const kChar* name)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_AddTool(obj->control, type, name)); 

    kCheck(GoSensor_Invalidate(sensor)); 
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK; 
}

GoFx(kStatus) GoSensor_AddMeasurement(GoSensor sensor, kSize index, const kChar* type)
{
    GoSensorClass* obj = GoSensor_Cast_(sensor); 

    kCheckState(GoSensor_IsReady(sensor)); 

    kCheck(GoSensor_Flush(sensor)); 

    kCheck(GoControl_AddMeasurement(obj->control, index, type)); 

    kCheck(GoSensor_Invalidate(sensor));
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK; 
}