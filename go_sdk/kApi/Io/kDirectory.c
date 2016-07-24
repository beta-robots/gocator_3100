/** 
 * @file    Directory.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kDirectory.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kPath.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>
#include <stdio.h>

kBeginStaticClass(k, kDirectory)
kEndStaticClass()

kFx(kStatus) kDirectory_InitStatic()
{
    kApiDirectoryFx handlers = { kNULL }; 

    handlers.create = kDirectory_CreateImpl; 
    handlers.exists = kDirectory_ExistsImpl; 
    handlers.move= kDirectory_MoveImpl; 
    handlers.del= kDirectory_DeleteImpl; 
    handlers.enumerate = kDirectory_EnumerateImpl; 
    handlers.setCurrent = kDirectory_SetCurrentImpl; 
    handlers.current = kDirectory_CurrentImpl; 
    handlers.appDirectory = kDirectory_ApplicationImpl; 
    handlers.tempDirectory = kDirectory_TempImpl; 
    handlers.appConfigDirectory = kDirectory_AppConfigImpl; 
    handlers.appDataDirectory = kDirectory_AppDataImpl; 
    handlers.pluginDirectory = kDirectory_PluginImpl; 
    handlers.toVirtual = kDirectory_ToVirtualImpl; 
    handlers.fromVirtual = kDirectory_FromVirtualImpl; 

    //respect handlers that have already been installed    
    if (kApiLib_HasDirectoryHandlers_())
    {
        kCheck(kOverrideFunctions(&handlers, sizeof(handlers), kApiLib_DirectoryHandlers_())); 
    }

    kCheck(kApiLib_SetDirectoryHandlers(&handlers)); 

    return kOK;
}

kFx(kStatus) kDirectory_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kDirectory_Create(const kChar* directory)
{
    kChar parent[kPATH_MAX]; 
    
    //create any missing parent directories
    while (kSuccess(kPath_Directory(directory, parent, kCountOf(parent))) && !kDirectory_Exists(parent))
    {
        kCheck(kDirectory_Create(parent)); 
    }

    //create the requested directory        
    return kApiLib_DirectoryHandlers_()->create(directory); 
}

kFx(kBool) kDirectory_Exists(const kChar* directory)
{    
    return kApiLib_DirectoryHandlers_()->exists(directory); 
}

kFx(kStatus) kDirectory_Copy(const kChar* source, const kChar* destination)
{
    kSize i;
    kArrayList files = kNULL; 
    kArrayList directories = kNULL; 
    kString srcItem = kNULL; 
    kChar srcPath[kPATH_MAX]; 
    kChar destPath[kPATH_MAX]; 

    if (kStrEquals(source, destination))
    {
        return kOK; 
    }

    kCheckArgs(kDirectory_Exists(source)); 

    //create destination root folder
    kCheck(kDirectory_Create(destination)); 

    kTry
    {
        //copy files
        kTest(kDirectory_Files(source, &files)); 

        for (i = 0; i < kArrayList_Count(files); ++i)
        {
            kTest(kArrayList_Item(files, i, &srcItem)); 
            kTest(kPath_Combine(source, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kPath_Combine(destination, kString_Chars(srcItem), destPath, kCountOf(destPath))); 
            kTest(kFile_Copy(srcPath, destPath));  
        }

        //copy directories
        kTest(kDirectory_Directories(source, &directories)); 

        for (i = 0; i < kArrayList_Count(directories); ++i)
        {
            kTest(kArrayList_Item(directories, i, &srcItem)); 
            kTest(kPath_Combine(source, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kPath_Combine(destination, kString_Chars(srcItem), destPath, kCountOf(destPath))); 
            kTest(kDirectory_Copy(srcPath, destPath));  
        }
    }
    kFinally
    {
        kCheck(kObject_Dispose(files)); 
        kCheck(kObject_Dispose(directories)); 
        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_Move(const kChar* source, const kChar* destination)
{
    return kApiLib_DirectoryHandlers_()->move(source, destination); 
}

kFx(kStatus) kDirectory_Delete(const kChar* directory)
{
    kSize i;
    kArrayList files = kNULL; 
    kArrayList directories = kNULL; 
    kString srcItem = kNULL; 
    kChar srcPath[kPATH_MAX]; 

    kCheckErr(kDirectory_Exists(directory), NOT_FOUND);

    kTry
    {
        //delete files
        kTest(kDirectory_Files(directory, &files)); 

        for (i = 0; i < kArrayList_Count(files); ++i)
        {
            kTest(kArrayList_Item(files, i, &srcItem)); 
            kTest(kPath_Combine(directory, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kFile_Delete(srcPath));  
        }

        //delete sub-directories
        kTest(kDirectory_Directories(directory, &directories)); 

        for (i = 0; i < kArrayList_Count(directories); ++i)
        {
            kTest(kArrayList_Item(directories, i, &srcItem)); 
            kTest(kPath_Combine(directory, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kDirectory_Delete(srcPath));  
        }
    
        //delete the target directory
        kTest(kApiLib_DirectoryHandlers_()->del(directory)); 
    }
    kFinally
    {
        kCheck(kObject_Dispose(files)); 
        kCheck(kObject_Dispose(directories)); 
        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_Enumerate(const kChar* directory, kBool includeFiles, kBool includeDirectories, kArrayList entries)
{    
    return kApiLib_DirectoryHandlers_()->enumerate(directory, includeFiles, includeDirectories, entries); 
}

kFx(kStatus) kDirectory_ListFiles(const kChar* directory, kArrayList files)
{    
    return kApiLib_DirectoryHandlers_()->enumerate(directory, kTRUE, kFALSE, files); 
}

kFx(kStatus) kDirectory_ListDirectories(const kChar* directory, kArrayList directories)
{    
    return kApiLib_DirectoryHandlers_()->enumerate(directory, kFALSE, kTRUE, directories); 
}

kFx(kStatus) kDirectory_ListEntries(const kChar* directory, kArrayList entries)
{    
    return kApiLib_DirectoryHandlers_()->enumerate(directory, kTRUE, kTRUE, entries); 
}

kFx(kStatus) kDirectory_SetCurrent(const kChar* directory)
{
    return kApiLib_DirectoryHandlers_()->setCurrent(directory); 
}

kFx(kStatus) kDirectory_Current(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->current(directory, capacity); 
}

kFx(kStatus) kDirectory_Application(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->appDirectory(directory, capacity); 
}

kFx(kStatus) kDirectory_Temp(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->tempDirectory(directory, capacity); 
}

kFx(kStatus) kDirectory_AppConfig(const kChar* appName, kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->appConfigDirectory(appName, directory, capacity); 
}

kFx(kStatus) kDirectory_AppData(const kChar* appName, kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->appDataDirectory(appName, directory, capacity); 
}

kFx(kStatus) kDirectory_Plugin(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->pluginDirectory(directory, capacity); 
}

kFx(kStatus) kDirectory_AppConfigImpl(const kChar* appName, kChar* directory, kSize capacity)
{    
    kCheck(kDirectory_Application(directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Combine(directory, kDIRECTORY_DEFAULT_CONFIG_FOLDER, directory, capacity)); 

    if (!kIsNull(appName))
    {
        kCheck(kPath_Combine(directory, appName, directory, capacity)); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_AppDataImpl(const kChar* appName, kChar* directory, kSize capacity)
{    
    kCheck(kDirectory_Application(directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Combine(directory, kDIRECTORY_DEFAULT_DATA_FOLDER, directory, capacity)); 

    if (!kIsNull(appName))
    {
        kCheck(kPath_Combine(directory, appName, directory, capacity)); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_FromVirtualImpl(const kChar* vPath, kChar* path, kSize capacity)
{
    return kStrCopy(path, capacity, vPath); 
}

kFx(kStatus) kDirectory_ToVirtualImpl(const kChar* path, kChar* vPath, kSize capacity)
{
    return kStrCopy(vPath, capacity, path); 
}

#if defined(K_WINDOWS)

kFx(kStatus) kDirectory_CreateImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(kPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    return CreateDirectory(wpath, NULL) ? kOK : kERROR_OS;
}

kFx(kBool) kDirectory_ExistsImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 
    DWORD attr = 0; 

    kCheck(kPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 
    
    attr = GetFileAttributes(wpath);

    return (attr != INVALID_FILE_ATTRIBUTES) && (attr & FILE_ATTRIBUTE_DIRECTORY);
}

kFx(kStatus) kDirectory_MoveImpl(const kChar* source, const kChar* destination)
{
    WCHAR wpathSrc[MAX_PATH];
    WCHAR wpathDst[MAX_PATH];  

    kCheck(kPath_NormalizedToNativeWideWin(source, wpathSrc, kCountOf(wpathSrc))); 
    kCheck(kPath_NormalizedToNativeWideWin(destination, wpathDst, kCountOf(wpathDst))); 
    
    return MoveFile(wpathSrc, wpathDst) ? kOK : kERROR_OS; 
}

kFx(kStatus) kDirectory_DeleteImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(kPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    return RemoveDirectory(wpath) ? kOK : kERROR_OS; 
}

kFx(kStatus) kDirectory_EnumerateImpl(const kChar* path, kBool includeFile, kBool includeDirectories, kArrayList entries)
{
    HANDLE findHandle = INVALID_HANDLE_VALUE;
    WCHAR searchFilter[MAX_PATH];
    kChar entry[MAX_PATH];
    WIN32_FIND_DATA findData;
    kString item = kNULL;

    kTry
    {
        kTest(kPath_NormalizedToNativeWideWin(path, searchFilter, kCountOf(searchFilter))); 
        wcscat(searchFilter, L"\\*"); 

        if ((findHandle = FindFirstFile(searchFilter, &findData)) == INVALID_HANDLE_VALUE)
        {
            kThrow(kERROR_OS);
        }

        kTest(kArrayList_Allocate(entries, kTypeOf(kString), 0)); 
        
        do
        {
            kBool isDirectory = (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0; 
            kBool isSpecial = (wcscmp(findData.cFileName, L".") == 0) || (wcscmp(findData.cFileName, L"..") == 0); 
            kBool filter = !isSpecial && ((includeFile && !isDirectory) || (includeDirectories && isDirectory)); 

            if (filter)
            {
                kTest(kPath_NativeWideToNormalizedWin(findData.cFileName, entry, kCountOf(entry))); 

                kTest(kString_Construct(&item, entry, kNULL)); 
                kTest(kArrayList_Add(entries, &item));
                item = kNULL;
            }
        } 
        while (FindNextFile(findHandle, &findData));
    } 
    kFinally
    {
        kCheck(kObject_Destroy(item)); 

        if (findHandle != INVALID_HANDLE_VALUE)
        {
            FindClose(findHandle);
        }

        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) kDirectory_SetCurrentImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(kPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    return SetCurrentDirectory(wpath) ? kOK : kERROR_OS;
}

kFx(kStatus) kDirectory_CurrentImpl(kChar* directory, kSize capacity)
{
    WCHAR wpath[MAX_PATH]; 

    if (GetCurrentDirectory(kCountOf(wpath), wpath) == 0)
    {
        return kERROR_OS;
    }
  
    kCheck(kPath_NativeWideToNormalizedWin(wpath, directory, capacity)); 

    return kOK; 
}

kFx(kStatus) kDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{
    WCHAR wmodulePath[kPATH_MAX]; 
    HANDLE module;
    
    if (kIsNull(module = GetModuleHandle(NULL)))
    {
        return kERROR; 
    }

    if (GetModuleFileName(module, wmodulePath, kCountOf(wmodulePath)) == 0)
    {
        return kERROR; 
    }

    kCheck(kPath_NativeWideToNormalizedWin(wmodulePath, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 

    return kOK;
}

kFx(kStatus) kDirectory_TempImpl(kChar* path, kSize capacity)
{
    WCHAR wpath[kPATH_MAX]; 

    if (GetTempPath(kCountOf(wpath), wpath) == 0)
    {
        return kERROR_OS;
    }
    
    kCheck(kPath_NativeWideToNormalizedWin(wpath, path, capacity)); 
    
    return kOK; 
}

kFx(kStatus) kDirectory_PluginImpl(kChar* directory, kSize capacity)
{
    return kDirectory_Application(directory, capacity); 
}

#elif defined(K_DARWIN)

kFx(kStatus) kDirectory_CreateImpl(const kChar* directory)
{   
    return (mkdir(directory, S_IRWXU) == -1) ? kERROR_OS : kOK; 
}

kFx(kBool) kDirectory_ExistsImpl(const kChar* directory)
{   
    struct stat st; 

    return (stat(directory, &st) != -1) && S_ISDIR(st.st_mode); 
}

kFx(kStatus) kDirectory_MoveImpl(const kChar* source, const kChar* destination)
{   
    return (rename(source, destination) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) kDirectory_DeleteImpl(const kChar* directory)
{   
    return (rmdir(directory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) kDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries)
{   
    DIR* dirIt = kNULL; 
    struct dirent* dirEntry = kNULL; 
    kString item = kNULL; 

    kCheckErr((dirIt = opendir(directory)) != kNULL, NOT_FOUND); 

    kTry
    {
        kTest(kArrayList_Allocate(entries, kTypeOf(kString), 0)); 
  
        while ((dirEntry = readdir(dirIt)) != kNULL)
        {
            kBool isDirectory = (dirEntry->d_type == DT_DIR); 
            kBool isSpecial = (kStrEquals(dirEntry->d_name, ".") || kStrEquals(dirEntry->d_name, "..")); 
            kBool filter = !isSpecial && ((includeFile && !isDirectory) || (includeDirectories && isDirectory)); 

            if (filter)
            {               
                kTest(kString_Construct(&item, dirEntry->d_name, kNULL)); 
                kTest(kArrayList_Add(entries, &item));
                item = kNULL;
            }
        } 
    }
    kFinally
    {
        closedir(dirIt);
        kObject_Destroy(item); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_SetCurrentImpl(const kChar* directory)
{   
    return (chdir(directory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) kDirectory_CurrentImpl(kChar* directory, kSize capacity)
{   
    return (getcwd(directory, capacity) == kNULL) ? kERROR_INCOMPLETE : kOK; 
}

kFx(kStatus) kDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{
    kChar modulePath[kPATH_MAX];
    uint32_t size = kCountOf(modulePath); 

    if (_NSGetExecutablePath(modulePath, &size) != 0)
    {
        return kERROR; 
    }

    kCheck(kPath_Normalize(modulePath, modulePath, kCountOf(modulePath)));  
    kCheck(kPath_ToVirtual(modulePath, modulePath, kCountOf(modulePath)));  

    kCheck(kPath_Directory(modulePath, directory, capacity));     
    kCheck(kPath_Directory(directory, directory, capacity));         

    return kOK;
}

kFx(kStatus) kDirectory_TempImpl(kChar* directory, kSize capacity)
{   
    //TODO: system-specific; could probably be improved (?)
    return kStrCopy(directory, capacity, "/tmp"); 
}

kFx(kStatus) kDirectory_PluginImpl(kChar* directory, kSize capacity)
{
    kCheck(kDirectory_Application(directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 

    kCheck(kPath_Combine(directory, "lib", directory, capacity)); 
    kCheck(kPath_Combine(directory, K_DEBUG_ENABLED ? "gnud" : "gnu", directory, capacity)); 

    return kOK; 
}

#elif  defined(K_LINUX)

kFx(kStatus) kDirectory_CreateImpl(const kChar* directory)
{   
    return (mkdir(directory, S_IRWXU) == -1) ? kERROR_OS : kOK; 
}

kFx(kBool) kDirectory_ExistsImpl(const kChar* directory)
{   
    struct stat st; 

    return (stat(directory, &st) != -1) && S_ISDIR(st.st_mode); 
}

kFx(kStatus) kDirectory_MoveImpl(const kChar* source, const kChar* destination)
{   
    return (rename(source, destination) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) kDirectory_DeleteImpl(const kChar* directory)
{   
    return (rmdir(directory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) kDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries)
{   
    DIR* dirIt = kNULL; 
    struct dirent* dirEntry = kNULL; 
    kString item = kNULL; 

    kCheckErr((dirIt = opendir(directory)) != kNULL, NOT_FOUND); 

    kTry
    {
        kTest(kArrayList_Allocate(entries, kTypeOf(kString), 0)); 
  
        while ((dirEntry = readdir(dirIt)) != kNULL)
        {
            kBool isDirectory = (dirEntry->d_type == DT_DIR); 
            kBool isSpecial = (kStrEquals(dirEntry->d_name, ".") || kStrEquals(dirEntry->d_name, "..")); 
            kBool filter = !isSpecial && ((includeFile && !isDirectory) || (includeDirectories && isDirectory)); 

            if (filter)
            {               
                kTest(kString_Construct(&item, dirEntry->d_name, kNULL)); 
                kTest(kArrayList_Add(entries, &item));
                item = kNULL;
            }
        } 
    }
    kFinally
    {
        closedir(dirIt);
        kObject_Destroy(item); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_SetCurrentImpl(const kChar* directory)
{   
    return (chdir(directory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) kDirectory_CurrentImpl(kChar* directory, kSize capacity)
{   
    return (getcwd(directory, capacity) == kNULL) ? kERROR_INCOMPLETE : kOK; 
}

kFx(kStatus) kDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{
    kChar modulePath[kPATH_MAX];
    ssize_t length; 
    
    if ((length = readlink("/proc/self/exe", modulePath, kCountOf(modulePath)-1)) == -1)
    {
        return kERROR; 
    }

    modulePath[length-1] = 0; 

    kCheck(kPath_Normalize(modulePath, modulePath, kCountOf(modulePath))); 
    kCheck(kPath_ToVirtual(modulePath, modulePath, kCountOf(modulePath)));  

    kCheck(kPath_Directory(modulePath, directory, capacity)); 

    return kOK;
}

kFx(kStatus) kDirectory_TempImpl(kChar* directory, kSize capacity)
{   
    //TODO: system-specific; could probably be improved (?)
    return kStrCopy(directory, capacity, "/tmp"); 
}

kFx(kStatus) kDirectory_PluginImpl(kChar* directory, kSize capacity)
{    
    kCheck(kDirectory_Application(directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 

    kCheck(kPath_Combine(directory, "lib", directory, capacity)); 
    kCheck(kPath_Combine(directory, K_DEBUG_ENABLED ? "gnud" : "gnu", directory, capacity)); 

    return kOK; 
}

#else

kFx(kStatus) kDirectory_CreateImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kBool) kDirectory_ExistsImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kFALSE; 
}

kFx(kStatus) kDirectory_MoveImpl(const kChar* source, const kChar* destination)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_DeleteImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_SetCurrentImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_CurrentImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_TempImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kDirectory_PluginImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

#endif


//deprecated
kFx(kStatus) kDirectory_Files(const kChar* directory, kArrayList* files)
{
    kArrayList output = kNULL; 
    kStatus exception; 

    kTry
    {   
        kTest(kArrayList_Construct(&output, kTypeOf(kString), 16, kNULL)); 
        kTest(kDirectory_ListFiles(directory, output));  

        *files = output; 
    }
    kCatch(&exception)
    {
        kCheck(kObject_Dispose(output)); 
        kEndCatch(exception); 
    }

    return kOK; 
}

//deprecated
kFx(kStatus) kDirectory_Directories(const kChar* directory, kArrayList* directories)
{
    kArrayList output = kNULL; 
    kStatus exception; 

    kTry
    {   
        kTest(kArrayList_Construct(&output, kTypeOf(kString), 16, kNULL)); 
        kTest(kDirectory_ListDirectories(directory, output));  

        *directories = output; 
    }
    kCatch(&exception)
    {
        kCheck(kObject_Dispose(output)); 
        kEndCatch(exception); 
    }

    return kOK; 
}

//deprecated
kFx(kStatus) kDirectory_Entries(const kChar* directory, kArrayList* entries)
{
    kArrayList output = kNULL; 
    kStatus exception; 

    kTry
    {   
        kTest(kArrayList_Construct(&output, kTypeOf(kString), 16, kNULL)); 
        kTest(kDirectory_ListEntries(directory, output));  

        *entries = output; 
    }
    kCatch(&exception)
    {
        kCheck(kObject_Dispose(output)); 
        kEndCatch(exception); 
    }

    return kOK; 
}
