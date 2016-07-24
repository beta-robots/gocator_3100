/** 
 * @file    Path.c
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kPath.h>
#include <stdio.h>

kBeginStaticClass(k, kPath)
kEndStaticClass()

kFx(kStatus) kPath_InitStatic()
{
    //set the default path separator if a user-supplied value has not been provided
    if (kApiLib_NativeSeparator_() == 0)
    {
        kCheck(kApiLib_SetNativeSeparator(kPATH_DEFAULT_NATIVE_SEPARATOR)); 
    }
   
    return kOK;
}

kFx(kStatus) kPath_ReleaseStatic()
{
    return kOK; 
}

kFx(kChar) kPath_Separator()
{
    return kPATH_SEPARATOR; 
}

kFx(kChar) kPath_AltSeparator()
{
    return kPATH_ALT_SEPARATOR; 
}

kFx(kChar) kPath_NativeSeparator()
{
    return kApiLib_NativeSeparator_();
}

kFx(kBool) kPath_IsSeparator(kChar ch)
{
    return (ch == kPath_Separator()); 
}

kFx(kChar) kPath_VolumeSeparator()
{
    return kPATH_VOLUME_SEPARATOR; 
}

kFx(kStatus) kPath_Combine(const kChar* segment1, const kChar* segment2, kChar* path, kSize capacity)
{
    kChar segment1Norm[kPATH_MAX]; 
    kChar segment2Norm[kPATH_MAX]; 
    kSize length1 = kStrLength(segment1); 

    //normalize inputs
    kCheck(kPath_Normalize(segment1, segment1Norm, kCountOf(segment1Norm))); 
    kCheck(kPath_Normalize(segment2, segment2Norm, kCountOf(segment2Norm))); 

    if (kStrEquals(segment2Norm, ""))
    {
        kCheck(kStrCopy(path, capacity, segment1Norm)); 
    }
    else if (kStrEquals(segment1Norm, ""))
    {
        kCheck(kStrCopy(path, capacity, segment2Norm)); 
    }
    else if (kPath_IsSeparator(segment1Norm[length1-1]) || kPath_IsSeparator(segment2Norm[0]))
    {
        kCheck(kStrPrintf(path, capacity, "%s%s", segment1Norm, segment2Norm)); 
    }
    else
    {
        kCheck(kStrPrintf(path, capacity, "%s%c%s", segment1Norm, kPath_Separator(), segment2Norm)); 
    }

    return kOK;
}

kFx(kStatus) kPath_Directory(const kChar* path, kChar* directory, kSize capacity)
{
    kChar pathNorm[kPATH_MAX]; 
    kSize pathLength = kStrLength(path); 
    kChar* division = kNULL; 
    
    //normalize inputs
    kCheck(kPath_Normalize(path, pathNorm, kCountOf(pathNorm))); 
    
    //find the last separator
    division = strrchr(pathNorm, kPath_Separator()); 
    
    if (kIsNull(division) || ((kSize)(division - pathNorm + 1) == pathLength))
    {
        //if there is no separator, or if the separator is at the end of the path (since the path is
        //in normal form, this can only mean root/windows-drive/unc-root), indicate that a parent 
        //directory was not found
        return kERROR_NOT_FOUND; 
    }
    else
    {
        kSize outputCapacity = (kSize)(division - pathNorm) + 2;         //+1 for separator, +1 for null

        if (outputCapacity > capacity) 
        {
            return kERROR_INCOMPLETE; 
        }

        //take the directory portion of the path
        kStrCopy(directory, outputCapacity, pathNorm); 
    
        //remove trailing separator, if appropriate
        kCheck(kPath_RemoveTrailingSeparator(directory)); 

        return kOK;
    }
}

kFx(kStatus) kPath_FileName(const kChar* path, kChar* fileName, kSize capacity)
{
    kChar pathNorm[kPATH_MAX]; 
    kChar* division = kNULL; 
    
    //normalize inputs
    kCheck(kPath_Normalize(path, pathNorm, kCountOf(pathNorm))); 

    //find the last separator
    division = strrchr(pathNorm, kPath_Separator()); 

    if (kIsNull(division))
    {
        if (kStrEquals(pathNorm, ""))
        {
            //if the string is empty, report that the file portion of the path was not found.
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //if no separator was found, but the path contains characters, assume that it represents a file name
            kCheck(kStrCopy(fileName, capacity, pathNorm)); 
        }
    }
    else
    {
        //take the file portion of the path
        kCheck(kStrCopy(fileName, capacity, division+1)); 
    }

    return kOK;
}

kFx(kStatus) kPath_Extension(const kChar* path, kChar* extension, kSize capacity)
{
    kChar name[kPATH_MAX]; 
    const kChar* division = kNULL; 
    
    kCheck(kPath_FileName(path, name, kCountOf(name))); 
   
    if (kIsNull(division = strrchr(name, '.')))
    {
        return kERROR_NOT_FOUND; 
    }
   
    kCheck(kStrCopy(extension, capacity, division+1)); 

    return kOK; 
}

kFx(kBool) kPath_IsAbsolute(const kChar* path)
{
    kChar pathNorm[kPATH_MAX]; 
    kChar separator = kPath_Separator(); 
    
    //normalize inputs
    if (kSuccess(kPath_Normalize(path, pathNorm, kCountOf(pathNorm))))
    {
        kSize length = kStrLength(path); 
        kBool isRoot = (length > 0) && (pathNorm[0] == separator);                          // "/", "//"
        kBool isVolumeRoot = (length > 2) && (pathNorm[1] == kPath_VolumeSeparator());      // "c:/"

        return isRoot || isVolumeRoot; 
    }

    return kFALSE; 
}

kFx(kSize) kPath_CommonLength(const kChar* pathA, const kChar* pathB)
{
    kChar separator = kPath_Separator(); 
    kSize segment = 0; 
    kSize i = 0; 

    while ((pathA[i] != 0) && (pathB[i] != 0) && (pathA[i] == pathB[i]))
    {
        if (pathA[i] == separator)
        {
            segment = i; 
        }
        i++; 
    }

    if (((pathA[i] == 0) && ((pathB[i] == 0) || (pathB[i] == separator))) ||
        ((pathB[i] == 0) && ((pathA[i] == 0) || (pathA[i] == separator))))
    {
        segment = i; 
    }

    return segment;  
}

kFx(kStatus) kPath_ToRelative(const kChar* pathA, const kChar* pathB, kChar* bRelativeToA, kSize capacity)
{
    kChar pathANorm[kPATH_MAX]; 
    kChar pathBNorm[kPATH_MAX]; 
    kSize commonLength = 0; 
    kSize i; 

    kCheckArgs(kPath_IsAbsolute(pathA) && kPath_IsAbsolute(pathB)); 

    //normalize inputs
    kCheck(kPath_Normalize(pathA, pathANorm, kCountOf(pathANorm))); 
    kCheck(kPath_Normalize(pathB, pathBNorm, kCountOf(pathBNorm))); 
   
    if (kStrEquals(pathANorm, pathBNorm))
    {
        //paths are identical
        return kStrCopy(bRelativeToA, capacity, "."); 
    }

    //absolute paths are guaranteed to have at least 1 character
    if (pathANorm[0] != pathBNorm[0])
    {
        //paths have no common component (possible with Windows volumes)
        return kStrCopy(bRelativeToA, capacity, pathBNorm); 
    }
    
    kCheck(kStrCopy(bRelativeToA, capacity, "")); 

    commonLength = kPath_CommonLength(pathANorm, pathBNorm);

    //for each topmost directory in the reference path that is *not* in the target path, 
    //add an instance of ".." to the relative path
    i = commonLength; 
    while ((pathANorm[i] != 0))
    {
        if (pathANorm[i++] == kPath_Separator())
        {
            kCheck(kPath_Combine(bRelativeToA, "..", bRelativeToA, capacity)); 
        }
    }

    if (pathBNorm[commonLength] != 0)
    {
        kCheck(kPath_Combine(bRelativeToA, &pathBNorm[commonLength+1], bRelativeToA, capacity)); 
    }

    return kOK;
}

kFx(kStatus) kPath_ToAbsolute(const kChar* pathA, const kChar* bRelativeToA, kChar* pathB, kSize capacity)
{
    kChar pathANorm[kPATH_MAX]; 
    kChar bRelativeToANorm[kPATH_MAX]; 
    kSize i = 0; 

    kCheckArgs(kPath_IsAbsolute(pathA)); 

    //normalize inputs
    kCheck(kPath_Normalize(pathA, pathANorm, kCountOf(pathANorm))); 
    kCheck(kPath_Normalize(bRelativeToA, bRelativeToANorm, kCountOf(bRelativeToANorm))); 

    if (kStrEquals(bRelativeToA, "."))
    {
        //relative path is same as absolute path
        return kStrCopy(pathB, capacity, pathA); 
    }

    if (kPath_IsAbsolute(bRelativeToA))
    {
        //input path was already in absolute form
        return kStrCopy(pathB, capacity, bRelativeToANorm); 
    }   

    kCheck(kStrCopy(pathB, capacity, pathANorm)); 

    //pop the topmost directory from the absolute reference path for each instance of ".." in the relative path
    while ((bRelativeToANorm[i] == '.') && (bRelativeToANorm[i+1] == '.'))
    {
        kCheck(kPath_Directory(pathB, pathB, capacity));  

        i += (bRelativeToANorm[i+2] == kPath_Separator()) ? 3 : 2; 
    }

    if (bRelativeToANorm[i] != 0)
    {
        kCheck(kPath_Combine(pathB, &bRelativeToANorm[i], pathB, capacity)); 
    }
        
    return kOK;
}

kFx(kStatus) kPath_Normalize(const kChar* path, kChar* normalized, kSize capacity)
{
    kChar separator = kPath_Separator(); 
    kChar altSeparator = kPath_AltSeparator();
    kSize length = kStrLength(path); 
    kSize i = 0; 
    
    kCheckArgs(capacity >= (length+1)); 
    
    //convert separators
    for (i = 0; i < length+1; ++i)
    {
        normalized[i] = (path[i] == altSeparator) ? separator : path[i]; 
    }

    kCheck(kPath_RemoveTrailingSeparator(normalized)); 
    
    return kOK; 
}

kFx(kStatus) kPath_ToNative(const kChar* path, kChar* native, kSize capacity)
{    
    kChar separator = kPath_Separator(); 
    kChar altSeparator = kPath_AltSeparator();
    kChar nativeSeparator = kPath_NativeSeparator(); 
    kSize length = kStrLength(path); 
    kSize i = 0; 
    
    kCheckArgs(capacity >= (length+1)); 
    
    //convert separators
    for (i = 0; i < length+1; ++i)
    {
        native[i] = ((path[i] == separator) || (path[i] == altSeparator)) ? nativeSeparator : path[i]; 
    }
    
    return kOK; 
}

kFx(kStatus) kPath_RemoveTrailingSeparator(kChar* path)
{
    kSize length = kStrLength(path); 
    
    //if trailing slash is present...
    if ((length > 0) && (path[length-1] == kPath_Separator()))
    {
        kBool isStandardRoot = (length == 1);                                           // "/"
        kBool isUncRoot = (length == 2) && (path[0] == kPath_Separator());              // "//"    
        kBool isVolumeRoot = (length == 3) && (path[1] == kPath_VolumeSeparator());     // "c:/"

        //if trailing slash is not a root...
        if (!isStandardRoot && !isUncRoot && !isVolumeRoot)
        {
            //remove trailing slash
            path[length-1] = 0; 
        }
    }

    return kOK; 
}

kFx(kStatus) kPath_LibraryName(const kChar* libraryName, kChar* fileName, kSize capacity)
{
    kChar tempName[kPATH_MAX]; 

    kCheck(kStrCopy(tempName, kCountOf(tempName), libraryName)); 

    return kStrPrintf(fileName, capacity, "%s.%s", tempName, kPATH_LIBRARY_EXTENSION); 
}

kFx(kBool) kPath_IsFileNameValid(const kChar* fileName)
{
    return (strcspn(fileName, "\\/:*?\"<>|") == strlen(fileName)); 
}

kFx(kStatus) kPath_FromVirtual(const kChar* vPath, kChar* path, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->fromVirtual(vPath, path, capacity); 
}

kFx(kStatus) kPath_ToVirtual(const kChar* path, kChar* vPath, kSize capacity)
{
    return kApiLib_DirectoryHandlers_()->toVirtual(path, vPath, capacity); 
}

#if defined(K_WINDOWS)

kFx(kStatus) kPath_NormalizedToNativeWideWin(const kChar* path, WCHAR* wpath, kSize capacity)
{
    kChar nativePath[kPATH_MAX]; 

    kCheck(kPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    if (MultiByteToWideChar(CP_UTF8, 0, nativePath, -1, wpath, (int)capacity) == 0)
    {
        return kERROR_PARAMETER; 
    }

    return kOK; 
}

kFx(kStatus) kPath_NativeWideToNormalizedWin(const WCHAR* wpath, kChar* path, kSize capacity)
{
    kChar nativePath[kPATH_MAX]; 
   
    if (WideCharToMultiByte(CP_UTF8, 0, wpath, -1, nativePath, kCountOf(nativePath), kNULL, kNULL) == 0)
    {
        return kERROR_PARAMETER; 
    }

    kCheck(kPath_Normalize(nativePath, nativePath, capacity)); 
    kCheck(kPath_ToVirtual(nativePath, path, capacity)); 

    return kOK; 
}

#endif
