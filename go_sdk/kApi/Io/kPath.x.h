/** 
 * @file    kPath.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PATH_X_H
#define K_API_PATH_X_H

kBeginHeader()

#define kxPATH_MAX                  (512)             
#define kPATH_SEPARATOR             '/'                ///< Normal path separator. 
#define kPATH_ALT_SEPARATOR         '\\'               ///< Alternative path separator. 

#if defined(K_WINDOWS)
#   define kPATH_DEFAULT_NATIVE_SEPARATOR     '\\'      ///< Default path separator used by underlying OS.
#   define kPATH_VOLUME_SEPARATOR             ':'       ///< Volume separator (Windows only, e.g. "c:"). 
#   define kPATH_LIBRARY_EXTENSION            "dll"     ///< Assumed file extension for dynamic libraries
#else
#   define kPATH_DEFAULT_NATIVE_SEPARATOR     '/' 
#   define kPATH_VOLUME_SEPARATOR              0
#   define kPATH_LIBRARY_EXTENSION            "so"  
#endif

typedef struct kPathStatic
{
    k32u placeholder;       //unused
} kPathStatic; 

kDeclareStaticClass(k, kPath)

kFx(kStatus) kPath_InitStatic();
kFx(kStatus) kPath_ReleaseStatic();

/** 
 * Given a library name (e.g. "kApi"), generates the expected file name of the dynamic library file (e.g. "kApi.dll"). 
 *
 * @public             @memberof kPath
 * @param  libraryName Library name. 
 * @param  fileName    Receives library file name. 
 * @param  capacity    Maximum number of characters (including null terminator) for fileName.
 * @return             Operation status.
 */
kFx(kStatus) kPath_LibraryName(const kChar* libraryName, kChar* fileName, kSize capacity); 

/** 
 * Returns the alternative path separator character.
 *
 * @public              @memberof kPath
 * @return              Alternative path separator character.
 */
kFx(kChar) kPath_AltSeparator();

/** 
 * Returns the native path separator character.
 *
 * @public              @memberof kPath
 * @return              Native path separator character.
 */
kFx(kChar) kPath_NativeSeparator();

/** 
 * Returns the volume separator character, if supported. 
 * 
 * Volume separators are used on Windows (i.e., the ":" in "c:\"). 
 *
 * @public              @memberof kPath
 * @return              Volume separator character. 
 */
kFx(kChar) kPath_VolumeSeparator();

/** 
 * Determines whether the provided path in an absolute path. 
 *
 * @public              @memberof kPath
 * @param  path         Path to be tested. 
 * @return              kTRUE if the path absolute, kFALSE otherwise.
 */
kFx(kBool) kPath_IsAbsolute(const kChar* path); 

/** 
 * Utility method that finds common length between two paths. 
 *
 * @public              @memberof kPath
 * @param  pathA        First path. 
 * @param  pathB        Second path. 
 * @return              Common path length, in characters.
 */
kFx(kSize) kPath_CommonLength(const kChar* pathA, const kChar* pathB); 

/** 
 * Utility method that removes extraneous trailing separators. 
 *
 * @public              @memberof kPath
 * @param  path         Path to be modified.  
 * @return              Common path length, in characters.
 */
kFx(kStatus) kPath_RemoveTrailingSeparator(kChar* path); 

/** 
 * Determines whether the provided file name is valid. 
 *
 * @public              @memberof kPath
 * @param  fileName     File name to be tested. 
 * @return              kTRUE if the file name is valid; kFALSE otherwise
 */
kFx(kBool) kPath_IsFileNameValid(const kChar* fileName); 

/** 
 * Converts a virtual path to a real path. 
 *
 * Virtual paths are used to simulate a file system structure that is different from the 
 * structure of the actual underlying file system. This function uses the toVirtual member of 
 * kApiLib_DirectoryHandlers_() to convert a virtual path to a real path.
 *
 * @public              @memberof kPath
 * @param  vPath        Virtual path. 
 * @param  path         Receives real path.
 * @param  capacity     Capacity of path argument. 
 * @return              Operation status. 
 */
kFx(kStatus) kPath_FromVirtual(const kChar* vPath, kChar* path, kSize capacity); 

/** 
 * Converts a real path to a virtual path. 
 *
 * Virtual paths are used to simulate a file system structure that is different from the 
 * structure of the actual underlying file system. This function uses the fromVirtual member of 
 * kApiLib_DirectoryHandlers_() to convert a real path to a virtual path.
 *
 * @public              @memberof kPath
 * @param  path         Real path.
 * @param  vPath        Receives virtual path. 
 * @param  capacity     Capacity of path argument. 
 * @return              Operation status. 
 */
kFx(kStatus) kPath_ToVirtual(const kChar* path, kChar* vPath, kSize capacity); 

#if defined(K_PLATFORM)
#   if defined(K_WINDOWS)

kFx(kStatus) kPath_NormalizedToNativeWideWin(const kChar* path, WCHAR* wpath, kSize capacity); 
kFx(kStatus) kPath_NativeWideToNormalizedWin(const WCHAR* wpath, kChar* path, kSize capacity); 

#   endif
#endif

kEndHeader()

#endif
