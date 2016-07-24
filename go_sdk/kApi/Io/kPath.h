/** 
 * @file    kPath.h
 * @brief   Declares the kPath class. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PATH_H
#define K_API_PATH_H

#include <kApi/kApiDef.h> 

kBeginHeader()

/** @relates kPath @{ */
#define kPATH_MAX      (kxPATH_MAX)         ///< Maximum supported path length.
/** @} */

/**
 * @class   kPath
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Collection of path manipulation functions. 
 * 
 * Most of the functions in this class expect paths to be in normal form (canonical 
 * separator character, no trailing slashes). The kPath_Normalize function can be 
 * used to normalize paths prior to calling other kPath functions. 
 * 
 * The kPath_ToNative function can be used to transform a path in normal form 
 * to a form suitable for the underlying system (native separator). 
 */
typedef kObject kPath; 

/** 
 * Returns the normalized path separator character.
 *
 * @public              @memberof kPath
 * @return              Path separator character.
 */
kFx(kChar) kPath_Separator();

/** 
 * Determines if the given character is equal to the normalized path separator.
 *
 * @public          @memberof kPath
 * @param   ch      Character to examine. 
 * @return          kTRUE if the character is the normalized path separator. 
 */
kFx(kBool) kPath_IsSeparator(kChar ch);

/** 
 * Combines two path segments using the normalized path separator character.
 *
 * The path (output) argument can refer to the same memory address as either of the segment (input) arguments. 
 * 
 * @public              @memberof kPath
 * @param   segment1    First path segment. 
 * @param   segment2    Second path segment. 
 * @param   path        Receives the combined path segments. 
 * @param   capacity    Maximum number of characters (including null terminator).  
 * @return              Operation status.
 */
kFx(kStatus) kPath_Combine(const kChar* segment1, const kChar* segment2, kChar* path, kSize capacity); 

/** 
 * Returns the parent directory for a given file or directory path. 
 *
 * The directory (output) argument can refer to the same memory address as the path (input) argument. 
 * 
 * If the path contains embedded relative path components, this function will not attempt 
 * to evaluate the meaning of those components. 
 *
 * @public              @memberof kPath
 * @param   path        File or directory path. 
 * @param   directory   Receives the parent directory.
 * @param   capacity    Maximum number of characters (including null terminator). 
 * @return              Operation status.
 */
kFx(kStatus) kPath_Directory(const kChar* path, kChar* directory, kSize capacity);

/** 
 * Given a file path, returns the portion of the path containing the file name.
 *
 * The fileName (output) argument can refer to the same memory address as the path (input) argument. 
 *
 * @public              @memberof kPath
 * @param   path        File path.
 * @param   fileName    Receives the file name portion of the path.  
 * @param   capacity    Maximum number of characters (including null terminator).  
 * @return              Operation status.
 */
kFx(kStatus) kPath_FileName(const kChar* path, kChar* fileName, kSize capacity); 

/** 
 * Given a file path, returns the portion of the path containing the file extension.
 *
 * The extension (output) argument can refer to the same memory address as the path (input) argument. 
 *
 * @public              @memberof kPath
 * @param   path        File path.
 * @param   extension   Receives the file extension portion of the path.   
 * @param   capacity    Maximum number of characters (including null terminator).  
 * @return              Operation status.
 */
kFx(kStatus) kPath_Extension(const kChar* path, kChar* extension, kSize capacity); 

/** 
 * Expresses an absolute path in relative form, in relation to a reference path.
 *
 * If pathA and pathB are rooted in different volumes, then bRelativeToA will receive an absolute path equal 
 * to pathB. 
 * 
 * The bRelativeToA (output) argument can refer to the same memory address as either the pathA or pathB 
 * (input) arguments. 
 *
 * @public                  @memberof kPath
 * @param  pathA            Absolute reference path.
 * @param  pathB            Absolute path to be re-expressed as relative to path A.
 * @param  bRelativeToA     Receives path b, expressed as relative to path A.
 * @param  capacity         Maximum number of characters (including null terminator) for the relative path.
 * @return                  Operation status.
 */
kFx(kStatus) kPath_ToRelative(const kChar* pathA, const kChar* pathB, kChar* bRelativeToA, kSize capacity); 

/** 
 * Finds the absolute path expressed by the combination of an absolute path and a relative path. 
 *
 * If bRelativeToA is already expressed in absolute form, then pathB will receive an absolute path equal 
 * to bRelativeToA.
 *
 * The pathB (output) argument can refer to the same memory address as either the pathA or bRelativeToA 
 * (input) arguments. 
 *
 * @public                  @memberof kPath
 * @param  pathA            Absolute reference path.
 * @param  bRelativeToA     Path b, expressed as relative to path A.
 * @param  pathB            Receives path b, expressed as an absolute path.  
 * @param  capacity         Maximum number of characters (including null terminator) for path b.
 * @return                  Operation status.
 */
kFx(kStatus) kPath_ToAbsolute(const kChar* pathA, const kChar* bRelativeToA, kChar* pathB, kSize capacity);

/** 
 * Transforms all path separators to normal form and removes trailing slashes.
 *
 * The normalized (output) argument can refer to the same memory address as the path (input) argument. 
 *
 * @public             @memberof kPath
 * @param  path        Input path, in native or mixed form.
 * @param  normalized  Receives transformed path, in normal form.
 * @param  capacity    Maximum number of characters (including null terminator) for normalized path.
 * @return             Operation status.
 */
kFx(kStatus) kPath_Normalize(const kChar* path, kChar* normalized, kSize capacity); 

/** 
 * Transforms all path separators to native form.
 *
 * The native (output) argument can refer to the same memory address as the path (input) argument. 
 *
 * @public             @memberof kPath
 * @param  path        Input path, in normal or mixed form.
 * @param  native      Receives transformed path, in native form.
 * @param  capacity    Maximum number of characters (including null terminator) for native path.
 * @return             Operation status.
 */
kFx(kStatus) kPath_ToNative(const kChar* path, kChar* native, kSize capacity); 

kEndHeader()

#include <kApi/Io/kPath.x.h>

#endif
