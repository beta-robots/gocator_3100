/** 
 * @file    kDirectory.h
 * @brief   Declares the kDirectory class. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DIRECTORY_H
#define K_API_DIRECTORY_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kDirectory
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Collection of directory-related functions. 
 */
typedef kObject kDirectory; 

/** 
 * Creates a directory at the specified location. 
 *
 * This function will fail if the directory already exists. Missing parent folders will be created automatically. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the directory. 
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Create(const kChar* directory); 

/** 
 * Reports whether the specified directory exists. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the directory. 
 * @return              kTRUE if the directory exists, kFALSE otherwise. 
 */
kFx(kBool) kDirectory_Exists(const kChar* directory);

/** 
 * Copies the specified directory, including all of its contents. 
 * 
 * @public              @memberof kDirectory
 * @param  source       Full path of the source directory. 
 * @param  destination  Full path of the destination directory.
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Copy(const kChar* source, const kChar* destination); 

/** 
 * Moves the specified directory, including all of its contents. 
 * 
 * @public              @memberof kDirectory
 * @param  source       Full path of the source directory. 
 * @param  destination  Full path of the destination directory.
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Move(const kChar* source, const kChar* destination); 

/** 
 * Deletes the specified directory, including all of its contents. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the directory. 
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Delete(const kChar* directory);

/** 
 * List the files in the specified directory. 
 * 
 * Use kArrayList_Purge to destroy the file names returned by this function.
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the directory. 
 * @param  files        Receives file names (kArrayList<kString>). 
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_ListFiles(const kChar* directory, kArrayList files); 

/** 
 * Creates a list of the sub-directories in the specified directory. 
 * 
 * Use kArrayList_Purge to destroy the directory names returned by this function.
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the directory. 
 * @param  directories  Receives directory names (kArrayList<kString>). 
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_ListDirectories(const kChar* directory, kArrayList directories); 

/** 
 * Creates a list of the file system entries in the specified directory. 
 * 
 * Use kArrayList_Purge to destroy the entry names returned by this function.
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the directory. 
 * @param  entries      Receives entry names (kArrayList<kString>). 
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_ListEntries(const kChar* directory, kArrayList entries); 

/** 
 * Sets the current working directory. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Full path of the desired working directory. 
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_SetCurrent(const kChar* directory);

/** 
 * Gets the current working directory. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Returns the full path of the current working directory. 
 * @param  capacity     Maximum number of characters (including null terminator).  
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Current(kChar* directory, kSize capacity); 

/** 
 * Gets the directory in which the application executable file resides. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Returns the full path of the application directory. 
 * @param  capacity     Maximum number of characters (including null terminator).   
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Application(kChar* directory, kSize capacity);

/** 
 * Gets the path of a directory suitable for temporary files.
 * 
 * @public              @memberof kDirectory
 * @param  directory    Returns the full path of the temp directory. 
 * @param  capacity     Maximum number of characters (including null terminator).   
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Temp(kChar* directory, kSize capacity);

/** 
 * Gets the directory from which an application should load its configuration/resource files. 
 *
 * This function assumes the standard folder organization of a zen-based application. 
 * 
 * @public              @memberof kDirectory
 * @param  appName      The name of the application (optional).  
 * @param  directory    Returns the full path of the requested directory. 
 * @param  capacity     Maximum number of characters (including null terminator).   
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_AppConfig(const kChar* appName, kChar* directory, kSize capacity);

/** 
 * Gets a directory suitable for an application to write data files. 
 *
 * This function assumes the standard folder organization of a zen-based application. 
 * 
 * @public              @memberof kDirectory
 * @param  appName      The name of the application (optional). 
 * @param  directory    Returns the full path of the requested directory. 
 * @param  capacity     Maximum number of characters (including null terminator).   
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_AppData(const kChar* appName, kChar* directory, kSize capacity);

/** 
 * Gets the directory where plug-ins are located. 
 *
 * This function assumes the standard folder organization of a zen-based application. 
 * 
 * @public              @memberof kDirectory
 * @param  directory    Returns the full path of the requested directory. 
 * @param  capacity     Maximum number of characters (including null terminator).   
 * @return              Operation status. 
 */
kFx(kStatus) kDirectory_Plugin(kChar* directory, kSize capacity);

kEndHeader()

#include <kApi/Io/kDirectory.x.h>

#endif
