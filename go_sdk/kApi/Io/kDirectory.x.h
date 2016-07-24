/** 
 * @file    kDirectory.x.h
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DIRECTORY_X_H
#define K_API_DIRECTORY_X_H

kBeginHeader()

#define kDIRECTORY_DEFAULT_DATA_FOLDER          "data"
#define kDIRECTORY_DEFAULT_CONFIG_FOLDER        "res"

typedef struct kDirectoryStatic
{
    k32u placeholder;       //unused
} kDirectoryStatic; 

kDeclareStaticClass(k, kDirectory)

kFx(kStatus) kDirectory_InitStatic();
kFx(kStatus) kDirectory_ReleaseStatic();

kFx(kStatus) kDirectory_CreateImpl(const kChar* directory); 
kFx(kBool) kDirectory_ExistsImpl(const kChar* directory);
kFx(kStatus) kDirectory_MoveImpl(const kChar* source, const kChar* destination); 
kFx(kStatus) kDirectory_DeleteImpl(const kChar* directory);
kFx(kStatus) kDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries); 
kFx(kStatus) kDirectory_SetCurrentImpl(const kChar* directory);
kFx(kStatus) kDirectory_CurrentImpl(kChar* directory, kSize capacity); 
kFx(kStatus) kDirectory_ApplicationImpl(kChar* directory, kSize capacity);
kFx(kStatus) kDirectory_TempImpl(kChar* directory, kSize capacity);
kFx(kStatus) kDirectory_AppConfigImpl(const kChar* appName, kChar* directory, kSize capacity);
kFx(kStatus) kDirectory_AppDataImpl(const kChar* appName, kChar* directory, kSize capacity);
kFx(kStatus) kDirectory_PluginImpl(kChar* directory, kSize capacity);
kFx(kStatus) kDirectory_FromVirtualImpl(const kChar* vPath, kChar* path, kSize capacity);
kFx(kStatus) kDirectory_ToVirtualImpl(const kChar* path, kChar* vPath, kSize capacity);


kFx(kStatus) kDirectory_Enumerate(const kChar* directory, kBool includeFiles, kBool includeDirectories, kArrayList entries); 

//deprecated
kFx(kStatus) kDirectory_Files(const kChar* directory, kArrayList* files); 
kFx(kStatus) kDirectory_Directories(const kChar* directory, kArrayList* directories); 
kFx(kStatus) kDirectory_Entries(const kChar* directory, kArrayList* entries); 

kEndHeader()

#endif
