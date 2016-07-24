/** 
 * @file    kPlugin.h
 * @brief   Declares the kPlugin class. 
 *
 * @internal
 * Copyright (C) 2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PLUGIN_H
#define K_API_PLUGIN_H

#include <kApi/kApiDef.h>

kBeginHeader()

/**
 * @class   kPlugin
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Represents a dynamically loaded assembly. 
 * 
 * This class relies on OS support for dynamic loading, and is only available on platforms that 
 * provide underlying support.
 */
//typedef kObject kPlugin;             --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kPlugin object.
 * 
 * @public              @memberof kPlugin
 * @param   plugin      Destination for the constructed object handle. 
 * @param   path        Path to the plugin library. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kPlugin_Construct(kPlugin* plugin, const kChar* path, kAlloc allocator);

/** 
 * Gets the assembly associated with this plugin.
 *
 * @public              @memberof kPlugin
 * @param   plugin      Plugin object.
 * @return              Assembly object. 
 */
kFx(kAssembly) kPlugin_Assembly(kPlugin plugin);

kEndHeader()

#include <kApi/Utils/kPlugin.x.h>

#endif
