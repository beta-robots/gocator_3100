/** 
 * @file    GoSdkLib.h
 * @brief   Gocator SDK library management functions. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_LIB_H
#define GO_SDK_LIB_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/kAssembly.h>
kBeginHeader()

/**
 */

/** 
 * Constructs the Gocator SDK library. 
 *
 * This function should be called prior to calling any other Gocator SDK functions. 
 * When the library is no longer needed, call kObject_Destroy on the assembly object
 * that is returned by this function.
 * 
 * This function can safely be called multiple times.  In order to ensure 
 * final cleanup, kObject_Destroy must be invoked a corresponding number of times. 
 *
 * @public
 * @version             Introduced in firmware 4.0.10.27
 * @param   assembly    Receives an assembly object representing the GoSdk library.
 * @return              Operation status. 
 */
GoFx(kStatus) GoSdk_Construct(kAssembly* assembly);

kEndHeader()
#include <GoSdk/GoSdkLib.x.h>

#endif 
