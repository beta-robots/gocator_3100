/** 
 * @file    kApi.h
 * @brief   Includes all public Zen headers. 
 *
 * @internal
 * Copyright (C) 2008-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_H
#define K_API_API_H

#include <kApi/kAlloc.h>
#include <kApi/kApiDef.h>
#include <kApi/kApiLib.h>
#include <kApi/kApiVersion.h>
#include <kApi/kAssembly.h>
#include <kApi/kObject.h>
#include <kApi/kType.h>
#include <kApi/kValue.h>

#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArray2.h>
#include <kApi/Data/kArray3.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kBox.h>
#include <kApi/Data/kBytes.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Data/kImage.h>
#include <kApi/Data/kList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kMath.h>
#include <kApi/Data/kQueue.h>
#include <kApi/Data/kString.h>
#include <kApi/Data/kXml.h>

#include <kApi/Io/kDat5Serializer.h>
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Io/kDirectory.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kHttpServer.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kHttpServerRequest.h>
#include <kApi/Io/kHttpServerResponse.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kNetwork.h>
#include <kApi/Io/kPath.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Io/kStream.h>
#include <kApi/Io/kTcpClient.h>
#include <kApi/Io/kTcpServer.h>
#include <kApi/Io/kUdpClient.h>

#include <kApi/Threads/kAtomic.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kMsgQueue.h>
#include <kApi/Threads/kPeriodic.h>
#include <kApi/Threads/kSemaphore.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Threads/kTimer.h>

#include <kApi/Utils/kBackTrace.h>
#include <kApi/Utils/kDebugAlloc.h>
#include <kApi/Utils/kDynamicLib.h>
#include <kApi/Utils/kEvent.h>
#include <kApi/Utils/kObjectPool.h>
#include <kApi/Utils/kPlugin.h>
#include <kApi/Utils/kPoolAlloc.h>
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/Utils/kUtils.h>

#endif
