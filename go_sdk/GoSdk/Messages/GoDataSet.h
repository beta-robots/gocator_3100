/** 
 * @file    GoDataSet.h
 * @brief   Declares the GoDataSet class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DATA_SET_H
#define GO_SDK_DATA_SET_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoDataSet
 * @extends kObject
 * @ingroup GoSdk-Data
 * @brief   Represents a collection of data channel or health channel messages. 
 */
typedef kObject GoDataSet; 

/** 
 * Gets the sender ID (serial number) associated with this message collection.
 *
 * @public             @memberof GoDataSet
 * @version            Introduced in firmware 4.0.10.27
 * @param   set        Message collection. 
 * @return             Sender ID.
 */
GoFx(k32u) GoDataSet_SenderId(GoDataSet set);

/** 
 * Returns the message count in this collection. 
 *
 * @public             @memberof GoDataSet
 * @version            Introduced in firmware 4.0.10.27
 * @param   set        Message collection. 
 * @return             Count of messages.
 */
GoFx(kSize) GoDataSet_Count(GoDataSet set);

/** 
 * Gets the message at the specified index.
 *
 * @public             @memberof GoDataSet
 * @version            Introduced in firmware 4.0.10.27
 * @param   set        Message collection. 
 * @param   index      Message index.
 * @return             Message object.
 */
GoFx(kObject) GoDataSet_At(GoDataSet set, kSize index);

kEndHeader()
#include <GoSdk/Messages/GoDataSet.x.h>

#endif
