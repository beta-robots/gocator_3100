/** 
 * @file    GoEthernet.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_ETHERNET_X_H
#define GO_SDK_ETHERNET_X_H

#include <GoSdk/Outputs/GoEthernet.h>
#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>
kBeginHeader()

#define GO_OUTPUT_SOURCE_TEXT_CAPACITY          (2048)

typedef struct GoEthernetClass
{   
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    kArrayList videoOptions; 
    kArrayList videoSources; 
    kArrayList rangeOptions; 
    kArrayList rangeSources; 
    kArrayList rangeIntensityOptions; 
    kArrayList rangeIntensitySources; 
    kArrayList profileOptions; 
    kArrayList profileSources; 
    kArrayList profileIntensityOptions; 
    kArrayList profileIntensitySources; 
    kArrayList surfaceIntensityOptions; 
    kArrayList surfaceIntensitySources; 
    kArrayList surfaceOptions; 
    kArrayList surfaceSources; 
    kArrayList measurementOptions; 
    kArrayList measurementSources; 

    GoEthernetProtocol protocol; 
    GoEipConfig eip;
    GoModbusConfig modbus;
    GoAsciiConfig ascii;
} GoEthernetClass; 

kDeclareClass(Go, GoEthernet, kObject)

#define GoEthernet_Cast_(CONTEXT)    kCastClass_(GoEthernet, CONTEXT)

GoFx(kStatus) GoEthernet_Construct(GoEthernet* ethernet, kObject sensor, kAlloc allocator); 

GoFx(kStatus) GoEthernet_Init(GoEthernet ethernet, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoEthernet_VRelease(GoEthernet ethernet); 

GoFx(kStatus) GoEthernet_Read(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_Write(GoEthernet ethernet, kXml xml, kXmlItem item); 

GoFx(kArrayList) GoEthernet_SourceList(GoEthernet ethernet, GoOutputSource type);
GoFx(kArrayList) GoEthernet_OptionList(GoEthernet ethernet, GoOutputSource type);

/** 
 * Enables or disables EthernetIP protocol implicit output.
 *
 * @public                  @memberof GoEthernet
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable implicit output. kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPImplicitOutputEnabled(GoEthernet ethernet, kBool enabled);

/** 
 * Returns the value of whether the EthernetIP protocol implicit output is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if implicit output is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_EIPImplicitOutputEnabled(GoEthernet ethernet);


kEndHeader()

#endif