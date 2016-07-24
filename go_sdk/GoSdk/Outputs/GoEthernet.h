/** 
 * @file    GoEthernet.h
 * @brief   Declares the GoEthernet class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_ETHERNET_H
#define GO_SDK_ETHERNET_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()


/**
 * @class   GoEthernet
 * @extends kObject
 * @ingroup GoSdk-Ethernet
 * @brief   Represents Ethernet output settings.
 */
typedef kObject GoEthernet; 

/** 
 * Gets the number of source options for the specified output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @return                  Count of source options.            
 */
GoFx(kSize) GoEthernet_OptionCount(GoEthernet ethernet, GoOutputSource type);

/** 
 * Gets the source option at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           Source option index. 
 * @return                  Source option.            
 */
GoFx(k32u) GoEthernet_OptionAt(GoEthernet ethernet, GoOutputSource type, kSize index);

/** 
 * Gets the number of sources of the specified output type that are currently selected for transmission.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @return                  Count of selected sources.            
 */
GoFx(kSize) GoEthernet_SourceCount(GoEthernet ethernet, GoOutputSource type); 

/** 
 * Gets the identifier of the selected output at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           Selected source index.
 * @return                  Source identifier.            
 */
GoFx(k32u) GoEthernet_SourceAt(GoEthernet ethernet, GoOutputSource type, kSize index); 

/** 
 * Selects the specified source for transmission.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   sourceId        Output source identifier.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_AddSource(GoEthernet ethernet, GoOutputSource type, k32u sourceId); 

/** 
 * Removes (deselects) the source at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           Index of the source to be removed.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_RemoveSource(GoEthernet ethernet, GoOutputSource type, kSize index); 

/** 
 * Removes (deselects) all selected sources for the specified output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_ClearSources(GoEthernet ethernet, GoOutputSource type); 

/** 
 * Removes (deselects) all selected sources for all possible ethernet output types.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_ClearAllSources(GoEthernet ethernet); 

/** 
 * Gets the protocol that the ethernet utilizes for output.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  The ethernet protocol.            
 */
GoFx(GoEthernetProtocol) GoEthernet_Protocol(GoEthernet ethernet); 

/** 
 * Sets the protocol which will be output via ethernet.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   protocol        The selected ethernet protocol.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_SetProtocol(GoEthernet ethernet, GoEthernetProtocol protocol); 

/** 
 * Gets the ASCII protocol operational mode.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Operation mode.
 */
GoFx(GoAsciiOperation) GoEthernet_AsciiOperation(GoEthernet ethernet);

/** 
 * Sets the ASCII operation mode.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   mode            The selected ASCII operation mode.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_SetAsciiOperation(GoEthernet ethernet, GoAsciiOperation mode);

/** 
 * Gets the ASCII protocol control channel port number.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Control channel port number.
 */
GoFx(k32u) GoEthernet_AsciiControlPort(GoEthernet ethernet);

/** 
 * Sets the port number of the ASCII control port.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   port            The selected ASCII control port value.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_SetAsciiControlPort(GoEthernet ethernet, k32u port);

/** 
 * Gets the ASCII protocol health channel port number.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Health channel port number.
 */
GoFx(k32u) GoEthernet_AsciiHealthPort(GoEthernet ethernet);

/** 
 * Sets the port number of the ASCII health port.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   port            The selected ASCII health port value.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_SetAsciiHealthPort(GoEthernet ethernet, k32u port);

/** 
 * Gets the ASCII protocol data channel port number.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Data channel port number.
 */
GoFx(k32u) GoEthernet_AsciiDataPort(GoEthernet ethernet);

/** 
 * Sets the port number of the ASCII data port.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   port            The selected ASCII data port value.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoEthernet_SetAsciiDataPort(GoEthernet ethernet, k32u port);

/** 
 * Gets the ASCII protocol output delimiter string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the string representing the ASCII protocol output delimiter.
 */
GoFx(kChar*) GoEthernet_AsciiDelimiter(GoEthernet ethernet);

/** 
 * Sets the ASCII protocol output delimiter string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output delimiter.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiDelimiter(GoEthernet ethernet, const kChar* string);

/** 
 * Gets the ASCII protocol output terminator string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the terminator representing the ASCII protocol output delimiter.
 */
GoFx(kChar*) GoEthernet_AsciiTerminator(GoEthernet ethernet);

/** 
 * Sets the ASCII protocol output terminator string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output terminator.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiTerminator(GoEthernet ethernet, const kChar* string);

/** 
 * Gets the ASCII protocol output invalid value string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the string representing the ASCII protocol output invalid value.
 */
GoFx(kChar*) GoEthernet_AsciiInvalidValue(GoEthernet ethernet);

/** 
 * Sets the ASCII protocol output invalid value string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output invalid value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiInvalidValue(GoEthernet ethernet, const kChar* string);

/** 
 * Gets the ASCII protocol output custom data format string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the string representing the ASCII protocol output custom data format.
 */
GoFx(kChar*) GoEthernet_AsciiCustomDataFormat(GoEthernet ethernet);

/** 
 * Sets the ASCII protocol output custom data format string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output custom data format.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiCustomDataFormat(GoEthernet ethernet, const kChar* string);

/** 
 * Enables or disables the ASCII protocol output custom data format.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable custom data format output. kFALSE to use the default output format.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_EnableAsciiCustomFormat(GoEthernet ethernet, kBool enabled);

/** 
 * Returns the value of whether the ASCII protocol custom data format is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if the custom data format is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_AsciiCustomFormatEnabled(GoEthernet ethernet);

/** 
 * Enables or disables EthernetIP protocol output buffering.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable buffering. kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPBufferingEnabled(GoEthernet ethernet, kBool enabled);

/** 
 * Returns the value of whether the EthernetIP protocol output buffering is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if buffering is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_EIPBufferingEnabled(GoEthernet ethernet);

/** 
 * Sets the EthernetIP protocol endian output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.1.3.106
 * @param   ethernet        GoEthernet object.
 * @param   type            The endian output type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPEndianOutputType(GoEthernet ethernet, GoEndianType type);

/** 
 * Returns the value of the EthernetIP protocol endian output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.1.3.106
 * @param   ethernet        GoEthernet object.
 * @return                  Endian output type.
 */
GoFx(kBool) GoEthernet_EIPEndianOutputType(GoEthernet ethernet);

/** 
 * Sets the EthernetIP protocol implicit trigger override.
 *
 * @public                  @memberof GoEthernet
 * @version             Introduced in firmware 4.2.4.7
 * @param   ethernet        GoEthernet object.
 * @param   value           The implicit trigger override value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPImplicitTriggerOverride(GoEthernet ethernet, GoImplicitTriggerOverride value);

/** 
 * Returns the value of the EthernetIP protocol implicit trigger override.
 *
 * @public                  @memberof GoEthernet
 * @version             Introduced in firmware 4.2.4.7
 * @param   ethernet        GoEthernet object.
 * @return                  Implicit trigger override value.
 */
GoFx(GoImplicitTriggerOverride) GoEthernet_EIPImplicitTriggerOverride(GoEthernet ethernet);

/** 
 * Enables or disables Modbus protocol output buffering.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable buffering. kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetModbusBufferingEnabled(GoEthernet ethernet, kBool enabled);

/** 
 * Returns the value of whether Modbus protocol output buffering is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if buffering is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_ModbusBufferingEnabled(GoEthernet ethernet);

kEndHeader()
#include <GoSdk/Outputs/GoEthernet.x.h>

#endif
