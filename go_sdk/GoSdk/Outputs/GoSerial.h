/** 
 * @file    GoSerial.h
 * @brief   Declares the GoSerial class. 
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SERIAL_H
#define GO_SDK_SERIAL_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoSerial
 * @extends kObject
 * @ingroup GoSdk-Serial
 * @brief   Represents Serial output settings.
 */
typedef kObject GoSerial;

/** 
 * Gets the count of serial protocol options.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.1.3.106
 * @param   serial          GoSerial object.
 * @return                  Protocol option.            
 */
GoFx(kSize) GoSerial_ProtocolOptionCount(GoSerial serial); 

/** 
 * Gets the serial protocol option at the specified index.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.1.3.106
 * @param   serial          GoSerial object.
 * @param   index           Protocol option index. 
 * @return                  Protocol option.            
 */
GoFx(k32u) GoSerial_ProtocolOptionAt(GoSerial serial, kSize index); 

/** 
 * Sets the Serial output protocol.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.1.3.106
 * @param   serial          GoSerial object.
 * @param   protocol        Protocol option value;
 * @return                  Operation status.            
 * @see                     GoSerial_ProtocolOptionCount, GoSerial_ProtocolOptionAt
 */
GoFx(kStatus) GoSerial_SetProtocol(GoSerial serial, k32u protocol);

/** 
 * Returns the current Serial output protocol value.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.1.3.106
 * @param   serial          GoSerial object.
 * @return  An integer corresponding to the current Serial output protocol.
 */
GoFx(k32u) GoSerial_Protocol(GoSerial serial);

/** 
 * Gets the number of source options.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  Source option.            
 */
GoFx(kSize) GoSerial_OptionCount(GoSerial serial); 

/** 
 * Gets the source option at the specified index.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   index           Source option index. 
 * @return                  Source option.            
 */
GoFx(k32u) GoSerial_OptionAt(GoSerial serial, kSize index); 

/** 
 * Gets the number of sources of the specified output type that are currently selected for transmission.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  Count of selected sources.            
 */
GoFx(kSize) GoSerial_SourceCount(GoSerial serial); 

/** 
 * Gets the identifier of the selected output source at the specified index.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   index           Selected source index.
 * @return                  Source identifier.            
 */
GoFx(k32u) GoSerial_SourceAt(GoSerial serial, kSize index); 

/** 
 * Selects the specified source for transmission.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   sourceId        Source identifier.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSerial_AddSource(GoSerial serial, k32u sourceId); 

/** 
 * Removes (de-selects) the source at the specified index.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   index           Index of the source to be removed.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSerial_RemoveSource(GoSerial serial, kSize index); 

/** 
 * Removes all selected sources for the specified output type.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSerial_ClearSources(GoSerial serial); 

/** 
 * Sets the Selcom protocol output rate.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   rate            The Selcom rate to use.
 * @return                  Operation status.            
 * @see                     GoSerial_SelcomRateOptionCount, GoSerial_SelcomRateOptionAt
 */
GoFx(kStatus) GoSerial_SetSelcomRate(GoSerial serial, k32u rate);

/** 
 * Gets the index of the currently selected Selcom output rate.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return  An integer corresponding to the current Selcom output rate.
 */
GoFx(k32u) GoSerial_SelcomRate(GoSerial serial);

/** 
 * Gets the list of Selcom rate options.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  An array list of Selcom rate options.
 */
GoFx(kArrayList) GoSerial_SelcomRateOptionList(GoSerial serial);

/** 
 * Returns the count of available Selcom protocol output rate options.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  Count of available Selcom protocol output rate options.
 */
GoFx(kSize) GoSerial_SelcomRateOptionCount(GoSerial serial);

/** 
 * Gets the Selcom rate option at the specified index.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   index           Selcom rate option index. 
 * @return                  Selcom rate option value.            
 */
GoFx(k32u) GoSerial_SelcomRateOptionAt(GoSerial serial, kSize index);

/** 
 * Sets the Selcom protocol output format.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   format          The output format to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSerial_SetSelcomFormat(GoSerial serial, GoSelcomFormat format);

/** 
 * Gets the current Selcom output format.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  The current Selcom output format.
 */
GoFx(GoSelcomFormat) GoSerial_SelcomFormat(GoSerial serial);

/** 
 * Gets the list of Selcom format options.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  An array list of Selcom format options.
 */
GoFx(kArrayList) GoSerial_SelcomFormatOptionList(GoSerial serial);

/** 
 * Returns the count of available Selcom protocol output format options.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  The count of available Selcom protocol output format options.
 */
GoFx(kSize) GoSerial_SelcomFormatOptionCount(GoSerial serial);

/** 
 * Gets the Selcom format option at the specified index.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   index           Selcom format option index. 
 * @return                  Selcom format option value.            
 */
GoFx(GoSelcomFormat) GoSerial_SelcomFormatOptionAt(GoSerial serial, kSize index);

/** 
 * Sets the Selcom protocol maximum data scale value.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   value           Maximum data scale value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSerial_SetSelcomDataScaleMax(GoSerial serial, k64f value);

/** 
 * Gets the Selcom protocol data scale maximum value.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  Maximum data scale value.
 */
GoFx(k64f) GoSerial_SelcomDataScaleMax(GoSerial serial);

/** 
 * Sets the Selcom protocol minimum data scale value.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   value           Minimum data scale value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSerial_SetSelcomDataScaleMin(GoSerial serial, k64f value);

/** 
 * Gets the Selcom protocol data scale minimum value.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  Minimum data scale value.
 */
GoFx(k64f) GoSerial_SelcomDataScaleMin(GoSerial serial);

/** 
 * Gets the ASCII protocol output delimiter string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  A pointer to the string representing the ASCII protocol output delimiter.
 */
GoFx(kChar*) GoSerial_AsciiDelimiter(GoSerial serial);

/** 
 * Sets the ASCII protocol output delimiter string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   string          A pointer to the string representing the ASCII protocol output delimiter.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSerial_SetAsciiDelimiter(GoSerial serial, const kChar* string);

/** 
 * Gets the ASCII protocol output terminator string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  A pointer to the terminator representing the ASCII protocol output delimiter.
 */
GoFx(kChar*) GoSerial_AsciiTerminator(GoSerial serial);

/** 
 * Sets the ASCII protocol output terminator string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   string          A pointer to the string representing the ASCII protocol output terminator.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSerial_SetAsciiTerminator(GoSerial serial, const kChar* string);

/** 
 * Gets the ASCII protocol output invalid value string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  A pointer to the string representing the ASCII protocol output invalid value.
 */
GoFx(kChar*) GoSerial_AsciiInvalidValue(GoSerial serial);

/** 
 * Sets the ASCII protocol output invalid value string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   string          A pointer to the string representing the ASCII protocol output invalid value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSerial_SetAsciiInvalidValue(GoSerial serial, const kChar* string);

/** 
 * Gets the ASCII protocol output custom data format string.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  A pointer to the string representing the ASCII protocol output invalid value.
 */
GoFx(kChar*) GoSerial_AsciiCustomDataFormat(GoSerial serial);

/** 
 * Sets the ASCII protocol output custom data format.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   string          A pointer to the string representing the ASCII protocol output custom data format.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSerial_SetAsciiCustomDataFormat(GoSerial serial, const kChar* string);

/** 
 * Sets the enabled state of the ASCII protocol custom data format.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @param   enabled         kTRUE to enable and kFALSE to disable.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSerial_EnableAsciiCustomFormat(GoSerial serial, kBool enabled);

/** 
 * Gets the enabled state of the ASCII protocol custom data format.
 *
 * @public                  @memberof GoSerial
 * @version                 Introduced in firmware 4.0.10.27
 * @param   serial          GoSerial object.
 * @return                  kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSerial_AsciiCustomFormatEnabled(GoSerial serial);

kEndHeader()
#include <GoSdk/Outputs/GoSerial.x.h>

#endif