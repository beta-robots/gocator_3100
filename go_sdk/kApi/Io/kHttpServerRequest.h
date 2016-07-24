/** 
 * @file    kHttpServerRequest.h
 * @brief   Declares the kHttpServerRequest class. 
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_REQUEST_H
#define K_API_HTTP_SERVER_REQUEST_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @class   kHttpServerRequest
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Supports HTTP server request parsing.
 */
//typedef kObject kHttpServerRequest;     --forward-declared in kApiDef.x.h 

/** 
 * Returns a string representing the HTTP request method (e.g. "GET", "POST"). 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              Method string. 
 */
kFx(const kChar*) kHttpServerRequest_Method(kHttpServerRequest request); 

/** 
 * Returns a string representing the HTTP request URI (/resources/page.html). 
 * 
 * The URI can be in absolute URI form (http://www.example.com/index.html) or absolute path form (/index.html). 
 * Use the kHttpServerRequest_UriPath function to access the URI in absolute path form. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              URI string. 
 */
kFx(const kChar*) kHttpServerRequest_Uri(kHttpServerRequest request); 

/** 
 * Returns a string representing the HTTP request URI in absolute path form (/resources/page.html). 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              Absolute path URI string. 
 */
kFx(const kChar*) kHttpServerRequest_UriPath(kHttpServerRequest request); 

/** 
 * Returns a value representing the HTTP version associated with this request.  
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              Request version.
 */
kFx(kVersion) kHttpServerRequest_Version(kHttpServerRequest request); 

/** 
 * Returns the total count of headers parsed from this request.  
 *
 * The headers reported by this function can include both leading and trailing headers. However, trailing headers 
 * are only reported after the final segment of a chunk-encoded message is parsed. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              Header count.
 */
kFx(kSize) kHttpServerRequest_HeaderCount(kHttpServerRequest request); 

/** 
 * Gets a reference to the first header. 
 *
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              First header, or kNULL). 
 */
kFx(kPointer) kHttpServerRequest_FirstHeader(kHttpServerRequest request); 

/** 
 * Given a header reference, gets a reference to the next header. 
 *
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @param   header      Current header. 
 * @return              Next header, or kNULL). 
 */
kFx(kPointer) kHttpServerRequest_NextHeader(kHttpServerRequest request, kPointer header); 

/** 
 * Gets the field name associated with a header reference. 
 * 
 * HTTP header field names are case-insensitive. To avoid ambiguity, the kHttpServerRequest
 * class converts all header names to Pascal caps (e.g. "Content-Length"). 
 *
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @param   header      Header reference. 
 * @return              Header field name. 
 */
kFx(const kChar*) kHttpServerRequest_HeaderName(kHttpServerRequest request, kPointer header); 

/** 
 * Gets the field value associated with a header reference. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @param   header      Header reference. 
 * @return              Header field value. 
 */
kFx(const kChar*) kHttpServerRequest_HeaderValue(kHttpServerRequest request, kPointer header); 

/** 
 * Finds the header field value associated with the given header field name, if present. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @param   name        Header field name. 
 * @return              Header field value, or kNULL. 
 */
kFx(const kChar*) kHttpServerRequest_FindHeaderValue(kHttpServerRequest request, const kChar* name); 

/** 
 * Reports whether the request has an associated message body. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              kTRUE if a message body is present, kFALSE otherwise. 
 */
kFx(kBool) kHttpServerRequest_HasBody(kHttpServerRequest request); 

/** 
 * Reports whether the message body is chunk-encoded. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              kTRUE if message body is chunk-encoded; kFALSE otherwise. 
 */
kFx(kBool) kHttpServerRequest_IsChunkCoded(kHttpServerRequest request); 

/** 
 * Reports the total message length for a simple (non-chunk-encoded) message. 
 * 
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @return              Message length, in bytes, or -1 if not applicable. 
 */
kFx(k64s) kHttpServerRequest_ContentLength(kHttpServerRequest request); 

/** 
 * Begins reading the message body. 
 *
 * For simple messages, call this function once to receive the total message length and a reference to 
 * a stream object that can be used to read the entire message.
 * 
 * For chunk-encoded messages, call this function to receive the length of the next chunk and a reference to 
 * a stream object that can be used to read the chunk. Each individual chunk must be read out before this 
 * function can be used to learn about the next chunk.  Reading is complete when a length of zero is reported 
 * by this function.
 *
 * @public              @memberof kHttpServerRequest
 * @param   request     Request object.
 * @param   length      Receives the amount of data, in bytes, that should be read from the stream.
 * @param   stream      Receives a reference to a stream object that should be used to read message content.
 * @return              Operation status.  
 */
kFx(kStatus) kHttpServerRequest_BeginRead(kHttpServerRequest request, k64u* length, kStream* stream); 

kEndHeader()

#include <kApi/Io/kHttpServerRequest.x.h>

#endif
