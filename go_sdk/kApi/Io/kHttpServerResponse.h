/** 
 * @file    kHttpServerResponse.h
 * @brief   Declares the kHttpServerResponse class.
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_RESPONSE_H
#define K_API_HTTP_SERVER_RESPONSE_H

#include <kApi/Io/kNetwork.h>

kBeginHeader()

/**
 * @struct  kHttpStatus
 * @extends kValue
 * @ingroup kApi-Io
 * @brief   HTTP status code.
 */
typedef k32s kHttpStatus; 

/** @relates kHttpStatus @{ */
#define kHTTP_STATUS_CONTINUE                       (100)       ///< Continue.
#define kHTTP_STATUS_SWITCHING_PROTOCOLS            (101)       ///< Switching protocols. 
#define kHTTP_STATUS_OK                             (200)       ///< OK. 
#define kHTTP_STATUS_CREATED                        (201)       ///< Created. 
#define kHTTP_STATUS_ACCEPTED                       (202)       ///< Accepted. 
#define kHTTP_STATUS_NON_AUTHORITATIVE              (203)       ///< Non-authoritative information. 
#define kHTTP_STATUS_NO_CONTENT                     (204)       ///< No content. 
#define kHTTP_STATUS_RESET_CONTENT                  (205)       ///< Reset content. 
#define kHTTP_STATUS_PARTIAL_CONTENT                (206)       ///< Partial content. 
#define kHTTP_STATUS_MULTIPLE_CHOICES               (300)       ///< Multiple choices. 
#define kHTTP_STATUS_MOVED_PERMANENTLY              (301)       ///< Moved permanently. 
#define kHTTP_STATUS_FOUND                          (302)       ///< Found. 
#define kHTTP_STATUS_SEE_OTHER                      (303)       ///< See other. 
#define kHTTP_STATUS_NOT_MODIFIED                   (304)       ///< Not modified.
#define kHTTP_STATUS_USE_PROXY                      (305)       ///< Use proxy. 
#define kHTTP_STATUS_TEMPORARY_REDIRECT             (307)       ///< Temporary redirect. 
#define kHTTP_STATUS_BAD_REQUEST                    (400)       ///< Bad request. 
#define kHTTP_STATUS_UNAUTHORIZED                   (401)       ///< Unauthorized. 
#define kHTTP_STATUS_PAYMENT_REQUIRED               (402)       ///< Payment required. 
#define kHTTP_STATUS_FORBIDDEN                      (403)       ///< Forbidden. 
#define kHTTP_STATUS_NOT_FOUND                      (404)       ///< Not found. 
#define kHTTP_STATUS_METHOD_NOT_ALLOWED             (405)       ///< Method not allowed. 
#define kHTTP_STATUS_NOT_ACCEPTABLE                 (406)       ///< Not acceptable. 
#define kHTTP_STATUS_PROXY_AUTH_REQUIRED            (407)       ///< Proxy authentication required. 
#define kHTTP_STATUS_REQUEST_TIMEOUT                (408)       ///< Request timeout. 
#define kHTTP_STATUS_CONFLICT                       (409)       ///< Conflict. 
#define kHTTP_STATUS_GONE                           (410)       ///< Gone. 
#define kHTTP_STATUS_LENGTH_REQUIRED                (411)       ///< Length required. 
#define kHTTP_STATUS_PRECONDITION_FAILED            (412)       ///< Precondition failed. 
#define kHTTP_STATUS_REQUEST_ENTITY_SIZE            (413)       ///< Request entity too large. 
#define kHTTP_STATUS_REQUEST_URI_SIZE               (414)       ///< Request URI size too large. 
#define kHTTP_STATUS_UNSUPPORTED_MEDIA_TYPE         (415)       ///< Unsupported media type. 
#define kHTTP_STATUS_INVALID_RANGE                  (416)       ///< Requested range not satisfiable. 
#define kHTTP_STATUS_EXPECTATION_FAILED             (417)       ///< Expectation failed. 
#define kHTTP_STATUS_INTERNAL_SERVER_ERROR          (500)       ///< Internal server error. 
#define kHTTP_STATUS_NOT_IMPLEMENTED                (501)       ///< Not implemented. 
#define kHTTP_STATUS_BAD_GATEWAY                    (502)       ///< Bad gateway. 
#define kHTTP_STATUS_SERVICE_UNAVAILABLE            (503)       ///< Service unavailable. 
#define kHTTP_STATUS_GATEWAY_TIMEOUT                (504)       ///< Gateway timeout. 
#define kHTTP_STATUS_UNSUPPORTED_VERSION            (505)       ///< HTTP version not supported. 
/** @} */

/**
 * @class   kHttpServerResponse
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Supports HTTP server response formatting.
 */
//typedef kObject kHttpServerResponse;     --forward-declared in kApiDef.x.h 

/** 
 * Sets the version associated with this HTTP response. 
 * 
 * By default, version 1.1 is assumed. 
 * 
 * This function can only be called prior to writing the message body. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   version     Message version. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_SetVersion(kHttpServerResponse response, kVersion version); 

/** 
 * Sets the HTTP status code associated with this response. 
 * 
 * By default, status code 200 (OK) is assumed. 
 * 
 * This function can only be called prior to writing the message body. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   status      HTTP status code. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_SetStatus(kHttpServerResponse response, kHttpStatus status); 

/** 
 * Sets the HTTP status description associated with this response. 
 * 
 * If a custom description is not provided, a default description based on the status code will be sent. 
 * 
 * This function can only be called prior to writing the message body. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   reason      HTTP status description. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_SetReason(kHttpServerResponse response, const kChar* reason); 

/** 
 * Instructs the HTTP server to close this connection when message processing is complete.
 * 
 * This function can only be called prior to writing the message body. 
 * 
 * Use of this function will automatically add the 'connection: closed' header. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   closed      kTRUE to close the connection; kFALSE otherwise. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_SetClosed(kHttpServerResponse response, kBool closed); 

/** 
 * Adds a header value to the response. 
 * 
 * Headers that can be determined from the information provided in other functions (e.g. 'Content-Length', 
 * 'Transfer-Encoding', 'Connection') will be generated automatically and should not normally be provided via this function. 
 *  
 * If multiple headers with the same field name are provided, each subsequent header value will be 
 * added to the previous header values (comma-separated). 
 * 
 * Leading headers should be added prior to writing the message body. Trailing headers can be generated by 
 * calling this function after writing at least one chunk-encoded body segment. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   name        Header field name. 
 * @param   value       Header field value. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_AddHeader(kHttpServerResponse response, const kChar* name, const kChar* value); 

/** 
 * Sets the value of a header in the response. 
 * 
 * Headers that can be determined from the information provided in other functions (e.g. 'Content-Length', 
 * 'Transfer-Encoding', 'Connection') will be generated automatically and should not normally be provided via this function. 
 *  
 * Leading headers should be added prior to writing the message body. Trailing headers can be generated by 
 * calling this function after writing at least one chunk-encoded body segment. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   name        Header field name. 
 * @param   value       Header field value. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_SetHeader(kHttpServerResponse response, const kChar* name, const kChar* value);

/** 
 * Begins writing a simple message body. 
 * 
 * Use the stream provided by this function to write the message body. 
 * 
 * Use of this function will automatically add the 'Content-Length' header. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   length      Total length of the message body, in bytes.
 * @param   stream      Receives a reference to a stream object that should be used to write the message body. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_BeginWriteContent(kHttpServerResponse response, k64u length, kStream* stream); 

/** 
 * Begins writing a chunk-encoded message body segment. 
 * 
 * Use the stream provided by this function to write the message chunk. Call this function with a length 
 * of zero to signify that writing is complete. 
 * 
 * Use of this function will automatically add the 'Transfer-Encoding: chunked' header. 
 * 
 * @public              @memberof kHttpServerResponse
 * @param   response    Response object.
 * @param   length      Length of the message chunk, in bytes.
 * @param   stream      Receives a reference to a stream object that should be used to write the message chunk. 
 * @return              Operation status. 
 */
kFx(kStatus) kHttpServerResponse_BeginWriteChunk(kHttpServerResponse response, k64u length, kStream* stream); 

kEndHeader()

#include <kApi/Io/kHttpServerResponse.x.h>

#endif
