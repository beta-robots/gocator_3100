/** 
 * @file    kHttpServerResponse.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_RESPONSE_X_H
#define K_API_HTTP_SERVER_RESPONSE_X_H

kBeginHeader()

kDeclareEnum(k, kHttpStatus, kValue)

typedef struct kHttpServerResponseClass
{
    kObjectClass base; 
    kHttpServerChannel channel;         //Parent. 
    kTcpClient client;                  //TCP connection (belongs to parent channel). 

    kMap headers;                       //Maps header field names to values -- kMap<kString, kString>.

    kVersion version;                   //HTTP version.  
    kHttpStatus status;                 //HTTP status code.  
    kString reason;                     //HTTP status reason. 
    kBool shouldClose;                  //Should the connection be closed after this response?
    kBool isChunkCoded;                 //Is the response body chunk-coded?
    k64s contentLength;                 //Content length header value (or -1, if not present). 

    kBool messageStarted;               //Have headers been written?
    kSize chunkIndex;                   //Current chunk index, used during body formatting.

    kString line;                       //Temp variable used for output formatting.

} kHttpServerResponseClass;

kDeclareClass(k, kHttpServerResponse, kObject)

kFx(kStatus) kHttpServerResponse_Construct(kHttpServerResponse* response, kHttpServerChannel channel, kAlloc allocator); 

kFx(kStatus) kHttpServerResponse_Init(kHttpServerResponse response, kType type, kHttpServerChannel channel, kAlloc alloc); 
kFx(kStatus) kHttpServerResponse_VRelease(kHttpServerResponse response);

kFx(kStatus) kHttpServerResponse_Begin(kHttpServerResponse response);
kFx(kStatus) kHttpServerResponse_Clear(kHttpServerResponse response);

kFx(kStatus) kHttpServerResponse_SetContentLength(kHttpServerResponse response, k64u length); 
kFx(kStatus) kHttpServerResponse_EnableChunkCoding(kHttpServerResponse response, kBool enabled); 

kFx(kStatus) kHttpServerResponse_BeginWriteMessage(kHttpServerResponse response); 

kFx(kStatus) kHttpServerResponse_AddKnownHeaders(kHttpServerResponse response); 
const kChar* kHttpServerResponse_DefaultReason(kHttpServerResponse response, kHttpStatus status); 

kFx(kStatus) kHttpServerResponse_FormatStatusLine(kHttpServerResponse response); 
kFx(kStatus) kHttpServerResponse_FormatHeaders(kHttpServerResponse response, kMap headerMap); 

kFx(kStatus) kHttpServerResponse_End(kHttpServerResponse response);

kFx(kBool) kHttpServerResponse_MessageStarted(kHttpServerResponse response);
kFx(kBool) kHttpServerResponse_Closed(kHttpServerResponse response); 

#define kHttpServerResponse_Cast_(R)            (kCastClass_(kHttpServerResponse, R))

kEndHeader()

#endif
