/** 
 * @file    kHttpServerRequest.x.h
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_REQUEST_X_H
#define K_API_HTTP_SERVER_REQUEST_X_H

kBeginHeader()

#define kHTTP_SERVER_REQUEST_LINE_CAPACITY          (8192)          //Maximum supported header line length (limits impact of client bugs). 
#define kHTTP_SERVER_REQUEST_HEADER_CAPACITY        (100)           //Maximum supported headers per message  (limits impact of client bugs). 

typedef struct kHttpServerRequestClass
{
    kObjectClass base; 
    kHttpServerChannel channel;             //Parent. 
    kTcpClient client;                      //TCP connection (belongs to parent channel). 

    kList headerLines;                      //List of parsed leading header lines -- kList<kString>.
    kMap headers;                           //Maps header field names to values -- kMap<kString, kString>. 

    kString method;                         //Method name from request line. 
    kString uri;                            //URI from request line. 
    kString uriPath;                        //URI path, parsed from request line uri. 
    kString versionStr;                     //Version string from request line.     
    kVersion version;                       //Version number parsed out from version string.
    k64s contentLength;                     //Content length header value (or -1, if not present). 
    kBool isChunkCoded;                     //Is the request body chunk-encoded?
    kBool expectContinue;                   //Does the client expect a 100-continue status message before sending actual response?

    kBool contentComplete;                  //Has the final body/content section been read out?
    kSize chunkIndex;                       //Current chunk index, used during body parsing.
    
    kString chunkLine;                      //Temp variable used when reading out chunk header. 
    kString findName;                       //Temp variable used when looking up header values. 
    kString tempStr;                        //Temp variable used for miscellaneous other purposes. 

} kHttpServerRequestClass;

kDeclareClass(k, kHttpServerRequest, kObject)

kFx(kStatus) kHttpServerRequest_Construct(kHttpServerRequest* request, kHttpServerChannel channel, kAlloc allocator); 

kFx(kStatus) kHttpServerRequest_Init(kHttpServerRequest request, kType type, kHttpServerChannel channel, kAlloc alloc); 
kFx(kStatus) kHttpServerRequest_VRelease(kHttpServerRequest request);

kFx(kStatus) kHttpServerRequest_Begin(kHttpServerRequest request);

kFx(kStatus) kHttpServerRequest_Clear(kHttpServerRequest request); 

kFx(kStatus) kHttpServerRequest_ReadHeaderLines(kHttpServerRequest request, kList headerLines); 
kFx(kStatus) kHttpServerRequest_CoalesceHeaderLines(kHttpServerRequest request, kList headerLines); 

kFx(kStatus) kHttpServerRequest_ParseRequestLine(kHttpServerRequest request); 
kFx(kStatus) kHttpServerRequest_ParseUriPath(kHttpServerRequest request, kString uri, kString uriPath); 

kFx(kStatus) kHttpServerRequest_ParseHeaders(kHttpServerRequest request, kList headerLines, kMap headerMap); 
kFx(kStatus) kHttpServerRequest_ParseHeader(kHttpServerRequest request, kString str, kMap headerMap); 

kFx(kStatus) kHttpServerRequest_ParseKnownHeaders(kHttpServerRequest request); 

kFx(kStatus) kHttpServerRequest_ReadLine(kHttpServerRequest request, kString line); 

kFx(kStatus) kHttpServerRequest_ExpectsContinue(kHttpServerRequest request); 

#define kHttpServerRequest_Cast_(R)         (kCastClass_(kHttpServerRequest, R))

kEndHeader()

#endif
