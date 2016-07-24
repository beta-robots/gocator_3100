/** 
 * @file    kHttpServerResponse.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServerResponse.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kHttpServer.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kTcpClient.h>

kBeginEnum(k, kHttpStatus, kValue)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_CONTINUE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_SWITCHING_PROTOCOLS)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_OK)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_CREATED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_ACCEPTED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_NON_AUTHORITATIVE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_NO_CONTENT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_RESET_CONTENT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_PARTIAL_CONTENT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_MULTIPLE_CHOICES)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_MOVED_PERMANENTLY)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_FOUND)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_SEE_OTHER)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_NOT_MODIFIED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_USE_PROXY)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_TEMPORARY_REDIRECT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_BAD_REQUEST)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_UNAUTHORIZED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_PAYMENT_REQUIRED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_FORBIDDEN)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_NOT_FOUND)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_METHOD_NOT_ALLOWED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_NOT_ACCEPTABLE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_PROXY_AUTH_REQUIRED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_REQUEST_TIMEOUT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_CONFLICT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_GONE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_LENGTH_REQUIRED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_PRECONDITION_FAILED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_REQUEST_ENTITY_SIZE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_REQUEST_URI_SIZE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_UNSUPPORTED_MEDIA_TYPE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_INVALID_RANGE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_EXPECTATION_FAILED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_INTERNAL_SERVER_ERROR)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_NOT_IMPLEMENTED)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_BAD_GATEWAY)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_SERVICE_UNAVAILABLE)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_GATEWAY_TIMEOUT)
    kAddEnumerator(kHttpStatus, kHTTP_STATUS_UNSUPPORTED_VERSION)
kEndEnum()

kBeginClass(k, kHttpServerResponse, kObject)
    kAddVMethod(kHttpServerResponse, kObject, VRelease)
kEndClass()

kFx(kStatus) kHttpServerResponse_Construct(kHttpServerResponse* response, kHttpServerChannel channel, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kHttpServerResponse); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, response)); 

    if (!kSuccess(status = kHttpServerResponse_Init(*response, type, channel, alloc)))
    {
        kAlloc_FreeRef(alloc, response); 
    }

    return status; 
} 

kFx(kStatus) kHttpServerResponse_Init(kHttpServerResponse response, kType type, kHttpServerChannel channel, kAlloc alloc)
{
    kHttpServerResponseClass* obj = response; 
    kStatus status; 

    kCheck(kObject_Init(response, type, alloc)); 

    kInitFields_(kHttpServerResponse, response); 

    obj->channel = channel; 
    obj->client = kHttpServerChannel_Client(channel); 

    kTry
    {
        kTest(kMap_Construct(&obj->headers, kTypeOf(kString), kTypeOf(kString), 0, alloc)); 

        kTest(kString_Construct(&obj->reason, kNULL, alloc));
        kTest(kString_Construct(&obj->line, kNULL, alloc));
    }
    kCatch(&status)
    {
        kHttpServerResponse_VRelease(response); 
        kEndCatch(status); 
    }
    
    return kOK;     
}

kFx(kStatus) kHttpServerResponse_VRelease(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    kCheck(kHttpServerResponse_Clear(response)); 

    kCheck(kObject_Destroy(obj->headers)); 

    kCheck(kObject_Destroy(obj->reason)); 
    kCheck(kObject_Destroy(obj->line)); 

    kCheck(kObject_VRelease(response)); 

    return kOK;   
}

kFx(kStatus) kHttpServerResponse_Begin(kHttpServerResponse response)
{
    return kHttpServerResponse_Clear(response); 
}

kFx(kStatus) kHttpServerResponse_Clear(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    kCheck(kMap_Purge(obj->headers)); 

    kCheck(kString_Clear(obj->reason)); 
    kCheck(kString_Clear(obj->line)); 

    obj->version = kVersion_Create(1, 1, 0, 0); 
    obj->status = kHTTP_STATUS_OK; 
    obj->contentLength = -1; 
    obj->isChunkCoded = kFALSE; 
    obj->shouldClose = kFALSE; 
    obj->messageStarted = kFALSE; 
    obj->chunkIndex = 0; 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_SetVersion(kHttpServerResponse response, kVersion version)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    obj->version = version; 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_SetStatus(kHttpServerResponse response, kHttpStatus status)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    obj->status = status; 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_SetReason(kHttpServerResponse response, const kChar* reason)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    return kString_Set(obj->reason, reason); 
}

kFx(kStatus) kHttpServerResponse_SetContentLength(kHttpServerResponse response, k64u length)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    obj->contentLength = length; 

    return kOK; 
} 

kFx(kStatus) kHttpServerResponse_EnableChunkCoding(kHttpServerResponse response, kBool enabled)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    obj->isChunkCoded = enabled; 

    return kOK; 
} 

kFx(kStatus) kHttpServerResponse_AddHeader(kHttpServerResponse response, const kChar* name, const kChar* value)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 
    kMap headerMap = obj->headers; 
    kMapItem headerItem = kNULL; 
    kString nameStr = kNULL; 
    kString valueStr = kNULL; 

    kTry
    {        
        kTest(kString_Construct(&nameStr, name, kObject_Alloc_(response))); 
        kTest(kHttpServerChannel_NormalizeHeaderCaps(kString_Chars(nameStr))); 
       
        if (kSuccess(kMap_FindItem(headerMap, &nameStr, &headerItem)))
        {
            kString existingValue = kMap_ValueAs_(headerMap, headerItem, kString); 

            kTest(kString_Addf(existingValue, ",%s", value)); 
        }
        else
        {
            kTest(kString_Construct(&valueStr, value, kObject_Alloc_(response))); 

            kTest(kMap_Add(headerMap, &nameStr, &valueStr)); 
            nameStr = valueStr = kNULL; 
        }
    }
    kFinally
    {
        kCheck(kObject_Destroy(nameStr)); 
        kCheck(kObject_Destroy(valueStr));

        kEndFinally(); 
    }
    
    return kOK; 
}

kFx(kStatus) kHttpServerResponse_SetHeader(kHttpServerResponse response, const kChar* name, const kChar* value)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 
    kMap headerMap = obj->headers; 
    kMapItem headerItem = kNULL; 
    kString nameStr = kNULL; 
    kString valueStr = kNULL; 

    kTry
    {        
        kTest(kString_Construct(&nameStr, name, kObject_Alloc_(response))); 
        kTest(kHttpServerChannel_NormalizeHeaderCaps(kString_Chars(nameStr))); 
       
        if (kSuccess(kMap_FindItem(headerMap, &nameStr, &headerItem)))
        {
            kString existingValue = kMap_ValueAs_(headerMap, headerItem, kString); 

            kTest(kString_Set(existingValue, value)); 
        }
        else
        {
            kTest(kString_Construct(&valueStr, value, kObject_Alloc_(response))); 

            kTest(kMap_Add(headerMap, &nameStr, &valueStr)); 
            nameStr = valueStr = kNULL; 
        }
    }
    kFinally
    {
        kCheck(kObject_Destroy(nameStr)); 
        kCheck(kObject_Destroy(valueStr));

        kEndFinally(); 
    }
    
    return kOK; 
}

kFx(kStatus) kHttpServerResponse_BeginWriteMessage(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    obj->messageStarted = kTRUE; 

    kCheck(kHttpServerResponse_FormatStatusLine(response));         

    kCheck(kHttpServerResponse_AddKnownHeaders(response)); 
    kCheck(kHttpServerResponse_FormatHeaders(response, obj->headers));  

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_AddKnownHeaders(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 
    kText128 buffer; 

    if (obj->contentLength >= 0)
    {
        kCheck(kStrPrintf(buffer, kCountOf(buffer), "%llu", obj->contentLength)); 
        kCheck(kHttpServerResponse_SetHeader(response, "Content-Length", buffer)); 
    }

    if (obj->isChunkCoded)
    {
        kCheck(kHttpServerResponse_AddHeader(response, "Transfer-Encoding", "chunked")); 
    }

    if (obj->shouldClose)
    {
        kCheck(kHttpServerResponse_AddHeader(response, "Connection", "close")); 
    }

    return kOK; 
}

const kChar* kHttpServerResponse_DefaultReason(kHttpServerResponse response, kHttpStatus status)
{
    switch (status)
    {
        case kHTTP_STATUS_CONTINUE:                 return "Continue"; 
        case kHTTP_STATUS_SWITCHING_PROTOCOLS:      return "Switching protocols"; 
        case kHTTP_STATUS_OK:                       return "OK"; 
        case kHTTP_STATUS_CREATED:                  return "Created"; 
        case kHTTP_STATUS_ACCEPTED:                 return "Accepted"; 
        case kHTTP_STATUS_NON_AUTHORITATIVE:        return "Non-authoritative information"; 
        case kHTTP_STATUS_NO_CONTENT:               return "No content"; 
        case kHTTP_STATUS_RESET_CONTENT:            return "Reset content"; 
        case kHTTP_STATUS_PARTIAL_CONTENT:          return "Partial content"; 
        case kHTTP_STATUS_MULTIPLE_CHOICES:         return "Multiple choices"; 
        case kHTTP_STATUS_MOVED_PERMANENTLY:        return "Moved permanently"; 
        case kHTTP_STATUS_FOUND:                    return "Found"; 
        case kHTTP_STATUS_SEE_OTHER:                return "See other"; 
        case kHTTP_STATUS_NOT_MODIFIED:             return "Not modified"; 
        case kHTTP_STATUS_USE_PROXY:                return "Use proxy"; 
        case kHTTP_STATUS_TEMPORARY_REDIRECT:       return "Temporary redirect"; 
        case kHTTP_STATUS_BAD_REQUEST:              return "Bad request"; 
        case kHTTP_STATUS_UNAUTHORIZED:             return "Unauthorized"; 
        case kHTTP_STATUS_PAYMENT_REQUIRED:         return "Payment required"; 
        case kHTTP_STATUS_FORBIDDEN:                return "Forbidden"; 
        case kHTTP_STATUS_NOT_FOUND:                return "Not found"; 
        case kHTTP_STATUS_METHOD_NOT_ALLOWED:       return "Method not allowed"; 
        case kHTTP_STATUS_NOT_ACCEPTABLE:           return "Not acceptable"; 
        case kHTTP_STATUS_PROXY_AUTH_REQUIRED:      return "Proxy authentication required"; 
        case kHTTP_STATUS_REQUEST_TIMEOUT:          return "Request timeout"; 
        case kHTTP_STATUS_CONFLICT:                 return "Conflict"; 
        case kHTTP_STATUS_GONE:                     return "Gone"; 
        case kHTTP_STATUS_LENGTH_REQUIRED:          return "Length required"; 
        case kHTTP_STATUS_PRECONDITION_FAILED:      return "Precondition failed"; 
        case kHTTP_STATUS_REQUEST_ENTITY_SIZE:      return "Request entity too large"; 
        case kHTTP_STATUS_REQUEST_URI_SIZE:         return "Request URI size too large"; 
        case kHTTP_STATUS_UNSUPPORTED_MEDIA_TYPE:   return "Unsupported media type"; 
        case kHTTP_STATUS_INVALID_RANGE:            return "Requested range not satisfiable"; 
        case kHTTP_STATUS_EXPECTATION_FAILED:       return "Expectation failed"; 
        case kHTTP_STATUS_INTERNAL_SERVER_ERROR:    return "Internal server error"; 
        case kHTTP_STATUS_NOT_IMPLEMENTED:          return "Not implemented"; 
        case kHTTP_STATUS_BAD_GATEWAY:              return "Bad gateway"; 
        case kHTTP_STATUS_SERVICE_UNAVAILABLE:      return "Service unavailable"; 
        case kHTTP_STATUS_GATEWAY_TIMEOUT:          return "Gateway timeout"; 
        case kHTTP_STATUS_UNSUPPORTED_VERSION:      return "HTTP version not supported"; 
        default:                                    return "Unknown";
    }
}

kFx(kStatus) kHttpServerResponse_FormatStatusLine(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    if (kString_Length(obj->reason) == 0)
    {
        kCheck(kString_Set(obj->reason, kHttpServerResponse_DefaultReason(response, obj->status))); 
    }

    kCheck(kString_Setf(obj->line, "HTTP/%u.%u %d %s\r\n", kVersion_Major(obj->version), kVersion_Minor(obj->version), obj->status, kString_Chars(obj->reason))); 
   
    kCheck(kStream_Write(obj->client, kString_Chars(obj->line), kString_Length(obj->line))); 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_FormatHeaders(kHttpServerResponse response, kMap headerMap)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 
    kMapItem header = kMap_First(headerMap); 

    //write all headers to the stream
    while (!kIsNull(header))
    {
        kString name = kMap_KeyAs_(headerMap, header, kString); 
        kString value = kMap_ValueAs_(headerMap, header, kString); 

        kCheck(kString_Setf(obj->line, "%s: %s\r\n", kString_Chars(name), kString_Chars(value))); 
        kCheck(kStream_Write(obj->client, kString_Chars(obj->line), kString_Length(obj->line))); 

        header = kMap_Next(headerMap, header); 
    }

    //add trailing crlf
    kCheck(kString_Set(obj->line, "\r\n"));
    kCheck(kStream_Write(obj->client, kString_Chars(obj->line), kString_Length(obj->line))); 

    //clear out headers
    kCheck(kMap_Purge(obj->headers)); 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_SetClosed(kHttpServerResponse response, kBool closed)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    obj->shouldClose = closed; 

    return kOK; 
}

kFx(kBool) kHttpServerResponse_Closed(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    return obj->shouldClose; 
}

kFx(kBool) kHttpServerResponse_MessageStarted(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    return obj->messageStarted; 
}

kFx(kStatus) kHttpServerResponse_BeginWriteContent(kHttpServerResponse response, k64u length, kStream* stream)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    if (!kIsNull(stream))
    {
        *stream = obj->client; 
    }

    kCheckState(!obj->messageStarted); 

    kCheck(kHttpServerResponse_SetContentLength(response, length)); 
    kCheck(kHttpServerResponse_BeginWriteMessage(response)); 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_BeginWriteChunk(kHttpServerResponse response, k64u length, kStream* stream)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    if (!kIsNull(stream))
    {
        *stream = obj->client; 
    }

    if (!obj->messageStarted)
    {
        kCheck(kHttpServerResponse_EnableChunkCoding(response, kTRUE)); 
        kCheck(kHttpServerResponse_BeginWriteMessage(response)); 
    }

    if (obj->chunkIndex != 0)
    {
        //write terminating CRLF from previous chunk
        kCheck(kString_Set(obj->line, "\r\n")); 
        kCheck(kStream_Write(obj->client, kString_Chars(obj->line), kString_Length(obj->line))); 
    }

    //write chunk length
    kCheck(kString_Setf(obj->line, "%llX\r\n", length)); 
    kCheck(kStream_Write(obj->client, kString_Chars(obj->line), kString_Length(obj->line))); 
    
    obj->chunkIndex++; 

    return kOK; 
}

kFx(kStatus) kHttpServerResponse_End(kHttpServerResponse response)
{
    kHttpServerResponseClass* obj = kHttpServerResponse_Cast_(response); 

    //if the response has no body, the headers may not have been written at this point
    if (!obj->messageStarted)
    {
        kCheck(kHttpServerResponse_BeginWriteMessage(response)); 
    }

    //if the response has a chunk-encoded body, write any final trailing headers here
    if (obj->isChunkCoded)
    {       
        kCheck(kHttpServerResponse_FormatHeaders(response, obj->headers)); 
    }

    //flush any outgoing buffered bytes to the stream
    kCheck(kStream_Flush(obj->client)); 

    return kOK;   
}
