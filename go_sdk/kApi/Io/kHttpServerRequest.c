/** 
 * @file    kHttpServerRequest.c
 *
 * @internal
 * Copyright (C) 2013-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServerRequest.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kHttpServer.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kTcpClient.h>
#include <stdio.h>
#include <ctype.h>

kBeginClass(k, kHttpServerRequest, kObject)
    kAddVMethod(kHttpServerRequest, kObject, VRelease)
kEndClass()

kFx(kStatus) kHttpServerRequest_Construct(kHttpServerRequest* request, kHttpServerChannel channel, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kType type = kTypeOf(kHttpServerRequest); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, request)); 

    if (!kSuccess(status = kHttpServerRequest_Init(*request, type, channel, alloc)))
    {
        kAlloc_FreeRef(alloc, request); 
    }

    return status; 
} 

kFx(kStatus) kHttpServerRequest_Init(kHttpServerRequest request, kType type, kHttpServerChannel channel, kAlloc alloc)
{
    kHttpServerRequestClass* obj = request; 
    kStatus status; 

    kCheck(kObject_Init(request, type, alloc)); 

    kInitFields_(kHttpServerRequest, request); 
    
    obj->channel = channel; 
    obj->client = kHttpServerChannel_Client(channel); 

    kTry
    {
        kTest(kList_Construct(&obj->headerLines, kTypeOf(kString), kHTTP_SERVER_REQUEST_HEADER_CAPACITY, alloc));    
        kTest(kMap_Construct(&obj->headers, kTypeOf(kString), kTypeOf(kString), kHTTP_SERVER_REQUEST_HEADER_CAPACITY, alloc));    

        kTest(kString_Construct(&obj->findName, kNULL, alloc)); 
        kTest(kString_Construct(&obj->chunkLine, kNULL, alloc)); 
        kTest(kString_Construct(&obj->tempStr, kNULL, alloc)); 
    }
    kCatch(&status)
    {
        kHttpServerRequest_VRelease(request); 
        kEndCatch(status); 
    }

    return kOK;     
}

kFx(kStatus) kHttpServerRequest_VRelease(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    kCheck(kHttpServerRequest_Clear(request)); 

    kCheck(kObject_Destroy(obj->headerLines));  
    kCheck(kObject_Destroy(obj->headers));  

    kCheck(kObject_Destroy(obj->findName));  
    kCheck(kObject_Destroy(obj->chunkLine));  
    kCheck(kObject_Destroy(obj->tempStr));  

    kCheck(kObject_VRelease(request)); 

    return kOK;   
}

kFx(kStatus) kHttpServerRequest_Begin(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    kCheck(kHttpServerRequest_Clear(request)); 

    kCheck(kHttpServerRequest_ReadHeaderLines(request, obj->headerLines)); 

    kCheck(kHttpServerRequest_ParseRequestLine(request)); 

    kCheck(kHttpServerRequest_CoalesceHeaderLines(request, obj->headerLines)); 

    kCheck(kHttpServerRequest_ParseHeaders(request, obj->headerLines, obj->headers)); 
    kCheck(kHttpServerRequest_ParseKnownHeaders(request)); 

    return kOK;   
}

kFx(kStatus) kHttpServerRequest_Clear(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    kCheck(kList_Purge(obj->headerLines)); 
    kCheck(kMap_Purge(obj->headers)); 
    
    kCheck(kDisposeRef(&obj->method)); 
    kCheck(kDisposeRef(&obj->uri)); 
    kCheck(kDisposeRef(&obj->uriPath)); 
    kCheck(kDisposeRef(&obj->versionStr)); 

    obj->contentLength = -1; 
    obj->isChunkCoded = kFALSE; 
    obj->contentComplete = kFALSE; 
    obj->chunkIndex = 0; 
    obj->expectContinue = kFALSE; 
    
    return kOK;
}

kFx(kStatus) kHttpServerRequest_ReadHeaderLines(kHttpServerRequest request, kList headerLines)
{
    kString line = kNULL; 

    kCheck(kList_Purge(headerLines)); 

    do 
    {
        kCheck(kString_Construct(&line, kNULL, kObject_Alloc_(request))); 
        kCheck(kList_Add(headerLines, &line, kNULL)); 

        kCheck(kHttpServerRequest_ReadLine(request, line)); 
    }
    while ((kString_Length(line) != 0) && (kList_Count(headerLines) < kHTTP_SERVER_REQUEST_HEADER_CAPACITY)); 

    if (kString_Length(line) == 0)
    {
        kCheck(kList_Remove(headerLines, kList_Last(headerLines))); 
        kCheck(kObject_Destroy(line)); 
        return kOK; 
    }
    else
    {
        return kERROR_INCOMPLETE; 
    }
}

kFx(kStatus) kHttpServerRequest_CoalesceHeaderLines(kHttpServerRequest request, kList headerLines)
{
    kListItem first = kList_First(headerLines); 
    kListItem second = kIsNull(first) ? kNULL : kList_Next(headerLines, first); 

    while (!kIsNull(second))
    {
        kString firstStr = kList_As_(headerLines, first, kString); 
        kString secondStr = kList_As_(headerLines, second, kString); 
        kChar* secondChars = kString_Chars(secondStr); 
       
        if ((kString_Length(secondStr) > 0) && isspace(secondChars[0]))
        {
            kCheck(kString_Add(firstStr, secondChars)); 
            
            kCheck(kList_Remove(headerLines, second)); 
            kCheck(kObject_Destroy(secondStr)); 
        }
        else
        {
            first = second; 
        }

        second = kList_Next(headerLines, first); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_ParseRequestLine(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 
    kListItem lineItem = kList_First(obj->headerLines); 
    kString line = kIsNull(lineItem) ? kNULL : kList_As_(obj->headerLines, lineItem, kString); 
    kArrayList tokens = kNULL; 
    const kChar* versionNumStr = kNULL; 
    k32u versionMajor, versionMinor; 

    kCheckErr(!kIsNull(line), FORMAT); 

    kTry
    {
        kTest(kList_Remove(obj->headerLines, lineItem)); 

        kTest(kString_Split(line, " ", &tokens, kObject_Alloc_(request))); 
        kTestErr(kArrayList_Count(tokens) == 3, FORMAT); 

        obj->method = kArrayList_As_(tokens, 0, kString); 
        obj->uri = kArrayList_As_(tokens, 1, kString); 
        obj->versionStr = kArrayList_As_(tokens, 2, kString); 

        kArrayList_Clear_(tokens);           
        
        kTest(kString_Construct(&obj->uriPath, kNULL, kObject_Alloc_(request))); 
        kTest(kHttpServerRequest_ParseUriPath(request, obj->uri, obj->uriPath));       

        versionNumStr = kStrFindFirst(kString_Chars(obj->versionStr), "/"); 

        kTestErr(!kIsNull(versionNumStr), FORMAT); 

        if (sscanf(++versionNumStr, "%u.%u", &versionMajor, &versionMinor) != 2)          
        {
            kThrow(kERROR_FORMAT); 
        }

        obj->version = kVersion_Create(versionMajor, versionMinor, 0, 0); 
    }
    kFinally
    {
        kObject_Destroy(line); 
        kObject_Dispose(tokens); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_ParseUriPath(kHttpServerRequest request, kString uri, kString uriPath)
{
    const kChar* separator; 

    if (!kIsNull(separator = kStrFindFirst(kString_Chars(uri), "://")))
    {
        if (!kIsNull(separator = kStrFindFirst(separator + 3, "/")))
        {
            //uri was in absolute form
            kCheck(kString_Set(uriPath, separator)); 
        }
        else
        {
            //uri was root in absolute form, but missing root slash
            kCheck(kString_Set(uriPath, "/")); 
        }
    }
    else
    {
        //uri was already in path-form
        kCheck(kString_Assign(uriPath, uri)); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_ParseHeaders(kHttpServerRequest request, kList headerLines, kMap headerMap)
{
    kListItem listIt = kList_First(headerLines);

    while (!kIsNull(listIt))
    {
        kString str = kList_As_(headerLines, listIt, kString); 
       
        kCheck(kHttpServerRequest_ParseHeader(request, str, headerMap)); 

        listIt = kList_Next(headerLines, listIt); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_ParseHeader(kHttpServerRequest request, kString str, kMap headerMap)
{
    kChar* delim = (kChar*) kStrFindFirst(kString_Chars(str), ":"); 
    kMapItem mapItem = kNULL; 
    kString name = kNULL; 
    kString value = kNULL; 

    kCheckErr(!kIsNull(delim), FORMAT); 
    delim[0] = 0; 

    kTry
    {
        kTest(kString_Construct(&name, kString_Chars_(str), kObject_Alloc_(request))); 
        kTest(kString_Trim(name)); 
        kTest(kHttpServerChannel_NormalizeHeaderCaps(kString_Chars(name))); 

        kTest(kString_Construct(&value, ++delim, kObject_Alloc_(request))); 
        kTest(kString_Trim(value)); 

        if (kSuccess(kMap_FindItem(headerMap, &name, &mapItem)))
        {
            kString existing = kMap_ValueAs_(headerMap, mapItem, kString); 

            kTest(kString_Addf(existing, ",%s", kString_Chars(value))); 
        }
        else
        {
            kTest(kMap_Add(headerMap, &name, &value)); 
            name = value = kNULL; 
        }
    }
    kFinally
    {
        kObject_Destroy(name); 
        kObject_Destroy(value); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_ParseKnownHeaders(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 
    const kChar* str = kNULL; 

    if (!kIsNull(str = kHttpServerRequest_FindHeaderValue(request, "Content-Length")))
    {
        sscanf(str, "%lld", &obj->contentLength); 
    }

    if (!kIsNull(str = kHttpServerRequest_FindHeaderValue(request, "Transfer-Encoding")))
    {
        obj->isChunkCoded = kTRUE; 
    }

    if (!kIsNull(str = kHttpServerRequest_FindHeaderValue(request, "Expect")))
    {
        kCheck(kString_Set(obj->tempStr, str)); 
        kCheck(kStrToLower(kString_Chars(obj->tempStr))); 

        obj->expectContinue = !kIsNull(kStrFindFirst(kString_Chars(obj->tempStr), "100-continue")); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_ReadLine(kHttpServerRequest request, kString line)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 
    const kSize EOL_LENGTH = 2; 
    kSize length = 0; 
    kChar* buffer = 0; 
    kBool isComplete = kFALSE; 

    kCheck(kString_Clear(line)); 
    kCheck(kString_Reserve(line, 64)); 
    buffer = kString_Chars(line); 

    do
    {
        if (length == kString_Capacity_(line))
        {
            kCheck(kString_Reserve(line, 2*length)); 
            buffer = kString_Chars_(line); 
        }

        kCheck(kStream_Read1_(obj->client, &buffer[length])); 
        length++; 

        isComplete = (length >= EOL_LENGTH) && (buffer[length-EOL_LENGTH] == '\r') && (buffer[length-EOL_LENGTH+1] == '\n'); 
    }
    while (!isComplete && (length < kHTTP_SERVER_REQUEST_LINE_CAPACITY)); 
    
    kCheckErr(isComplete, INCOMPLETE); 
    
    kCheck(kString_SetLength(line, length - EOL_LENGTH));
   
    return kOK; 
}
    
kFx(const kChar*) kHttpServerRequest_Method(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return kString_Chars(obj->method); 
}

kFx(const kChar*) kHttpServerRequest_Uri(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return kString_Chars(obj->uri); 
}

kFx(const kChar*) kHttpServerRequest_UriPath(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return kString_Chars(obj->uriPath); 
}

kFx(kVersion) kHttpServerRequest_Version(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return obj->version; 
}

kFx(kSize) kHttpServerRequest_HeaderCount(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return kMap_Count(obj->headers); 
}

kFx(kPointer) kHttpServerRequest_FirstHeader(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return kMap_First(obj->headers); 
}

kFx(kPointer) kHttpServerRequest_NextHeader(kHttpServerRequest request, kPointer header)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return kMap_Next(obj->headers, header); 
}

kFx(const kChar*) kHttpServerRequest_HeaderName(kHttpServerRequest request, kPointer header)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 
    kString name = kMap_KeyAs_(obj->headers, header, kString); 

    return kString_Chars(name); 
}

kFx(const kChar*) kHttpServerRequest_HeaderValue(kHttpServerRequest request, kPointer header)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 
    kString value = kMap_ValueAs_(obj->headers, header, kString); 

    return kString_Chars(value); 
}

kFx(const kChar*) kHttpServerRequest_FindHeaderValue(kHttpServerRequest request, const kChar* name)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 
    kString valueStr = kNULL; 

    if (kSuccess(kString_Set(obj->findName, name)))
    {
        kHttpServerChannel_NormalizeHeaderCaps(kString_Chars(obj->findName)); 

        if (kSuccess(kMap_Find(obj->headers, &obj->findName, &valueStr)))
        {
            return kString_Chars(valueStr); 
        }
    }

    return kNULL; 
}

kFx(kBool) kHttpServerRequest_HasBody(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return (obj->contentLength >= 0) || obj->isChunkCoded; 
}

kFx(k64s) kHttpServerRequest_ContentLength(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return obj->contentLength; 
}

kFx(kBool) kHttpServerRequest_IsChunkCoded(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return obj->isChunkCoded; 
}

kFx(kStatus) kHttpServerRequest_ExpectsContinue(kHttpServerRequest request)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    return obj->expectContinue; 
}

kFx(kStatus) kHttpServerRequest_BeginRead(kHttpServerRequest request, k64u* length, kStream* stream)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    *length = 0; 
    *stream = obj->client; 

    kCheckState(kHttpServerRequest_HasBody(request)); 
    
    if (!obj->contentComplete)
    {
        if (!obj->isChunkCoded)
        {
            *length = (k64u)obj->contentLength; 
            obj->contentComplete = kTRUE; 
        }
        else
        {
            if (obj->chunkIndex != 0)
            {
                //read terminating CRLF from previous chunk
                kCheck(kHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            }

            //read chunk descriptor; ignore extensions
            kCheck(kHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            kCheckErr(sscanf(kString_Chars(obj->chunkLine), "%llX", length) == 1, FORMAT); 

            if (*length == 0)
            {
                obj->contentComplete = kTRUE; 
                
                //read optional trailing headers
                kCheck(kHttpServerRequest_ReadHeaderLines(request, obj->headerLines)); 
                kCheck(kHttpServerRequest_CoalesceHeaderLines(request, obj->headerLines)); 
                kCheck(kHttpServerRequest_ParseHeaders(request, obj->headerLines, obj->headers)); 
            }

            obj->chunkIndex++; 
        }
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_BeginReadChunk(kHttpServerRequest request, k64u* length, kStream* stream)
{
    kHttpServerRequestClass* obj = kHttpServerRequest_Cast_(request); 

    *length = 0; 
    *stream = obj->client; 

    kCheckState(kHttpServerRequest_HasBody(request)); 
    
    if (!obj->contentComplete)
    {
        if (!obj->isChunkCoded)
        {
            *length = (k64u)obj->contentLength; 
            obj->contentComplete = kTRUE; 
        }
        else
        {
            if (obj->chunkIndex != 0)
            {
                //read terminating CRLF from previous chunk
                kCheck(kHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            }

            //read chunk descriptor; ignore extensions
            kCheck(kHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            kCheckErr(sscanf(kString_Chars(obj->chunkLine), "%llX", length) == 1, FORMAT); 

            if (*length == 0)
            {
                obj->contentComplete = kTRUE; 

                //read terminating CRLF from final chunk
                kCheck(kHttpServerRequest_ReadLine(request, obj->chunkLine)); 
                
                //read optional trailing headers
                kCheck(kHttpServerRequest_ReadHeaderLines(request, obj->headerLines)); 
                kCheck(kHttpServerRequest_CoalesceHeaderLines(request, obj->headerLines)); 
                kCheck(kHttpServerRequest_ParseHeaders(request, obj->headerLines, obj->headers)); 
            }

            obj->chunkIndex++; 
        }
    }

    return kOK; 
}

