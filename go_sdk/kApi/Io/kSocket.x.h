/** 
 * @file    kSocket.x.h
 *
 * @internal
 * Copyright (C) 2004-2014 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SOCKET_X_H
#define K_API_SOCKET_X_H

kBeginHeader()

kDeclareEnum(k, kSocketType, kValue)
kDeclareBitEnum(k, kSocketEvent, kValue)
kDeclareClass(k, kSocket, kObject)

#define kSOCKET_MAX_WAIT_GROUP              (64)
#define kSOCKET_WAIT_COUNT                  (3)

#if defined (K_PLATFORM)

#if defined (K_WINDOWS)
#   define kSocketHandle                SOCKET
#   define kSocket_Select               select
#   define kSOCKET_INVALID_SOCKET       INVALID_SOCKET
#   define kSOCKET_ERROR                SOCKET_ERROR
#   define kSOCKET_WOULD_BLOCK          WSAEWOULDBLOCK
#   define kSOCKET_SHUTDOWN_BOTH        SD_BOTH       
#   define kSOCKET_IN_PROGRESS          (0)
#   define kSOCKET_TIMEDOUT             WSAETIMEDOUT
#   define kSOCKET_MSGSIZE              WSAEMSGSIZE
#   define kSOCKET_MAX_IO_SIZE          (0x7FFFFFFF)
#   define kSOCKET_MAX_BUFFER           (0xFFFFFFFF)

#elif defined (K_DSP_BIOS)
#   define kSocketHandle                SOCKET
#   define socklen_t                    int
#   define kSocket_Select               fdSelect
#   define kSOCKET_INVALID_SOCKET       INVALID_SOCKET
#   define kSOCKET_ERROR                SOCKET_ERROR
#   define kSOCKET_SHUTDOWN_BOTH        SHUT_RDWR
#   define kSOCKET_WOULD_BLOCK          EWOULDBLOCK
#   define kSOCKET_IN_PROGRESS          EINPROGRESS
#   define kSOCKET_TIMEDOUT             ETIMEDOUT
#   define kSOCKET_MSGSIZE              ENOMEM
#   define kSOCKET_MAX_BUFFER           (0xFFFFFFFF)

#elif defined (K_VX_KERNEL)
#   define kSocketHandle                unsigned int
#   define kSocket_Select               select
#   define kSOCKET_INVALID_SOCKET       (kSocketHandle)(~0)
#   define kSOCKET_ERROR                (ERROR)
#   define kSOCKET_SHUTDOWN_BOTH        SHUT_RDWR
#   define kSOCKET_WOULD_BLOCK          EWOULDBLOCK
#   define kSOCKET_IN_PROGRESS          EINPROGRESS
#   define kSOCKET_TIMEDOUT             EAGAIN
#   define kSOCKET_MSGSIZE              ENOMEM
#   define kSOCKET_MAX_BUFFER           (0xFFFFFFFF)

#elif defined (K_POSIX)
#   define kSocketHandle                unsigned int
#   define kSocket_Select               select
#   define kSOCKET_INVALID_SOCKET       (kSocketHandle)(~0)
#   define kSOCKET_ERROR                (-1)
#   define kSOCKET_SHUTDOWN_BOTH        SHUT_RDWR
#   define kSOCKET_WOULD_BLOCK          EWOULDBLOCK
#   define kSOCKET_IN_PROGRESS          EINPROGRESS
#   define kSOCKET_TIMEDOUT             EAGAIN
#   define kSOCKET_MSGSIZE              ENOMEM
#   if defined (K_QNX)
#       define kSOCKET_MAX_BUFFER       (0x0000FFFF)
#   else 
#       define kSOCKET_MAX_BUFFER       (0xFFFFFFFF)
#   endif

#endif

typedef struct kSocketClass
{
    kObjectClass base;      
    kIpVersion ipVersion;           //Internet Protocol version.
    kSocketType socketType;         //Type of socket (TCP, UDP). 
    kSocketHandle handle;           //OS socket handle. 
    kSocketEvent eventTypes;        //Events to use with socket wait operations.
    kSocketEvent eventStatus;       //Events detected during most recent wait call.
    kStatus status;                 //Current socket error status.
    kBool isBlocking;               //Is the socket in blocking mode?
} kSocketClass;

kFx(kStatus) kSocket_ConstructFromHandle(kSocket* sockt, kIpVersion ipVersion, kSocketType socktType, kSocketHandle handle, kAlloc allocator); 
kFx(kStatus) kSocket_Init(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socktType, kAlloc allocator); 
kFx(kStatus) kSocket_InitFromHandle(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socktType, kSocketHandle handle, kAlloc allocator); 
kFx(kStatus) kSocket_VRelease(kSocket sockt); 

kFx(k32s) kSocket_GetLastError();
kFx(kSSize) kSocket_Recv(kSocket sockt, void* buffer, kSize size);
kFx(kSSize) kSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from);
kFx(kSSize) kSocket_Send(kSocket sockt, const void* buffer, kSize size);
kFx(kSSize) kSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to);
kFx(kStatus) kSocket_ShutdownHandle(kSocketHandle socktHandle);
kFx(kStatus) kSocket_CloseHandle(kSocketHandle socktHandle);
kFx(struct timeval*) kSocket_FormatTimeVal(struct timeval* tv, k64u timeout); 

#endif

#define kSocket_Cast_(S)                  (kCastClass_(kSocket, S))
#define kSocket_Handle_(S)                (kCast(kSocketClass*, S)->handle)
#define kSocket_Status_(S)                (kCast(kSocketClass*, S)->status)

/* 
 * FireSync 5 compatibility/transition definitions.  These will eventually be removed. 
 */

#if defined(K_COMPAT_5)

    //simple renaming (handled by porting script)
#   define kTYPE_SOCKET             kTypeOf(kSocket)
#   define kSocket_Destroy          kObject_Destroy

    //more challenging cases (may require manual porting, once K_COMPAT_5 removed)
#   define kSocket_Construct5(S, T)       kSocket_Construct(S, T, kNULL)
#   define kSocket_Accept5(S, C)          kSocket_Accept(S, C, kNULL)
#   define kSocket_Handle(S)              kCast(kSocketClass*, S)->handle

    kFx(kStatus) kSocket_WaitAny5(const kSocket* sockets, k32u count, kBool* status, k64u timeout);
    kFx(kStatus) kSocket_Write5(kSocket sockt, const void* buffer, kSize size);
    kFx(kStatus) kSocket_ClearErrors(kSocket sockt);

#endif

kEndHeader()

#endif
