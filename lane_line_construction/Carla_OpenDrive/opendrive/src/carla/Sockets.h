// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#if _WIN32
#include <Ws2tcpip.h>
#include <winsock2.h> ///< socket
#else
#include <arpa/inet.h>  ///< getsockname
#include <netinet/in.h> ///< sockaddr_in
#include <sys/socket.h> ///< socket
#include <unistd.h>     ///< close
#endif

#define SOCK_INVALID_INDEX -1