/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-03-27 14:13:40
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /c++_workspace/tcp_test/src/udpClientServer.cpp
 * @Copyright:
 */
// UDP Client & Server -- classes to ease handling sockets
// Copyright (C) 2013  Made to Order Software Corp.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "udpClientServer.hpp"

#include <string.h>
#include <unistd.h>

namespace udp_client_server {

// ========================= CLIENT =========================

/** \brief Initialize a UDP client object.
 *
 * This function initializes the UDP client object using the address and the
 * port as specified.
 *
 * The port is expected to be a host side port number (i.e. 59200).
 *
 * The \p addr parameter is a textual address. It may be an IPv4 or IPv6
 * address and it can represent a host name or an address defined with
 * just numbers. If the address cannot be resolved then an error occurs
 * and constructor throws.
 *
 * \note
 * The socket is open in this process. If you fork() or exec() then the
 * socket will be closed by the operating system.
 *
 * \warning
 * We only make use of the first address found by getaddrinfo(). All
 * the other addresses are ignored.
 *
 * \exception udp_client_server_runtime_error
 * The server could not be initialized properly. Either the address cannot be
 * resolved, the port is incompatible or not available, or the socket could
 * not be created.
 *
 * \param[in] addr  The address to convert to a numeric IP.
 * \param[in] port  The port number.
 */
udp_client::udp_client(const std::string &addr, int port)
    : f_port(port), f_addr(addr) {
  char decimal_port[16];
  snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
  decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &f_addrinfo));
  if (r != 0 || f_addrinfo == NULL) {
    throw udp_client_server_runtime_error(
        ("invalid address or port: \"" + addr + ":" + decimal_port + "\"")
            .c_str());
  }
  f_socket =
      socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  if (f_socket == -1) {
    freeaddrinfo(f_addrinfo);
    throw udp_client_server_runtime_error(
        ("could not create socket for: \"" + addr + ":" + decimal_port + "\"")
            .c_str());
  }
}

/** \brief Clean up the UDP client object.
 *
 * This function frees the address information structure and close the socket
 * before returning.
 */
udp_client::~udp_client() {
  freeaddrinfo(f_addrinfo);
  close(f_socket);
}

/** \brief Retrieve a copy of the socket identifier.
 *
 * This function return the socket identifier as returned by the socket()
 * function. This can be used to change some flags.
 *
 * \return The socket used by this UDP client.
 */
int udp_client::get_socket() const { return f_socket; }

/** \brief Retrieve the port used by this UDP client.
 *
 * This function returns the port used by this UDP client. The port is
 * defined as an integer, host side.
 *
 * \return The port as expected in a host integer.
 */
int udp_client::get_port() const { return f_port; }

/** \brief Retrieve a copy of the address.
 *
 * This function returns a copy of the address as it was specified in the
 * constructor. This does not return a canonalized version of the address.
 *
 * The address cannot be modified. If you need to send data on a different
 * address, create a new UDP client.
 *
 * \return A string with a copy of the constructor input address.
 */
std::string udp_client::get_addr() const { return f_addr; }

/** \brief Send a message through this UDP client.
 *
 * This function sends \p msg through the UDP client socket. The function
 * cannot be used to change the destination as it was defined when creating
 * the udp_client object.
 *
 * The size must be small enough for the message to fit. In most cases we
 * use these in Snap! to send very small signals (i.e. 4 bytes commands.)
 * Any data we would want to share remains in the Cassandra database so
 * that way we can avoid losing it because of a UDP message.
 *
 * \param[in] msg  The message to send.
 * \param[in] size  The number of bytes representing this message.
 *
 * \return -1 if an error occurs, otherwise the number of bytes sent. errno
 * is set accordingly on error.
 */
int udp_client::send(const char *msg, size_t size) {
  return sendto(f_socket, msg, size, 0, f_addrinfo->ai_addr,
                f_addrinfo->ai_addrlen);
}

// ========================= SEVER =========================

/** \brief Initialize a UDP server object.
 *
 * This function initializes a UDP server object making it ready to
 * receive messages.
 *
 * The server address and port are specified in the constructor so
 * if you need to receive messages from several different addresses
 * and/or port, you'll have to create a server for each.
 *
 * The address is a string and it can represent an IPv4 or IPv6
 * address.
 *
 * Note that this function calls connect() to connect the socket
 * to the specified address. To accept data on different UDP addresses
 * and ports, multiple UDP servers must be created.
 *
 * \note
 * The socket is open in this process. If you fork() or exec() then the
 * socket will be closed by the operating system.
 *
 * \warning
 * We only make use of the first address found by getaddrinfo(). All
 * the other addresses are ignored.
 *
 * \exception udp_client_server_runtime_error
 * The udp_client_server_runtime_error exception is raised when the address
 * and port combinaison cannot be resolved or if the socket cannot be
 * opened.
 *
 * \param[in] addr  The address we receive on.
 * \param[in] port  The port we receive from.
 */
udp_server::udp_server(const std::string &addr, int port)
    : f_port(port), f_addr(addr) {
  char decimal_port[16];
  snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
  decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &f_addrinfo));
  if (r != 0 || f_addrinfo == NULL) {
    throw udp_client_server_runtime_error(
        ("invalid address or port for UDP socket: \"" + addr + ":" +
         decimal_port + "\"")
            .c_str());
  }
  f_socket =
      socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  if (f_socket == -1) {
    freeaddrinfo(f_addrinfo);
    throw udp_client_server_runtime_error(
        ("could not create UDP socket for: \"" + addr + ":" + decimal_port +
         "\"")
            .c_str());
  }
  r = bind(f_socket, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);
  if (r != 0) {
    freeaddrinfo(f_addrinfo);
    close(f_socket);
    throw udp_client_server_runtime_error(
        ("could not bind UDP socket with: \"" + addr + ":" + decimal_port +
         "\"")
            .c_str());
  }
}

/** \brief Clean up the UDP server.
 *
 * This function frees the address info structures and close the socket.
 */
udp_server::~udp_server() {
  freeaddrinfo(f_addrinfo);
  close(f_socket);
}

/** \brief The socket used by this UDP server.
 *
 * This function returns the socket identifier. It can be useful if you are
 * doing a select() on many sockets.
 *
 * \return The socket of this UDP server.
 */
int udp_server::get_socket() const { return f_socket; }

/** \brief The port used by this UDP server.
 *
 * This function returns the port attached to the UDP server. It is a copy
 * of the port specified in the constructor.
 *
 * \return The port of the UDP server.
 */
int udp_server::get_port() const { return f_port; }

/** \brief Return the address of this UDP server.
 *
 * This function returns a verbatim copy of the address as passed to the
 * constructor of the UDP server (i.e. it does not return the canonalized
 * version of the address.)
 *
 * \return The address as passed to the constructor.
 */
std::string udp_server::get_addr() const { return f_addr; }

/** \brief Wait on a message.
 *
 * This function waits until a message is received on this UDP server.
 * There are no means to return from this function except by receiving
 * a message. Remember that UDP does not have a connect state so whether
 * another process quits does not change the status of this UDP server
 * and thus it continues to wait forever.
 *
 * Note that you may change the type of socket by making it non-blocking
 * (use the get_socket() to retrieve the socket identifier) in which
 * case this function will not block if no message is available. Instead
 * it returns immediately.
 *
 * \param[in] msg  The buffer where the message is saved.
 * \param[in] max_size  The maximum size the message (i.e. size of the \p msg
 * buffer.)
 *
 * \return The number of bytes read or -1 if an error occurs.
 */
int udp_server::recv(char *msg, size_t max_size) {
  return ::recv(f_socket, msg, max_size, 0);
}

/** \brief Wait for data to come in.
 *
 * This function waits for a given amount of time for data to come in. If
 * no data comes in after max_wait_ms, the function returns with -1 and
 * errno set to EAGAIN.
 *
 * The socket is expected to be a blocking socket (the default,) although
 * it is possible to setup the socket as non-blocking if necessary for
 * some other reason.
 *
 * This function blocks for a maximum amount of time as defined by
 * max_wait_ms. It may return sooner with an error or a message.
 *
 * \param[in] msg  The buffer where the message will be saved.
 * \param[in] max_size  The size of the \p msg buffer in bytes.
 * \param[in] max_wait_ms  The maximum number of milliseconds to wait for a
 * message.
 *
 * \return -1 if an error occurs or the function timed out, the number of bytes
 * received otherwise.
 */
int udp_server::timed_recv(char *msg, size_t max_size, int max_wait_ms) {
  fd_set s;
  FD_ZERO(&s);
  FD_SET(f_socket, &s);
  struct timeval timeout;
  timeout.tv_sec = max_wait_ms / 1000;
  timeout.tv_usec = (max_wait_ms % 1000) * 1000;
  int retval = select(f_socket + 1, &s, &s, &s, &timeout);
  if (retval == -1) {
    // select() set errno accordingly
    return -1;
  }
  if (retval > 0) {
    // our socket has data
    return ::recv(f_socket, msg, max_size, 0);
  }

  // our socket has no data
  errno = EAGAIN;
  return -1;
}

}  // namespace udp_client_server

// vim: ts=4 sw=4 etS