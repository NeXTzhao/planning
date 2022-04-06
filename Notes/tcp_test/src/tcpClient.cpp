// /***
//  * @Autor: your name
//  * @Data: Do not edit
//  * @LastEditTime: 2022-03-26 11:58:02
//  * @LastEditors: your name
//  * @Brief:
//  * @FilePath: /c++_workspace/tcp_test/src/client.cpp
//  * @Copyright:
//  */
// #include <client.hpp>

// namespace TCP {
// Client::Client() {
//   sock = -1;
//   port = 0;
//   address = " ";
// }

// bool Client::conn(string address, int port) {
//   //如果是-1 则创建不成功
//   if (-1 == sock) {
//     //重新创建
//     sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
//     if (-1 == sock) {
//       std::cout << "could not create socket" << '\n';
//     }

//     std::cout << "socket creat" << '\n';
//   } else {
//   }

//   if (-1 == inet_addr(address.c_str())) {
//     struct hostent *he;
//     struct in_addr **addr_list;

//     if (he = gethostbyname(address.c_str()) == nullptr) {
//       herror("gethostbyname");
//       std::cout << "Failed to resolve hostname\n";

//       return false;
//     }
//     addr_list = (struct in_addr **)he->h_addr_list;

//     for (int i = 0; addr_list[i] != nullptr; ++i)
//     {
//       server.sin_addr = *addr_list[i];
//       break;
//     }

//   }
// }

// }  // namespace TCP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#define BUFFSIZE 2048
#define SERVER_IP \
  "192.168.19.12"  // 指定服务端的IP，记得修改为你的服务端所在的ip
#define SERVER_PORT 16555  // 指定服务端的port

int main() {
  struct sockaddr_in servaddr;
  char buff[BUFFSIZE];
  int sockfd;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == sockfd) {
    printf("Create socket error(%d): %s\n", errno, strerror(errno));
    return -1;
  }
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  inet_pton(AF_INET, SERVER_IP, &servaddr.sin_addr);
  servaddr.sin_port = htons(SERVER_PORT);
  if (-1 == connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) {
    printf("Connect error(%d): %s\n", errno, strerror(errno));
    return -1;
  }
  printf("Please input: ");
  scanf("%s", buff);
  send(sockfd, buff, strlen(buff), 0);
  bzero(buff, sizeof(buff));
  recv(sockfd, buff, BUFFSIZE - 1, 0);
  printf("Recv: %s\n", buff);
  close(sockfd);
  return 0;
}