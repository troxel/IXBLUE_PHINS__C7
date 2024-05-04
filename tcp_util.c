/*--------------------------------------------------

Utility Functions to facilitate tcp communications.

Version History
----------------
Orignal: SOT Jan 2024  
---------------------------------------------------*/

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "tcp_util.h"

// ------------------------
int open_tcp_sock(int port) {
  int sockfd;
  struct sockaddr_in server_addr;

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) 
      return -1;

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  //server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
  server_addr.sin_port = htons(port);

  if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    perror("ERROR on binding\n");
    return -1;
  }

  return sockfd;
}

// ------------------------
ssize_t read_tcp_socket(int sockfd, void *buffer, size_t length) {
  struct sockaddr src_addr;
  socklen_t src_addr_len = sizeof(struct sockaddr);
 
  return recvfrom(sockfd, buffer, length, 0, &src_addr, &src_addr_len);
}

// ------------------------
ssize_t write_tcp_socket(int sockfd, const void *buffer, size_t length, const struct sockaddr *dest_addr, socklen_t addrlen) {
  return sendto(sockfd, buffer, length, 0, dest_addr, addrlen);
}

// Function to close a tcp socket
int close_tcp_socket(int sockfd) {
  return close(sockfd);
}



