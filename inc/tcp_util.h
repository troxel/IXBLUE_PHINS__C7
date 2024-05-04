/*--------------------------------------------------

Utility Function include to facilitate UDP communications.

Version History
----------------
Orignal: Troxel Jan 2024  
---------------------------------------------------*/

#ifndef udp_util_h
#define udp_util_h

int open_udp_sock(int port);

ssize_t read_udp_socket(int sockfd, void *buffer, size_t length);

ssize_t write_udp_socket(int sockfd, const void *buffer, size_t length, const struct sockaddr *dest_addr, socklen_t addrlen);

int close_udp_socket(int sockfd);


#endif
