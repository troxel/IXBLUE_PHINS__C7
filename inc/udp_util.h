/*--------------------------------------------------

Utility Function include to facilitate UDP communications.

Version History
----------------
Orignal: Troxel Jan 2024  
---------------------------------------------------*/

#ifndef udp_util_h
#define udp_util_h

int open_udp_svr_sock(int port);

ssize_t read_udp_socket(int sockfd, void *buffer, size_t length);

#endif
