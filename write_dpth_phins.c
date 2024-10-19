#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <time.h>
#include <unistd.h>
#include <byteswap.h>
#include <math.h>
#include <error.h>
#include <errno.h>

#include "udp_util.h"
#include "phins_util.h"
#include "parameters.h"

/* -------------------------------------------*/
int main() {

    int sockfd;

    // A test to see if PHINS is polling for data. 
    // sockfd = open_udp_svr_sock(PHINS_DPTH_PORT);
    // uint8_t dg_buf[256];
    // ssize_t dg_len; 
    // memset(dg_buf, 0, 256);

    // printf("Socket Opened %d\n",sockfd);

    // dg_len = read_udp_socket(sockfd, dg_buf, BUFFER_SIZE);
    // if ( dg_len < 1 ) {
    //         perror("Read Error UDP Socket Phins");
    //         exit(1); 
    // }
    // printf(">%ld %u",dg_len,dg_buf[0]);
    // exit(0);

    // Create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket()");
        exit(1);
    }

    struct sockaddr_in dest_addr;
    socklen_t dest_addr_len = sizeof(struct sockaddr_in);
    memset(&dest_addr, 0, sizeof(dest_addr));
  
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(PHINS_DPTH_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(PHINS_IP);

    char dpth_str[16];
    memset(dpth_str, 0, 16);

    uint32_t cnt = 0;

    float psi2ft= 0.433;
    
    float depth = 400;

    while (cnt < 50) {

        float psi = depth * psi2ft; 

        sprintf(dpth_str,"*0001%.1f\r\n",psi); 
       
        ssize_t rtn = sendto(sockfd,  (const char *)dpth_str, strlen(dpth_str) , 0, (struct sockaddr *) &dest_addr, dest_addr_len);
        if ( rtn < 0 ) {
            perror("Send Error ");
        }
        else {
            printf("sent: %s",dpth_str);
        }

        sleep(1);
    }

    //close(sockfd);
    return 0;
}

