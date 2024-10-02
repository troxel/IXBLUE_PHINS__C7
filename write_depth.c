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
#include "tcp_util.h"
#include "phins_util.h"
#include "parameters.h"

#define PHINS_DEPTH_PORT 8122

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stbn_util.h>

// Define the header size
#define HEADER_SIZE 25

// Per Manual: All 16 and 32 bits integers are represented in Big Endian convention (MSB sent first).

int sockfd;

void byte_array_to_hex_string(const unsigned char *bytes, int len);
uint32_t get_utc_time_100us();
uint32_t milliseconds_since_midnight_gmt();

uint8_t Debug = 1; 

// Function to prepare the header
void prepare_header(uint8_t *header, uint16_t payload_size) {
    // Header 1: 'I'
    header[0] = 'I';

    // Header 2: 'X'
    header[1] = 'X';

    // Protocol Version: 0x03
    header[2] = 0x03;

    // Navigation Data Bit Mask: 0x00000000
    header[3] = 0x00;
    header[4] = 0x00;
    header[5] = 0x00;
    header[6] = 0x00;

    // Extended Navigation Data Bit Mask: 0x00000000
    header[7] = 0x00;
    header[8] = 0x00;
    header[9] = 0x00;
    header[10] = 0x00;

    // External Data Bit Mask: 0x00 02 00 00 (9th bit zero based)
    header[11] = 0x00;
    header[12] = 0x00;
    header[13] = 0x02;
    header[14] = 0x00;

    // Total Telegram Size: 25 + payload size 
    // For Depth block 25 + 12 + 4 = 41 or 0x29
    uint16_t total_size = HEADER_SIZE + payload_size + 4; // 4 bytes for checksum
    header[15] = (total_size >> 8) & 0xFF;  // High byte
    header[16] = total_size & 0xFF;         // Low byte

    // External Sensors Timestamp Reference: 0x00 (UTC time)
    header[17] = 0x00;

    // RFU (Reserved for Future Use): Set to 0
    memset(&header[18], 0x00, 7);
}

// Function to prepare the payload (Depth data block (bit n°9))
void prepare_payload(uint8_t *payload, float depth, float depth_stddev) {

    // Depth Data Block (Bit 9)
   
    //int32_t utc_ts = -1;
    int32_t utc_ts = 0x7FC00000;  // Uses time of reception for timing. 
    //int32_t utc_ts = 0;
    //int32_t utc_ts = milliseconds_since_midnight_gmt();  // Rqrs NTP

    int32_t utc_ts_be = htonl(utc_ts);

    // First element is 4 bytes of utc_time in big endian order
    memcpy(payload,&utc_ts_be,sizeof utc_ts_be);

    // -------------------------------
    // From Dan at Exail
    // 49 58 03 00 00 00 00 00 00 00 00 00 00 02 00 00 29 00 00 00 00 00 00 00 00 7F C0 00 00 3F 80 00 00 3D CC CC CD 00 00 05 6F   - dan

    depth = l2b_f((uint8_t *) &depth);

    // Depth (Float, 4 bytes swapped)
    memcpy(&payload[4],&depth,sizeof depth);

    // Depth Standard Deviation (Float, 4 bytes swapped)
    depth_stddev = l2b_f((uint8_t *) &depth_stddev);

    memcpy(&payload[8],&depth_stddev,sizeof depth_stddev);
}

// ------------------------------------
// main entry
// ------------------------------------
int main() {
    uint8_t header[HEADER_SIZE];
    uint16_t payload_size = 12;  // Depth data block size (12 bytes)
    uint8_t payload[payload_size];
    uint32_t chksum;

    size_t n_elements = (HEADER_SIZE + payload_size + sizeof chksum);
    uint8_t datagram[n_elements];

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
    dest_addr.sin_port        = htons(PHINS_DEPTH_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(PHINS_IP);

    // Prepare the header
    prepare_header(header, payload_size);

    while (1) {

        memcpy(datagram, header, HEADER_SIZE);

        // Prepare the payload (depth data)
        float depth = 400.432;
        float depth_stddev = 1;

        prepare_payload(payload, depth, depth_stddev);

        memcpy(datagram + HEADER_SIZE, payload, payload_size);

        chksum = checksum(datagram, n_elements);

        uint32_t chksum_be = htonl(chksum);  // Convert to big-endian

        // Append the checksum to the end of the datagram
        memcpy(datagram + HEADER_SIZE + payload_size, &chksum_be, sizeof chksum_be);

        ssize_t rtn = sendto(sockfd, datagram, sizeof datagram, 0, (struct sockaddr *) &dest_addr, dest_addr_len);
        if ( rtn < 0 ) {
            perror("Send Error ");
        }
        else {
            printf("sent datagram %lu bytes\n",rtn);
            if ( Debug ) { byte_array_to_hex_string(datagram,sizeof datagram); }
        }
        
        sleep(1);
    }

    return 0;
}

// -------------------------------------------------
// Debug display function layout out the byte string
// -------------------------------------------------
void byte_array_to_hex_string(const unsigned char *bytes, int len) {
    int i;
    char hex_string[1024];
    char separator = ' '; 

    for (i = 0; i < len; i++) {
        sprintf(hex_string + i * 3, "%02X%c", bytes[i], (i < len - 1) ? separator : '\0');
    }
    printf("%s\n",hex_string);

    printf("hdr1\t\t\t%02X %c%c\n",bytes[0],hex_string[0],hex_string[1]);
    printf("hdr2\t\t\t%02X\n",bytes[1]);
    printf("vers\t\t\t%02X\n",bytes[2]);
    printf("NavBlk\t\t\t%02X %02X %02X %02X\n",bytes[3],bytes[4],bytes[5],bytes[6]);
    printf("ExtBlk\t\t\t%02X %02X %02X %02X\n",bytes[7],bytes[8],bytes[9],bytes[10]);
    printf("ExnBlk\t\t\t%02X %02X %02X %02X\n",bytes[11],bytes[12],bytes[13],bytes[14]);
    printf("Size\t\t\t%02X %02X = %u\n",bytes[15],bytes[16],(uint16_t)bytes[16]);
    printf("TS Ref\t\t\t%02X\n",bytes[17]);
    printf("RFU\t\t\t%02X %02X %02X %02X %02X %02X %02X \n",bytes[18],bytes[19],bytes[20],bytes[21],bytes[22],bytes[23],bytes[24]);

    printf("Vldyt\t\t\t%02X %02X %02X %02X\n",bytes[25],bytes[26],bytes[27],bytes[28]);

    printf("Dpth\t\t\t%02X %02X %02X %02X\n",bytes[29],bytes[30],bytes[31],bytes[32]);
    printf("Dpthstd\t\t\t%02X %02X %02X %02X\n",bytes[33],bytes[34],bytes[35],bytes[36]);

    printf("chksum\t\t\t%02X %02X %02X %02X\n",bytes[37],bytes[38],bytes[39],bytes[40]);
}

//----------------------------------------------------
// Require PHINS in NTP mode
uint32_t milliseconds_since_midnight_gmt() {
    // Get the current time in GMT
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    // Get the current time in UTC
    struct tm gmt_time;
    gmtime_r(&ts.tv_sec, &gmt_time);

    // Calculate the time since midnight
    uint64_t milliseconds = gmt_time.tm_hour * 36000000 +  // Hours to milliseconds
                            gmt_time.tm_min * 600000 +     // Minutes to milliseconds
                            gmt_time.tm_sec * 10000 +      // Seconds to milliseconds
                            ts.tv_nsec / 10000000;         // Nanoseconds to milliseconds

    return (uint32_t)milliseconds;
}

























// // STDBIN v3 structs input (page 154)
// struct stbn_input_hdr_t {
//     uint8_t h1;
//     uint8_t h2;
//     uint8_t proto;
//     uint32_t navg_mask;  // 0x0 for input
//     uint32_t extd_mask;  // 0x0 for input
//     uint32_t extn_mask;
//     uint16_t stbn_size;
//     uint8_t  ts_extn;    // 0x0 for UTC
//     uint8_t rfu[7];      // 0x0 RFU reserved-for-future use?
// };

// // Page 167 
// struct stbn_input_depth_t {
//     uint32_t ts_validity;
//     float depth;
//     float depth_stdev;
// };

// uint32_t extn_mask = 0x100;

// /* -------------------------------------------*/
// // Attempt to send depth data directly to phins
// // 
// /* -------------------------------------------*/
// int main() {

//     int sockfd;

//     size_t datagram_size = sizeof(struct stbn_input_hdr_t) + sizeof(struct stbn_input_depth_t) + sizeof(uint32_t);

//     struct stbn_input_hdr_t stbn_input_hdr;

//     memset(&stbn_input_hdr,0,sizeof(struct stbn_input_hdr_t));

//     stbn_input_hdr.h1 = 'I';
//     stbn_input_hdr.h2 = 'X';
//     stbn_input_hdr.proto = 0x03;
//     stbn_input_hdr.navg_mask = 0;
//     stbn_input_hdr.extd_mask = 0;
//     stbn_input_hdr.extn_mask = l2b_32((uint8_t *)&extn_mask); // bit 9 depth data
//     //stbn_input_hdr.extn_mask = 0x100; // bit 9 depth data
//     stbn_input_hdr.stbn_size = l2b_32((uint8_t *)&datagram_size);
//     //stbn_input_hdr.stbn_size = datagram_size;
//     stbn_input_hdr.ts_extn   = 0x0;

//     struct stbn_input_depth_t stbn_input_depth;

//     // Create socket
//     sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sockfd < 0) {
//         perror("socket()");
//         exit(1);
//     }

//     struct sockaddr_in dest_addr;
//     socklen_t dest_addr_len = sizeof(struct sockaddr_in);
//     memset(&dest_addr, 0, sizeof(dest_addr));
  
//     dest_addr.sin_family      = AF_INET;
//     dest_addr.sin_port        = htons(PHINS_DEPTH_PORT);
//     dest_addr.sin_addr.s_addr = inet_addr(PHINS_IP);
    
//     uint8_t* datagram = (uint8_t*)malloc(datagram_size);
//     if (!datagram) {
//         perror("Failed to allocate memory for datagram");
//         return EXIT_FAILURE;
//     }

//     memset(datagram, 0, datagram_size);

//     // Copy the header into the datagram (doesn't change)
//     memcpy(datagram, &stbn_input_hdr, sizeof(stbn_input_hdr));

//     uint32_t cnt = 0;

//     float depth = 100.5;

//     while (cnt++ < 150) {

//         float rand_depth = (double)rand() / (double)RAND_MAX;

//         stbn_input_depth.depth = depth + rand_depth; 
//         stbn_input_depth.depth_stdev = 0.5; 
        
//         memcpy(datagram + sizeof(stbn_input_hdr), &stbn_input_depth, sizeof(stbn_input_depth));

//         uint32_t chksum = checksum(datagram, sizeof(stbn_input_hdr) + sizeof(stbn_input_depth));

// printf(">> depth %0.3f %X\n",stbn_input_depth.depth,chksum);

//         // Append the checksum to the end of the datagram
//         memcpy(datagram + sizeof(stbn_input_hdr) + sizeof(stbn_input_depth), &chksum, sizeof(chksum));

//         ssize_t rtn = sendto(sockfd, datagram, datagram_size , 0, (struct sockaddr *) &dest_addr, dest_addr_len);
//         if ( rtn < 0 ) {
//             perror("Send Error ");
//         }
//         else {
//             printf("sent datagram\n");
//         }

//         sleep(3);
//     }

//     close(sockfd);
//     return 0;
// }