//============================================================================
// Process  : Acquires Lat/Long/Depth from the formnet and sends a GGA string
//               to the PHINS.  
// Author   : Steven Troxel 
// Record of Changes: 
//
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>

#include <time.h>
#include <unistd.h>
#include <math.h>

// Shared mem stuff
#include <sys/mman.h>
#include <pthread.h>

// Serial stuff
#include <fcntl.h>
#include <termios.h>

// Semaphor
#include <semaphore.h>
#include <sys/stat.h>

// project
#include "udp_util.h"
#include "phins_util.h"
#include "parameters.h"

#define SHM_FILE_ATACS "atacs_data"

double haversine(double lat1, double lon1, double lat2, double lon2);

uint8_t Verbose = true;

// ---------------------------------------------------------
// Shared memory structures
// ---------------------------------------------------------
typedef struct {
    uint32_t gps_cnt;
    uint32_t atacs_cnt;
    uint32_t skip_atacs;
    uint32_t quality;
    uint32_t gps_time;
    uint32_t delay;
    pthread_mutex_t mutex;
} ATACS_FRAME_t;

/* -------------------------------------------*/
// MAIN Entry Point
/* -------------------------------------------*/
int main() {

    // -----------------------------
    // Create Socket to PHINS
    // -----------------------------
    int sock2phins_fd;
    uint8_t buffer[BUFFER_SIZE];

    memset(buffer, 0, BUFFER_SIZE);

    sock2phins_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock2phins_fd < 0) {
       perror("Phins socket");
       exit(1);
    }

    struct sockaddr_in dest_addr;
    socklen_t dest_addr_len = sizeof(struct sockaddr_in);
    memset(&dest_addr, 0, sizeof(dest_addr));
  
    // Network properties defined in parameters.h
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(PHINS_GPS_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(PHINS_IP);

    struct gps_data_t gps_data;

    char nmea_str[256];
    memset(nmea_str, 0, 256);

    uint32_t cnt = 0;

    // -------------------------------- 
    // The heart of this process 
    // -------------------------------- 
    while (1) {

        // Replace with call to global reflective memory
        gps_data.latitude = 48.070420640765484;
        gps_data.longitude = -116.48900605297399;

        // Time stamp in milliseconds from epoch
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        gps_data.time_epoch = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

        // Absurd check of incoming
        if ( (gps_data.latitude > 49) || (gps_data.latitude < 47) ) {
            fprintf(stderr, "Latitude error %.7f\n", gps_data.latitude);
            continue;
        }

        if ( (gps_data.longitude < -117 ) || (gps_data.longitude > -116) ) {
            fprintf(stderr, "Longitude error %.7f\n", gps_data.longitude);
            continue;
        }

        // GPS quality decoder
        // 0 Data invalid N/A
        // 1 10m    Natural
        // 2 3m     Differential
        // 3 10m    Military
        // 4 0.1m   RTK
        // 5 0.3m  Float RTK
        gps_data.quality = 1; 

        gps_data.altitude = -121.92; // 400 ft
        
        buildGGA(&gps_data, nmea_str);

        if ( (cnt++ % 10) == 0 ) {
           printf("%s",nmea_str);
        }
 
        ssize_t rtn = sendto(sock2phins_fd,  (const char *)nmea_str, strlen(nmea_str) , 0, (struct sockaddr *) &dest_addr, dest_addr_len);
        if ( rtn < 0 ) {
            perror("Send Error ");
        }

        sleep(1);

    }

    close(sock2phins_fd);
    return 0;
}

