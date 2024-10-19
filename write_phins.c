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
    uint8_t buffer[BUFFER_SIZE];

    memset(buffer, 0, BUFFER_SIZE);

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
    dest_addr.sin_port        = htons(PHINS_GPS_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(PHINS_IP);

    struct gps_data_t gps_data;
    char nmea_str[256];
    memset(nmea_str, 0, 256);

    uint32_t cnt = 0;

    double lat_in = INIT_LAT;
    double lon_in = INIT_LON;

    // Lat/Lon of select LSV track data. 
    // Other parameters at the location: Spd:11.3 RPY: -1.5, -0.6, 37.1
    // Position 1740 in complete.csv
    lat_in = 48.070420640765484;
    lon_in = -116.48900605297399;

    void cnvt_degree_to_gga(double degree_in, char* gga_latlon);

    srand(time(NULL));

    while (cnt < 500) {

        // Absurd check of incoming
        if ( (lat_in > 49) || (lat_in < 47) ) continue;
        if ( (lon_in < -117 ) || (lon_in > -116) ) continue;

        // gps quality decoder
        // 0 Data invalid N/A
        // 1 10m    Natural
        // 2 3m     Differential
        // 3 10m    Military
        // 4 0.1m   RTK
        // 5 0.3m  Float RTK

        //int rnd = rand();
        //double rand_pos = (double)rnd*pow(2,-51); 
        //printf(">> %.12lf\n",rand_pos);

        gps_data.latitude = lat_in;
        gps_data.longitude = fabs(lon_in);
        gps_data.altitude = -121.92; // 400 ft
        gps_data.quality = 5;
        gps_data.hdop = 1.1;
        gps_data.satellites = 8;
        
        // $GPGGA,001052.00,4758.76758057,N,11633.61604700,E,6,03,38.603,-2.112,M,,M,,*60
        // $GPGGA,005704.01,4802.37924115,N,11632.77614893,E,6,03,85592.812,-0.214,M,,M,,*55
        // $GPGGA,054913,4802.07087454,N,11633.61666800,W,1,08,,15.00,M,,M,,*6A

        buildGGA(&gps_data, nmea_str);

        printf("%u %s",cnt, nmea_str);
        fflush(stdout);
        cnt++;

        ssize_t rtn = sendto(sockfd,  (const char *)nmea_str, strlen(nmea_str) , 0, (struct sockaddr *) &dest_addr, dest_addr_len);
        if ( rtn < 0 ) {
            perror("Send Error ");
        }

        sleep(8);
    }

    //close(sockfd);
    return 0;
}

