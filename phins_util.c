/*--------------------------------------------------

Utility Functions related to communicating with the PHINS.

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
#include <math.h>
#include <time.h>
#include <stdint.h>

#include "phins_util.h"

// -------------------------------------
// GPS related functions
// -------------------------------------

// -------------------------------------
void buildGGA(const struct gps_data_t *gps_data, char *nmea_str) {

    char tmp_str[256];
    size_t maxSize = 512; 

    // Convert to gga representation of lat/lon
    char lat_gga[21];
    char lon_gga[21];

    cnvt_degree_to_gga(gps_data->latitude,lat_gga);
    cnvt_degree_to_gga(gps_data->longitude,lon_gga);

    // Build from current time
    time_t now = time(NULL);
	struct tm* utc_time = gmtime(&now);

    // $GPGGA,210540.92,4758.62885822,N,11633.67555637,W,6,03,124721.781,-0.018,M,,M,,*73

    // Format the NMEA GGA string (without the checksum)
    snprintf(tmp_str, sizeof(tmp_str),
        "GPGGA,%02u%02u%02u,%s,N,%s,W,%d,%02d,%.1f,%.2f,M,,,,0000",
        utc_time->tm_hour,utc_time->tm_min,utc_time->tm_sec,
        lat_gga,
        lon_gga,
        gps_data->quality, 
        gps_data->satellites, 
        gps_data->hdop,
        gps_data->altitude);

    // $GPGGA,023756,4802.07087454,N,11633.61666800,W,1,00,15.0,0.00,M,,M,,*43

    // From Interface Manual page 137
    // $--GGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M,,,,0000*18
    // $--GGA,064036.289,4836.5375,N,00740.9373,E,1,04,3.2,200.2,M,,,,0000*0E
    
    // $GPGGA,223815,    4802.0708,N,11633.6166,W,2,08,1.1,15.00,M,,,,0000*0F
    // Calculate the checksum
    unsigned char checksum = checksum_gps(tmp_str);

    // Format the final NMEA string with checksum
    snprintf(nmea_str, maxSize, "$%s*%02X\r\n", tmp_str, checksum);
}

// -------------------------------------
// Control Commands from chapter 8 Interface doc
// -------------------------------------

// -------------------------------------
void cmd_mnl_gps(const struct cmd_mnl_gps_data_t *cmd_data, char *cmd_str) {

    char tmp_str[200];

    snprintf(tmp_str, sizeof(tmp_str),"PIXSE,CONFIG,MANGPS,%lf,%lf,%lf,%f,%f,%f",
        cmd_data->lat, cmd_data->lon, cmd_data->alt, 
        cmd_data->lat_stddev, cmd_data->lon_stddev, cmd_data->alt_stddev);
 
    unsigned char checksum = checksum_gps(tmp_str);

    // Format the final NMEA string with checksum
    sprintf(cmd_str, "$%s*%02X\r\n", tmp_str, checksum); 
}

// -------------------------------------
void cmd_zupt(uint8_t mode, char *cmd_str) {

    char tmp_str[200];

    sprintf(tmp_str,"PIXSE,CONFIG,ZUP___,%d",mode);
    unsigned char checksum = checksum_gps(tmp_str);

    // Format the final NMEA string with checksum
    sprintf(cmd_str, "$%s*%02X\r\n", tmp_str, checksum); 
}

// -------------------------------------
void cmd_reset(char *cmd_str) {
    strcpy(cmd_str,"$PIXSE,CONFIG,RESET_*57\r\n");
}

// -------------------------------------
unsigned char checksum_gps(const char *nmeaStr) {
    unsigned char checksum = 0;
    while (*nmeaStr) {
        checksum ^= *nmeaStr++;
    }
    return checksum;
}

// -------------------------------------------------------
// Note this is not a general purpose conversion but works 
// for lat/lon expected at lpo idaho. 
void cnvt_degree_to_gga(double degree_in, char* gga_latlon)
{
	double degree_floor = fabs(degree_in);

	uint16_t degree_int = floor(degree_floor);

	// Remove the whole number from the total to find the decimal.
	double degree_fraction = degree_floor - degree_int;

	// Multiply by 60 to get the minutes value. 
	double minutes = degree_fraction * 60;

	// Now assemble the weird GGA way of doing lat/lon lllmm.mmmmm
    sprintf(gga_latlon,"%u%011.8lf",degree_int,minutes);
}