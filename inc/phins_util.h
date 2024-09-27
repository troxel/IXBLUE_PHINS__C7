/*--------------------------------------------------


Version History
----------------
Orignal: Troxel March 2024  
---------------------------------------------------*/

#ifndef phins_util_h
#define phins_util_h

// -------------- Types ------------

// GPS GGA data structs
struct gps_data_t {
    uint64_t time_epoch;
    char* time_str[12];
    double latitude_rtk;
    double longitude_rtk;
    double latitude;
    double longitude;
    double position_err;    // Added magnitude of error
    double altitude;        // Altitude in meters
    int quality;            // GPS Quality indicator
    float hdop;             // Horiz dilution of precision
    float orth_height;      // Orthometric Height (depth?)
    int satellites;         // Number of satellites in use
    int atacs;              // Signifies ATACS update
};

// Not used. 
struct cmd_mnl_gps_data_t {
   double lat;
   double lon;
   double alt;
   float lat_stddev;
   float lon_stddev;
   float alt_stddev;
};

// -------------- Protos ------------
uint32_t l2b_32(uint8_t * bp);
uint16_t l2b_16(uint8_t b1,uint8_t b2);
uint32_t b2l_32(uint8_t * bp);
uint16_t b2l_16(uint8_t b1,uint8_t b2);
uint16_t l2b_16(uint8_t b1,uint8_t b2);
float b2l_f(uint8_t * bp);
float b2l_lf(uint8_t * bp);

uint32_t checksum(uint8_t * bp, ssize_t length);
unsigned char checksum_gps(const char *nmea_str);

void buildGGA(const struct gps_data_t *gps_data, char *nmea_str);
void cnvt_degree_to_gga(double degree_in, char* gga_latlon);

void cmd_mnl_gps(const struct cmd_mnl_gps_data_t *gps_data, char *msg_str_out);
void cmd_zupt(uint8_t mode, char *cmd_str);
void cmd_reset(char *cmd_str);

void buildDEPIN(const float dep, char *nmea_str);

#endif
