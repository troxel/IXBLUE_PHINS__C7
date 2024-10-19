/*--------------------------------------------------

Utility Function include to facilitate UDP communications.

Version History
----------------
Orignal: Steve Troxel Jan 2024  
---------------------------------------------------*/

#ifndef parameters_util_h
#define parameters_util_h

#ifdef QNX
   #define LOCAL_IP "130.46.83.30"
   #define PHINS_IP "130.46.83.248"
#else
   #define LOCAL_IP "130.46.83.30"
   #define PHINS_IP "130.46.83.248"
#endif

#define PHINS_STDBIN_PORT 2002
#define PHINS_GPS_PORT 8121
#define PHINS_DPTH_PORT 8122
#define PHINS_CMD_PORT 8110

#define BUFFER_SIZE 1024

#define INIT_LAT 47.9794444
#define INIT_LON -116.5602778

#define BUFFER_SIZE 1024

#define SHM_FILE "phins_data"

#define SEM_TIME_FILE "/gps_timing"

#endif
