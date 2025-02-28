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
#include <byteswap.h>
#include <math.h>
#include <error.h>

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

#define ONE_FOOT_IN_DEGREES (1.0/364000.0) // Approximation for simplicity
#define SHM_FILE_ATACS "atacs_data"

double haversine(double lat1, double lon1, double lat2, double lon2);

uint8_t Verbose = true;

// -----------------------------
// Shared memory structures
// -----------------------------
typedef struct {
    uint32_t gps_cnt;
    uint32_t atacs_cnt;
    uint32_t skip_atacs;
    uint32_t quality;
    uint32_t gps_time;
    uint32_t delay;
    pthread_mutex_t mutex;
} ATACS_FRAME_t;

// typedef struct {
//     uint32_t gps_cnt;
//     uint32_t atacs_cnt;
//     uint32_t skip_atacs;
//     uint32_t quality;
//     uint32_t gps_time;
//     uint32_t delay;
// } GPS_FRAME_t;

/* ---------------------------------------------------------*/
/*  */
/*----------------------------------------------------------*/
void print_hdr(FILE * fp) {

    // Print out header
    fprintf(fp,"lat_rtk,lon_rtk,lat,lon,pos_err,alt,qlty,hdop,sats,time_epoch,atacs\n");

}

/* ---------------------------------------------------------*/
/*  */
/*----------------------------------------------------------*/
void print_rec_csv(struct gps_data_t *gdp, FILE * fp) {
    
    char csv_str[1024];
    memset(csv_str, 0, sizeof(*csv_str));

    int cnt = 0;

    cnt = sprintf(csv_str, "%.15lf,", gdp->latitude_rtk);
    cnt += sprintf(csv_str+cnt, "%.15lf," , gdp->longitude_rtk);
    cnt += sprintf(csv_str+cnt, "%.15lf," , gdp->latitude);
    cnt += sprintf(csv_str+cnt, "%.15lf," , gdp->longitude);
    cnt += sprintf(csv_str+cnt, "%.15lf," , gdp->position_err);
    cnt += sprintf(csv_str+cnt, "%.4f,"  , gdp->altitude);
    cnt += sprintf(csv_str+cnt, "%d,"  , gdp->quality);
    cnt += sprintf(csv_str+cnt, "%.4f,", gdp->hdop);
    cnt += sprintf(csv_str+cnt, "%d,",   gdp->satellites);
    cnt += sprintf(csv_str+cnt, "%lu,",   gdp->time_epoch);
    cnt += sprintf(csv_str+cnt, "%u",    gdp->atacs);

    fprintf(fp,"%s\n",csv_str);
    fflush(fp);
}

/* -------------------------------------------*/
FILE *open_file()
{

    time_t t;
    struct tm *tmp;

    char filename[48];
    char filespec[64];
    char linkspec[64];
    char *data_dir = "/opt/data";

    // Get current time
    t = time(NULL);
    tmp = localtime(&t);
    if (tmp == NULL) {
        perror("ERROR on localtime");
        exit(1);
    }

    // Create file name
    strftime(filename, sizeof(filename), "%Y_%m_%d_%H-%M_gps.csv", tmp);
    snprintf(filespec,sizeof(filespec),"%s/%s",data_dir,filename);

    // Open file
    FILE *fp = fopen(filespec, "wb");
    if (fp == NULL) {
        perror("ERROR opening file");
        exit(1);
    }

    printf("Opened file %s\n",filespec);

    snprintf(linkspec,sizeof(linkspec),"%s/%s",data_dir,"gps_data_current.csv");
    unlink(linkspec);
    symlink(filespec,linkspec);

    return (fp);
}

/* ---------------------------------------------------------*/
/* if a file named "record" exists in directory record data */
/*----------------------------------------------------------*/
void record_data(struct gps_data_t *gps_data_p) {

    static FILE *fp = NULL;

    // If file 'record' exist record data to file
    if (access("record", F_OK) == 0) {

        if (! fp ) {
            fp = open_file();
            print_hdr(fp);
        }

        if ( fp ) {

            // // Time stamp in milliseconds from epoch
            // struct timespec ts;
            // clock_gettime(CLOCK_REALTIME, &ts);
            // gps_data_p->time_epoch = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
            
            print_rec_csv(gps_data_p, fp);
            //fwrite(gps_data_p, sizeof(struct gps_data_t), 1, fp);
            fflush(fp);
        }

    } else {

        if ( fp ) {
            printf("Closing gps file");
            fclose(fp);
            fp = NULL;
        }
    }
}

/* -------------------------------------------*/
// Open serial port to read a line at a time
int open_serial_port(const char *device, int baudrate) {

    int fd = open(device, O_RDWR | O_NOCTTY);
    // int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_serial_port: Unable to open device");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE; // Mask the character size bits
    options.c_cflag |= CS8;    // Select 8 data bits
    options.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    options.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable any software flow control
    options.c_iflag |= IGNCR; // Ignore CR
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_lflag |= ICANON; // Canonical mode (line by line)
    tcsetattr(fd, TCSANOW, &options); // Apply the settings to the port
    
    // Blocking mode
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    return fd;
}

/* -------------------------------------------*/
// Function to add random error to a coordinate, error in degrees
double addRandomError(double coordinate) {
    double error = (rand() % 2001 - 1000) / 1000.0; // Random number between -1 and 1
    double degreesError = error * ONE_FOOT_IN_DEGREES;
    return(coordinate + degreesError);
}

/* -------------------------------------------*/
// Function to parse latitude and longitude from DDMM.MMMM and DDDMM.MMMM formats
double parseCoordinate(double coordinate) {

    double degrees, minutes;
    degrees = coordinate / 100.0; // This will get the whole part as degrees and the fraction as minutes

    int intDegrees = (int)degrees; // Truncate to get only the degree part
    minutes = (degrees - intDegrees) * 100.0 / 60.0; // Convert minutes to degrees
    degrees = intDegrees + minutes; // Combine degrees and minutes (now in degrees)
    
    return degrees;
}

/* -------------------------------------------*/
// Function to parse a GPGGA string
void parseGPGGA(char *nmea, struct gps_data_t* gps_data) {
    char *token;
    int part = 0;
  
    double lat_gps,lon_gps; 

    token = strtok(nmea, ",");

    // $GPGGA,001052.00,4758.76758057,N,11633.61604700,E,6,03,38.603,-2.112,M,,M,,*60
    while (token != NULL) {
        part++;

        switch(part) {
            // case 2: // Latitude value
            //     strcpy(*gps_data->time_str,token); // time string
            //    printf(">>>>>>>>>>>>>>>>> %s\n",*gps_data->time_str);

            case 3: // Latitude value
                lat_gps = atof(token); // In GGA format
                break;
            case 5: // Longitude value
                lon_gps = - atof(token); // In GGA format
                break;
            case 7: // Quality
                gps_data->quality = atoi(token);
                break;
            case 8: // Sats
                gps_data->satellites = atoi(token);
                break;
            case 9: // hdop
                gps_data->hdop = atof(token);
                break;
            case 10: // Altitude
                gps_data->altitude = atof(token);
                break;
        }

        token = strtok(NULL, ",");
    }

    // Convert latitude and longitude to decimal degrees
    gps_data->latitude_rtk  = parseCoordinate(lat_gps);
    gps_data->longitude_rtk = parseCoordinate(lon_gps);

    // Add random error ~1 ft
    gps_data->latitude  = addRandomError(gps_data->latitude_rtk);
    gps_data->longitude = addRandomError(gps_data->longitude_rtk);

    gps_data->position_err = haversine(gps_data->latitude, gps_data->longitude, gps_data->latitude_rtk, gps_data->longitude_rtk);
    printf("<>> %lf\n", gps_data->position_err);

}

// ------------------------------
// Struct for incoming commands
// ------------------------------
struct cmd_t {
    char* fspec[2];
    union { int cmd[2];
            struct {
                int delay;
                int skip_atacs;
            } type;
    };
};

// ------------------------------
// Check for commands. The file is erased
// so to undo a command have to send the command 
// again 0 for argument
// ------------------------------
void get_commands(struct cmd_t * cmd_p) {

    uint8_t i;
    for (i = 0; i<2; i++ ) {

        FILE *file;
        //int value = 0;

        // Attempt to open the file for reading
        file = fopen(cmd_p->fspec[i], "r");
        if (file == NULL) {
            //printf("File does not exist. %s\n", cmd_p->fspec[i]);
            continue; // Indicate failure
        }

        // File exists, read an integer from the file
        if ( fscanf(file, "%d", &(cmd_p->cmd[i]) ) != 1 ) {
            fclose(file); // Close the file
            perror("Failed to read");
            continue; // Indicate failure
        }

        fclose(file); // Close the file

        printf("command %s %d\n",cmd_p->fspec[i], cmd_p->cmd[i]);

        // Remove the file
        if (remove(cmd_p->fspec[i]) != 0) {
            perror("Error removing the file");
            continue; // Indicate failure
        }
    }
}

/* -------------------------------------------*/
// MAIN MAIN MAIN
/* -------------------------------------------*/
int main() {

    struct cmd_t cmds = {};
    cmds.fspec[0] = "./delay";
    cmds.fspec[1] = "./skip_atacs";

    //const char *device = "/dev/ttyATACS";
    const char *device = "/dev/ttyUSB0";
    
    //const int baudrate = B19200;
    const int baudrate = B115200;

    // -----------------------------
    // Semaphor timing
    // -----------------------------
    sem_t *sem = sem_open(SEM_TIME_FILE, O_CREAT, 0644, 0);

    // -----------------------------
    // Socket
    // -----------------------------
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

    // Lat/Lon of select LSV track data. 
    // Other parameters at the location: Spd:11.3 RPY: -1.5, -0.6, 37.1
    // Position 1740 in complete.csv
    //lat_in = 48.070420640765484;
    //on_in = -116.48900605297399;

    srand(time(NULL));

    int serial_fd = open_serial_port(device, baudrate);
    if ( serial_fd < 0 ) {
        return EXIT_FAILURE;
    }
    printf("opened serial port %s\n",device);

    char read_buf[256];
    memset(read_buf, '\0', sizeof(read_buf));
    
    // ----------------------------------------------
    // Setup Shared Memory setup
    // ----------------------------------------------
    ATACS_FRAME_t *atacs_frame_p;

    printf(">>> ATACS Frame Size %ld\n\n",sizeof(ATACS_FRAME_t));

    int shm_fd = shm_open(SHM_FILE_ATACS, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
    }
 
    if ( ftruncate(shm_fd, sizeof(ATACS_FRAME_t)) != 0 ) {
        perror("ftruncate error");
        return(1);
    }

    printf("Opened shm %s of size %ld\n",SHM_FILE_ATACS,sizeof(ATACS_FRAME_t));

    atacs_frame_p = mmap(0, sizeof(ATACS_FRAME_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (atacs_frame_p == MAP_FAILED) {
        perror("mmap");
    }

    pthread_mutex_init( &atacs_frame_p->mutex, NULL);

    ATACS_FRAME_t atacs_frame_out; 

    int num_bytes;

    // -------------------------------- 
    // The heart of this process 
    // -------------------------------- 
    while (1) {

        // Block on read a line 
        num_bytes = read(serial_fd, read_buf, sizeof(read_buf) - 1);

        printf(">%s",read_buf);

        // Time stamp in milliseconds from epoch
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        gps_data.time_epoch = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

        // Only care about GGA strings    
        if ( strstr(read_buf,"GGA") == NULL ) continue; 

        // Cheap check on GPS antenna - don't send semaphore if it is disconnected
        if ( strstr(read_buf,",,,,") != NULL ) continue; 

        // Signals getinu_data
        sem_post(sem);

        if (num_bytes > 0) {
            read_buf[num_bytes] = 0; // Null-terminate the string

            //printf(">%d %s\n", cnt, read_buf);

            // Extract the latitude, longitude and attitude
            printf("read data %s\n",read_buf);
            parseGPGGA(read_buf, &gps_data);

            if ( Verbose ) printf("Lat/Lon %.7lf, %.7lf, %.7lf\n", gps_data.latitude, gps_data.longitude, gps_data.altitude);

        } else if (num_bytes < 0) {
            printf("Error reading: %s %d\n", strerror(errno),num_bytes);
	        printf("%d %d %d\n",errno,EAGAIN,EWOULDBLOCK);
	    }

        // Absurd check of incoming
        if ( (gps_data.latitude > 49) || (gps_data.latitude < 47) ) {
            fprintf(stderr, "Latitude error %.7f\n", gps_data.latitude);
            continue;
        }

        if ( (gps_data.longitude < -117 ) || (gps_data.longitude > -116) ) {
            fprintf(stderr, "Longitude error %.7f\n", gps_data.longitude);
            continue;
        }

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

        gps_data.altitude = -121.92; // 400 ft
        
        atacs_frame_out.gps_cnt = cnt++;

        get_commands(&cmds);

        // send GGA on a 8 second cycle
        gps_data.atacs = 0;   // indicator if this is an ATACS cycle
        if ( ( cnt % 8 ) == 0 ) {

            // $GPGGA,001052.00,4758.76758057,N,11633.61604700,E,6,03,38.603,-2.112,M,,M,,*60
            // $GPGGA,005704.01,4802.37924115,N,11632.77614893,E,6,03,85592.812,-0.214,M,,M,,*55
            // $GPGGA,054913,4802.07087454,N,11633.61666800,W,1,08,,15.00,M,,M,,*6A
            buildGGA(&gps_data, nmea_str);

            atacs_frame_out.skip_atacs = cmds.type.skip_atacs; 
            atacs_frame_out.delay = cmds.type.delay; 

            atacs_frame_out.quality = gps_data.quality; 

            // Skipping atacs? 
            if ( cmds.type.skip_atacs > 0 ) {
                cmds.type.skip_atacs--;
            }
            else {
                gps_data.atacs = 1; 

                // Delay position to PHINs test    
                if ( cmds.type.delay > 0 ) { usleep(cmds.type.delay);  }

                atacs_frame_out.atacs_cnt++;

                ssize_t rtn = sendto(sockfd,  (const char *)nmea_str, strlen(nmea_str) , 0, (struct sockaddr *) &dest_addr, dest_addr_len);
                if ( rtn < 0 ) {
                    perror("Send Error ");
                }
   
                printf("ATACS %u %lu %s",cnt, gps_data.time_epoch, nmea_str);
                fflush(stdout);
            }
        }

        record_data(&gps_data);

        // Transfer to shared memory region... protecting with mutex 
        // mutex lock/unlock is acting up and squirrelly. 
        // Not using for this file as only for passing information to gui not critical

        // rtn = pthread_mutex_lock( &(atacs_frame_p->mutex) );
        // if ( rtn ) { printf(">lockrtn %u  %s\n",rtn,strerror(rtn)); perror("lock mutex"); } 

        memcpy(atacs_frame_p, &atacs_frame_out, sizeof(ATACS_FRAME_t));
        
        // rtn = pthread_mutex_unlock( &(atacs_frame_p->mutex) );
        // if ( rtn ) { printf(">unlockrtn %u %s\n",rtn,strerror(rtn)); perror("unlock mutex"); } 
    }

    sem_close(sem);
    sem_unlink(SHM_FILE_ATACS);

    //close(sockfd);
    return 0;
}

