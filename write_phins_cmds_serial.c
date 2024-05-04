#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/socket.h>

// Serial stuff
#include <fcntl.h>
#include <termios.h>

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
    //options.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    // per phins documentation parity is set to odd. 
    options.c_cflag |= PARODD;

    //options.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    // per phins documentation 2 stop bits. 
    options.c_cflag |= CSTOPB;

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
int main(int argc, char *argv[]) {

     const char *device = "/dev/ttyUSB2";
    const int baudrate = B57600;
    
    // Time stuff
    time_t tm;
    struct tm *tmp;
    const char* fmt = "%a, %d %b %y %T";
    char tm_str[50];

    // Serial port stuff 
    int serial_fd = open_serial_port(device, baudrate);
    if ( serial_fd < 0 ) {
        return EXIT_FAILURE;
    }

    char read_buf[256];
    memset(read_buf, '\0', sizeof(read_buf));

    // Command file stuff. The loop reads the file, check if the input in the file
    // corresponds to a command and then deletes the file, waiting for it to appear again. 
    char *filename = "./phins_cmd.txt";
    FILE *file;
    char fline[256];

    char read_buffer[5120];
    char cmd_str[512]; 
    memset(cmd_str, 0, 512);

    char command[30];
    uint32_t mode; 

    ssize_t rtn; 

    while (1) {

        if(access(filename, F_OK) != -1) {

            // Open the file
            file = fopen(filename, "r");
            if(file == NULL) {
                perror("Error opening file");
            }
            else {

                // Read the first cmd from the file
                if( fgets(fline, sizeof(fline), file) != NULL ) {

                    command[0] = '\0';
                    if ( strchr(fline,',') ) {
                        if ( sscanf(fline,"%[^,],%u",command,&mode) == 2 ) {
                            printf("Found %s,%u\n",command,mode);            
                        }
                        else {
                            perror("Error Did not recognize command\n");
                            printf("Line in file:\n%s",fline);
                            uint8_t rtn = sscanf(fline,"%[^,],%u",command,&mode);
                            printf("rtn : %d\n",rtn);
                        }
                    }
                    else {
                        sscanf(fline,"%s",command);
                    }

                } else {
                    perror("Command file is empty or an error occurred while reading");
                }
            }

            fclose(file);
            if(remove(filename) != 0) {
                perror("Error deleting file");
            }

            // Now act upon the command read from the file
            cmd_str[0] = '\0';
            if (strcmp(command, "gps") == 0) {

                // Cmd manual gps. Note once this is sent I was not able to send commands to GNNS1. 
                // Might be able on GNNS2... don't know yet. 
                struct cmd_mnl_gps_data_t gps_data; 
                gps_data.lat = INIT_LAT;
                gps_data.lon = INIT_LON;
                gps_data.alt = 15;
                gps_data.lat_stddev = 1;
                gps_data.lon_stddev = 1;
                gps_data.alt_stddev = 1;

                cmd_mnl_gps(&gps_data, cmd_str);

            } else if (strcmp(command, "reset") == 0) {
                
                cmd_reset(cmd_str);

            } else if (strcmp(command, "zupt") == 0) {

                // Input bounds
                if ( ( mode > 6 ) || ( mode < 0 ) ) mode = 0; 
        
                cmd_zupt(mode, cmd_str);
            }
            else {
                printf("No valid command requested: %s\n",command);
            }    

            if ( cmd_str[0] ) {
                printf("\n-------------\nSENDING: %s\n--------------\n", cmd_str);
                fflush(stdout);

                rtn = write(serial_fd, cmd_str, strlen(cmd_str));

                if ( rtn < 0 ) {  
                    perror("Send failed");
                } 
                printf("\nSENT %ld BYTES\n\n",rtn);
            }

        } // end file command handling. 

        tm = time(NULL);
        tmp = localtime(&tm);
        strftime(tm_str, sizeof(tm_str), fmt, tmp);
        printf("%s\n", tm_str); 

        // hanging on the below line
        rtn = read( serial_fd , read_buffer, 5120);
        printf("-------\n%s\n", read_buffer); 
        printf("%lu\n", rtn); 
        //sleep(1);
 
    }

    close(serial_fd);
    return 0;
}
