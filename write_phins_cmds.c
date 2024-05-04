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

void reset_self() {

   printf("resetting systemctl restart phins_write_phins_cmds.service\n");
   system("sudo systemctl restart phins_write_phins_cmds.service");
   printf("bye\n");
   fflush(stdout);
}


/* -------------------------------------------*/
int main(int argc, char *argv[]) {

    int sockfd;

    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
       perror("socket()");
       exit(1);
    }

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
  
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(PHINS_CMD_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(PHINS_IP);

    // Connect to the server
    uint16_t cnt = 1;
    char err_msg[256];
    printf("Attempt connection\n");
    while ( connect(sockfd, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0 ) {
        sprintf(err_msg,"%d Connection Failed... Retrying",cnt);
        perror(err_msg);
        sleep(2);
        if ( cnt++ > 100 ) { exit(0); }
    }
    printf("Connected %s\n", PHINS_IP); 

    // Time stuff
    time_t tm;
    struct tm *tmp;
    const char* fmt = "%a, %d %b %y %T";
    char tm_str[50];

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
    uint8_t reset_flg = 0; 
    uint32_t count = 0;

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
                reset_flg = 1;

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

                rtn = send(sockfd, cmd_str, strlen(cmd_str), 0);
                if ( rtn < 0 ) {  
                    perror("Send failed resetting");
                    reset_self();
                } 

                printf("\nSENT %ld BYTES\n\n",rtn);
                fflush(stdout);

                // If reset is sent have to restart this process as tcp doesn't work anymore
                // Might use RS232 to solve this behavior. 
                if ( reset_flg ) {

                    int rtn;

                    // Reset as tcp needs to reconnect.
                    sleep(40);
                    reset_self();
                }
            }

        } // end file command handling. 

        if ( ( count++ % 120 ) == 0 ) {
            tm = time(NULL);
            tmp = localtime(&tm);
            strftime(tm_str, sizeof(tm_str), fmt, tmp);
            printf("%s\n", tm_str); 
            fflush(stdout);
        }

        sleep(1);
    
        // Not reading that data sent from the PHINS at this point. 
        // It sends out PHINS standard protocol at 1 Hz.
        rtn = read( sockfd , read_buffer, 5120);
        //printf("-------\n%s\n", read_buffer); 
        //printf("%lu\n", rtn); 
 
    }

    close(sockfd);
    return 0;
}
