#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>
#include <unistd.h>
#include <byteswap.h>

// shared mem stuff
#include <sys/mman.h>
#include <fcntl.h>

#include "udp_util.h"
#include "phins_util.h"
#include "stbn_util.h"
#include "shm_util.h"

#include "parameters.h"

/* -------------------------------------------*/
void error(const char *msg) {
    perror(msg);
    exit(1);
}

void struct2csv(PHINS_FRAME_t*, char*, ssize_t);

/* -------------------------------------------*/
int main(int argc, char* argv[]) {

    FILE *fp = NULL;
    PHINS_DATA_t phd;
    PHINS_FRAME_t pframe;

    memset(&phd, 0, sizeof(PHINS_DATA_t));
    memset(&pframe, 0, sizeof(PHINS_FRAME_t));

    char csv_str[1024];

    if ( argc != 2 ) {
        error("Need to provide a INS binary file on the command line");
    }

    if (access(argv[1], F_OK) != 0) {
        error("Can't open file");
    }

    fp = fopen(argv[1], "rb");
    if ( fp == NULL ) {
        error("Error opening file");
    }
  
    // Print out header
    printf("ET,");  
    printf("Ttag_S,");
    printf("CV_Ack,"); 
    printf("CV_Rej,"); 
    printf("Sub_Mode,");
    printf("Mode,");
    printf("Align_Stat,");
    printf("Align_Prg,");
    printf("Validity_1,");
    printf("Status_1,");
    printf("Lat_S,");
    printf("Lon_S,");
    printf("Alt,");
    printf("VXB,"); 
    printf("VYB,");
    printf("VZB,");
    printf("AXB,");
    printf("AYB,");
    printf("AZB,");
    printf("CBNXX,");
    printf("CBNXY,");
    printf("CBNXZ,");
    printf("CBNYX,");
    printf("CBNYY,");
    printf("CBNYZ,");
    printf("CNEXX,");
    printf("CNEXY,");
    printf("CNEXZ,");
    printf("CNEYX,");
    printf("CNEYY,");
    printf("CNEYZ,");
    printf("Roll,");
    printf("Pitch,");
    printf("Yaw,");
    printf("R_Accel,");
    printf("P_Accel,");
    printf("Y_Accel,");	
    printf("Alpha,");
    printf("INS_ERR_WD"); 
    printf("time_epoc\n"); 

    while(!feof(fp)) {

        fread(&pframe,sizeof(PHINS_FRAME_t),1,fp);

        struct2csv(&pframe, csv_str, sizeof(csv_str));

        printf(csv_str);

        // checking on the width of phthread_mutex_t
        //printf(">>%lu\n",sizeof(pthread_mutex_t));
    }

    fclose(fp);
    return 0;
}

// ------------------------------------------------------------------------
void struct2csv(PHINS_FRAME_t *pframe_p, char *csv_str, ssize_t buf_size) {

    PHINS_DATA_t *phd_p = &pframe_p->phins_status.phins_data; 

    // csv_str better point to a suffient sized buffer or I will hunt you down. 
    int i,j = 0;
    int cnt = 0;

    cnt = sprintf(csv_str, "%.7f," , .04);
    cnt += sprintf(csv_str+cnt, "%.7f," , .05);  // ET
    cnt += sprintf(csv_str+cnt, "%.7f,", .06);   // GNC time
    cnt += sprintf(csv_str+cnt, "%hx,", phd_p->INS_CV_Ack);
    cnt += sprintf(csv_str+cnt, "%hX,", phd_p->INS_CV_Rej);
    cnt += sprintf(csv_str+cnt, "%hX,", phd_p->INS_mode);
    cnt += sprintf(csv_str+cnt, "%hX,", phd_p->INS_submode);
    cnt += sprintf(csv_str+cnt, "%hhX,", phd_p->INS_align_stat);
    cnt += sprintf(csv_str+cnt, "%hhX,", phd_p->INS_align_prg);
    cnt += sprintf(csv_str+cnt, "%hhX,", phd_p->INS_validity1);
    cnt += sprintf(csv_str+cnt, "%hhX,", phd_p->INS_status1);
    cnt += sprintf(csv_str+cnt, "%.15f,", phd_p->INS_lat);
    cnt += sprintf(csv_str+cnt, "%.15f,", phd_p->INS_long);


    printf("%7f %7f\n",phd_p->INS_lat,phd_p->INS_long);


    cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_depth * 3.28084);

    for( i=0;i<3;i++) cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_vel[i]);

    for( i=0;i<3;i++) cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_acc[i]);
    
    for( i=0;i<2;i++) {
        for(j=0;j<3;j++) cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_Cmat[i][j]);
    }     

    for( i=0;i<2;i++) {
        for(j=0;j<3;j++) cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_CNE[i][j]);
    }

    for( i=0;i<3;i++) cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_attitude[i]);
    for( i=0;i<3;i++) cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_att_rate[i]);
    cnt += sprintf(csv_str+cnt, "%.7f,", phd_p->INS_wander_angle);
    cnt += sprintf(csv_str+cnt, "%hhx\n",0);
    cnt += sprintf(csv_str+cnt, "%lu\n",pframe_p->time_epoch);
 
    if ( i > buf_size ) { 
        perror("Buffer Overflow");
        exit(1);
    }

}