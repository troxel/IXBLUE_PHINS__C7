#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>
#include <unistd.h>
#include <byteswap.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>

// shared mem stuff
#include <sys/mman.h>
#include <fcntl.h>

// Semaphor
#include <semaphore.h>
#include <sys/stat.h>

// project
#include "shm_util.h"
#include "parameters.h"

char filename[48];
char filespec[64];
char linkspec[64];
char *data_dir = "/opt/data";

/* ---------------------------------------------------------*/
/*  */
/*----------------------------------------------------------*/
void print_hdr(FILE * fp) {

    // Print out header
    fprintf(fp,"ET,");  
    fprintf(fp,"Ttag_S,");
    fprintf(fp,"CV_Ack,"); 
    fprintf(fp,"CV_Rej,"); 
    fprintf(fp,"Sub_Mode,");
    fprintf(fp,"Mode,");
    fprintf(fp,"Align_Stat,");
    fprintf(fp,"Align_Prg,");
    fprintf(fp,"Validity_1,");
    fprintf(fp,"Status_1,");
    fprintf(fp,"Lat_S,");
    fprintf(fp,"Lon_S,");
    fprintf(fp,"Alt,");
    fprintf(fp,"VXB,"); 
    fprintf(fp,"VYB,");
    fprintf(fp,"VZB,");
    fprintf(fp,"AXB,");
    fprintf(fp,"AYB,");
    fprintf(fp,"AZB,");
    fprintf(fp,"CBNXX,");
    fprintf(fp,"CBNXY,");
    fprintf(fp,"CBNXZ,");
    fprintf(fp,"CBNYX,");
    fprintf(fp,"CBNYY,");
    fprintf(fp,"CBNYZ,");
    fprintf(fp,"CNEXX,");
    fprintf(fp,"CNEXY,");
    fprintf(fp,"CNEXZ,");
    fprintf(fp,"CNEYX,");
    fprintf(fp,"CNEYY,");
    fprintf(fp,"CNEYZ,");
    fprintf(fp,"Roll,");
    fprintf(fp,"Pitch,");
    fprintf(fp,"Yaw,");
    fprintf(fp,"R_Accel,");
    fprintf(fp,"P_Accel,");
    fprintf(fp,"Y_Accel,");	
    fprintf(fp,"Alpha,");
    fprintf(fp,"INS_ERR_WD,"); 
    fprintf(fp,"time_epoch\n"); 

}

/* ---------------------------------------------------------*/
/*  */
/*----------------------------------------------------------*/
void print_rec_csv(PHINS_FRAME_t *pframe_p, FILE * fp) {
    
    char csv_str[1024];
    memset(csv_str, 0, sizeof(*csv_str));

    PHINS_DATA_t *phd_p = &pframe_p->phins_status.phins_data; 

    int i,j = 0;
    int cnt = 0;

    cnt = sprintf(csv_str, "%.7f," , .05);  // ET
    cnt += sprintf(csv_str+cnt, "%.7f,", .06);   // GNC time
    cnt += sprintf(csv_str+cnt, "%hu,", phd_p->INS_CV_Ack);
    cnt += sprintf(csv_str+cnt, "%hu,", phd_p->INS_CV_Rej);
    cnt += sprintf(csv_str+cnt, "%hu,", phd_p->INS_mode);
    cnt += sprintf(csv_str+cnt, "%hu,", phd_p->INS_submode);
    cnt += sprintf(csv_str+cnt, "%hhu,", phd_p->INS_align_stat);
    cnt += sprintf(csv_str+cnt, "%hhu,", phd_p->INS_align_prg);
    cnt += sprintf(csv_str+cnt, "%hhu,", phd_p->INS_validity1);
    cnt += sprintf(csv_str+cnt, "%hhu,", phd_p->INS_status1);
    cnt += sprintf(csv_str+cnt, "%.15f,", phd_p->INS_lat);
    cnt += sprintf(csv_str+cnt, "%.15f,", phd_p->INS_long);

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
    cnt += sprintf(csv_str+cnt, "%hhx,",0);
    cnt += sprintf(csv_str+cnt, "%lu",pframe_p->time_epoch);
 
    fprintf(fp,"%s\n",csv_str);
    fflush(fp);

}

/* -------------------------------------------*/
FILE *open_file()
{

    time_t t;
    struct tm *tmp;
    //char filename[64];

    // Get current time
    t = time(NULL);
    tmp = localtime(&t);
    if (tmp == NULL) {
        perror("ERROR on localtime");
        exit(1);
    }

    // Create file name
    // strftime(filename, sizeof(filename), "%Y_%m_%d_%H-%M_phins.bin", tmp);
    strftime(filename, sizeof(filename), "%Y_%m_%d_%H-%M_phins.csv", tmp);
    snprintf(filespec,sizeof(filespec),"%s/%s",data_dir,filename);

    // Open file
    FILE *fp = fopen(filespec, "wb");
    if (fp == NULL) {
        perror("ERROR opening file");
        exit(1);
    }

    printf("Opened file %s\n",filespec);

    snprintf(linkspec,sizeof(linkspec),"%s/%s",data_dir,"phins_data_current.csv");
    // snprintf(linkspec,sizeof(linkspec),"%s/%s",data_dir,"phins_data_current.bin");
    unlink(linkspec);
    symlink(filespec,linkspec);

    return (fp);
}

/* ---------------------------------------------------------*/
/* if a file named "record" exists in directory record data */
/*                                                          */
/* Modified to print csv vice bin 4/2/2024                  */
/*                                                          */
/*----------------------------------------------------------*/
void record_data(PHINS_FRAME_t *phins_frame_p) {

    static FILE *fp = NULL;
    //PHINS_DATA_t *phins_data_p; // use if printing binary 

    // If file 'record' exist record data to file
    if (access("record", F_OK) == 0) {

        if (! fp ) {
            fp = open_file();

            print_hdr(fp);
        }

        if ( fp ) {
            
            //phins_data_p = &(phins_frame_p->phins_status.phins_data);
            //fwrite(phins_data_p, sizeof(PHINS_DATA_t), 1, fp);

            print_rec_csv(phins_frame_p, fp);

            //fwrite(phins_frame_p, sizeof(PHINS_FRAME_t), 1, fp);
            //fflush(fp);
            //printf(">> status >> %0X %0x\n",phins_frame_p->algo_stat[0],phins_frame_p->algo_stat[1]);
        }

    } else {

        if ( fp ) {
            printf("Closing file %s\n",filespec);
            fclose(fp);
            fp = NULL;
        }
    }
}

/* -------------------------------------------*/
int main(int argc, char *argv[])
{
    int cnt = 0;

    PHINS_FRAME_t phins_frame;
    static uint32_t count = 0;

    // -----------------------------
    // Semaphor timing
    // -----------------------------
    sem_t *sem = sem_open(SEM_TIME_FILE, O_CREAT, 0644, 0);

    while ( 1 )
    {
        // Wait on gps signal
        sem_wait(sem);

        // Read from shared memory
        read_ins_frame(&phins_frame);

        // print every once in while for monitoring. Remove for GNC implimentation
        if ( ( count++ % 30 ) == 0 )
        {
            printf("lat>%5.2lf algo_stat0 > %04X\n",phins_frame.phins_status.phins_data.INS_lat, phins_frame.algo_stat[0]);
            fflush(stdout);
        }

        // Appending to file for uplake testing
        record_data(&phins_frame);

        cnt++;
        sleep(1);
    }

    sem_close(sem);
    sem_unlink(SEM_TIME_FILE);

    return 0;
}
