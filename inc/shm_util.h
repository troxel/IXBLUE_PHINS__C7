/*--------------------------------------------------

Utility Function include to facilitate UDP communications.

Version History
----------------
Orignal: Troxel Feb 2024  
---------------------------------------------------*/

#ifndef SHARED_H
#define SHARED_H

#include <pthread.h>


// -------------------------------------------
// Original INS Data Struct from GNC
// -------------------------------------------
typedef struct {
    struct {
        float INS_data_timetag;
        unsigned char INS_mode;
        unsigned char INS_submode;
        unsigned char INS_CV_Ack;
        unsigned char INS_CV_Rej;
        unsigned short INS_align_prg;
        unsigned short INS_align_stat;
        unsigned short INS_status1;
        unsigned short INS_validity1;
        double INS_lat;
        double INS_long;
        float INS_depth;
        float INS_vel[3];
        float INS_acc[3];
        float INS_Cmat[2][3];
        float INS_CNE[2][3];
        float INS_attitude[3];
        float INS_att_rate[3];
        float INS_wander_angle;
    }INS_data;
    unsigned short INS_1553_error_word;
    unsigned short Spare_pad;
} INS_STATUS_t;

//----------------------------------
// Analog PHINS struct using a defined type
// instead of an anonymous struct which can't be 
// referenced. 
//----------------------------------
typedef struct {
        float INS_data_timetag;
        unsigned char INS_mode;
        unsigned char INS_submode;
        unsigned char INS_CV_Ack;
        unsigned char INS_CV_Rej;
        unsigned short INS_align_prg;
        unsigned short INS_align_stat;
        unsigned short INS_status1;
        unsigned short INS_validity1;
        double INS_lat;
        double INS_long;
        float INS_depth;
        float INS_vel[3];
        float INS_acc[3];
        float INS_Cmat[2][3];
        float INS_CNE[2][3];
        float INS_attitude[3];
        float INS_att_rate[3];
        float INS_wander_angle;
} PHINS_DATA_t;

// -------------------------------------------
// Identical to the INS_STATUS_t
// -------------------------------------------
typedef struct {
    PHINS_DATA_t phins_data; 
    unsigned short INS_1553_error_word;
    unsigned short Spare_pad;
} PHINS_STATUS_t;

// -------------------------------------------
// Shared memory structure with mutex
// -------------------------------------------
typedef struct {
    PHINS_STATUS_t phins_status;
    pthread_mutex_t mutex;
    uint32_t algo_stat[4];
    uint32_t sys_stat[3];
    uint32_t usr_stat;
    uint64_t time_epoch;
} PHINS_FRAME_t;

uint8_t read_ins_frame(PHINS_FRAME_t *ptr);
void close_shm();

#endif