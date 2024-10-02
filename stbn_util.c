/*--------------------------------------------------

Utility Functions related to communicating with the PHINS.

Version History
----------------
Orignal: SOT Jan 2024  
---------------------------------------------------*/

#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include "shm_util.h"
#include "stbn_util.h"
#include "phins_util.h"

#define CNVT_CNE (pow(2.0,30))
#define CNVT_LATLON (pow(2.0,31))

double deg2rad = M_PI/180; 
double m2ft = 3.280839895;
uint8_t vb = 0;

// -----------------------------------------------------------------------------------
// proc_stbn_buf() - processes the buffer return from the PHINS and parcels out the 
// various bits of data into the supplied stuctures. The stdbin protocol is composed 
// of user selected blocks that is configured via the web interface. This is not a 
// general purpose solution that reads the masks and figures out the offsets in the 
// buffers. There is a Perl utility function in ./util that helps determine the offsets.
//
// Current Selections
// ------------------------------------------------------------
// 0,      25,     Attitude & Heading
// 1,      37,     Attitude & Heading standard deviation
// 4,      49,     Heading/Roll/Pitch Rate
// 6,      61,     Acceleration in Subsea vehicle frame
// 7,      73,     Position
// 8,      94,     Position standard deviation
// 9,      110,    Speed in geographic frame
// 15,     122,    INS algorithm status
// 16,     138,    INS system status
// 17,     150,    INS user status
// 22,     154,    Speed in Subsea vehicle frame
// 23,     166,    Acceleration in geographic frame not compensated from gravity
// 24,     178,    Course and speed over ground
// 25,     186,    Temperatures (ACC/FOG/ANA)
// 26,     198,    Attitude quaternion data
// 28,     214,    Raw accelerations in Subsea vehicle frame not compensated from gravity
// 32,     226,    Extended1: Rot Accel in Body frame
// 34,     238,    Extended1: Raw rotation rate in Subsea vehicle frame
// 64,     250,    Extended2: UTC date
// 65,     255,    Extended2: GPS1

// -----------------------------------------------------------------------------------
// This function loads the INS_STATUS_t buffer. Bad name as it actual contains the INS
// data using this name to match the GNC code. 
// -----------------------------------------------------------------------------------
int8_t proc_phins_frame_buf(uint8_t *dg_buf, ssize_t dg_len, PHINS_FRAME_t *phins_frame_in) {
   
    // Check validity... 
    uint32_t chksum = checksum(dg_buf,dg_len);
    uint32_t chksum_pyld = b2l_32(&dg_buf[dg_len-4]);

    if ( chksum != chksum_pyld) {
        printf("Error! Checksum validation failed %d %d\n", chksum, chksum_pyld);
        return(false); 
    }  

    // The byte swapping functions might need to be changed on GCC hardware.
    struct stbn_hdr_t stbn_hdr;

    stbn_hdr.h1    = dg_buf[0];
    stbn_hdr.h2    = dg_buf[1];
    stbn_hdr.proto = dg_buf[2];
    stbn_hdr.navg_mask   = b2l_32(&dg_buf[3]);
    stbn_hdr.extd_mask   = b2l_32(&dg_buf[7]);
    stbn_hdr.extn_mask   = b2l_32(&dg_buf[11]);
    stbn_hdr.stbn_size = b2l_16(dg_buf[15],dg_buf[16]);
    stbn_hdr.data_validity_tm = b2l_32(&dg_buf[17]);
    stbn_hdr.counter          = b2l_32(&dg_buf[21]);

    //phins_frame_in->counter = stbn_hdr.counter; 

    // ----------------------------
    // ---- payload conversion ----
    // ----------------------------

    // Reference deep data structure for convienence 
    PHINS_DATA_t *data_p = &phins_frame_in->phins_status.phins_data; 

    data_p->INS_data_timetag = (float)stbn_hdr.data_validity_tm/10000; 

    // Need lat/long at top to calculate wander angle
    double lat = b2l_lf(&dg_buf[73]) * deg2rad;
    double lon = b2l_lf(&dg_buf[81]) * deg2rad;
    if ( lon > M_PI ) { lon = lon - 2*M_PI; }

    // ---- Ref ISRP_TO_1553_01 ----
    // These are filtered and probably do not have earths rotation rate
    //data_p->INS_att_rate[0] = b2l_f(&dg_buf[49]);
    //data_p->INS_att_rate[1] = b2l_f(&dg_buf[53]);
    //data_p->INS_att_rate[2] = b2l_f(&dg_buf[57]);

    // These are raw body frames with earth rotation rate... assume that is what is wanted. 
    data_p->INS_att_rate[0] = b2l_f(&dg_buf[238]);
    data_p->INS_att_rate[1] = b2l_f(&dg_buf[242]);
    data_p->INS_att_rate[2] = b2l_f(&dg_buf[246]);

    // Raw accels with earth g present (PHINS is not currently sending raw with G)
    data_p->INS_acc[0] = b2l_f(&dg_buf[214]) * m2ft;
    data_p->INS_acc[1] = b2l_f(&dg_buf[218]) * m2ft;
    data_p->INS_acc[2] = b2l_f(&dg_buf[222]) * m2ft;

    // ---- Ref ISRP_TO_1553_03 ----

    // Calc the Wander angle. Equation from linear regression...
    //data_p->INS_wander_angle = -0.7527 * lon - 2.395;

    // Better fit with more precision using complete set of data
    data_p->INS_wander_angle = -0.7439446 * lon -2.37746942;

    // Yaw, Roll, Pitch in PHINS order. Note indices changes to tradition roll,pitch,yaw.
    data_p->INS_attitude[2] = b2l_f(&dg_buf[25]);   // yaw 
    data_p->INS_attitude[0] = b2l_f(&dg_buf[29]);   // roll 
    data_p->INS_attitude[1] = - b2l_f(&dg_buf[33]); // pitch (NWU to NED pitch is reversed)

    data_p->INS_attitude[2] *= deg2rad;
    data_p->INS_attitude[0] *= deg2rad; 
    data_p->INS_attitude[1] *= deg2rad; 

    // Add the wander angle to the yaw
    data_p->INS_attitude[2] += data_p->INS_wander_angle;

    data_p->INS_vel[0] = b2l_f(&dg_buf[154]) * m2ft;
    data_p->INS_vel[1] = b2l_f(&dg_buf[158]) * m2ft;
    data_p->INS_vel[2] = b2l_f(&dg_buf[162]) * m2ft;

    // ---- Ref ISRP_TO_1553_02 ----
    double cos_wan = cos(data_p->INS_wander_angle);
    double sin_wan = sin(data_p->INS_wander_angle);

    double cos_r = cos(data_p->INS_attitude[0]);
    double sin_r = sin(data_p->INS_attitude[0]);

    double cos_p = cos(data_p->INS_attitude[1]);
    double sin_p = sin(data_p->INS_attitude[1]);

    double cos_y = cos(data_p->INS_attitude[2]);
    double sin_y = sin(data_p->INS_attitude[2]);

    data_p->INS_Cmat[0][0] = cos_p * cos_y;
	data_p->INS_Cmat[0][1] = (sin_r * sin_p * cos_y) - (sin_y * cos_r);
	data_p->INS_Cmat[0][2] = (sin_p * cos_r * cos_y) + (sin_r * sin_y);

	data_p->INS_Cmat[1][0] = sin_y * cos_p;
	data_p->INS_Cmat[1][1] = (sin_r * sin_p * sin_y) + (cos_r * cos_y);
	data_p->INS_Cmat[1][2] = (sin_p * sin_y * cos_r) - (sin_r * cos_y);

    // ---- Ref ISRP_TO_1553_04 ----
    //data_p->alt_ref = dg_buf[89];
    data_p->INS_depth = b2l_f(&dg_buf[90]);

    double cos_lat = cos(lat);
    double sin_lat = sin(lat);

    double cos_lon = cos(lon);
    double sin_lon = sin(lon);

    data_p->INS_CNE[0][0] = (double)(  (sin_wan * sin_lon) - (cos_wan * sin_lat * cos_lon) );
    data_p->INS_CNE[0][1] = (double)( -(cos_wan * sin_lon) - (sin_wan * sin_lat * cos_lon) );
    data_p->INS_CNE[0][2] = (double)( -(cos_lat * cos_lon) );
    data_p->INS_CNE[1][0] = (double)( -(sin_wan * cos_lon) - (cos_wan * sin_lat * sin_lon) );
    data_p->INS_CNE[1][1] = (double)(  (cos_wan * cos_lon) - (sin_wan * sin_lat * sin_lon) );
    data_p->INS_CNE[1][2] = (double)( -(cos_lat * sin_lon) );

    // ---- Ref ISRP_TO_HUD_1 ----
    data_p->INS_lat  = lat;  // Radians
    data_p->INS_long = lon;  // Radians

    // Status, modes and progress handled seperately. 

    phins_frame_in->algo_stat[0] = b2l_32(&dg_buf[122]);
    phins_frame_in->algo_stat[1] = b2l_32(&dg_buf[126]);
    phins_frame_in->algo_stat[2] = b2l_32(&dg_buf[130]);
    phins_frame_in->algo_stat[3] = b2l_32(&dg_buf[134]);

    phins_frame_in->sys_stat[0] = b2l_32(&dg_buf[138]);
    phins_frame_in->sys_stat[1] = b2l_32(&dg_buf[142]);
    phins_frame_in->sys_stat[2] = b2l_32(&dg_buf[146]);

    phins_frame_in->usr_stat = b2l_32(&dg_buf[150]);

    uint32_t algo_stat0 = phins_frame_in->algo_stat[0]; 

    //printf("algo status ---------->> %04X \n",algo_stat0);

    // -------------------
    // Alignment Progress 
    // -------------------
    // Following Submergence code on alignment progress. 
    data_p->INS_align_prg = 0;
    if ( algo_stat0 & 0x01 )
    {
        // Submergence comment:  After fine alignment 0.8nm/hr accuracy, to match existing system
        data_p->INS_align_prg = 12;
    }
    else if ( algo_stat0 & 0x04 )
    {
        // Submergence comment: During fine alignment 3.2nm/hr accuracy, to allow GNC Kalman filter fine alignment
        //
        // However the PHINS transitions from Fine alignment to Nav mode in about 3 seconds.
        // The transition of the modes is like this 
        //
        //              0x02 -> 0x07 -> 0x05 
        // ~-- 5min ------|---- 3 Sec ----|
        // Where 
        // 0x01 = Nav mode
        // 0x02 = Course Align
        // 0x04 = Fine
        //
        // Therefore this state fine alignment (9) will never show up
        data_p->INS_align_prg = 9;
    }
    else if ( algo_stat0 & 0x02) 
    {
        //init
        data_p->INS_align_prg = 0;
    }

    // -------------------
    // INS_validity1 (ISRP_TO_1553_01)
    // -------------------
    // bit 0 -> Body Frame Valid
    // bit 1 -> Attitude valid
    // bit 2 -> Altitude loop valid
    // bit 8 -> correction vector applied
    // -------------------
    uint16_t validity1 = 0x0; 

    if ( algo_stat0 & 0x01 ) {
        validity1 = validity1 | 0x01;
        validity1 = validity1 | 0x02;
        //validity1 = validity1 | 0x04;  // Data from lsv run shows this bit off during normal ops 
    }
    
    // correct vector (now atacs) received and accepted. 
    if ( algo_stat0 & 0x2000 ) {
        validity1 = validity1 | 0x80;
    }
    data_p->INS_validity1 = validity1;

    // -------------------
    // INS_status1 (ISRP_TO_1553_03)
    // -------------------
    // bit 0 -> ATT Good
    // bit 1 -> NAV Good
    // bit 2 -> Altitude loop valid
    // bit 3 -> Altitude reverence available 
    // bit 4 -> Pure Inertial Altitude 
    // bit 5 -> No Time Sync Pulse 
    // -------------------
    uint16_t status1 = 0x0; 
    
    if ( algo_stat0 & 0x01 ) {
        status1 = status1 | 0x01;
        status1 = status1 | 0x02;
        status1 = status1 | 0x10;
        status1 = status1 | 0x20;
    }
    
    data_p->INS_status1 = status1;

    // -------------------
    // INS_mode (ISRP_TO_1553_07)
    // -------------------
    // enum 0 -> Init
    // enum 1 -> Level
    // enum 2 -> Stored Heading Alignment
    // enum 3 -> Gyrocompass Alignment
    // enum 4 -> Navigation Mode  
    // enum 5 -> Attitude Reference  
    // enum 6 -> Test  
    // -------------------
    uint16_t mode = 0x0; 

    if ( algo_stat0 & 0x01 ) {
        mode = 4;
    } 
    data_p->INS_mode = mode;

    // -------------------
    // INS_submode (ISRP_TO_1553_07)
    // -------------------
    // enum 0 -> None
    // enum 1 -> Best Available True Heading
    // enum 2 -> Enhanced Interrupted Align (align in two different headings)
    // enum 3 -> In flight Align 
    // enum 4 -> Aided Navigation   
    // -------------------
    uint16_t submode = 0; 
    if ( algo_stat0 & 0x01 ) {
        submode = 4;
    } 
    data_p->INS_submode = submode;

    // -------------------
    // INS_align_stat (ISRP_TO_1553_07)
    // -------------------
    // Bit 0 = Level Complete
    // Bit 1 = Degraded Nav Ready
    // Bit 2 = Nav Ready
    // Bit 8 = GC Alignment performed
    // Bit 9 = Aligned to last lat/long
    // Bit 10 = Early Nav Selected

    // Not sure what to do. Notice in lake data this word is 257 -> 1 0000 0001
    uint16_t align_stat = 0; 
    if ( align_stat & 0x01 ) {
         align_stat = align_stat | 0x01;
         align_stat = align_stat | 0x80;
    }
    else if ( algo_stat0 & 0x04) {
        align_stat = align_stat | 0x01;
        align_stat = align_stat | 0x02;
        align_stat = align_stat | 0x04;
        align_stat = align_stat | 0x80;
    }
    else if ( algo_stat0 & 0x02 )
    {
        align_stat = align_stat | 0x01;
    }
    data_p->INS_align_stat = align_stat;

    // -------------------
    // INS_CV_Ack/Rej (ISRP_TO_1553_07)
    // -------------------
    // INS_CV_Ack boolean
    // INS_CV_Ack boolean

    data_p->INS_CV_Ack = 0;
    if ( algo_stat0 & 0x01000 ) {   // GPS received
        data_p->INS_CV_Ack = 1;
    }

    data_p->INS_CV_Rej = 0;
    if ( algo_stat0 & 0x08000 ) {    // GPS Rejected
        data_p->INS_CV_Rej = 1;
    }

    return(1);
}

// -----------------------------------------------------------------------------------
// Old code below. The first approach was to package the data into byte string to look just 
// like the LN100. Later I decided that using the native INS_Status struct was more efficient. 
//------------------------------------------------------------------------------------
// int8_t proc_stbn_buf(uint8_t *dg_buf, ssize_t dg_len, struct stbn_hdr_t *stbn_hdr, struct stbn_payload_v1_t *stbn_pyld) {

//     // First check validity... 
//     uint32_t chksum = checksum(dg_buf,dg_len);
//     uint32_t chksum_pyld = b2l_32(&dg_buf[dg_len-4]);

//     if ( chksum != chksum_pyld) {
//         printf("Error! Checksum validation failed %d %d\n", chksum, chksum_pyld);
//         return(-1); 
//     }  

//     // The byte swapping functions might need to be changed on GCC hardware. 

//     stbn_hdr->h1    = dg_buf[0];
//     stbn_hdr->h2    = dg_buf[1];
//     stbn_hdr->proto = dg_buf[2];
//     stbn_hdr->navg_mask   = b2l_32(&dg_buf[3]);
//     stbn_hdr->extd_mask   = b2l_32(&dg_buf[7]);
//     stbn_hdr->extn_mask   = b2l_32(&dg_buf[11]);
//     stbn_hdr->stbn_size = b2l_16(dg_buf[15],dg_buf[16]);
//     stbn_hdr->data_validity_tm = b2l_32(&dg_buf[17]);
//     stbn_hdr->counter         = b2l_32(&dg_buf[21]);

//     // ---- payload conversion ----
//     stbn_pyld->attitude[0] = b2l_f(&dg_buf[25]) * deg2rad;    // yaw
//     stbn_pyld->attitude[1] = b2l_f(&dg_buf[29]) * deg2rad;    // roll
//     stbn_pyld->attitude[2] = b2l_f(&dg_buf[33]) * deg2rad;    // pitch

//     stbn_pyld->rpy_rate[0] = b2l_f(&dg_buf[49]);
//     stbn_pyld->rpy_rate[1] = b2l_f(&dg_buf[53]);
//     stbn_pyld->rpy_rate[2] = b2l_f(&dg_buf[57]);

//     stbn_pyld->bdy_lin_accl[0] = b2l_f(&dg_buf[61]) * m2ft;
//     stbn_pyld->bdy_lin_accl[1] = b2l_f(&dg_buf[65]) * m2ft;
//     stbn_pyld->bdy_lin_accl[2] = b2l_f(&dg_buf[69]) * m2ft;

//     stbn_pyld->lat = b2l_lf(&dg_buf[73]);
//     stbn_pyld->lon = b2l_lf(&dg_buf[81]);
//     if ( stbn_pyld->lon > 180 ) { stbn_pyld->lon = stbn_pyld->lon - 360; }

//     stbn_pyld->alt_ref = dg_buf[88];
//     stbn_pyld->alt = b2l_f(&dg_buf[89]);

//     stbn_pyld->ins_algo_stat[0] = b2l_32(&dg_buf[122]);
//     stbn_pyld->ins_algo_stat[1] = b2l_32(&dg_buf[126]);
//     stbn_pyld->ins_algo_stat[2] = b2l_32(&dg_buf[130]);
//     stbn_pyld->ins_algo_stat[3] = b2l_32(&dg_buf[134]);

//     stbn_pyld->ins_sys_stat[0] = b2l_32(&dg_buf[138]);
//     stbn_pyld->ins_sys_stat[1] = b2l_32(&dg_buf[142]);
//     stbn_pyld->ins_sys_stat[2] = b2l_32(&dg_buf[146]);

//     stbn_pyld->ins_usr_stat = b2l_32(&dg_buf[150]);

//     stbn_pyld->bdy_velocity[0] = b2l_f(&dg_buf[154]) * m2ft;
//     stbn_pyld->bdy_velocity[1] = b2l_f(&dg_buf[158]) * m2ft;
//     stbn_pyld->bdy_velocity[2] = b2l_f(&dg_buf[162]) * m2ft;

//     stbn_pyld->geo_accl[0] = b2l_f(&dg_buf[166]) * m2ft;
//     stbn_pyld->geo_accl[1] = b2l_f(&dg_buf[170]) * m2ft;
//     stbn_pyld->geo_accl[2] = b2l_f(&dg_buf[174]) * m2ft;

//     stbn_pyld->temps[0] = b2l_f(&dg_buf[186]);
//     stbn_pyld->temps[1] = b2l_f(&dg_buf[190]);
//     stbn_pyld->temps[2] = b2l_f(&dg_buf[194]);

//     stbn_pyld->quat[0] = b2l_f(&dg_buf[198]);
//     stbn_pyld->quat[1] = b2l_f(&dg_buf[202]);
//     stbn_pyld->quat[2] = b2l_f(&dg_buf[206]);
//     stbn_pyld->quat[3] = b2l_f(&dg_buf[210]);

//     stbn_pyld->bdy_lin_accl_raw[0] = b2l_f(&dg_buf[214]) * m2ft;
//     stbn_pyld->bdy_lin_accl_raw[1] = b2l_f(&dg_buf[218]) * m2ft;
//     stbn_pyld->bdy_lin_accl_raw[2] = b2l_f(&dg_buf[222]) * m2ft;
    
//     stbn_pyld->bdy_ang_accl[0] = b2l_f(&dg_buf[226]) * deg2rad;
//     stbn_pyld->bdy_ang_accl[1] = b2l_f(&dg_buf[230]) * deg2rad;
//     stbn_pyld->bdy_ang_accl[2] = b2l_f(&dg_buf[234]) * deg2rad;
    
//     stbn_pyld->bdy_ang_rate[0] = b2l_f(&dg_buf[238]) * deg2rad;
//     stbn_pyld->bdy_ang_rate[1] = b2l_f(&dg_buf[242]) * deg2rad;
//     stbn_pyld->bdy_ang_rate[2] = b2l_f(&dg_buf[246]) * deg2rad;

//     if ( vb == 1 )
//     {
//         printf("%X %X %X %X  ",stbn_pyld->ins_algo_stat[0],stbn_pyld->ins_algo_stat[1],stbn_pyld->ins_algo_stat[2],stbn_pyld->ins_algo_stat[3]);
//         printf("%X %X %X ",stbn_pyld->ins_sys_stat[0],stbn_pyld->ins_sys_stat[1],stbn_pyld->ins_sys_stat[2]);
//         printf("%f %f %f ",stbn_pyld->attitude[0],stbn_pyld->attitude[1],stbn_pyld->attitude[2]);
//         printf("%lf %lf ",stbn_pyld->lat,stbn_pyld->lon);
//     }

//     printf("\n");
//     printf("Algo Status 0x%X 0x%X \n",stbn_pyld->ins_algo_stat[0],stbn_pyld->ins_algo_stat[1]);
//     printf("lat/lon > %lf %lf\n",stbn_pyld->lat,stbn_pyld->lon);
//     printf("temps   > %f : %f : %f\n\n",stbn_pyld->temps[0],stbn_pyld->temps[1],stbn_pyld->temps[2]);
//     printf("body frame acceleration w0/g        (block 6)> %f : %f : %f\n",stbn_pyld->bdy_lin_accl[0],stbn_pyld->bdy_lin_accl[1],stbn_pyld->bdy_lin_accl[2]);
//     printf("geo frame acceleration            (block 23)> %f : %f : %f\n",stbn_pyld->geo_accl[0],stbn_pyld->geo_accl[1],stbn_pyld->geo_accl[2]);
//     printf("body frame acceleration w/g      (block 28)> %f : %f : %f\n",stbn_pyld->bdy_lin_accl_raw[0],stbn_pyld->bdy_lin_accl_raw[1],stbn_pyld->bdy_lin_accl_raw[2]);
//     printf("body raw rotations w/earth rot (ext block 2)> %f : %f : %f\n",stbn_pyld->bdy_ang_rate[0],stbn_pyld->bdy_ang_rate[1],stbn_pyld->bdy_ang_rate[2]);

//     //printf("\nalt_ref = %d, alt = %lf\n",stbn_pyld->alt_ref,stbn_pyld->alt);

//     fflush(stdout);

//     return(1);
// }


// -------------------------------------
// GNC datablock constructors
// -------------------------------------
// The index in the ins_to_gnc_0# arrays should match those in the getinu.c file in the GNC
// These indexes should match the LN100 doc (TM-447) by subtracting 3 words.  
// -------------------------------------

// -------------------------------------
// Utility Functions
// -------------------------------------
// -------------------------------------
uint32_t b2l_32(uint8_t * bp) {
    uint32_t uint32 = 0;
    uint32 += *(bp) << 24;
    uint32 += *(bp + 1) << 16;
    uint32 += *(bp + 2) << 8;
    uint32 += *(bp + 3);
    return uint32;
}

// -------------------------------------
uint32_t l2b_32(uint8_t * bp) {
    uint32_t uint32 = 0;
    uint32 += *(bp) << 0;
    uint32 += *(bp + 1) << 8;
    uint32 += *(bp + 2) << 16;
    uint32 += *(bp + 3) << 24;
    return uint32;
}

// -------------------------------------
uint16_t b2l_16(uint8_t b1,uint8_t b2) {
    uint16_t uint16 = 0;
    uint16 += b1 << 8;
    uint16 += b2;
    return uint16;
}

// -------------------------------------
uint16_t l2b_16(uint8_t b1,uint8_t b2) {
    uint16_t uint16 = 0;
    uint16 += b2 << 8;
    uint16 += b1;
    return uint16;
}

// -------------------------------------
float b2l_f(uint8_t * bp) {

    float f_val;
    uint8_t b[] = {*(bp+3), *(bp+2), *(bp+1), *(bp) };  // Floating point appears to be stored in lsb first 
    memcpy(&f_val, &b, sizeof(float));
    return f_val;
}

// -------------------------------------
float b2l_fr(uint8_t * bp) {

    float f_val;
    uint8_t b[] = {*(bp), *(bp+1), *(bp+2), *(bp+3) }; 
    memcpy(&f_val, &b, sizeof(float));
    return f_val;
}

// -------------------------------------
float b2l_lf(uint8_t*  bp) {

    double lf_val;
    uint8_t b[8];
    for (int i=0;i<8;i++) {
        b[7-i] = *(bp + i);
    }   

    memcpy(&lf_val, &b, sizeof(double));
    return lf_val;
}

// -------------------------------------
// Ditto of b2l_f but name may cause confusion
// Action the same
// -------------------------------------
float l2b_f(uint8_t * bp) {

    float f_val;
    uint8_t b[] = {*(bp+3), *(bp+2), *(bp+1), *(bp) };  // Floating point appears to be stored in lsb first 
    memcpy(&f_val, &b, sizeof(float));
    return f_val;
}

// -------------------------------------
uint32_t checksum(uint8_t* bp, ssize_t length) {

    uint32_t chksum = 0;
    for (int i=0;i<=length-4;i++) {
        chksum += *(bp + i);
    }   

    return chksum;
}
