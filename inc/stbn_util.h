/*--------------------------------------------------

Utility Function include to facilitate UDP communications.

Version History
----------------
Orignal: Troxel Jan 2024  
---------------------------------------------------*/

#ifndef stbn_util_h
#define stbn_util_h

#include "shm_util.h"

// -------------- Types ------------
// STDBIN v3 structs
struct stbn_hdr_t {
    uint8_t h1;
    uint8_t h2;
    uint8_t proto;
    uint32_t navg_mask;
    uint32_t extd_mask;
    uint32_t extn_mask;
    uint16_t stbn_size;
    uint32_t data_validity_tm;
    uint32_t counter;
};

struct stbn_payload_v1_t {
    float attitude[3];
    float rpy_rate[3];
    float bdy_lin_accl[3];
    double lat;
    double lon;
    uint8_t alt_ref;
    float alt;
    uint32_t ins_algo_stat[4];
    uint32_t ins_sys_stat[3];
    uint32_t ins_usr_stat;
    float    bdy_velocity[3];
    float    geo_accl[3];
    uint32_t AHRS_algo_stat;
    uint32_t AHRS_sys_stat[3];
    uint32_t AHRS_usr_stat;
    float    temps[3];
    float    quat[4];
    float    dcm[3][3];
    float    bdy_lin_accl_raw[3];
    float    bdy_ang_accl[3];
    float    bdy_ang_rate[3];
 };



// -------------------- Prototypes ----------------------------------------
int8_t proc_phins_frame_buf(uint8_t *dg_buf, ssize_t dg_len, PHINS_FRAME_t *pins_frame);

int8_t proc_stbn_buf(uint8_t *dg_buf, ssize_t dg_len, struct stbn_hdr_t *stbn_hdr, struct stbn_payload_v1_t *stbn_pyld);

void bld_ins_to_gnc_01(struct stbn_payload_v1_t *stbn_pyld, uint16_t *ins_to_gnc_01);
void bld_ins_to_gnc_02(struct stbn_payload_v1_t *stbn_pyld, uint16_t *ins_to_gnc_02);
void bld_ins_to_gnc_03(struct stbn_payload_v1_t *stbn_pyld, uint16_t *ins_to_gnc_03);
void bld_ins_to_gnc_04(struct stbn_payload_v1_t *stbn_pyld, uint16_t *ins_to_gnc_04);
void bld_ins_to_gnc_07(struct stbn_payload_v1_t *stbn_pyld, uint16_t *ins_to_gnc_07);
void bld_ins_to_hud_01(struct stbn_payload_v1_t *stbn_pyld, uint16_t *ins_to_hud_01);

float b2l_f(uint8_t *);
float l2b_f(uint8_t *);

#endif
