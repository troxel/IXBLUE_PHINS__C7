#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>
#include <unistd.h>
#include <byteswap.h>
#include <errno.h>

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

/* -------------------------------------------*/
int main(int argc, char* argv[]) {

    int sockfd;
    uint8_t dg_buf[BUFFER_SIZE];
    PHINS_FRAME_t *phins_frame_p;
    uint32_t err_cnt=0; 
    int rtn; 

    // Create UDP socket
    sockfd = open_udp_svr_sock(PHINS_STDBIN_PORT);

    ssize_t dg_len; 
    memset(dg_buf, 0, BUFFER_SIZE);

    printf("UDP Socket Opened %d\n",sockfd);

    // Set up the ins data structure and share it.
    int shm_fd = shm_open(SHM_FILE, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
    }
 
    if ( ftruncate(shm_fd, sizeof(PHINS_FRAME_t)) != 0 ) {
        perror("ftruncate error");
        return(1);
    }

    printf("Opened shm %s of size %ld\n",SHM_FILE,sizeof(PHINS_FRAME_t));

    phins_frame_p = mmap(0, sizeof(PHINS_FRAME_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (phins_frame_p == MAP_FAILED) {
        perror("mmap");
    }

    // Initialize the shared memory mutex
    // pthread_mutexattr_t attr;
    // pthread_mutexattr_init(&attr);
    // pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    // pthread_mutex_init( &(phins_frame_p->mutex), &attr);

    // Using just this init results in pthread_mutex_lock to lockup on second cycle through the loop. Not sure why at this point. Rqs more investigation.
    pthread_mutex_init( &(phins_frame_p->mutex), NULL);
    
    uint32_t count = 0;
    PHINS_FRAME_t phins_frame_out; 
    while (1) {

        // Waits for buffer of data... 
        dg_len = read_udp_socket(sockfd, dg_buf, BUFFER_SIZE);
        if ( dg_len < 1 ) {
            perror("Read Error UDP Socket STDBIN Phins");
            err_cnt++;
            sleep(1);
            continue; 
        }

        // Time stamp in milliseconds from epoch
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        phins_frame_out.time_epoch = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

        // ---------------------------------------------------
        //------- Populate phins_frame_out with phins data ----
        // ---------------------------------------------------
        if ( ! proc_phins_frame_buf(dg_buf, dg_len, &phins_frame_out) ) {
            perror("STDBIN Check Sum Failed");
            continue; 
        }

        // Transfer to shared memory region... protecting with mutex 
        // mutex lock/unlock and returning an error but process error is success? Need investigation.
        // If successful returns zero. Otherwise, an error number is returned to indicate the error.
        rtn = pthread_mutex_lock( &(phins_frame_p->mutex) );
        if ( rtn ) { printf(">lockrtn %u  %s\n",rtn,strerror(rtn)); perror("lock mutex"); } 

        memcpy(phins_frame_p, &phins_frame_out, sizeof(PHINS_FRAME_t));

        rtn = pthread_mutex_unlock( &(phins_frame_p->mutex) );
        if ( rtn ) { printf(">unlockrtn %u %s\n",rtn,strerror(rtn)); perror("unlock mutex"); } 

        // Create a log entry running at >100 Hz. Remove when moving to GNC 
        if ( (count++ % 1000) == 0 )
        {
            printf(">> time %.f dpth %9.4f yaw %9.4f err_cnt:%u algo %04X\n", 
                phins_frame_p->phins_status.phins_data.INS_data_timetag * 10000, 
                phins_frame_p->phins_status.phins_data.INS_depth,
                phins_frame_p->phins_status.phins_data.INS_attitude[2],err_cnt, phins_frame_p->algo_stat[0]);
            fflush(stdout);    
        } 

    }

    close(sockfd);
    return 0;
}
