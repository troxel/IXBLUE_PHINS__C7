/*--------------------------------------------------

Utility Functions related to writing and reading shared memory INS data.

Version History
----------------
Orignal: S Troxel Feb 2024
---------------------------------------------------*/

#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// shared mem stuff
#include <sys/mman.h>
#include <fcntl.h>

#include "shm_util.h"
#include "parameters.h"


// -------------------------------------
// Shared Memory related functions
// -------------------------------------

// File scope variables shared by the functions below. 
PHINS_FRAME_t *frame_ptr=NULL;
int shm_fd;

//-----------------------------------------------------
uint8_t read_ins_frame(PHINS_FRAME_t *frame_out_ptr) {

    static uint8_t init_flg = false;

    // ----- Init section
    if ( ! init_flg ) {

        // Using an init here so this entire function can be used without a seperate init section
        // to minimize GNC software effected. 

        // Set up the ins data structure and share it.
        shm_fd = shm_open(SHM_FILE, O_CREAT | O_RDWR, 0666);
        printf("Open shm file %s\n",SHM_FILE);
        if (shm_fd == -1) {
            perror("shm_open");
            return(false);
        }

        frame_ptr = mmap(0, sizeof(PHINS_FRAME_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (frame_ptr == MAP_FAILED) {
            perror("mmap");
            return(false);
        }

        init_flg = true; 
    }

    pthread_mutex_lock( &(frame_ptr->mutex) );
    memcpy(frame_out_ptr,frame_ptr,sizeof(PHINS_FRAME_t));
    pthread_mutex_unlock( &(frame_ptr->mutex) );

    return(true);
}

//-----------------------------------------------------
void close_shm() {

    munmap(frame_ptr, sizeof(PHINS_FRAME_t));
    close(shm_fd);
}    
