/*-----------------------------------------------------------------------------
can_lib.h -- Interface to CAN library

Copyright (c) 1994 JANZ Computer AG
All Rights Reserved


Created 96/01/23 by Stefan Althoefer
Version 1.8 of 96/10/31
-----------------------------------------------------------------------------*/


#ifndef can_lib_DEFINED
#define can_lib_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "semaphore.h"

#include "defs.h"
#include "mican.h"
#include "dpm.h"
#include "vmod.h"
#include "pthread.h"


/*------- Structure Definitions ---------------------------------------------*/

/* Defaults used by can_device_setup() */
#define DEFAULT_STD_IF_WBUFFERS     20
#define DEFAULT_STD_IF_RBUFFERS     40

/* This is a revision number for the dpm_dev_t structure. This
   should be incremented when the structure changes in a way
   that would result in binary incompatibility between old
   libraries (DLLs) and new applications (or vice versa).
   However, to avoid such situations, we included some expansion
   space in the structure. */
#define DPM_DEV_T_REV   1


/*
 * NOTE:
 * NEVER EVER CHANGE THE ORDER OF ELEMENTS IN THIS STRUCTURE.
 * THIS WOULD BREAK BINARY COMPATIBILTY BETWEEN LIBRARY AND
 * APPLICATION. ALLOCATE IN THE RESERVED ARRAY INSTEAD.
 */
typedef struct {
    /*
     * Input area (things to be provided by user)
     */
    int     dpm_dev_t_rev;
    char    *devname;

    int     std_if_rbuffers;
    int     std_if_wbuffers;
    int     std_if_wbuffers_high;
    int     std_if_wbuffers_low;

    int     fast_if_rbuffers;
    int     fast_if_wbuffers;
    int     fast_if_wqueue_cnt;

    int     dpm_size;
    int     start_of_pp;

    char    *fw_check_string;

    void    (*std_if_handler)(void *, Message *);
    void    *std_if_context;
    Message *std_if_rxbuff_p;
    void    (*fast_if_handler)(void *, FastMessage *);
    void    *fast_if_context;
    FastMessage *fast_if_rxbuff_p;

    int     in_rsvd[20];   /* For expansion */

    /*
     * Output area (things to be used by user)
     */
    int     fd;
    int     fw_version;

    int     out_rsvd[20];   /* For expansion */

    /*
     * User defined storage
     */
    void    *user[4];

    /*
     * Internal storage (QNX)
     */
    pthread_t threadId;

    int     internal_rsvd[20];   /* For expansion */
} dpm_dev_t;


/*------ Declarations ---------------------------------------------*/

/* Open the device and return a descriptor. Only one process may
   hold the the device open at a time. */
extern int can_open _PARAMS((char  *));
extern int can_alt_open _PARAMS((char *));

/* Close a device */
extern void can_close _PARAMS((int fd));

/* Reset a device */
extern void can_reset _PARAMS((int fd));

/* Declare a signal (for OS-9 systems only) */
extern int can_sigdcl _PARAMS((int fd, int pid, int sig));

/* Undeclare a signal (for OS-9 systems only) */
extern void can_sigdel _PARAMS((int fd));

/* Declare a semaphore (for VxWorks systems only) */
extern int can_semdcl _PARAMS((int fd, int idx, sem_t *sem));

/* Delete a semaphore (for VxWorks systems only) */
extern void can_semdel _PARAMS((int fd));

/* Receive a message from a module */
extern int can_recv _PARAMS((int fd, Message *pm));

/* Receive a fast message from a module */
extern int can_recv_fast _PARAMS((int fd, FastMessage *pm));

/* Send a middle prior message to a module */
extern int can_send _PARAMS((int fd, Message *pm));

/* Send a high prior message to a module */
extern int can_send_hi  _PARAMS((int fd, Message *pm));

/* Send a low_prior message to a module */
extern int can_send_low _PARAMS((int fd, Message *pm));

/* Send a fast message to a module */
extern int can_fast_send _PARAMS((int fd, FastMessage *pm));

/* Send a fast message to a module */
extern int can_fast_send_prio _PARAMS((int fd, int prioQueue, FastMessage *pm));

/* One communication step (MS-DOS systems only) */
extern void can_comm _PARAMS((int fd));

/* Switch to new stylish hostinterface */
extern void ican2_select_hostif _PARAMS((int fd, int rbuffers, int wbuffers));

/* Switch to new stylish hostinterface with dedicated priorized-messsage- */
/* buffer-size */
extern void ican2_select_hostif_prio _PARAMS((int fd, int rbuffers, 
	int wbuffers, int wbuffers_hi, int wbuffers_low));

/* Init fast CANbus access */
extern int ican2_init_fast_can _PARAMS((int fd, int rbuffers, int wbuffers));

/* Init fast CANbus access */
extern int ican2_init_fast_can_prio _PARAMS((int fd, int rbuffers, int wbuffers,
                                         int numOfPrioQueues));

/* Copy a block of data to the DPM */
extern void ican2_todpmcpy _PARAMS((int fd, unsigned int d,
                                        unsigned char *s, int n));

/* Copy a block of data from the DPM */
extern void ican2_fromdpmcpy _PARAMS((int fd, unsigned char *d,
                                        unsigned int s, int n));

/* Initialize table; holds Id-specific message-queue-ids, where */
/* fast messages with the corresponding ID should be routed to  are stored */
extern int init_id_msg_q_table(int fd);

/* Deinitialize above described table */
extern int deinit_id_msg_q_table(int fd);

/* Enter one message-queue-id into the above described table. */
/* this entry causes a message-sending not to the default, common queue */
/* but to a user defined queue */
int add_msg_q_to_id_table(
#ifdef VXWORKS
        int             fd,
        unsigned short  id,
        MSG_Q_ID        target_queue
#endif
#ifdef LINUX
        int             fd,
        unsigned short  id
#endif
); 

/* removes one entry like above described */
int rem_msg_q_from_id_table(
        int             fd,
        unsigned short  id
);

/* Initialize table; holds device and ID where messages from this device */
/* and with a certain CAN-ID should be routed to. */
extern int can_init_layer_2_routing _PARAMS((int fd));

/* Deinitialize above described table */
extern int can_deinit_layer_2_routing _PARAMS((int fd));

/* */
extern int can_init_route_id _PARAMS((
        int             fd,
        int             Id,
        int             max_routes
));

/* */
extern int can_add_route_to_id _PARAMS((
        int             fd,
        int             source_Id,
        int             dest_fd,
        int             dest_Id
));

/* */
extern unsigned int can_drv_idvers _PARAMS((
        int             fd
));

/* initiate a tpu request */
extern void ican2_tpurequest _PARAMS((int fd));

extern void ican_objdic _PARAMS((
                int fd,
                int objdic_index,
                int objdic_subindex,
                int access_type,
                void *entry_structure
));

extern void ican_init_safe_pp _PARAMS((
	int     fd
));

extern int ican_read_pp _PARAMS((
        int     fd,
        int     dataSize,
        int     offset,
        char    *buffer
));
 
extern void ican_write_pp _PARAMS((
        int     fd,
        int     dataSize,
        int     offset,
        char     *buffer
));

void init_dpm_dev_struct _PARAMS((
        dpm_dev_t   *p,
        int         libRev
));

int ican_check_dpm_resources _PARAMS((
        dpm_dev_t   *res,
        char        *buf,
        int         n
));

int can_device_destroy _PARAMS((
        dpm_dev_t    *p
));

int can_device_init_struct _PARAMS((
        dpm_dev_t    *p
));

int can_device_setup _PARAMS((
        dpm_dev_t    *p,
        char         *buf,
        int          n
));

unsigned int can_device_idvers _PARAMS((
        int          fd
));

#ifdef __cplusplus
}
#endif

#endif /* !can_lib_DEFINED */
