//DC-mode panasonic control

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include "ecrt.h"



// Application parameters
#define FREQUENCY 4000
#define CLOCK_TO_USE CLOCK_REALTIME
#define CONFIGURE_PDOS 1

// Optional features
#define PDO_SETTING	1
#define SDO_ACCESS      1


/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain_r = NULL;
static ec_domain_state_t domain_r_state = {};
static ec_domain_t *domain_w = NULL;
static ec_domain_state_t domain_w_state = {};

static ec_slave_config_t *sc  = NULL;
static ec_slave_config_state_t sc_state = {};
/****************************************************************************/

// process data
static uint8_t *domain_r_pd = NULL;
static uint8_t *domain_w_pd = NULL;
#define pana  		0,0
#define panasonic 	0x0000066f, 0x525100d1

/***************************************************************************/

//signal to turn off servo on state
static unsigned int servo_flag =0;
static unsigned int deactive;
// offsets for PDO entries
static unsigned int ctrl_word   ;
static unsigned int mode  ;
static unsigned int tar_torq    ;
static unsigned int max_torq    ;
static unsigned int tar_pos    ;
static unsigned int max_speed  ;
static unsigned int touch_probe_func ;
static unsigned int tar_vel ;
static unsigned int error_code  ;
static unsigned int status_word;
static unsigned int mode_display ;
static unsigned int pos_act;
static unsigned int vel_act;
static unsigned int torq_act;
static unsigned int touch_probe_status;
static unsigned int touch_probe_pos;
static unsigned int digital_input;

static signed long temp[8]={};

//rx pdo entry 
const static ec_pdo_entry_reg_t domain_r_regs[] = 
{
        {pana,panasonic,0x6040,00,&ctrl_word              },
        {pana,panasonic,0x6060,00,&mode                   },
        {pana,panasonic,0x6071,00,&tar_torq               },
        {pana,panasonic,0x6072,00,&max_torq               },
        {pana,panasonic,0x607a,00,&tar_pos                },
        {pana,panasonic,0x6080,00,&max_speed              },
        {pana,panasonic,0x60b8,00,&touch_probe_func       },
        {pana,panasonic,0x60ff,00,&tar_vel                },
        {}
};

//tx pdo entry
const static ec_pdo_entry_reg_t domain_w_regs[] = 
{
        {pana,panasonic,0x603f,00,&error_code             },
        {pana,panasonic,0x6041,00,&status_word            },
        {pana,panasonic,0x6061,00,&mode_display           },
        {pana,panasonic,0x6064,00,&pos_act                },
        {pana,panasonic,0x606c,00,&vel_act                },
        {pana,panasonic,0x6077,00,&torq_act               },
        {pana,panasonic,0x60b9,00,&touch_probe_status     },
        {pana,panasonic,0x60ba,00,&touch_probe_pos        },
        {pana,panasonic,0x60fd,00,&digital_input          },
        {}
};




float move_value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};


/*****************************************************************************/

#if PDO_SETTING

static ec_pdo_entry_info_t slave_0_pdo_entries[] = 
{
        {0x6040, 0x00, 16},
        {0x6060, 0x00, 8 },
        {0x6071, 0x00, 16},
        {0x6072, 0x00, 16},
        {0x607a, 0x00, 32},
        {0x6080, 0x00, 32},
        {0x60b8, 0x00, 16},
        {0x60ff, 0x00, 32},
        {0x603f, 0x00, 16},
        {0x6041, 0x00, 16},
        {0x6061, 0x00, 8 },
        {0x6064, 0x00, 32},
        {0x606c, 0x00, 32},
        {0x6077, 0x00, 16},
        {0x60b9, 0x00, 16},
        {0x60ba, 0x00, 32},
        {0x60fd, 0x00, 32},
};//{index,subindex,lenth}

static ec_pdo_info_t slave_0_pdos[] = 
{
	{0x1600, 8, slave_0_pdo_entries + 0},
	{0x1a00, 9, slave_0_pdo_entries + 8},
};


static ec_sync_info_t slave_0_syncs[] = 
{
	{0, EC_DIR_OUTPUT, 0, NULL,EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
        {0xff}
};

#endif
/***********************************************************************/
