//Panasonic servo motor

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "ecrt.h"

// Application parameters
#define FREQUENCY 1000

static ec_master_t *master = NULL;
static ec_domain_t *domain = NULL;
static ec_slave_config_t *sc  = NULL;


// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;



// process data
static uint8_t *domain_pd = NULL;
#define pana             0
#define panasonic 	 0x0000066f

//signal to turn off servo on state
static unsigned int servo_flog =0;

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

float move_value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;

//pdo entry
const static ec_pdo_entry_reg_t domain1_regs[] = 
{
	{pana,panasonic,0x6040,00,	&ctrl_word		},
	{pana,panasonic,0x6060,00,	&mode			},
	{pana,panasonic,0x6071,00,	&tar_torq		},
	{pana,panasonic,0x6072,00,	&max_torq		},
	{pana,panasonic,0x607a,00,	&tar_pos		},
	{pana,panasonic,0x6080,00,	&max_speed		},
	{pana,panasonic,0x60b8,00,	&touch_probe_func	},
	{pana,panasonic,0x60ff,00,	&tar_vel		},
	{pana,panasonic,0x603f,00,	&error_code		},
	{pana,panasonic,0x6041,00,	&status_word		},
	{pana,panasonic,0x6061,00,	&mode_display		},
	{pana,panasonic,0x6064,00,	&pos_act		},
	{pana,panasonic,0x606c,00,	&vel_act		},
	{pana,panasonic,0x6077,00,	&torq_act		},
	{pana,panasonic,0x60b9,00, &touch_probe_status	},
	{pana,panasonic,0x60ba,00,	&touch_probe_pos	},
	{pana,panasonic,0x60fd,00,	&digital_input		},
        {}
};


//pdo setting
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

tatic ec_pdo_info_t slave_0_pdos[] = 
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



void cyclic_task()
{

}

/****************************************************************************/

void signal_handler(int signum) {
    switch (signum) {
        case SIGALRM:
            sig_alarms++;
	    break;
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{

}