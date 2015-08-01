//Yaskawa servo motor

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

//yaswawa vendor_id & product_code
#define yas             0
#define yaskawa         0x02200001

static ec_master_t *master = NULL;
static ec_domain_t *domain_r = NULL;
static ec_domain_t *domain_w = NULL;

static ec_slave_config_t *sc  = NULL;


// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

// process data
static uint8_t *domain_r_pd = NULL;
static uint8_t *domain_w_pd = NULL;

//
// offsets for PDO entries
static unsigned int ctrl_word   ;
static unsigned int target_pos  ;
static unsigned int tar_velo    ;
static unsigned int tar_torq    ;
static unsigned int max_torq    ;
static unsigned int modeofoper  ;
static unsigned int interpolateddata ;

static unsigned int status_word ;
static unsigned int actual_pos  ;
static unsigned int torq_actu_val;
static unsigned int following_actu_val;
static unsigned int modeofop_display;
static unsigned int touch_probe_stat;
static unsigned int touch_probe_val;

static signed long temp[8]={};

//rx pdo entry of  motor
const static ec_pdo_entry_reg_t domain_r_regs[] = 
{
        {yas,yaskawa,0x6040,00,&ctrl_word              },
        {yas,yaskawa,0x607a,00,&target_pos             },
        {yas,yaskawa,0x60ff,00,&tar_velo               },
        {yas,yaskawa,0x6071,00,&tar_torq               },
        {yas,yaskawa,0x6072,00,&max_torq               },
        {yas,yaskawa,0x6060,00,&modeofoper             },
        {yas,yaskawa,0x60c1,01,&interpolateddata       },
	{}
};

const static ec_pdo_entry_reg_t domain_w__regs[] = 
{
        {yas,yaskawa,0x6041,00,&status_word            },
        {yas,yaskawa,0x6064,00,&actual_pos             },
        {yas,yaskawa,0x6077,00,&torq_actu_val          },
        {yas,yaskawa,0x60f4,00,&following_actu_val     },
        {yas,yaskawa,0x6061,00,&modeofop_display       },
        {yas,yaskawa,0x60b9,00,&touch_probe_stat       },
        {yas,yaskawa,0x60ba,00,&touch_probe_val        },
 	{}
};

//signal to turn off servo on state
static unsigned int servo_flag;

float move_value = 0;

static unsigned int counter = 0;
static unsigned int blink = 0;


void cyclic_task()
{

}

//
void signal_handler(int signum) {
    switch (signum) {
        case SIGALRM:
            sig_alarms++;
	    break;
    }
}

int main(int argc, char **argv)
{

}
