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
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain_r);
    ecrt_domain_process(domain_w);

    temp[0]=EC_READ_U16(domain_w_pd + status_word);
    temp[1]=EC_READ_U32(domain_w_pd + actual_pos);
    
    if (counter) 
    {
    	counter--;
    } 
    else 
    { 
    // do this at 1 Hz
    counter = FREQUENCY;
    blink = !blink;
    }

    // write process data
    if(servo_flag==1)
    {	//servo off
    	EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0006 );
        //deactive++;
    }
    else if( (temp[0]&0x004f) == 0x0040  )
    {
        EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0006 );
        printf("%x\n",temp[0]);
    }
    else if( (temp[0]&0x006f) == 0x0021)
    {
        EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0007 );
        printf("%x\n",temp[0]);
    }
    else if( (temp[0]&0x006f) == 0x0023)
    {
	EC_WRITE_U16(domain_r_pd+ctrl_word, 0x000f);
	EC_WRITE_S32(domain_r_pd+target_pos, 0);
        EC_WRITE_S32(domain_r_pd+tar_velo, 0xfffff);
        EC_WRITE_S32(domain_r_pd+max_torq, 0xf00);
        EC_WRITE_S32(domain_r_pd+modeofoper, 8);
        printf("%x\n",temp[0]);
     }
    //operation enabled 
    else if ((temp[0]&0x006f) == 0x0027)
    {            	
        EC_WRITE_S32(domain_r_pd+target_pos, (move_value+=2000) );
        EC_WRITE_U16(domain_r_pd+ctrl_word, 0x001f);
        printf("%x\n",temp[0]);
    }

    // send process data
    ecrt_domain_queue(domain_r);
    ecrt_domain_queue(domain_w);
    ecrt_master_send(master);
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
    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain_r = ecrt_master_create_domain(master);
    if (!domain_r)
        return -1;
    domain_w = ecrt_master_create_domain(master);
    if (!domain_w)
	return -1;

    if (!(sc = ecrt_master_slave_config(master, yas, yaskawa))) 
    {
        fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }


    if (ecrt_domain_reg_pdo_entry_list(domain_r, domain_r_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain_w, domain_w_regs)) 
    {
	fprintf(stderr, "PDO entry registration failed!\n");
	return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain_r_pd = ecrt_domain_data(domain_r))) 
    {
        return -1;
    }

    if (!(domain_w_pd = ecrt_domain_data(domain_w))) 
    {
        return -1;
    }

    pid_t pid = getpid();

    if (setpriority(PRIO_PROCESS, pid, -19))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",strerror(errno));

    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0)) 
    {
        fprintf(stderr, "Failed to install signal handler!\n");
        return -1;
    }

    printf("Started.\n");
    
    while (1) 
    {
        pause();

        while (sig_alarms != user_alarms) 
        {
            cyclic_task();
            user_alarms++;
        }
    }

    return 0;
}
