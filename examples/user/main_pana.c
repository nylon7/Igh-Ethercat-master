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
	// receive process data
    	ecrt_master_receive(master);
   	ecrt_domain_process(domain);
        
	temp[0]=EC_READ_U16(domain_pd + status_word);
        temp[1]=EC_READ_S32(domain_pd + mode_display);
                
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
	
	printf("after value =%x\n",temp[0]);	
	
	// write process data

        if(servo_flog==1)
        {
         	//servo off
         	EC_WRITE_U16(domain_pd+ctrl_word, 0x0006 );
        }

        else if( (temp[0]&0x004f) == 0x0040  )
        {
                EC_WRITE_U16(domain_pd+ctrl_word, 0x0006 );
        }

        else if( (temp[0]&0x006f) == 0x0021)
        {
                EC_WRITE_U16(domain_pd+ctrl_word, 0x0007 );
	}

        else if( (temp[0]&0x006f) == 0x0023)
        {
             	EC_WRITE_U16(domain_pd+ctrl_word, 0x000f );
		EC_WRITE_S32(domain_pd+tar_pos,0);
		EC_WRITE_S32(domain_pd+tar_vel, 0xffff);
                EC_WRITE_S32(domain_pd+max_torq, 0xf00);
        }
	//operation enabled

        else if( (temp[0]&0x006f) == 0x0027)
	{
                EC_WRITE_S32(domain_pd+tar_pos, (value+=1) );
                EC_WRITE_U16(domain_pd+ctrl_word, 0x001f);
        }

	// send process data

    	ecrt_domain_queue(domain);
    	ecrt_master_send(master);
	
}

/****************************************************************************/

void signal_handler(int signum) 
{
    switch (signum) 
    {
        case SIGALRM:
            sig_alarms++;
	    break;
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{
    struct sigaction sa;

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain = ecrt_master_create_domain(master);
    if (!domain)
        return -1;


    if (!(sc = ecrt_master_slave_config(master, pana, panasonic))) 
    {
        fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }

//SDO_ACCESS
    if (ecrt_slave_config_sdo8(sc, 0x6060, 0, 8))
    {
	return -1;
    }  



//CONFIGURE_PDOS
    printf("Configuring PDOs...\n");

    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }




    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain_pd = ecrt_domain_data(domain))) 
    {
        return -1;
    }


    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",trerror(errno));

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
