//DC-mode yaskawa control

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
#define FREQUENCY 500
#define CLOCK_TO_USE CLOCK_REALTIME
#define CONFIGURE_PDOS 1

// Optional features
#define PDO_SETTING1	1

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
#define yas  		0,0
#define yaskawa 	0x00000539, 0x02200001

/***************************** 20140224  ************************************/

//signal to turn off servo on state
static unsigned int servo_flag;
static unsigned int deactive;

// offsets for PDO entries
static unsigned int ctrl_word	;			
static unsigned int target_pos	;	
static unsigned int tar_velo	;			
static unsigned int tar_torq	;			
static unsigned int max_torq	;			
static unsigned int modeofoper	;			
static unsigned int interpolateddata ;

static unsigned int status_word	;			
static unsigned int actual_pos	;	
static unsigned int torq_actu_val;		
static unsigned int following_actu_val;	
static unsigned int modeofop_display;	
static unsigned int touch_probe_stat;	
static unsigned int touch_probe_val;	

static signed long temp[8]={};

//rx pdo entry
const static ec_pdo_entry_reg_t domain_r_regs[] = {
   	{yas,yaskawa,0x6040,00,&ctrl_word		},//rx
	{yas,yaskawa,0x607a,00,&target_pos		},
	{yas,yaskawa,0x60ff,00,&tar_velo		},
	{yas,yaskawa,0x6071,00,&tar_torq		},
	{yas,yaskawa,0x6072,00,&max_torq		},
	{yas,yaskawa,0x6060,00,&modeofoper		},
	{yas,yaskawa,0x60c1,01,&interpolateddata	},
	{}
};

//tx pdo entry
const static ec_pdo_entry_reg_t domain_w_regs[] = {
    	{yas,yaskawa,0x6041,00,	&status_word		},//tx
	{yas,yaskawa,0x6064,00,	&actual_pos		},
	{yas,yaskawa,0x6077,00,	&torq_actu_val		},
	{yas,yaskawa,0x60f4,00,	&following_actu_val	},
	{yas,yaskawa,0x6061,00,	&modeofop_display	},
	{yas,yaskawa,0x60b9,00,	&touch_probe_stat	},
	{yas,yaskawa,0x60ba,00,	&touch_probe_val	},
	{}
};

float move_value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

/*****************************************************************************/

#if PDO_SETTING1

//yaskawa PDO mapping
static ec_pdo_entry_info_t slave_0_pdo_entries[] = 
{
	{0x6040, 0x00, 16},//RXPDO
	{0x607a, 0x00, 32},
	{0x60ff, 0x00, 32},
	{0x6071, 0x00, 16},
	{0x6072, 0x00, 16},
	{0x6060, 0x00, 8},
	{0x0000, 0x00, 8},
	{0x60c1, 0x01, 32},
	{0x6041, 0x00, 16},//TXPDO
	{0x6064, 0x00, 32},
	{0x6077, 0x00, 16},
	{0x60f4, 0x00, 32},
	{0x6061, 0x00, 8},
	{0x0000, 0x00, 32},
	{0x60b9, 0x00, 16},
	{0x60ba, 0x00, 32},
};//{index,subindex,lenth}

static ec_pdo_info_t slave_0_pdos[] = 
{
	{0x1600, 8, slave_0_pdo_entries + 0},
	{0x1a00, 8, slave_0_pdo_entries + 8},
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


/*****************************************************************************/


/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
	struct timespec result;

	if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) 
	{
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	} 

	else 
	{
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	}

	return result;
}

/*****************************************************************************/

void endsignal(int sig)
{
	
	servo_flag = 1;
	signal( SIGINT , SIG_DFL );
}

/*****************************************************************************/


void check_domain_r_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain_r, &ds);

	//struct timespec time_wc1,time_wc2;
	if (ds.working_counter != domain_r_state.working_counter)
		printf("domain_r: WC %u.\n", ds.working_counter);
	if (ds.wc_state != domain_r_state.wc_state)
        	printf("domain_r: State %u.\n", ds.wc_state);

    domain_r_state = ds;
}

void check_domain_w_state(void)
{
    ec_domain_state_t ds2;
    ecrt_domain_state(domain_w, &ds2);

	if (ds2.working_counter != domain_w_state.working_counter)
		printf("domain_w: WC %u.\n", ds2.working_counter);
	if (ds2.wc_state != domain_w_state.wc_state)
       	 	printf("domain_w: State %u.\n", ds2.wc_state);
	
    domain_w_state = ds2;
}



void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

	while(1) 
	{

		if(deactive==1)
		{
			break;
		}

		wakeupTime = timespec_add(wakeupTime, cycletime);
     		clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

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
		{
			//servo off
			EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0006 );
		}	

		else if( (temp[0]&0x004f) == 0x0040  )
		{
			EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0006 );
		}

		else if( (temp[0]&0x006f) == 0x0021)
		{
			EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0007 );
		}

		else if( (temp[0]&0x027f) == 0x0233)
		{
			EC_WRITE_U16(domain_r_pd+ctrl_word, 0x000f);
			EC_WRITE_S32(domain_r_pd+interpolateddata, 0);
			EC_WRITE_S32(domain_r_pd+max_torq, 0xf00);
			EC_WRITE_S32(domain_r_pd+modeofoper, 7);
		}
		
		//operation enabled
		else if( (temp[0]&0x027f) == 0x0237)
		{
			//600 800
			EC_WRITE_S32(domain_r_pd+interpolateddata,( move_value+=1000 ));
			EC_WRITE_U16(domain_r_pd+ctrl_word, 0x001f);

		}
		// write application time to master
		clock_gettime(CLOCK_TO_USE, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));

		
		if (sync_ref_counter) 
		{
			sync_ref_counter--;
		} 

		else 
		{
			sync_ref_counter = 1; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}

		ecrt_master_sync_slave_clocks(master);

		
		
		// send process data
		ecrt_domain_queue(domain_r);
		ecrt_domain_queue(domain_w);
		
		ecrt_master_send(master);

		

	}

}

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
	
	
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) 
    {
		perror("mlockall failed");
		return -1;
    }

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
    
#if CONFIGURE_PDOS

    printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }
	

#endif

	
    if (ecrt_domain_reg_pdo_entry_list(domain_r, domain_r_regs)) 
    {
        fprintf(stderr, "1st motor RX_PDO entry registration failed!\n");
        return -1;
    }	
	
    if (ecrt_domain_reg_pdo_entry_list(domain_w, domain_w_regs)) 
    {
        fprintf(stderr, "1st motor TX_PDO entry registration failed!\n");
        return -1;
    }
		
	
    ecrt_slave_config_dc(sc,0x0300,4000000,125000,0,0);  

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
    if (setpriority(PRIO_PROCESS, pid, -20))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));

	signal( SIGINT , endsignal ); //interrupt program		
	printf("Starting cyclic function.\n");
    	cyclic_task();
	ecrt_release_master(master);
	
    return 0;
}

