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
#define yas             1,0
#define yaskawa         0x00000539, 0x02200001

static ec_master_t *master = NULL;
static ec_domain_t *domain_w = NULL;
static ec_domain_t *domain_r = NULL;

static ec_slave_config_t *sc  = NULL;


// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

// process data
static uint8_t *domain_w_pd = NULL;
static uint8_t *domain_r_pd = NULL;

//signal to turn off servo on state
static unsigned int servo_flag;

float move_value = 0;

static unsigned int counter = 0;
static unsigned int blink = 0;
