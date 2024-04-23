#ifndef EC_CONFIG_H_
#define EC_CONFIG_H_

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <pthread.h>
#include <sched.h> /* sched_setscheduler() */
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>

#include "ethercat/include/ecrt.h"

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING
#define CYCLIC_POSITION            8   /*Operation mode for 0x6060:0*/       /*csp模式*/      //位置模式
#define Number 3
#define MAX_PDO_ENTRIES            12+1 //最大支持的PDO参数映射

#define VID_PID 0x5a65726f, 0x00029252

/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC/ FREQUENCY)    /*本次设置周期PERIOD_NS为1ms*/

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_pdo_entry_reg_t domain1_regs[MAX_PDO_ENTRIES];

// process data
static uint8_t *domain1_pd = NULL;

/****************111111111111***********************/
static ec_slave_config_t *sc[Number] ;    //根据从站的个数定
static ec_slave_config_state_t sc_state[Number] ;

typedef struct{
    unsigned int ctrl_word;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int max_motor_speed;
    unsigned int target_velocity;
    unsigned int operation_mode;

    unsigned int error_code;
    unsigned int status_word;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int operation_mode_display;

}offset_t;
extern offset_t erob_offset[];

typedef void (*FnCyc)(void);

/*Config PDOs*****只需要在需要读取电机更多的状态的时候进行改写，所有从站共用一个*/
static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    /*RxPdo 0x1608*/
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6071, 0x00, 16}, /* Target Torque */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x6080, 0x00, 32}, /* Max Motor Speed */
    {0x60ff, 0x00, 32}, /* Target Velocity */
    {0x6060, 0x00, 8}, /* Modes of Operation */
    
    /*TxPdo 0x1A06*/
    {0x603f, 0x00, 16}, /* Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x606c, 0x00, 32}, /* Velocity Actual value */
    {0x6077, 0x00, 16}, /* Torque Actual Value */
    {0x6061, 0x00, 8}, /* Modes of Operation Display*/
    //{0x6078, 0x00, 16},  /*actual current*/
};

static ec_pdo_info_t slave_0_pdos[] = {                 
    {0x1608, 6, slave_0_pdo_entries + 0},
    {0x1a06, 6, slave_0_pdo_entries + 6},
};    //其中第二行的参数需要根据上面的txpdo与rxpdo的个数进行修改

static ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};  //不需要修改

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

struct timespec timespec_add(struct timespec time1, struct timespec time2);
void check_domain1_state(void);
void check_master_state(void);
void check_slave_config_states(void);
void *ec_thread_func(void *data = NULL);
int config_ec(void);

#endif