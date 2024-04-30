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
#define Number 2
#define MAX_PDO_ENTRIES            12+1 //最大支持的PDO参数映射

#define VID_PID 0x5a65726f, 0x00029252

/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC/ FREQUENCY)    /*本次设置周期PERIOD_NS为1ms*/

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/
extern uint8_t *domain1_pd;

typedef struct{
    unsigned int ctrl_word;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int max_motor_speed;
    unsigned int target_velocity;
    unsigned int operation_mode;
    unsigned int operation_mo_temp;

    unsigned int error_code;
    unsigned int status_word;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int operation_mode_display;
    unsigned int operation_mode_temp;

}offset_t;
extern offset_t erob_offset[];

typedef void (*FnCyc)(void);

/*Config PDOs*****只需要在需要读取电机更多的状态的时候进行改写，所有从站共用一个*/

struct timespec timespec_add(struct timespec time1, struct timespec time2);
void check_domain1_state(void);
void check_master_state(void);
void check_slave_config_states(void);
void *ec_thread_func(void *data = NULL);
int config_ec(void);

#endif