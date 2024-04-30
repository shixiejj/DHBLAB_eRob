#include "ec_config.h"

offset_t erob_offset[Number];
// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_pdo_entry_reg_t domain1_regs[14*Number+1];

// process data
uint8_t *domain1_pd = NULL;

/****************111111111111***********************/
static ec_slave_config_t *sc_arr[Number];    //根据从站的个数定
static ec_slave_config_state_t sc_state[Number];

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

/*Config PDOs*****只需要在需要读取电机更多的状态的时候进行改写，所有从站共用一个*/
ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    /*RxPdo 0x1608*/
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6071, 0x00, 16}, /* Target Torque */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x6080, 0x00, 32}, /* Max Motor Speed */
    {0x60ff, 0x00, 32}, /* Target Velocity */
    {0x6060, 0x00, 8}, /* Modes of Operation */
    {0x6061, 0x00, 8}, /* Modes of Operation Display*/
    
    /*TxPdo 0x1A06*/
    {0x603f, 0x00, 16}, /* Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x606c, 0x00, 32}, /* Velocity Actual value */
    {0x6077, 0x00, 16}, /* Torque Actual Value */
    {0x6061, 0x00, 8}, /* Modes of Operation Display*/
    {0x6060, 0x00, 8}, /* Modes of Operation */
    //{0x6078, 0x00, 16},  /*actual current*/
};

ec_pdo_info_t slave_0_pdos[] = {                 
    {0x1600, 7, slave_0_pdo_entries + 0},
    {0x1a00, 7, slave_0_pdo_entries + 7},
};    //其中第二行的参数需要根据上面的txpdo与rxpdo的个数进行修改

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};  //不需要修改


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

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

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

/******************************5555555555555555*******************************/

void check_slave_config_states(void)
{
    int i;
    ec_slave_config_state_t s[Number] = {}; // 根据从站的个数进行修改************************
    for (i = 0; i < Number; i++)
    {
        ecrt_slave_config_state(sc_arr[i], &s[i]);
        if (s[i].al_state != sc_state[i].al_state)
        {
            printf("slave: State 0x%02X.\n", s[i].al_state);
        }
        if (s[i].online != sc_state[i].online)
        {
            printf("slave: %s.\n", s[i].online ? "online" : "offline");
        }
        if (s[i].operational != sc_state[i].operational)
        {
            printf("slave: %soperational.\n", s[i].operational ? "" : "Not ");
        }
        sc_state[i] = s[i];
    }
}
/****************************************************************************/

//参数输入用户自定义循环函数，不做过多占时处理，尽量做简单的数据处理
void *ec_thread_func(void *data)
{
    FnCyc func_cyc = (FnCyc)data;
    struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while (1)
    {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL); // 此处休眠等待到达控制周期时间
        // Write application time to master
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;
        if (latency_ns > latency_max_ns)
        {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns)
        {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns)
        {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns)
        {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns)
        {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns)
        {
            exec_min_ns = exec_ns;
        }
#endif
        // receive process data
        ecrt_master_receive(master);  // 主站从设备获取数据帧并处理报文
        ecrt_domain_process(domain1); // 判断数据域报文的状态
        // check process data state (optional)//判断数据域、主站、从站状态是否发生变化，有变化就提示信息
        check_domain1_state(); // 数据域
        // 每一秒钟计算一下估计时间
        if (counter)
        {
            counter--;
        }
        else
        { // do this at 1 Hz
            counter = FREQUENCY;
            // check for master state (optional)
            check_master_state(); // 主站
            //  check for slave configuration state(s)
            check_slave_config_states(); // 从站
#ifdef MEASURE_TIMING
            // output timing stats
            printf("period     %10u ... %10u\n",
                   period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                   exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                   latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif
            // calculate new process data
            blink = !blink;
        }

        if (sync_ref_counter)
        {
            sync_ref_counter--;
        } 
        else 
        {
            sync_ref_counter = 1; // sync every cycle   每个循环周期用来同步
            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));

            //用户自定义循环函数
            if(func_cyc != NULL) {
                func_cyc();
            }


        }
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain1);//将数据域的所有报文插入到主站的报文序列
        ecrt_master_send(master);//将主站所有报文发送到传输序列
        #ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
        #endif

    }
}

int config_ec(void)
{
    master = ecrt_request_master(0); // 请求EtherCAT主机进行实时操作。
    if (!master)
        return -1;
    domain1 = ecrt_master_create_domain(master); // 创建新的进程数据域
    if (!domain1)
        return -1;

    for (int i = 0; i < Number; i++)
    {
        sc_arr[i] = ecrt_master_slave_config(master, 0, i, VID_PID);
        if (!sc_arr[i]) // 第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
            return -1;
        printf("sc_arr%d:%d\n",i,sc_arr[i]);
    }

    printf("Configuring PDOs...\n");
    for (int i = 0; i < Number; i++)
    {
        if (ecrt_slave_config_pdos(sc_arr[i], EC_END, slave_0_syncs))
        { // 指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
            fprintf(stderr, "Failed to configure slave PDOs!\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("*Success to configuring slave PDOs*\n");
        }
    }

    int p = 0;
    for (int i = 0; i < Number; i++)
    {
        printf("*PDO regs config*:%d\n",i);
        for (int j = 0; j < 14; j++)
        {
            domain1_regs[p].alias = 0;
            domain1_regs[p].position = i;
            domain1_regs[p].vendor_id = 0x5a65726f;
            domain1_regs[p].product_code = 0x00029252;
            domain1_regs[p].index = slave_0_pdo_entries[j].index;
            domain1_regs[p].subindex = slave_0_pdo_entries[j].subindex;
            domain1_regs[p].offset = (unsigned int *)&erob_offset[i] + j;
            domain1_regs[p].bit_position = 0;
            p++;
        }
    }
    domain1_regs[p].index = 0;
    printf("*finish PDO regs config!*\n");
    const ec_pdo_entry_reg_t *domain1_regs_const = domain1_regs;
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs_const))
    { // 为进程数据域注册一组PDO项。参数一：创建的进程数据域，参数二：pdo注册数组
        fprintf(stderr, "PDO entry registration failed!\n");
        exit(EXIT_FAILURE);
    }
    printf("*finish PDO regs entry!*\n");
    // configure SYNC signals for this slave
    for (int i = 0; i < Number; i++)
    {
        printf("*start congfig slave dc:%d*\n",i);
        ecrt_slave_config_dc(sc_arr[i], 0x0300, PERIOD_NS, 500000, 0, 0); // 此处sync0 shift time设置为周期的30%-40%
    }

    printf("finish congfig slave dc!\n");
    if (ecrt_master_activate(master)) // 以上激活主站
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1)))
    { // 返回域的进程数据
        return -1;
    }
    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = 98;

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }
    return 0;
}