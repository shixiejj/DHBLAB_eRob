#include<stdio.h>
#include<stdlib.h>
#include <iostream>
#include <modbus/modbus.h>


#define MODBUS_DEV_NAME    "/dev/ttyS5"
#define MODBUS_BAUD 115200

//处理获得力传感器数据，
class T521G2
{
private:
    modbus_t* _mb;

public:
    T521G2();
    ~T521G2();

    int32_t get_val_sensor();
    int32_t get_status_sensor();
    void write_val_sensor(int addr, int value);
    int start();
};