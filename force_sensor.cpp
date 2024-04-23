#include "force_sensor.h"
#include <modbus/modbus.h>

T521G2::T521G2()
{
}

int T521G2::start()
{
    struct timeval response_timeout;
    this->_mb = modbus_new_rtu(MODBUS_DEV_NAME, MODBUS_BAUD,'N',8,1);//open port
    modbus_rtu_set_serial_mode(this->_mb, MODBUS_RTU_RS485);
    modbus_set_debug(this->_mb, 1);
    modbus_set_slave(this->_mb,1);//set slave address
	if (this->_mb == NULL)               
	{
    	std::cout<<"Unable to allocate libmodbus contex\n"<<std::endl;
    	return -1;
	}

/* Save original timeout */
    // modbus_get_response_timeout(_mb, &old_response_to_sec, &old_response_to_usec);
    // std::cout<<"old time out : "<<old_response_to_sec<<"and"<<old_response_to_usec <<std::endl;
    //std::cout<<"change response time\n"<<std::endl;
    //modbus_set_response_timeout(this->_mb, 0,800000);
    std::cout<<"start connection\n"<<std::endl;
    int rc = modbus_connect(this->_mb);
    if (rc == -1) //等待连接设备
    {
        std::cout<<"Connection failed:%s\n"<<std::endl;
        modbus_free(this->_mb);
        return -1;
    }else if(rc == 0){
        return 0;
    }else{
        return 1;
    }
}

T521G2::~T521G2()
{
    // 关闭 Modbus 连接
    modbus_close(this->_mb);
    modbus_free(this->_mb);
}

int32_t T521G2::get_val_sensor()
{
    int32_t val;
    uint16_t readBuffer[8] = {0};  // 用于存储读取的数据
    int rc = modbus_read_registers(_mb, 0, 2, readBuffer);
    if (rc == -1) 
    {
        std::cerr << "Failed to read Modbus registers: " << modbus_strerror(errno) << std::endl;
        return -1;
    } 
    else
    {
        val = readBuffer[0] << 16 | readBuffer[1];
        return val;
    }
}

int32_t T521G2::get_status_sensor()
{
    return 0;
}

void T521G2::write_val_sensor(int addr, int value)
{
    int rc = modbus_write_bit(this->_mb, addr, value);
    if (rc == -1) {
        std::cerr << "Failed to write Modbus coil: " << modbus_strerror(errno) << std::endl;
    } else {
        std::cout << "Write successful" << std::endl;
    }
}
