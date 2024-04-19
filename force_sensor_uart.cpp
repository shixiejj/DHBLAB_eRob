#include "force_sensor_uart.h"

//接收回调函数
// void __attribute__((weak)) RxData_CallBack(std::vector<unsigned char> &data, int fd) {
// 	for (auto c : data)
// 		printf("%c", c);
// }

void RxData_CallBack(std::vector<unsigned char> &data, int fd, uint8_t *return_data) {
    if(data[0] == 0xAA && data[1] == 0x55) {
        uint32_t sensor_data[6] = {0};
        uint8_t sensor_sta[6] = {0};
        for(int i = 0; i < 6; i++) {
            sensor_data[i] = (uint32_t)(data[4*i] | data[4*i+1] << 8 | data[4*i+2] << 16 | data[4*i+3] << 24);
        }
        for(int i = 0; i < 6; i++) {
            sensor_sta[i] = data[i];
        }
        memcpy(return_data, sensor_data, 24);
        memcpy(return_data + 24, sensor_sta, 6);
    }else {
        printf("uart recieve error!");
    }
}

//监听线程　读取的数据存放在容器
void* Listen(void *arg) {
	int get;
	SerialPort::Data_t *data = static_cast<SerialPort::Data_t *>(arg);
	int fd = *((int*) data->fd);
	std::vector<unsigned char> RX_buf(128);
	while (1) {
		get = read(fd, &RX_buf[0], 128);
		if (get > 0) {
			printf("recieve!");
			std::vector<unsigned char> RX_data;
			for (int c = 0; c < get; ++c)
				RX_data.push_back(RX_buf[c]);
			RxData_CallBack(RX_data, fd, (uint8_t*)data->fsd);
			RX_data.clear();
		}else {
			//printf("no data!");
		}
	}
	return NULL;
}

//发送单个字节
bool SerialPort::Send(unsigned char byte) {
	return (write(this->fd, &byte, 1) == -1) ? false : true;
}
bool operator <<(SerialPort port, unsigned char byte) {
	return (write(port.fd, &byte, 1) == -1) ? false : true;
}
//发送多个字节
bool SerialPort::Send(std::vector<unsigned char> data) {
	return (write(this->fd, &data[0], data.size()) == -1) ? false : true;
}
bool operator <<(SerialPort port, std::vector<unsigned char> data) {
	return (write(port.fd, &data[0], data.size()) == -1) ? false : true;
}
//发送多个字节
bool SerialPort::Send(char *data, unsigned int len) {
	return (write(this->fd, data, len) == -1) ? false : true;
}
bool operator <<(SerialPort port, char const *data) {
	return (write(port.fd, data, strlen(data)) == -1) ? false : true;
}

//打开串口
bool SerialPort::Open() {
	//打开串口
	this->fd = open(this->Config.DevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (this->fd == -1)
		return false;
	if (fcntl(this->fd, F_SETFL, 0) < 0)
		return false;
	if (isatty(STDIN_FILENO) == 0)
		return false;

	this->Tran_Data.fd = &this->fd;
	this->Tran_Data.fsd = &this->Force_Sensor;

	//清空缓存
	tcflush(fd, TCIOFLUSH);
	fcntl(fd, F_SETFL, 0);

	//开启监听线程
	pthread_create(&this->listen_thread, NULL, &Listen, &this->Tran_Data);
	pthread_detach(this->listen_thread);

	return true;
}

//关闭串口
void SerialPort::Close() {
	//关闭串口入口
	close(this->fd);
	//关闭监听线程
	pthread_cancel(this->listen_thread);
}

//配置串口
bool SerialPort::LoadConfig() {
	//设置参数
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0)
		return false;
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	//DataBits
	switch (this->Config.DataBits) {
	case this->DataBits7:
		newtio.c_cflag |= CS7;
		break;
	case this->DataBits8:
		newtio.c_cflag |= CS8;
		break;
	}

	//Parity
	switch (this->Config.Parity) {
	case this->ParityEven:
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case this->ParityNone:
		newtio.c_cflag &= ~PARENB;
		break;
	case this->ParityOdd:
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	}

	//BaudRate
	cfsetispeed(&newtio, this->Config.BaudRate);
	cfsetospeed(&newtio, this->Config.BaudRate);

	//StopBits
	switch (this->Config.StopBits) {
	case this->StopBits1:
		newtio.c_cflag &= ~CSTOPB;
		break;
	case this->StopBits2:
		newtio.c_cflag |= CSTOPB;
		break;
	}

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd, TCIOFLUSH);
	fcntl(fd, F_SETFL, 0);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
		return false;

	return true;
}