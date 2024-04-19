#!/bin/bash
# g++ -o openrc *.cpp *.h project_base/* slave/* -lrt -m32 -lpthread -std=c++11
#g++ -o openrc main.cpp general_6s.cpp tcpsocket.cpp UR.cpp *.h project_base/* slave/* -lrt -m32 -lpthread -std=c++11
g++ -o openrc main.cpp general_6s.cpp tcpsocket.cpp UR.cpp *.h  force_sensor_uart.cpp force_sensor.cpp project_base/* slave/* -lrt -lpthread -m32 -std=c++11 -lmodbus

