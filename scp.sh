#!/bin/bash
# g++ -o openrc *.cpp *.h project_base/* slave/* -lrt -m32 -lpthread -std=c++11

scp openrc inexbot@192.168.3.3:~/robot 
scp camera_trajectory.txt inexbot@192.168.3.3:~/robot