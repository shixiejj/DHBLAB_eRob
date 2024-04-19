/*
 * tcpsocket.h
 *
 *  Created on: Oct 20, 2021
 *      Author: inexbot
 */

#ifndef OPENRC_TCPSOCKET_H_
#define OPENRC_TCPSOCKET_H_

#include <pthread.h>
#include <stdio.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>
#include <iostream>
#include <vector>
#include<fstream>
#include "robot_type.h"
class tcpsocket
{

public:
	void initSocket();
	void createPthread();
	int listenfd,connfd;
	struct sockaddr_in servaddr;
	char buff[4096];
	int n;
	void createTextFile(std::string jobFileName);
	void storeFile(std::string filepath, std::string  fileText);
	void doAction(std::string jobFileName);
	void moveAxis(int , int );
	void deleteFile(std::string filepath);
	void runFile(std::string filepath);
	void runCMD(const std::string &temp);
	void runAxistoPOS_MOVEJ( VectorXd );
	void runAxistoPOS_MOVEL( VectorXd );
	std::string currentFile =" ";

};



#endif /* OPENRC_TCPSOCKET_H_ */
