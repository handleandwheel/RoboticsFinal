#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "ftp/FtpControl.h"
#include "eigen3/Eigen/Core"
#include "MotionPlan.h"
#include "RobotCommunication.h"

using namespace std;

RobotCommunication::RobotCommunication()
{
	// server info
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
}

RobotCommunication::RobotCommunication(string addr, int port)
{
	// server info
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr(addr.data());
	server_addr.sin_port = htons(port);
}

RobotCommunication::~RobotCommunication()
{
	closesocket(s_server);
	// release dll
	WSACleanup();
}

void RobotCommunication::connect_server()
{
	// initialize socket
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	// check version
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
	// create socket
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}
}

void RobotCommunication::initialize_command()
{
	// login
	communicate("[1# System.Login 0]", "[login]", 1200);
	// enable
	communicate("[2# Robot.PowerEnable 1,1]", "[enable]", 1200);
	// auto mode
	communicate("[3# System.Auto 1]", "[AutoMode]", 200);
	// system abort
	communicate("[3# System.Abort 1]", "[Abort]");
	// system start
	communicate("[4# System.Start 1]", "[Start]");
	// robot home
	communicate("[4# Robot.Home 1]", "[Home]");
	// speed
	communicate("[6# System.Speed 10]", "[Speed]");
}

void RobotCommunication::communicate(const char *s_send, const string &log_header, int delay=100)
{
	char recv_buf[200] = {};
	int send_len = send(s_server, s_send, 100, 0);
	int recv_len = recv(s_server, recv_buf, 100, 0);
	cout << log_header << "\t" << recv_buf << endl;
	Sleep(delay);
}

void RobotCommunication::initialize()
{
	connect_server();
	initialize_command();
}
