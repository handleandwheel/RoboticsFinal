#include "winsock.h"
#include "string"

using namespace std;


class RobotCommunication
{
public:
	RobotCommunication();
	RobotCommunication(string addr, int port);
	~RobotCommunication();
	void initialize();
	void communicate(const char* s_send, const string& log_header, int delay);
private:
	void connect_server();
	void initialize_command();
	SOCKET s_server;
	SOCKADDR_IN server_addr;
};