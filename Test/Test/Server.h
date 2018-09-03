#pragma once
#include <iostream>
#include <WS2tcpip.h>
#include <string>
#pragma comment(lib, "ws2_32.lib")

using namespace std;

class Server
{
public:
	void initialiseServer();
	void sendData(int x, int y, int depth, int foundCircle, int rad);
	void closeServer();
private:

	SOCKET clientSocket;

};

