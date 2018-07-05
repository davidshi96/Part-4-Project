#include "stdafx.h"
#include "Server.h"

void Server::initialiseServer()
{

	// Initialze winsock
	WSADATA wsData;
	WORD ver = MAKEWORD(2, 2);

	int wsOk = WSAStartup(ver, &wsData);
	if (wsOk != 0)
	{
		cerr << "Can't Initialize winsock! Quitting" << endl;
		return;
	}

	// Create a socket
	SOCKET listening = socket(AF_INET, SOCK_STREAM, 0);
	if (listening == INVALID_SOCKET)
	{
		cerr << "Can't create a socket! Quitting" << endl;
		return;
	}

	// Bind the ip address and port to a socket
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(54000);
	hint.sin_addr.S_un.S_addr = INADDR_ANY; // Could also use inet_pton .... 

	bind(listening, (sockaddr*)&hint, sizeof(hint));

	// Tell Winsock the socket is for listening 
	listen(listening, SOMAXCONN);

	// Wait for a connection
	sockaddr_in client;
	int clientSize = sizeof(client);
	clientSocket = accept(listening, (sockaddr*)&client, &clientSize);

	char host[NI_MAXHOST];		// Client's remote name
	char service[NI_MAXSERV];	// Service (i.e. port) the client is connect on

	ZeroMemory(host, NI_MAXHOST); // same as memset(host, 0, NI_MAXHOST);
	ZeroMemory(service, NI_MAXSERV);

	if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
	{
		cout << host << " connected on port " << service << endl;
	}
	else
	{
		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
		cout << host << " connected on port " <<
			ntohs(client.sin_port) << endl;
	}

	// Close listening socket
	closesocket(listening);

	return;

}

void Server::sendData(int x, int y, int depth)
{
	string X = to_string(x);
	while (size(X) < 4)
	{
		X.insert(0, 1, '0');
	}
	
	string Y = to_string(y);
	while (size(Y) < 4)
	{
		Y.insert(0, 1, '0');
	}

	string DEPTH = to_string(depth);
	while (size(DEPTH) < 4)
	{
		DEPTH.insert(0, 1, '0');
	}

	string dataToSend = X + "," + Y + "," + DEPTH + "\n";
	if (clientSocket == 0)
	{
		cout << "failed to accept client connection" << endl;
	}
	else
	{
		send(clientSocket, dataToSend.c_str(), dataToSend.size(), 0); // Send the string 
	}
}

void Server::closeServer()
{
	closesocket(clientSocket);
	WSACleanup();
}
