/**
 * @file tcphandler.hpp
 * @author Adam Eriksson
 * @brief tcp handler
 * @version 0.1
 * @date 2020-04-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef TCPHANDLER_H
#define TCPHANDLER_H

#include <sys/socket.h> 
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h> 
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>


class [[deprecated("TCPHandler will be removed in a future release. Use TCPServer/TCPClient instead.")]] TCPHandler
{
public:

	typedef enum {
		OFF,
		CLIENT,
		SERVER
	} OperationMode;

	typedef enum {
		SUCCESS = 0,
		FAILURE = -1
	} ErrorCode;

	TCPHandler();
	TCPHandler(int PORT, const std::string IPaddr, std::string ClientorServer= "off", int sockoption = 1, int flagsock = SOCK_STREAM);
    
    ~TCPHandler();

	TCPHandler(TCPHandler&& other);

	//! Copy constructor unexpected for file descriptors
	TCPHandler(const TCPHandler& other) = delete;

	int CreateClient(int PORT, const std::string IPaddr);
    int CreateClient();
	int CreateServer(int PORT, const std::string IPaddr, int sockoption = 1 );
    int CreateServer();
    int TCPHandlerbind();

	template<typename T>
	int receiveTCP(std::vector<T> &msg, int timeout) {
		int bytesread;
		struct pollfd pfd;
		assert(sizeof (T) == sizeof (char));

		pfd.events = POLLIN;
		if (ClientorServer == "Server") {
			pfd.fd = sockserver;
		}
		else if (ClientorServer == "Client") {
			pfd.fd = sockfd;
		}
		else {
			std::cerr <<"\n TCP handler is not defined as Server nor Client\n"<< std::endl;
			return FAILURE;
		}

		if (timeout != 0) {
			std::cout << "Timeout: " << timeout << ", fd: " << pfd.fd << std::endl;
			timeout = poll(&pfd, 1, timeout);
			if (timeout == 0) {
				std::cout <<"\n TCP handler timed out in the recive call.\n Time before timeout is set to "
						 << timeout <<"ms" << std::endl;
				return 0;
			}
			else if (timeout < 0) {
				std::cerr << "\n TCP handler poll error" << std::endl;
				return FAILURE;
			}
		}
        
		if (timeout >= 0) {
			
			bytesread = recv(pfd.fd, msg.data() , msg.size(), MSG_DONTWAIT);
			if (bytesread > 0) {
				return bytesread;
			}
			if (bytesread == 0) {
				std::cout << "Remote closed TCP connection" << std::endl;
				ConnectionON = FAILURE;
				return FAILURE;
			}
			if (bytesread < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
				if (timeout == 0) {
					return 0;
				}
				else {
					std::cerr << "Poll indicated data present but no data was received" << std::endl;
					return FAILURE;
				}
			}

		}
		return FAILURE;
	}

	template<typename T>
	int sendTCP(const std::vector<T>& msg) {
		int sendfd = 0;
		assert(sizeof (T) == sizeof (char));
		if (ClientorServer == "Server") {
			sendfd = sockserver;
		}
		else if (ClientorServer == "Client") {
			sendfd = sockfd;
		}
		else {
			std::cout <<"\n TCP handler is not defined as Server nor Client"<< std::endl;
			return FAILURE;
		}
		return send(sendfd, msg.data(), msg.size(), 0);
	}

    int TCPHandlerListen();
    int TCPHandlerAccept(int time = 0);

    int TCPHandlerConnect();
    void TCPHandlerclose();
	
	/**
	* @brief returns std::string of the private variabel ClientorServer. Tells if class is declared as client or server. 
	* 
	*/
    std::string getClientorServer()const {return ClientorServer; }
	
	/**
	* @brief get the value on connectionON private vairbel, 
	* which tells you if tcp connection is established
	* 
	* @return int 1 = connection is established, -1 = no connection established.
	*/
	int getConnectionOn() const {return static_cast<int>(ConnectionON);}
	bool isConnected() const { return connected; }
	/**
	* @brief Returns the class errorstate
	* 
	* @return int 1 = setup of client or server succeded, -1 = setup of client or sever failed 
	*/
	int getErrorState()const {return static_cast<int>(errorState);}

	std::string getClientIP();
	std::string getServerIP();
    /*Port number for the TCP handler*/
    int PORT = -1;
    /*IPaddress for the TCP handler (optional)*/
	std::string IPaddr;
	/*If the TCP handler should send data as broadcast or not 1> is true */
    int sockoption = 1;
	int flagsock  = SOCK_STREAM;
	// flag that tells us if TCP connection is established
private:
	bool connected = false;
	struct sockaddr_in hostAddr;
	struct sockaddr_in peerAddr;

	struct sockaddr_in servaddr;
	struct sockaddr_in cliaddr;
    int sockserver;
    int sockfd;

	int ConnectionON;
	ErrorCode errorState = FAILURE;
    
    std::string ClientorServer;


};
#endif
