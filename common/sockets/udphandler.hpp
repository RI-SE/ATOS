#ifndef UDPHANDLER_H
#define UDPHANDLER_H

#include <iostream>
#include <vector>
#include <cassert>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

class [[deprecated("UDPHandler will be removed in a future release. Use UDPServer/UDPClient instead.")]] UDPHandler
{
public:

	typedef enum {
		UDPHANDLER_SUCCESS = 1,
		UDPHANDLER_FAIL = -1
	} ERROR_UDPHANDLER;

    UDPHandler();
	UDPHandler(int PORT, const std::string IPaddr, int broadcast, std::string ClientorServer= "off");
    
    ~UDPHandler();

	UDPHandler(UDPHandler&& other);

	//! Copy constructor unexpected for file descriptors
	UDPHandler(const UDPHandler& other) = delete;


	int CreateClient(int PORT, const std::string IPaddr, int broadcast = 0);
    int CreateClient();
	int CreateServer(int PORT, const std::string IPaddr, int broadcast = 0);
    int CreateServer();
    int UDPHandlerbind();

	template<typename T>
	int receiveUDP(std::vector<T> &msg) {
		int bytesread = 0;
		socklen_t len;
		assert(sizeof (T) == sizeof (char));

		struct sockaddr* addr;
		if (ClientorServer == "Server") {
			len = sizeof(cliaddr);
			addr = reinterpret_cast<struct sockaddr *>(&cliaddr);
		}
		else if (ClientorServer == "Client") {
			len = sizeof(servaddr);
			addr = reinterpret_cast<struct sockaddr *>(&servaddr);
		}
		else {
			std::cerr <<"\n UDP handler is not defined as Server nor Client\n"<< std::endl;
			return UDPHANDLER_FAIL;
		}

		bytesread = recvfrom(sockfd, msg.data(), msg.size(), MSG_DONTWAIT, addr, &len);
		if (bytesread > 0) {
			return bytesread;
		}
		if (bytesread == 0) {
			std::cerr <<"\n UDP socket closed unexpectedly\n"<< std::endl;
			return UDPHANDLER_FAIL;
		}
		if (bytesread < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			return 0;
		}
		return bytesread;
	}

	template<typename T>
	long sendUDP(const std::vector<T> &msg) {
		socklen_t len = 0;
		struct sockaddr * addr = nullptr;
		assert(sizeof (T) == sizeof (char));
		if (ClientorServer == "Server") {
			len = sizeof(servaddr);
			addr = reinterpret_cast<struct sockaddr *>(&cliaddr);
		}
		else if (ClientorServer == "Client") {
			len = sizeof(cliaddr);
			addr = reinterpret_cast<struct sockaddr *>(&servaddr);
		}
		else {
			std::cout <<"\n UDP handler is not defined as Server nor Client, is defined as: "<<ClientorServer<< std::endl;
			return -1;
		}
		ssize_t res = -1;
		res = sendto(sockfd, msg.data(), msg.size(), 0, addr, len);
		if (res < 0){
			printf("Errno in UDPHandler SendUDP! %s\n", strerror(errno));
		}
	

		return res;
	}

	int UDPHandlerConnect();
    void UDPHandlerclose();
	/**
	* @brief returns std::string of the private variabel ClientorServer. Tells if class is declared as client or server. 
	* 
	*/
	std::string getClientorServer()const {return ClientorServer;}
	/**
 	* @brief Get the state of the class.
 	* 
 	* @return int 1 if clint/server setup succeded, -1 clint/server setup failed.
 	*/
	int getErrorState() const {return static_cast<int>(errorState);}
	int setIP(const std::string IP);
	/**
	 * @brief returns class variabel IPaddr
	 * 
	 * @return std::string "Ipaddr that the was set wehn configuring client or server"
	 */
	std::string getIPaddr() const {return static_cast<std::string>(IPaddr);}
	
	std::string getClientIP (); 
	std::string getServerIP ();
    /*Port number for the UDP handler*/
    int PORT = -1;
    /*IPaddress for the UDP handler (optional)*/
	
	std::string IPaddr; // TODO: Move to private  given no function is usiong it atm
    /*If the UDP handler should send data as brodcast or not 1> is true */
    int broadcast = -1;
	
private:
	struct sockaddr_in servaddr;
	struct sockaddr_in cliaddr;
    int sockfd;
    std::string ClientorServer;
	ERROR_UDPHANDLER errorState = UDPHANDLER_FAIL;


};
#endif
