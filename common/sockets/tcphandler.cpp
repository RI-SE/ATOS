/**
 * @file tcphandler.cpp
 * @author Adam Eriksson 
 * @brief  TCP handler class that can be either server or client. 
 * @version 0.1
 * @date 2020-04-07
 * 
 * @copyright Copyright (c) AstaZero 2020
 * 
 */

#include <unistd.h>

#include "tcphandler.hpp"

#define MAX_WAITING_CONNECTIONS 3 // TODO make this modifiable

/**
 * @brief Construct a new TCPHandler::TCPHandler object.
 *  If all variabels are define it as initialise a server or client.
 * create new struct sockaddr_in addr.
 * @param P Port number.
 * @param IP IP addres. 
 * @param CorS Choose to create Server or Client. Acceptable Input is "Server" or "Client". 
 * @param sockoptionServer Set options in setsockoption
 * @param flagsock Set flags on socket like O_NONBLOCK
 */
TCPHandler::TCPHandler(int P, const std::string IP, std::string CorS, int sockoptionServer, int flag)
{
    PORT = P;
    IPaddr = IP;
    sockoption = sockoptionServer;
    flagsock = flag;

	if (CorS == "Server") {
        ClientorServer = CorS;
        errorState = static_cast<ErrorCode>(CreateServer());
    }
	else if (CorS == "Client") {
        ClientorServer = CorS;
        errorState = static_cast<ErrorCode>(CreateClient());
    }
}
/**
 * @brief Construct a new TCPHandler::TCPHandler object
 * But without the initialise variabels.
 * create new struct sockaddr_in addr.
 */
TCPHandler::TCPHandler()
{
}
/**
 * @brief Destroy the TCPHandler::TCPHandler object
 * 
 */
TCPHandler::~TCPHandler()
{
	TCPHandlerclose();
}


/*!
 * \brief TCPHandler::TCPHandler Move constructor, invalidate
 *			data of copied object after moving it to constructed
 *			object
 * \param other Object to be moved into constructed object
 */
TCPHandler::TCPHandler(TCPHandler&& other) :
	PORT(other.PORT),
	IPaddr(other.IPaddr),
	sockoption(other.sockoption),
	flagsock(other.flagsock),
	ConnectionON(other.ConnectionON),
	servaddr(other.servaddr),
	cliaddr(other.cliaddr),
	sockserver(other.sockserver),
	sockfd(other.sockfd),
	errorState(other.errorState),
	ClientorServer(other.ClientorServer)
{
	other.sockfd = -1;
	other.sockserver = -1;
	other.errorState = FAILURE;
	other.ConnectionON = -1;
	other.IPaddr.clear();
	other.PORT = -1;
	memset(&other.servaddr, 0, sizeof (other.servaddr));
	memset(&other.cliaddr, 0, sizeof (other.cliaddr));
}
/**
 * @brief Define the TCPHandler as a Client 
 * initialise the Client with port to read and write to. 
 * Ipaddres to read from.
 * 
 * @param P port number
 * @param IP IP addres 
 * @return int success = 1, fail = -1. 
 */
int TCPHandler::CreateClient(int P, const std::string IP)
{
    PORT = P;
    IPaddr = IP;

	return CreateClient();
}
/**
 * @brief define TCPHandler as a client.
 * need to initlise the port, before calling on this function. 
 * 
 * @return int success = 1, fail = -1.  
 */
int TCPHandler::CreateClient()
{
	if (PORT == -1) {
        perror("\nError:TCPHandler CreateClient, Port number not defined in TCPhandler class\n");
		return(FAILURE);
    }
	if( ClientorServer== "Server") {
        perror("\nError: TCPHandler CreateClient, Class already declared as Server \n");
		return FAILURE;
    }
    ClientorServer = "Client";

    if ( (sockfd = socket(AF_INET, SOCK_STREAM | flagsock, 0)) < 0 ) { 
        perror("Error: TCPHandler CreateClient: Socket creation failed"); 
		return FAILURE;
    }
	memset(&servaddr, 0, sizeof(servaddr));
    
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
    
	if(!IPaddr.empty()) {
		if(inet_pton(AF_INET, IPaddr.c_str(), &servaddr.sin_addr) <= 0) {
            perror("\nError: TCPHandler CreateClient, Invalid address/ Address not supported \n"); 
			return FAILURE;
        }
    } 
	else {
        perror("\n Error:TCPHandler CreateClient, No Ipaddres declared for Client\n");
		return FAILURE;
    }
	return TCPHandlerConnect();
}
/**
 * @brief Define the TCPHandler as a Server.
 * initialise the Server with port to read and write to. 
 * Ipaddres to read from 
 * create new struct sockaddr_in addr.
 * 
 * @param P Port number.
 * @param IP IP address
 * @return int success = 1, fail = -1.  
 */
int TCPHandler::CreateServer(int P, std::string IP, int sockoption)
{
	PORT = P;
    IPaddr = IP;
	if (ClientorServer == "Client") {
        perror("\nError: TCPHandler CreateServer, Class already declared as Client \n");
		return(FAILURE);
    }
    
	ClientorServer = "Server";

    if ( (sockfd = socket(AF_INET, SOCK_STREAM |flagsock, 0)) < 0 ) { 
        perror("Error: TCPHandler CreateServer, Socket creation failed"); 
		return(FAILURE);
    }
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &sockoption, sizeof(sockoption))) 
    { 
        perror("Error: TCPHandler CreateServer setsockopt failed"); 
		return(FAILURE);
    } 
	memset(&cliaddr, 0, sizeof(cliaddr));
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	if (!IPaddr.empty()) {
		if(inet_pton(AF_INET, IPaddr.c_str(), &servaddr.sin_addr) <= 0) {
            perror("\nError: TCPHandler CreateServer, Invalid address/ Address not supported \n"); 
			return FAILURE;
        }
    } 
	else {
		 servaddr.sin_addr.s_addr = INADDR_ANY;
    }

	if(TCPHandlerbind() < 0) {
		ConnectionON = FAILURE;
		return FAILURE;
    }
	if(TCPHandlerListen() < 0) {
		ConnectionON = FAILURE;
		return FAILURE;
    }

	return TCPHandlerAccept();
}
/**
 * @brief Create Server for the TCPhandler.
 * The port needs to be defined for this. Otherwise it will fail.
 * create new struct sockaddr_in addr.
 * 
 * @return int success = 1, fail = -1. 
 */
int TCPHandler::CreateServer()
{
	if (PORT ==-1) {
        perror("\nError:TCPHandler CreateServer, Port number not defined in TCPhandler class \n");
		return(FAILURE);
    }
	if (ClientorServer == "Client") {
        perror("\nError: TCPHandler CreateServer, Class already declared as Client \n");
		return FAILURE;
    }
    
	ClientorServer = "Server";

    if ( (sockfd = socket(AF_INET, SOCK_STREAM | flagsock, 0)) < 0 ) { 
        perror("Error: TCPHandler CreateServer, Socket creation failed"); 
        
		return FAILURE;
    }
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &sockoption, sizeof(sockoption)))  {
        perror("Error: TCPHandler CreateServer setsockopt failed"); 
        
		return FAILURE;
    }
	memset(&cliaddr, 0, sizeof(cliaddr));
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	if(!IPaddr.empty()) {
		if (inet_pton(AF_INET, IPaddr.c_str(), &servaddr.sin_addr) <= 0) {
            perror("\nError: TCPHandler CreateServer, Invalid address/ Address not supported \n"); 
			ConnectionON = FAILURE;
			return FAILURE;
        }
    } 
	else {
		 servaddr.sin_addr.s_addr = INADDR_ANY;
    }

	if(TCPHandlerbind() < 0) {
		ConnectionON = FAILURE;
		return FAILURE;
    }

	if(TCPHandlerListen() < 0) {
		ConnectionON = FAILURE;
		return FAILURE;
    }
  
	return (TCPHandlerAccept());

}
/**
 * @brief Binds the socket in the TCPhandler
 * 
 * @return int success = 1, fail = -1. 
 */

int TCPHandler::TCPHandlerbind()
{
   if ( bind(sockfd, (const struct sockaddr *)(&servaddr), sizeof(servaddr)) < 0 )
    { 
        perror("\n Error: TCPHandler TCPHandlerbind, bind failed \n");    
		return FAILURE;
    }
    
	return(SUCCESS);

}
/**
 * @brief Connect to specific socket.
 * 
 * @return int success = 1, fail = -1. 
 */
int TCPHandler::TCPHandlerConnect()
{
	if(connect(sockfd, (struct sockaddr *)(&servaddr), sizeof(servaddr)) < 0)
    { 
        TCPHandlerclose();
        perror("\n Error: TCPHandler TCPHandlerConnect, Connect Failed \n"); 
		ConnectionON = FAILURE;
		return FAILURE;
       
    }
	ConnectionON = SUCCESS;
	return SUCCESS;
}
/**
 * @brief Listen for establishement of connection from the client
 * 
 * @return int success = 1, fail = -1.  
 */
int TCPHandler::TCPHandlerListen()
{
	if(listen(sockfd, MAX_WAITING_CONNECTIONS) < 0)
    {
        perror("Error: TCPhandler TCPHandlerListen , Listen failed");
		ConnectionON = FAILURE;
		return FAILURE;
    }
	ConnectionON = SUCCESS;
	return(SUCCESS);
}

/**
 * @brief Accept connection from client and establish a new socket for sending data.
 * 
 * @param time timer for poll function if 0 no poll waiting.
 * @return int int success = 1, fail = -1.
 */
int TCPHandler::TCPHandlerAccept(int time)
{
	socklen_t addrlen = sizeof(cliaddr);
    struct pollfd pfd;
    pfd.events = POLLIN;
    pfd.fd = sockserver;
    int timeout;

    if(flagsock !=SOCK_STREAM && time >= 0 )
    {
        timeout = poll(&pfd, 1, time);
    }
    
    sockserver = accept(sockfd, (struct sockaddr *)(&cliaddr),
					   &addrlen);

	if (sockserver < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    { 
        ConnectionON = 0;
        return ConnectionON;
    } 
    else if (sockserver < 0){
        perror("Error: TCPHandler TCPHandlerAccept, Accept failed"); 
		ConnectionON = FAILURE;
		return FAILURE;
    }
	ConnectionON = SUCCESS;
	return(SUCCESS);
}

/**
 * @brief Closes the TCPhandler sockets.
 * 
 */
void TCPHandler::TCPHandlerclose()
{
	if(sockfd) {
        close(sockfd);
    }
	if(sockserver) {
        close(sockserver);
    }
}
/**
 * @brief Return the client address in a client server connection
 * 
 * @return std::string IP Address
 */
std::string TCPHandler::getClientIP() {
    char IPstr [INET_ADDRSTRLEN]; 
    inet_ntop(AF_INET, &(cliaddr.sin_addr), IPstr, INET_ADDRSTRLEN);
    std::string clientIP (IPstr); 
    return (clientIP);
}

/**
 * @brief Returns the server in a client server connection
 * 
 * @return std::string IP Address
 */
std::string TCPHandler::getServerIP() {
    char IPstr [INET_ADDRSTRLEN]; 
    inet_ntop(AF_INET, &(servaddr.sin_addr), IPstr, INET_ADDRSTRLEN);
    std::string serverIP (IPstr); 
    return (serverIP);
}



