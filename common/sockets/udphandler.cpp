/**
 * @file udphandler.cpp
 * @author Adam Eriksson 
 * @brief  UDP handler class that can be either server or client. 
 * @version 0.1
 * @date 2020-04-07
 * 
 * @copyright Copyright (c) AstaZero 2020
 * 
 */

#include <climits>
#include <string.h>
#include <unistd.h>


#include "udphandler.hpp"

/**
 * @brief Construct a new UDPHandler::UDPHandler object.
 *  If all variabels are define it as initialise a server or client.
 * create new struct sockaddr_in servaddr.
 * @param P Port number.
 * @param IP IP address.
 * @param b UDP broadcast option, if b>0 then it creates a brodcast client or server. 
 * @param CorS Choose to create Server or Client. Acceptable Input is "Server" or "Client". 
 */
UDPHandler::UDPHandler(int P, const std::string IP, int b, std::string CorS)
{
    PORT = P;
    IPaddr = IP;
    broadcast = b;
    
    if (CorS == "Server")
    {  
        ClientorServer = CorS;
		errorState = static_cast<ERROR_UDPHANDLER>(CreateServer());
    }
    else if (CorS == "Client")
    {
        ClientorServer = CorS;
		errorState = static_cast<ERROR_UDPHANDLER>(CreateClient());
    }
    
    
}
/**
 * @brief Construct a new UDPHandler::UDPHandler object
 * But without the initialise variabels.
 * create new struct sockaddr_in servaddr.
 */
UDPHandler::UDPHandler()
{
}
/**
 * @brief Destroy the UDPHandler::UDPHandler object
 * 
 */
UDPHandler::~UDPHandler()
{
	UDPHandlerclose();
}

/*!
 * \brief UDPHandler::UDPHandler Move constructor, invalidate
 *			data of copied object after moving it to constructed
 *			object
 * \param other Object to be moved into constructed object
 */
UDPHandler::UDPHandler(UDPHandler&& other) :
	PORT(other.PORT),
	IPaddr(other.IPaddr),
	broadcast(other.broadcast),
	servaddr(other.servaddr),
	cliaddr(other.cliaddr),
	sockfd(other.sockfd),
	ClientorServer(other.ClientorServer),
	errorState(other.errorState)
{
	other.sockfd = -1;
	other.errorState = UDPHANDLER_FAIL;
	other.IPaddr.clear();
	other.PORT = -1;
	other.broadcast = -1;
	memset(&other.servaddr, 0, sizeof (other.servaddr));
	memset(&other.cliaddr, 0, sizeof (other.cliaddr));
}


/**
 * @brief Define the UDPHandler as a Client 
 * initialise the Client with port to read and write to. 
 * Ipaddres to read from and if it should broadcast or not.  
 * 
 * @param P port number
 * @param IP IP addres 
 * @param b if b>0 then the client is a brodcast client
 * @return int success = 1, fail = -1. 
 */
int UDPHandler::CreateClient(int P, const std::string IP, int b)
{
    PORT = P;
	IPaddr = IP;
    broadcast = b;
	return CreateClient();
}

/**
 * @brief define UDPHandler as a client.
 * need to initlise the port, before calling on this function. 
 * 
 * @return int success = 1, fail = -1.  
 */
int UDPHandler::CreateClient()
{
	if (PORT == -1) {
        perror("\nError: UDPHandler CreateClient, Port number not defined in UDPhandler class\n");
        return(UDPHANDLER_FAIL);
    }
	if( ClientorServer== "Server") {
        perror("\nError: UDPHandler CreateClient, Class already declared as Server \n");
        return(UDPHANDLER_FAIL);
    }
    ClientorServer = "Client";

    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("Error: UDPHandler CreateClient, Socket creation failed"); 
        return(UDPHANDLER_FAIL);    
    }
	if (broadcast>0) {
		if(setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
            perror("Error: UDPHandler CreateClient, Setsocket to bradcast failed");
        }
    }
	memset(&servaddr, 0, sizeof(servaddr));
    
	servaddr.sin_family = AF_INET;
	if (PORT < USHRT_MAX) {
		servaddr.sin_port = htons(static_cast<unsigned short>(PORT));
	}
	else {
		std::cerr << "UDP port configuration invalid: " << PORT << std::endl;
		return UDPHANDLER_FAIL;
	}

   

    return( setIP(IPaddr));
}
/**
 * @brief Define the UDPHandler as a Server.
 * initialise the Server with port to read and write to. 
 * Ipaddres to read from and if it should broadcast or not. 
 * create new struct sockaddr_in cliaddr.
 * 
 * @param P Port number.
 * @param IP IP address
 * @param b brodcast option brodcast if b>0
 * @return int success = 1, fail = -1.  
 */
int UDPHandler::CreateServer(int P, const std::string IP, int b)
{
    PORT =P;
    IPaddr = IP;
	broadcast = b;

	return CreateServer();
}
/**
 * @brief Create Server for the UDphandler.
 * The port needs to be defined for this. Otherwise it will fail.
 * create new struct sockaddr_in cliaddr.
 * 
 * @return int success = 1, fail = -1. 
 */
int UDPHandler::CreateServer()
{
    if(PORT ==-1) {
        perror("\nError: UDPHandler CreateServer, Port number not defined in UDPhandler class \n");
        return(UDPHANDLER_FAIL);
    }
    if(ClientorServer == "Client") {
        perror("\nError: UDPHandler CreateServer, Class already declared as Client \n");
        return(UDPHANDLER_FAIL);
    }
    
	ClientorServer = "Server";

    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("Error: UDPHandler CreateServer, Socket creation failed"); 
        return(UDPHANDLER_FAIL);    
    } 

	if(setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        perror("Error: UDPHandler CreateServer, Setsocket to bradcast failed ");
		return UDPHANDLER_FAIL;
    }

	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));
	servaddr.sin_family = AF_INET;
	if (PORT < USHRT_MAX) {
		servaddr.sin_port = htons(static_cast<unsigned short>(PORT));
	}
	else {
		std::cerr << "UDP port configuration invalid: " << PORT << std::endl;
		return UDPHANDLER_FAIL;
	}


    return(setIP(IPaddr));
    
}


/**
 * @brief Binds the socket in the udphandler
 * 
 * @return int success = 1, fail = -1. 
 */
int UDPHandler::UDPHandlerbind()
{
	if ( bind(sockfd, (const struct sockaddr *)(&servaddr), sizeof(servaddr)) < 0 ) {
        perror("Error: UDPHandler bind failed ");
        return(UDPHANDLER_FAIL);
    }
     
    return(UDPHANDLER_SUCCESS); 

}
/**
 * @brief Connect to specific socket.
 * 
 * @return int success = 1, fail = -1. 
 */
int UDPHandler::UDPHandlerConnect()
{
    // Need to disolve the existing association for the connect call/socket, when using the same client to reconnect to a new IP
    // with the connect or sendto functions.  
    // Therefore tempoary change the socket behaviour to AF_UNSPEC, before trying to connect to another IP again. read "man connect" in Unix for more info
    int res = -1;
    struct sockaddr tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.sa_family = AF_UNSPEC;
    res = connect (sockfd, (const struct sockaddr*) (&tmp), sizeof(tmp)); 
    res = connect(sockfd, (const struct sockaddr *)(&servaddr), sizeof(servaddr));
    if (res < 0) {
        printf("Errno in UDPHandler UDPhandlerConnect! %s\n", strerror(errno));
        return res;
    }

    return res;
}

/**
 * @brief Closes the UDphandler socket.
 * 
 */
void UDPHandler::UDPHandlerclose()
{
    close(sockfd);
}

/**
 * @brief Set the new IP/interface given the sockets and parts already configured
 * 
 * @param IPaddr std::string IPaddres to bind the class to
 */
int UDPHandler::setIP(const std::string IPaddr) {

    if(!IPaddr.empty()) {
		if(inet_pton(AF_INET, IPaddr.c_str(), &servaddr.sin_addr)<=0) {
            perror("\nError: UDPHandler CreateClient, Invalid address/ Address not supported \n"); 
            return UDPHANDLER_FAIL; 
        } 
    }
	else {
		 servaddr.sin_addr.s_addr = INADDR_ANY;
    }
    
    if(ClientorServer == "Client") {
        return (UDPHandlerConnect()); 
    }
    else if (ClientorServer == "Server") {
        return (UDPHandlerbind());
    }
    else {
        perror("\nError: UDPHandler not Configured properly, set Client or Server");
        return UDPHANDLER_FAIL;
    }
}
/**
 * @brief Return the client address in a client server connection
 * 
 * @return std::string IP Address
 */
std::string UDPHandler::getClientIP (){
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
std::string UDPHandler::getServerIP (){
    char IPstr [INET_ADDRSTRLEN]; 
    inet_ntop(AF_INET, &(servaddr.sin_addr), IPstr, INET_ADDRSTRLEN);
    std::string serverIP (IPstr); 
    return (serverIP);
}
