#include "module.hpp"

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/sysinfo.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>
#include <unistd.h>
#include <ifaddrs.h>
#include "maestroTime.h"
#include "datadictionary.h"
#include "util.h"

#include <chrono>
#include <map>

class SystemControl : public Module
{
public:
	SystemControl();
	const int64_t getQueueEmptyPollPeriod();
	bool isWorking();
	bool shouldExit();
	void initialize(LOG_LEVEL logLevel);
	void signalHandler(int signo);
	void receiveUserCommand();
	void processUserCommand();
	void sendUnsolicitedData();
	void pollModuleStatus();

private:
	static inline std::string const module_name = "system_control";
	//const std::string module_name = std::string("SystemControl");
	/* constants and datatypes */
	const int64_t QUEUE_EMPTY_POLL_PERIOD_NS = 10000000;

	static const int SYSTEM_CONTROL_RESPONSE_CODE_OK = 0x0001;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_ERROR = 0x0F10;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE = 0x0F20;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE = 0x0F25;

	static const int SYSTEM_CONTROL_RESPONSE_CODE_INVALID_LENGTH = 0x0F30;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_BUSY = 0x0F40;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_INVALID_SCRIPT = 0x0F50;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_INVALID_ENCRYPTION_CODE = 0x0F60;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_DECRYPTION_ERROR = 0x0F61;
	static const int SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA = 0x0F62;

	static const int SMALL_BUFFER_SIZE_6 = 6;
	static const int SMALL_BUFFER_SIZE_16 = 16;
	static const int SMALL_BUFFER_SIZE_20 = 20;
	static const int SMALL_BUFFER_SIZE_128 = 128;
	static const int SMALL_BUFFER_SIZE_1024 = 1024;

	static const int SYSTEM_CONTROL_ARG_CHAR_COUNT = 2;
	static const int SYSTEM_CONTROL_COMMAND_MAX_LENGTH = 32;
	static const int SYSTEM_CONTROL_ARG_MAX_COUNT = 6;
	static const int SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH = 32;

	static const int SYSTEM_CONTROL_RVSS_DATA_BUFFER = 128;

	static const int TCP_RECV_BUFFER_SIZE = 2048;
	
	static const int SYSTEM_CONTROL_SEND_BUFFER_SIZE = 1024;

	static const int SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE = 64;

	static const int SYSTEM_CONTROL_SERVER_PARAMETER_LIST_SIZE = 1024;

	static const int SYSTEM_CONTROL_RX_PACKET_SIZE = 1280;
	static const int SYSTEM_CONTROL_TX_PACKET_SIZE = SYSTEM_CONTROL_RX_PACKET_SIZE;
	static const int IPC_BUFFER_SIZE = SYSTEM_CONTROL_RX_PACKET_SIZE;

	typedef enum {
		SERVER_STATE_UNDEFINED,
		SERVER_STATE_INITIALIZED,
		SERVER_STATE_IDLE,
		SERVER_STATE_READY,
		SERVER_STATE_RUNNING,
		SERVER_STATE_INWORK,
		SERVER_STATE_ERROR,
	} ServerState_t;

	typedef enum {
		Idle_0, GetServerStatus_0, ArmScenario_0, DisarmScenario_0, StartScenario_1, stop_0, AbortScenario_0,
		InitializeScenario_0, ConnectObject_0, DisconnectObject_0, GetServerParameterList_0,
		SetServerParameter_2, GetServerParameter_1, DownloadFile_1, UploadFile_4, CheckFileDirectoryExist_1,
		GetRootDirectoryContent_0, GetDirectoryContent_1, DeleteTrajectory_1, DeleteGeofence_1,
		DeleteFileDirectory_1,
		ClearTrajectories_0, ClearGeofences_0, ClearObjects_0, RemoteControl_1, RemoteControlManoeuvre_2,
		SetObjectEnableStatus_2,
		GetObjectEnableStatus_1,
		CreateDirectory_1, GetTestOrigin_0, replay_1, control_0, Exit_0,
		start_ext_trigg_1, ClearAllScenario_0 , DownloadDirectoryContent_1, DownloadTrajFiles_0, nocommand
	} SystemControlCommand_t;

	struct content_dir_info {
		int exist;
		int fd;
		char *info_buffer;
		int size;
	};

	typedef enum {
		MSCP_BACK_TO_START = 3
	} MSCPRemoteControlCommand;
	
	/* callbacks */
	void onGetStatusResponse(const ROSChannels::GetStatusResponse::message_type::SharedPtr) override;
	void onFailureMessage(const ROSChannels::Failure::message_type::SharedPtr) override;
	void onBackToStartResponse(const ROSChannels::BackToStartResponse::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;

	/* methods */
	SystemControlCommand_t SystemControlFindCommand(const char *CommandBuffer,
													SystemControlCommand_t * CurrentCommand, int *ArgCount);
	I32 SystemControlInitServer(int *ClientSocket, int *ServerHandle, struct in_addr *ip_addr);
	I32 SystemControlConnectServer(int *sockfd, const char *name, const uint32_t port);
	void SystemControlSendBytes(const char *data, int length, int *sockfd, int debug);
	void SystemControlSendControlResponse(U16 ResponseStatus, const char * ResponseString, const char * ResponseData,
										I32 ResponseDataLength, I32 * Sockfd, U8 Debug);
	I32 SystemControlBuildControlResponse(U16 ResponseStatus, char * ResponseString, char * ResponseData,
										I32 ResponseDataLength, U8 Debug);
	void SystemControlFileDownloadResponse(U16 ResponseStatus, const char * ResponseString,
										I32 ResponseDataLength, I32 * Sockfd, U8 Debug);
	void SystemControlSendLog(const char * LogString, I32 * Sockfd, U8 Debug);
	void SystemControlSendMONR(const char * LogString, I32 * Sockfd, U8 Debug);
	void SystemControlCreateProcessChannel(const char * name, const U32 port, I32 * sockfd,
												struct sockaddr_in *addr);
	//I32 SystemControlSendUDPData(I32 *sockfd, struct sockaddr_in* addr, char *SendData, I32 Length, U8 debug);
	I32 SystemControlReadServerParameterList(char * ParameterList, U8 debug);
	I32 SystemControlGetServerParameter(char * ParameterName, char * ReturnValue, U32 BufferLength,
										U8 Debug);
	I32 SystemControlSetServerParameter(char * ParameterName, char * NewValue, U8 Debug);
	I32 SystemControlCheckFileDirectoryExist(char * ParameterName, char * ReturnValue, U8 Debug);
	I32 SystemControlUploadFile(const char * Filename, const char * FileSize, const char * PacketSize, const char * FileType, char * ReturnValue,
								char * CompleteFilePath, U8 Debug);
	I32 SystemControlReceiveRxData(I32 * sockfd, const char * Path, const char * FileSize, const char * PacketSize, char * ReturnValue,
								U8 Debug);
	char SystemControlDeleteTrajectory(const char * trajectoryName, const size_t nameLen);
	char SystemControlDeleteGeofence(const char * geofenceName, const size_t nameLen);
	char SystemControlDeleteGenericFile(const char * filePath, const size_t nameLen);
	char SystemControlClearTrajectories(void);
	char SystemControlClearGeofences(void);
	char SystemControlClearObjects(void);
	I32 SystemControlDeleteFileDirectory(const char * Path, char * ReturnValue, U8 Debug);
	I32 SystemControlBuildFileContentInfo(const char * Path, U8 Debug);
	I32 SystemControlDestroyFileContentInfo(const char * Path, U8 RemoveFile);
	I32 SystemControlSendFileContent(I32 * sockfd, const char * Path, const char * PacketSize, char * ReturnValue, U8 Remove,
									U8 Debug);
	I32 SystemControlCreateDirectory(const char * Path, char * ReturnValue, U8 Debug);
	I32 SystemControlBuildRVSSTimeChannelMessage(char * RVSSData, U32 * RVSSDataLengthU32,
												U8 Debug);
	I32 SystemControlBuildRVSSMaestroChannelMessage(char * RVSSData, U32 * RVSSDataLengthU32,
													U8 SysCtrlState, U8 Debug);
	I32 SystemControlBuildRVSSAspChannelMessage(char * RVSSData, U32 * RVSSDataLengthU32, U8 Debug);
	int32_t SystemControlSendRVSSMonitorChannelMessages(int *socket, struct sockaddr_in *addr);
	void SystemControlUpdateRVSSSendTime(struct timeval *currentRVSSSendTime, uint8_t RVSSRate_Hz);

	I32 SystemControlGetStatusMessage(const char *respondingModule, U8 debug);

	ssize_t SystemControlReceiveUserControlData(I32 socket, char * dataBuffer, size_t dataBufferLength);
	char SystemControlVerifyHostAddress(char *ip);

	void appendSysInfoString(char *ControlResponseBuffer, const size_t bufferSize);

	/* variables */
	const char *SystemControlStatesArr[7] =
		{ "UNDEFINED", "INITIALIZED", "IDLE", "READY", "RUNNING", "INWORK", "ERROR" };
	const char *SystemControlOBCStatesArr[8] =
		{ "UNDEFINED", "IDLE", "INITIALIZED", "CONNECTED", "ARMED", "RUNNING", "REMOTECONTROL", "ERROR" };
	const char *POSTRequestMandatoryContent[3] = { "POST", "HTTP/1.1", "\r\n\r\n" };
	const char *SystemControlCommandsArr[37] = {
		"Idle_0", "GetServerStatus_0", "ArmScenario_0", "DisarmScenario_0", "StartScenario_1", "stop_0",
		"AbortScenario_0", "InitializeScenario_0",
		"ConnectObject_0", "DisconnectObject_0", "GetServerParameterList_0", "SetServerParameter_2",
		"GetServerParameter_1", "DownloadFile_1", "UploadFile_4", "CheckFileDirectoryExist_1",
		"GetRootDirectoryContent_0", "GetDirectoryContent_1", "DeleteTrajectory_1", "DeleteGeofence_1",
		"DeleteFileDirectory_1",
		"ClearTrajectories_0", "ClearGeofences_0", "ClearObjects_0", "RemoteControl_1",
		"RemoteControlManoeuvre_2",
		"SetObjectEnableStatus_2",
		"GetObjectEnableStatus_1", "CreateDirectory_1", "GetTestOrigin_0", "replay_1",
		"control_0",
		"Exit_0", "start_ext_trigg_1", "ClearAllScenario_0", "DownloadDirectoryContent_1", "DownloadTrajFiles_0"
	};
	char SystemControlCommandArgCnt[SYSTEM_CONTROL_ARG_CHAR_COUNT];
	char SystemControlStrippedCommand[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];
	char SystemControlArgument[SYSTEM_CONTROL_ARG_MAX_COUNT][SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH];
	const char * STR_SYSTEM_CONTROL_RX_PACKET_SIZE="1280";
	const char * STR_SYSTEM_CONTROL_TX_PACKET_SIZE="1200";

	content_dir_info SystemControlDirectoryInfo;

	U8 ModeU8 = 0;
	ServiceSessionType SessionData;
	struct timeval CurrentTimeStruct;
	I32 FileLengthI32 = 0;
	U8 RVSSRateU8 = DEFAULT_RVSS_RATE;
	U32 RVSSConfigU32 = DEFAULT_RVSS_CONF;
	U64 OldTimeU64 = 0;
	U64 PollRateU64 = 0;
	U64 CurrentTimeU64 = 0;
	volatile int iExit;
	I32 ClientResult = 0;
	ServerState_t SystemControlState = SERVER_STATE_UNDEFINED;
	I32 ServerHandle;
	I32 ClientSocket = 0;
	struct sockaddr_in RVSSChannelAddr;
	struct in_addr ip_addr;
	I32 RVSSChannelSocket;
	struct timeval nextRVSSSendTime = { 0, 0 };

	OBCState_t objectControlState = OBC_STATE_UNDEFINED;
	SystemControlCommand_t SystemControlCommand = Idle_0;
	SystemControlCommand_t PreviousSystemControlCommand = Idle_0;
	uint16_t responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
	int CommandArgCount = 0, CurrentInputArgCount = 0;
	char pcBuffer[IPC_BUFFER_SIZE];
	char inchr;
	struct timeval tvTime;
	const struct timeval VirtualMachineLagCompensation = { VIRTUAL_MACHINE_LAG_COMPENSATION_S,
		VIRTUAL_MACHINE_LAG_COMPENSATION_US
	};
	ObjectPosition OP;
	int i, i1;
	char *StartPtr, *StopPtr, *CmdPtr, *OpeningQuotationMarkPtr, *ClosingQuotationMarkPtr, *StringPos;
	struct timespec tTime;
	enum COMMAND iCommand;
	ssize_t bytesReceived = 0;
	char ObjectIP[SMALL_BUFFER_SIZE_16];
	char ObjectPort[SMALL_BUFFER_SIZE_6];
	U64 uiTime;
	U32 DelayedStartU32;
	char TextBufferchar[SMALL_BUFFER_SIZE_20];
	char ServerIPchar[SMALL_BUFFER_SIZE_20];
	char Usernamechar[SMALL_BUFFER_SIZE_20];
	char Passwordchar[SMALL_BUFFER_SIZE_20];
	U16 ServerPortU16;
	I32 ServerSocketI32 = 0;
	char RemoteServerRxData[1024];
	struct timespec sleep_time, ref_time;
	U64 TimeDiffU64 = 0;
	char ControlResponseBuffer[SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE];
	char TextBuffer20[SMALL_BUFFER_SIZE_20];
	char UserControlIPchar[SMALL_BUFFER_SIZE_20];
	U16 MilliU16 = 0, NowU16 = 0;
	U64 GPSmsU64 = 0;
	char ParameterListchar[SYSTEM_CONTROL_SERVER_PARAMETER_LIST_SIZE];
	U32 LengthU32 = 0;
	char BinBuffer[SMALL_BUFFER_SIZE_1024];
	char TxBuffer[SYSTEM_CONTROL_TX_PACKET_SIZE];

	HTTPHeaderContent HTTPHeader;

	char RVSSData[SYSTEM_CONTROL_RVSS_DATA_BUFFER];
	U32 RVSSMessageLengthU32;
	U16 PCDMessageCodeU16;
	char RxFilePath[MAX_FILE_PATH];

	std::map<std::string, std::chrono::steady_clock::time_point> moduleResponseTable;

	ROSChannels::Init::Pub initPub;
	ROSChannels::Connect::Pub connectPub;
	ROSChannels::Disconnect::Pub disconnectPub;
	ROSChannels::Arm::Pub armPub;
	ROSChannels::Disarm::Pub disarmPub;
	ROSChannels::Start::Pub startPub;
	ROSChannels::Stop::Pub stopPub;
	ROSChannels::Abort::Pub abortPub;
	ROSChannels::AllClear::Pub allClearPub;
	ROSChannels::BackToStart::Pub backToStartPub;
	ROSChannels::DataDictionary::Pub dataDictionaryPub;
	ROSChannels::RemoteControlEnable::Pub remoteControlEnablePub;
	ROSChannels::RemoteControlDisable::Pub remoteControlDisablePub;
	ROSChannels::EnableObject::Pub enableObjectPub;
	ROSChannels::Exit::Pub exitPub;
	ROSChannels::GetStatus::Pub getStatusPub;

	ROSChannels::Failure::Sub failureSub;
	ROSChannels::GetStatusResponse::Sub getStatusResponseSub;


};
