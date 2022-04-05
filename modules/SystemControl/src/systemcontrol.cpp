#include "systemcontrol.hpp"

#define SYSTEM_CONTROL_SERVICE_POLL_TIME_MS 5000
#define SYSTEM_CONTROL_TASK_PERIOD_MS 1
#define SYSTEM_CONTROL_RVSS_TIME_MS 10

#define SYSTEM_CONTROL_GETSTATUS_TIME_MS 5000
#define SYSTEM_CONTROL_GETSTATUS_TIMEOUT_MS 2000
#define SYSTEM_CONTROL_NO_OF_MODULES_IN_USE 2	//TODO Create a file containing a list of which modules should be used. Check with this list to see if each module has responded.

#define SYSTEM_CONTROL_CONTROL_PORT   54241	// Default port, control channel
#define SYSTEM_CONTROL_PROCESS_PORT   54242	// Default port, process channel

#define GetCurrentDir getcwd
#define REMOVE_FILE 1
#define KEEP_FILE 0

#define RVSS_TIME_CHANNEL 1
#define RVSS_MONITOR_CHANNEL 2
#define RVSS_MAESTRO_CHANNEL 4
#define RVSS_ASP_CHANNEL 8

#define ENABLE_COMMAND_STRING "ENABLE"
#define DISABLE_COMMAND_STRING "DISABLE"

#define MAESTRO_GENERIC_FILE_TYPE     1
#define MAESTRO_TRAJ_FILE_TYPE        2
#define MAESTRO_CONF_FILE_TYPE        3
#define MAESTRO_GEOFENCE_FILE_TYPE    4
#define MAESTRO_OBJECT_FILE_TYPE	  5
#define MSCP_RESPONSE_DATALENGTH_BYTES 4
#define MSCP_RESPONSE_STATUS_CODE_BYTES 2

#define MAESTRO_TRAJ_DIRECTORY_STRING "traj/"


SystemControl::SystemControl()
: Module(SystemControl::module_name),
initPub(*this),
connectPub(*this),
disconnectPub(*this),
armPub(*this),
disarmPub(*this),
startPub(*this),
stopPub(*this),
abortPub(*this),
allClearPub(*this),
backToStartPub(*this),
dataDictionaryPub(*this),
remoteControlEnablePub(*this),
remoteControlDisablePub(*this),
enableObjectPub(*this),
exitPub(*this),
getStatusPub(*this),
failureSub(*this, std::bind(&SystemControl::onFailureMessage, this, std::placeholders::_1)),
getStatusResponseSub(*this, std::bind(&SystemControl::onGetStatusResponse, this, std::placeholders::_1))
{
}; 

const int64_t SystemControl::getQueueEmptyPollPeriod(){return QUEUE_EMPTY_POLL_PERIOD_NS;}

void SystemControl::onAbortMessage(const Empty::SharedPtr){}

void SystemControl::onAllClearMessage(const Empty::SharedPtr){}

void SystemControl::onFailureMessage(const UInt8::SharedPtr msg){
	if (SystemControlState == SERVER_STATE_INWORK) {
		enum COMMAND failedCommand = (COMMAND) (msg->data);

		if (failedCommand == COMM_INIT && PreviousSystemControlCommand == InitializeScenario_0) {
			SystemControlState = SERVER_STATE_IDLE;
			SystemControlCommand = Idle_0;
			RCLCPP_INFO(get_logger(), "Initialization failed");
			// TODO: report to user?
		}
		else {
			RCLCPP_ERROR(get_logger(), "Unhandled FAILURE (command: %u) reply in state %s",
						msg->data, SystemControlStatesArr[SystemControlState]);
		}
	}
	else {
	RCLCPP_WARN(get_logger(), "Received unexpected FAILURE (command: %u) reply in state %s",
				msg->data, SystemControlStatesArr[SystemControlState]);
	// TODO: React more?
	}
}

void SystemControl::onGetStatusResponse(const String::SharedPtr msg){
	moduleResponseTable[msg->data] = std::chrono::steady_clock::now();
}

void SystemControl::onBackToStartResponse(const Int8::SharedPtr msg){
	if(msg->data == BTS_FAIL){
		RCLCPP_DEBUG(get_logger(), "Back-to-start result: %s", msg->data);
		SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "BTS:",
											"0", 1,
											&ClientSocket, 0);		//TODO: Send SMCP response the right way(?)
	}
	else if(msg->data == BTS_PASS){
		RCLCPP_DEBUG(get_logger(), "Back-to-start result: %s", msg->data);
		memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
		SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "BTS:",
											"1", 1,
											&ClientSocket, 0);
	}
	else{
		RCLCPP_ERROR(get_logger(), "Cannot parse COMM_BACKTOSTART_RESPONSE message.");
	}
}

void SystemControl::signalHandler(int signo) {
	if (signo == SIGINT) {
		RCLCPP_WARN(get_logger(), "Caught keyboard interrupt");
		iExit = 1;
	}
	else {
		RCLCPP_ERROR(get_logger(), "Caught unhandled signal");
	}
}

bool SystemControl::isWorking(){
	return !(SystemControlState != SERVER_STATE_INWORK && (ClientResult < 0));
}
bool SystemControl::shouldExit(){
	return iExit;
}

void SystemControl::initialize(LOG_LEVEL logLevel){
	LogInit(get_name(), logLevel);
	RCLCPP_INFO(get_logger(), "System control task running with PID: %i", getpid());

	if (requestDataDictInitialization()) {
		// Map state and object data into memory
		if (DataDictionaryInitStateData() != READ_OK) {
			DataDictionaryFreeStateData();
			throw std::runtime_error("Found no previously initialized shared memory for state data");
		}
		if (DataDictionaryInitObjectData() != READ_OK) {
			DataDictionaryFreeObjectData();
			throw std::runtime_error("Found no previously initialized shared memory for object data");
		}
		if (DataDictionaryInitRVSSAsp() != READ_OK) {
			DataDictionaryFreeRVSSAsp();
			throw std::runtime_error("Found no previously initialized shared memory for RVSS ASP data");
		}

		DataDictionaryGetRVSSConfigU32(&RVSSConfigU32);
		RCLCPP_INFO(get_logger(), "RVSSConfigU32 = %d", RVSSConfigU32);

		DataDictionaryGetRVSSRateU8(&RVSSRateU8);
		RCLCPP_INFO(get_logger(), "Real-time variable subscription service rate set to %u Hz", RVSSRateU8);
	}
	else {
		throw std::runtime_error("Unable to initialize data dictionary");
	}
	if (ModeU8 == 0) {

	}
	else if (ModeU8 == 1) {
		SessionData.SessionIdU32 = 0;
		SessionData.UserIdU32 = 0;
		SessionData.UserTypeU8 = 0;

		PollRateU64 = SYSTEM_CONTROL_SERVICE_POLL_TIME_MS;
		CurrentTimeU64 =
			(uint64_t) CurrentTimeStruct.tv_sec * 1000 + (uint64_t) CurrentTimeStruct.tv_usec / 1000;
		OldTimeU64 = CurrentTimeU64;

	}
}


void SystemControl::receiveUserCommand()
{
	if (SystemControlState == SERVER_STATE_ERROR) {
		this->abortPub.publish(Empty());
		return;
	}

	if (ModeU8 == 0) {
		if (ClientSocket <= 0) {
			if (SystemControlState == SERVER_STATE_UNDEFINED) {
				//Do some initialization

				//Send COMM_DATA_DICT to notify to update data from DataDictionary
				this->dataDictionaryPub.publish(Empty());
				SystemControlState = SERVER_STATE_INITIALIZED;
			}

			if (USE_LOCAL_USER_CONTROL == 0) {

				ClientResult = SystemControlInitServer(&ClientSocket, &ServerHandle, &ip_addr);
				bzero(UserControlIPchar, SMALL_BUFFER_SIZE_20);
				sprintf((char* )UserControlIPchar, "%s", inet_ntoa(ip_addr));
				RCLCPP_INFO(get_logger(), "UserControl IP address is %s", inet_ntoa(ip_addr));
				SystemControlCreateProcessChannel(UserControlIPchar, SYSTEM_CONTROL_PROCESS_PORT,
													&RVSSChannelSocket, &RVSSChannelAddr);

			}
			if (USE_LOCAL_USER_CONTROL == 1) {
				ClientResult =
					SystemControlConnectServer(&ClientSocket, LOCAL_USER_CONTROL_IP,
												LOCAL_USER_CONTROL_PORT);
				SystemControlCreateProcessChannel(LOCAL_USER_CONTROL_IP, SYSTEM_CONTROL_PROCESS_PORT,
													&RVSSChannelSocket, &RVSSChannelAddr);
			}

			SystemControlState = SERVER_STATE_IDLE;
		}

		PreviousSystemControlCommand = SystemControlCommand;
		bzero(pcBuffer, IPC_BUFFER_SIZE);

		ClientResult = SystemControlReceiveUserControlData(ClientSocket, pcBuffer, sizeof (pcBuffer));

		if (ClientResult <= -1) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				RCLCPP_ERROR(get_logger(), "Failed to receive from command socket");
				try{
					this->abortPub.publish(Empty());
				}
				catch(...){
					util_error("Fatal communication fault when sending ABORT command");
				}
			}
		}
		else if (ClientResult == 0) {
			RCLCPP_INFO(get_logger(), "Client closed connection");
			close(ClientSocket);
			ClientSocket = -1;
			if (USE_LOCAL_USER_CONTROL == 0) {
				close(ServerHandle);
				ServerHandle = -1;
			}

			SystemControlCommand = AbortScenario_0;	//Oops no client is connected, go to AbortScenario_0
			SystemControlState == SERVER_STATE_UNDEFINED;	// TODO: Should this be an assignment?
		}
		else if (ClientResult > 0 && ClientResult < TCP_RECV_BUFFER_SIZE) {
			// TODO: Move this entire decoding process into a separate function
			for (i = 0; i < SYSTEM_CONTROL_ARG_MAX_COUNT; i++)
				bzero(SystemControlArgument[i], SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH);

			CurrentInputArgCount = 0;
			StartPtr =  pcBuffer;
			StopPtr = pcBuffer;
			CmdPtr = NULL;
			StringPos = pcBuffer;

			// Check so that all POST request mandatory content is contained in the message
			for (i = 0;
					i < sizeof (POSTRequestMandatoryContent) / sizeof (POSTRequestMandatoryContent[0]);
					++i) {

				StringPos = strstr(StringPos, POSTRequestMandatoryContent[i]);
				if (StringPos == NULL) {
					CmdPtr = NULL;
					break;
				}
				else {
					CmdPtr = StringPos + strlen(POSTRequestMandatoryContent[i]);
				}
			}

			if (CmdPtr != NULL) {
				// It is now known that the request contains "POST" and "\r\n\r\n", so we can decode the header
				UtilDecodeHTTPRequestHeader((char*)pcBuffer, &HTTPHeader);

				if (HTTPHeader.Host[0] == '\0') {
					RCLCPP_INFO(get_logger(), "Unspecified host in request <%s>", pcBuffer);
				}
				else if (SystemControlVerifyHostAddress(HTTPHeader.Host)) {
					// Find opening parenthesis
					StartPtr = strchr(CmdPtr, '(');
					// If there was no opening or closing parenthesis, the format is not correct
					if (StartPtr == NULL || strchr(StartPtr, ')') == NULL)
						RCLCPP_WARN(get_logger(),
									"Received command not conforming to MSCP standards");
					else {
						StartPtr++;
						while (StopPtr != NULL) {
							StopPtr = (char *)strchr(StartPtr, ',');

							// If there are no commas past this point, just copy the rest
							if (StopPtr == NULL) {
								strncpy(SystemControlArgument[CurrentInputArgCount], StartPtr,
										(uint64_t) strchr(StartPtr, ')') - (uint64_t) StartPtr);
							}
							// Otherwise, check if the comma we found was inside quotation marks
							else {
								OpeningQuotationMarkPtr = (char *)strchr(StartPtr, '"');

								if (OpeningQuotationMarkPtr == NULL) {
									// It was not within quotation marks: copy until the next comma
									strncpy(SystemControlArgument[CurrentInputArgCount], StartPtr,
											(uint64_t) StopPtr - (uint64_t) StartPtr);
								}
								else if (OpeningQuotationMarkPtr != NULL
											&& OpeningQuotationMarkPtr < StopPtr) {
									// A quotation mark was found and it was before the next comma: find the closing quotation mark
									ClosingQuotationMarkPtr =
										(char *)strchr(OpeningQuotationMarkPtr + 1, '"');


									if (ClosingQuotationMarkPtr == NULL) {
										CmdPtr = NULL;
										StopPtr = NULL;
										RCLCPP_WARN(get_logger(),
													"Received MSCP command with single quotation mark");
										break;
									}
									else {
										// Copy all arguments within quotation marks including the quotation marks
										strncpy(SystemControlArgument[CurrentInputArgCount],
												OpeningQuotationMarkPtr + 1,
												(uint64_t) (ClosingQuotationMarkPtr) -
												(uint64_t) (OpeningQuotationMarkPtr + 1));
										// Find next comma after closing quotation mark
										StopPtr = strchr(ClosingQuotationMarkPtr, ',');
									}
								}
							}
							StartPtr = StopPtr + 1;
							if (SystemControlArgument[CurrentInputArgCount][0] != '\0') {
								CurrentInputArgCount++;	// In case the argument was empty, don't add it to the argument count
							}
						}

						if (CmdPtr != NULL) {
							SystemControlFindCommand(CmdPtr, &SystemControlCommand, &CommandArgCount);
							RCLCPP_DEBUG(get_logger(), "Received MSCP command %s", SystemControlCommand);
						}
						else
							RCLCPP_WARN(get_logger(), "Invalid MSCP command received");
					}
				}
				else {
					RCLCPP_INFO(get_logger(),
								"Request specified host <%s> not among known local addresses",
								HTTPHeader.Host);
				}
			}
			else {
				RCLCPP_WARN(get_logger(),
							"Received badly formatted HTTP request: <%s>, must contain \"POST\" and \"\\r\\n\\r\\n\"",
							pcBuffer);
			}
		}
		else {
			RCLCPP_WARN(get_logger(), "Ignored received TCP message which was too large to handle");
		}

	}
	else if (ModeU8 == 1) {	/* use util.c function to call time
								gettimeofday(&CurrentTimeStruct, NULL);
								CurrentTimeU64 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;
								*/
		CurrentTimeU64 = UtilgetCurrentUTCtimeMS();
		TimeDiffU64 = CurrentTimeU64 - OldTimeU64;

	}


	if (DataDictionaryGetOBCState(&objectControlState) != READ_OK) {
		RCLCPP_ERROR(get_logger(), "Error fetching object control state");
		// TODO more?
	}

	if (SystemControlState == SERVER_STATE_INWORK) {
		if (SystemControlCommand == AbortScenario_0) {
			SystemControlCommand = SystemControlCommand;
		}
		else if (SystemControlCommand == GetServerStatus_0) {
			RCLCPP_DEBUG(get_logger(), "State: %s, OBCState: %s, PreviousCommand: %s",
						SystemControlStatesArr[SystemControlState],
						SystemControlOBCStatesArr[objectControlState],
						SystemControlCommandsArr[PreviousSystemControlCommand]);
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			ControlResponseBuffer[0] = SystemControlState;
			ControlResponseBuffer[1] = objectControlState;	//OBCStateU8;
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetServerStatus:",
												ControlResponseBuffer, 2, &ClientSocket, 0);
			SystemControlCommand = PreviousSystemControlCommand;
		}
		else if (SystemControlCommand != PreviousSystemControlCommand) {
			RCLCPP_WARN(get_logger(),
						"Command not allowed, SystemControl is busy in state %s, PreviousCommand: %s",
						SystemControlStatesArr[SystemControlState],
						SystemControlCommandsArr[PreviousSystemControlCommand]);
			SystemControlSendLog
				("[SystemControl] Command not allowed, SystemControl is busy in state INWORK.\n",
					&ClientSocket, 0);
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			SystemControlCommand = PreviousSystemControlCommand;
		}
	}
}

void SystemControl::sendUnsolicitedData(){
		TimeSetToCurrentSystemTime(&tvTime);

	if (timercmp(&tvTime, &nextRVSSSendTime, >)) {
		SystemControlUpdateRVSSSendTime(&nextRVSSSendTime, RVSSRateU8);

		if (RVSSChannelSocket != 0 && RVSSConfigU32 > 0) {
			memset(RVSSData, 0, sizeof (RVSSData));

			if (RVSSConfigU32 & RVSS_TIME_CHANNEL) {
				SystemControlBuildRVSSTimeChannelMessage(RVSSData, &RVSSMessageLengthU32, 0);
				UtilSendUDPData((uint8_t*) module_name.c_str(), &RVSSChannelSocket, &RVSSChannelAddr, (uint8_t*) RVSSData,
								RVSSMessageLengthU32, 0);
			}

			if (RVSSConfigU32 & RVSS_MAESTRO_CHANNEL) {
				SystemControlBuildRVSSMaestroChannelMessage(RVSSData, &RVSSMessageLengthU32,
															SystemControlState, 0);
				UtilSendUDPData((uint8_t*) module_name.c_str(), &RVSSChannelSocket, &RVSSChannelAddr, (uint8_t*) RVSSData,
								RVSSMessageLengthU32, 0);
			}

			if (RVSSConfigU32 & RVSS_ASP_CHANNEL) {
				SystemControlBuildRVSSAspChannelMessage(RVSSData, &RVSSMessageLengthU32, 0);
				UtilSendUDPData((uint8_t*) module_name.c_str(), &RVSSChannelSocket, &RVSSChannelAddr, (uint8_t*) RVSSData,
								RVSSMessageLengthU32, 0);
			}

			if (RVSSConfigU32 & RVSS_MONITOR_CHANNEL) {
				// Build and send MONR data of all objects
				if (RVSSChannelSocket != 0 && RVSSConfigU32 & RVSS_MONITOR_CHANNEL && bytesReceived >= 0) {
					SystemControlSendRVSSMonitorChannelMessages(&RVSSChannelSocket, &RVSSChannelAddr);
				}
			}
		}
	}
}

void SystemControl::processUserCommand()
{
	switch (SystemControlCommand) {
		// can you access GetServerParameterList_0, GetServerParameter_1, SetServerParameter_2 and DISarmScenario and Exit from the GUI
	case Idle_0:
		break;
	case GetServerStatus_0:
		DataDictionaryGetOBCState(&objectControlState);
		if (SystemControlCommand != PreviousSystemControlCommand) {
			RCLCPP_DEBUG(get_logger(), "State: %s, OBCState: %s",
						SystemControlStatesArr[SystemControlState],
						SystemControlOBCStatesArr[objectControlState]);
		}
		SystemControlCommand = Idle_0;
		bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
		ControlResponseBuffer[0] = SystemControlState;
		ControlResponseBuffer[1] = objectControlState;
		appendSysInfoString(ControlResponseBuffer + 2, sizeof (ControlResponseBuffer) - 2);
		RCLCPP_DEBUG(get_logger(), "Sending response: %s", ControlResponseBuffer);
		SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetServerStatus:",
											ControlResponseBuffer, strlen(ControlResponseBuffer),
											&ClientSocket, 0);
		break;
	case GetServerParameterList_0:
		SystemControlCommand = Idle_0;
		bzero(ParameterListchar, SYSTEM_CONTROL_SERVER_PARAMETER_LIST_SIZE);
		SystemControlReadServerParameterList(ParameterListchar, 0);
		SystemControlSendControlResponse(strlen(ParameterListchar) >
											0 ? SYSTEM_CONTROL_RESPONSE_CODE_OK :
											SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA, "GetServerParameterList:",
											ParameterListchar, strlen(ParameterListchar), &ClientSocket, 0);
		break;
	case GetTestOrigin_0:
		SystemControlCommand = Idle_0;
		bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
		DataDictionaryGetOriginLatitudeString(TextBuffer20, SMALL_BUFFER_SIZE_20);
		strcat(ControlResponseBuffer, TextBuffer20);
		strcat(ControlResponseBuffer, ";");
		DataDictionaryGetOriginLongitudeString(TextBuffer20, SMALL_BUFFER_SIZE_20);
		strcat(ControlResponseBuffer, TextBuffer20);
		strcat(ControlResponseBuffer, ";");
		DataDictionaryGetOriginAltitudeString(TextBuffer20, SMALL_BUFFER_SIZE_20);
		strcat(ControlResponseBuffer, TextBuffer20);
		strcat(ControlResponseBuffer, ";");

		SystemControlSendControlResponse(strlen(ParameterListchar) >
											0 ? SYSTEM_CONTROL_RESPONSE_CODE_OK :
											SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA, "GetTestOrigin:",
											ControlResponseBuffer, strlen(ControlResponseBuffer),
											&ClientSocket, 0);
		break;
	case GetServerParameter_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlGetServerParameter(SystemControlArgument[0], ControlResponseBuffer,
											SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE, 0);
			SystemControlSendControlResponse(strlen(ControlResponseBuffer) >
												0 ? SYSTEM_CONTROL_RESPONSE_CODE_OK :
												SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA, "GetServerParameter:",
												ControlResponseBuffer, strlen(ControlResponseBuffer),
												&ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(), "Wrong parameter count in GetServerParameter(Name)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case SetServerParameter_2:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSetServerParameter(SystemControlArgument[0], SystemControlArgument[1], 1);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "SetServerParameter:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			//Send COMM_DATA_DICT to notify to update data from DataDictionary
			this->dataDictionaryPub.publish(Empty());
		}
		else {
			RCLCPP_ERROR(get_logger(), "Wrong parameter count in SetServerParameter(Name, Value)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case CheckFileDirectoryExist_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "CheckFileDirectoryExist:",
												ControlResponseBuffer, 1, &ClientSocket, 0);

		}
		else {
			RCLCPP_ERROR(get_logger(), "Wrong parameter count in CheckFFExist(path)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case CreateDirectory_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlCreateDirectory(SystemControlArgument[0], ControlResponseBuffer, 0);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "CreateDirectory:",
												ControlResponseBuffer, 1, &ClientSocket, 0);

		}
		else {
			RCLCPP_ERROR(get_logger(), "Wrong parameter count in CreateDirectory(path)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case GetRootDirectoryContent_0:
		RCLCPP_INFO(get_logger(), "GetRootDirectory called: defaulting to GetDirectoryContent");
	case GetDirectoryContent_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetDirectoryContent:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
			if (ControlResponseBuffer[0] == FOLDER_EXIST) {
				UtilCreateDirContent((uint8_t*) SystemControlArgument[0],(uint8_t*) "dir.info");
				bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				FileLengthI32 = SystemControlBuildFileContentInfo("dir.info", 0);
				SystemControlFileDownloadResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK,
													"SubGetDirectoryContent:", FileLengthI32, &ClientSocket,
													0);
				SystemControlSendFileContent(&ClientSocket, "dir.info", STR_SYSTEM_CONTROL_TX_PACKET_SIZE,
												SystemControlDirectoryInfo.info_buffer, KEEP_FILE, 0);
				SystemControlDestroyFileContentInfo("dir.info", 1);
			}
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in GetDirectoryContent(path)! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case DeleteTrajectory_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
			*ControlResponseBuffer =
				SystemControlDeleteTrajectory(SystemControlArgument[0],
												sizeof (SystemControlArgument[0]));
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DeleteTrajectory:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in DeleteTrajectory(name)! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case DeleteGeofence_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
			*ControlResponseBuffer =
				SystemControlDeleteGeofence(SystemControlArgument[0], sizeof (SystemControlArgument[0]));
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DeleteGeofence:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in DeleteGeofence(name)! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case DeleteFileDirectory_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
			*ControlResponseBuffer =
				SystemControlDeleteGenericFile(SystemControlArgument[0],
												sizeof (SystemControlArgument[0]));
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DeleteFileDirectory:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in DeleteFileDirectory(path)! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case ClearTrajectories_0:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
			*ControlResponseBuffer = SystemControlClearTrajectories();
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ClearTrajectories:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in ClearTrajectories()! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case ClearGeofences_0:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
			*ControlResponseBuffer = SystemControlClearGeofences();
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ClearGeofences:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in ClearGeofences()! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case ClearObjects_0:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
			*ControlResponseBuffer = SystemControlClearObjects();
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ClearObjects:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in ClearObjects()! got:%d, expected:%d",
						CurrentInputArgCount, CommandArgCount);
			SystemControlCommand = Idle_0;
		}
		break;
	case DownloadFile_1:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DownloadFile:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
			if (ControlResponseBuffer[0] == FILE_EXIST) {
				bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				FileLengthI32 = SystemControlBuildFileContentInfo(SystemControlArgument[0], 0);
				SystemControlFileDownloadResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "SubDownloadFile:",
													FileLengthI32, &ClientSocket, 0);
				SystemControlSendFileContent(&ClientSocket, SystemControlArgument[0],
												STR_SYSTEM_CONTROL_TX_PACKET_SIZE,
												SystemControlDirectoryInfo.info_buffer, KEEP_FILE, 0);
				SystemControlDestroyFileContentInfo(SystemControlArgument[0], 0);
			}

		}
		else {
			RCLCPP_ERROR(get_logger(), "Wrong parameter count in GetDirectoryContent(path)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case DownloadTrajFiles_0:
	case DownloadDirectoryContent_1:	
		if (CurrentInputArgCount == CommandArgCount) {
			char functionReturnName[50];
			memset(functionReturnName, 0, 50);
			memset(ControlResponseBuffer, 0, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			if(SystemControlCommand == DownloadTrajFiles_0){
				strcat(functionReturnName, "DownloadTrajFiles:");
				ControlResponseBuffer[0] = FOLDER_EXIST;
			} else if(SystemControlCommand == DownloadDirectoryContent_1){
				strcat(functionReturnName, "DownloadDirectoryContent:");
				SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
			}
			if(ControlResponseBuffer[0] == FOLDER_EXIST){
				if(SystemControlCommand == DownloadTrajFiles_0){
					UtilCreateDirContent((uint8_t*) MAESTRO_TRAJ_DIRECTORY_STRING,(uint8_t*) "dir.info");
				} else if(SystemControlCommand == DownloadDirectoryContent_1){
					UtilCreateDirContent((uint8_t*) SystemControlArgument[0], (uint8_t*) "dir.info");
				}

				char TestDirectoryPath[MAX_PATH_LENGTH];
				memset(TestDirectoryPath, 0,MAX_PATH_LENGTH);
				char CompletePath[MAX_PATH_LENGTH];
				memset(CompletePath, 0, MAX_PATH_LENGTH);
				char InPath[MAX_PATH_LENGTH];
				memset(InPath, 0, MAX_PATH_LENGTH);

				UtilGetTestDirectoryPath(TestDirectoryPath, sizeof (TestDirectoryPath));
				strcat(CompletePath, TestDirectoryPath);
				strcat(CompletePath,"dir.info");
				int rows = UtilCountFileRowsInPath(CompletePath, strlen(CompletePath));
				char RowBuffer[SMALL_BUFFER_SIZE_128];

				for(int i = 0; i < rows; i ++)
				{
					memset(CompletePath, 0, MAX_PATH_LENGTH);
					strcat(CompletePath, TestDirectoryPath);
					strcat(CompletePath,"dir.info");
					UtilGetRowInFile(CompletePath, strlen(CompletePath), i, RowBuffer, SMALL_BUFFER_SIZE_128);
					if(*RowBuffer == 'F'){
						memset(InPath, 0, MAX_PATH_LENGTH);
						if(SystemControlCommand == DownloadTrajFiles_0) strcat(InPath, MAESTRO_TRAJ_DIRECTORY_STRING);
						else if(SystemControlCommand == DownloadDirectoryContent_1) strcat(InPath, SystemControlArgument[0]);
						strcat(InPath, strstr(RowBuffer, "-")+1);
						memset(ControlResponseBuffer, 0, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
						FileLengthI32 = SystemControlBuildFileContentInfo(InPath, 0);
						SystemControlFileDownloadResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, functionReturnName,
													FileLengthI32, &ClientSocket, 0);
						SystemControlSendFileContent(&ClientSocket, InPath,
												STR_SYSTEM_CONTROL_TX_PACKET_SIZE,
												SystemControlDirectoryInfo.info_buffer, KEEP_FILE, 1);
						SystemControlDestroyFileContentInfo(InPath, 0);
					}

				}
				SystemControlDirectoryInfo.exist = 1; //Force to exist
				SystemControlDestroyFileContentInfo("dir.info", 1);

			} else {
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, functionReturnName,
													ControlResponseBuffer, 3, &ClientSocket, 0);
			}
			SystemControlCommand = Idle_0;
		} else {
			RCLCPP_ERROR(get_logger(), "Wrong parameter count in GetDirectoryContent(path)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case UploadFile_4:
		if (CurrentInputArgCount == CommandArgCount) {
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			bzero(RxFilePath, MAX_FILE_PATH);
			SystemControlUploadFile(SystemControlArgument[0], SystemControlArgument[1],
									SystemControlArgument[2], SystemControlArgument[3],
									ControlResponseBuffer, RxFilePath, 0);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "UploadFile:",
												ControlResponseBuffer, 1, &ClientSocket, 0);
			RCLCPP_DEBUG(get_logger(), "UploadFile filelength: %s", SystemControlArgument[1]);
			if (ControlResponseBuffer[0] == SERVER_PREPARED_BIG_PACKET_SIZE)	//Server is ready to receive data
			{
				RCLCPP_INFO(get_logger(), "Receiving file: %s", SystemControlArgument[0]);
				SystemControlReceiveRxData(&ClientSocket, RxFilePath,
											SystemControlArgument[1], STR_SYSTEM_CONTROL_RX_PACKET_SIZE,
											ControlResponseBuffer, 0);
			}
			else if (ControlResponseBuffer[0] == PATH_INVALID_MISSING) {
				RCLCPP_INFO(get_logger(), "Failed receiving file: %s", SystemControlArgument[0]);
				SystemControlReceiveRxData(&ClientSocket, RxFilePath, SystemControlArgument[1],
											STR_SYSTEM_CONTROL_RX_PACKET_SIZE, ControlResponseBuffer, 0);
				SystemControlDeleteFileDirectory(RxFilePath, ControlResponseBuffer, 0);
				ControlResponseBuffer[0] = PATH_INVALID_MISSING;
			}
			else {
				RCLCPP_INFO(get_logger(), "Receiving file: %s", SystemControlArgument[0]);
				SystemControlReceiveRxData(&ClientSocket, RxFilePath,
											SystemControlArgument[1], SystemControlArgument[2],
											ControlResponseBuffer, 0);
			}

			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "SubUploadFile:",
												ControlResponseBuffer, 1, &ClientSocket, 0);

		}
		else {
			RCLCPP_ERROR(get_logger(),
						"Wrong parameter count in UploadFile(path, filesize, packetsize, filetype)!");
			SystemControlCommand = Idle_0;
		}
		break;
	case InitializeScenario_0:
		if (SystemControlState == SERVER_STATE_IDLE && objectControlState == OBC_STATE_IDLE) {
			try{
				this->initPub.publish(Empty());
			}
			catch(...){
				RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending INIT command");
				SystemControlState = SERVER_STATE_ERROR;

			}
			SystemControlState = SERVER_STATE_INWORK;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "InitializeScenario:",
												ControlResponseBuffer, 0, &ClientSocket, 0);

			SystemControlSendLog("[SystemControl] Sending INIT.\n", &ClientSocket, 0);
		}
		else if (SystemControlState == SERVER_STATE_INWORK && objectControlState == OBC_STATE_INITIALIZED) {
			SystemControlSendLog
				("[SystemControl] Simulate that all objects becomes successfully configured.\n",
					&ClientSocket, 0);
			SystemControlCommand = Idle_0;
			SystemControlState = SERVER_STATE_IDLE;
		}
		else if (SystemControlState == SERVER_STATE_IDLE) {
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE,
												"InitializeScenario:", ControlResponseBuffer, 0,
												&ClientSocket, 0);
			SystemControlSendLog("[SystemControl] INIT received, state errors!\n", &ClientSocket, 0);
			SystemControlCommand = PreviousSystemControlCommand;
		}
		break;
	case ConnectObject_0:
		if (SystemControlState == SERVER_STATE_IDLE && objectControlState == OBC_STATE_INITIALIZED) {
			try{
				this->connectPub.publish(Empty());
			}
			catch(...){
				RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending CONNECT command");
				SystemControlState = SERVER_STATE_ERROR;

			}		
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ConnectObject:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			SystemControlSendLog("[SystemControl] Sending CONNECT.\n", &ClientSocket, 0);
		}
		else if (SystemControlState == SERVER_STATE_INWORK && objectControlState == OBC_STATE_CONNECTED) {
			SystemControlSendLog("[SystemControl] Simulate that all objects are connected.\n",
									&ClientSocket, 0);
			SystemControlCommand = Idle_0;
			SystemControlState = SERVER_STATE_IDLE;
		}
		else if (SystemControlState == SERVER_STATE_IDLE) {
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE,
												"ConnectObject:", ControlResponseBuffer, 0, &ClientSocket,
												0);
			SystemControlSendLog("[SystemControl] CONNECT received, state errors!\n", &ClientSocket, 0);
			SystemControlCommand = PreviousSystemControlCommand;
		}
		break;
	case DisconnectObject_0:
		if (SystemControlState == SERVER_STATE_IDLE) {
			try{
				this->disconnectPub.publish(Empty());
			}
			catch(...){
				RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending DISCONNECT command");
				SystemControlState = SERVER_STATE_ERROR;				
			}
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DisconnectObject:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			SystemControlSendLog("[SystemControl] Sending DISCONNECT.\n", &ClientSocket, 0);
		}
		else {
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE,
												"ConnectObject:", ControlResponseBuffer, 0, &ClientSocket,
												0);
			SystemControlSendLog("[SystemControl] DISCONNECT received, state errors!\n", &ClientSocket,
									0);
			SystemControlCommand = PreviousSystemControlCommand;
		}
		break;
	case ArmScenario_0:
		if (SystemControlState == SERVER_STATE_IDLE && objectControlState == OBC_STATE_CONNECTED) {
			SystemControlState = SERVER_STATE_INWORK;
			try{
				this->armPub.publish(Empty());
			}
			catch(...){
				RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending ARM command");
				SystemControlState = SERVER_STATE_ERROR;				
			}
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ArmScenario:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			SystemControlSendLog("[SystemControl] Sending ARM.\n", &ClientSocket, 0);
		}
		else if (SystemControlState == SERVER_STATE_INWORK && objectControlState == OBC_STATE_ARMED) {
			SystemControlSendLog("[SystemControl] Simulate that all objects become armed.\n",
									&ClientSocket, 0);

			SystemControlCommand = Idle_0;
			SystemControlState = SERVER_STATE_IDLE;
		}
		else if (SystemControlState == SERVER_STATE_IDLE) {
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE,
												"ArmScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
			SystemControlSendLog("[SystemControl] ARM received, state errors!\n", &ClientSocket, 0);
			SystemControlCommand = PreviousSystemControlCommand;
		}
		break;
	case RemoteControl_1:
		responseCode = SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE;
		if (CurrentInputArgCount == CommandArgCount) {
			if (SystemControlState == SERVER_STATE_IDLE
				&& (objectControlState == OBC_STATE_CONNECTED
					|| objectControlState == OBC_STATE_REMOTECTRL)) {
				if (!strcasecmp(SystemControlArgument[0], ENABLE_COMMAND_STRING)
					&& objectControlState == OBC_STATE_CONNECTED) {
					RCLCPP_INFO(get_logger(), "Requesting enabling of remote control");
					this->remoteControlEnablePub.publish(Empty());
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
				}
				else if (!strcasecmp(SystemControlArgument[0], DISABLE_COMMAND_STRING)
							&& objectControlState == OBC_STATE_REMOTECTRL) {
					RCLCPP_INFO(get_logger(), "Requesting disabling of remote control");
					this->remoteControlDisablePub.publish(Empty());
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
				}
				else {
					RCLCPP_WARN(get_logger(), "Incorrect remote control command");
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
				}
			}
			else {
				responseCode = SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE;
			}

		}
		else {
			RCLCPP_WARN(get_logger(), "Remote control command parameter count error");
			responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
		}
		SystemControlCommand = Idle_0;
		SystemControlSendControlResponse(responseCode, "RemoteControl:",
											ControlResponseBuffer, 0, &ClientSocket, 0);
		break;
	case RemoteControlManoeuvre_2:
		if (CurrentInputArgCount == CommandArgCount) {
			if (SystemControlState == SERVER_STATE_IDLE && objectControlState == OBC_STATE_REMOTECTRL) {
				auto msg = ManoeuvreCommand();
				if (inet_pton(AF_INET, SystemControlArgument[0], &msg.object_ip) != -1) {
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
					switch (atoi(SystemControlArgument[1])) {
					case MSCP_BACK_TO_START:
						msg.manoeuvre = MANOEUVRE_BACK_TO_START;
						break;
					default:
						responseCode = SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE;
					}
					if (responseCode != SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE) {
						this->backToStartPub.publish(msg);
						responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
					}
				}
				else {
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
				}
			}
			else {
				responseCode = SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE;
			}
		}
		else {
			RCLCPP_WARN(get_logger(), "Remote control manoeuvre command parameter count error");
			responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
		}
		SystemControlCommand = Idle_0;
		SystemControlSendControlResponse(responseCode, "RemoteControlManoeuvre:",
											ControlResponseBuffer, 0, &ClientSocket, 0);
		break;
	case SetObjectEnableStatus_2:
		if (CurrentInputArgCount == CommandArgCount) {
			if (SystemControlState == SERVER_STATE_IDLE && objectControlState != OBC_STATE_RUNNING) {
				auto msg = ObjectEnabled();
				if (inet_pton(AF_INET, SystemControlArgument[0], &(msg.object_ip)) != -1) {
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
					switch (atoi(SystemControlArgument[1])) {
					case OBJECT_ENABLED:
						msg.enabled = OBJECT_ENABLED;
						break;
					case OBJECT_DISABLED:
						msg.enabled = OBJECT_DISABLED;
						break;
					default:
						responseCode = SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE;
					}
					if (responseCode != SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE) {
						this->enableObjectPub.publish(msg);
						responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
					}
				}
				else {
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
				}
			}
			else {
				responseCode = SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE;
			}
		}
		else {
			RCLCPP_WARN(get_logger(), "SetObjectEnableStatus command parameter count error");
			responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
		}
		SystemControlCommand = Idle_0;
		SystemControlSendControlResponse(responseCode, "SetObjectEnableStatus:",
											ControlResponseBuffer, 0, &ClientSocket, 0);
		break;
	case GetObjectEnableStatus_1:
		if (CurrentInputArgCount == CommandArgCount) {
			if (SystemControlState == SERVER_STATE_IDLE) {
				memset(pcBuffer, 0, sizeof (pcBuffer));
				ObjectEnabledCommandType enableCommand;

				if (inet_pton(AF_INET, SystemControlArgument[0], &enableCommand.objectIP) != -1) {
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_OK;
					DataDictionaryGetObjectEnableStatusByIp(enableCommand.objectIP,
															&enableCommand.Enabled);
					memset(ControlResponseBuffer, 0, sizeof (ControlResponseBuffer));
					ControlResponseBuffer[0] = (uint8_t) enableCommand.Enabled;
					SystemControlSendControlResponse(responseCode, "GetObjectEnableStatus:",
														ControlResponseBuffer, 1, &ClientSocket, 0);
				}
				else
					responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
			}
			else
				responseCode = SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE;
		}
		else {
			RCLCPP_WARN(get_logger(), "GetObjectEnableStatus command parameter count error");
			responseCode = SYSTEM_CONTROL_RESPONSE_CODE_ERROR;
		}

		if (responseCode != SYSTEM_CONTROL_RESPONSE_CODE_OK)
			SystemControlSendControlResponse(responseCode, "GetObjectEnableStatus:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
		SystemControlCommand = Idle_0;
		break;
	case StartScenario_1:
		if (CurrentInputArgCount == CommandArgCount) {
			if (SystemControlState == SERVER_STATE_IDLE && objectControlState == OBC_STATE_ARMED)	//Temporary!
			{
				bzero(pcBuffer, IPC_BUFFER_SIZE);
				TimeSetToCurrentSystemTime(&tvTime);

				if (TIME_COMPENSATE_LAGING_VM)
					timersub(&tvTime, &VirtualMachineLagCompensation, &tvTime);

				RCLCPP_INFO(get_logger(), "Current timestamp (epoch): %lu", TimeGetAsUTCms(&tvTime));

				DelayedStartU32 = atoi(SystemControlArgument[0]);
				sprintf(pcBuffer, "%" PRIi64 ";%" PRIu32 ";", TimeGetAsUTCms(&tvTime), DelayedStartU32);
				RCLCPP_INFO(get_logger(), "Sending START <%s> (delayed +%u ms)", pcBuffer,
							DelayedStartU32);
				try{
					this->startPub.publish(Empty());
				}
				catch(...){
					RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending STRT command");
					SystemControlState = SERVER_STATE_ERROR;				
				}
				bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "StartScenario:",
													ControlResponseBuffer, 0, &ClientSocket, 0);
				SystemControlState = SERVER_STATE_INWORK;
				//SystemControlState = SERVER_STATE_IDLE; //Temporary!
				//SystemControlCommand = Idle_0; //Temporary!
			}
			else if (SystemControlState == SERVER_STATE_INWORK && objectControlState == OBC_STATE_RUNNING) {

				SystemControlCommand = Idle_0;
				SystemControlState = SERVER_STATE_IDLE;
			}
			else if (SystemControlState == SERVER_STATE_IDLE) {
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE,
													"StartScenario:", ControlResponseBuffer, 0,
													&ClientSocket, 0);
				SystemControlSendLog("[SystemControl] START received, state errors!\n", &ClientSocket, 0);
				SystemControlCommand = PreviousSystemControlCommand;
			}

		}
		else
			RCLCPP_WARN(get_logger(), "START command parameter count error");
		break;
	case stop_0:
		try{
			this->stopPub.publish(Empty());
			SystemControlState = SERVER_STATE_IDLE;
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "stop:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
		}
		catch(...){this->disconnectPub.publish(Empty());
			RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending DISCONSTOPNECTSTOP command");
			SystemControlState = SERVER_STATE_ERROR;				
		}
		break;
	case AbortScenario_0:
		try{
			this->abortPub.publish(Empty());
			SystemControlState = SERVER_STATE_IDLE;
			SystemControlCommand = Idle_0;
			if (ClientSocket >= 0) {
				bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				ControlResponseBuffer[0] = 1;
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "AbortScenario:",
													ControlResponseBuffer, 1, &ClientSocket, 0);
			}
		}
		catch(...){
			RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending ABORT command");
			SystemControlState = SERVER_STATE_ERROR;
		}
	break;
	case ClearAllScenario_0:
		try{
			this->allClearPub.publish(Empty());
			SystemControlState = SERVER_STATE_IDLE;
			SystemControlCommand = Idle_0;
			if (ClientSocket >= 0) {
				bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				ControlResponseBuffer[0] = 1;
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ClearAllScenario:",
													ControlResponseBuffer, 1, &ClientSocket, 0);
			}
		}
		catch (...){
			RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending COMM_ABORT_DONE command");
			SystemControlState = SERVER_STATE_ERROR;
		}
	break;
	case Exit_0:
		try{
			this->exitPub.publish(Empty());
			iExit = 1;
			usleep(1000000);
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "Exit:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			close(ClientSocket);
			ClientSocket = -1;
			if (USE_LOCAL_USER_CONTROL == 0) {
				close(ServerHandle);
				ServerHandle = -1;
			}
			RCLCPP_INFO(get_logger(), "Server closing");
		}
		catch(...){
			RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending EXIT command");
			SystemControlState = SERVER_STATE_ERROR;

		}
		/*
		if (iCommSend(COMM_EXIT, NULL, 0) < 0) {
			RCLCPP_ERROR(get_logger(), "Fatal communication fault when sending EXIT command");
			SystemControlState = SERVER_STATE_ERROR;
		}
		else {
			iExit = 1;
			GSD->ExitU8 = 1;
			usleep(1000000);
			SystemControlCommand = Idle_0;
			bzero(ControlResponseBuffer, SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "Exit:",
												ControlResponseBuffer, 0, &ClientSocket, 0);
			close(ClientSocket);
			ClientSocket = -1;
			if (USE_LOCAL_USER_CONTROL == 0) {
				close(ServerHandle);
				ServerHandle = -1;
			}
			RCLCPP_INFO(get_logger(), "Server closing");
		}*/
		break;

	default:

		break;
	}
}

void SystemControl::pollModuleStatus() {
	static auto nextPollTime = std::chrono::steady_clock::now();
	constexpr auto pollPeriod = std::chrono::seconds(1);
	if (std::chrono::steady_clock::now() > nextPollTime) {
		getStatusPub.publish(ROSChannels::GetStatus::message_type());
		nextPollTime = std::chrono::steady_clock::now() + pollPeriod;
	}
}

SystemControl::SystemControlCommand_t SystemControl::SystemControlFindCommand(const char *CommandBuffer,
												SystemControl::SystemControlCommand_t * CurrentCommand,
												int *CommandArgCount) {

	SystemControl::SystemControlCommand_t command;
	char StrippedCommandBuffer[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];

	bzero(StrippedCommandBuffer, SYSTEM_CONTROL_COMMAND_MAX_LENGTH);
	//printf("CommandBuffer: %s\n", CommandBuffer);
	strncpy(StrippedCommandBuffer, CommandBuffer,
			(uint64_t) strchr(CommandBuffer, '(') - (uint64_t) CommandBuffer);
	//printf("StrippedCommandBuffer: %s\n", StrippedCommandBuffer);

	for (command = Idle_0; command != nocommand; command=(SystemControlCommand_t)(command+1)) {
		bzero(SystemControl::SystemControlCommandArgCnt, SYSTEM_CONTROL_ARG_CHAR_COUNT);
		bzero(SystemControl::SystemControlStrippedCommand, SYSTEM_CONTROL_COMMAND_MAX_LENGTH);
		strncpy(SystemControlStrippedCommand, SystemControlCommandsArr[(int)command],
				(uint64_t) strchr(SystemControlCommandsArr[(int)command],
								  '_') - (uint64_t) SystemControlCommandsArr[(int)command]);
		strncpy(SystemControlCommandArgCnt, strchr(SystemControlCommandsArr[(int)command], '_') + 1,
				strlen(SystemControlCommandsArr[(int)command]) -
				((uint64_t) strchr(SystemControlCommandsArr[(int)command], '_') -
				 (uint64_t) SystemControlCommandsArr[(int)command] + 1));

		if (!strcmp(SystemControlStrippedCommand, StrippedCommandBuffer)) {
			{
				*CommandArgCount = atoi(SystemControlCommandArgCnt);
				*CurrentCommand = command;
				return command;
			}
		}
	}
	return nocommand;
}


/*!
 * \brief SystemControlReceiveUserControlData Performs similarly to the recv function (see manpage for recv) except that it
 *        only fills the input data buffer with messages ending with ";\r\n\r\n" and saves any remaining data in a local
 *        buffer awaiting the next call to this function.
 * \param socket Socket on which MSCP HTTP communication is expected to arrive
 * \param dataBuffer Data buffer where read data is to be stored
 * \param dataBufferLength Maximum number of bytes possible to store in the data buffer
 * \return Number of bytes printed to dataBuffer where 0 means that the connection has been severed. A return value of -1
 *        constitutes an error with the appropriate errno has been set (see manpage for recv) with the addition of
 *         - ENOBUFS if the data buffer is too small to hold the received message
 */
ssize_t SystemControl::SystemControlReceiveUserControlData(I32 socket, char * dataBuffer, size_t dataBufferLength) {
	static char recvBuffer[TCP_RECV_BUFFER_SIZE];
	static size_t bytesInBuffer = 0;
	const char endOfMessagePattern[] = ";\r\n\r\n";
	char *endOfMessage = NULL;
	ssize_t readResult;
	size_t messageLength = 0;

	readResult = recv(socket, recvBuffer + bytesInBuffer, sizeof (recvBuffer) - bytesInBuffer, MSG_DONTWAIT);
	if (readResult > 0) {
		bytesInBuffer += (size_t)readResult;
	}

	if (bytesInBuffer > 0) {
		if ((endOfMessage = strstr(recvBuffer, endOfMessagePattern)) != NULL) {
			endOfMessage += sizeof (endOfMessagePattern) - 1;
			messageLength = (size_t)(endOfMessage - recvBuffer);
		}
		else {
			messageLength = 0;
			readResult = -1;
			errno = EAGAIN;
			RCLCPP_WARN(get_logger(), "Part of message received");
		}

		if (bytesInBuffer >= messageLength) {
			if (dataBufferLength < messageLength) {
				RCLCPP_WARN(get_logger(), "Discarding message too large for data buffer");
				readResult = -1;
				errno = ENOBUFS;
			}
			else {
				memcpy(dataBuffer, recvBuffer, messageLength);
				readResult = (ssize_t) messageLength;
			}
			bytesInBuffer -= messageLength;
			memmove(recvBuffer, recvBuffer + messageLength, bytesInBuffer);
		}
	}

	return readResult;
}

void SystemControl::SystemControlSendMONR(const char * MONRStr, I32 * Sockfd, U8 Debug) {
	int i, n, j, t;
	char Length[4];
	char Header[2] = { 0, 2 };
	char Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = 2 + strlen(MONRStr);
	Length[0] = (char) (n >> 24);
	Length[1] = (char) (n >> 16);
	Length[2] = (char) (n >> 8);
	Length[3] = (char) n;


	if (n + 4 < SYSTEM_CONTROL_SEND_BUFFER_SIZE) {
		for (i = 0, j = 0; i < 4; i++, j++)
			Data[j] = Length[i];
		for (i = 0; i < 2; i++, j++)
			Data[j] = Header[i];
		t = strlen(MONRStr);
		for (i = 0; i < t; i++, j++)
			Data[j] = *(MONRStr + i);
		//SystemControlSendBytes(Data, n + 4, Sockfd, 0);
		UtilSendTCPData((uint8_t*) "System Control", (uint8_t*) Data, n + 4, Sockfd, 0);
	}
	else
		RCLCPP_ERROR(get_logger(), "MONR string longer than %d bytes!", SYSTEM_CONTROL_SEND_BUFFER_SIZE);
}


void SystemControl::SystemControlSendLog(const char * LogString, I32 * Sockfd, U8 Debug) {
	int i, n, j, t;
	char Length[4];
	char Header[2] = { 0, 2 };
	char Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = 2 + strlen(LogString);
	Length[0] = (char) (n >> 24);
	Length[1] = (char) (n >> 16);
	Length[2] = (char) (n >> 8);
	Length[3] = (char) n;

	//SystemControlSendBytes(Length, 4, Sockfd, 0);
	//SystemControlSendBytes(Header, 5, Sockfd, 0);
	//SystemControlSendBytes(LogString, strlen(LogString), Sockfd, 0);


	if (n + 4 < SYSTEM_CONTROL_SEND_BUFFER_SIZE) {
		for (i = 0, j = 0; i < 4; i++, j++)
			Data[j] = Length[i];
		for (i = 0; i < 2; i++, j++)
			Data[j] = Header[i];
		t = strlen(LogString);
		for (i = 0; i < t; i++, j++)
			Data[j] = *(LogString + i);
		//SystemControlSendBytes(Data, n + 4, Sockfd, 0);
		UtilSendTCPData((uint8_t*) "System Control", (uint8_t*) Data, n + 4, Sockfd, 0);
	}
	else
		RCLCPP_ERROR(get_logger(), "Log string longer than %d bytes!", SYSTEM_CONTROL_SEND_BUFFER_SIZE);

}


void SystemControl::SystemControlSendControlResponse(U16 ResponseStatus, const char * ResponseString, const char * ResponseData,
									  I32 ResponseDataLength, I32 * Sockfd, U8 Debug) {
	int i, n, j, t;
	char Length[4];
	char Status[2];
	char Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = 2 + strlen(ResponseString) + ResponseDataLength;
	Length[0] = (char) (n >> 24);
	Length[1] = (char) (n >> 16);
	Length[2] = (char) (n >> 8);
	Length[3] = (char) n;
	Status[0] = (char) (ResponseStatus >> 8);
	Status[1] = (char) ResponseStatus;

	if (n + 4 < SYSTEM_CONTROL_SEND_BUFFER_SIZE) {
		for (i = 0, j = 0; i < 4; i++, j++)
			Data[j] = Length[i];
		for (i = 0; i < 2; i++, j++)
			Data[j] = Status[i];
		t = strlen(ResponseString);
		for (i = 0; i < t; i++, j++)
			Data[j] = *(ResponseString + i);
		for (i = 0; i < ResponseDataLength; i++, j++)
			Data[j] = ResponseData[i];

		if (Debug) {
			for (i = 0; i < n + 4; i++)
				printf("%x-", Data[i]);
			printf("\n");
		}

		//SystemControlSendBytes(Data, n + 4, Sockfd, 0);
		UtilSendTCPData((uint8_t*) "System Control", (uint8_t*) Data, n + 4, Sockfd, 0);
	}
	else
		RCLCPP_ERROR(get_logger(), "Response data more than %d bytes!", SYSTEM_CONTROL_SEND_BUFFER_SIZE);
}


void SystemControl::SystemControlFileDownloadResponse(U16 ResponseStatus, const char * ResponseString,
									   I32 ResponseDataLength, I32 * Sockfd, U8 Debug) {
	int i, n, j, t;
	char Length[MSCP_RESPONSE_DATALENGTH_BYTES];
	char Status[MSCP_RESPONSE_STATUS_CODE_BYTES];
	char Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = MSCP_RESPONSE_STATUS_CODE_BYTES + strlen(ResponseString) + ResponseDataLength;
	Length[0] = (char) (n >> 24);
	Length[1] = (char) (n >> 16);
	Length[2] = (char) (n >> 8);
	Length[3] = (char) n;
	Status[0] = (char) (ResponseStatus >> 8);
	Status[1] = (char) ResponseStatus;

	for (i = 0, j = 0; i < MSCP_RESPONSE_DATALENGTH_BYTES; i++, j++)
		Data[j] = Length[i];
	for (i = 0; i < MSCP_RESPONSE_STATUS_CODE_BYTES; i++, j++)
		Data[j] = Status[i];
	t = strlen(ResponseString);
	for (i = 0; i < t; i++, j++)
		Data[j] = *(ResponseString + i);

	if (Debug) {
		for (i = 0; i < n + MSCP_RESPONSE_DATALENGTH_BYTES; i++)
			printf("%x-", Data[i]);
		printf("\n");
	}

	UtilSendTCPData((uint8_t*) "System Control", (uint8_t*) Data,
					MSCP_RESPONSE_DATALENGTH_BYTES + MSCP_RESPONSE_STATUS_CODE_BYTES +
					strlen(ResponseString), Sockfd, 0);
}



I32 SystemControl::SystemControlBuildControlResponse(U16 ResponseStatus, char * ResponseString, char * ResponseData,
									  I32 ResponseDataLength, U8 Debug) {
	int i = 0, n = 0, j = 0, t = 0;
	char Length[MSCP_RESPONSE_DATALENGTH_BYTES];
	char Status[MSCP_RESPONSE_STATUS_CODE_BYTES];
	char Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = MSCP_RESPONSE_STATUS_CODE_BYTES + strlen(ResponseString) + ResponseDataLength;
	Length[0] = (char) (n >> 24);
	Length[1] = (char) (n >> 16);
	Length[2] = (char) (n >> 8);
	Length[3] = (char) n;
	Status[0] = (char) (ResponseStatus >> 8);
	Status[1] = (char) ResponseStatus;

	if (n + MSCP_RESPONSE_DATALENGTH_BYTES < SYSTEM_CONTROL_SEND_BUFFER_SIZE) {
		for (i = 0, j = 0; i < MSCP_RESPONSE_DATALENGTH_BYTES; i++, j++)
			Data[j] = Length[i];
		for (i = 0; i < MSCP_RESPONSE_STATUS_CODE_BYTES; i++, j++)
			Data[j] = Status[i];
		t = strlen(ResponseString);
		for (i = 0; i < t; i++, j++)
			Data[j] = *(ResponseString + i);
		for (i = 0; i < ResponseDataLength; i++, j++)
			Data[j] = ResponseData[i];

		for (i = 0; i < n; i++)
			*(ResponseData + i) = Data[i];	//Copy back

		if (Debug) {
			for (i = 0; i < n + MSCP_RESPONSE_DATALENGTH_BYTES; i++)
				printf("%x-", Data[i]);
			printf("\n");
		}

	}
	else
		RCLCPP_ERROR(get_logger(), "Response data more than %d bytes!", SYSTEM_CONTROL_SEND_BUFFER_SIZE);




	return n;
}


void SystemControl::SystemControlSendBytes(const char *data, int length, int *sockfd, int debug) {
	int i, n;

	if (debug == 1) {
		printf("Bytes sent: ");
		int i = 0;

		for (i = 0; i < length; i++)
			printf("%d ", (char) * (data + i));
		printf("\n");
	}

	n = write(*sockfd, data, length);
	if (n < 0) {
		util_error("[SystemControl] ERR: Failed to send on control socket");
	}
}



I32 SystemControl::SystemControlInitServer(int *ClientSocket, int *ServerHandle, struct in_addr *ip_addr) {

	struct sockaddr_in command_server_addr;
	struct sockaddr_in cli_addr;
	socklen_t cli_length;
	unsigned int control_port = SYSTEM_CONTROL_CONTROL_PORT;
	int optval = 1;
	int result = 0;
	int sockFlags = 0;

	enum COMMAND iCommand;
	ssize_t bytesReceived = 0;


	/* Init user control socket */
	RCLCPP_INFO(get_logger(), "Init control socket");

	*ServerHandle = socket(AF_INET, SOCK_STREAM, 0);
	if (*ServerHandle < 0) {
		perror("[SystemControl] ERR: Failed to create control socket");
		::exit(1);
	}

	bzero((char *)&command_server_addr, sizeof (command_server_addr));

	command_server_addr.sin_family = AF_INET;
	command_server_addr.sin_addr.s_addr = INADDR_ANY;
	command_server_addr.sin_port = htons(control_port);

	optval = 1;
	result = setsockopt(*ServerHandle, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

	if (result < 0) {
		perror("[SystemControl] ERR: Failed to call setsockopt");
		::exit(1);
	}

	if (bind(*ServerHandle, (struct sockaddr *)&command_server_addr, sizeof (command_server_addr)) < 0) {
		perror("[SystemControl] ERR: Failed to bind to control socket");
		::exit(1);
	}

	/* Monitor and control sockets up. Wait for central to connect to control socket to get server address */
	RCLCPP_INFO(get_logger(), "Listening for connection from client...");

	listen(*ServerHandle, 1);
	cli_length = sizeof (cli_addr);

	/* Set socket to nonblocking */
	sockFlags = fcntl(*ServerHandle, F_GETFL, 0);
	if (sockFlags == -1)
		util_error("Error calling fcntl");

	sockFlags = sockFlags | O_NONBLOCK;
	if (fcntl(*ServerHandle, F_SETFL, sockFlags))
		util_error("Error calling fcntl");

	do {
		*ClientSocket = accept(*ServerHandle, (struct sockaddr *)&cli_addr, &cli_length);
		if ((*ClientSocket == -1 && errno != EAGAIN && errno != EWOULDBLOCK) || iExit)
			util_error("Failed to establish connection");
		// Varför läser man bus-meddelanden här? Tömmer den bara sin queue?
		//bytesReceived = iCommRecv(&iCommand, pcRecvBuffer, SC_RECV_MESSAGE_BUFFER, NULL);
	} while (*ClientSocket == -1);

	RCLCPP_INFO(get_logger(), "Connection established: %s:%i", inet_ntoa(cli_addr.sin_addr),
			   htons(command_server_addr.sin_port));

	ip_addr->s_addr = cli_addr.sin_addr.s_addr;	//Set IP-address of Usercontrol

	if (*ClientSocket < 0) {
		perror("[SystemControl] ERR: Failed to accept from central");
		::exit(1);
	}

	return result;
}



I32 SystemControl::SystemControlConnectServer(int *sockfd, const char *name, const uint32_t port) {
	struct sockaddr_in serv_addr;
	struct hostent *server;

	char buffer[256];
	int iResult;

	*sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (*sockfd < 0) {
		util_error("[SystemControl] ERR: Failed to open control socket");
	}

	server = gethostbyname(name);
	if (server == NULL) {
		util_error("[SystemControl] ERR: Unknown host ");
	}

	bzero((char *)&serv_addr, sizeof (serv_addr));
	serv_addr.sin_family = AF_INET;

	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(port);

	RCLCPP_INFO(get_logger(), "Attempting to connect to control socket: %s:%i", name, port);

	do {
		iResult = ::connect(*sockfd, (struct sockaddr *)&serv_addr, sizeof (serv_addr));

		if (iResult < 0) {
			if (errno == ECONNREFUSED) {
				RCLCPP_WARN(get_logger(), "Unable to connect to UserControl, retrying in 3 sec...");
				(void)sleep(3);
			}
			else {
				util_error("[SystemControl] ERR: Failed to connect to control socket");
			}
		}
	} while (iResult < 0);


	iResult = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);

	RCLCPP_DEBUG(get_logger(), "Maestro connected to UserControl: %s:%i", name, port);
	return iResult;

}


void SystemControl::SystemControlCreateProcessChannel(const char * name, const U32 port, I32 * sockfd,
											  struct sockaddr_in *addr) {
	int result;
	struct hostent *object;

	/* Connect to object safety socket */

	RCLCPP_DEBUG(get_logger(), "Creating process channel socket");

	*sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (*sockfd < 0) {
		util_error("[SystemControl] ERR: Failed to connect to process channel socket");
	}

	/* Set address to object */
	object = gethostbyname(name);

	if (object == 0) {
		util_error("[SystemControl] ERR: Unknown host");
	}

	bcopy((char *)object->h_addr, (char *)&addr->sin_addr.s_addr, object->h_length);
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);

	/* set socket to non-blocking */
	result = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
	if (result < 0) {
		util_error("[SystemControl] ERR: calling fcntl");
	}

	RCLCPP_INFO(get_logger(), "Created process channel socket and address: %s:%d", name, port);
}

/*!
 * \brief SystemControlVerifyHostAddress Checks if addr matches any of Maesto's own ip addresses
 * \param addr IP address to match against own IPs
 * \return true if match, false if not
 */
char SystemControl::SystemControlVerifyHostAddress(char *addr) {
	struct ifaddrs *ifaddr, *ifa;
	int family, s, n;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1) {
		RCLCPP_ERROR(get_logger(), "Could not get interface data");
		freeifaddrs(ifaddr);
		return 0;
	}

	// Iterate over linked list using ifa
	for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
		if (ifa->ifa_addr == NULL) {
			// Interface had no addresses, skip to next
			continue;
		}

		family = ifa->ifa_addr->sa_family;
		if (family == AF_INET || family == AF_INET6) {
			s = getnameinfo(ifa->ifa_addr,
							(family == AF_INET) ? sizeof (struct sockaddr_in) :
							sizeof (struct sockaddr_in6), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
			if (s != 0) {
				RCLCPP_ERROR(get_logger(), "getnameinfo() failed: %s", gai_strerror(s));
				continue;
			}

			if (strcmp(host, addr) == 0)
				return 1;
			else
				continue;
		}
		else {
			continue;
		}
	}

	freeifaddrs(ifaddr);
	return 0;
}


I32 SystemControl::SystemControlGetServerParameter(char * ParameterName, char * ReturnValue, U32 BufferLength,
									U8 Debug) {
	bzero(ReturnValue, 20);
	dbl ValueDbl = 0;
	U32 ValueU32 = 0;
	U16 ValueU16 = 0;
	U8 ValueU8 = 0;

	sprintf(ReturnValue, "%s", ParameterName);
	strcat(ReturnValue, ": ");
	BufferLength = BufferLength - strlen(ReturnValue);

	if (strcmp("OriginLatitude", ParameterName) == 0) {
		DataDictionaryGetOriginLatitudeDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.12f", ValueDbl);
	}
	else if (strcmp("OriginLongitude", ParameterName) == 0) {
		DataDictionaryGetOriginLongitudeDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.12f", ValueDbl);
	}
	else if (strcmp("OriginAltitude", ParameterName) == 0) {
		DataDictionaryGetOriginAltitudeDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.12f", ValueDbl);
	}
	else if (strcmp("VisualizationServerName", ParameterName) == 0) {
		DataDictionaryGetVisualizationServerIPString(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("ASPMaxTimeDiff", ParameterName) == 0) {
		DataDictionaryGetASPMaxTimeDiffDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.3f", ValueDbl);
	}
	else if (strcmp("ASPMaxTrajDiff", ParameterName) == 0) {
		DataDictionaryGetASPMaxTrajDiffDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.3f", ValueDbl);
	}
	else if (strcmp("ASPStepBackCount", ParameterName) == 0) {
		DataDictionaryGetASPStepBackCountU32(&ValueU32);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu32, ValueU32);
	}
	else if (strcmp("ASPFilterLevel", ParameterName) == 0) {
		DataDictionaryGetASPFilterLevelDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.3f", ValueDbl);
	}
	else if (strcmp("ASPMaxDeltaTime", ParameterName) == 0) {
		DataDictionaryGetASPMaxDeltaTimeDbl(&ValueDbl);
		sprintf(ReturnValue + strlen(ReturnValue), "%3.3f", ValueDbl);
	}
	else if (strcmp("TimeServerIP", ParameterName) == 0) {
		DataDictionaryGetTimeServerIPString(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("TimeServerPort", ParameterName) == 0) {
		DataDictionaryGetTimeServerPortU16(&ValueU16);
		sprintf(ReturnValue, "%" PRIu16, ValueU16);
	}
	else if (strcmp("SimulatorIP", ParameterName) == 0) {
		DataDictionaryGetSimulatorIPString(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("SimulatorTCPPort", ParameterName) == 0) {
		DataDictionaryGetSimulatorTCPPortU16(&ValueU16);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu16, ValueU16);
	}
	else if (strcmp("SimulatorUDPPort", ParameterName) == 0) {
		DataDictionaryGetSimulatorUDPPortU16(&ValueU16);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu16, ValueU16);
	}
	else if (strcmp("SimulatorMode", ParameterName) == 0) {
		DataDictionaryGetSimulatorModeU8(&ValueU8);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu8, ValueU8);
	}
	else if (strcmp("VOILReceivers", ParameterName) == 0) {
		DataDictionaryGetVOILReceiversString(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("DTMReceivers", ParameterName) == 0) {
		DataDictionaryGetDTMReceiversString(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("SupervisorIP", ParameterName) == 0) {
		DataDictionaryGetExternalSupervisorIPString(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("SupervisorTCPPort", ParameterName) == 0) {
		DataDictionaryGetSupervisorTCPPortU16(&ValueU16);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu16, ValueU16);
	}
	else if (strcmp("MiscData", ParameterName) == 0) {
		DataDictionaryGetMiscData(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	else if (strcmp("RVSSConfig", ParameterName) == 0) {
		DataDictionaryGetRVSSConfigU32(&ValueU32);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu32, ValueU32);
	}
	else if (strcmp("RVSSRate", ParameterName) == 0) {
		DataDictionaryGetRVSSRateU8(&ValueU8);
		sprintf(ReturnValue + strlen(ReturnValue), "%" PRIu8, ValueU8);
	}
	else if (strcmp("ScenarioName", ParameterName) == 0) {
		DataDictionaryGetScenarioName(ReturnValue + strlen(ReturnValue), BufferLength);
	}
	return 0;
}



I32 SystemControl::SystemControlSetServerParameter(char * parameterName, char * newValue, U8 debug) {

	ReadWriteAccess_t result = PARAMETER_NOTFOUND;
	U32 object_transmitter_ids[MAX_OBJECTS];
	U32 numberOfObjects;
	GeoPosition origin;
	char *endptr;

	if (parameterName == NULL || newValue == NULL) {
		RCLCPP_ERROR(get_logger(), "Set server parameter null pointer error");
		return -1;
	}

	if (debug) {
		LogPrint("SetServerParameter: %s = %s", parameterName, newValue);
	}

	enum ConfigurationFileParameter parameter =
		UtilParseConfigurationParameter(parameterName, strlen(parameterName) + 1);

	switch (parameter) {
	case CONFIGURATION_PARAMETER_SCENARIO_NAME:
		result = DataDictionarySetScenarioName(newValue, strlen(newValue) + 1);
		break;
	case CONFIGURATION_PARAMETER_ORIGIN_LATITUDE:
		if ((result = DataDictionaryGetNumberOfObjects(&numberOfObjects)) != READ_OK) {
			RCLCPP_ERROR(get_logger(), "Data dictionary number of objects read error"
					   "- cannot set origin on objects");
			break;
		}

		if ((result =
			 DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, numberOfObjects)) != READ_OK) {
			RCLCPP_ERROR(get_logger(),
					   "Data dictionary TransmitterID read error" "- cannot set origin on objects");
			break;
		}

		for (unsigned int i = 0; i < numberOfObjects; ++i) {
			if (object_transmitter_ids[i] == 0) {
				continue;
			}
			else {
				if (DataDictionaryGetOrigin(object_transmitter_ids[i], &origin) != READ_OK) {
					RCLCPP_ERROR(get_logger(), "Data dictionary origin read error"
							   "- cannot read the origin for object %u", object_transmitter_ids[i]);
					result = UNDEFINED;
					continue;
				}
				origin.Latitude = strtod(newValue, &endptr);
				if (endptr == newValue) {
					RCLCPP_ERROR(get_logger(), "Unable to convert origin setting %s to double", newValue);
					result = UNDEFINED;
					break;
				}
				if (DataDictionarySetOrigin(&object_transmitter_ids[i], &origin) != WRITE_OK) {
					RCLCPP_ERROR(get_logger(), "Data dictionary origin write error"
							   "- cannot set the origin for object %u", object_transmitter_ids[i]);
					result = UNDEFINED;
					break;
				}
			}
		}

		// TODO remove
		result = DataDictionarySetOriginLatitudeDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE:
		if ((result = DataDictionaryGetNumberOfObjects(&numberOfObjects)) != READ_OK) {
			RCLCPP_ERROR(get_logger(), "Data dictionary number of objects read error"
					   "- cannot set origin on objects");
			break;
		}

		if ((result =
			 DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, numberOfObjects)) != READ_OK) {
			RCLCPP_ERROR(get_logger(),
					   "Data dictionary TransmitterID read error" "- cannot set origin on objects");
			break;
		}

		for (unsigned int i = 0; i < numberOfObjects; ++i) {
			if (object_transmitter_ids[i] == 0) {
				continue;
			}
			else {
				if (DataDictionaryGetOrigin(object_transmitter_ids[i], &origin) != READ_OK) {
					RCLCPP_ERROR(get_logger(), "Data dictionary origin read error"
							   "- cannot read the origin for object %u", object_transmitter_ids[i]);
					result = UNDEFINED;
					continue;
				}
				origin.Longitude = strtod(newValue, &endptr);
				if (endptr == newValue) {
					RCLCPP_ERROR(get_logger(), "Unable to convert origin setting %s to double", newValue);
					result = UNDEFINED;
					break;
				}
				if (DataDictionarySetOrigin(&object_transmitter_ids[i], &origin) != WRITE_OK) {
					RCLCPP_ERROR(get_logger(), "Data dictionary origin write error"
							   "- cannot set the origin for object %u", object_transmitter_ids[i]);
					result = UNDEFINED;
					break;
				}
			}
		}

		// TODO remove
		result = DataDictionarySetOriginLongitudeDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE:
		if ((result = DataDictionaryGetNumberOfObjects(&numberOfObjects)) != READ_OK) {
			RCLCPP_ERROR(get_logger(), "Data dictionary number of objects read error"
					   "- cannot set origin on objects");
			break;
		}

		if ((result =
			 DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, numberOfObjects)) != READ_OK) {
			RCLCPP_ERROR(get_logger(),
					   "Data dictionary TransmitterID read error" "- cannot set origin on objects");
			break;
		}

		for (unsigned int i = 0; i < numberOfObjects; ++i) {
			if (object_transmitter_ids[i] == 0) {
				continue;
			}
			else {
				if (DataDictionaryGetOrigin(object_transmitter_ids[i], &origin) != READ_OK) {
					RCLCPP_ERROR(get_logger(), "Data dictionary origin read error"
							   "- cannot read the origin for object %u", object_transmitter_ids[i]);
					result = UNDEFINED;
					continue;
				}
				origin.Altitude = strtod(newValue, &endptr);
				if (endptr == newValue) {
					RCLCPP_ERROR(get_logger(), "Unable to convert origin setting %s to double", newValue);
					result = UNDEFINED;
					break;
				}
				if (DataDictionarySetOrigin(&object_transmitter_ids[i], &origin) != WRITE_OK) {
					RCLCPP_ERROR(get_logger(), "Data dictionary origin write error"
							   "- cannot set the origin for object %u", object_transmitter_ids[i]);
					result = UNDEFINED;
					break;
				}
			}
		}

		// TODO remove
		result = DataDictionarySetOriginAltitudeDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME:
		result = DataDictionarySetVisualizationServerU32(newValue);
		break;
	case CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF:
		result = DataDictionarySetASPMaxTimeDiffDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_ASP_MAX_TRAJ_DIFF:
		result = DataDictionarySetASPMaxTrajDiffDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT:
		result = DataDictionarySetASPStepBackCountU32(newValue);
		break;
	case CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL:
		result = DataDictionarySetASPFilterLevelDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME:
		result = DataDictionarySetASPMaxDeltaTimeDbl(newValue);
		break;
	case CONFIGURATION_PARAMETER_TIME_SERVER_IP:
		result = DataDictionarySetTimeServerIPU32(newValue);
		break;
	case CONFIGURATION_PARAMETER_TIME_SERVER_PORT:
		result = DataDictionarySetTimeServerPortU16(newValue);
		break;
	case CONFIGURATION_PARAMETER_SIMULATOR_IP:
		result = DataDictionarySetSimulatorIPU32(newValue);
		break;
	case CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP:
		result = DataDictionarySetSimulatorTCPPortU16(newValue);
		break;
	case CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP:
		result = DataDictionarySetSimulatorUDPPortU16(newValue);
		break;
	case CONFIGURATION_PARAMETER_SIMULATOR_MODE:
		result = DataDictionarySetSimulatorModeU8(newValue);
		break;
	case CONFIGURATION_PARAMETER_VOIL_RECEIVERS:
		result = DataDictionarySetVOILReceiversString(newValue);
		break;
	case CONFIGURATION_PARAMETER_DTM_RECEIVERS:
		result = DataDictionarySetDTMReceiversString(newValue);
		break;
	case CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP:
		result = DataDictionarySetExternalSupervisorIPU32(newValue);
		break;
	case CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP:
		result = DataDictionarySetSupervisorTCPPortU16(newValue);
		break;
	case CONFIGURATION_PARAMETER_RVSS_CONFIG:
		result = DataDictionarySetRVSSConfigU32((uint32_t) strtoul(newValue, NULL, 10));
		break;
	case CONFIGURATION_PARAMETER_RVSS_RATE:
		result = DataDictionarySetRVSSRateU8((uint8_t) strtoul(newValue, NULL, 10));
		break;
	case CONFIGURATION_PARAMETER_MISC_DATA:
		RCLCPP_WARN(get_logger(), "Unable to set MiscData - size unknown");
		result = DataDictionarySetMiscData(newValue, 0);
		break;
	case CONFIGURATION_PARAMETER_INVALID:
		RCLCPP_WARN(get_logger(), "Attempted to set invalid parameter %s", parameterName);
	default:
		RCLCPP_ERROR(get_logger(), "No action is implemented for setting parameter %s");
	}

	return result == WRITE_OK ? 0 : -1;
}



I32 SystemControl::SystemControlReadServerParameterList(char * ParameterList, U8 Debug) {

	char *line = NULL;
	size_t len = 0;
	FILE *fd;
	char confPathDir[MAX_FILE_PATH];
	ssize_t read;

	UtilGetConfDirectoryPath(confPathDir, sizeof (confPathDir));
	strcat(confPathDir, CONF_FILE_NAME);

	fd = fopen(confPathDir, "r");
	if (fd != NULL) {
		while ((read = getline(&line, &len, fd)) != -1) {
			if (strlen(line) > 0) {
				strcat(ParameterList, line);
				strcat(ParameterList, ";");
			}
		}
		fclose(fd);
		free(line);
	}
	else {
		RCLCPP_ERROR(get_logger(), "Unable to open file %s", confPathDir);
	}

	if (Debug) {
		LogPrint("ParameterList = %s\n", ParameterList);
	}

	return strlen(ParameterList);
}


I32 SystemControl::SystemControlBuildFileContentInfo(const char * Path, U8 Debug) {
	struct stat st;
	char CompletePath[MAX_FILE_PATH];
	bzero(CompletePath, MAX_FILE_PATH);
	if (SystemControlDirectoryInfo.exist)
		return -1;
	UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, Path);
	stat(CompletePath, &st);
	printf("CompletePath = %s\n", CompletePath);

	// Create mmap of the file and return the length
	SystemControlDirectoryInfo.fd = open(CompletePath, O_RDWR);
	SystemControlDirectoryInfo.info_buffer = 
		(char*) mmap(NULL, st.st_size, PROT_READ | PROT_WRITE, MAP_PRIVATE, SystemControlDirectoryInfo.fd, 0);
	SystemControlDirectoryInfo.size = st.st_size;
	SystemControlDirectoryInfo.exist = 1;
	return st.st_size;
}

I32 SystemControl::SystemControlDestroyFileContentInfo(const char * Path, U8 RemoveFile) {
	char CompletePath[MAX_FILE_PATH];
	struct stat st;

	if (!SystemControlDirectoryInfo.exist){
		return -1;
	}
	UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, Path);
	munmap(SystemControlDirectoryInfo.info_buffer, SystemControlDirectoryInfo.size);
	close(SystemControlDirectoryInfo.fd);
	SystemControlDirectoryInfo.exist = 0;
	if (RemoveFile == 1) {
		remove(CompletePath);
	}
	return 0;
}

I32 SystemControl::SystemControlCheckFileDirectoryExist(char * ParameterName, char * ReturnValue, U8 Debug) {

	DIR *pDir;
	FILE *fd;
	char CompletePath[MAX_FILE_PATH];

	bzero(CompletePath, MAX_FILE_PATH);
	UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, ParameterName);

	*ReturnValue = PATH_INVALID_MISSING;

	pDir = opendir(CompletePath);
	if (pDir == NULL) {
		fd = fopen(CompletePath, "r");
		if (fd != NULL) {
			*ReturnValue = FILE_EXIST;	//File exist
			fclose(fd);
		}
	}
	else {
		*ReturnValue = FOLDER_EXIST;	//Directory exist
		closedir(pDir);
	}


	if (Debug)
		LogPrint("%d %s", *ReturnValue, CompletePath);


	return 0;
}

/*!
 * \brief SystemControlDeleteTrajectory Deletes the chosen trajectory
 * \param trajectoryName Name of the trajectory file
 * \param nameLen Length of the name string
 * \return Returns ::SUCCEDED_DELETE upon successfully deleting a file, otherwise ::FAILED_DELETE
 */
char SystemControl::SystemControlDeleteTrajectory(const char * trajectoryName, const size_t nameLen) {
	return UtilDeleteTrajectoryFile(trajectoryName, nameLen) ? FAILED_DELETE : SUCCEEDED_DELETE;
}

/*!
 * \brief SystemControlDeleteGeofence Deletes the chosen geofence
 * \param trajectoryName Name of the geofence file
 * \param nameLen Length of the name string
 * \return Returns ::SUCCEDED_DELETE upon successfully deleting a file, otherwise ::FAILED_DELETE
 */
char SystemControl::SystemControlDeleteGeofence(const char * geofenceName, const size_t nameLen) {
	return UtilDeleteGeofenceFile(geofenceName, nameLen) ? FAILED_DELETE : SUCCEEDED_DELETE;
}

/*!
 * \brief SystemControlDeleteGenericFile Deletes the chosen geofence
 * \param trajectoryName Name of the geofence file
 * \param nameLen Length of the name string
 * \return Returns ::SUCCEDED_DELETE upon successfully deleting a file, otherwise ::FAILED_DELETE
 */
char SystemControl::SystemControlDeleteGenericFile(const char * filePath, const size_t nameLen) {
	return UtilDeleteGenericFile(filePath, nameLen) ? FAILED_DELETE : SUCCEEDED_DELETE;
}

/*!
 * \brief SystemControlClearTrajectories Clears the trajectory directory on the machine
 * \return Returns ::SUCCEDED_DELETE upon successfully deleting a file, otherwise ::FAILED_DELETE.
 */
char SystemControl::SystemControlClearTrajectories(void) {
	if (UtilDeleteTrajectoryFiles() != 0) {
		return FAILED_DELETE;
	}
	return SUCCEEDED_DELETE;
}

/*!
 * \brief SystemControlClearGeofences Clears the geofence directory on the machine
 * \return Returns ::SUCCEDED_DELETE upon successfully deleting a file, otherwise ::FAILED_DELETE.
 */
char SystemControl::SystemControlClearGeofences(void) {
	if (UtilDeleteGeofenceFiles() != 0) {
		return FAILED_DELETE;
	}
	return SUCCEEDED_DELETE;
}

/*!
 * \brief SystemControlClearObjects Clears the objects directory on the machine
 * \return Returns ::SUCCEDED_DELETE upon successfully deleting a file, otherwise ::FAILED_DELETE.
 */
char SystemControl::SystemControlClearObjects(void) {
	if (UtilDeleteObjectFiles() != 0) {
		return FAILED_DELETE;
	}
	return SUCCEEDED_DELETE;
}

I32 SystemControl::SystemControlDeleteFileDirectory(const char * Path, char * ReturnValue, U8 Debug) {

	DIR *pDir;
	FILE *fd;
	char CompletePath[MAX_FILE_PATH];

	bzero(CompletePath, MAX_FILE_PATH);
	UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, Path);

	*ReturnValue = PATH_INVALID_MISSING;

	pDir = opendir(CompletePath);
	if (pDir == NULL) {
		fd = fopen(CompletePath, "r");
		if (fd == NULL) {
			*ReturnValue = PATH_INVALID_MISSING;	//Missing file
		}
		else {
			if (0 == remove(CompletePath))	//Delete file
			{
				*ReturnValue = SUCCEEDED_DELETE;
			}
			else {
				*ReturnValue = FAILED_DELETE;
			}
		}
	}
	else {
		if (0 == remove(CompletePath))	//Delete directory
		{
			*ReturnValue = SUCCEEDED_DELETE;
		}
		else {
			*ReturnValue = FAILED_DELETE;
		}
	}

	if (*ReturnValue == SUCCEEDED_DELETE)
		RCLCPP_INFO(get_logger(), "Deleted %s", CompletePath);
	else if (*ReturnValue == FAILED_DELETE)
		RCLCPP_INFO(get_logger(), "Failed to delete %s", CompletePath);

	return 0;
}


I32 SystemControl::SystemControlCreateDirectory(const char * Path, char * ReturnValue, U8 Debug) {

	DIR *pDir;
	FILE *fd;
	char CompletePath[MAX_FILE_PATH];

	bzero(CompletePath, MAX_FILE_PATH);
	UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, Path);

	*ReturnValue = PATH_INVALID_MISSING;

	pDir = opendir(CompletePath);
	if (pDir == NULL) {
		fd = fopen(CompletePath, "r");
		if (fd != NULL) {
			*ReturnValue = FILE_EXIST;	//This is a file!
			fclose(fd);
		}
		else {
			if (0 == mkdir(CompletePath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))	//Make the new directory
			{
				*ReturnValue = SUCCEDED_CREATE_FOLDER;
			}
			else {
				*ReturnValue = FAILED_CREATE_FOLDER;
			}
		}
	}
	else {
		*ReturnValue = FOLDER_EXIST;	//Directory exist
		closedir(pDir);
	}

	if (Debug)
		LogPrint("%d %s", *(ReturnValue), CompletePath);

	if (*ReturnValue == SUCCEDED_CREATE_FOLDER)
		RCLCPP_INFO(get_logger(), "Directory created: %s", CompletePath);

	return 0;
}


I32 SystemControl::SystemControlUploadFile(const char * Filename, const char * FileSize, const char * PacketSize, const char * FileType, char * ReturnValue,
							char * CompleteFilePath, U8 Debug) {

	FILE *fd;
	char CompletePath[MAX_FILE_PATH];

	memset(CompletePath, 0, sizeof (CompletePath));
	//GetCurrentDir(CompletePath, MAX_FILE_PATH);
	//strcat(CompletePath, Filename);
	if (Filename == NULL || FileSize == NULL || PacketSize == NULL || FileType == NULL || ReturnValue == NULL) {
		RCLCPP_ERROR(get_logger(), "Invalid function parameter passed to upload file handler function");
		return -1;
	}

	switch (atoi(FileType)) {
	case MAESTRO_GENERIC_FILE_TYPE:
		UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
		break;
	case MAESTRO_TRAJ_FILE_TYPE:
		UtilGetTrajDirectoryPath(CompletePath, sizeof (CompletePath));
		break;
	case MAESTRO_CONF_FILE_TYPE:
		UtilGetConfDirectoryPath(CompletePath, sizeof (CompletePath));
		break;
	case MAESTRO_GEOFENCE_FILE_TYPE:
		UtilGetGeofenceDirectoryPath(CompletePath, sizeof (CompletePath));
		break;
	case MAESTRO_OBJECT_FILE_TYPE:
		UtilGetObjectDirectoryPath(CompletePath, sizeof (CompletePath));
		break;
	default:
		RCLCPP_ERROR(get_logger(), "Received invalid file type upload request");
		//Create temporary file for handling data anyway
		UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
		strcat(CompletePath, "/file.tmp");
		fd = fopen(CompletePath, "r");
		if (fd != NULL) {
			fclose(fd);
			remove(CompletePath);	//Remove file if exist
		}
		fd = fopen(CompletePath, "w+");	//Create the temporary file

		*ReturnValue = PATH_INVALID_MISSING;
		return -1;
	}
	strcat(CompletePath, Filename);
	strcpy(CompleteFilePath, CompletePath);

	if (Debug) {
		LogPrint("Filename: %s", Filename);
		LogPrint("FileSize: %s", FileSize);
		LogPrint("PacketSize: %s", PacketSize);
		LogPrint("FileType: %s", FileType);
		LogPrint("CompletePath: %s", CompletePath);
		LogPrint("CompleteFilePath: %s", CompleteFilePath);
	}

	if (atoi(PacketSize) > SYSTEM_CONTROL_RX_PACKET_SIZE) {	//Check packet size
		*ReturnValue = SERVER_PREPARED_BIG_PACKET_SIZE;
		return 0;
	}

	fd = fopen(CompletePath, "r");
	if (fd != NULL) {
		fclose(fd);
		remove(CompletePath);	//Remove file if exist
		RCLCPP_INFO(get_logger(), "Deleted existing file <%s>", CompletePath);
	}

	fd = fopen(CompletePath, "w+");	//Create the file
	if (fd != NULL) {
		*ReturnValue = SERVER_PREPARED;	//Server prepared
		fclose(fd);
		return 0;
	}
	else {
		//Failed to open path create temporary file
		UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
		strcat(CompletePath, "/file.tmp");
		fd = fopen(CompletePath, "r");
		if (fd != NULL) {
			fclose(fd);
			remove(CompletePath);	//Remove file if exist
		}
		fd = fopen(CompletePath, "w+");	//Create the temporary file

		*ReturnValue = PATH_INVALID_MISSING;

		return 0;
	}

	return -1;
}


I32 SystemControl::SystemControlReceiveRxData(I32 * sockfd, const char * Path, const char * FileSize, const char * PacketSize, char * ReturnValue,
							   U8 Debug) {

	FILE *fd;
	char CompletePath[MAX_FILE_PATH];

	bzero(CompletePath, MAX_FILE_PATH);
	//UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, Path);
	U32 FileSizeU32 = atoi(FileSize);
	U16 PacketSizeU16 = atoi(PacketSize);
	I32 ClientStatus = 0, Time1 = 0, Time2 = 0, TimeDiff = 0, i = 0, j = 0;
	char RxBuffer[SYSTEM_CONTROL_RX_PACKET_SIZE];
	U32 TotalRxCount = 0, TransmissionCount = (U32) (FileSizeU32 / PacketSizeU16), RestCount =
		FileSizeU32 % PacketSizeU16;
	struct timeval CurTime;


	if (Debug) {
		LogPrint("Receive Rx data:");
		LogPrint("Path: %s", Path);
		LogPrint("FileSize: %s", FileSize);
		LogPrint("PacketSize: %s", PacketSize);
		LogPrint("CompletePath: %s", CompletePath);
	}



	fd = fopen(CompletePath, "w+");
	if (fd != NULL) {

		gettimeofday(&CurTime, NULL);
		Time1 = CurTime.tv_sec * 1000 + CurTime.tv_usec / 1000;

		while (TotalRxCount < FileSizeU32 && TimeDiff < 3000) {
			gettimeofday(&CurTime, NULL);
			Time2 = CurTime.tv_sec * 1000 + CurTime.tv_usec / 1000;


			if (i == TransmissionCount) {
				PacketSizeU16 = RestCount;
			}

			bzero(RxBuffer, PacketSizeU16);
			ClientStatus = recv(*sockfd, RxBuffer, PacketSizeU16, MSG_WAITALL);

			if (ClientStatus > 0) {
				i++;
				fwrite(RxBuffer, 1, ClientStatus, fd);
				fflush(fd);
				if (Debug) {
					printf("%d, %d, %d, %d, %d, %d :", i, ClientStatus, TotalRxCount, TimeDiff, PacketSizeU16,
						   FileSizeU32);
					for (j = 0; j < 10; j++)
						printf("%x-", RxBuffer[j]);
					printf("...\n");
				}
				TotalRxCount = TotalRxCount + ClientStatus;
				gettimeofday(&CurTime, NULL);
				Time1 = CurTime.tv_sec * 1000 + CurTime.tv_usec / 1000;
			}


			TimeDiff = abs(Time1 - Time2);
		}

		fclose(fd);

		if (TotalRxCount == FileSizeU32) {
			*ReturnValue = FILE_UPLOADED;
		}
		else if (TotalRxCount > FileSizeU32) {
			*ReturnValue = FILE_TO_MUCH_DATA;
			remove(CompletePath);
			RCLCPP_INFO(get_logger(), "CORRUPT FILE, REMOVING...");
		}
		else {
			*ReturnValue = TIME_OUT;
			remove(CompletePath);
			RCLCPP_INFO(get_logger(), "CORRUPT FILE, REMOVING...");
		}

		RCLCPP_INFO(get_logger(), "Rec count = %d, Req count = %d", TotalRxCount, FileSizeU32);

	}

	return 0;
}


I32 SystemControl::SystemControlSendFileContent(I32 * sockfd, const char * Path, const char * PacketSize, char * ReturnValue, U8 Remove,
								 U8 Debug) {
	FILE *fd;
	char CompletePath[MAX_FILE_PATH];

	bzero(CompletePath, MAX_FILE_PATH);
	UtilGetTestDirectoryPath(CompletePath, sizeof (CompletePath));
	strcat(CompletePath, Path);
	U32 FileSizeU32 = 0;
	U16 PacketSizeU16 = atoi(PacketSize);
	I32 ClientStatus = 0, Time1 = 0, Time2 = 0, TimeDiff = 0, i = 0, j = 0;
	char TxBuffer[SYSTEM_CONTROL_TX_PACKET_SIZE];
	U32 TotalRxCount = 0, TransmissionCount = 0, RestCount = 0;
	struct timeval CurTime;
	struct stat st;

	stat(CompletePath, &st);
	TransmissionCount = (U32) (st.st_size) / PacketSizeU16;
	RestCount = (U32) (st.st_size) % PacketSizeU16;

	if (Debug) {
		LogPrint("Send file content:");
		LogPrint("%s", Path);
		LogPrint("%s", PacketSize);
		LogPrint("%s", CompletePath);
	}

	fd = fopen(CompletePath, "r");

	if (fd != NULL) {

		for (i = 0; i < TransmissionCount; i++) {
			bzero(TxBuffer, PacketSizeU16);
			fread(TxBuffer, 1, PacketSizeU16, fd);
			//SystemControlSendBytes(TxBuffer, PacketSizeU16, sockfd, 0); //Send a packet
			UtilSendTCPData((uint8_t*) "System Control", (uint8_t*) TxBuffer, PacketSizeU16, sockfd, 0);
		}

		if (RestCount > 0) {
			bzero(TxBuffer, PacketSizeU16);
			fread(TxBuffer, 1, RestCount, fd);
			//SystemControlSendBytes(TxBuffer, RestCount, sockfd, 0); //Send the rest
			UtilSendTCPData((uint8_t*) "System Control", (uint8_t*) TxBuffer, RestCount, sockfd, 0);
		}

		fclose(fd);

		if (Remove)
			remove(CompletePath);

		RCLCPP_INFO(get_logger(), "Sent file: %s, total size = %d, transmissions = %d", CompletePath,
				   (PacketSizeU16 * TransmissionCount + RestCount), i + 1);

	}

	return 0;
}


/*!
 * \brief SystemControlUpdateRVSSSendTime Adds a time interval onto the specified time struct in accordance
 *			with the rate parameter
 * \param currentRVSSSendTime Struct containing the time at which the last RVSS message was sent. After this
 *			function has been executed, the struct contains the time at which the next RVSS message is to be
 *			sent.
 * \param RVSSRate_Hz Rate at which RVSS messages are to be sent - if this parameter is 0 the value
 *			is clamped to 1 Hz
 */
void SystemControl::SystemControlUpdateRVSSSendTime(struct timeval *currentRVSSSendTime, uint8_t RVSSRate_Hz) {
	struct timeval RVSSTimeInterval, timeDiff, currentTime;

	RVSSRate_Hz = RVSSRate_Hz == 0 ? 1 : RVSSRate_Hz;	// Minimum frequency 1 Hz
	RVSSTimeInterval.tv_sec = (long)(1.0 / RVSSRate_Hz);
	RVSSTimeInterval.tv_usec = (long)((1.0 / RVSSRate_Hz - RVSSTimeInterval.tv_sec) * 1000000.0);

	// If there is a large difference between the current time and the time at which RVSS was sent, update based
	// on current time instead of last send time to not spam messages until caught up
	TimeSetToCurrentSystemTime(&currentTime);
	timersub(&currentTime, currentRVSSSendTime, &timeDiff);
	if (timercmp(&timeDiff, &RVSSTimeInterval, <)) {
		timeradd(currentRVSSSendTime, &RVSSTimeInterval, currentRVSSSendTime);
	}
	else {
		timeradd(&currentTime, &RVSSTimeInterval, currentRVSSSendTime);
	}
}


/*
SystemControlBuildRVSSTimeChannelMessage builds a message from data in *GPSTime. The message is stored in *RVSSData.
See the architecture document for the protocol of RVSS.

- *RVSSData the buffer the message
- *RVSSDataLengthU32 the length of the message
- *GPSTime current time data
- Debug enable(1)/disable(0) debug printouts (Not used)
*/
I32 SystemControl::SystemControlBuildRVSSTimeChannelMessage(char * RVSSData, U32 * RVSSDataLengthU32,
											 U8 Debug) {
	struct timeval systemTime;
	time_t ttime;
	struct tm* ttm;
	char tmpbuf[64];
	TimeSetToCurrentSystemTime(&systemTime);
	ttime = systemTime.tv_sec;

	ttm = localtime(&ttime);
	strftime(tmpbuf, sizeof (tmpbuf), "%Y-%m-%d %H:%M:%S", ttm);
	snprintf(RVSSData, SYSTEM_CONTROL_RVSS_DATA_BUFFER, "%s.%06ld", tmpbuf, systemTime.tv_usec);

	if (Debug) {
		LogPrint(RVSSData);
	}
	*RVSSDataLengthU32 = strlen(RVSSData);

	return 0;
}


/*
SystemControlBuildRVSSMaestroChannelMessage builds a message from OBCState in DataDictionary and SysCtrlState. The message is stored in *RVSSData.
See the architecture document for the protocol of RVSS.

- *RVSSData the buffer the message
- *RVSSDataLengthU32 the length of the message
- U8 SysCtrlState the SystemControl state (SystemControlState)
- Debug enable(1)/disable(0) debug printouts (Not used)
*/
I32 SystemControl::SystemControlBuildRVSSMaestroChannelMessage(char * RVSSData, U32 * RVSSDataLengthU32,
												U8 SysCtrlState, U8 Debug) {
	I32 MessageIndex = 0, i;
	char *p;


	RVSSMaestroType RVSSMaestroData;
	OBCState_t obcState;

	RVSSMaestroData.MessageLengthU32 = SwapU32((U32) sizeof (RVSSMaestroType) - 4);
	RVSSMaestroData.ChannelCodeU32 = SwapU32((U32) RVSS_MAESTRO_CHANNEL);
	DataDictionaryGetOBCState(&obcState);
	RVSSMaestroData.OBCStateU8 = (uint8_t) (obcState);
	RVSSMaestroData.SysCtrlStateU8 = SysCtrlState;

	p = (char *) & RVSSMaestroData;
	for (i = 0; i < sizeof (RVSSMaestroType); i++)
		*(RVSSData + i) = *p++;
	*RVSSDataLengthU32 = i;

	if (Debug) {

	}

	return 0;
}



#define MAX_MONR_STRING_LENGTH 1024
/*!
 * \brief SystemControlSendRVSSMONRChannelMessages Sends RVSS monitoring data messages based on objects present
 *			in data dictionary
 * \param socket Socket descriptor pointer for RVSS socket
 * \param addr Address struct pointer for RVSS socket
 * \return 0 on success, -1 otherwise
 */
int32_t SystemControl::SystemControlSendRVSSMonitorChannelMessages(int *socket, struct sockaddr_in *addr) {
	uint32_t messageLength = 0;
	uint32_t RVSSChannel = RVSS_MONITOR_CHANNEL;
	char RVSSData[MAX_MONR_STRING_LENGTH];
	char *monitorDataString = RVSSData + sizeof (messageLength) + sizeof (RVSSChannel);
	uint32_t *transmitterIDs = NULL;
	uint32_t numberOfObjects;
	ObjectDataType monitorData;

	// Get number of objects present in shared memory
	if (DataDictionaryGetNumberOfObjects(&numberOfObjects) != READ_OK) {
		RCLCPP_ERROR(get_logger(),
				   "Data dictionary number of objects read error - RVSS messages cannot be sent");
		return -1;
	}

	// Allocate an array for objects' transmitter IDs
	transmitterIDs = (uint32_t*) malloc(numberOfObjects * sizeof (uint32_t));
	if (transmitterIDs == NULL) {
		RCLCPP_ERROR(get_logger(), "Memory allocation error - RVSS messages cannot be sent");
		return -1;
	}

	// Get transmitter IDs for all connected objects
	if (DataDictionaryGetObjectTransmitterIDs(transmitterIDs, numberOfObjects) != READ_OK) {
		free(transmitterIDs);
		RCLCPP_ERROR(get_logger(),
				   "Data dictionary transmitter ID read error - RVSS messages cannot be sent");
		return -1;
	}

	RCLCPP_DEBUG(get_logger(), "%s: Found %u transmitter IDs", __FUNCTION__, numberOfObjects);
	// Loop over transmitter IDs, sending a message on the RVSS channel for each
	int32_t retval = 0;

	for (uint32_t i = 0; i < numberOfObjects; ++i) {
		if (transmitterIDs[i] != 0) {
			monitorData.ClientID = transmitterIDs[i];
			struct timeval recvTime;

			if (DataDictionaryGetMonitorDataReceiveTime(transmitterIDs[i], &recvTime) != READ_OK) {
				RCLCPP_ERROR(get_logger(),
						   "Data dictionary monitor data read error for transmitter ID %u - RVSS message cannot be sent",
						   transmitterIDs[i]);
				retval = -1;
			}
			else {
				if (recvTime.tv_sec == 0 && recvTime.tv_usec == 0) {
					continue;
				}
			}

			if (DataDictionaryGetObjectIPByTransmitterID(transmitterIDs[i], &monitorData.ClientIP) != READ_OK) {
				RCLCPP_ERROR(get_logger(),
						   "Data dictionary monitor data read error for transmitter ID %u - RVSS message cannot be sent",
						   transmitterIDs[i]);
				retval = -1;
			}
			else if (DataDictionaryGetMonitorData(transmitterIDs[i], &monitorData.MonrData) != READ_OK) {
				RCLCPP_ERROR(get_logger(),
						   "Data dictionary monitor data read error for transmitter ID %u - RVSS message cannot be sent",
						   transmitterIDs[i]);
				retval = -1;
			}
			else if (UtilObjectDataToString(monitorData, monitorDataString,
											sizeof (RVSSData) - (size_t)(monitorDataString - RVSSData)) ==
					 -1) {
				RCLCPP_ERROR(get_logger(), "Error building object data string");
				retval = -1;
			}
			else {
				RCLCPP_DEBUG(get_logger(), "%s: Transmitter ID %u", __FUNCTION__, transmitterIDs[i]);
				messageLength =
					(uint32_t) (strlen(monitorDataString) + sizeof (messageLength) + sizeof (RVSSChannel));
				messageLength = htole32(messageLength);
				RVSSChannel = htole32(RVSSChannel);
				memcpy(RVSSData, &messageLength, sizeof (messageLength));
				memcpy(RVSSData + sizeof (messageLength), &RVSSChannel, sizeof (RVSSChannel));

				UtilSendUDPData((uint8_t*) module_name.c_str(), socket, addr, (uint8_t*) RVSSData, messageLength, 0);
			}
		}
	}
	free(transmitterIDs);
	return retval;
}


/*
SystemControlBuildRVSSAspChannelMessage shall be used for sending ASP-debug data. The message is stored in *RVSSData.
See the architecture document for the protocol of RVSS.

- *RVSSData the buffer the message
- *RVSSDataLengthU32 the length of the message
- *ASPdata TBD
- Debug enable(1)/disable(0) debug printouts (Not used)
*/

I32 SystemControl::SystemControlBuildRVSSAspChannelMessage(char * RVSSData, U32 * RVSSDataLengthU32, U8 Debug) {
	RVSSTimeType RVSSTimeData;




	return 0;
}


void SystemControl::appendSysInfoString(char *ControlResponseBuffer, const size_t bufferSize) {

	size_t remainingBufferSpace = bufferSize;
	struct sysinfo info;
	char stringBuffer[255];
	long hours, minutes, seconds;

	// Server uptime
	sysinfo(&info);

	hours = info.uptime / 3600;
	minutes = (info.uptime - (3600 * hours)) / 60;
	seconds = (info.uptime - (3600 * hours) - (minutes * 60));

	snprintf(stringBuffer, sizeof (stringBuffer), "Machine uptime: %ld:%ld:%ld\n", hours, minutes, seconds);
	strncat(ControlResponseBuffer, stringBuffer, remainingBufferSpace - 1);
	remainingBufferSpace -= strlen(stringBuffer);

	unsigned long startTime = UtilGetPIDUptime(getpid()).tv_sec;

	unsigned long long timeAtStart = startTime / (unsigned long long)(sysconf(_SC_CLK_TCK));
	long serverUptime = (long)((unsigned long long)info.uptime - timeAtStart);


	hours = serverUptime / 3600;
	minutes = (serverUptime - (3600 * hours)) / 60;
	seconds = (serverUptime - (3600 * hours) - (minutes * 60));
	sprintf(stringBuffer, "Server uptime: %ld:%ld:%ld\n", hours, minutes, seconds);
	strncat(ControlResponseBuffer, stringBuffer, remainingBufferSpace - 1);
	remainingBufferSpace -= strlen(stringBuffer);
	//Make it clear that this is placeholder data
	strncat(ControlResponseBuffer, "Maestro powerlevel: 90001\n", remainingBufferSpace - 1);
}

