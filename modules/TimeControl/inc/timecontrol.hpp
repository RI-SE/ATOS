#include "module.hpp"

#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>

#include "journal.h"
#include "maestroTime.h"
#include "datadictionary.h"
#include "logging.h"

class TimeControl : public Module
{
public:
	static inline const std::string module_name = "time_control";
	TimeControl();
	bool shouldExit() const;
	const int64_t getQueueEmptyPollPeriod() const;
	void calibrateTime(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel);
	void signalHandler(int signo);
	void initialize(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel);
private:
	ROSChannels::Exit::Sub exitSub;
	/* definitions */
	static const int TIME_CONTROL_HOSTNAME_BUFFER_SIZE = 20;
	static const int TIME_CONTROL_RECEIVE_BUFFER_SIZE = 80;
	static const int TIME_CONTROL_TASK_PERIOD_MS = 1;
	static const int TIME_CONTROL_TX_BUFFER_SIZE = 14;
	static const int REPLY_TIMEOUT_S = 3;

	static inline const std::string LOG_PATH = "./timelog/";
	static inline const std::string LOG_FILE = "time.log";
	static const int LOG_BUFFER_LENGTH = 128;

	static const int FIX_QUALITY_NONE = 0;
	static const int FIX_QUALITY_BASIC = 1;
	static const int FIX_QUALITY_DIFFERENTIAL = 2;

	static const int QUEUE_EMPTY_POLL_PERIOD_NS = 10000000;

	static const int PROTO2_STATUS_INDEX = 0;
	static const int PROTO2_LENGTH_INDEX = 2;
	static const int PROTO2_ID_INDEX = 5;
	static const int PROTO2_SERNO_INDEX = 8;
	static const int PROTO2_MESSAGE_CODE_INDEX = 11;
	static const int PROTO2_MESSAGE_VERSION_INDEX = 12;
	static const int PROTO2_YEAR_INDEX = 13;
	static const int PROTO2_MONTH_INDEX = 15;
	static const int PROTO2_DAYOFMONTH_INDEX = 16;
	static const int PROTO2_HOUR_INDEX = 17;
	static const int PROTO2_MINUTE_INDEX = 18;
	static const int PROTO2_SECOND_INDEX = 19;
	static const int PROTO2_MILLISECOND_INDEX = 20;
	static const int PROTO2_MICROSECOND_INDEX = 22;
	static const int PROTO2_SECOND_COUNTER_INDEX = 24;
	static const int PROTO2_GPS_MILLISECONDS_INDEX = 28;
	static const int PROTO2_GPS_MINUTES_INDEX = 36;
	static const int PROTO2_GPS_WEEK_INDEX = 40;
	static const int PROTO2_GPSSOW_INDEX = 42;
	static const int PROTO2_GPSSOD_INDEX = 46;
	static const int PROTO2_ETSI_INDEX = 50;
	static const int PROTO2_LATITUDE_INDEX = 58;
	static const int PROTO2_LONGITUDE_INDEX = 62;
	static const int PROTO2_FIX_QUALITY_INDEX = 66;
	static const int PROTO2_SATELLITE_COUNT_INDEX = 67;

	static const int PROTO2_SETUP_MESSAGE_LENGTH = 10;
	static const int PROTO2_SETUP_TIME_FEED_MESSAGE_CODE = 0x17;
	static const int PROTO2_SETUP_TIME_FEED_ACTIVE = 1;
	static const int PROTO2_SETUP_TIME_FEED_DEACTIVATE = 0;
	static const int PROTO2_SETUP_TIME_FEED_USE_PERIOD_IN_FILE = 0;
	static const int PROTO2_SETUP_TIME_FEED_USE_PERIOD_IN_MESSAGE = 1;
	static const int PROTO2_SETUP_TIME_FEED_INTERVAL = 1000;
	static const int PROTO2_SETUP_TIME_FEED_INTERVAL_FAST = 100;

	/* callbacks */
	void onAbortMessage(Empty::SharedPtr) override;
	void onExitMessage(Empty::SharedPtr) override;

	/* variables */
	volatile int iExit = 0;
	C8 TextBufferC8[TIME_CONTROL_HOSTNAME_BUFFER_SIZE];
	C8 ServerIPC8[TIME_CONTROL_HOSTNAME_BUFFER_SIZE];
	U16 ServerPortU16 = DEFAULT_TIME_SERVER_PORT;
	I32 SocketfdI32 = -1;
	struct sockaddr_in time_addr;

	I32 result;
	C8 TimeBuffer[TIME_CONTROL_RECEIVE_BUFFER_SIZE];
	C8 LogBuffer[LOG_BUFFER_LENGTH];
	I32 ReceivedNewData, i;

	C8 PitipoSetupMessage[TIME_CONTROL_TX_BUFFER_SIZE] = { 0, 0, 0, PROTO2_SETUP_MESSAGE_LENGTH,
		0, 0, 0, PROTO2_SETUP_TIME_FEED_MESSAGE_CODE,
		PROTO2_SETUP_TIME_FEED_ACTIVE, PROTO2_SETUP_TIME_FEED_USE_PERIOD_IN_MESSAGE,
		0, 0, (uint8_t) (PROTO2_SETUP_TIME_FEED_INTERVAL >> 8),
		(uint8_t) PROTO2_SETUP_TIME_FEED_INTERVAL
	};
	struct timespec sleep_time, ref_time;
	C8 MqRecvBuffer[MBUS_MAX_DATALEN];
	struct timeval tv, ExecTime;
	struct tm *tm;

	U32 IpU32;
	U8 PrevSecondU8;
	U16 CurrentMilliSecondU16, PrevMilliSecondU16;
	U8 CycleCount = 0;

	enum COMMAND command;
	char busReceiveBuffer[MBUS_MAX_DATALEN];

	/*functions*/
	void TimeControlDecodeTimeBuffer(TimeType * GPSTime, C8 * TimeBuffer, C8 debug);
	void TimeControlRecvTime(int *sockfd, C8 * buffer, int length, int *receivedNewData);
	int TimeControlSendUDPData(int *sockfd, struct sockaddr_in *addr, C8 * SendData, int Length,
								char debug);
	uint32_t TimeControlIPStringToInt(char * IP);
	int TimeControlCreateTimeChannel(const char *name, const uint32_t port, int *sockfd,
									struct sockaddr_in *addr);
	U16 TimeControlGetMillisecond(TimeType * GPSTime);
};
