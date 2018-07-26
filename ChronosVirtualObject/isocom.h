#ifndef ISOCOM_H
#define ISOCOM_H


#include <QObject>
#include <QUdpSocket>
#include <QDataStream>

#include "tcpserversimple.h"
//#include "packetinterface.h"
#include "utility.h"
#include "vbytearray.h"




// ISO information

// Control Center status
#define ISO_CC_STATUS_OK 0x01
#define ISO_CC_STATUS_EMERGENCY_ABORT 0x02

// Object internal state
#define ISO_OBJECT_INTERNAL_STATE_READY_TO_ARM 0x00
#define ISO_OBJECT_INTERNAL_STATE_NOT_READY_TO_ARM 0x01

// Object states
#define ISO_OBJECT_STATE_INIT       0x01
#define ISO_OBJECT_STATE_ARMED      0x02
#define ISO_OBJECT_STATE_DISARMED   0x03
#define ISO_OBJECT_STATE_RUNNING    0x04
#define ISO_OBJECT_STATE_POST_RUN   0x05

// Packet defines
#define ISO_SYNC_WORD 0x7E7E
#define ISO_PART_SYNC_WORD 0x7E

// Message types

#define ISO_MSG_DOTM 0x0001
#define ISO_MSG_OSEM 0x0002
#define ISO_MSG_OSTM 0x0003
#define ISO_MSG_STRT 0x0004
#define ISO_MSG_HEAB 0x0005
#define ISO_MSG_MONR 0x0006


// VALUE ID

#define ISO_VALUE_ID_LAT 0x0020
#define ISO_VALUE_ID_LON 0x0021
#define ISO_VALUE_ID_ALT 0x0022
#define ISO_VALUE_ID_DateISO8601 0x0004
#define ISO_VALUE_ID_GPS_WEEK 0x0003
#define ISO_VALUE_ID_GPS_SEC_OF_WEEK 0x0002
#define ISO_VALUE_ID_MAX_WAY_DEV 0x0070
#define ISO_VALUE_ID_MAX_LATERAL_DEV 0x0072
#define ISO_VALUE_ID_MIN_POS_ACCURACY 0x0074
#define ISO_VALUE_ID_STATE_CHANGE_REQ 0x0064
#define ISO_VALUE_ID_DELAYED_START 0x0001
#define ISO_VALUE_ID_REL_TIME 0x0001
#define ISO_VALUE_ID_X_POS 0x0010
#define ISO_VALUE_ID_Y_POS 0x0011
#define ISO_VALUE_ID_Z_POS 0x0012
#define ISO_VALUE_ID_HEADING 0x0030
#define ISO_VALUE_ID_LONG_SPEED 0x0040
#define ISO_VALUE_ID_LAT_SPEED 0x0041
#define ISO_VALUE_ID_LONG_ACC 0x0050
#define ISO_VALUE_ID_LAT_ACC 0x0051

// Binary IDs

#define CONTENT_BINARY_ID_0 0x0001
#define CONTENT_BINARY_ID_1 0x0002
#define CONTENT_BINARY_ID_2 0x0004
#define CONTENT_BINARY_ID_3 0x0008
#define CONTENT_BINARY_ID_4 0x0010
#define CONTENT_BINARY_ID_5 0x0020
#define CONTENT_BINARY_ID_6 0x0040
#define CONTENT_BINARY_ID_7 0x0080
#define CONTENT_BINARY_ID_8 0x0100
#define CONTENT_BINARY_ID_9 0x0200
#define CONTENT_BINARY_ID_10 0x0400
#define CONTENT_BINARY_ID_11 0x0800
#define CONTENT_BINARY_ID_12 0x1000
#define CONTENT_BINARY_ID_13 0x2000
#define CONTENT_BINARY_ID_14 0x4000
#define CONTENT_BINARY_ID_15 0x8000

#define OSEM_MANDATORY_CONTENT 0x01FF
#define OSTM_MANDATORY_CONTENT 0x0001
#define STRT_MANDATORY_CONTENT 0x0003
#define DOTM_MANDATORY_CONTENT 0x01FF

// DATA
#define ISO_MIN_CONTENT_DATA 5

#define ISO_TCP_STATE_READ_HEADER 0
#define ISO_TCP_STATE_READ_DATA 1
#define ISO_TCP_STATE_READ_FOOTER 2

#define LEAP_MSEC_DIFF_UTC_GPS 18000

#define ISO_MESSAGE_HEADER_BYTE_LENGTH 10
typedef struct
{
    quint8 TxID;
    quint8 MESSAGE_COUNTER;
    quint8 ACK_REQ;
    quint8 PROTOCOL_VERSION;
    quint16 MESSAGE_ID;
    quint32 MESSAGE_LENGTH;
} ISO_MESSAGE_HEADER;

typedef struct {
    quint16 CRC;
} ISO_MESSAGE_FOOTER;

typedef struct {
    qint64 lat;
    qint64 lon;
    qint32 alt;
    quint32 dateISO8601;
    quint16 GPSweek;
    quint32 GPSsecOfWeek;
    quint16 MaxWayDev;
    quint16 MaxLatDev;
    quint16 MinPosAccuracy;
} osem;

typedef struct
{
    quint8 state_change;
} ostm;

typedef struct
{
    quint32 GPSsecOfWeek_start_time;
    quint32 delay_ms;
} strt;

typedef struct
{
    quint32 GPSsecOfWeek;
    quint8 cc_status;
} heab;

typedef struct
{
    quint32 rel_time;
    qint32 x;
    qint32 y;
    qint32 z;
    quint16 heading;
    qint16 lon_speed;
    qint16 lat_speed;
    qint16 lon_acc;
    qint16 lat_acc;
} dotm_pt;

typedef struct
{
    uint64_t time_stamp;
    int32_t x;
    int32_t y;
    int32_t z;
    uint16_t heading;
    int16_t lon_speed;
    int16_t lat_speed;
    int16_t lon_acc;
    int16_t lat_acc;
    uint8_t drive_direction;
    uint8_t object_state;
    uint8_t ready_to_arm;
} monr;


class ISOcom : public QObject
{
    Q_OBJECT
public:
    ISOcom(QObject *parent = 0);
    bool startServer(int udpSocket, int tcpSocket);
    //bool startServer(PacketInterface *packet);


    bool sendMonr(monr msg);
    //bool sendTOM(chronos_tom tom);

    static bool packetHeaderRx(QDataStream &data, ISO_MESSAGE_HEADER &header);
    static bool packetDataRx(QDataStream &data, const quint32 data_length, QByteArray &packet_data);
    static bool packetFooterRx(QDataStream &data, ISO_MESSAGE_FOOTER &footer);
    static qint64 streamPop6Bytes(QDataStream &data);
    static qint64 streamPush6Bytes(QDataStream &data);

    static quint64 gpsMStoUTCms(quint64 gps_time_ms)
    {
        return gps_time_ms + LEAP_MSEC_DIFF_UTC_GPS;
    }
    static quint64 utcMStoGPSms(quint64 utc_time_ms)
    {
        return utc_time_ms - LEAP_MSEC_DIFF_UTC_GPS;
    }

signals:
    void osem_processed(osem data);
    void ostm_processed(ostm data);
    void strt_processed(strt data);
    void dotm_processed(QVector<dotm_pt> data);
    void heab_processed(heab data);

private slots:
    void tcpPacketRx(QByteArray data);
    void tcpConnectionChanged(bool connected);
    void udpPacketRx();

private:
    TcpServerSimple *mTcpServer;
    QUdpSocket *mUdpSocket;
    QHostAddress mUdpHostAddress;
    quint16 mUdpPort;

    quint8 mTcpPacketState = 0;
    ISO_MESSAGE_HEADER mTcpMessageHeader;
    ISO_MESSAGE_FOOTER mTcpMessageFooter;
    QByteArray mTcpMessageData;

    int mHeabPollCnt;

    bool decodeMsg(quint8 type, quint32 len, QByteArray payload);

    bool processMessage(const QByteArray &data, const quint16 &msg_ID, const quint32 &msg_len, const quint8 &protocol_version);

    //bool getValidContent(void* data_loc,VByteArray &vb,uint16_t VALUE_ID, uint8_t TYPE_ID);

    // Message Process methods
    bool processOSEM(QDataStream &msg_stream, const quint32 &msg_len, osem &osem_msg);
    bool processDOTM(QDataStream &msg_stream, const quint32 &msg_len, QVector<dotm_pt> &dotm_pt);
    bool processOSTM(QDataStream &msg_stream, const quint32 &msg_len, ostm &ostm_msg);
    bool processSTRT(QDataStream &msg_stream, const quint32 &msg_len, strt &strt_msg);
    bool processHEAB(QDataStream &msg_stream, const quint32 &msg_len, heab &heab_msg);



};

#endif // ISOCOM_H
