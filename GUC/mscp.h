#ifndef MSCPITERPRETER_H
#define MSCPITERPRETER_H
#include <QDebug>
#include <QByteArray>
#include <QDataStream>

#define ARM_CMD_STR "ArmScenario"
#define DISARM_CMD_STR "DisarmScenario"
#define START_CMD_STR "StartScenario"
#define ABORT_CMD_STR "AbortScenario"
#define GETSERVERSTATUS_CMD_STR "GetServerStatus"
#define INITIALIZE_SCENARIO_CMD_STR "InitializeScenario"
#define CONNECT_OBJ_CMD_STR "ConnectObject"
#define DISCONNECT_OBJ_CMD_STR "DisconnectObject"

#define MSCP_SYSTEM_CONTROL_STATE_INITIALIZED 1
#define MSCP_SYSTEM_CONTROL_STATE_IDLE 2
#define MSCP_SYSTEM_CONTROL_STATE_INWORK 5
#define MSCP_SYSTEM_CONTROL_STATE_ERROR 6

#define MSCP_OBJECT_CONTROL_STATE_IDLE 1
#define MSCP_OBJECT_CONTROL_STATE_INITIALIZED 2
#define MSCP_OBJECT_CONTROL_STATE_CONNECTED 3
#define MSCP_OBJECT_CONTROL_STATE_ARMED 4
#define MSCP_OBJECT_CONTROL_STATE_RUNNING 5
#define MSCP_OBJECT_CONTROL_STATE_ERROR 6

#define RESPONSE_LENGTH_BYTES 4
#define RESPONSE_CODE_BYTES 2
#define RESPONSE_COMMAND_TEXT_BYTES 100

namespace MSCP {

    typedef struct
    {
        quint32 msg_length;
        qint16 code;
        qint8 msg_id;
        char command_text[RESPONSE_COMMAND_TEXT_BYTES];
    }response_header;

    typedef struct
    {
        qint8 system_ctrl;
        qint8 object_ctrl;
    } server_status;

    enum RESPONSE_ID
    {
        SERVER_STATUS = 0,
        INIT_SCENARIO,
        CONNECT_OBJ,
        DISCONNECT_OBJ,
        ARM,
        START,
        ABORT
    };

    enum HEADER_ERROR_CODE
    {
        HEADER_OK = 0,
        HEADER_LENGTH_ERROR,
        HEADER_COMMAND_NOT_FOUND,
        HEADER_INVALID_MSG_ID,
        HEADER_MSG_LENGTH_ERROR
    };

    qint8 mapMSCPCommand(QByteArray &textBuffer);

    int readServerResponseHeader(const QByteArray &data, response_header &header ,QByteArray &tail);

    //bool readGetStatusMsg(const QByteArray &bytearray,qint16 &responsecode, server_status &status);

    void readGetStatusData(const QByteArray &data,server_status &status);

    bool msgToHTTPPOSTByteArray(const QString &IPaddress,const QString &msg, QByteArray &bytearray);

    // Done with new iplementation


    qint8 build_InitializeScenario(const QString &IPaddress, QByteArray &bytearray);
    qint8 build_GetStatus(const QString &IPaddress, QByteArray &bytearray);



    // Not done
    qint8 build_Arm(const QString &IPaddress, QByteArray &bytearray);

    qint8 build_Start(const QString &IPaddress, int delayms, QByteArray &bytearray);
    qint8 build_Abort(const QString &IPaddress, QByteArray &bytearray);


    qint8 build_ConnectObject(const QString &IPaddress, QByteArray &bytearray);
    qint8 build_DisconnectObject(const QString &IPaddress, QByteArray &bytearray);
}


#endif // MSCPITERPRETER_H
