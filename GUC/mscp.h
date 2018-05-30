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

#define RESPONSE_LENGTH_BYTES 4
#define RESPONSE_CODE_BYTES 2
#define HEADER_LENGTH_BYTES RESPONSE_LENGTH_BYTES + RESPONSE_CODE_BYTES

namespace MSCP {

    typedef struct
    {
        quint32 msg_length;
        qint16 code;
        qint8 msg_id;
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



    bool readServerResponseHeader(const QByteArray &data, response_header &header ,QByteArray &tail);

    bool readGetStatusMsg(const QByteArray &bytearray,qint16 &responsecode, server_status &status);

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
