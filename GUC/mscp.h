#ifndef MSCPITERPRETER_H
#define MSCPITERPRETER_H
#include <QDebug>
#include <QByteArray>

#define ARM_CMD_STR "ArmScenario"
#define DISARM_CMD_STR "DisarmScenario"
#define START_CMD_STR "StartScenario"
#define ABORT_CMD_STR "AbortScenario"
#define GETSERVERSTATUS_CMD_STR "GetServerStatus"
#define INITIALIZE_OBJECT_CONTROL_CMD_STR "InitializeObjectControl"
#define CONNECT_OBJ_CMD_STR "ConnectObject"

namespace MSCP {



    void readServerResponse();

    bool msgToHTTPPOSTByteArray(const QString &IPaddress,const QString &msg, QByteArray &bytearray);

    bool build_Arm(const QString &IPaddress, QByteArray &bytearray);
    bool build_Disarm(const QString &IPaddress, QByteArray &bytearray);
    bool build_Start(const QString &IPaddress, int delayms, QByteArray &bytearray);
    bool build_Abort(const QString &IPaddress, QByteArray &bytearray);

    bool build_GetStatus(const QString &IPaddress, QByteArray &bytearray);
    bool build_InitializeObjectControl(const QString &IPaddress, QByteArray &bytearray);
    bool build_ConnectObject(const QString &IPaddress, QByteArray &bytearray);
}


#endif // MSCPITERPRETER_H
