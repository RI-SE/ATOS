#ifndef MSCPITERPRETER_H
#define MSCPITERPRETER_H
#include <QDebug>
#include <QByteArray>

#define ARM_CMD_STR "ArmScenario"
#define DISARM_CMD_STR "DisarmScenario"
#define START_CMD_STR "StartScenario"
#define ABORT_CMD_STR "AbortScenario"

namespace MSCP {



    void readServerResponse();

    bool msgToHTTPPOSTByteArray(const QString &IPaddress,const QString &msg, QByteArray &bytearray);

    bool buildArmMsgByteArray(const QString &IPaddress, QByteArray &bytearray);
    bool buildDisarmMsgByteArray(const QString &IPaddress, QByteArray &bytearray);
    bool buildStartMsgByteArray(const QString &IPaddress, int delayms, QByteArray &bytearray);
    bool buildAbortMsgByteArray(const QString &IPaddress, QByteArray &bytearray);
}


#endif // MSCPITERPRETER_H
