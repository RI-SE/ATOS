#include "mscp.h"

namespace MSCP {
    void readServerResponse()
    {
        // TODO
        qDebug() << "Read Server Response.";
    }

    bool msgToHTTPPOSTByteArray(const QString &IPaddress, const QString &msg, QByteArray &bytearray)
    {
        // TODO

        QString string_holder ="";
        string_holder += "POST /maestro HTTP/1.1\n";
        string_holder += "Host: " + IPaddress + "\n";
        string_holder += msg;
        qDebug() << string_holder;

        bytearray.clear();
        bytearray = string_holder.toLocal8Bit();
        return true;
    }

    bool build_Arm(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(ARM_CMD_STR) + "();",bytearray);
        return true;
    }
    bool build_Disarm(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(DISARM_CMD_STR) + "();",bytearray);
        return true;
    }
    bool build_Start(const QString &IPaddress, int delayms, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(START_CMD_STR) + "(" + QString::number(delayms) +");",bytearray);
        return true;
    }

    bool build_Abort(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(ABORT_CMD_STR) + "();",bytearray);
        return true;
    }

    bool build_GetStatus(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(GETSERVERSTATUS_CMD_STR)+ "();", bytearray);
        return true;
    }

    bool build_InitializeObjectControl(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(INITIALIZE_OBJECT_CONTROL_CMD_STR) + "();", bytearray);
        return true;
    }

    bool build_ConnectObject(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(CONNECT_OBJ_CMD_STR) + "(-1);",bytearray);
        return true;
    }
}

