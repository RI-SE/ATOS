#include "mscp.h"

namespace MSCP {



    bool readServerResponse(QByteArray &bytearray)
    {


        // TODO
        qDebug() << "Read Server Response.";

        return true;
    }
    bool readGetStatusMsg(const QByteArray &bytearray,qint16 &responsecode, server_status &status)
    {
        int header_length = RESPONSE_LENGTH_BYTES + RESPONSE_CODE_BYTES;
        int array_size = bytearray.size();
        if (array_size < header_length) return false;

        quint32 msg_length = 0;
        //QDataStream stream(bytearray.mid(0,header_length));
        QDataStream stream(bytearray);
        stream >> msg_length;
        stream >> responsecode;

        qint8 letter;

        QByteArray command;
        bool command_found = false;

        while(!stream.atEnd())
        {
            stream >> letter;
            if ((char)letter == ':')
            {
                command_found = true;
                break;
            }
            command.push_back(letter);
        }

        if (!command_found) return false;

        if (QString::compare(command,GETSERVERSTATUS_CMD_STR) == 0)
        {
            stream >> status.system_ctrl;
            stream >> status.object_ctrl;
            return true;
        }

        return false;

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

    qint8 build_Arm(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(ARM_CMD_STR) + "();",bytearray);
        return true;
    }
    qint8 build_Disarm(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(DISARM_CMD_STR) + "();",bytearray);
        return true;
    }
    qint8 build_Start(const QString &IPaddress, int delayms, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(START_CMD_STR) + "(" + QString::number(delayms) +");",bytearray);
        return true;
    }

    qint8 build_Abort(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(ABORT_CMD_STR) + "();",bytearray);
        return true;
    }

    qint8 build_GetStatus(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(GETSERVERSTATUS_CMD_STR)+ "();", bytearray);
        return SERVER_STATUS;
    }

    qint8 build_InitializeScenario(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(INITIALIZE_SCENARIO_CMD_STR) + "();", bytearray);
        return true;
    }

    qint8 build_ConnectObject(const QString &IPaddress, QByteArray &bytearray)
    {
        msgToHTTPPOSTByteArray(IPaddress,QString(CONNECT_OBJ_CMD_STR) + "(-1);",bytearray);
        return true;
    }
}

