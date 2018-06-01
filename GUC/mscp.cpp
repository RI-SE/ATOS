#include "mscp.h"

namespace MSCP {

qint8 mapMSCPCommand(QByteArray &textBuffer)
{
    QString command(textBuffer);

    if (QString::compare(command,GETSERVERSTATUS_CMD_STR) == 0) return SERVER_STATUS;
    else if (QString::compare(command,INITIALIZE_SCENARIO_CMD_STR) == 0) return INIT_SCENARIO;
    else if (QString::compare(command,CONNECT_OBJ_CMD_STR) == 0) return CONNECT_OBJ;
    else if (QString::compare(command,DISCONNECT_OBJ_CMD_STR) == 0) return DISCONNECT_OBJ;
    else if (QString::compare(command,INITIALIZE_SCENARIO_CMD_STR) == 0) return INIT_SCENARIO;
    else if (QString::compare(command,ARM_CMD_STR) == 0) return ARM;
    else if (QString::compare(command,START_CMD_STR) == 0) return START;
    else if (QString::compare(command,ABORT_CMD_STR) == 0) return ARM;
    else {
        qDebug() << "COMMAND: " << command;
        return -1;
    }

}

bool readServerResponseHeader(const QByteArray &data, response_header &header ,QByteArray &tail)
{

    int header_length = RESPONSE_LENGTH_BYTES + RESPONSE_CODE_BYTES;
    int array_size = data.size();
    if (array_size < header_length) return false;

    QDataStream stream(data);
    stream >> header.msg_length;
    stream >> header.code;

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

    // Find the command code
    header.msg_id = mapMSCPCommand(command);

    // Create the tail with data
    int tail_length = array_size - (header_length + command.size() + 1); // +1 for the colon
    tail = data.mid(array_size-tail_length,tail_length);

    return true;
}
/*
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
*/
void readGetStatusData(const QByteArray &data,server_status &status)
{
    QDataStream stream(data);
    stream >> status.system_ctrl;
    stream >> status.object_ctrl;
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
    return ARM;
}

qint8 build_Start(const QString &IPaddress, int delayms, QByteArray &bytearray)
{
    msgToHTTPPOSTByteArray(IPaddress,QString(START_CMD_STR) + "(" + QString::number(delayms) +");",bytearray);
    return START;
}

qint8 build_Abort(const QString &IPaddress, QByteArray &bytearray)
{
    msgToHTTPPOSTByteArray(IPaddress,QString(ABORT_CMD_STR) + "();",bytearray);
    return ABORT;
}

qint8 build_GetStatus(const QString &IPaddress, QByteArray &bytearray)
{
    msgToHTTPPOSTByteArray(IPaddress,QString(GETSERVERSTATUS_CMD_STR)+ "();", bytearray);
    return SERVER_STATUS;
}

qint8 build_InitializeScenario(const QString &IPaddress, QByteArray &bytearray)
{
    msgToHTTPPOSTByteArray(IPaddress,QString(INITIALIZE_SCENARIO_CMD_STR) + "();", bytearray);
    return INIT_SCENARIO;
}

qint8 build_ConnectObject(const QString &IPaddress, QByteArray &bytearray)
{
    msgToHTTPPOSTByteArray(IPaddress,QString(CONNECT_OBJ_CMD_STR) + "(-1);",bytearray);
    return CONNECT_OBJ;
}

qint8 build_DisconnectObject(const QString &IPaddress, QByteArray &bytearray)
{
    msgToHTTPPOSTByteArray(IPaddress,QString(DISCONNECT_OBJ_CMD_STR) + "();",bytearray);
    return DISCONNECT_OBJ;
}
}

