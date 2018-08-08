#include "backend.h"


BackEnd::BackEnd(QObject *parent) :
    QObject(parent)
{

    mTcphandler = new TCPhandler();
    timer = new QTimer();
    QObject::connect(mTcphandler, SIGNAL(debugComMsg(QString)),
                     this,SLOT(handleDebugComMsg(QString)));
    QObject::connect(mTcphandler,SIGNAL(connectionChanged(int)),
                     this,SLOT(handleConnectionChanged(int)));
    QObject::connect(mTcphandler, SIGNAL(receivedData(QByteArray)),
                     this,SLOT(handleReceivedData(QByteArray)));
    QObject::connect(timer, SIGNAL(timeout()),
                     this,SLOT(update()));

    expected_response_id = new QLinkedList<qint8>();
    timer->start(500);

}
//****************************************
// Public Q_INVOKABLE methods
//****************************************

bool BackEnd::sendArmToHost()
{
    QByteArray data;
    qint8 id = MSCP::build_Arm(m_hostName,data);
    sendToHost(data,id);
    return true;
}



bool BackEnd::sendStartToHost(int delayms)
{
    QByteArray data;
    qint8 id = MSCP::build_Start(m_hostName,delayms,data);
    sendToHost(data,id);
    return true;
}
bool BackEnd::sendAbortToHost()
{
    QByteArray data;
    qint8 id = MSCP::build_Abort(m_hostName,data);
    sendToHost(data,id);
    return true;
}

bool BackEnd::sendGetStatus()
{
    QByteArray data;
    qint8 id = MSCP::build_GetStatus(m_hostName,data);
    sendToHost(data,id);
    return true;
}

bool BackEnd::sendInit()
{
    QByteArray data;
    qint8 id = MSCP::build_InitializeScenario(m_hostName,data);
    sendToHost(data,id);
    return true;
}

bool BackEnd::sendConnectObject()
{
    QByteArray data;
    qint8 id = MSCP::build_ConnectObject(m_hostName,data);
    sendToHost(data,id);
    return true;
}

bool BackEnd::sendDisconnectObject()
{
    QByteArray data;
    qint8 id = MSCP::build_DisconnectObject(m_hostName,data);
    sendToHost(data,id);
    return true;
}

//****************************************
// Public Q_Property methods
//****************************************
QString BackEnd::hostName()
{
    qDebug() << "Host name fetched";
    return m_hostName;
}

void BackEnd::setHostName(const QString &hostName)
{
    if (hostName == m_hostName)
        return;
    m_addressValidity = addressValid(hostName);
    m_hostName = hostName;
    emit hostNameChanged();
}

QString BackEnd::connectionText()
{
    return m_connectionText;
}

void BackEnd::setConnectionText(const QString &connectionText)
{
    m_connectionText = connectionText;
    emit connectionTextChanged();
}

int BackEnd::addressValidity()
{
    return m_addressValidity;
}

int BackEnd::sysCtrlStatus()
{
    return m_sysCtrlStatus;
}

void BackEnd::setSysCtrlStatus(int status)
{
    if (status == m_sysCtrlStatus) return;

    switch (status) {
    case 5:
        m_sysCtrlStatus = 3;
        break;
    case 6:
        m_sysCtrlStatus = 4;
        break;
    default:
        m_sysCtrlStatus = status;
        break;
    }
    emit sysCtrlStatusChanged();
}

int BackEnd::objCtrlStatus()
{
    return m_objCtrlStatus;
}

void BackEnd::setObjCtrlStatus(int status)
{
    if (status == m_objCtrlStatus) return;
    m_objCtrlStatus = status;
    emit objCtrlStatusChanged();
}

//****************************************
// Public common methods
//****************************************


void BackEnd::handleDebugMessage(const QString &msg)
{

    //QDateTime datetime = QDateTime::currentDateTime();
    QDate date = QDate::currentDate();
    QTime time = QTime::currentTime();
    QString datetime_string = "[DEBUG]["
            + QString::number(date.year()) + "/"
            + QString::number(date.month()) + "/"
            + QString::number(date.day())   + " "
            + QString::number(time.hour())  + ":"
            + QString::number(time.minute()) + ":"
            + QString::number(time.second()) + ":"
            + QString::number(time.msec()) + "]: ";

    qDebug() << datetime_string << msg;
    emit newDebugMessage( datetime_string + msg );
}

void BackEnd::handleRecDataDebugMessage(const QString &msg)
{
    handleDebugMessage("[TCP_REC]"+msg);
}

//****************************************
// SLOTS
//****************************************
void BackEnd::handleDebugComMsg(const QString &msg)
{
    setConnectionText(msg);
    handleDebugMessage(msg);
}

void BackEnd::handleConnectionChanged(const int &isConnected)
{
    switch (isConnected) {
    case TCP_STATE_CONNECTED:
        emit enterStartScreen();
        break;
    case TCP_STATE_DISCONNECTED:
        emit enterConnectionScreen();
        break;
    default:
        break;
    }
}

void BackEnd::handleReceivedData(const QByteArray &data)
{

    MSCP::response_header header;
    QByteArray command_data;

    qint8 HEADER_ERROR_CODE = MSCP::readServerResponseHeader(data,header,command_data);

    switch (HEADER_ERROR_CODE) {
    case MSCP::HEADER_OK:

        if (!isExpectedResponse(header.msg_id))
        {
            handleDebugMessage("Received unexpected response. ID:" + QString::number(header.msg_id));
            return;
        }
        //handleDebugMessage("Command received: " + QString(header.command_text));
        break;

    case MSCP::HEADER_LENGTH_ERROR:
        handleDebugMessage("Invalid header format: Length too short.");
        return;
    case MSCP::HEADER_MSG_LENGTH_ERROR:
        handleDebugMessage("Packet length error: Length too short.");
        return;
    case MSCP::HEADER_COMMAND_NOT_FOUND:
        handleDebugMessage("Packet contains no command.");
        handleDebugMessage("REC_DATA: " + QString(command_data));
        return;
    case MSCP::HEADER_INVALID_MSG_ID:
        // Most likely log data from server
        handleDebugMessage("Unrecognized command.");
        handleDebugMessage("REC_DATA: " + QString(command_data));
        return;
    default:

        return;
    }




    switch (header.msg_id) {
    case MSCP::SERVER_STATUS:
        MSCP::server_status status;
        MSCP::readGetStatusData(command_data,status);

        setSysCtrlStatus(status.system_ctrl);
        setObjCtrlStatus(status.object_ctrl);

        break;
    case MSCP::INIT_SCENARIO:
        //sendGetStatus();
        handleDebugMessage("INIT SCENARIO");
        //sendConnectObject();
        break;
    case MSCP::CONNECT_OBJ:
        handleDebugMessage("CONNECT");
        //sendGetStatus();
        break;
    case MSCP::ARM:
        handleDebugMessage("ARM");
        //sendGetStatus();
        break;
    case MSCP::START:
        handleDebugMessage("START");
        //sendGetStatus();
        break;
    case MSCP::ABORT:
        handleDebugMessage("ABORT");
        //sendGetStatus();
    default:
        handleDebugMessage("Unknown response id.");
        //sendGetStatus();
        break;
    }
}

void BackEnd::update()
{
    if (mTcphandler->isConnected())
    {
        sendGetStatus();
    }

    if (m_sysCtrlStatus == MSCP_SYSTEM_CONTROL_STATE_IDLE &&
            m_objCtrlStatus == MSCP_OBJECT_CONTROL_STATE_INITIALIZED)
    {
        sendConnectObject();
    }
}
