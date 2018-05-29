#include "backend.h"


BackEnd::BackEnd(QObject *parent) :
    QObject(parent)
{

    mTcphandler = new TCPhandler();
    QObject::connect(mTcphandler, SIGNAL(debugComMsg(QString)),
                     this,SLOT(handleDebugComMsg(QString)));
    QObject::connect(mTcphandler,SIGNAL(connectionChanged(int)),
                     this,SLOT(handleConnectionChanged(int)));
    QObject::connect(mTcphandler, SIGNAL(receivedData(QByteArray)),
                     this,SLOT(handleReceivedData(QByteArray)));

}

//****************************************
// Public Q_INVOKABLE methods
//****************************************

bool BackEnd::sendArmToHost()
{
    QByteArray data;
    MSCP::build_Arm(m_hostName,data);
    sendToHost(data,-1);
    setSysCtrlStatus(2);
    return true;
}

bool BackEnd::sendDisarmToHost()
{
    QByteArray data;
    MSCP::build_Disarm(m_hostName,data);
    sendToHost(data,-1);
    return true;
}

bool BackEnd::sendStartToHost(int delayms)
{
    QByteArray data;
    MSCP::build_Start(m_hostName,delayms,data);
    sendToHost(data,-1);
    return true;
}
bool BackEnd::sendAbortToHost()
{
    QByteArray data;
    MSCP::build_Abort(m_hostName,data);
    sendToHost(data,-1);
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
    MSCP::build_InitializeScenario(m_hostName,data);
    sendToHost(data,-1);
    return true;
}

bool BackEnd::sendConnectObject()
{
    QByteArray data;
    MSCP::build_ConnectObject(m_hostName,data);
    sendToHost(data,-1);
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
    m_sysCtrlStatus = status;
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
    QString datetime_string = "["
            + QString::number(date.year()) + "/"
            + QString::number(date.month()) + "/"
            + QString::number(date.day())   + " "
            + QString::number(time.hour())  + ":"
            + QString::number(time.minute()) + ":"
            + QString::number(time.second()) + ":"
            + QString::number(time.msec()) + "]: ";

    emit newDebugMessage( datetime_string + msg );
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
    switch (expected_response_id) {
    case MSCP::SERVER_STATUS:
        MSCP::server_status status;
        qint16 response_code;
        if (MSCP::readGetStatusMsg(data,response_code,status))
        {
            setSysCtrlStatus(status.system_ctrl);
            setObjCtrlStatus(status.object_ctrl);
        }
        else
        {
            handleDebugMessage("Failed to read message.");
        }
        break;
    case -1:
        handleDebugMessage("Received unexpected response.");
        break;
    default:
        handleDebugMessage("Unknown response id.");
        break;
    }

}
