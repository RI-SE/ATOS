#include "backend.h"


BackEnd::BackEnd(QObject *parent) :
    QObject(parent)
{

    mTcphandler = new TCPhandler();
    QObject::connect(mTcphandler, SIGNAL(debugComMsg(QString)),
            this,SLOT(handleDebugComMsg(QString)));
    QObject::connect(mTcphandler,SIGNAL(connectionChanged(int)),
                     this,SLOT(handleConnectionChanged(int)));
}

QString BackEnd::hostName()
{
    qDebug() << "Host name fetched";
    return m_hostName;
}

void BackEnd::setHostName(const QString &hostName)
{
    qDebug() << "Setting host name.";
    if (hostName == m_hostName)
        return;
    m_addressValidity = addressValid(hostName);
    qDebug() << "IP_VALID:" << m_addressValidity;
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


// SLOTS
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
