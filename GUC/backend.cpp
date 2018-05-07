#include "backend.h"


BackEnd::BackEnd(QObject *parent) :
    QObject(parent)
{
    //IPConnecitonHandler *iph = new IPConnecitonHandler();
    //iph->destroy();
    mTcphandler = new TCPhandler();
    //mTcphandler->establishConnection("127.0.0.1",53241);
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


// SLOTS
void BackEnd::handleDebugComMsg(const QString &msg)
{
    setConnectionText(msg);
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
