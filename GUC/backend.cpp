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
}

QString BackEnd::userName()
{
    return m_userName;
}

void BackEnd::setUserName(const QString &userName)
{
    if (userName == m_userName)
        return;

    m_userName = userName;
    emit userNameChanged();
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

// SLOTS
void BackEnd::handleDebugComMsg(const QString &msg)
{
    setConnectionText(msg);
}
