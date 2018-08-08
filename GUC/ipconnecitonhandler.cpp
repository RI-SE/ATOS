#include "ipconnecitonhandler.h"

IPConnecitonHandler::IPConnecitonHandler(QObject *parent) : QObject(parent)
{
}

bool IPConnecitonHandler::startTCPserver(int port){

    if (mTcpServer) delete mTcpServer;

    mTcpServer = new QTcpServer(this);

    connect(mTcpServer,SIGNAL(newConnection()), this, SLOT());

    if (!mTcpServer->listen(QHostAddress::Any,  port)) {
        return false;
    }

    return true;
}
