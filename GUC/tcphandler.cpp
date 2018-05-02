#include "tcphandler.h"

TCPhandler::TCPhandler(QObject *parent)
{

}


bool TCPhandler::establishConnection(QString IP_addr, int port)
{
    if (!isValidIP(IP_addr)) return false;

    if (mTcpServer)
    {
        qDebug() << "Closing down TCP server connection.";
        mTcpServer->close();
        delete mTcpServer;
    }
    if (mTcpSocket)
    {
        qDebug() << "Closing down TCP socket connection.";
        mTcpSocket->close();
        delete mTcpSocket;
    }

    mTcpSocket = new QTcpSocket(this);

    connect(mTcpSocket,SIGNAL(connected()),
            this, SLOT(connectionEstablished()));
    connect(mTcpSocket,SIGNAL(error(QAbstractSocket::SocketError)),
            this,SLOT(handleConnectionError(QAbstractSocket::SocketError)));
    emit debugComMsg("Connecting to host...");


    mTcpSocket->connectToHost(IP_addr,port);


    return true;
}

bool TCPhandler::isValidIP(QString IP_addr)
{
    QHostAddress address(IP_addr);

    //qDebug() << "Address" << address.toString();

    return !address.isNull();//QAbstractSocket::IPv4Protocol == address.protocol();
}

bool TCPhandler::listenForConnection(int port)
{
    //TODO
    return true;
}


// PRIVATE SLOTS
void TCPhandler::connectionEstablished()
{
    QString msg = "Established TCP connection. IP: " +
            mTcpSocket->peerAddress().toString() + " port: " +
            QString::number(mTcpSocket->peerPort()) ;
    qDebug() << msg;

    emit debugComMsg(msg);
}

void TCPhandler::handleConnectionError(QAbstractSocket::SocketError error)
{
    QString msg = "Connection error: " + QString::number(error);
    qDebug() << msg;
    emit debugComMsg(msg);
}


