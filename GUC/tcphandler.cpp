#include "tcphandler.h"



TCPhandler::TCPhandler(QObject *parent)
{

}


bool TCPhandler::establishConnection(QString IP_addr, int port)
{
    if (!isValidIP(IP_addr))
    {
        emit debugComMsg("Incorrect IP-address.");
        return false;
    }

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
    connect(mTcpSocket,SIGNAL(disconnected()),
            this,SLOT(connectionTerminated()));
    connect(mTcpSocket,SIGNAL(stateChanged(QAbstractSocket::SocketState)),
            this,SLOT(connectionStateChanged(QAbstractSocket::SocketState)));
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

bool TCPhandler::sendData(const QByteArray &data)
{
    if (mTcpSocket){
        mTcpSocket->write(data);
        return true;
    }
    return false;
}


// PRIVATE SLOTS
void TCPhandler::connectionEstablished()
{
    QString msg = "Established TCP connection. IP: " +
            mTcpSocket->peerAddress().toString() + " port: " +
            QString::number(mTcpSocket->peerPort()) ;
    //qDebug() << msg;

    connect(mTcpSocket,SIGNAL(readyRead()),
            this,SLOT(dataAvailable()));

    emit connectionChanged(TCP_STATE_CONNECTED);
    emit debugComMsg(msg);
}

void TCPhandler::connectionTerminated()
{
    QString msg = "Disconnected";
    qDebug() << msg;
    emit connectionChanged(TCP_STATE_DISCONNECTED);
    emit debugComMsg(msg);
}

void TCPhandler::connectionStateChanged(QAbstractSocket::SocketState state)
{

    //TODO: Add state text
    QString msg = "Connection State Changed: " + QString::number(state);
    qDebug() << msg;
    emit debugComMsg(msg);
}

void TCPhandler::handleConnectionError(QAbstractSocket::SocketError error)
{
    //TODO: Add error text
    QString msg = "Connection error: " + QString::number(error);
    qDebug() << msg;
    emit debugComMsg(msg);
}

void TCPhandler::dataAvailable()
{
    QByteArray input_data = mTcpSocket->readAll();
    emit receivedData(input_data);
    emit debugComMsg("DATA_RX: " + QString(input_data.toHex()));
}


