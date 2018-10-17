#include "tcphandler.h"



TCPhandler::TCPhandler(QObject *parent)
{
    ipaddr = new QHostAddress();
    mTcpSocket = new QTcpSocket(this);

    connect(mTcpSocket,SIGNAL(connected()),
            this, SLOT(connectionEstablished()));
    connect(mTcpSocket,SIGNAL(disconnected()),
            this,SLOT(connectionTerminated()));
    connect(mTcpSocket,SIGNAL(stateChanged(QAbstractSocket::SocketState)),
            this,SLOT(connectionStateChanged(QAbstractSocket::SocketState)));
    connect(mTcpSocket,SIGNAL(error(QAbstractSocket::SocketError)),
            this,SLOT(handleConnectionError(QAbstractSocket::SocketError)));
    connect(mTcpSocket,SIGNAL(readyRead()),
            this,SLOT(dataAvailable()));

}
TCPhandler::~TCPhandler(){
    delete ipaddr;
    delete mTcpSocket;
}


bool TCPhandler::establishConnection(QString IP_addr, int port)
{
    if (!ipaddr->setAddress(IP_addr))
    {
        emit debugComMsg("Incorrect IP-address: " + IP_addr);
        return false;
    }

    mTcpSocket->connectToHost(IP_addr,port);
    emit debugComMsg("Connecting to host [" + IP_addr + ";" + QString::number(port) + "]");
    return true;
}

void TCPhandler::closeConnection() {

    mTcpSocket->close();
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
    return false;
}

bool TCPhandler::sendData(const QByteArray &data)
{
    if (mTcpSocket){
        if(mTcpSocket->state() == QAbstractSocket::ConnectedState)
            mTcpSocket->write(data);
        //emit debugComMsg("Sending " + QString::number(data.length()) + "B:" + QString(data.toHex()));
        return true;
    }
    return false;
}

int TCPhandler::getConnectionState()
{
    return mTcpSocket->state();
}




// PRIVATE SLOTS
void TCPhandler::connectionEstablished()
{
    QString msg = "Established TCP connection."; /* +
            mTcpSocket->peerAddress().toString() + " port: " +
            QString::number(mTcpSocket->peerPort()) ;*/
    //qDebug() << msg;



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

    QString msg = "Connection State Changed: " + QString::number(state);
    qDebug() << msg;
    emit debugComMsg(msg);
}

void TCPhandler::handleConnectionError(QAbstractSocket::SocketError error)
{
    QString msg = "Connection error: " + QString::number(error);
    qDebug() << msg;
    emit debugComMsg(msg);
}

void TCPhandler::dataAvailable()
{
    QByteArray input_data = mTcpSocket->readAll();
    emit receivedData(input_data);
    //emit debugComMsg("DATA_RX: " + QString(input_data.toHex()));
}


