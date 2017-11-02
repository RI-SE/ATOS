#include "tcpclientsimple.h"

TcpClientSimple::TcpClientSimple(QObject *parent) : QObject(parent)
{
    connect(&client,SIGNAL(connected()),this,SLOT(handleConnected()));
    connect(&client,SIGNAL(disconnected()),this,SLOT(handleDisconnected()));
    connect(&client,SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(handleSocketError(QAbstractSocket::SocketError)));
    connect(&client,SIGNAL(readyRead()),this,SLOT(handleInputDataAvailable()));
}
void TcpClientSimple::connectToServer()
{
    QString IP = "127.0.0.1";
    int port = 2948;
    QHostAddress addr(IP);
    client.connectToHost(addr, port);
    //qDebug() << "Connecting to server on " << IP << ":" << port;
}

void TcpClientSimple::handleConnected()
{
    //qDebug() << "Connected to server.";
    emit connectionChanged(true);
}

void TcpClientSimple::handleDisconnected()
{
    //qDebug() << "Disconnected from server.";
    emit connectionChanged(false);
}

void TcpClientSimple::handleSocketError(QAbstractSocket::SocketError error)
{
    emit socketError(error);
    //qDebug() << "Socket error " << error;
/*
    switch (error) {
    case QAbstractSocket::ConnectionRefusedError:
        //msleep(1000);
        //qDebug() << "Connection Refused.";
        connectToServer();
        break;
    default:
        break;
    }
    */

}

void TcpClientSimple::handleInputDataAvailable()
{
    QByteArray data = client.readAll();
    emit dataRx(data);
}
