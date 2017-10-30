/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "tcpserversimple.h"
#include <QDebug>

TcpServerSimple::TcpServerSimple(QObject *parent) : QObject(parent)
{
    mTcpServer = new QTcpServer(this);
    mTcpSocket = 0;

    connect(mTcpServer, SIGNAL(newConnection()), this, SLOT(newTcpConnection()));
}

bool TcpServerSimple::startServer(int port)
{
    if (!mTcpServer->listen(QHostAddress::Any,  port)) {
        return false;
    }
    qDebug() << "Starting server on port " << QString::number(port);
    return true;
}

void TcpServerSimple::stopServer()
{

    mTcpServer->close();

    if (mTcpSocket) {
        mTcpSocket->close();
        delete mTcpSocket;
        mTcpSocket = 0;
        emit connectionChanged(false);
    }
    qDebug() << "Closing server.";
}

bool TcpServerSimple::sendData(const QByteArray &data)
{
    bool res = false;

    if (mTcpSocket) {
        mTcpSocket->write(data);
        res = true;
    }

    return res;
}

QString TcpServerSimple::errorString()
{
    return mTcpServer->errorString();
}

QString TcpServerSimple::getIP()
{
    QHostAddress IP = mTcpServer->serverAddress();
    return IP.toString();
}

QString TcpServerSimple::getPORT()
{
    return QString::number(mTcpSocket->peerPort());
}

void TcpServerSimple::newTcpConnection()
{
    QTcpSocket *socket = mTcpServer->nextPendingConnection();

    if (mTcpSocket) {
        socket->close();
        delete socket;
    } else {
        mTcpSocket = socket;

        if (mTcpSocket) {
            connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
            connect(mTcpSocket, SIGNAL(disconnected()),
                    this, SLOT(tcpInputDisconnected()));
            connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
                    this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
            emit connectionChanged(true);
        }
    }
}

void TcpServerSimple::tcpInputDisconnected()
{
    mTcpSocket->deleteLater();
    mTcpSocket = 0;
    emit connectionChanged(false);
}

void TcpServerSimple::tcpInputDataAvailable()
{
    QByteArray data = mTcpSocket->readAll();
    emit dataRx(data);
}

void TcpServerSimple::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;
    mTcpSocket->close();
    delete mTcpSocket;
    mTcpSocket = 0;
    emit connectionChanged(false);
}

void TcpServerSimple::dataToSend(QByteArray &data)
{
    sendData(data);
}

