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

#ifndef TCPSERVERSIMPLE_H
#define TCPSERVERSIMPLE_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>

class TcpServerSimple : public QObject
{
    Q_OBJECT
public:
    explicit TcpServerSimple(QObject *parent = 0);
    bool startServer(int port);
    void stopServer();
    bool sendData(const QByteArray &data);

    void connectToRemote(QString IP_ADDR, int port);
    QString errorString();
    QString getIP();
    QString getPORT();

signals:
    void dataRx(const QByteArray &data);
    void connectionChanged(bool connected);

public slots:
    void newTcpConnection();
    void newTcpRemoteConnection();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void dataToSend(QByteArray &data);

    void handleHostFound();

private:
    QTcpServer *mTcpServer;
    QTcpSocket *mTcpSocket;

};

#endif // TCPSERVERSIMPLE_H
