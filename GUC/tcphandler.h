#ifndef TCPHANDLER_H
#define TCPHANDLER_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QDebug>
#include <QHostAddress>

#define TCP_STATE_DISCONNECTED 0x00
#define TCP_STATE_CONNECTED 0x01

// Handles one server or client TCP connection
class TCPhandler : public QObject
{
    Q_OBJECT

public:
    explicit TCPhandler(QObject *parent = nullptr);

    bool establishConnection(QString IP_addr, int port);

    bool listenForConnection(int port);

    static bool isValidIP(QString IP_addr);

signals:
    void debugComMsg(const QString &msg);
    void connectionChanged(const int &isConnected);
private:
    QTcpServer *mTcpServer = NULL;
    QTcpSocket *mTcpSocket = NULL;

private slots:
    void connectionEstablished();
    void connectionTerminated();
    void connectionStateChanged(QAbstractSocket::SocketState);
    void handleConnectionError(QAbstractSocket::SocketError);
};

#endif // TCPHANDLER_H
