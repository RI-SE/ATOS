#ifndef TCPHANDLER_H
#define TCPHANDLER_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QDebug>
#include <QHostAddress>

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
private:
    QTcpServer *mTcpServer = NULL;
    QTcpSocket *mTcpSocket = NULL;

private slots:
    void connectionEstablished();
    void handleConnectionError(QAbstractSocket::SocketError);
};

#endif // TCPHANDLER_H
