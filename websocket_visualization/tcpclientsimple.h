#ifndef TCPCLIENTSIMPLE_H
#define TCPCLIENTSIMPLE_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>

class TcpClientSimple : public QObject
{
    Q_OBJECT
public:
    TcpClientSimple(QObject *parent = nullptr);
    ~TcpClientSimple(){};

    void connectToServer();


signals:
    void dataRx(QByteArray);
    void connectionChanged(bool connected);
    void socketError(QAbstractSocket::SocketError);
private slots:
    void handleConnected();
    void handleDisconnected();
    void handleSocketError(QAbstractSocket::SocketError);
    void handleInputDataAvailable();
public slots:
private:
    QTcpSocket client;
};

#endif // TCPCLIENTSIMPLE_H
