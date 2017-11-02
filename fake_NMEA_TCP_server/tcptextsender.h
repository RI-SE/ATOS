#ifndef TCPTEXTSENDER_H
#define TCPTEXTSENDER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include <QThread>
#include <QFile>
#include "client.h"

class TCPTextSender : public QThread
{
    Q_OBJECT
public:
    TCPTextSender(QString file, QObject *parent = 0);
    ~TCPTextSender();

    void connectToRec();
    static int readTextFile(QString filepath,QVector<QString> &output);
    //void start();
    void run() override;

signals:
    void sendData(QString);

public slots:
    void startTransfer();

    void quitApplication();

private slots:
    void handleConnected();
    void handleDisconnected();
    void handleSocketError(QAbstractSocket::SocketError);

    void handleSendData(QString);

private:
    QTcpSocket client;

    bool isConnected = false;
    uint16_t port;
    //QLinkedList<QString> loadedTextFile;
    QString filepath;
    bool shutdown = false;
};

#endif // TCPTEXTSENDER_H
