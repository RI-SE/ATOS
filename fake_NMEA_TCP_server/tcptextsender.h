#ifndef TCPTEXTSENDER_H
#define TCPTEXTSENDER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include <QThread>
#include <QFile>
#include "tcpserversimple.h"

class TCPTextSender : public QThread
{
    Q_OBJECT
public:
    TCPTextSender(uint16_t port,QString file, QObject *parent = 0);
    ~TCPTextSender();

    static int readTextFile(QString filepath,QVector<QString> &output);
    //void start();
    void run() override;

private slots:
    void handleConnectionChanged(bool);

private:
    TcpServerSimple *mTcpServer;
    bool isConnected = false;
    uint16_t port;
    //QLinkedList<QString> loadedTextFile;
    QString filepath;
    bool shutdown = false;
};

#endif // TCPTEXTSENDER_H
