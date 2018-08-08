#ifndef IPCONNECITONHANDLER_H
#define IPCONNECITONHANDLER_H

#include <QObject>
#include <QTcpSocket>
#include <QTcpServer>
#include <QUdpSocket>

class IPConnecitonHandler : public QObject
{
    Q_OBJECT
public:
    explicit IPConnecitonHandler(QObject *parent = nullptr);
    ~IPConnecitonHandler(){
        if (mUdpSocket) delete mUdpSocket;
        if (mTcpSocket) delete mTcpSocket;
        if (mTcpServer) delete mTcpServer;
    }
    void destroy(){
        this->~IPConnecitonHandler();
    }
    bool startTCPserver(int port);
    //bool startTCPclient(QString IPaddress, int port);

signals:

public slots:

private:
    QUdpSocket *mUdpSocket = NULL;
    QTcpSocket *mTcpSocket = NULL;
    QTcpServer *mTcpServer = NULL;
};

#endif // IPCONNECITONHANDLER_H
