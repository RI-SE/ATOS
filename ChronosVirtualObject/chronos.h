#ifndef CHRONOS_H
#define CHRONOS_H

#include <QObject>
#include <QUdpSocket>

#include "tcpserversimple.h"
//#include "packetinterface.h"
#include "datatypes.h"
#include "vbytearray.h"

class Chronos : public QObject
{
    Q_OBJECT
public:
    Chronos(QObject *parent = 0);
    bool startServer(int udpSocket, int tcpSocket);
    //bool startServer(PacketInterface *packet);


    bool sendMonr(chronos_monr monr);

signals:
    void handle_osem(chronos_osem data);
    void handle_dopm(QVector<chronos_dopm_pt>);
    void handle_heab(chronos_heab heab);

private slots:
    void tcpRx(QByteArray data);
    void tcpConnectionChanged(bool connected);
    void readPendingDatagrams();
    //void stateReceived(quint8 id, CAR_STATE state);

private:
    TcpServerSimple *mTcpServer;
    //PacketInterface *mPacket;
    QUdpSocket *mUdpSocket;
    QHostAddress mUdpHostAddress;
    quint16 mUdpPort;

    int mTcpState;
    quint8 mTcpType;
    quint32 mTcpLen;
    QByteArray mTcpData;

    int mHeabPollCnt;

    bool decodeMsg(quint8 type, quint32 len, QByteArray payload);

    void processDopm(QVector<chronos_dopm_pt> path);
    void processOsem(chronos_osem osem);
    void processOstm(chronos_ostm ostm);
    void processStrt(chronos_strt strt);
    void processHeab(chronos_heab heab);


};

#endif // CHRONOS_H
