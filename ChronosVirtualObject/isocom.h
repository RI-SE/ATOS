#ifndef ISOCOM_H
#define ISOCOM_H


#include <QObject>
#include <QUdpSocket>

#include "tcpserversimple.h"
//#include "packetinterface.h"
#include "datatypes.h"
#include "utility.h"
#include "vbytearray.h"

class ISOcom : public QObject
{
    Q_OBJECT
public:
    ISOcom(QObject *parent = 0);
    bool startServer(int udpSocket, int tcpSocket);
    //bool startServer(PacketInterface *packet);


    //bool sendMonr(chronos_monr monr);
    //bool sendTOM(chronos_tom tom);

signals:
    void osem_processed(osem data);
    void ostm_processed(ostm data);
    void strt_processed(strt data);
    void dotm_processed(QVector<dotm_pt> data);
    void heab_processed(heab data);
    /*
    void handle_osem(chronos_osem data);
    void handle_dopm(QVector<chronos_dopm_pt> dopm);
    void handle_heab(chronos_heab heab);
    void handle_ostm(chronos_ostm ostm);
    void handle_strt(chronos_strt strt);
    void handle_sypm(chronos_sypm sysm);
    void handle_mtsp(chronos_mtsp mtsp);
    void handle_tcm(chronos_tcm tcm);
    */
private slots:
    void PacketRx(QByteArray data);
    void tcpConnectionChanged(bool connected);
    void readPendingDatagrams();
    //void stateReceived(quint8 id, CAR_STATE state);

private:
    TcpServerSimple *mTcpServer;
    //PacketInterface *mPacket;
    QUdpSocket *mUdpSocket;
    QHostAddress mUdpHostAddress;
    quint16 mUdpPort;

    //int mTcpState;
    //quint8 mTcpType;
    //quint32 mTcpLen;l
    //QByteArray mTcpData;

    int mHeabPollCnt;

    bool decodeMsg(quint8 type, quint32 len, QByteArray payload);

    bool processMessages(QByteArray data);

    bool getValidContent(void* data_loc,VByteArray &vb,uint16_t VALUE_ID, uint8_t TYPE_ID);
    /*
    void processOsem(chronos_osem osem);

    void processDopm(QVector<chronos_dopm_pt> path);

    void processOstm(chronos_ostm ostm);
    void processStrt(chronos_strt strt);
    void processHeab(chronos_heab heab);
    void processSypm(chronos_sypm sypm);
    void processMtsp(chronos_mtsp mtsp);
    void processTCM(chronos_tcm tcm);
    */


};

#endif // ISOCOM_H
