#include "isocom.h"
#include <QDebug>
#include <cmath>

ISOcom::ISOcom(QObject *parent) : QObject(parent)
{
    //mPacket = 0;
    mTcpServer = new TcpServerSimple(this);
    mUdpSocket = new QUdpSocket(this);

    mUdpHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;

    mHeabPollCnt = 0;

    connect(mTcpServer, SIGNAL(dataRx(QByteArray)),
            this, SLOT(PacketRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool)),
            this, SLOT(tcpConnectionChanged(bool)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
}

//bool ISOcomstartServer(PacketInterface *packet)
bool ISOcom::startServer(int udpSocket, int tcpSocket)
{

    bool res = mTcpServer->startServer(tcpSocket);//53241

    if (!res) {
        qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
    }

    if (res) {
        mUdpSocket->close();
        res = mUdpSocket->bind(QHostAddress::Any, udpSocket); //53240
    }

    if (!res) {
        qWarning() << "Starting UDP server failed:" << mUdpSocket->errorString();
    }

    qDebug() << "Started CHRONOS client";
/*
    if (res && mPacket) {
        connect(mPacket, SIGNAL(stateReceived(quint8,CAR_STATE)),
                this, SLOT(stateReceived(quint8,CAR_STATE)));
    }
*/
    return res;
}

void ISOcom::PacketRx(QByteArray data)
{
    ISO_PACKAGE_INFO info;

    uint8_t mPacketState = 0;
    QByteArray message_queue;

    bool MSG_CRC_OK = false;
    bool MSG_REC = false;

    for (char c: data) {
        switch (mPacketState) {
        //---------------------------------------
        // Check for the sync word
        case 0:
            /*
            mTcpType = (quint8)c;
            mTcpLen = 0;
            mTcpData.clear();
            mPacketState++;
            */
            // Clear the data types
            message_queue.clear();
            info.ACK_REQ = 0;
            info.PACKAGE_COUNTER = 0;
            info.PACKAGE_LENGTH = 0;
            info.TxID = 0;


            mPacketState = (uint8_t)c && ISO_PART_SYNC_WORD ? mPacketState + 1 : 0;
            break;
        case 1:

            mPacketState = (uint8_t)c && ISO_PART_SYNC_WORD ? mPacketState + 1 : 0;
            break;
        //---------------------------------------
        // Transmitter ID
        case 2:
            info.TxID = (uint8_t) c;
            mPacketState++;
            break;
        //---------------------------------------
        // Package Counter
        case 3:
            info.PACKAGE_COUNTER = (uint8_t)c;
            mPacketState++;
            break;
        //---------------------------------------
        // Ack Request
        case 4:
            info.ACK_REQ = (uint8_t)c;
            mPacketState++;
            break;
        case 5:
            info.PACKAGE_LENGTH |= ((quint8)c) << 24;
            mPacketState++;
            break;
        case 6:
            info.PACKAGE_LENGTH |= ((quint8)c) << 16;
            mPacketState++;
            break;
        case 7:
            info.PACKAGE_LENGTH |= ((quint8)c) << 8;
            mPacketState++;
            break;
        case 8:
            info.PACKAGE_LENGTH |= ((quint8)c);
            mPacketState++;
            break;
        case 9:
            message_queue.append(c);
            if ((uint32_t)message_queue.size() >= info.PACKAGE_LENGTH) {
                mPacketState++;
                qDebug() << "All messages recieved!";
                MSG_REC = true;
            }
            break;
        case 10:
            info.CRC |= (uint8_t)c << 8;
            mPacketState++;
            break;
        case 11:
            info.CRC |= (uint8_t)c;
            qDebug() << "Whole package recieved!";
            // Calculate CRC correct
            MSG_CRC_OK = true;
            break;
        default:
            break;
        }
        if (MSG_REC && MSG_CRC_OK)
            qDebug() << "processing messages";
            //process message
    }
}

void ISOcom::tcpConnectionChanged(bool connected)
{
    if (connected) {
        qDebug() << "Chronos TCP connection accepted";
    } else {
        qDebug() << "Chronos TCP disconnected";
    }
}

void ISOcom::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                &mUdpHostAddress, &mUdpPort);

        VByteArray vb(datagram);
        quint8 type = vb.vbPopFrontUint8();
        quint16 len = vb.vbPopFrontUint32();
        decodeMsg(type, len, vb);
    }
}


/*
void ISOcom::stateReceived(quint8 id, CAR_STATE state)
{
    (void)id;
    (void)state;

    // TODO: Send monr message
}*/

bool ISOcom::decodeMsg(quint8 type, quint32 len, QByteArray payload)
{
    (void)type;
    (void)len;
    (void)payload;

    switch (type) {
    case CHRONOS_MSG_OSEM: {
        chronos_osem osem;
        VByteArray vb(payload);
        osem.lat = vb.vbPopFrontDouble32(1e7);
        osem.lon = vb.vbPopFrontDouble32(1e7);
        osem.alt = vb.vbPopFrontDouble32(1e2);
        osem.heading = vb.vbPopFrontDouble16(1e1);
        //processOsem(osem);
    } break;


    default:
        break;
    }

    return true;
}
/*
bool ISOcom::sendMonr(chronos_monr monr)
{
    if (QString::compare(mUdpHostAddress.toString(), "0.0.0.0") == 0) {
        return false;
    }

    VByteArray vb;
    vb.vbAppendInt8(CHRONOS_MSG_MONR);
    vb.vbAppendInt32(24);
    vb.vbAppendUint48(monr.ts);
    vb.vbAppendInt32((int32_t)(monr.lat * 1e7));
    vb.vbAppendInt32((int32_t)(monr.lon * 1e7));
    vb.vbAppendInt32((int32_t)(monr.alt * 1e2));
    vb.vbAppendUint16((uint16_t)(monr.speed * 1e2));
    vb.vbAppendUint16((uint16_t)(monr.heading * 1e1));
    vb.vbAppendUint8(monr.direction);
    vb.vbAppendUint8(monr.status);

    mUdpSocket->writeDatagram(vb, mUdpHostAddress, mUdpPort);

    return true;
}

bool ISOcom::sendTOM(chronos_tom tom)
{
    if (QString::compare(mUdpHostAddress.toString(), "0.0.0.0") == 0) {
        return false;
    }

    VByteArray vb;
    vb.vbAppendUint8(CHRONOS_MSG_TOM);
    vb.vbAppendUint32(8); // Not present in the current system
    vb.vbAppendUint8(tom.trigger_id);
    vb.vbAppendUint8(tom.trigger_type);
    vb.vbAppendUint48(tom.trigger_etsi_time);

    mUdpSocket->writeDatagram(vb, mUdpHostAddress, mUdpPort);

    return true;
}

*/
