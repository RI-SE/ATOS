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

    // TODO: change this to use VByteArray instead of char looping

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
                //qDebug() << "All messages recieved!";
                MSG_REC = true;
            }
            break;
        case 10:
            info.CRC |= (uint8_t)c << 8;
            mPacketState++;
            break;
        case 11:
            info.CRC |= (uint8_t)c;
            //qDebug() << "Whole package recieved!";
            // Calculate CRC correct
            //if (info.CRC == 0)
                MSG_CRC_OK = true;
            break;
        default:
            break;
        }
        if (MSG_REC && MSG_CRC_OK)
        {
            //qDebug() << "processing messages";
            //process message
            processMessages(message_queue);
            //qDebug() << "T_ID:"<< info.TxID << "PKG_C:" << info.PACKAGE_COUNTER << "ACK_REQ:" << info.ACK_REQ;
        }
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

        PacketRx(datagram);
/*
        VByteArray vb(datagram);
        quint8 type = vb.vbPopFrontUint8();
        quint16 len = vb.vbPopFrontUint32();
        decodeMsg(type, len, vb);*/
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

bool ISOcom::processMessages(QByteArray data)
{
    VByteArray msg_data(data);
    uint16_t MSG_ID = 0;
    uint32_t MSG_NR_CONTENT = 0;
    while(msg_data.size()>2)
        //As long as MSG ID exists
    {
        MSG_ID = msg_data.vbPopFrontUint16();
        MSG_NR_CONTENT = msg_data.vbPopFrontUint32();
        switch (MSG_ID) {
        case ISO_MSG_DOTM:
        {
            if (MSG_NR_CONTENT % ISO_MSG_DOTM_POINT_NoC > 0)
            {
                qDebug() << "DOPM NoC is not consistent with the amount of points recieved.";
                return false;
            }
            QVector<dotm_pt> trajectory;
            dotm_pt point;
            uint32_t remaining_content = MSG_NR_CONTENT;
            while (remaining_content >= ISO_MSG_DOTM_POINT_NoC)
            {
                if(!getValidContent(&(point.rel_time),msg_data,ISO_VALUE_ID_REL_TIME,ISO_TYPE_ID_U32)) return false;
                if(!getValidContent(&(point.x),msg_data,ISO_VALUE_ID_X_POS,ISO_TYPE_ID_I32)) return false;
                if(!getValidContent(&(point.y),msg_data,ISO_VALUE_ID_X_POS,ISO_TYPE_ID_I32)) return false;
                if(!getValidContent(&(point.z),msg_data,ISO_VALUE_ID_X_POS,ISO_TYPE_ID_I32)) return false;
                if(!getValidContent(&(point.heading),msg_data,ISO_VALUE_ID_HEADING,ISO_TYPE_ID_U16)) return false;
                if(!getValidContent(&(point.lon_speed),msg_data,ISO_VALUE_ID_LON_SPEED,ISO_TYPE_ID_I16)) return false;
                if(!getValidContent(&(point.lat_speed),msg_data,ISO_VALUE_ID_LAT_SPEED,ISO_TYPE_ID_I16)) return false;
                if(!getValidContent(&(point.lon_acc),msg_data,ISO_VALUE_ID_LON_ACC,ISO_TYPE_ID_I16)) return false;
                if(!getValidContent(&(point.lat_acc),msg_data,ISO_VALUE_ID_LAT_ACC,ISO_TYPE_ID_I16)) return false;
                trajectory.append(point);
                remaining_content -= ISO_MSG_DOTM_POINT_NoC;
            }
            qDebug() << "DOTM received and handled.";
            emit dotm_processed(trajectory);
            break;

        }
        case ISO_MSG_OSEM:
        {
            if (MSG_NR_CONTENT != ISO_MSG_OSEM_NoC)
            {
                qDebug() << "OSEM NoC = " << ISO_MSG_OSEM_NoC << ", MSG NoC = " << MSG_NR_CONTENT;
                return false;
            }
            osem origin;
            if(!getValidContent(&(origin.lat),msg_data,ISO_VALUE_ID_LAT_POS,ISO_TYPE_ID_I32)) return false;
            if(!getValidContent(&(origin.lon),msg_data,ISO_VALUE_ID_LON_POS,ISO_TYPE_ID_I32)) return false;
            if(!getValidContent(&(origin.alt),msg_data,ISO_VALUE_ID_ALT_POS,ISO_TYPE_ID_I32)) return false;
            qDebug() << "OSEM received and handled.";
            emit osem_processed(origin);
            break;
        }
        case ISO_MSG_OSTM:
        {
            if (MSG_NR_CONTENT != ISO_MSG_OSTM_NoC)
            {
                qDebug() << "Number of Contents (NoC) not recognized.";
                qDebug() << "OSTM NoC = " << ISO_MSG_OSTM_NoC << ", MSG NoC = " << MSG_NR_CONTENT;
                return false;
            }
            ostm state;
            if(!getValidContent(&(state.state_change),msg_data,ISO_VALUE_ID_FLAG,ISO_TYPE_ID_U8)) return false;
            qDebug() << "OSTM received and handled.";
            qDebug() << "State change = " << state.state_change << "requested.";
            emit ostm_processed(state);
            break;
        }
        case ISO_MSG_STRT:
        {
            if (MSG_NR_CONTENT != ISO_MSG_STRT_NoC)
            {
                qDebug() << "STRT NoC = " << ISO_MSG_STRT_NoC << ", MSG NoC = " << MSG_NR_CONTENT;
                return false;
            }
            strt msg;
            if(!getValidContent(&(msg.abs_start_time),msg_data,ISO_VALUE_ID_ABS_TIME,ISO_TYPE_ID_U48)) return false;
            //uint64_t cTime = utility::getCurrentETSItimeMS();
            qDebug() << "STRT received and handled.";
            emit strt_processed(msg);
            break;
        }
        case ISO_MSG_HEAB:
        {
            if (MSG_NR_CONTENT != ISO_MSG_HEAB_NoC)
            {
                qDebug() << "HEAB NoC = " << ISO_MSG_HEAB_NoC << ", MSG NoC = " << MSG_NR_CONTENT;
                return false;
            }
            heab heartbeat;
            if(!getValidContent(&(heartbeat.tx_time),msg_data,ISO_VALUE_ID_ABS_TIME,ISO_TYPE_ID_U48)) return false;
            if(!getValidContent(&(heartbeat.cc_status),msg_data,ISO_VALUE_ID_FLAG,ISO_TYPE_ID_U8)) return false;
            //qDebug() << "HEAB received and handled.";
            emit heab_processed(heartbeat);
            break;
        }
        default:
            break;
        }
    }
    return true;
}

bool ISOcom::getValidContent(void *data_loc, VByteArray &vb,uint16_t VALUE_ID, uint8_t TYPE_ID)
{
    uint16_t read_VALUE_ID = vb.vbPopFrontUint16();
    uint8_t read_TYPE_ID = vb.vbPopFrontUint8();
    if(read_VALUE_ID == VALUE_ID && read_TYPE_ID == TYPE_ID)
    {
        switch (read_TYPE_ID) {
        case ISO_TYPE_ID_CHAR:
            *((char*)data_loc) = vb.vbPopFrontInt8();
            break;
        case ISO_TYPE_ID_U8:
            *((uint8_t*)data_loc) = vb.vbPopFrontUint8();
            break;
        case ISO_TYPE_ID_I8:
            *((int8_t*)data_loc) = vb.vbPopFrontInt8();
            break;
        case ISO_TYPE_ID_U16:
            *((uint16_t*)data_loc) = vb.vbPopFrontUint16();
            break;
        case ISO_TYPE_ID_I16:
            *((int16_t*)data_loc) = vb.vbPopFrontInt16();
            break;
        case ISO_TYPE_ID_U32:
            *((uint32_t*)data_loc) = vb.vbPopFrontUint32();
            break;
        case ISO_TYPE_ID_I32:
            *((int32_t*)data_loc) = vb.vbPopFrontInt32();
            break;
        case ISO_TYPE_ID_U48:
            *((uint64_t*)data_loc) = vb.vbPopFrontUint48();
            break;
        default:
        {
            qDebug() << "TypeID" << TYPE_ID << "does not exist.";
            return false;
        }
        }
    }
    else {
        qDebug() << "Value ID and Type ID does not match the valid combination.";
        return false;
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
