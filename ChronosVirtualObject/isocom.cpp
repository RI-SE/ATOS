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
            this, SLOT(tcpPacketRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool)),
            this, SLOT(tcpConnectionChanged(bool)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(udpPacketRx()));
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
    return res;
}

bool ISOcom::packetHeaderRx(QDataStream &data, ISO_MESSAGE_HEADER &header)
{
    //if (data.size() < ISO_MESSAGE_HEADER_BYTE_LENGTH) return false;

    quint8 one_byte;
    quint8 packetState = 0;
    header.TxID = 0;
    header.MESSAGE_COUNTER = 0;
    header.ACK_REQ = 0;
    header.PROTOCOL_VERSION = 0;
    header.MESSAGE_ID = 0;
    header.MESSAGE_LENGTH = 0;

    while( !data.atEnd() )
    {
        switch (packetState) {
        case 0:
            data >> one_byte;
            packetState = one_byte && ISO_PART_SYNC_WORD ? packetState + 1 : packetState;
            break;
        case 1:
            data >> one_byte;
            packetState = one_byte && ISO_PART_SYNC_WORD ? packetState + 1 : 0;
            break;
        case 2:
            data >> header.TxID;
            packetState++;
            break;
        case 3:
            data >> header.MESSAGE_COUNTER;
            packetState++;
            break;
        case 4:
            data >> one_byte;
            header.ACK_REQ = one_byte >> 7;
            header.PROTOCOL_VERSION = one_byte & 127;
            packetState++;
            break;
        case 5:
            data >> header.MESSAGE_ID;
            packetState++;
            break;
        case 6:
            data >> header.MESSAGE_LENGTH;
            return true;
        default:
            break;
        }
    }
    return false;
}

bool ISOcom::packetDataRx(QDataStream &data, quint32 data_length, QByteArray &packet_data)
{
    quint8 read_byte = 0;
    quint32 bytes_read = 0;
    while(!data.atEnd())
    {
        data >> read_byte;
        packet_data.append(read_byte);
        bytes_read++;
        if(bytes_read >= data_length) return true;
    }
    return false;
}

bool ISOcom::packetFooterRx(QDataStream &data, ISO_MESSAGE_FOOTER &footer)
{
    data >> footer.CRC;
    return data.status() == QDataStream::Ok;
}

void ISOcom::tcpPacketRx(QByteArray data)
{

    QDataStream stream(data);
    stream.setByteOrder(QDataStream::LittleEndian);


    switch (mTcpPacketState) {
    case ISO_TCP_STATE_READ_HEADER:
        if(!packetHeaderRx(stream,mTcpMessageHeader)) break;
        mTcpMessageData.clear();
        mTcpPacketState = ISO_TCP_STATE_READ_DATA;
    case ISO_TCP_STATE_READ_DATA:
        if(!packetDataRx(stream,mTcpMessageHeader.MESSAGE_LENGTH,mTcpMessageData)) break;
        mTcpPacketState = ISO_TCP_STATE_READ_FOOTER;
    case ISO_TCP_STATE_READ_FOOTER:
        packetFooterRx(stream,mTcpMessageFooter);
        mTcpPacketState = ISO_TCP_STATE_READ_HEADER;
        if(mTcpMessageFooter.CRC == 0){
            processMessage( mTcpMessageData,
                            mTcpMessageHeader.MESSAGE_ID,
                            mTcpMessageHeader.MESSAGE_LENGTH,
                            mTcpMessageHeader.PROTOCOL_VERSION);
        }
        else
        {
            qDebug() << "CRC not correct:" << mTcpMessageFooter.CRC;
        }
        break;
    default:
        break;
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

void ISOcom::udpPacketRx()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &mUdpHostAddress, &mUdpPort);

        ISO_MESSAGE_HEADER header;
        QByteArray msg_data;
        ISO_MESSAGE_FOOTER footer;
        QDataStream stream(datagram);
        stream.setByteOrder(QDataStream::LittleEndian);

        if(!packetHeaderRx(stream, header)) continue;
        if(!packetDataRx(stream,header.MESSAGE_LENGTH,msg_data)) continue;
        if(!packetFooterRx(stream,footer)) continue;

        // TODO: add crc check
        processMessage(msg_data,header.MESSAGE_ID,header.MESSAGE_LENGTH,header.PROTOCOL_VERSION);

    }
}


qint64 ISOcom::streamPop6Bytes(QDataStream &data)
{

    qint64 return_val = 0;
    quint8 readData[6];
    //if (data.readRawData(readData,6) < 0){ return 0; qDebug() << "Unable to read data.";}

    data >> readData[0];
    data >> readData[1];
    data >> readData[2];
    data >> readData[3];
    data >> readData[4];
    data >> readData[5];

    if (data.byteOrder() == QDataStream::BigEndian)
    {
        return_val = static_cast<quint64>(readData[0]) << 40;
        return_val |= static_cast<quint64>(readData[1]) << 32;
        return_val |= static_cast<quint64>(readData[2]) << 24;
        return_val |= static_cast<quint64>(readData[3]) << 16;
        return_val |= static_cast<quint64>(readData[4]) << 8;
        return_val |= static_cast<quint64>(readData[5]);
    }
    else
    {
        return_val = static_cast<quint64>(readData[0]) ;
        return_val |= static_cast<quint64>(readData[1]) << 8;
        return_val |= static_cast<quint64>(readData[2]) << 16;
        return_val |= static_cast<quint64>(readData[3]) << 24;
        return_val |= static_cast<quint64>(readData[4]) << 32;
        return_val |= static_cast<quint64>(readData[5]) << 40;
    }
    return return_val;
}

bool ISOcom::processMessage(const QByteArray &data,const quint16 &msg_id,const quint32 &msg_len,const quint8 &protocol_version)
{
    QDataStream msg_data(data);
    msg_data.setByteOrder(QDataStream::LittleEndian);

    switch (msg_id) {
    case ISO_MSG_HEAB:
    {
        heab heab_msg;
        if (!processHEAB(msg_data,msg_len,heab_msg)) return false;
        emit heab_processed(heab_msg);
        break;
    }
    case ISO_MSG_OSEM:
    {
        osem osem_msg;
        if (!processOSEM(msg_data,msg_len,osem_msg)) return false;
        emit osem_processed(osem_msg);
        break;
    }
    case ISO_MSG_DOTM:
    {
        qDebug() << "DOTM Registered";
        QVector<dotm_pt> traj;
        if (!processDOTM(msg_data,msg_len,traj)) return false;
        emit dotm_processed(traj);
        break;
    }
    case ISO_MSG_OSTM:
    {
        ostm ostm_msg;
        if (!processOSTM(msg_data,msg_len,ostm_msg)) return false;
        emit ostm_processed(ostm_msg);
        break;
    }
    case ISO_MSG_STRT:
    {
        strt strt_msg;
        if (!processSTRT(msg_data,msg_len,strt_msg)) return false;
        emit strt_processed(strt_msg);
        break;
    }
    default:
    {
        qDebug() << "Unknown message recieved";
        return false;
    }
    }
    //}
    return true;
}


bool ISOcom::processOSEM(QDataStream &msg_stream, const quint32 &msg_len, osem &osem_msg)
{
    quint32 remaining_content = msg_len;

    quint16 value_id = 0;
    quint16 content_len = 0;

    //const quint16 expected_mandatory_content = 0x01FF;
    qint16 mandatory_content_check = 0;

    while(!msg_stream.atEnd())
    {
        if (remaining_content < ISO_MIN_CONTENT_DATA) {
            qDebug() << "OSEM_ERROR: Not enough data to fill content";
            return false;
        }
        // Read Content Header
        msg_stream >> value_id;
        msg_stream >> content_len;
        remaining_content -= (content_len + sizeof(value_id) + sizeof(content_len));

        switch (value_id) {
        case ISO_VALUE_ID_LAT:
            osem_msg.lat = streamPop6Bytes(msg_stream);
            mandatory_content_check |= CONTENT_BINARY_ID_0;
            break;
        case ISO_VALUE_ID_LON:
            osem_msg.lon = streamPop6Bytes(msg_stream);
            mandatory_content_check |= CONTENT_BINARY_ID_1;
            break;
        case ISO_VALUE_ID_ALT:
            msg_stream >> osem_msg.alt;
            mandatory_content_check |= CONTENT_BINARY_ID_2;
            break;
        case ISO_VALUE_ID_DateISO8601:
            msg_stream >> osem_msg.dateISO8601;
            mandatory_content_check |= CONTENT_BINARY_ID_3;
            break;
        case ISO_VALUE_ID_GPS_WEEK:
            msg_stream >> osem_msg.GPSweek;
            mandatory_content_check |= CONTENT_BINARY_ID_4;
            break;
        case ISO_VALUE_ID_GPS_SEC_OF_WEEK:
            msg_stream >> osem_msg.GPSsecOfWeek;
            mandatory_content_check |= CONTENT_BINARY_ID_5;
            break;
        case ISO_VALUE_ID_MAX_WAY_DEV:
            msg_stream >> osem_msg.MaxWayDev;
            mandatory_content_check |= CONTENT_BINARY_ID_6;
            break;
        case ISO_VALUE_ID_MAX_LATERAL_DEV:
            msg_stream >> osem_msg.MaxLatDev;
            mandatory_content_check |= CONTENT_BINARY_ID_7;
            break;
        case ISO_VALUE_ID_MIN_POS_ACCURACY:
            msg_stream >> osem_msg.MinPosAccuracy;
            mandatory_content_check |= CONTENT_BINARY_ID_8;
            break;
        default:
            msg_stream.skipRawData(content_len);
            qDebug() << "OSEM_WARNING: Unrecognized content. ID:" << value_id;
            break;
        }
    }
    // Print OSEM
    qDebug() << "OSEM:" <<
                "\nLAT:" << osem_msg.lat <<
                "\nLON:" << osem_msg.lon <<
                "\nALT:" << osem_msg.alt <<
                "\nDate:" << osem_msg.dateISO8601 <<
                "\nGPSW:" << osem_msg.GPSweek <<
                "\nGPSS:" << osem_msg.GPSsecOfWeek <<
                "\nWAYDEV:" << osem_msg.MaxWayDev <<
                "\nLATDEV:" << osem_msg.MaxLatDev <<
                "\nMINPOSACC:" << osem_msg.MinPosAccuracy;
    if(mandatory_content_check!=OSEM_MANDATORY_CONTENT)
    {
        qDebug() << "OSEM_ERROR: Missing mandatory content";
        return false;
    }
    return true;

}

bool ISOcom::processDOTM(QDataStream &msg_stream, const quint32 &msg_len, QVector<dotm_pt> &traj)
{
    quint32 remaining_content = msg_len;

    quint16 value_id = 0;
    quint16 content_len = 0;

    //const quint16 expected_mandatory_content = 0x01FF;
    qint16 mandatory_content_check = 0;

    dotm_pt dotm_msg;
    dotm_msg.heading = 0;
    dotm_msg.lat_acc = 0;
    dotm_msg.lat_speed = 0;
    dotm_msg.lon_acc = 0;
    dotm_msg.lon_speed = 0;
    dotm_msg.rel_time = 0;
    dotm_msg.x = 0;
    dotm_msg.y = 0;
    dotm_msg.z = 0;


    while(!msg_stream.atEnd())
    {

        if (remaining_content < ISO_MIN_CONTENT_DATA) {
            qDebug() << "DOTM_ERROR: Not enough data to fill content";
            return false;
        }
        // Read Content Header
        msg_stream >> value_id;
        msg_stream >> content_len;
        remaining_content -= (content_len + sizeof(value_id) + sizeof(content_len));

        switch (value_id) {
        case ISO_VALUE_ID_REL_TIME:
            msg_stream >> dotm_msg.rel_time;
            mandatory_content_check |= CONTENT_BINARY_ID_0;
            break;
        case ISO_VALUE_ID_X_POS:
            msg_stream >> dotm_msg.x;
            mandatory_content_check |= CONTENT_BINARY_ID_1;
            break;
        case ISO_VALUE_ID_Y_POS:
            msg_stream >> dotm_msg.y;
            mandatory_content_check |= CONTENT_BINARY_ID_2;
            break;
        case ISO_VALUE_ID_Z_POS:
            msg_stream >> dotm_msg.z;
            mandatory_content_check |= CONTENT_BINARY_ID_3;
            break;
        case ISO_VALUE_ID_HEADING:
            msg_stream >> dotm_msg.heading;
            mandatory_content_check |= CONTENT_BINARY_ID_4;
            break;
        case ISO_VALUE_ID_LONG_SPEED:
            msg_stream >> dotm_msg.lon_speed;
            mandatory_content_check |= CONTENT_BINARY_ID_5;
            break;
        case ISO_VALUE_ID_LAT_SPEED:
            msg_stream >> dotm_msg.lat_speed;
            mandatory_content_check |= CONTENT_BINARY_ID_6;
            break;
        case ISO_VALUE_ID_LONG_ACC:
            msg_stream >> dotm_msg.lon_acc;
            mandatory_content_check |= CONTENT_BINARY_ID_7;
            break;
        case ISO_VALUE_ID_LAT_ACC:
            msg_stream >> dotm_msg.lat_acc;
            mandatory_content_check |= CONTENT_BINARY_ID_8;
            break;
        default:
            msg_stream.skipRawData(content_len);
            qDebug() << "DOTM_WARNING: Unrecognized content. ID:" << value_id;
            break;
        }

        if (mandatory_content_check == DOTM_MANDATORY_CONTENT){

            traj.append(dotm_msg);
            // Clear data
            dotm_msg.heading = 0;
            dotm_msg.lat_acc = 0;
            dotm_msg.lat_speed = 0;
            dotm_msg.lon_acc = 0;
            dotm_msg.lon_speed = 0;
            dotm_msg.rel_time = 0;
            dotm_msg.x = 0;
            dotm_msg.y = 0;
            dotm_msg.z = 0;

            mandatory_content_check = 0;
        }
    }

    return true;
}

bool ISOcom::processOSTM(QDataStream &msg_stream, const quint32 &msg_len, ostm &ostm_msg)
{

    quint32 remaining_content = msg_len;

    quint16 value_id = 0;
    quint16 content_len = 0;

    //const quint16 expected_mandatory_content = 0x01FF;
    qint16 mandatory_content_check = 0;

    while(!msg_stream.atEnd())
    {
        if (remaining_content < ISO_MIN_CONTENT_DATA) {
            qDebug() << "OSTM_ERROR: Not enough data to fill content";
            return false;
        }
        // Read Content Header
        msg_stream >> value_id;
        msg_stream >> content_len;
        remaining_content -= (content_len + sizeof(value_id) + sizeof(content_len));

        switch (value_id) {
        case ISO_VALUE_ID_STATE_CHANGE_REQ:
            msg_stream >> ostm_msg.state_change;
            mandatory_content_check |= CONTENT_BINARY_ID_0;
            break;
        default:
            msg_stream.skipRawData(content_len);
            qDebug() << "OSEM_WARNING: Unrecognized content. ID:" << value_id;
            break;
        }
    }
    // Print OSTM
    qDebug() << "OSTM:" <<
                "\nSTATE_CHANGE_REC:" << ostm_msg.state_change;

    if(mandatory_content_check!=OSTM_MANDATORY_CONTENT)
    {
        qDebug() << "OSTM_ERROR: Missing mandatory content";
        return false;
    }
    return true;
}

bool ISOcom::processSTRT(QDataStream &msg_stream, const quint32 &msg_len, strt &strt_msg)
{

    quint32 remaining_content = msg_len;

    quint16 value_id = 0;
    quint16 content_len = 0;

    //const quint16 expected_mandatory_content = 0x01FF;
    qint16 mandatory_content_check = 0;

    while(!msg_stream.atEnd())
    {
        if (remaining_content < ISO_MIN_CONTENT_DATA) {
            qDebug() << "STRT_ERROR: Not enough data to fill content";
            return false;
        }
        // Read Content Header
        msg_stream >> value_id;
        msg_stream >> content_len;
        remaining_content -= (content_len + sizeof(value_id) + sizeof(content_len));

        switch (value_id) {
        case ISO_VALUE_ID_GPS_SEC_OF_WEEK:
            msg_stream >> strt_msg.GPSsecOfWeek_start_time;
            mandatory_content_check |= CONTENT_BINARY_ID_0;
            break;
        case ISO_VALUE_ID_DELAYED_START:
            msg_stream >> strt_msg.delay_ms;
            mandatory_content_check |= CONTENT_BINARY_ID_1;
            break;
        default:
            msg_stream.skipRawData(content_len);
            qDebug() << "STRT_WARNING: Unrecognized content. ID:" << value_id;
            break;
        }
    }
    // Print STRT
    qDebug() << "STRT:" <<
                "\nGPSS:" << strt_msg.GPSsecOfWeek_start_time <<
                "\nDELAY:" << strt_msg.delay_ms;

    if(mandatory_content_check!=STRT_MANDATORY_CONTENT)
    {
        qDebug() << "STRT_ERROR: Missing mandatory content";
        return false;
    }
    return true;
}


bool ISOcom::processHEAB(QDataStream &msg_stream, const quint32 &msg_len, heab &heab_msg)
{
    if (msg_len != 5) return false;
    msg_stream >> heab_msg.GPSsecOfWeek;
    msg_stream >> heab_msg.cc_status;
    return true;
}

bool ISOcom::sendMonr(monr msg)
{
    if (QString::compare(mUdpHostAddress.toString(), "0.0.0.0") == 0) {
        return false;
    }
    /*

    VByteArray monr_msg;
    monr_msg.vbAppendUint16(ISO_MSG_MONR);
    monr_msg.vbAppendUint32(ISO_MSG_MONR_NoC);

    monr_msg.vbAppendUint16(ISO_VALUE_ID_ABS_TIME); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_U48);  // TYPE ID
    monr_msg.vbAppendUint48(msg.time_stamp); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_X_POS); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_I32);  // TYPE ID
    monr_msg.vbAppendInt32(msg.x); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_Y_POS); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_I32);  // TYPE ID
    monr_msg.vbAppendInt32(msg.y); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_Z_POS); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_I32);  // TYPE ID
    monr_msg.vbAppendInt32(msg.z); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_HEADING); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_U16);  // TYPE ID
    monr_msg.vbAppendInt16(msg.heading); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_LON_SPEED); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_I16);  // TYPE ID
    monr_msg.vbAppendInt16(msg.lon_speed); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_LAT_SPEED); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_I16);  // TYPE ID
    monr_msg.vbAppendInt16(msg.lat_speed); // DATA

    monr_msg.vbAppendInt16(ISO_VALUE_ID_LON_ACC); // VALUE ID
    monr_msg.vbAppendInt8(ISO_TYPE_ID_I16);  // TYPE ID
    monr_msg.vbAppendInt16(msg.lon_acc); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_LAT_ACC); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_I16);  // TYPE ID
    monr_msg.vbAppendInt16(msg.lat_acc); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_FLAG); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_U8);  // TYPE ID
    monr_msg.vbAppendUint8(msg.drive_direction); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_FLAG); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_U8);  // TYPE ID
    monr_msg.vbAppendUint8(msg.object_state); // DATA

    monr_msg.vbAppendUint16(ISO_VALUE_ID_FLAG); // VALUE ID
    monr_msg.vbAppendUint8(ISO_TYPE_ID_U8);  // TYPE ID
    monr_msg.vbAppendUint8(msg.ready_to_arm); // DATA


    VByteArray package_header;
    //Build package header
    package_header.vbAppendUint16(ISO_SYNC_WORD);
    package_header.vbAppendUint8(0); // Tx ID
    package_header.vbAppendUint8(0); // Pkg counter
    package_header.vbAppendUint8(0); // Ack request
    package_header.vbAppendUint32(monr_msg.size());

    VByteArray to_send = package_header.append(monr_msg);
    to_send.vbAppendUint16(0); //CRC


    mUdpSocket->writeDatagram(to_send, mUdpHostAddress, mUdpPort);
*/
    return true;
}

