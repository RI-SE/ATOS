#include "chronos.h"
#include <QDebug>
#include <cmath>

Chronos::Chronos(QObject *parent) : QObject(parent)
{
    //mPacket = 0;
    mTcpServer = new TcpServerSimple(this);
    mUdpSocket = new QUdpSocket(this);

    mUdpHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;

    mTcpState = 0;
    mTcpType = 0;
    mTcpLen = 0;

    mHeabPollCnt = 0;

    connect(mTcpServer, SIGNAL(dataRx(QByteArray)),
            this, SLOT(tcpRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool)),
            this, SLOT(tcpConnectionChanged(bool)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
}

//bool Chronos::startServer(PacketInterface *packet)
bool Chronos::startServer()
{
    //mPacket = packet;

    bool res = mTcpServer->startServer(53241);

    if (!res) {
        qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
    }

    if (res) {
        mUdpSocket->close();
        res = mUdpSocket->bind(QHostAddress::Any, 53240);
    }

    if (!res) {
        qWarning() << "Starting UDP server failed:" << mUdpSocket->errorString();
    }

    qDebug() << "Started CHRONOS server";
/*
    if (res && mPacket) {
        connect(mPacket, SIGNAL(stateReceived(quint8,CAR_STATE)),
                this, SLOT(stateReceived(quint8,CAR_STATE)));
    }
*/
    return res;
}

void Chronos::tcpRx(QByteArray data)
{
    for (char c: data) {
        switch (mTcpState) {
        case 0:
            mTcpType = (quint8)c;
            mTcpLen = 0;
            mTcpData.clear();
            mTcpState++;
            break;

        case 1:
            mTcpLen = ((quint8)c) << 24;
            mTcpState++;
            break;

        case 2:
            mTcpLen |= ((quint8)c) << 16;
            mTcpState++;
            break;

        case 3:
            mTcpLen |= ((quint8)c) << 8;
            mTcpState++;
            break;

        case 4:
            mTcpLen |= (quint8)c;
            mTcpState++;
            break;

        case 5:
            mTcpData.append(c);
            if (mTcpData.size() >= (int)mTcpLen) {
                mTcpState = 0;
                decodeMsg(mTcpType, mTcpLen, mTcpData);
            }
            break;

        default:
            break;
        }
    }
}

void Chronos::tcpConnectionChanged(bool connected)
{
    if (connected) {
        qDebug() << "Chronos TCP connection accepted";
    } else {
        qDebug() << "Chronos TCP disconnected";
    }
}

void Chronos::readPendingDatagrams()
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
void Chronos::stateReceived(quint8 id, CAR_STATE state)
{
    (void)id;
    (void)state;

    // TODO: Send monr message
}*/

bool Chronos::decodeMsg(quint8 type, quint32 len, QByteArray payload)
{
    (void)type;
    (void)len;
    (void)payload;

    switch (type) {
    case CHRONOS_MSG_DOPM: {
        QVector<chronos_dopm_pt> path;
        VByteArray vb(payload);

        while (vb.size() >= 25) {
            chronos_dopm_pt pt;
            pt.tRel = vb.vbPopFrontUint32();
            pt.x = vb.vbPopFrontDouble32(1e3);
            pt.y = vb.vbPopFrontDouble32(1e3);
            pt.z = vb.vbPopFrontDouble32(1e3);
            pt.heading = vb.vbPopFrontDouble16(1e1);
            pt.speed = vb.vbPopFrontDouble16(1e2);
            pt.accel = vb.vbPopFrontInt16();
            pt.curvature = vb.vbPopFrontInt16();
            pt.mode = vb.vbPopFrontUint8();
            path.append(pt);
        }

        // Subsample points
        QVector<chronos_dopm_pt> path_redued;

        if (path.size() > 0) {
            path_redued.append(path.first());
            for (chronos_dopm_pt pt: path) {
                chronos_dopm_pt pt_last = path_redued.last();

                if (sqrt((pt.x - pt_last.x) * (pt.x - pt_last.x) +
                         (pt.y - pt_last.y) * (pt.y - pt_last.y) +
                         (pt.z - pt_last.z) * (pt.z - pt_last.z)) > 0.5) {
                    path_redued.append(pt);
                }
            }
        }

        processDopm(path_redued);
    } break;

    case CHRONOS_MSG_OSEM: {
        chronos_osem osem;
        VByteArray vb(payload);
        osem.lat = vb.vbPopFrontDouble32(1e7);
        osem.lon = vb.vbPopFrontDouble32(1e7);
        osem.alt = vb.vbPopFrontDouble32(1e2);
        osem.heading = vb.vbPopFrontDouble16(1e1);
        processOsem(osem);
    } break;

    case CHRONOS_MSG_OSTM: {
        chronos_ostm ostm;
        ostm.armed = payload.at(0);
        processOstm(ostm);
    } break;

    case CHRONOS_MSG_STRT: {
        chronos_strt strt;
        VByteArray vb(payload);

        strt.type = vb.vbPopFrontUint8();

        if (vb.size() >= 6) {
            strt.ts = vb.vbPopFrontUint48();
        }

        processStrt(strt);
    } break;

    case CHRONOS_MSG_HEAB: {
        chronos_heab heab;
        heab.status = payload.at(0);
        processHeab(heab);
    } break;

    default:
        break;
    }

    return true;
}

/* TODO: Add possibility to save the DOPM message as a trajectory file */

void Chronos::processDopm(QVector<chronos_dopm_pt> path)
{
    qDebug() << "DOPM HANDLED";
    /*
    if (mPacket) {
        mPacket->clearRoute(255);
    }*/
    emit handle_dopm(path);



    for (chronos_dopm_pt pt: path) {
/*

        qDebug() << "-- Point" <<
                    "X:" << pt.x <<
                    "Y:" << pt.y <<
                    "Z:" << pt.z <<
                    "T:" << pt.tRel <<
                    "Speed:" << pt.speed * 3.6;
*/
        /*
        if (mPacket) {
            QList<LocPoint> points;
            LocPoint lpt;
            lpt.setXY(pt.x, pt.y);
            lpt.setSpeed(pt.speed);
            points.append(lpt);

            mPacket->setRoutePoints(255, points);
        }
        */
    }
}

void Chronos::processOsem(chronos_osem osem)
{
    qDebug() << "OSEM HANDLED";
    /*
    double llh[3];
    llh[0] = osem.lat;
    llh[1] = osem.lon;
    llh[2] = osem.alt;


    // TODO: Rotate route with heading

    if (mPacket) {
        mPacket->setEnuRef(255, llh);
    }*/
    emit handle_osem(osem);
}

void Chronos::processOstm(chronos_ostm ostm)
{
    qDebug() << "OSTM RX";
    (void)ostm;
}

void Chronos::processStrt(chronos_strt strt)
{
    qDebug() << "STRT RX";
    (void)strt;
/*
    if (mPacket) {
        mPacket->setApActive(255, true);
    }
    */
}

void Chronos::processHeab(chronos_heab heab)
{
//    qDebug() << "HEAB RX";
    (void)heab;
/*
    if (mPacket) {
        if (heab.status == 1) {
            mHeabPollCnt++;

            if (mHeabPollCnt >= 4) {
                mHeabPollCnt = 0;
                mPacket->getState(255);
            }
        } else {
            mPacket->setApActive(255, false);
        }
    }
    */
}

bool Chronos::sendMonr(chronos_monr monr)
{
    if (QString::compare(mUdpHostAddress.toString(), "0.0.0.0") == 0) {
        return false;
    }

    VByteArray vb;
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
