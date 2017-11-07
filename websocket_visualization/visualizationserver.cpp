#include "visualizationserver.h"

#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"
#include <QtCore/QDebug>

#include <QThread>

extern "C" {
#include "util.h"
}


VisualizationServer::VisualizationServer(Generator* gen, uint16_t genTime, quint16 port, bool debug,int com_type, QObject *parent):
    QThread(parent),
    m_pWebSocketServer(new QWebSocketServer(QStringLiteral("Websocket Visualization Server"),
                                            QWebSocketServer::NonSecureMode, this)),
    m_clients(),
    m_debug(debug)
{

    comm = com_type;
    if(gen != NULL)
    {
        m_gen = gen;
        mTimerId = startTimer(genTime);
    }
    else
    {
        switch (comm) {
        case CHRONOS_VIZUALIZATION_SERVER:
            (void)iCommInit(IPC_RECV,MQ_VA,0);
            break;
        case NMEA_TCP_SERVER:
            mTcpServer = new TcpServerSimple();

            connect(mTcpServer,SIGNAL(connectionChanged(bool)),
                    this,SLOT(tcpServerConnectionChanged(bool)));
            connect(mTcpServer,SIGNAL(dataRx(QByteArray)),this,SLOT(processNMEAmsg(QByteArray)));
            mTcpServer->startServer(TEST_PORT);
            break;
        case NMEA_TCP_CLIENT:
            mTcpClient = new TcpClientSimple();

            // Create reading connection on the client side
            connect(mTcpClient,SIGNAL(dataRx(QByteArray)),this,SLOT(processNMEAmsg(QByteArray)));
            connect(mTcpClient,SIGNAL(socketError(QAbstractSocket::SocketError)),this,SLOT(handleSocketError(QAbstractSocket::SocketError)));
            connection_port = RTKEXPLORER_PORT;
            mTcpClient->connectToServer(connection_port);
            qDebug() << "Connecting to server";


        default:
            break;
        }

        connect(this, &VisualizationServer::sendTextMessage,this, &VisualizationServer::onSendTextMessage);
    }

    if (m_pWebSocketServer->listen(QHostAddress::Any, port))
    {
        if (m_debug)
        {
            qDebug() << "Websocket Visualization Adapter listening on port" << port;
        }
        connect(m_pWebSocketServer, &QWebSocketServer::newConnection,
                this, &VisualizationServer::onNewConnection);
        connect(m_pWebSocketServer, &QWebSocketServer::closed, this, &VisualizationServer::closed);
    }
}

VisualizationServer::~VisualizationServer()
{
    if(m_gen != NULL)
    {
        killTimer(mTimerId);
    }
    else
    {
        (void)iCommClose();
    }
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
}

void VisualizationServer::run()
{
    switch (comm) {
    case CHRONOS_VIZUALIZATION_SERVER: {
        int iCommand;
        char cpBuffer[100];
        bool boExit = false;

        while(!boExit)
        {
            bzero(cpBuffer,100);
            (void)iCommRecv(&iCommand,cpBuffer,100);

            if(iCommand == COMM_MONI)
            {
                QString qsTemp(cpBuffer);
                emit sendTextMessage(qsTemp);
            }
            else if(iCommand == COMM_EXIT)
            {
                boExit = true;
            }
        }
    }
        break;
    case NMEA_TCP_SERVER:
    case NMEA_TCP_CLIENT:{

        bool shutdown = false;
        QTextStream s(stdin);

        while(!shutdown)
        {
            //printf("Input: ");
            QString input = (s.readLine()).toLower();
            if(input.contains("exit"))
            {
                shutdown = true;
            }
            else
            {
                qDebug() << "------------" <<
                            "\n#RCM: " << nr_rcm_rec <<
                            "\n#GGA:" << nr_gga_rec <<
                            "\n#Package sent: " << pack_sent <<
                            "\n------------";
            }
        }


    }
    default:
        break;
    }
    qDebug() << "Exiting the application";
    emit closed();
}

void VisualizationServer::timerEvent(QTimerEvent *event)
{
    (void)event;
    QList<QString> messages = m_gen->GetNextMessages();
    foreach(QString mess, messages)
    {
        onSendTextMessage(mess);
    }
}

void VisualizationServer::onNewConnection()
{
    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    connect(pSocket, &QWebSocket::textMessageReceived, this, &VisualizationServer::processTextMessage);
    connect(pSocket, &QWebSocket::binaryMessageReceived, this, &VisualizationServer::processBinaryMessage);
    connect(pSocket, &QWebSocket::disconnected, this, &VisualizationServer::socketDisconnected);

    m_clients << pSocket;
}

void VisualizationServer::onSendTextMessage(QString message)
{
    if (m_debug)
    {
        static uint32_t times = 0;
        qDebug() << times++ << " message to send :" << message;
    }

    foreach(QWebSocket* client, m_clients)
    {
        client->sendTextMessage(message);
    }

}

void VisualizationServer::processTextMessage(QString message)
{
    if (m_debug)
    {
        qDebug() << "Message received:" << message;
    }
}

void VisualizationServer::processBinaryMessage(QByteArray message)
{
    if (m_debug)
    {
        qDebug() << "Binary Message received:" << message;
    }
}

void VisualizationServer::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug)
    {
        qDebug() << "socketDisconnected:" << pClient;
    }
    if (pClient)
    {
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
}

void VisualizationServer::processNMEAmsg(QByteArray data)
{

    QList<QByteArray> temp = data.split('$');
    for(int i = 0; i<temp.size();i++)
    {
        QString line(temp.at(i));
        fetchNMEAinfo(line,msg_info);

        if (msg_info.GGA_rcvd && msg_info.RMC_rcvd)
        {
            pack_sent++;
            // send the information in the msg_info struct to the vizualizer
            onSendTextMessage(infoToString(msg_info));
            msg_info.GGA_rcvd = false;
            msg_info.RMC_rcvd = false;
        }
    }
}

void VisualizationServer::tcpServerConnectionChanged(bool connected)
{
    if (connected)
    {
        qDebug() << "Connected to TCP server.";
        qDebug() << "IP: " << mTcpServer->getIP();
        qDebug() << "PORT: " << mTcpServer->getPORT();
    }
    else
    {
        qDebug() << "Disconnected from server";
        mTcpServer->stopServer();
    }
}

void VisualizationServer::handleSocketError(QAbstractSocket::SocketError error)
{
    switch (error) {
    case QAbstractSocket::ConnectionRefusedError:
        msleep(1000);
        mTcpClient->connectToServer(connection_port);
        break;
    default:
        break;
    }
}


int VisualizationServer::fetchNMEAinfo(QString &nmea_msg,nmea_info_t &info)
{
    QString buffer = "";

    QStringList fields = nmea_msg.split(',');
    if (fields.size() == 0) return -1;



    buffer = fields.at(0);
    if(buffer.contains("RMC"))
    {
        qDebug() << "RMC handled";
        nr_rcm_rec++;
        /*
        0   $GPRMC          Recommended Minimum sentence C
        1   123519       Fix taken at 12:35:19 UTC
        2   A            Status A=active or V=Void.
        3   4807.038
        4   N   Latitude 48 deg 07.038' N
        5   01131.000,
        6   E  Longitude 11 deg 31.000' E
        7   022.4        Speed over the ground in knots
        8   084.4        Track angle in degrees True
        9   230394       Date - 23rd of March 1994
        10   003.1,W      Magnetic Variation
        11  *6A          The checksum data, always begins with *
        */
        QByteArray strtime    = ((QString)fields.at(1)).toLocal8Bit();
        QByteArray status     = ((QString)fields.at(2)).toLocal8Bit();
        QByteArray lat        = ((QString)fields.at(3)).toLocal8Bit();
        QByteArray northsouth = ((QString)fields.at(4)).toLocal8Bit();
        QByteArray lon        = ((QString)fields.at(5)).toLocal8Bit();
        QByteArray eastwest   = ((QString)fields.at(6)).toLocal8Bit();
        QByteArray speed      = ((QString)fields.at(7)).toLocal8Bit();
        QByteArray heading    = ((QString)fields.at(8)).toLocal8Bit();
        QByteArray date       = ((QString)fields.at(9)).toLocal8Bit();

//        printf("%s,%s,%s,%s,%s,%s,%s,%s\n",strtime.data(),lat.data(),northsouth.data(),lon.data(),eastwest.data(),speed.data(),heading.data(),date.data());

        info.lat = ConvertLatitudeNMEAtoETSICDD(lat.data(),northsouth.data(),status.data());
        info.lon = ConvertLongitudeNMEAtoETSICDD(lon.data(),eastwest.data(),status.data());

        //qDebug() << "Lat: " << info.lat << " Lon: " << info.lon;

        info.heading = ConvertHeadingValueNMEAtoETSICDD(heading.data(),status.data());
        info.speed = ConvertSpeedValueNMEAtoETSICDD(speed.data(),status.data());
        info.time = ConvertTimestapItsNMEAtoETSICDD(strtime.data(),date.data(),status.data());
        info.RMC_rcvd = true;

    }
    else if (buffer.contains("GGA"))
    {

        /*
        0   GGA          Global Positioning System Fix Data
        1   123519       Fix taken at 12:35:19 UTC
        2   4807.038
        3   N   Latitude 48 deg 07.038' N
        4   01131.000
        5   E  Longitude 11 deg 31.000' E
        6   1            Fix quality: 0 = invalid
                                  1 = GPS fix (SPS)
                                  2 = DGPS fix
                                  3 = PPS fix
                      4 = Real Time Kinematic
                      5 = Float RTK
                                  6 = estimated (dead reckoning) (2.3 feature)
                      7 = Manual input mode
                      8 = Simulation mode
        7   08           Number of satellites being tracked
        8   0.9          Horizontal dilution of position
        9   545.4,M      Altitude, Meters, above mean sea level
        10   46.9,M       Height of geoid (mean sea level) above WGS84
                         ellipsoid
        11  (empty field) time in seconds since last DGPS update
        12  (empty field) DGPS station ID number
        13  *47          the checksum data, always begins with *
        * */
        qDebug() << "GGA handled";
        nr_gga_rec++;
        QByteArray alt        = ((QString)fields.at(9)).toLocal8Bit();
        char a[2] = "A";
        info.alt = ConvertAltitudeValueNMEAtoETSICDD(alt.data(),a);
        info.GGA_rcvd = true;
    }
    else {
        return -2;
    }
    return 0;
}

QString VisualizationServer::infoToString(nmea_info_t &info)
{

    QString visualString = "0;0;" +
            QString::number(info.time) + ";" +
            QString::number(info.lat) + ";" +
            QString::number(info.lon) + ";" +
            QString::number(info.alt) + ";" +
            QString::number(info.speed) + ";" +
            QString::number(info.heading) + ";" +
            QString::number(0);//drive direction
    qDebug() << "Sending:" << visualString;
    return visualString;
}

