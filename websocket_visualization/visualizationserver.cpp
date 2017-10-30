#include "visualizationserver.h"

#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"
#include <QtCore/QDebug>

#include <QThread>

extern "C" {
#include "util.h"
}

static double nmea_parse_val(char *str) {
    int ind = -1;
    int len = strlen(str);
    double retval = 0.0;

    for (int i = 2;i < len;i++) {
        if (str[i] == '.') {
            ind = i - 2;
            break;
        }
    }

    if (ind >= 0) {
        char a[len + 1];
        memcpy(a, str, ind);
        a[ind] = ' ';
        memcpy(a + ind + 1, str + ind, len - ind);

        double l1, l2;
        if (sscanf(a, "%lf %lf", &l1, &l2) == 2) {
            retval = l1 + l2 / 60.0;
        }
    }

    return retval;
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

            break;
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
    case NMEA_TCP_SERVER: {
        mTcpServer = new TcpServerSimple();

        connect(mTcpServer,SIGNAL(connectionChanged(bool)),
                this,SLOT(tcpServerConnectionChanged(bool)));
        mTcpServer->startServer(TEST_PORT);
        bool shutdown = false;
        QTextStream s(stdin);

        while(!shutdown)
        {
            //printf("Input: ");
            QString input = "";//(s.readLine()).toLower();
            if(input.contains("exit"))
            {
                shutdown = true;
            }
        }

        mTcpServer->stopServer();
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
    QTextStream in(data);

    while(!in.atEnd()) {
        QString line = in.readLine();
        nmea_gga_info_t gga;
        int res = decodeNmeaGGA(line.toLocal8Bit(), gga);

        qDebug() << "Received GAA";
        // TODO: Add printing capabilities
        //emit clientGgaRx(res, gga);
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
    }
}

/**
 * @brief NmeaServer::decodeNmeaGGA
 * Decode NMEA GGA message.
 *
 * @param data
 * NMEA data.
 *
 * @param gga
 * GGA struct to fill.
 *
 * @return
 * -1: Type is not GGA
 * >= 0: Number of decoded fields.
 */
int VisualizationServer::decodeNmeaGGA(QByteArray data, VisualizationServer::nmea_gga_info_t &gga)
{
    char nmea_str[1024];
    int ms = -1;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    int fix_type = 0;
    int sats = 0;
    double hdop = 0.0;
    double diff_age = -1.0;

    int dec_fields = 0;

    setlocale(LC_NUMERIC, "C");

    bool found = false;
    const char *str = data.constData();
    int len = strlen(str);

    for (int i = 0;i < 10;i++) {
        if ((i + 5) >= len) {
            break;
        }

        if (    str[i] == 'G' &&
                str[i + 1] == 'G' &&
                str[i + 2] == 'A' &&
                str[i + 3] == ',') {
            found = true;
            strcpy(nmea_str, str + i + 4);
            break;
        }
    }

    if (found) {
        char *gga, *str;
        int ind = 0;

        str = nmea_str;
#ifdef Q_OS_UNIX
        gga = strsep(&str, ",");
#else
        gga = mystrsep(&str, ",");
#endif

        while (gga != 0) {
            switch (ind) {
            case 0: {
                // Time
                int h, m, s, ds;
                dec_fields++;

                if (sscanf(gga, "%02d%02d%02d.%d", &h, &m, &s, &ds) == 4) {
                    ms = h * 60 * 60 * 1000;
                    ms += m * 60 * 1000;
                    ms += s * 1000;
                    ms += ds * 10;
                } else {
                    ms = -1;
                }
            } break;

            case 1: {
                // Latitude
                dec_fields++;
                lat = nmea_parse_val(gga);
            } break;

            case 2:
                // Latitude direction
                dec_fields++;
                if (*gga == 'S' || *gga == 's') {
                    lat = -lat;
                }
                break;

            case 3: {
                // Longitude
                dec_fields++;
                lon = nmea_parse_val(gga);
            } break;

            case 4:
                // Longitude direction
                dec_fields++;
                if (*gga == 'W' || *gga == 'w') {
                    lon = -lon;
                }
                break;

            case 5:
                // Fix type
                dec_fields++;
                if (sscanf(gga, "%d", &fix_type) != 1) {
                    fix_type = 0;
                }
                break;

            case 6:
                // Sattelites
                dec_fields++;
                if (sscanf(gga, "%d", &sats) != 1) {
                    sats = 0;
                }
                break;

            case 7:
                // hdop
                dec_fields++;
                if (sscanf(gga, "%lf", &hdop) != 1) {
                    hdop = 0.0;
                }
                break;

            case 8:
                // Altitude
                dec_fields++;
                if (sscanf(gga, "%lf", &height) != 1) {
                    height = 0.0;
                }
                break;

            case 10: {
                // Altitude 2
                double h2 = 0.0;
                dec_fields++;
                if (sscanf(gga, "%lf", &h2) != 1) {
                    h2 = 0.0;
                }

                height += h2;
            } break;

            case 12: {
                // Correction age
                dec_fields++;
                if (sscanf(gga, "%lf", &diff_age) != 1) {
                    diff_age = -1.0;
                }
            } break;

            default:
                break;
            }

#ifdef Q_OS_UNIX
            gga = strsep(&str, ",");
#else
            gga = mystrsep(&str, ",");
#endif
            ind++;
        }
    } else {
        dec_fields = -1;
    }

    gga.lat = lat;
    gga.lon = lon;
    gga.height = height;
    gga.fix_type = fix_type;
    gga.n_sat = sats;
    gga.t_tow = ms;
    gga.h_dop = hdop;
    gga.diff_age = diff_age;

    return dec_fields;
}
