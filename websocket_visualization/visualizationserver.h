#ifndef SERVER_H
#define SERVER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include <QThread>

#include "generator.h"
#include "tcpclientsimple.h"
#include "tcpserversimple.h"
#include "nmea2etsi.h"

#define RTKEXPLORER_PORT 2948
#define TEST_PORT 52340

QT_FORWARD_DECLARE_CLASS(QWebSocketServer)
QT_FORWARD_DECLARE_CLASS(QWebSocket)

QT_FORWARD_DECLARE_CLASS(MessageQueueThread)


class VisualizationServer : public QThread
{
    Q_OBJECT


public:

    typedef struct {
        double lat;
        double lon;
        double height;
        double t_tow;
        int n_sat;
        int fix_type;
        double h_dop;
        double diff_age;
    } nmea_gga_info_t;

    typedef struct {
        bool GGA_rcvd;
        bool RMC_rcvd;
        uint32_t lat;
        uint32_t lon;
        int32_t alt;
        uint16_t speed;
        uint16_t heading;
        uint64_t time;
    } nmea_info_t;

    typedef enum {
        CHRONOS_VIZUALIZATION_SERVER,
        NMEA_TCP_SERVER,
        NMEA_TCP_CLIENT
    } COM_TYPE;

    explicit VisualizationServer( Generator* gen, uint16_t genTime, quint16 port, bool debug, int com_type, QObject *parent = Q_NULLPTR);
    ~VisualizationServer();

    void onSendTextMessage(QString message);

signals:
    void sendTextMessage(QString message);

protected:
    void run() override;
    void timerEvent(QTimerEvent *event);

Q_SIGNALS:
    void closed();

private Q_SLOTS:
    void onNewConnection();
    void processTextMessage(QString message);
    void processBinaryMessage(QByteArray message);
    void socketDisconnected();

    void processNMEAmsg(QByteArray messsage);

    void tcpServerConnectionChanged(bool);

    void handleSocketError(QAbstractSocket::SocketError);



private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
    Generator* m_gen;

    TcpServerSimple *mTcpServer;
    TcpClientSimple *mTcpClient;

    int connection_port;
    // Statistics variables

    int nr_rcm_rec = 0;
    int nr_gga_rec = 0;
    int pack_sent = 0;

    bool m_debug;
    int mTimerId;
    uint32_t m_genTime;
    MessageQueueThread *m_Thread;
    int comm;
    nmea_info_t msg_info={ 0,0,0,0,0,0,0,0 };

    //int decodeNmeaGGA(QByteArray data, nmea_gga_info_t &gga);
    int fetchNMEAinfo(QString &nmea_msg,nmea_info_t &info);
    QString infoToString(nmea_info_t &info);
};

#endif // SERVER_H
