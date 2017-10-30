#ifndef SERVER_H
#define SERVER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include <QThread>

#include "generator.h"
#include "tcpserversimple.h"

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

    typedef enum {
        CHRONOS_VIZUALIZATION_SERVER,
        NMEA_TCP_SERVER
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

private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
    Generator* m_gen;
    TcpServerSimple *mTcpServer;
    bool m_debug;
    int mTimerId;
    uint32_t m_genTime;
    MessageQueueThread *m_Thread;
    int comm;

    int decodeNmeaGGA(QByteArray data, nmea_gga_info_t &gga);
};

#endif // SERVER_H
