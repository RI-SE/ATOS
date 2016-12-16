#ifndef SERVER_H
#define SERVER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include "generator.h"

QT_FORWARD_DECLARE_CLASS(QWebSocketServer)
QT_FORWARD_DECLARE_CLASS(QWebSocket)

class VisualizationServer : public QObject
{
    Q_OBJECT
public:
    explicit VisualizationServer(Generator* gen, uint16_t genTime, quint16 port, bool debug, QObject *parent = Q_NULLPTR);
    ~VisualizationServer();

    void SendTextMessage(QString message);

protected:
    void timerEvent(QTimerEvent *event);

Q_SIGNALS:
    void closed();

private Q_SLOTS:
    void onNewConnection();
    void processTextMessage(QString message);
    void processBinaryMessage(QByteArray message);
    void socketDisconnected();

private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
    Generator* m_gen;
    bool m_debug;
    int mTimerId;
    uint32_t m_genTime;
};

#endif // SERVER_H
