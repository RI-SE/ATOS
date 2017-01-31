#ifndef SERVER_H
#define SERVER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>

#include <QThread>

#include "generator.h"

QT_FORWARD_DECLARE_CLASS(QWebSocketServer)
QT_FORWARD_DECLARE_CLASS(QWebSocket)

QT_FORWARD_DECLARE_CLASS(MessageQueueThread)

class VisualizationServer : public QThread
{
    Q_OBJECT
public:
    explicit VisualizationServer(Generator* gen, uint16_t genTime, quint16 port, bool debug, QObject *parent = Q_NULLPTR);
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

private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
    Generator* m_gen;
    bool m_debug;
    int mTimerId;
    uint32_t m_genTime;
    MessageQueueThread *m_Thread;
};

#endif // SERVER_H
