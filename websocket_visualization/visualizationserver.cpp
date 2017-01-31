#include "visualizationserver.h"

#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"
#include <QtCore/QDebug>

#include <QThread>

extern "C" {
#include "util.h"
}

VisualizationServer::VisualizationServer(Generator* gen, uint16_t genTime, quint16 port, bool debug, QObject *parent):
    QThread(parent),
    m_pWebSocketServer(new QWebSocketServer(QStringLiteral("Websocket Visualization Server"),
                                            QWebSocketServer::NonSecureMode, this)),
    m_clients(),
    m_debug(debug)
{

    if(gen != NULL)
    {
        m_gen = gen;
        mTimerId = startTimer(genTime);
    }
    else
    {
        (void)iCommInit(IPC_RECV,MQ_VA,0);

        start();
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
    int iCommand;
    char cpBuffer[100];

    while(true)
    {
        bzero(cpBuffer,100);
        (void)iCommRecv(&iCommand,cpBuffer,100);

        if(iCommand == COMM_MONI)
        {
            QString qsTemp(cpBuffer);
            emit sendTextMessage(qsTemp);
        }
    }
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

    // TODO m_clients mutex
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
