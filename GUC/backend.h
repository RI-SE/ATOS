#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <QDateTime>
#include <QLinkedList>
#include <QTimer>

#include "tcphandler.h"
#include "mscp.h"

#define SERVER_PORT 54241

#define MAX_RESPONSE_LIST_LENGTH 1024

class BackEnd : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString connectionText READ connectionText WRITE setConnectionText NOTIFY connectionTextChanged)
    Q_PROPERTY(int addressValidity READ addressValidity NOTIFY addressValidityChanged)
    Q_PROPERTY(int sysCtrlStatus READ sysCtrlStatus WRITE setSysCtrlStatus NOTIFY sysCtrlStatusChanged)
    Q_PROPERTY(int objCtrlStatus READ objCtrlStatus WRITE setObjCtrlStatus NOTIFY objCtrlStatusChanged)
public:


    explicit BackEnd(QObject *parent = nullptr);

    Q_INVOKABLE void initConnect(){
        mTcphandler->establishConnection(hostName(),SERVER_PORT);
    }

    Q_INVOKABLE void serverDisconnect(){
        mTcphandler->closeConnection();
    }

    // Button functions
    Q_INVOKABLE bool sendArmToHost();
    Q_INVOKABLE bool sendStartToHost(int delayms);
    Q_INVOKABLE bool sendAbortToHost();
    Q_INVOKABLE bool sendGetStatus();
    Q_INVOKABLE bool sendInit();
    Q_INVOKABLE bool sendConnectObject();
    Q_INVOKABLE bool sendDisconnectObject();


    Q_INVOKABLE int addressValid(QString ip_addr){
        return (int) TCPhandler::isValidIP(ip_addr);
    }

    QString hostName();
    Q_INVOKABLE void setHostName(const QString &hostName);

    QString connectionText();
    void setConnectionText(const QString &connectionText);

    int addressValidity();

    int sysCtrlStatus();
    void setSysCtrlStatus(int status);

    int objCtrlStatus();
    void setObjCtrlStatus(int status);

    void handleDebugMessage(const QString &msg);
    void handleRecDataDebugMessage(const QString &msg);
signals:
    void hostNameChanged();
    void connectionTextChanged();
    void addressValidityChanged();
    void sysCtrlStatusChanged();
    void objCtrlStatusChanged();

    void newDebugMessage(QString debugText);

    void enterStartScreen();
    void enterConnectionScreen();

private:
    QString m_hostName = "";
    QString m_connectionText = "test";
    TCPhandler *mTcphandler;
    int m_addressValidity;

    int m_sysCtrlStatus = 0;
    int m_objCtrlStatus = 0;

    //QQueue<qint8> *expected_response_id;
    QLinkedList<qint8> *expected_response_id;

    QTimer *timer;

    bool addExpectedResponseID(qint8 msg_id)
    {
        // TODO add check to not add too many arguments
        if (expected_response_id->size() > MAX_RESPONSE_LIST_LENGTH) {return false; qDebug() << "SEND LIST IS FULL";}
        expected_response_id->append(msg_id);
        return true;
    }

    bool isExpectedResponse(qint8 msg_id){

        QLinkedList<qint8>::iterator i;
        for(i = expected_response_id->begin(); i != expected_response_id->end();++i)
        {
            //qDebug() << "LinkedList:" <<*i;
            if(*i == msg_id){
                expected_response_id->erase(i);
                return true;
            }
        }

        return false;
    }

    void sendToHost(const QByteArray &data, const qint8 expected_return_code)
    {
        emit newDebugMessage("Created Message:\n" + QString(data));
        addExpectedResponseID(expected_return_code);
        mTcphandler->sendData(data);
    }

private slots:
    void handleDebugComMsg(const QString &msg);
    void handleConnectionChanged(const int &isConnected);
    void handleReceivedData(const QByteArray &data);
    void update();
};

#endif // BACKEND_H
