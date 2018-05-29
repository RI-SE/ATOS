#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <QDateTime>

#include "tcphandler.h"
#include "mscp.h"


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
        mTcphandler->establishConnection(hostName(),54241);
    }

    Q_INVOKABLE void serverDisconnect(){
        mTcphandler->closeConnection();
    }

    // Button functions
    Q_INVOKABLE bool sendArmToHost();
    Q_INVOKABLE bool sendDisarmToHost();
    Q_INVOKABLE bool sendStartToHost(int delayms);
    Q_INVOKABLE bool sendAbortToHost();
    Q_INVOKABLE bool sendGetStatus();
    Q_INVOKABLE bool sendInit();
    Q_INVOKABLE bool sendConnectObject();


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

    qint8 expected_response_id = -1;

    void sendToHost(const QByteArray &data, const qint8 expected_return_code)
    {
        emit newDebugMessage("Created Message:\n" + QString(data));
        expected_response_id = expected_return_code;
        mTcphandler->sendData(data);
    }

private slots:
    void handleDebugComMsg(const QString &msg);
    void handleConnectionChanged(const int &isConnected);
    void handleReceivedData(const QByteArray &data);
};

#endif // BACKEND_H
