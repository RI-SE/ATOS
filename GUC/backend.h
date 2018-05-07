#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QDebug>

#include "tcphandler.h"




class BackEnd : public QObject
{
    Q_OBJECT
    //Q_PROPERTY(QString hostName READ hostName WRITE setHostName NOTIFY hostNameChanged)
    Q_PROPERTY(QString connectionText READ connectionText WRITE setConnectionText NOTIFY connectionTextChanged)
    Q_PROPERTY(int addressValidity READ addressValidity NOTIFY addressValidityChanged)
    //Q_PROPERTY(int backendState READ backendState WRITE setBackendState NOTIFY backendStateChanged)
public:


    explicit BackEnd(QObject *parent = nullptr);

    Q_INVOKABLE void initConnect(){
        qDebug() << "Connect clicked!";
        mTcphandler->establishConnection(hostName(),53241);
    }

    Q_INVOKABLE int addressValid(QString ip_addr){
        return (int) TCPhandler::isValidIP(ip_addr);
    }

    QString hostName();
    Q_INVOKABLE void setHostName(const QString &hostName);

    QString connectionText();
    void setConnectionText(const QString &connectionText);

    int addressValidity();
signals:
    void hostNameChanged();
    void connectionTextChanged();
    void addressValidityChanged();

    void enterStartScreen();
    void enterConnectionScreen();

private:
    QString m_hostName = "";
    QString m_connectionText = "test";
    TCPhandler *mTcphandler;
    int m_addressValidity;

private slots:
    void handleDebugComMsg(const QString &msg);
    void handleConnectionChanged(const int &isConnected);
};

#endif // BACKEND_H
