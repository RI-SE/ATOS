#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QDebug>

#include "tcphandler.h"

class BackEnd : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString userName READ userName WRITE setUserName NOTIFY userNameChanged)
    Q_PROPERTY(QString connectionText READ connectionText WRITE setConnectionText NOTIFY connectionTextChanged)

public:
    explicit BackEnd(QObject *parent = nullptr);

    Q_INVOKABLE void initConnect(){
        //qDebug() << "Connect clicked!";
        mTcphandler->establishConnection(userName(),53241);
    }

    Q_INVOKABLE int addressValid(QString ip_addr){
        return (int) TCPhandler::isValidIP(ip_addr);
    }

    QString userName();
    void setUserName(const QString &userName);

    QString connectionText();
    void setConnectionText(const QString &connectionText);

signals:
    void userNameChanged();
    void connectionTextChanged();

private:
    QString m_userName = "127.0.0.1";
    QString m_connectionText = "test";
    TCPhandler *mTcphandler;
    int m_IPvalid;

private slots:
    void handleDebugComMsg(const QString &msg);

};

#endif // BACKEND_H
