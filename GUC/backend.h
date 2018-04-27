#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QDebug>

#include "ipconnecitonhandler.h"

class BackEnd : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString userName READ userName WRITE setUserName NOTIFY userNameChanged)

public:
    explicit BackEnd(QObject *parent = nullptr);

    Q_INVOKABLE void connect(){
        qDebug() << "Connect clicked!";
    }

    QString userName();
    void setUserName(const QString &userName);

signals:
    void userNameChanged();

private:
    QString m_userName;// "10.130.23.14";
};

#endif // BACKEND_H
