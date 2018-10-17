#pragma once
#ifndef OSCACTOR_H
#define OSCACTOR_H

#include <QObject>
#include "osclib.h"

class OSCActor : public QObject, public OSCObject
{
public:
    OSCActor(QString name = "");
    OSCActor(const OSCActor & other);

    ~OSCActor();
    QString getName() {return m_name;}
    void setName(QString new_name) {m_name = new_name;}

    bool isSame(QString other);

    void printobject();
private:
    QString m_name = "";
};

#endif // OSCACTOR_H
