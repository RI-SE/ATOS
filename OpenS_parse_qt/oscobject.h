#pragma once
#ifndef OSCOBJECT_H
#define OSCOBJECT_H

#include <QObject>
#include "osclib.h"

class OSCObject : public QObject, public OSCType
{
public:
    OSCObject(QString name = "");
    OSCObject(const OSCObject & other);

    QString getName() {return m_name;}

    ~OSCObject();

    bool isSame(QString other);

    void printobject();

private:
    QString m_name = "";
};

#endif // OSCOBJECT_H
