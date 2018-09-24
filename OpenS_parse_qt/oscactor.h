#pragma once
#ifndef OSCACTOR_H
#define OSCACTOR_H

#include <QObject>
#include "osclib.h"

class OSCActor : public QObject
{
public:
    OSCActor(QString name = "");

    QString getName() {return m_name;}
    void setName(QString new_name) {m_name = new_name;}

private:
    QString m_name = "";
};

#endif // OSCACTOR_H
