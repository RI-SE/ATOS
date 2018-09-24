#pragma once
#ifndef OSCCONDITION_H
#define OSCCONDITION_H

#include <QObject>
#include "osclib.h"

class OSCCondition : public QObject
{
public:
    OSCCondition(QString name = "",OSCAction *action = 0);
private:
    OSCAction *m_action;
    QString m_name;
};

#endif // OSCCONDITION_H
