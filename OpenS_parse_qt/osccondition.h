#pragma once
#ifndef OSCCONDITION_H
#define OSCCONDITION_H

#include <QObject>
#include "osclib.h"

class OSCCondition : public QObject
{
public:
    OSCCondition(QString name = "",OSCPrivateAction *action = 0);
private:
    OSCPrivateAction *m_action;
    QString m_name;
};

#endif // OSCCONDITION_H
