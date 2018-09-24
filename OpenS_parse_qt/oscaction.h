#pragma once
#ifndef OSCACTION_H
#define OSCACTION_H

#include <QObject>
#include "osclib.h"

class OSCAction : public QObject, public OSCObject
{
public:

    OSCAction(OSCActor *actor = 0, QString actionType = "");
    OSCAction(const OSCAction & other);

    void printobject();
private:
    OSCActor *m_owner;
    QString m_actionType;
};

#endif // OSCACTION_H
