#pragma once
#ifndef OSCACTION_H
#define OSCACTION_H

#include <QObject>
#include "osclib.h"

class OSCAction : public QObject
{
public:
    OSCAction(OSCActor *actor = 0);
private:
    OSCActor *m_owner;
};

#endif // OSCACTION_H
