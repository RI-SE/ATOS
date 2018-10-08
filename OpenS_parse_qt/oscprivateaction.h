#pragma once
#ifndef OSCPRIVATEACTION_H
#define OSCPRIVATEACTION_H

#include <QObject>
#include "osclib.h"

class OSCPrivateAction : public OSCType
{
public:
    OSCPrivateAction(OSCObject *actor = 0, QString actionType = "");
    OSCPrivateAction(const OSCPrivateAction & other);

    ~OSCPrivateAction();
    QString getName() {return m_actionType;}
    void setName(QString new_actionType) {m_actionType = new_actionType;}

    void printobject();
private:
    OSCObject *m_owner;
    QString m_actionType = "";
};




#endif // OSCPRIVATEACTION_H
