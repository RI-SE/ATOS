#pragma once
#ifndef OSCPRIVATEACTION_H
#define OSCPRIVATEACTION_H

#include <QObject>
#include "osclib.h"

typedef qint8 OSCPrivateActionType;

class OSCPrivateAction : public OSCType
{
public:

    typedef enum {
        UNKNOWN_ACTION = -1,
        MEETING_ACTION
    } ActionType;

    OSCPrivateAction(OSCObject *actor = 0, QString actionType = "");
    OSCPrivateAction(const OSCPrivateAction & other);

    ~OSCPrivateAction();
    QString getName() {return m_actionType;}
    void setName(QString new_actionType) {m_actionType = new_actionType;}

    OSCPrivateActionType getPrivateActionType();

    static OSCPrivateActionType mapOSCPrivateAction(const QString &type_str);

    void printobject();
protected:
    OSCObject *m_owner;
    QString m_actionType = "";
};

class OSCMeetingAction : public OSCPrivateAction
{
public:
    OSCMeetingAction(OSCObject *actor = 0);
    //OSCMeetingAction(const OSCMeetingAction &other);
};


#endif // OSCPRIVATEACTION_H
