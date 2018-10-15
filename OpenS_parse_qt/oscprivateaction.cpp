#include "oscprivateaction.h"

OSCPrivateAction::OSCPrivateAction(OSCObject *actor, QString actionType)
{
    this->m_owner = actor;
    this->m_actionType = actionType;
}

OSCPrivateAction::OSCPrivateAction(const OSCPrivateAction & other)
{
    this->m_owner = other.m_owner;
    this->m_actionType =  other.m_actionType;
}

OSCPrivateAction::~OSCPrivateAction() {}


OSCPrivateActionType OSCPrivateAction::getPrivateActionType()
{
    return mapOSCPrivateAction(m_actionType);
}

OSCPrivateActionType OSCPrivateAction::mapOSCPrivateAction(const QString &type_str)
{
    static QHash<QString, OSCPrivateActionType> map = {
        {OSC_KEYWORD_MEETING_ACTION, OSCPrivateAction::MEETING_ACTION}
    };
    return  map.value(type_str,OSCPrivateAction::UNKNOWN_ACTION);
}

void OSCPrivateAction::printobject()
{
    QString actorstring = m_owner == NULL ? "NULL" : m_owner->getName();
    qDebug() << "OSCPrivateAction( " << actorstring << ", " << this->m_actionType << " )";
}


/* OSCMeetingAction */

OSCMeetingAction::OSCMeetingAction(OSCObject *actor)
{
    this->m_owner = actor;
    this->m_actionType = OSC_KEYWORD_MEETING_ACTION;
}
