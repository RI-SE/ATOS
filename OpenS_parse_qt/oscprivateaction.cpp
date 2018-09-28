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

void OSCPrivateAction::printobject()
{
    QString actorstring = m_owner == NULL ? "NULL" : m_owner->getName();
    qDebug() << "OSCPrivateAction( " << actorstring << ", " << this->m_actionType << " )";
}
