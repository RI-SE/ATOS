#include "oscaction.h"

OSCAction::OSCAction(OSCObject *actor, QString actionType)
{
    this->m_owner = actor;
    this->m_actionType = actionType;
}

OSCAction::OSCAction(const OSCAction & other)
{
    this->m_owner = other.m_owner;
    this->m_actionType =  other.m_actionType;
}

OSCAction::~OSCAction() {}

void OSCAction::printobject()
{
    QString actorstring = m_owner == NULL ? "NULL" : m_owner->getName();
    qDebug() << "OSCAction( " << actorstring << ", " << this->m_actionType << " )";
}
