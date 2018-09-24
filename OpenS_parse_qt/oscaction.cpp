#include "oscaction.h"

OSCAction::OSCAction(OSCActor *actor, QString actionType)
{
    this->m_owner = actor;
    this->m_actionType = actionType;
}

OSCAction::OSCAction(const OSCAction & other)
{
    this->m_owner = other.m_owner;
}


void OSCAction::printobject()
{
    QString actorstring = m_owner == NULL ? "NULL" : m_owner->getName();
    qDebug() << "OSCAction( " << actorstring << ", " << m_actionType << " )";
}
