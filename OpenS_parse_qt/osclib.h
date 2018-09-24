#pragma once
#ifndef OSCLIB_H
#define OSCLIB_H

#define OSC_KEYWORD_ACTORS "Entities"
#define OSC_KEYWORD_ACTOR "Object"
#define OSC_KEYWORD_STORYBOARD "Storyboard"
#define OSC_KEYWORD_INIT "Init"
#define OSC_KEYWORD_ACTIONS "Actions"
#define OSC_KEYWORD_ACTION "Action"
#define OSC_KEYWORD_PRIVATE "Private"

#include <QDebug>

class OSCActor;
class OSCAction;
class OSCCondition;

class OSCObject
{
public:
    virtual ~OSCObject();
    virtual void printobject() = 0;
};

#include "oscactor.h"
#include "oscaction.h"
#include "osccondition.h"

#endif // OSCLIB_H
