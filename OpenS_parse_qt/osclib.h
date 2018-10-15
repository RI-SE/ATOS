#pragma once
#ifndef OSCLIB_H
#define OSCLIB_H

#define OSC_KEYWORD_ACTORS "Entities"
#define OSC_KEYWORD_ACTOR "Object"
#define OSC_KEYWORD_STORYBOARD "Storyboard"
#define OSC_KEYWORD_PARAMETERDECLARATION "ParameterDeclaration"
#define OSC_KEYWORD_PARAMETER "Parameter"
#define OSC_KEYWORD_INIT "Init"
#define OSC_KEYWORD_ACTIONS "Actions"
#define OSC_KEYWORD_PRIVATE "Private"
#define OSC_KEYWORD_CATALOG "Catalogs"
#define OSC_KEWORD_DIRECTORY "Directory"


#define OSC_KEYWORD_ACTION "Action"
#define OSC_KEYWORD_MEETING_ACTION "Meeting"

#include <QDebug>
#include <QSharedPointer>

class OSCObject;
class OSCPrivateAction;
class OSCCondition;
class OSCParameter;
class OSCCatalogReference;



class OSCType
{
public:
    virtual ~OSCType();
    virtual void printobject() = 0;
};

#include "oscobject.h"
#include "oscprivateaction.h"
#include "osccondition.h"
#include "oscparameter.h"
#include "osccatalogreference.h"

#endif // OSCLIB_H
