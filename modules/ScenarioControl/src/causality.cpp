#include "causality.h"

Causality::Causality(Causality::TriggerOperator_t op)
{
    oper = op;
    actions = std::set<Action*>();
    triggers = std::set<Trigger*>();
}

Causality::Causality(Action* a, TriggerOperator_t op)
{
    actions.insert(a);;
    oper = op;
    triggers = std::set<Trigger*>();
}

Causality::Causality(Trigger* t, Action* a, TriggerOperator_t op)
{
    triggers.insert(t);
    actions.insert(a);
    oper = op;
}

Action* Causality::getActionByID(Action::ActionID_t id)
{
    for (Action* a : actions)
    {
        if (a->getID() == id)
            return a;
    }
    return nullptr;
}

Trigger* Causality::getTriggerByID(Trigger::TriggerID_t id)
{
    for (Trigger* t : triggers)
    {
        if (t->getID() == id)
            return t;
    }
    return nullptr;
}
