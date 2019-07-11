#include "causality.h"

Causality::Causality(Causality::TriggerOperator_t op)
{
    oper = op;
    action = Action(0,Action::ACTION_NONE,0);
    triggers = std::set<Trigger>();
}

Causality::Causality(Action a, TriggerOperator_t op)
{
    action = a;
    oper = op;
    triggers = std::set<Trigger>();
}

Causality::Causality(Trigger t, Action a, TriggerOperator_t op)
{
    triggers.insert(t);
    action = a;
    oper = op;
}
