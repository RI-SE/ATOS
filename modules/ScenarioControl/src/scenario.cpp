#include "scenario.h"

Scenario::Scenario()
{

}

Scenario::~Scenario()
{
    causalities.clear();
    allTriggers.clear();
    allActions.clear();
}

Scenario::ScenarioReturnCode_t Scenario::addTrigger(Trigger &t)
{
    if (allTriggers.find(t) != allTriggers.end())
        return DUPLICATE_ELEMENT;

    allTriggers.insert(t);
    return OK;
}

Scenario::ScenarioReturnCode_t Scenario::addAction(Action &a)
{
    if (allActions.find(a) != allActions.end())
        return DUPLICATE_ELEMENT;

    allActions.insert(a);
    return OK;
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggersWithActions(std::set<Trigger *> ts, std::set<Action *> as)
{
    Causality c(Causality::AND);

    for (Trigger* t : ts)
    {
        c.addTrigger(t);
    }

    for (Action* a : as)
    {
        c.addAction(a);
    }

    causalities.insert(c);
    return OK;
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggerWithAction(Trigger *t, Action *a)
{
    std::set<Trigger*> ts;
    std::set<Action*> as;
    ts.insert(t);
    as.insert(a);
    return linkTriggersWithActions(ts, as);
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggersWithAction(std::set<Trigger*> ts, Action* a)
{
    std::set<Action*> as;
    as.insert(a);
    return linkTriggersWithActions(ts, as);
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggerWithActions(Trigger *t, std::set<Action *> as)
{
    std::set<Trigger*> ts;
    ts.insert(t);
    return linkTriggersWithActions(ts, as);
}


