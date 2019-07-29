#include "scenario.h"

#include <fstream>
#include <stdexcept>

#include "logging.h"

Scenario::Scenario(const std::string scenarioFilePath)
{
    initialize(scenarioFilePath);
}


Scenario::~Scenario()
{
    clear();
}

void Scenario::clear()
{
    for (Trigger* t : allTriggers)
        delete t;

    for (Action* a : allActions)
        delete a;

    causalities.clear();
    allTriggers.clear();
    allActions.clear();
}

void Scenario::initialize(const std::string scenarioFilePath)
{
    std::ifstream file;
    file.exceptions(std::ios_base::badbit | std::ios_base::failbit);

    clear();
    LogMessage(LOG_LEVEL_DEBUG, "Opening scenario file <%s>", scenarioFilePath.c_str());
    file.open(scenarioFilePath);

    try {
        parseScenarioFile(file);
    } catch (std::invalid_argument) {
        file.close();
        throw;
    }
    file.close();

    LogMessage(LOG_LEVEL_INFO, "Successfully initialized scenario with %d unique triggers and %d unique actions", allTriggers.size(), allActions.size());
}

void Scenario::parseScenarioFile(std::ifstream &file)
{
    LogMessage(LOG_LEVEL_DEBUG, "Parsing scenario file");
    // TODO: read file, throw std::invalid_argument if badly formatted
    // TODO: decode file into triggers and actions
    BrakeTrigger* bt = new BrakeTrigger(1);
    Action* mqttAction = new Action(5, Action::ACTION_TEST_SCENARIO_COMMAND, 1);

    bt->appendParameter(Trigger::TRIGGER_PARAMETER_PRESSED);
    bt->parseParameters();

    addTrigger(bt);
    addAction(mqttAction);

    linkTriggerWithAction(bt, mqttAction);
}

Scenario::ScenarioReturnCode_t Scenario::addTrigger(Trigger* t)
{
    for (Trigger* knownTrigger : allTriggers)
    {
        if (knownTrigger->getID() == t->getID()) return DUPLICATE_ELEMENT;
    }

    allTriggers.insert(t);
    return OK;
}

Scenario::ScenarioReturnCode_t Scenario::addAction(Action* a)
{
    for (Action* knownAction : allActions)
    {
        if (knownAction->getID() == a->getID()) return DUPLICATE_ELEMENT;
    }

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

void Scenario::refresh(void) const
{
    for (const Causality &c : causalities)
    {
        c.refresh();
    }
}

