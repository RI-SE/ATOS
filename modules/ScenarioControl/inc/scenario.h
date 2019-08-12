#ifndef SCENARIO_H
#define SCENARIO_H

#include <set>
#include <fstream>

#include "trigger.h"
#include "braketrigger.h"
#include "action.h"
#include "causality.h"
#include "logging.h"

class Scenario
{
public:
    typedef enum {OK, NOT_OK, DUPLICATE_ELEMENT, INVALID_ARGUMENT, NOT_FOUND} ScenarioReturnCode_t;

    Scenario(const std::string scenarioFilePath);
    Scenario() {}
    ~Scenario();

    void initialize(const std::string scenarioFilePath);
    void sendConfiguration(void) const;

    ScenarioReturnCode_t linkTriggersWithActions(std::set<Trigger*> tps, std::set<Action*> aps);
    ScenarioReturnCode_t linkTriggersWithAction(std::set<Trigger*> tps, Action* ap);
    ScenarioReturnCode_t linkTriggerWithActions(Trigger* tp, std::set<Action*> aps);
    ScenarioReturnCode_t linkTriggerWithAction(Trigger* tp, Action* ap);

    ScenarioReturnCode_t addTrigger(Trigger* tp);
    ScenarioReturnCode_t addAction(Action* ap);

    std::set<Causality> getCausalities(void);

    template<typename T>
    ScenarioReturnCode_t updateTrigger(Trigger::TriggerID_t id, T value)
    {
        Trigger::TriggerReturnCode_t retval;
        for (Trigger* tp : allTriggers)
        {
            if (tp->getID() == id)
            {
                retval = tp->update(value);
                if (retval == Trigger::TRIGGER_OCCURRED) LogMessage(LOG_LEVEL_DEBUG, "Triggered ID %u", id);
                return (retval == Trigger::NOT_OK) ? INVALID_ARGUMENT : OK;
            }
        }
        return NOT_FOUND;
    }

    void resetISOTriggers(void);
    void refresh(void) const;
    void clear(void);

    ScenarioReturnCode_t updateTrigger(const MonitorDataType&);

private:
    std::set<Causality> causalities;
    std::set<Trigger*> allTriggers;
    std::set<Action*> allActions;

    void parseScenarioFile(std::ifstream &file);
};

#endif
