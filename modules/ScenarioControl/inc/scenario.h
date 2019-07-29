
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

    ScenarioReturnCode_t linkTriggersWithActions(std::set<Trigger*> ts, std::set<Action*> as);
    ScenarioReturnCode_t linkTriggersWithAction(std::set<Trigger*> ts, Action* a);
    ScenarioReturnCode_t linkTriggerWithActions(Trigger* t, std::set<Action*> as);
    ScenarioReturnCode_t linkTriggerWithAction(Trigger* t, Action* a);

    ScenarioReturnCode_t addTrigger(Trigger* t);
    ScenarioReturnCode_t addAction(Action* a);

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

    void refresh(void) const;
    void clear(void);

private:
    std::set<Causality> causalities;
    std::set<Trigger*> allTriggers;
    std::set<Action*> allActions;

    void parseScenarioFile(std::ifstream &file);
};
