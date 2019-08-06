#include "logging.h"
#include "maestroTime.h"

#include <fstream>
#include <stdexcept>

#include "scenario.h"


#include "isotrigger.h"
#include "externalaction.h"

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
    for (Trigger* tp : allTriggers)
        delete tp;

    for (Action* ap : allActions)
        delete ap;

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

/*!
 * \brief Scenario::sendConfiguration Sends TRCM and ACCM according to previously initialized scenario
 */
void Scenario::sendConfiguration(void) const
{
    for (Trigger* tp : allTriggers)
    {
        if(iCommSendTRCM(tp->getConfigurationMessageData()) == -1)
            util_error("Fatal communication error sending TRCM");
    }

    for (Action* ap : allActions)
    {
        if(iCommSendACCM(ap->getConfigurationMessageData()) == -1)
            util_error("Fatal communication error sending ACCM");
    }
}

void Scenario::parseScenarioFile(std::ifstream &file)
{
    LogMessage(LOG_LEVEL_DEBUG, "Parsing scenario file");
    // TODO: read file, throw std::invalid_argument if badly formatted
    // TODO: decode file into triggers and actions
    // TODO: link triggers and actions

    // PLACEHOLDER CODE
    BrakeTrigger* bt = new BrakeTrigger(1);
    InfrastructureAction* mqttAction = new InfrastructureAction(5, 1);
    const char brakeObjectIPString[] = "127.0.0.1";
    const char mqttObjectIPString[] = "127.0.0.1";
    in_addr brakeObjectIP, mqttObjectIP;
    inet_pton(AF_INET, brakeObjectIPString, &brakeObjectIP);
    inet_pton(AF_INET, mqttObjectIPString, &mqttObjectIP);

    bt->appendParameter(Trigger::TriggerParameter_t::TRIGGER_PARAMETER_PRESSED);
    bt->parseParameters();
    bt->setObjectIP(brakeObjectIP.s_addr);

    mqttAction->appendParameter(Action::ActionParameter_t::ACTION_PARAMETER_VS_BRAKE_WARNING);
    mqttAction->setObjectIP(mqttObjectIP.s_addr);
    mqttAction->setExecuteDalayTime({1,0});

    addTrigger(bt);
    addAction(mqttAction);

    linkTriggerWithAction(bt, mqttAction);
}

Scenario::ScenarioReturnCode_t Scenario::addTrigger(Trigger* tp)
{
    for (Trigger* knownTrigger : allTriggers)
    {
        if (knownTrigger->getID() == tp->getID()) return DUPLICATE_ELEMENT;
    }

    allTriggers.insert(tp);
    return OK;
}

Scenario::ScenarioReturnCode_t Scenario::addAction(Action* ap)
{
    for (Action* knownAction : allActions)
    {
        if (knownAction->getID() == ap->getID()) return DUPLICATE_ELEMENT;
    }

    allActions.insert(ap);
    return OK;
}


Scenario::ScenarioReturnCode_t Scenario::linkTriggersWithActions(std::set<Trigger *> ts, std::set<Action *> aps)
{
    Causality c(Causality::AND);

    for (Trigger* tp : ts)
    {
        c.addTrigger(tp);
    }

    for (Action* ap : aps)
    {
        c.addAction(ap);
    }

    causalities.insert(c);
    return OK;
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggerWithAction(Trigger *tp, Action *ap)
{
    std::set<Trigger*> tps;
    std::set<Action*> aps;
    tps.insert(tp);
    aps.insert(ap);
    return linkTriggersWithActions(tps, aps);
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggersWithAction(std::set<Trigger*> tps, Action* ap)
{
    std::set<Action*> aps;
    aps.insert(ap);
    return linkTriggersWithActions(tps, aps);
}

Scenario::ScenarioReturnCode_t Scenario::linkTriggerWithActions(Trigger *tp, std::set<Action *> aps)
{
    std::set<Trigger*> tps;
    tps.insert(tp);
    return linkTriggersWithActions(tps, aps);
}

void Scenario::refresh(void) const
{
    for (const Causality &c : causalities)
    {
        c.refresh();
    }
}

void Scenario::resetISOTriggers(void)
{
    for (Trigger* tp : allTriggers)
    {
        if (dynamic_cast<ISOTrigger*>(tp) != nullptr)
        {
            // "untrigger" the trigger
            tp->update();
        }
    }
}

Scenario::ScenarioReturnCode_t Scenario::updateTrigger(const MonitorDataType &monr)
{
    for (Trigger* tp : allTriggers)
    {
        if(tp->getObjectIP() == monr.ClientIP && dynamic_cast<ISOTrigger*>(tp) == nullptr)
        {
            switch (tp->getTypeCode())
            {
            case Trigger::TriggerTypeCode_t::TRIGGER_BRAKE:
                struct timeval monrTime, currentTime;
                TimeSetToCurrentSystemTime(&currentTime);
                TimeSetToGPStime(&monrTime, TimeGetAsGPSweek(&currentTime), monr.MONR.GPSQmsOfWeekU32);
                tp->update(static_cast<double>(monr.MONR.LongitudinalSpeedI16*100), monrTime);
                break;
            default:
                LogMessage(LOG_LEVEL_WARNING, "Unhandled trigger type in update: %s",
                           tp->getTypeAsString(tp->getTypeCode()).c_str());
            }
        }
    }
}

