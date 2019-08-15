#include "logging.h"
#include "maestroTime.h"

#include <fstream>
#include <stdexcept>
#include <regex>

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
    std::string debugStr;

    clear();
    LogMessage(LOG_LEVEL_DEBUG, "Opening scenario file <%s>", scenarioFilePath.c_str());
    file.open(scenarioFilePath);

    if (file.is_open())
    {
        try {
            parseScenarioFile(file);
        } catch (std::invalid_argument) {
            file.close();
            throw;
        }
        file.close();
    }
    else {
        throw std::ifstream::failure("Unable to open file <" + scenarioFilePath + ">");
    }
    LogMessage(LOG_LEVEL_INFO, "Successfully initialized scenario with %d unique triggers and %d unique actions", allTriggers.size(), allActions.size());

    debugStr =  "Triggers:\n";
    for (Trigger* tp : allTriggers)
        debugStr += "\t" + tp->getTypeAsString(tp->getTypeCode()) + "\n";

    debugStr += "Actions:\n";
    for (Action* ap : allActions)
        debugStr += "\t" + ap->getTypeAsString(ap->getTypeCode()) + "\n";

    debugStr.pop_back();
    LogMessage(LOG_LEVEL_DEBUG, debugStr.c_str());
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


void Scenario::splitLine(const std::string &line, const char delimiter, std::vector<std::string> &result)
{
    std::istringstream strm(line);
    std::string part;
    while (getline(strm,part,delimiter))
    {
        result.push_back(part);
    }
    return;
}

void Scenario::parseScenarioFileLine(const std::string &inputLine)
{
    using namespace std;
    if (inputLine[0] == '#' || inputLine.length() == 0) return;
    string line = inputLine;

    // Remove whitespace
    string::iterator endPos = remove_if(line.begin(), line.end(), ::isspace);
    line.erase(endPos,line.end());

    // Split into parts
    vector<string> parts;
    constexpr char delimiter = ';';
    splitLine(line,delimiter,parts);

    // Match and act on relevant fields
    regex ipAddrPattern("([0-2]?[0-9]?[0-9]\\.){3}([0-2]?[0-9]?[0-9])"); // Match 3 "<000-299>." followed by "<000-299>"
    regex triggerActionPattern("(([a-zA-Z_])+\\[([a-zA-Z0-9\\.,<=>_])+\\])+");
    in_addr triggerIP, actionIP;
    string errMsg;
    set<Action*> actions;
    set<Trigger*> triggers;
    enum {TRIGGER_IP,TRIGGER,ACTION_IP,ACTION,DONE} parseState = TRIGGER_IP;

    for (const string &part : parts)
    {
        switch (parseState)
        {
        case TRIGGER_IP:
            if(!regex_match(part,ipAddrPattern))
            {
                errMsg = "Specified trigger IP address field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            if(inet_pton(AF_INET, part.c_str(), &triggerIP) <= 0)
            {
                errMsg = "Error parsing IP string <" + part + ">";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }
            parseState = TRIGGER;
            break;
        case TRIGGER:
            if(!regex_match(part,triggerActionPattern))
            {
                errMsg = "Trigger configuration field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            triggers = parseTriggerConfiguration(part);
            for (Trigger* tp : triggers)
            {
                tp->setObjectIP(triggerIP.s_addr);
                for (Trigger* knownTrigger : allTriggers)
                    tp = tp->isSimilar(*knownTrigger) ? knownTrigger : tp;
                addTrigger(tp);
            }
            parseState = ACTION_IP;
            break;
        case ACTION_IP:
            if(!regex_match(part,ipAddrPattern))
            {
                errMsg = "Specified action IP address field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            if(inet_pton(AF_INET, part.c_str(), &actionIP) <= 0)
            {
                errMsg = "Error parsing IP string <" + part + ">";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }
            parseState = ACTION;
            break;
        case ACTION:
            if(!regex_match(part,triggerActionPattern))
            {
                errMsg = "Action configuration field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            actions = parseActionConfiguration(part);
            for (Action* ap : actions)
            {
                ap->setObjectIP(actionIP.s_addr);
                addAction(ap);
            }
            parseState = DONE;
            break;
        case DONE:
            if (part.length() != 0) LogMessage(LOG_LEVEL_WARNING,"Ignored tail field of row in configuration file: <%s>",part.c_str());
            break;
        }
    }

    linkTriggersWithActions(triggers, actions);
    return;
}

std::set<Trigger*> Scenario::parseTriggerConfiguration(const std::string &inputConfig)
{
    using namespace std;
    regex triggerPattern("([a-zA-Z_]+)\\[([^,^\\]]+)(?:,([^,^\\]]+))?(?:,([^,^\\]]+))?\\]");
    smatch match;
    string errMsg;
    vector<string> configs;
    Trigger::TriggerTypeCode_t triggerType;
    set<Trigger*> returnTriggers;
    Trigger* trigger;
    Trigger::TriggerID_t baseTriggerID = static_cast<Trigger::TriggerID_t>(allTriggers.size());

    splitLine(inputConfig, ']', configs);
    for (string &config : configs)
        config.append("]");

    for(const string &config : configs)
    {
        if (!regex_search(config, match, triggerPattern))
        {
            errMsg = "The following is not a valid configuration: <" + config + ">";
            LogMessage(LOG_LEVEL_ERROR,errMsg.c_str());
            throw invalid_argument(errMsg);
        }
        vector<string> matches;
        for (unsigned int i = 0; i < match.size(); i++)
            matches.push_back(match[i].str());

        triggerType = Trigger::asTypeCode(match[1]);
        switch (triggerType) {
        case TRIGGER_BRAKE:
            trigger = new BrakeTrigger(baseTriggerID + static_cast<Trigger::TriggerID_t>(returnTriggers.size()));
            // TODO: the OR between the Maestro trigger and possible TREO messages
            break;
        default:
            // TODO: implement ISO triggers with TRCM and whatnot
            // trigger = new ISOTrigger(baseTriggerID + static_cast<Trigger::TriggerID_t>(returnTriggers.size(), triggerType);
            errMsg = "Unimplemented trigger type " + match[0].str();
            LogMessage(LOG_LEVEL_ERROR,errMsg.c_str());
            throw logic_error(errMsg);
            break;
        }
        for (unsigned int i = 2; i < match.size(); ++i)
            if (!match[i].str().empty()) trigger->appendParameter(match[i].str());
        returnTriggers.insert(trigger);
    }
    return returnTriggers;
}

std::set<Action*> Scenario::parseActionConfiguration(const std::string &inputConfig)
{
    using namespace std;
    regex triggerPattern("([a-zA-Z_]+)\\[([^,^\\]]+)(?:,([^,^\\]]+))?(?:,([^,^\\]]+))?\\]");
    smatch match;
    string errMsg;
    vector<string> configs;
    Action::ActionTypeCode_t actionType;
    set<Action*> returnActions;
    Action* action;
    Action::ActionID_t baseActionID = static_cast<Action::ActionID_t>(allActions.size());

    splitLine(inputConfig, ']', configs);
    for (string &config : configs)
        config.append("]");

    for(const string &config : configs)
    {
        if (!regex_search(config, match, triggerPattern))
        {
            errMsg = "The following is not a valid configuration: <" + config + ">";
            LogMessage(LOG_LEVEL_ERROR,errMsg.c_str());
            throw invalid_argument(errMsg);
        }

        actionType = Action::asTypeCode(match[1]);
        switch (actionType)
        {
        case ACTION_INFRASTRUCTURE:
            action = new InfrastructureAction(baseActionID + static_cast<Action::ActionID_t>(returnActions.size()));
            break;
        default:
            action = new ExternalAction(baseActionID + static_cast<Action::ActionID_t>(returnActions.size()), actionType);
            break;
        }
        for (unsigned int i = 2; i < match.size(); ++i)
            if(!match[i].str().empty()) action->appendParameter(match[i].str());
        returnActions.insert(action);
    }

    return returnActions;
}

void Scenario::parseScenarioFile(std::ifstream &file)
{
    LogMessage(LOG_LEVEL_DEBUG, "Parsing scenario file");

    std::string line;
    while ( std::getline(file, line) ) parseScenarioFileLine(line);

    return;
    // TODO: read file, throw std::invalid_argument if badly formatted
    // TODO: decode file into triggers and actions
    // TODO: link triggers and actions

    // PLACEHOLDER CODE
    BrakeTrigger* bt = new BrakeTrigger(1);
    InfrastructureAction* mqttAction = new InfrastructureAction(5, 1);
    ExternalAction* brakeLightAction = new ExternalAction(6,Action::ActionTypeCode_t::ACTION_MISC_DIGITAL_OUTPUT,1);
    const char brakeObjectIPString[] = "127.0.0.1";
    const char mqttObjectIPString[] = "127.0.0.1";
    const char ledObjectIPString[] = "10.130.254.197";
    in_addr brakeObjectIP, mqttObjectIP, ledObjectIP;
    inet_pton(AF_INET, brakeObjectIPString, &brakeObjectIP);
    inet_pton(AF_INET, mqttObjectIPString, &mqttObjectIP);
    inet_pton(AF_INET, ledObjectIPString, &ledObjectIP);

    bt->appendParameter(Trigger::TriggerParameter_t::TRIGGER_PARAMETER_PRESSED);
    bt->parseParameters();
    bt->setObjectIP(brakeObjectIP.s_addr);

    brakeLightAction->appendParameter(Action::ActionParameter_t::ACTION_PARAMETER_SET_TRUE);
    brakeLightAction->setObjectIP(ledObjectIP.s_addr);

    mqttAction->appendParameter(Action::ActionParameter_t::ACTION_PARAMETER_VS_BRAKE_WARNING);
    mqttAction->setObjectIP(mqttObjectIP.s_addr);
    mqttAction->setExecuteDelayTime({1,0});

    addTrigger(bt);
    addAction(mqttAction);
    addAction(brakeLightAction);

    linkTriggerWithAction(bt, mqttAction);
    linkTriggerWithAction(bt, brakeLightAction);
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
                tp->update(static_cast<double>(monr.MONR.LongitudinalSpeedI16/100.0), monrTime);
                break;
            default:
                LogMessage(LOG_LEVEL_WARNING, "Unhandled trigger type in update: %s",
                           tp->getTypeAsString(tp->getTypeCode()).c_str());
            }
        }
    }
}

