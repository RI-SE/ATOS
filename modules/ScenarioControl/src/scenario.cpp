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

/*!
 * \brief Scenario::splitLine Splits a line at specified delimiter and stores the generated substrings in a vector (excluding the delimiters)
 * \param line Line to be split
 * \param delimiter Delimiter at which line is to be split
 * \param result Vector in which to store substrings generated by splitting operation
 */
void Scenario::splitLine(const std::string &line, const char delimiter, std::vector<std::string> &result)
{
    std::istringstream strm(line);
    std::string part;
    while (getline(strm,part,delimiter)) result.push_back(part);
    return;
}

/*!
 * \brief Scenario::parseScenarioFileLine Converts a line from the trigger and action configuration file into a number of linked triggers
 * and actions, and modifies members accordingly. Lines which are empty or start with a pound sign are ignored.
 * \param inputLine Line to be decoded
 */
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

    // Match relevant field according to below patterns
    regex ipAddrPattern("([0-2]?[0-9]?[0-9]\\.){3}([0-2]?[0-9]?[0-9])"); // Match 3 "<000-299>." followed by "<000-299>"
    regex triggerActionPattern("(([a-zA-Z_])+\\[([a-zA-Z0-9\\.,<=>_])+\\])+");
    in_addr triggerIP, actionIP;
    string errMsg;
    set<Action*> actions;
    set<Trigger*> triggers;

    // Expect a line to consist of trigger IP, triggers, action IP and actions in that order
    enum {TRIGGER_IP,TRIGGER,ACTION_IP,ACTION,DONE} parseState = TRIGGER_IP;

    for (const string &part : parts) {
        switch (parseState) {
        case TRIGGER_IP:
            if(!regex_match(part,ipAddrPattern)) {
                errMsg = "Specified trigger IP address field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            if(inet_pton(AF_INET, part.c_str(), &triggerIP) <= 0) {
                errMsg = "Error parsing IP string <" + part + ">";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }
            parseState = TRIGGER;
            break;
        case TRIGGER:
            if(!regex_match(part,triggerActionPattern)) {
                errMsg = "Trigger configuration field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            triggers = parseTriggerConfiguration(part);
            for (Trigger* tp : triggers) {
                tp->setObjectIP(triggerIP.s_addr);
                for (Trigger* knownTrigger : allTriggers)
                {
                    if(tp->isSimilar(*knownTrigger))
                    {
                        triggers.erase(tp);
                        triggers.insert(knownTrigger);
                        tp = knownTrigger;
                    }
                }
                addTrigger(tp);
            }
            parseState = ACTION_IP;
            break;
        case ACTION_IP:
            if(!regex_match(part,ipAddrPattern)) {
                errMsg = "Specified action IP address field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            if(inet_pton(AF_INET, part.c_str(), &actionIP) <= 0) {
                errMsg = "Error parsing IP string <" + part + ">";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }
            parseState = ACTION;
            break;
        case ACTION:
            if(!regex_match(part,triggerActionPattern)) {
                errMsg = "Action configuration field <" + part + "> is invalid";
                LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
                throw invalid_argument(errMsg);
            }

            actions = parseActionConfiguration(part);
            for (Action* ap : actions) {
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

/*!
 * \brief Scenario::parseTriggerConfiguration Parses a field from the trigger and action configuration file corresponding to trigger
 * configurations.
 * \param inputConfig The part of a configuration line which corresponds to trigger configurations
 * \return A set of trigger pointers according to what was found in the configuration
 */
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

    // Split the configuration to allow for several in a row
    splitLine(inputConfig, ']', configs);
    for (string &config : configs)
        config.append("]");

    // Loop through the specified triggers and generate a trigger for each
    for(const string &config : configs)
    {
        if (!regex_search(config, match, triggerPattern))
        {
            errMsg = "The following is not a valid configuration: <" + config + ">";
            LogMessage(LOG_LEVEL_ERROR,errMsg.c_str());
            throw invalid_argument(errMsg);
        }

        triggerType = Trigger::asTypeCode(match[1]);
        switch (triggerType) {
        // If the trigger type has Maestro monitoring implemented, use that
        case TRIGGER_BRAKE:
            trigger = new BrakeTrigger(baseTriggerID + static_cast<Trigger::TriggerID_t>(returnTriggers.size()));
            // TODO: possibly the OR between the Maestro trigger and possible TREO messages
            break;
        default:
            // Trigger with unimplemented Maestro monitoring: let object handle trigger reporting
            trigger = new ISOTrigger(baseTriggerID + static_cast<Trigger::TriggerID_t>(returnTriggers.size()), triggerType);
            break;
        }

        // Transfer specified parameters to relevant containers
        for (unsigned int i = 2; i < match.size(); ++i)
            if (!match[i].str().empty()) trigger->appendParameter(match[i].str());
        returnTriggers.insert(trigger);
    }

    return returnTriggers;
}

/*!
 * \brief Scenario::parseActionConfiguration Parses a field from the trigger and action configuration file corresponding to action
 * configurations.
 * \param inputConfig The part of a configuration line which corresponds to action configurations
 * \return A set of action pointers according to what was found in the configuration
 */
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

    // Split the configuration to allow for several in a row
    splitLine(inputConfig, ']', configs);
    for (string &config : configs)
        config.append("]");

    // Loop through specified actions and generate an action for each
    for(const string &config : configs)
    {
        if (!regex_search(config, match, triggerPattern))
        {
            errMsg = "The following is not a valid configuration: <" + config + ">";
            LogMessage(LOG_LEVEL_ERROR,errMsg.c_str());
            throw invalid_argument(errMsg);
        }

        actionType = Action::asTypeCode(match[1]);
        switch (actionType) {
        // Handle any specialised action types
        case ACTION_INFRASTRUCTURE:
            action = new InfrastructureAction(baseActionID + static_cast<Action::ActionID_t>(returnActions.size()));
            break;
        default:
            // Regular action (only ACCM and EXAC)
            action = new ExternalAction(baseActionID + static_cast<Action::ActionID_t>(returnActions.size()), actionType);
            break;
        }

        // Transfer specified parameters to relevant containers
        for (unsigned int i = 2; i < match.size(); ++i)
            if(!match[i].str().empty()) action->appendParameter(match[i].str());
        returnActions.insert(action);
    }

    return returnActions;
}

/*!
 * \brief Scenario::parseScenarioFile Parses the trigger and action configuration file into relevant triggers and actions, and links between them.
 * \param file File stream for an open configuration file
 */
void Scenario::parseScenarioFile(std::ifstream &file)
{
    LogMessage(LOG_LEVEL_DEBUG, "Parsing scenario file");

    std::string line;
    while ( std::getline(file, line) ) parseScenarioFileLine(line);
    return;
}

/*!
 * \brief Scenario::addTrigger Appends a trigger to the list of known triggers, unless another trigger of the same ID is already in place.
 * \param tp Trigger pointer to be added
 * \return Return code according to ::ScenarioReturnCode_t
 */
Scenario::ScenarioReturnCode_t Scenario::addTrigger(Trigger* tp)
{
    for (Trigger* knownTrigger : allTriggers)
        if (knownTrigger->getID() == tp->getID()) return DUPLICATE_ELEMENT;

    LogMessage(LOG_LEVEL_DEBUG,"Adding trigger with ID: %d",tp->getID());
    allTriggers.insert(tp);
    return OK;
}

/*!
 * \brief Scenario::addAction Appends an action to the list of known actions, unless another action of the same ID is already in place.
 * \param ap Action pointer to be added
 * \return Return code according to ::ScenarioReturnCode_t
 */
Scenario::ScenarioReturnCode_t Scenario::addAction(Action* ap)
{
    for (Action* knownAction : allActions)
        if (knownAction->getID() == ap->getID()) return DUPLICATE_ELEMENT;

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
				if (monr.data.speed.isValid && monr.data.isTimestampValid)
				{
					tp->update(monr.data.speed.longitudinal_m_s, monr.data.timestamp);
				}
				else
				{
					LogMessage(LOG_LEVEL_WARNING, "Could not update trigger type %s due to invalid monitor data values",
							   tp->getTypeAsString(tp->getTypeCode()).c_str());
				}
                break;
            default:
                LogMessage(LOG_LEVEL_WARNING, "Unhandled trigger type in update: %s",
                           tp->getTypeAsString(tp->getTypeCode()).c_str());
            }
        }
    }
}

