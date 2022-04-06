#include "causality.h"
#include "logging.h"

namespace maestro {
	Causality::Causality(Causality::TriggerOperator_t op)
	{
		oper = op;
		actions = std::set<Action*>();
		triggers = std::set<Trigger*>();
	}

	Causality::Causality(Action* ap, TriggerOperator_t op)
	{
		actions.insert(ap);
		oper = op;
		triggers = std::set<Trigger*>();
	}

	Causality::Causality(Trigger* tp, Action* ap, TriggerOperator_t op)
	{
		triggers.insert(tp);
		actions.insert(ap);
		oper = op;
	}

	Action* Causality::getActionByID(Action::ActionID_t id) const
	{
		for (Action* ap : actions)
		{
			if (ap->getID() == id)
				return ap;
		}
		return nullptr;
	}

	Trigger* Causality::getTriggerByID(Trigger::TriggerID_t id) const
	{
		for (Trigger* tp : triggers)
		{
			if (tp->getID() == id)
				return tp;
		}
		return nullptr;
	}

	std::string Causality::getOperatorString() const
	{
		return oper == OR ? "OR" : "AND";
	}

	void Causality::executeIfActive(void) const
	{
		if (isActive())
		{
			for (Action* ap : actions)
			{
				ap->execute();
			}
		}
	}

	bool Causality::isActive() const
	{
		bool active = oper == AND;
		for (Trigger* tp : triggers)
		{
			active = (oper == OR) ? active || tp->isActive() : active && tp->isActive();
		}
		return active;
	}

	bool Causality::operator==(const Causality &other) const
	{
		for (Trigger* other_tp : other.getTriggers())
		{
			if (triggers.find(other_tp) == triggers.end())
				return false;
		}

		for (Action* other_ap : other.getActions())
		{
			if (actions.find(other_ap) == actions.end())
				return false;
		}

		return other.getOperator() == oper;
	}

	bool Causality::operator<(const Causality &other) const
	{
		std::set<Trigger*>::iterator trigIterThis, trigIterOther;
		std::set<Action*>::iterator actIterThis, actIterOther;

		if (*this == other) return false;

		if (other.getOperator() != oper) return oper == OR;

		// They have the same operator, check which has more triggers
		if (other.getTriggers().size() != triggers.size())
		{
			return triggers.size() < other.getTriggers().size();
		}

		// They have the same amount of triggers, check which has more actions
		if (other.getActions().size() != actions.size())
		{
			return actions.size() < other.getActions().size();
		}

		// They are of equal size, compare trigger IDs returning the first found less than
		for (trigIterThis = triggers.begin(), trigIterOther = other.getTriggers().begin();
			trigIterThis != triggers.end() && trigIterOther != other.getTriggers().end();
			++trigIterThis, ++trigIterOther){
			if ((*trigIterThis)->getID() != (*trigIterOther)->getID())
				return (*trigIterThis)->getID() < (*trigIterOther)->getID();
		}

		// All triggers are equal, compare action IDs returning the first found less than
		for (actIterThis = actions.begin(), actIterOther = other.getActions().begin();
			actIterThis != actions.end() && actIterOther != other.getActions().end();
			++actIterThis, ++actIterOther){
			if ((*actIterThis)->getID() != (*actIterOther)->getID())
				return (*actIterThis)->getID() < (*actIterOther)->getID();
		}

		// Since we checked for equality at start, we should never get here
		return false;
	}
}