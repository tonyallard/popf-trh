/*
 * PDDLDomainFactory.h
 *
 *  Created on: 18 Jul 2015
 *      Author: tony
 */

#ifndef __PDDL_PDDLDOMAINFACTORY
#define __PDDL_PDDLDOMAINFACTORY

#include "Proposition.h"
#include "TIL.h"
#include "PendingAction.h"
#include "PDDLDomain.h"

#include "../minimalstate.h"

using namespace std;

namespace PDDL {
class PDDLDomainFactory {
private:
	//Singleton Instance
	static PDDLDomainFactory * INSTANCE;

	//constants
	static const std::string REQUIRED_PROPOSITION;
	static const std::string TIL_ACHIEVED_PROPOSITION;
	
	//Private constructor
	PDDLDomainFactory(const VAL::domain * domain);
	//Singleton
	PDDLDomainFactory(PDDLDomainFactory const & other) {
	}
	;
	PDDLDomainFactory& operator=(PDDLDomainFactory const&) {
	}
	;

	//Instance Vars
	list<PDDL::PDDLObject> types;
	list<PDDL::Proposition> domainPredicates;
	list<PDDL::Proposition> functions;
	std::list<std::pair<std::string, std::string> > constants;
	list<std::string> domainOperators;
	list<std::string> domainOperatorNames;

	//Private Functions
	list<string> getDomainRequirements(VAL::pddl_req_flag flags,
			bool deTILed);
	list<PDDL::PDDLObject> getTypes(const VAL::pddl_type_list * types);

	list<PDDL::Proposition> getPredicates(const std::list<PendingAction> & pendingActions =
			std::list<PendingAction>(),
			const std::list<PDDL::Proposition> & tilPredicates = std::list<PDDL::Proposition>(),
			const std::list<PDDL::Proposition> & tilGoalPredicates = std::list<PDDL::Proposition>(),
			const std::list<PDDL::Proposition> & tilRequiredObjects = std::list<PDDL::Proposition>());
	
	list<PDDL::Proposition> getDomainPredicates(const VAL::pred_decl_list * predicates);
	list<PDDL::Proposition> getFunctions(const VAL::func_decl_list * functions);
	std::list<std::pair<std::string, std::string> > getConstantsFromDomain(
			const VAL::const_symbol_list * constants);
	list<std::string> getDomainOperators(const VAL::operator_list * operators);
	list<std::string> getDomainOperatorNames(const VAL::operator_list * operators);
	list<string> getActions(const std::list<PendingAction> & pendingActions,
	std::list<string> deTILedActions);
	std::string getDurativeAction(const VAL::durative_action * action);
	std::string getAction(const VAL::action * action);
	list<string> getdeTILedActions(std::list<TIL> tils,
		std::list<PDDL::Proposition> & tilActionPreconditions, 
		std::list<PDDL::Proposition> & tilGoalPredicates,
		std::list<PDDL::Proposition> & tilRequiredObjects,
		std::list<PDDL::Proposition> & tilRequiredObjectsParameterised);
	std::string getdeTILedAction(const TIL & til,
		std::list<PDDL::Proposition> & tilActionPreconditions,
		std::list<PDDL::Proposition> & tilGoalPredicates, 
		std::list<PDDL::Proposition> & tilRequiredObjects,
		std::list<PDDL::Proposition> & tilRequiredObjectsParameterised);
	std::list<PDDL::PendingAction> getPendingActions(
			const Planner::MinimalState & state, double timestamp, 
			std::set<PDDLObject> & objectSymbolTable,
			list<PDDL::Proposition> & pendingActionRequiredObjects);
	list<string> getPendingActions(const std::list<PendingAction> & pendingActions);
	std::list<PDDL::Proposition> getPendingActionRequiredObjectPropositions(
			string actionName, std::set<PDDLObject> parameters);
	std::string getConditions(const VAL::goal * goal, bool isForDurativeAction);
	std::list<PDDL::TIL> getTILs(const Planner::MinimalState & state, 
			std::set<PDDLObject> & objectSymbolTable);
	/*Methods to get condition literals*/
	std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > getConditions(
			int actionID, std::set<PDDLObject> & parameters);
	std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > convertLiterals_AddSignAndTime(
			std::list<Inst::Literal*> literals, VAL::time_spec timeQualifier,
			bool isPositive, std::set<PDDLObject> & parameters);

public:
	static PDDLDomainFactory * getInstance();

	//Needs to be public so PDDLStateFactory can use it to generate state
	inline const std::list<std::pair<std::string, std::string> > & getConstants() {return constants;}
	//Needs to be public so TRH can determine plan actions from Heuristic
	bool isDomainOperator(string name);

	PDDL::PDDLDomain getDomain(const VAL::domain * domain,
			const std::list<PendingAction> & pendingActions);

	PDDL::PDDLDomain getDeTILedDomain(
		const VAL::domain * domain, const Planner::MinimalState & state, 
		double timestamp);
};
}

#endif /* __PDDL_PDDLDOMAINFACTORY */
