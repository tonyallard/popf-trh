/*
 * PDDLDomainFactory.cpp
 *
 *  Created on: 18 Jul 2015
 *      Author: tony
 */

#include <string>
#include <sstream>
#include <algorithm>
#include <list>

#include "ptree.h"

#include "PDDLDomainFactory.h"
#include "TIL.h"
#include "PDDLUtils.h"
#include "PropositionFactory.h"
#include "TILFactory.h"
#include "PNEFactory.h"
#include "ExpressionTree.h"
#include "../TRH/TRH.h"

#include "../RPGBuilder.h"
#include "../temporalanalysis.h"

using namespace std;

namespace PDDL {

const string PDDLDomainFactory::TIL_ACHIEVED_PROPOSITION = "til-achieved";
const string PDDLDomainFactory::REQUIRED_PROPOSITION = "required";

PDDLDomainFactory * PDDLDomainFactory::INSTANCE = NULL;

PDDLDomainFactory * PDDLDomainFactory::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new PDDLDomainFactory(VAL::current_analysis->the_domain);
	}
	return INSTANCE;
}

PDDLDomainFactory::PDDLDomainFactory(const VAL::domain * domain) {
	types = getTypes(domain->types);
	domainPredicates = getDomainPredicates(domain->predicates);
	functions = getFunctions(domain->functions);
	constants = getConstantsFromDomain(domain->constants);
	domainOperators = getDomainOperators(domain->ops);
	domainOperatorNames = getDomainOperatorNames(domain->ops);
}

PDDL::PDDLDomain PDDLDomainFactory::getDomain(const VAL::domain * domain,
		const std::list<PendingAction> & pendingActions) {
	string name = domain->name;
	list<string> requirements = getDomainRequirements(domain->req, false);
	list<PDDL::Proposition> predicates = getPredicates(pendingActions);
	return PDDLDomain(name, requirements, types, predicates, functions,
			constants, domainOperators);
}

PDDL::PDDLDomain PDDLDomainFactory::getDeTILedDomain(const VAL::domain * domain,
		const Planner::MinimalState & state, double timestamp) {

	//Construct De-TILed Features
	list<string> deTILedActions;
	std::list<PDDL::Proposition> tilPredicates;
	std::list<PDDL::Proposition> tilGoalPredicates;
	std::list<PDDL::Proposition> tilRequiredObjects;
	std::set<PDDLObject> domainObjectSymbolTable;
	std::list<PDDL::Proposition> tilRequiredObjectsParameterised;
	list<PDDL::Proposition> pendingActionRequiredObjects;

	std::list<PDDL::TIL> tils = getTILs(state, domainObjectSymbolTable);
	bool hasTils = tils.size();
	if (hasTils) {
		deTILedActions = getdeTILedActions(tils, tilPredicates, tilGoalPredicates,
				tilRequiredObjects, tilRequiredObjectsParameterised);
	}
	std::list<PDDL::PendingAction> pendingActions = getPendingActions(state,
			timestamp, domainObjectSymbolTable, pendingActionRequiredObjects);

	//Construct the remainder of the domain
	string name = domain->name;
	list<string> requirements = getDomainRequirements(domain->req, hasTils);
	list<PDDL::PDDLObject> types = getTypes(domain->types);
	list<PDDL::Proposition> predicates = getPredicates(pendingActions,
			tilPredicates, tilGoalPredicates, tilRequiredObjectsParameterised);
	list<string> actions = getActions(pendingActions, deTILedActions);

	return PDDLDomain(name, requirements, types, predicates, functions,
			constants, actions, tils, tilPredicates, tilGoalPredicates, tilRequiredObjects,
			pendingActionRequiredObjects, domainObjectSymbolTable);
}

bool PDDLDomainFactory::isDomainOperator(string name) {
	std::list<string>::const_iterator opItr = domainOperatorNames.begin();
	for (; opItr != domainOperatorNames.end(); opItr++) {
		if (name.compare(*opItr) == 0)  {
			return true;
		}
	}
	return false;
}

list<string> PDDLDomainFactory::getDomainRequirements(VAL::pddl_req_flag flags,
		bool deTILed) {
	list<string> result;
	//Explicitly Add Equality because
	//They are used in de-tiled domains (for de-tiled actions)
	if ((flags & VAL::E_EQUALITY) || deTILed)
		result.push_back(":equality ");
	if (flags & VAL::E_STRIPS)
		result.push_back(":strips ");
	if (flags & VAL::E_TYPING)
		result.push_back(":typing ");
	if (flags & VAL::E_DISJUNCTIVE_PRECONDS)
		result.push_back(":disjunctive-preconditions ");
	if (flags & VAL::E_EXT_PRECS)
		result.push_back(":existential-preconditions ");
	if (flags & VAL::E_UNIV_PRECS)
		result.push_back(":universal-preconditions ");
	if (flags & VAL::E_COND_EFFS)
		result.push_back(":conditional-effects ");
	if (flags & VAL::E_NFLUENTS)
		result.push_back(":fluents ");
	if (flags & VAL::E_OFLUENTS)
		result.push_back(":object-fluents ");
	if (flags & VAL::E_ACTIONCOSTS)
		result.push_back(":action-costs ");
	if (flags & VAL::E_DURATIVE_ACTIONS)
		result.push_back(":durative-actions ");
	if (flags & VAL::E_DURATION_INEQUALITIES)
		result.push_back(":duration-inequalities ");
	if (flags & VAL::E_CONTINUOUS_EFFECTS)
		result.push_back(":continuous-effects ");
	//Explicitly Add Negative PreConditions because
	//They are used in de-tiled domains (for de-tiled actions)
	if ((flags & VAL::E_NEGATIVE_PRECONDITIONS) || deTILed)
		result.push_back(":negative-preconditions ");
	if (flags & VAL::E_DERIVED_PREDICATES)
		result.push_back(":derived-predicates ");
	//If deTiled ignore TIL requirement
	if ((flags & VAL::E_TIMED_INITIAL_LITERALS) && !deTILed)
		result.push_back(":timed-initial-literals ");
	if (flags & VAL::E_PREFERENCES)
		result.push_back(":preferences ");
	if (flags & VAL::E_CONSTRAINTS)
		result.push_back(":constraints ");
	if (flags & VAL::E_TIME)
		result.push_back(":time ");
	return result;
}

list<PDDL::PDDLObject> PDDLDomainFactory::getTypes(
		const VAL::pddl_type_list * types) {
	list<PDDL::PDDLObject> typeList;
	if (types) {
		VAL::pddl_type_list::const_iterator typeItr = types->begin();
		for (; typeItr != types->end(); typeItr++) {
			const VAL::pddl_type * type = *typeItr;
			PDDL::PDDLObject pddlObject = getPDDLObject(type);
			typeList.push_back(pddlObject);
		}
	}
	return typeList;
}

list<PDDL::Proposition> PDDLDomainFactory::getDomainPredicates(
		const VAL::pred_decl_list * predicates) {
	list<PDDL::Proposition> domainPropositions;
	//get standard domain predicates
	VAL::pred_decl_list::const_iterator predItr = predicates->begin();
	for (; predItr != predicates->end(); predItr++) {
		const VAL::pred_decl * predicate = *predItr;
		PDDL::Proposition domProp =
				PropositionFactory::getInstance()->getProposition(predicate);
		domainPropositions.push_back(domProp);
	}
	return domainPropositions;
}

list<PDDL::Proposition> PDDLDomainFactory::getPredicates(
		const std::list<PendingAction> & pendingActions /*=empty list*/,
		const std::list<PDDL::Proposition> & tilPredicates /*=empty list*/,
		const std::list<PDDL::Proposition> & tilGoalPredicates /*==empty list*/,
		const std::list<PDDL::Proposition> & tilRequiredObjects /*=empty list*/) {
	list<PDDL::Proposition> propositions;

	//Add Domain Predicates
	propositions.insert(propositions.end(), domainPredicates.begin(),
			domainPredicates.end());
	//Add Predicates for de-TILed Actions
	//This ensures correct TIL ordering
	propositions.insert(propositions.end(), tilPredicates.begin(),
			tilPredicates.end());
	//This ensures all the TILs are achieved
	propositions.insert(propositions.end(), tilGoalPredicates.begin(),
			tilGoalPredicates.end());
	//Add Required Predicates for Partial Actions
	if (pendingActions.size()) {
		std::list<PendingAction>::const_iterator pendActItr =
				pendingActions.begin();
		for (; pendActItr != pendingActions.end(); pendActItr++) {
			list<PDDL::Proposition> requiredPredicates =
					pendActItr->getRequiredPropositionsParameterised();
			propositions.insert(propositions.end(), requiredPredicates.begin(),
					requiredPredicates.end());
		}
	}
	//Add Required Predicates for De-TILed Actions
	//This ensures the correct Objects are used for a TIL
	propositions.insert(propositions.end(), tilRequiredObjects.begin(),
			tilRequiredObjects.end());
	
	return propositions;
}

list<PDDL::Proposition> PDDLDomainFactory::getFunctions(
		const VAL::func_decl_list * functions) {
	list<PDDL::Proposition> functionList;
	//Guard against domains with no functions
	if (functions == 0) {
		return functionList;
	}
	VAL::func_decl_list::const_iterator funcItr = functions->begin();
	for (; funcItr != functions->end(); funcItr++) {
		PDDL::Proposition function = getFunction(*funcItr);
		functionList.push_back(function);
	}
	return functionList;
}

std::list<std::pair<std::string, std::string> > PDDLDomainFactory::getConstantsFromDomain(
		const VAL::const_symbol_list * constants) {
	std::list<std::pair<std::string, std::string> > constantList;
	if (constants) {
		VAL::const_symbol_list::const_iterator constItr = constants->begin();
		for (; constItr != constants->end(); constItr++) {
			const VAL::const_symbol * con = *constItr;
			string name = con->getName();
			std::transform(name.begin(), name.end(), name.begin(), ::toupper);
			std::pair<std::string, std::string> constant(name,
					getPDDLTypeString(con));
			constantList.push_back(constant);
		}
	}
	return constantList;
}

list<string> PDDLDomainFactory::getDomainOperators(
		const VAL::operator_list * operators) {
	list<string> domOperators;
	VAL::operator_list::const_iterator opsItr = operators->begin();
	for (; opsItr != operators->end(); opsItr++) {
		const VAL::operator_ * op = *opsItr;
		//Determine type of action
		const VAL::durative_action * dur_action =
				dynamic_cast<const VAL::durative_action *>(op);
		const VAL::action * action = dynamic_cast<const VAL::action *>(op);
		if (dur_action) {
			domOperators.push_back(getDurativeAction(dur_action));
		} else {
			domOperators.push_back(getAction(action));
		}
	}
	return domOperators;
}

list<string> PDDLDomainFactory::getDomainOperatorNames(
		const VAL::operator_list * operators) {
	list<string> domOperatorNames;
	VAL::operator_list::const_iterator opsItr = operators->begin();
	for (; opsItr != operators->end(); opsItr++) {
		const VAL::operator_ * op = *opsItr;
		domOperatorNames.push_back(op->name->getName());
	}
	return domOperatorNames;
}

list<string> PDDLDomainFactory::getActions(
		const std::list<PendingAction> & pendingActions,
		std::list<string> deTILedActions) {
	list<string> actions;
	//Add Domain Operators
	actions.insert(actions.end(), domainOperators.begin(),
			domainOperators.end());
	//Add Pending Actions
	list<string> pendingActionList = getPendingActions(pendingActions);
	actions.insert(actions.end(), pendingActionList.begin(),
			pendingActionList.end());
	//Add deTILed Actions
	actions.insert(actions.end(), deTILedActions.begin(), deTILedActions.end());
	return actions;
}

string PDDLDomainFactory::getDurativeAction(
		const VAL::durative_action * action) {
	ostringstream output;
	//get header
	output << "\t(:durative-action " << action->name->getName() << endl;
	//get parameters
	output << "\t\t:parameters (" << getArgumentString(action->parameters)
			<< ")" << endl;
	//get duration
	const VAL::timed_goal * duration =
			dynamic_cast<const VAL::timed_goal *>(action->dur_constraint);
	const VAL::comparison * durationalGoal =
			dynamic_cast<const VAL::comparison *>(duration->getGoal());
	output << "\t\t:duration " << getExpressionString(durationalGoal) << endl;
	//get conditions
	output << "\t\t:condition" << endl
			<< getGoalString(action->precondition, 3) << endl;
	//get effects
	output << "\t\t:effect" << endl << getEffectsString(action->effects)
			<< endl;
	output << "\t)";
	return output.str();
}

string PDDLDomainFactory::getAction(const VAL::action * action) {
	ostringstream output;
	//get header
	output << "\t(:action " << action->name->getName() << endl;
	//get parameters
	output << "\t\t:parameters (" << getArgumentString(action->parameters)
			<< ")" << endl;
	//get preconditions
	output << "\t\t:precondition" << endl
			<< getGoalString(action->precondition, 3) << endl;
	//get effects
	output << "\t\t:effect" << endl << getEffectsString(action->effects)
			<< endl;
	output << "\t)";
	return output.str();
}

list<string> PDDLDomainFactory::getdeTILedActions(std::list<TIL> tils,
		std::list<PDDL::Proposition> & tilActionPreconditions,
		std::list<PDDL::Proposition> & tilGoalPredicates,
		std::list<PDDL::Proposition> & tilRequiredObjects,
		std::list<PDDL::Proposition> & tilRequiredObjectsParameterised) {
	//sort list by timestamp
	tils.sort(PDDL::TIL::TILTimestampComparator);
	//Look at each TIL one at a time;
	list<string> actions;
	std::list<TIL>::const_reverse_iterator tilItr = tils.rbegin();
	const std::list<TIL>::const_reverse_iterator tilItrEnd = tils.rend();
	for (; tilItr != tilItrEnd; tilItr++) {
		string deTILedAction = getdeTILedAction(*tilItr, tilActionPreconditions,
				tilGoalPredicates, tilRequiredObjects, 
				tilRequiredObjectsParameterised);
		actions.push_back(deTILedAction);
	}
	return actions;
}

string PDDLDomainFactory::getdeTILedAction(const TIL & til,
		std::list<PDDL::Proposition> & tilActionPreconditions,
		std::list<PDDL::Proposition> & tilGoalPredicates,
		std::list<PDDL::Proposition> & tilRequiredObjects,
		std::list<PDDL::Proposition> & tilRequiredObjectsParameterised) {
	//Create special proposition for this TIL
	std::list<string> arguments;
	PDDL::Proposition tilLit(til.getName(), arguments);
	PDDL::Proposition tilComplete(til.getName() + "-complete", arguments);
	//Find all parameters for this TIL Action and generate the paramtable
	std::map<PDDLObject, std::string> parameterTable =
			PDDL::generateParameterTable(til.getParameters());
	//List of objects that are required
	std::list<PDDL::Proposition> requiredObjects;

	//Generate action string
	ostringstream output;
	output << "\t(:action " << til.getName() << endl;
	//Add parameters and create the required predicates on the way
	output << "\t\t:parameters( ";
	std::map<PDDLObject, std::string>::const_iterator paramItr =
			parameterTable.begin();
	int paramNum = 1;
	for (; paramItr != parameterTable.end(); paramItr++) {
		output << paramItr->second << " - " << paramItr->first.getTypeString()
				<< " ";
		//Proposition to ensure correct objects are used
		ostringstream propName;
		propName << PDDLDomainFactory::REQUIRED_PROPOSITION << "-"
				<< til.getName() << "-" << paramNum;
		std::list<std::string> args;
		args.push_back(paramItr->first.getName());
		PDDL::Proposition paramRequired(propName.str(), args);
		requiredObjects.push_back(paramRequired);
		tilRequiredObjectsParameterised.push_back(
				paramRequired.getParameterisedProposition(parameterTable,
						true));
		paramNum++;
	}
	output << ")" << endl;

	// Pre-conditions
	output << "\t\t:precondition (and" << endl;
	//Add requiredment that TIL happens only once (one shot)
	output << "\t\t\t" << tilLit << std::endl;

	//Add predicates to ensure correct objects are used
	std::list<PDDL::Proposition>::const_iterator reqObjItr =
			requiredObjects.begin();
	for (; reqObjItr != requiredObjects.end(); reqObjItr++) {
		output << "\t\t\t" << reqObjItr->toParameterisedString(parameterTable)
				<< endl;
	}

	output << "\t\t)" << endl;
	
	//Add Effects
	output << "\t\t:effect (and" << endl;
	std::list<PDDL::Proposition>::const_iterator addItr =
			til.getAddEffects().begin();
	const std::list<PDDL::Proposition>::const_iterator addItrEnd =
			til.getAddEffects().end();
	for (; addItr != addItrEnd; addItr++) {
		output << "\t\t\t" << addItr->toParameterisedString(parameterTable)
				<< endl;
	}
	// add special predicate to indicate that the til is complete
	// This is the one shot to remove the TIL requirement
	output << "\t\t\t(not " << tilLit << ")" << endl;
	output << "\t\t\t" << tilComplete << endl;
	// Add effect to engage next TIL
	if (!tilActionPreconditions.empty()) {
		output << "\t\t\t" << *tilActionPreconditions.rbegin() << endl;
	}

	// Del Effects
	// Add predicates to make cargo unavailable
	std::list<PDDL::Proposition>::const_iterator delItr =
			til.getDelEffects().begin();
	const std::list<PDDL::Proposition>::const_iterator delItrEnd =
			til.getDelEffects().end();
	for (; delItr != delItrEnd; delItr++) {
		output << "\t\t\t(not " << delItr->toParameterisedString(parameterTable)
				<< ")" << endl;
	}

	//Remove Predicates that required certain objects
	reqObjItr = requiredObjects.begin();
	for (; reqObjItr != requiredObjects.end(); reqObjItr++) {
		output << "\t\t\t(not "
				<< reqObjItr->toParameterisedString(parameterTable) << ")"
				<< endl;
	}

	//Footer
	output << "\t\t)" << endl << "\t)";
	//Add Precondition that tracks completion towards the goal
	tilGoalPredicates.push_back(tilComplete);
	//Add the TIL Proposition to the list of preconditions for future TILS
	//This retains precedence ordering of TILs
	tilActionPreconditions.push_back(tilLit);
	//Add required objects to domain list
	tilRequiredObjects.insert(tilRequiredObjects.end(), requiredObjects.begin(),
			requiredObjects.end());
	return output.str();
}

/**
 * Get the actions in a state that are in the middle of executing.
 * For example the start snap action has been executed, but not the end snap action
 */
std::list<PDDL::PendingAction> PDDLDomainFactory::getPendingActions(
		const Planner::MinimalState & state, double timestamp, 
		std::set<PDDLObject> & objectSymbolTable,
		list<PDDL::Proposition> & pendingActionRequiredObjects) {
	std::list<PendingAction> pendingActions;
	//Cycle through Facts held up by executing actions (these are effects coming into play)
	std::map<int, std::set<int> >::const_iterator saItr =
			state.startedActions.begin();
	const std::map<int, std::set<int> >::const_iterator saItrEnd =
			state.startedActions.end();

	for (; saItr != saItrEnd; saItr++) {

		Inst::instantiatedOp* action = Planner::RPGBuilder::getInstantiatedOp(
			saItr->first);
		std::string name = PDDL::getOperatorName(action);
		std::set<PDDLObject> parameters;

		//For each action get its conditions
		std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > conditions =
				getConditions(saItr->first, parameters);

		// Get action duration
		double minDur = Planner::RPGBuilder::getOpMinDuration(saItr->first, 1);
		double maxDur = Planner::RPGBuilder::getOpMaxDuration(saItr->first, 1);
		//Duration inequalities not supported
		if (minDur != maxDur) {
			cerr << "Duration inequalities not yet supported. Taking the mean." 
				<< endl;
			minDur = (minDur + maxDur) / 2;
		}
		//Determine how long the action has been running for 
		//and compute the remaining duraiton
		double actionStartTime = 0.0;
		const vector<pair<double, double> > & tsBounds = 
			Planner::TemporalAnalysis::getActionTSBounds()[saItr->first];
		const double startMin = tsBounds[0].first;
        const double startMax = tsBounds[0].second;

		const double endMin = tsBounds[1].first;
        const double endMax = tsBounds[1].second;

		actionStartTime = startMax;

		//Calculate remaining duration
		double remainingDur = minDur - (timestamp - actionStartTime);
		//ensure it is above 0
		remainingDur = max(EPSILON, remainingDur);
		//ensure it is no longer than max duraiton
		remainingDur = min(minDur, remainingDur);

		// Literals Added by Action
		std::list<std::pair<PDDL::Proposition, VAL::time_spec> > propositionalAddEffects;
		std::list<Inst::Literal*> adds =
				Planner::RPGBuilder::getEndPropositionAdds()[saItr->first];
		std::list<Inst::Literal*>::const_iterator addItr = adds.begin();
		const std::list<Inst::Literal*>::const_iterator addItrEnd = adds.end();
		for (; addItr != addItrEnd; addItr++) {
			const VAL::proposition * realProp = (*addItr)->toProposition();
			parameters = extractParameters(*addItr, parameters, constants);
			Proposition prop =
					PropositionFactory::getInstance()->getProposition(realProp);
			propositionalAddEffects.push_back(
					std::pair<PDDL::Proposition, VAL::time_spec>(prop,
							VAL::time_spec::E_AT_END));
		}

		// Literals Deleted by Action
		std::list<std::pair<PDDL::Proposition, VAL::time_spec> > propositionalDelEffects;
		std::list<Inst::Literal*> deletes =
				Planner::RPGBuilder::getEndPropositionDeletes()[saItr->first];
		std::list<Inst::Literal*>::const_iterator delItr = deletes.begin();
		const std::list<Inst::Literal*>::const_iterator delItrEnd =
				deletes.end();
		for (; delItr != delItrEnd; delItr++) {
			const VAL::proposition * realProp = (*delItr)->toProposition();
			parameters = extractParameters(*delItr, parameters, constants);
			Proposition prop =
					PropositionFactory::getInstance()->getProposition(realProp);
			propositionalDelEffects.push_back(
					std::pair<PDDL::Proposition, VAL::time_spec>(prop,
							VAL::time_spec::E_AT_END));
		}

		// PNEs Added by Action
		std::list<std::pair<PDDL::PNEEffect, VAL::time_spec> > pneEffects;
		list<Planner::RPGBuilder::NumericEffect> endNumEffects =
				Planner::RPGBuilder::getActionsToEndNumericEffects()[saItr->first];
		list<Planner::RPGBuilder::NumericEffect>::const_iterator numEffItr =
				endNumEffects.begin();
		for (; numEffItr != endNumEffects.end(); numEffItr++) {
			Planner::RPGBuilder::NumericEffect numEff = *numEffItr;
			Inst::PNE* aPNE = Planner::RPGBuilder::getPNE(numEff.fluentIndex);
			parameters = extractParameters(aPNE, parameters, constants);
			parameters = extractParameters(&numEff.formula, parameters, constants);
			PDDL::PNE pne = PNEFactory::getInstance()->getPNE(aPNE, 0);
			PNEEffect pneEffect(pne, numEff.op, numEff.formula);
			pneEffects.push_back(std::pair<PDDL::PNEEffect, VAL::time_spec>(pneEffect,
					VAL::time_spec::E_AT_END));
		}
		//Get Pending Action Required Objects
		list<PDDL::Proposition> requiredObjects =
				getPendingActionRequiredObjectPropositions(name, parameters);
		//Add required objects to conditions and effects
		list<PDDL::Proposition>::const_iterator reqObjItr =
				requiredObjects.begin();
		for (; reqObjItr != requiredObjects.end(); reqObjItr++) {

			std::pair<VAL::time_spec, bool> required(VAL::time_spec::E_AT_START,
					true);
			VAL::time_spec notRequiredTimeSpec = VAL::time_spec::E_AT_END;

			pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > requiredCondition(
					*reqObjItr, required);
			pair<PDDL::Proposition, VAL::time_spec> notRequiredEffect(
					*reqObjItr, notRequiredTimeSpec);
			conditions.push_back(requiredCondition);
			propositionalDelEffects.push_back(notRequiredEffect);
		}
		//Insert all required objects into master list for state
		pendingActionRequiredObjects.insert(pendingActionRequiredObjects.end(),
				requiredObjects.begin(), requiredObjects.end());

		// insert all parameters into master table
		objectSymbolTable.insert(parameters.begin(), parameters.end());
		//Generate symbol table for this action
		map<PDDLObject, string> parameterTable = PDDL::generateParameterTable(
				parameters);

		PendingAction pendingAction(name, parameterTable,
				propositionalAddEffects, propositionalDelEffects, pneEffects,
				conditions, requiredObjects, remainingDur);

		pendingActions.push_back(pendingAction);
	}
	return pendingActions;
}

list<string> PDDLDomainFactory::getPendingActions(
		const std::list<PendingAction> & pendingActions) {
	list<string> pendingActionList;
	std::list<PendingAction>::const_iterator actItr = pendingActions.begin();
	const std::list<PendingAction>::const_iterator actItrEnd =
			pendingActions.end();
	for (; actItr != actItrEnd; actItr++) {
		ostringstream pendingAction;
		pendingAction << *actItr;
		pendingActionList.push_back(pendingAction.str());
	}
	return pendingActionList;
}

std::list<PDDL::Proposition> PDDLDomainFactory::getPendingActionRequiredObjectPropositions(
		string actionName, std::set<PDDLObject> parameters) {
	list<PDDL::Proposition> requiredObjects;
	std::set<PDDLObject>::const_iterator paramItr = parameters.begin();
	int paramNum = 1;
	for (; paramItr != parameters.end(); paramItr++) {
		ostringstream name;
		name << PDDLDomainFactory::REQUIRED_PROPOSITION << "-" << actionName
				<< "-" << paramNum;
		paramNum++;
		list<string> params;
		string param = paramItr->getName();
		transform(param.begin(), param.end(), param.begin(), ::toupper);
		params.push_back(param);
		PDDL::Proposition prop(name.str(), params);
		requiredObjects.push_back(prop);
	}
	return requiredObjects;
}

std::list<PDDL::TIL> PDDLDomainFactory::getTILs(
		const Planner::MinimalState & state, std::set<PDDLObject> & objectSymbolTable) {

	std::list<PDDL::TIL> tils;
	//Cycle thourgh normals TILs
	vector<Planner::RPGBuilder::FakeTILAction*> theTILs = Planner::RPGBuilder::getTILVec();

	for (int tilItr = state.nextTIL; tilItr < theTILs.size(); tilItr++) {
		const Planner::RPGBuilder::FakeTILAction * tilAction = theTILs[tilItr];

		PDDL::extractParameters(tilAction, objectSymbolTable, constants);
		PDDL::TIL til = PDDL::TILFactory::getInstance()->getTIL(*tilAction, tilItr, constants);
		tils.push_back(til);
	}
	return tils;
}

/**
 * Get all conditions and return parameters
 */
std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > PDDLDomainFactory::getConditions(
		int actionID, std::set<PDDLObject> & parameters) {

	std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > conditions;

	/*Postconditions*/
	std::list<Inst::Literal*> positivePostConditions =
			Planner::RPGBuilder::getEndPropositionalPreconditions()[actionID];
	std::list<Inst::Literal*> negativePostConditions =
			Planner::RPGBuilder::getEndNegativePropositionalPreconditions()[actionID];

	std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > tempConditions =
			convertLiterals_AddSignAndTime(positivePostConditions,
					VAL::time_spec::E_AT_END, true, parameters);
	conditions.insert(conditions.end(), tempConditions.begin(),
			tempConditions.end());

	tempConditions = convertLiterals_AddSignAndTime(negativePostConditions,
			VAL::time_spec::E_AT_END, false, parameters);
	conditions.insert(conditions.end(), tempConditions.begin(),
			tempConditions.end());

	/*Invariant Conditions*/
	std::list<Inst::Literal*> positiveInvariantConditions =
			Planner::RPGBuilder::getInvariantPropositionalPreconditions()[actionID];

	std::list<Inst::Literal*> negativeInvariantConditions =
			Planner::RPGBuilder::getInvariantNegativePropositionalPreconditions()[actionID];

	tempConditions = convertLiterals_AddSignAndTime(positiveInvariantConditions,
			VAL::time_spec::E_OVER_ALL, true, parameters);
	conditions.insert(conditions.end(), tempConditions.begin(),
			tempConditions.end());

	tempConditions = convertLiterals_AddSignAndTime(negativeInvariantConditions,
			VAL::time_spec::E_OVER_ALL, false, parameters);
	conditions.insert(conditions.end(), tempConditions.begin(),
			tempConditions.end());
	return conditions;
}

/**
 * Convert Colin's Literals to our datamodel and add their sign and when they apply, with respect to the action
 */
std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > PDDLDomainFactory::convertLiterals_AddSignAndTime(
		std::list<Inst::Literal*> literals, VAL::time_spec timeQualifier,
		bool isPositive, std::set<PDDLObject> & parameters) {
	std::list<pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > > conditions;

	std::list<Inst::Literal*>::const_iterator condItr = literals.begin();
	std::list<Inst::Literal*>::const_iterator condItrEnd = literals.end();

	for (; condItr != condItrEnd; condItr++) {
		const VAL::proposition * realCond = (*condItr)->toProposition();
		//This is required to ensure the parameters are created
		PDDL::extractParameters(*condItr, parameters, constants);
		PDDL::Proposition literal =
				PropositionFactory::getInstance()->getProposition(realCond);
		conditions.push_back(
				pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> >(
						literal,
						std::pair<VAL::time_spec, bool>(timeQualifier,
								isPositive)));
	}
	return conditions;
}

}
