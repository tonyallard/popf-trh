/*
 * PDDLStateFactory.cpp
 *
 *  Created on: 2 Jul 2015
 *      Author: tony
 */

#include <string>
#include <list>
#include <set>
#include <sstream>

#include "PDDLStateFactory.h"
#include "PDDLUtils.h"
#include "PDDLDomainFactory.h"
#include "PropositionFactory.h"
#include "PNEFactory.h"

#include "../RPGBuilder.h"

using namespace std;

namespace PDDL {

const std::string PDDLStateFactory::DEFAULT_METRIC_PNE = "total-time";

PDDLStateFactory::PDDLStateFactory(const Planner::MinimalState & initialState,
		std::list<std::pair<std::string, std::string> > constants) {
	this->constants = constants;
	std::list<PDDL::Proposition> stdPropositions = getPropositions(initialState,
			objectParameterTable);
	staticPropositions = getStaticPropositions(stdPropositions,
			objectParameterTable);
	std::list<PDDL::PNE> stdPNEs = getPNEs(initialState, objectParameterTable);
	staticPNEs = getStaticPNEs(stdPNEs, objectParameterTable);
	goals = getPropositionalGoals(objectParameterTable);
	metric = getMetric();
}

// PDDLState PDDLStateFactory::getPDDLState(const MinimalState & state,
// 		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic) {
// 	std::set<PDDLObject> objectSymbolTable = this->objectParameterTable;

// 	std::list<Proposition> propositions = PDDLStateFactory::getPropositions(
// 			state, objectSymbolTable);
// 	std::list<PNE> pnes = getPNEs(state, objectSymbolTable);
// 	std::list<TIL> tils = getTILs(state, timestamp, objectSymbolTable);
// 	std::list<PendingAction> pendingActions = getPendingActions(state,
// 			timestamp, objectSymbolTable);
// 	addRequiredPropositionsForPendingActions(pendingActions, propositions);
// 	std::list<string> planPrefix = getPlanPrefix(plan);
// 	return PDDLState(objectSymbolTable, propositions, pnes, tils,
// 			pendingActions, goals, metric, planPrefix, heuristic, timestamp);
// }

PDDLState PDDLStateFactory::getDeTILedPDDLState(
		const Planner::MinimalState & state,
		const std::list<Planner::FFEvent>& plan, double timestamp,
		double heuristic, const std::list<PDDL::Proposition> & tilPredicates,
		const std::list<PDDL::Proposition> & tilRequiredObjects,
		const std::list<PDDL::Proposition> & pendingActionRequiredObjects,
		const std::set<PDDLObject> & domainObjectSymbolTable) {

	std::set<PDDLObject> objectSymbolTable;

	//Insert objects found from reading static data
	objectSymbolTable.insert(this->objectParameterTable.begin(),
			this->objectParameterTable.end());
	//Insert objects found from TILs
	objectSymbolTable.insert(domainObjectSymbolTable.begin(),
			domainObjectSymbolTable.end());

	std::list<Proposition> propositions = PDDLStateFactory::getPropositions(
			state, objectSymbolTable);
	std::list<PNE> pnes = getPNEs(state, objectSymbolTable);

	addRequiredPropositionsForPendingActions(pendingActionRequiredObjects,
			propositions);
	addTILPropositions(tilRequiredObjects, propositions);

	std::list<string> planPrefix = PDDL::getPlanPrefix(plan);
	PDDLState theState(objectSymbolTable, propositions, tilPredicates, pnes,
			goals, metric, planPrefix, heuristic, timestamp);

	return theState;
}

PDDL::Metric PDDLStateFactory::getMetric() {
	Planner::RPGBuilder::Metric * metric = Planner::RPGBuilder::getMetric();
	list<std::string> variables;
	list<int>::const_iterator varItr = metric->variables.begin();
	for (; varItr != metric->variables.end(); varItr++) {
		//Colin seems to fill PNE index lists with negative values
		//if the PNE is total-time, so define explicitly
		std::string var = "(" + DEFAULT_METRIC_PNE + ")";
		if ((*varItr >= 0) && (*varItr < Planner::RPGBuilder::getPNECount())) {
			Inst::PNE* aPNE = Planner::RPGBuilder::getPNE(*varItr);
			string name = aPNE->getHead()->getName();
			transform(name.begin(), name.end(), name.begin(), ::toupper);
			var = "(" + name + ")";
		}
		variables.push_back(var);
	}
	PDDL::Metric aMetric(metric->minimise, variables);
	return aMetric;
}

/**
 * Gets the propositional Goals.
 * N.B. We say propositional because we assume they are positive.
 * FIXME Add support for negative propositonal goals
 */
std::list<Proposition> PDDLStateFactory::getPropositionalGoals(
		std::set<PDDLObject> & objectSymbolTable) {
	std::list<Proposition> goals;
	std::list<Inst::Literal*>::const_iterator goalItr =
			Planner::RPGBuilder::getLiteralGoals().begin();
	const std::list<Inst::Literal*>::const_iterator goalItrEnd =
			Planner::RPGBuilder::getLiteralGoals().end();
	for (; goalItr != goalItrEnd; goalItr++) {
		Inst::Literal* aLiteral = (*goalItr);
		Proposition prop = PropositionFactory::getInstance()->getProposition(
				aLiteral->toProposition());
		PDDL::extractParameters(aLiteral, objectSymbolTable, constants);
		goals.push_back(prop);
	}
	return goals;
}

/**
 * Add propositions which ensure correct objects are used for pending actions
 */
void PDDLStateFactory::addRequiredPropositionsForPendingActions(
		const std::list<PDDL::Proposition> & pendingActionRequiredObjects,
		std::list<Proposition> & propositions) {
	propositions.insert(propositions.end(),
			pendingActionRequiredObjects.begin(),
			pendingActionRequiredObjects.end());
}

std::list<PDDL::Proposition> PDDLStateFactory::getPropositions(
		const Planner::MinimalState & state,
		std::set<PDDLObject> & objectSymbolTable) {
	std::list<PDDL::Proposition> literals;
	//Cycle through State Literal Facts
	const Planner::StateFacts & stateFacts = state.first;
	Planner::StateFacts::const_iterator cfItr = stateFacts.begin();
	const Planner::StateFacts::const_iterator cfEnd = stateFacts.end();
	for (; cfItr != cfEnd; cfItr++) {
		int literalID = *cfItr;
		Inst::Literal * aliteral = Planner::RPGBuilder::getLiteral(literalID);
		const VAL::proposition * realCond = aliteral->toProposition();
		PDDL::extractParameters(aliteral, objectSymbolTable, constants);
		PDDL::Proposition literal =
				PropositionFactory::getInstance()->getProposition(realCond);
		literals.push_back(literal);
	}
	//Insert static propositions
	literals.insert(literals.end(), staticPropositions.begin(),
			staticPropositions.end());

	return literals;
}

/**
 * FIXME: Does not generate parameter table for propositions created from actual PDDL Parse Tree
 */
std::list<PDDL::Proposition> PDDLStateFactory::getStaticPropositions(
		std::list<PDDL::Proposition> & dynamicLiterals,
		std::set<PDDLObject> & objectSymbolTable) {
	std::list<PDDL::Proposition> staticLiterals;

	//Look for static literals, these are usually new and unique, no need for checks
	std::vector<pair<bool, bool> > modelledStaticLiterals =
			Planner::RPGBuilder::getStaticLiterals();
	std::vector<pair<bool, bool> >::const_iterator slItr =
			modelledStaticLiterals.begin();
	const std::vector<pair<bool, bool> >::const_iterator slItrEnd =
			modelledStaticLiterals.end();
	int i = 0;
	for (; slItr != slItrEnd; slItr++) {
		Inst::Literal * aliteral = Planner::RPGBuilder::getLiteral(i++);
		const VAL::proposition * realCond = aliteral->toProposition();
		if (slItr->first && slItr->second) {
			PDDL::extractParameters(aliteral, objectSymbolTable, constants);
			PDDL::Proposition literal =
					PropositionFactory::getInstance()->getProposition(realCond);
			staticLiterals.push_back(literal);
		}
	}

	/*Look for all other literals, these are ones missed by all
	 * other checks and requires going back to the original
	 * state description. These will require duplications checks.*/

	VAL::pc_list<VAL::simple_effect*> & props =
			VAL::current_analysis->the_problem->initial_state->add_effects;
	VAL::pc_list<VAL::simple_effect*>::const_iterator propItr = props.begin();
	for (; propItr != props.end(); propItr++) {
		VAL::simple_effect* prop = *propItr;
		PDDL::Proposition prop2 =
				PropositionFactory::getInstance()->getProposition(prop->prop);

		if ((std::find(dynamicLiterals.begin(), dynamicLiterals.end(), prop2)
				== dynamicLiterals.end())
				&& (std::find(staticLiterals.begin(), staticLiterals.end(),
						prop2) == staticLiterals.end())) {

			PDDL::extractParameters(prop, objectSymbolTable, constants);
			staticLiterals.push_back(prop2);
		}
	}
	return staticLiterals;
}

std::list<PDDL::PNE> PDDLStateFactory::getPNEs(
		const Planner::MinimalState & state,
		std::set<PDDLObject> & objectSymbolTable) {
	std::list<PDDL::PNE> pnes;
	//Cycle through PNEs
	const int pneCount = state.secondMin.size();
	for (int i = 0; i < pneCount; i++) {
		Inst::PNE* aPNE = Planner::RPGBuilder::getPNE(i);

		PDDL::extractParameters(aPNE, objectSymbolTable, constants);
		PDDL::PNE pne = PNEFactory::getInstance()->getPNE(aPNE,
				state.secondMin[i]);
		pnes.push_back(pne);
	}

	pnes.insert(pnes.end(), staticPNEs.begin(), staticPNEs.end());
	return pnes;
}

/**
 * FIXME: Does not generate parameter table for propositions created from actual PDDL Parse Tree
 */
std::list<PDDL::PNE> PDDLStateFactory::getStaticPNEs(
		std::list<PDDL::PNE> dynamicPNEs,
		std::set<PDDLObject> & objectSymbolTable) {
	std::list<PDDL::PNE> staticPNEs;
	/*Look for all other literals, these are ones missed by all
	 * other checks and requires going back to the original
	 * state description. These will require duplications checks.*/

	VAL::pc_list<VAL::assignment*> & pnes =
			VAL::current_analysis->the_problem->initial_state->assign_effects;
	VAL::pc_list<VAL::assignment*>::const_iterator pneItr = pnes.begin();
	for (; pneItr != pnes.end(); pneItr++) {
		VAL::assignment* pne = *pneItr;
		double value = ((VAL::num_expression*) pne->getExpr())->double_value();
		PDDL::PNE pne2 = PNEFactory::getInstance()->getPNE(pne->getFTerm(),
				value);
		//Add iif it doesn't already exists
		if ((std::find(dynamicPNEs.begin(), dynamicPNEs.end(), pne2)
				== dynamicPNEs.end())
				&& (std::find(staticPNEs.begin(), staticPNEs.end(), pne2)
						== staticPNEs.end())) {
			PDDL::extractParameters(pne->getFTerm()->getArgs(), 
					objectSymbolTable, constants);
			staticPNEs.push_back(pne2);
		}
	}
	return staticPNEs;

}

/**
 * Cycles through each TIL, and then creates the
 * required propositions
 */
void PDDLStateFactory::addTILPropositions(
		const std::list<PDDL::Proposition> & requiredObjects,
		std::list<Proposition> & propositions) {
	//TIL Object Required Predicates
	propositions.insert(propositions.end(), requiredObjects.begin(),
			requiredObjects.end());
}

}
