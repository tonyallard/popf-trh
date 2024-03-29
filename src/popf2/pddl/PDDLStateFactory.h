/*
 * PDDLStateFactory.h
 *
 *  Created on: 2 Jul 2015
 *      Author: tony
 */

#ifndef __PDDLSTATEFACTORY
#define __PDDLSTATEFACTORY

#include <list>
#include <set>

#include "PDDLObject.h"
#include "Proposition.h"
#include "PDDLState.h"
#include "Metric.h"

#include "../minimalstate.h"
#include "../FFEvent.h"

namespace PDDL {

class PDDLStateFactory {
public:

	PDDLStateFactory(const Planner::MinimalState &initialState, std::list<std::pair<std::string, std::string> > constants);
	~PDDLStateFactory() {
		if (metric != PDDL::Metric::NO_METRIC) {
			delete metric;
		}
	};
	PDDLState getDeTILedPDDLState(const Planner::MinimalState & state,
			const std::list<Planner::FFEvent>& plan,
			double timestamp, double heuristic,
			const std::list<PDDL::Proposition> & tilPredicates,
			const std::list<PDDL::Proposition> & tilGoalPredicates,
			const std::list<PDDL::Proposition> & tilRequiredObjects,
			const std::list<PDDL::Proposition> & pendingActionRequiredObjects,
			const std::set<PDDLObject> & tilObjectSymbolTable);

private:
	//CLASS CONSTANTS
	static const std::string DEFAULT_METRIC_PNE;
	//Object vars
	std::list<std::pair<std::string, std::string> > constants;
	std::list<PDDL::Proposition> staticPropositions;
	std::list<PDDL::PNE> staticPNEs;
	std::set<PDDLObject> objectParameterTable;
	std::list<PDDL::Proposition> goals;
	const PDDL::Metric * metric;

	std::list<PDDL::Proposition> getPropositions(
					const Planner::MinimalState & state, std::set<PDDLObject> & objectSymbolTable);
	std::list<PDDL::Proposition> getStaticPropositions(
			std::list<PDDL::Proposition> & dynamicLiterals, std::set<PDDLObject> & objectSymbolTable);
	std::list<PDDL::PNE> getPNEs(const Planner::MinimalState & state, std::set<PDDLObject> & objectSymbolTable);
	std::list<PDDL::PNE> getStaticPNEs(
			std::list<PDDL::PNE> dynamicPNEs,
			std::set<PDDLObject> & objectSymbolTable);
	std::list<PDDL::Proposition> getPropositionalGoals(std::set<PDDLObject> & objectSymbolTable);
	const PDDL::Metric * getMetric();

	void addTILPropositions(
		const std::list<PDDL::Proposition> & requiredObjects,
		std::list<Proposition> & propositions);

	void addRequiredPropositionsForPendingActions(
			const std::list<PDDL::Proposition> & pendingActionRequiredObjects,
			std::list<Proposition> & propositions);

};

}

#endif /* __PDDLSTATEFACTORY */
