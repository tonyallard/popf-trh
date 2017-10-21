/*
 * PDDLState.h
 *
 *  Created on: 19 Jun 2015
 *      Author: tony
 */

#ifndef COLIN_PDDLSTATE_H_
#define COLIN_PDDLSTATE_H_

#include <string>
#include <list>
#include <set>

#include "Proposition.h"
#include "PNE.h"
#include "TIL.h"
#include "PendingAction.h"
#include "Metric.h"

namespace PDDL {

class PDDLState {

private:
	std::set<PDDLObject> objectSymbolTable;
	std::list<Proposition> literals;
	std::list<PNE> pnes;
	std::list<PDDL::Proposition> tilAchievedPredicates;
	std::list<Proposition> goals;
	PDDL::Metric metric;
	std::list<string> planPrefix;
	double timestamp;
	double heuristic;

	string getPlanPrefixString();

public:
	PDDLState(std::set<PDDLObject> objectSymbolTable,
			std::list<Proposition> literals, std::list<Proposition> tilAchievedPredicates,
			std::list<PNE> pnes, std::list<Proposition> goals,
			Metric metric, std::list<string> planPrefix,
			double heuristic, double timestamp) :
					objectSymbolTable(objectSymbolTable), literals(literals), 
					tilAchievedPredicates(tilAchievedPredicates), pnes(pnes), 
					goals(goals), metric(metric), planPrefix(planPrefix),
					heuristic(heuristic), timestamp(timestamp) {
	}
	;
	std::string toString();
	std::string getObjectSymbolTableString();
	std::string getLiteralString();
	std::string getPNEString();
	std::string getTILGoalString();
	std::string getGoalString();
	inline double getTimestamp() {
		return timestamp;
	}
	void writeStateToFile(std::string filePath, std::string fileName);
	void writeDeTILedStateToFile(std::string filePath, std::string fileName);
};

}

#endif /* COLIN_PDDLSTATE_H_ */
