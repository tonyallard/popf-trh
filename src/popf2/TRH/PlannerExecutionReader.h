/*
 * TRH.h
 *
 *  Created on: 11 Nov 2017
 *      Author: tony
 */

#ifndef __TRH_PlannerExecutionReader
#define __TRH_PlannerExecutionReader

#include <list>
#include <string>

#include "../pddl/PDDLState.h"
#include "../pddl/PDDLStateFactory.h"
#include "../ExtendedMinimalState.h"
#include "../FFEvent.h"
#include "../util/Util.h"
#include "../lpscheduler.h"

using namespace std;

namespace TRH {
class PlannerExecutionReader {
private:
	static const string H_VAL_DELIM;
	static const string RELAXED_PLAN_SIZE_DELIM;
	static const string H_STATES_EVAL_DELIM;
	static const string H_PLAN_DELIM;
	static const string SOLUTION_FOUND;

	static const string H_PLAN_DELIM_START; 
	static const string H_PLAN_DELIM_STOP;

	int statesEvaluatedInHeuristic;
	int relaxedPlanSize;
	list<Planner::FFEvent> relaxedPlan;
	list<Planner::ActionSegment> helpfulActions;
	bool solutionFound;

	int getHeuristicStatesEvaluated(const string & plannerOutput);
	int getRelaxedPLanLength(const string & plannerOutput);
	list<string> getRelaxedPlanStr(const string & output);
	list<Planner::ActionSegment> getHelpfulActions(list<string> planStr);
	list<Planner::FFEvent> getRelaxedPlan(list<string> planStr, 
		std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> tilMap);
	bool getIsSolutionFound(const string & plannerOutput);
public:
	PlannerExecutionReader(string plannerOutput, 
		std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> tilMap);

	inline int getHeuristicStatesEvaluated() {
		return statesEvaluatedInHeuristic;
	}

	inline int getRelaxedPLanLength() {
		return relaxedPlanSize;
	}

	inline list<Planner::ActionSegment> & getHelpfulActions() {
		return helpfulActions;
	}

	inline list<Planner::FFEvent> & getRelaxedPlan () {
		return relaxedPlan;
	}

	inline bool isSolutionFound() {
		return solutionFound;
	}
};
}

#endif /* __TRH_PlannerExecutionReader */
