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

struct BacklogItem {
       
	double startTime;
	Planner::FFEvent * start;
	Planner::FFEvent end;
	
	BacklogItem(Planner::FFEvent * start, Planner::FFEvent & end, double startTime) :
	start(start), end(end), startTime(startTime){

	}

	friend bool operator<(const BacklogItem& l, const BacklogItem& r)
	{
		return std::tie(l.startTime)
			< std::tie(r.startTime); // keep the same order
	}
};

class PlannerExecutionReader {
private:
	static const string H_VAL_DELIM;
	static const string RELAXED_PLAN_SIZE_DELIM;
	static const string H_STATES_EVAL_DELIM;
	static const string H_DEAD_ENDS_DELIM;
	static const string H_PLAN_DELIM;
	static const string SOLUTION_FOUND;

	static const string H_PLAN_DELIM_START; 
	static const string H_PLAN_DELIM_STOP;

	int statesEvaluatedInHeuristic;
	int deadEndsEncounteredInHeuristic;
	int relaxedPlanLength;
	list<Planner::FFEvent> relaxedPlan;
	list<Planner::ActionSegment> helpfulActions;
	bool solutionFound;

	int getHeuristicStatesEvaluated(const string & plannerOutput);
	int getDeadEndsEncountered(const string & plannerOutput);
	list<string> getRelaxedPlanStr(const string & output);
	list<Planner::ActionSegment> getHelpfulActions(
		const list<Planner::FFEvent> & plan,
		const Planner::MinimalState & state, 
		double timeStamp);
	list<Planner::FFEvent> getRelaxedPlan(list<string> planStr, 
		const std::list<PDDL::TIL> & tils);
	bool getIsSolutionFound(const string & plannerOutput);
public:
	PlannerExecutionReader(string plannerOutput, 
		const std::list<PDDL::TIL> & tils,
		const Planner::MinimalState & state, 
		double timeStamp);

	inline int getHeuristicStatesEvaluated() {
		return statesEvaluatedInHeuristic;
	}

	inline int getDeadEndsEncountered() {
		return deadEndsEncounteredInHeuristic;
	}

	inline int getRelaxedPlanLength() {
		return relaxedPlanLength;
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
