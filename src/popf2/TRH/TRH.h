/*
 * TRH.h
 *
 *  Created on: 26 Jan 2016
 *      Author: tony
 */

#ifndef __TRH_TRH
#define __TRH_TRH

#include <list>

#include "../pddl/PDDLState.h"
#include "../pddl/PDDLStateFactory.h"
#include "../ExtendedMinimalState.h"
#include "../FFEvent.h"
#include "../util/Util.h"
#include "../lpscheduler.h"

using namespace std;

namespace TRH {
class TRH {
private:
	static const char * H_CMD;
	static const string TEMP_FILE_PATH;
	static const string TEMP_FILE_PREFIX;
	static const string TEMP_DOMAIN_SUFFIX;
	static const string TEMP_FILE_EXT;
	static const string H_VAL_DELIM;
	static const string RELAXED_PLAN_SIZE_DELIM;
	static const string H_STATES_EVAL_DELIM;
	static const string H_PLAN_DELIM;
	static const string TEMP_STATE_PATH;
	static TRH * INSTANCE;

	static const string H_PLAN_DELIM_START; 
	static const string H_PLAN_DELIM_STOP;

	static int generateNewInstanceID();
	
	/*Used to ensure unique state files per instance*/
	const int TRH_INSTANCE_ID;

	//Singleton
	TRH(int trhInstanceID) : TRH_INSTANCE_ID(trhInstanceID) {
	};
	TRH(TRH const & other) : TRH_INSTANCE_ID(generateNewInstanceID()){
	}
	;
	TRH& operator=(TRH const&) {
	}
	;

	string buildCommand();
	string writeTempState(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory);
	void writeBadState(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory, int stateNum);
	void writeStateToFile(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory, string fileName);
	void removeTempState(string fileName);
	list<Planner::FFEvent> getActions(list<Planner::FFEvent> & actionList);
	list<string> getRelaxedPlanStr(const string & output);
	list<Planner::ActionSegment> getRelaxedPlan(list<string> planStr);
	map<double, Planner::ActionSegment> getRelaxedPlan(list<string> planStr, 
		double timestamp);
	list<Planner::FFEvent> getRelaxedEventList(list<string> planStr, 
		double timestamp);
	std::pair<Planner::MinimalState, list<Planner::FFEvent> > reprocessPlan(list<Planner::FFEvent> & oldSoln);
	Planner::SearchQueueItem * applyTILsIfRequired(Planner::SearchQueueItem * currSQI, double timestamp);
	static bool evaluateStateAndUpdatePlan(auto_ptr<Planner::SearchQueueItem> & succ,
		const Planner::ActionSegment & actionToBeApplied,
		Planner::ExtendedMinimalState & state, 
		Planner::ExtendedMinimalState * prevState,
		Planner::ParentData * const incrementalData,
		std::list<Planner::FFEvent> & header);

public:
	static TRH * getInstance();
	pair<double, int> getHeuristic(Planner::ExtendedMinimalState & theState,
		std::list<Planner::FFEvent>& plan, std::list<Planner::FFEvent> & now,
		double timestamp, double heuristic, list<Planner::ActionSegment> & helpfulActions,
		PDDL::PDDLStateFactory pddlFactory);
	static double TIME_SPENT_IN_HEURISTIC;
	static double TIME_SPENT_IN_PRINTING_TO_FILE;
	static double TIME_SPENT_CONVERTING_PDDL_STATE;
};
}

#endif /* __TRH_TRH */
