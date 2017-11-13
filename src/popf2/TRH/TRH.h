/*
 * TRH.h
 *
 *  Created on: 26 Jan 2016
 *      Author: tony
 */

#ifndef __TRH_TRH
#define __TRH_TRH

#include <list>

#include "../pddl/PDDLDomain.h"
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
	static const string H_PLAN_DELIM;
	static const string TEMP_STATE_PATH;
	static TRH * INSTANCE;

	/*Used to ensure unique state files per instance*/
	const int trhInstanceID;
	const string hCommand;
	string stateFileName;

	static int generateNewInstanceID();
	//Singleton
	TRH();
	TRH(TRH const & other):trhInstanceID(other.trhInstanceID), 
										hCommand(other.hCommand), 
										stateFileName(other.stateFileName){

	}
	;
	TRH operator=(TRH const& other) {
		return TRH(other);
	}
	;

	string runPlanner();
	string buildCommand();

	void addRelaxedPlan(list<Planner::FFEvent> & proposedPlan, 
			list<Planner::FFEvent> & relaxedPlan);
	void writeBadState(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory, int stateNum);
	pair<PDDL::PDDLDomain, PDDL::PDDLState> writeStateToFile(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory, const string & filename);
	void removeTempState(string fileName);

	std::pair<Planner::MinimalState, list<Planner::FFEvent> > reprocessPlan(list<Planner::FFEvent> & oldSoln);
	static bool evaluateStateAndUpdatePlan(auto_ptr<Planner::SearchQueueItem> & succ,
		const Planner::ActionSegment & actionToBeApplied,
		Planner::ExtendedMinimalState & state, 
		Planner::ExtendedMinimalState * prevState,
		Planner::ParentData * const incrementalData,
		std::list<Planner::FFEvent> & header);

public:
	static TRH * getInstance();
	static double TIME_SPENT_IN_HEURISTIC;
	static double TIME_SPENT_IN_PRINTING_TO_FILE;
	static double TIME_SPENT_CONVERTING_PDDL_STATE;
	
	pair<double, int> getHeuristic(Planner::ExtendedMinimalState & theState,
		std::list<Planner::FFEvent>& plan, std::list<Planner::FFEvent> & now,
		double timestamp, double heuristic, list<Planner::ActionSegment> & helpfulActions,
		PDDL::PDDLStateFactory pddlFactory);
};
}

#endif /* __TRH_TRH */
