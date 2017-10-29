#include <stdlib.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <sstream>
#include <limits>
#include <random>

#include "../util/Util.h"
#include "TRH.h"
#include "PDDLDomain.h"
#include "PDDLState.h"
#include "PDDLDomainFactory.h"
#include "PendingAction.h"
#include "Proposition.h"
#include "PDDLObject.h"
#include "PDDLUtils.h"
#include "../FFSolver.h"
#include "instantiation.h"
#include "../temporalanalysis.h"

namespace TRH {

TRH * TRH::INSTANCE = NULL;
const char * TRH::H_CMD = "./lib/colin-clp";
const string TRH::TEMP_FILE_PATH = "/tmp/";
const string TRH::TEMP_FILE_PREFIX = "temp";
const string TRH::TEMP_DOMAIN_SUFFIX = "-domain";
const string TRH::TEMP_FILE_EXT = ".pddl";
const string TRH::H_VAL_DELIM = "State Heuristic Value is: ";
const string TRH::RELAXED_PLAN_SIZE_DELIM = "Relaxed plan length is: ";
const string TRH::H_STATES_EVAL_DELIM = "; States evaluated: ";
double TRH::TIME_SPENT_IN_HEURISTIC = 0.0;
double TRH::TIME_SPENT_IN_PRINTING_TO_FILE = 0.0;
double TRH::TIME_SPENT_CONVERTING_PDDL_STATE = 0.0;

const string TRH::H_PLAN_DELIM_START = "=====Plan Start====="; 
const string TRH::H_PLAN_DELIM_STOP = "=====Plan Stop=====";

TRH * TRH::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new TRH(generateNewInstanceID());
	}
	return INSTANCE;
}

int TRH::generateNewInstanceID() {
	static std::random_device rd;
	static std::default_random_engine generator(rd());
	static std::uniform_int_distribution<int> distribution(0,
		std::numeric_limits<int>::max());
	return distribution(generator);
}

pair<double, int> TRH::getHeuristic(Planner::ExtendedMinimalState & theState,
		std::list<Planner::FFEvent>& header, std::list<Planner::FFEvent> & now, 
		double timestamp, double heuristic, list<Planner::ActionSegment> & helpfulActions, 
		PDDL::PDDLStateFactory pddlFactory) {

	const Planner::MinimalState & state = theState.getInnerState();

	Planner::FF::STATES_EVALUATED++;
	string stateName = writeTempState(state, header, timestamp, heuristic, pddlFactory);

	clock_t begin_time = clock();

	std::shared_ptr<FILE> pipe(popen(buildCommand().c_str(), "r"), pclose);
	if (!pipe)
		return std::make_pair(-1.0, -1.0);
	char buffer[128];
	std::string result = "";
	while (!feof(pipe.get())) {
		if (fgets(buffer, 128, pipe.get()) != NULL)
			// cout << buffer;
			result += buffer;
	}

	TRH::TRH::TIME_SPENT_IN_HEURISTIC += double( clock () - begin_time ) /  CLOCKS_PER_SEC;
	removeTempState(stateName);
	int pos = result.find(H_STATES_EVAL_DELIM);
	if (pos != -1) {
		int posEnd = result.find("\n", pos);
		string statesEvalStr = result.substr(pos + H_STATES_EVAL_DELIM.size(), posEnd-(pos + H_STATES_EVAL_DELIM.size()));
		int statesEval = stoi(statesEvalStr);
		// printf("Heuristic States Eval: %s\n", statesEvalStr.c_str());
		Planner::FF::STATES_EVALUATED_IN_HEURISTIC += statesEval;
	}
	pos = result.find(H_VAL_DELIM);
	if (pos == -1) {
		// cerr << "Problem was unsolvable - therefore heuristic value of -1.0" << endl;
		static int badStateNum = 0;
		//writeBadState(state, header, timestamp, heuristic, pddlFactory, badStateNum++);
		return std::make_pair(-1.0, -1.0);
	}
	string h_val_str = result.substr(pos + H_VAL_DELIM.size());
	// printf("%s\n", h_val_str.c_str());
	double hval = stod(h_val_str);
	
	int relaxedPlanSize = 0;
	pos = result.find(RELAXED_PLAN_SIZE_DELIM);
	if (pos != -1) {
		int posEnd = result.find("\n", pos);
		string relaxedPlanSizeStr = result.substr(pos + RELAXED_PLAN_SIZE_DELIM.size(), 
			posEnd-(pos + RELAXED_PLAN_SIZE_DELIM.size()));
		relaxedPlanSize = stoi(relaxedPlanSizeStr);
		// printf("Relaxed Plan Length: %s\n", relaxedPlanSizeStr.c_str());
	}
	if (hval == 0.0) {
		list<string> relaxedPlanStr = getRelaxedPlanStr(result);
		list<Planner::FFEvent> proposedPlan(getActions(header));

		list<Planner::FFEvent> nowList = getActions(now);
		proposedPlan.insert(proposedPlan.end(), nowList.begin(), nowList.end());

		list<Planner::FFEvent> relaxedPlan = getRelaxedEventList(relaxedPlanStr, timestamp);
		proposedPlan.insert(proposedPlan.end(), relaxedPlan.begin(), relaxedPlan.end());

		std::pair<Planner::MinimalState,
			list<Planner::FFEvent> > solution = reprocessPlan(proposedPlan);
		Planner::FF::workingBestSolution.update(solution.second, solution.first.temporalConstraints, 
			Planner::FF::evaluateMetric(solution.first, list<Planner::FFEvent>(), false));
	} else if (Planner::FF::helpfulActions) {
		list<string> relaxedPlanStr = getRelaxedPlanStr(result);
		list<Planner::ActionSegment> relaxedPlanActions = getRelaxedPlan(relaxedPlanStr);
		helpfulActions.insert(helpfulActions.end(), relaxedPlanActions.begin(), 
			relaxedPlanActions.end());
	}
	return std::make_pair (hval, relaxedPlanSize);
}

string TRH::buildCommand() {
	ostringstream cmd;
	cmd << H_CMD << " " 
		<< TEMP_FILE_PATH << TEMP_FILE_PREFIX << TRH_INSTANCE_ID 
			<< TEMP_DOMAIN_SUFFIX << TEMP_FILE_EXT
		<< " " << TEMP_FILE_PATH << TEMP_FILE_PREFIX << TRH_INSTANCE_ID 
			<< TEMP_FILE_EXT;
	return cmd.str();
}

/**
 * Uses regex to extract all plan actions from the output.
 * Has been commented out as regex support was only added officially
 * in GCC 4.9. However, GCC 4.9 is incompatible with COIN-OR (mem fault)
 */
//DEPRECATED: Until COIN-OR is compatible with GCC-4.9
// list<string> TRH::getRelaxedPlanStr(const string & planStr) {
// 	regex plan_reg("[[:digit:]]+\\.?[[:digit:]]*: \\([[:alnum:][:s:]\\-_]+\\)  \\[[[:digit:]]+\\.?[[:digit:]]*\\]");
// 	smatch match;
// 	list<string> relaxedPlanStr;
// 	string testStr (planStr);
// 	while (regex_search(testStr, match, plan_reg)) {
// 		for (auto elm:match){
// 			relaxedPlanStr.push_back(elm);
// 			testStr = match.suffix().str();
// 		}
// 	}
// 	return relaxedPlanStr;
// }

/**
 *
 * Runs through a list of FFEvent objects and returns
 * a list of the same objects without any TILs
 *
 * Used to remove TILs from a prefix for reprocessing
 *
 */
list<Planner::FFEvent> TRH::getActions(list<Planner::FFEvent> & actionList) {
	list<Planner::FFEvent> header;
	list<Planner::FFEvent>::const_iterator actionItr = actionList.begin();
	for (; actionItr != actionList.end(); actionItr++) {
		if (actionItr->time_spec != VAL::time_spec::E_AT) {
			header.push_back(*actionItr);
		}
	}
	return header;
}

list<string> TRH::getRelaxedPlanStr(const string & output) {
	list<string> relaxedPlanStr;

	int startPos = output.find(H_PLAN_DELIM_START);
	int endPos = output.find(H_PLAN_DELIM_STOP);
	string planSection = output.substr(startPos + H_PLAN_DELIM_START.size(), 
		endPos-(startPos + H_PLAN_DELIM_START.size()));
	// cout << "Relaxed Plan" << endl;
	// cout << planSection << endl;
	//Iterate through actions
	std::istringstream iss(planSection);
	for (std::string line; std::getline(iss, line); ){
		//Guaranteed to be a double if this is an action
		string firstFive = line.substr(0, 5);
		if (Util::isDouble(firstFive.c_str())) {
			// cout << line << endl;
			relaxedPlanStr.push_back(line);
		}
	}
	return relaxedPlanStr;
}

map<double, Planner::ActionSegment> TRH::getRelaxedPlan(list<string> planStr, 
	double timestamp) {
	map<double, Planner::ActionSegment> rPlan;

	list<string>::const_iterator planStrItr = planStr.begin();
	for (; planStrItr != planStr.end(); planStrItr++) {
		string actionStr = *planStrItr;
		int actionNameStartPos = actionStr.find("(");
		int actionNameEndPos = actionStr.find(")") + 1;
		string actionInstance = actionStr.substr(actionNameStartPos, 
			actionNameEndPos - actionNameStartPos);
		string actionName = actionInstance.substr(1, actionInstance.find(" ") - 1);
		Inst::instantiatedOp * op = PDDL::getOperator(actionInstance);
		
		int startTimePos = actionStr.find(":");
		double startTime = stod(actionStr.substr(0, startTimePos));
		//Add the timestamp to the action to fit after the prefix
		startTime += timestamp;

		int actionDurationStartPos = actionStr.find("[") + 1;
		int actionDurationEndPos = actionStr.find("]");
		double duration = stod(actionStr.substr(actionDurationStartPos, 
			actionDurationEndPos - actionDurationStartPos));

		if (op != 0) {
			Planner::ActionSegment start_snap_action = Planner::ActionSegment(op, 
				VAL::E_AT_START, 0, Planner::RPGHeuristic::emptyIntList);
			rPlan.insert(pair<double, Planner::ActionSegment>(startTime, start_snap_action));
			if (!Planner::RPGBuilder::getRPGDEs(op->getID()).empty()) {
				//Durative Action
				Planner::ActionSegment end_snap_action = Planner::ActionSegment(op, 
					VAL::E_AT_END, 0, Planner::RPGHeuristic::emptyIntList);
				rPlan.insert(pair<double, Planner::ActionSegment>(startTime + duration, end_snap_action));
			}
		}
	}
	return rPlan;
}

list<Planner::ActionSegment> TRH::getRelaxedPlan(list<string> planStr) {
	list<Planner::ActionSegment> rPlan;
	// cout << "Helpful Actions" << endl;
	list<string>::const_iterator planStrItr = planStr.begin();
	for (; planStrItr != planStr.end(); planStrItr++) {
		string actionStr = *planStrItr;
		// cout << actionStr << endl;
		int actionNameStartPos = actionStr.find("(");
		int actionNameEndPos = actionStr.find(")") + 1;
		string actionInstance = actionStr.substr(actionNameStartPos, 
			actionNameEndPos - actionNameStartPos);
		Inst::instantiatedOp * op = PDDL::getOperator(actionInstance);

		if (op != 0) {
			Planner::ActionSegment start_snap_action = Planner::ActionSegment(op, 
				VAL::E_AT_START, 0, Planner::RPGHeuristic::emptyIntList);
			rPlan.push_back(start_snap_action);
			if (!Planner::RPGBuilder::getRPGDEs(op->getID()).empty()) {
				//Durative Action
				Planner::ActionSegment end_snap_action = Planner::ActionSegment(op, 
					VAL::E_AT_END, 0, Planner::RPGHeuristic::emptyIntList);
				rPlan.push_back(end_snap_action);
			}
		}
	}
	return rPlan;
}

list<Planner::FFEvent> TRH::getRelaxedEventList(list<string> planStr, 
	double timestamp) {
	list<Planner::FFEvent> rPlan;
	
	list<string>::const_iterator planStrItr = planStr.begin();
	for (; planStrItr != planStr.end(); planStrItr++) {
		string actionStr = *planStrItr;
		int actionNameStartPos = actionStr.find("(");
		int actionNameEndPos = actionStr.find(")") + 1;
		string actionInstance = actionStr.substr(actionNameStartPos, 
			actionNameEndPos - actionNameStartPos);
		string actionName = actionInstance.substr(1, actionInstance.find(" ") - 1);
		Inst::instantiatedOp * op = PDDL::getOperator(actionInstance);
		
		int startTimePos = actionStr.find(":");
		double startTime = stod(actionStr.substr(0, startTimePos));
		//Add the timestamp to the action to fit after the prefix
		startTime += timestamp;

		int actionDurationStartPos = actionStr.find("[") + 1;
		int actionDurationEndPos = actionStr.find("]");
		double duration = stod(actionStr.substr(actionDurationStartPos, 
			actionDurationEndPos - actionDurationStartPos));

		if (op != 0) {
			Planner::FFEvent start_snap_action = Planner::FFEvent(op, 
				startTime, startTime);
			start_snap_action.lpTimestamp = startTime;
			rPlan.push_back(start_snap_action);
			if (!Planner::RPGBuilder::getRPGDEs(op->getID()).empty()) {
				//Durative Action
				double durActEndStart = startTime + duration;
				Planner::FFEvent end_snap_action = Planner::FFEvent(op, 
				rPlan.size() - 1, durActEndStart, durActEndStart);
				end_snap_action.lpTimestamp = durActEndStart;
				rPlan.push_back(end_snap_action);
			}
		}
	}
	return rPlan;
}

Planner::SearchQueueItem * TRH::applyTILsIfRequired(Planner::SearchQueueItem * currSQI, 
		double timestamp) {

	int numTILs = Planner::RPGBuilder::timedInitialLiteralsVector.size();
	
	for (int i = currSQI->state()->getInnerState().nextTIL; i < numTILs; i++) {

		Planner::RPGBuilder::FakeTILAction * til = Planner::RPGBuilder::timedInitialLiteralsVector[i];
		if (til->duration <= timestamp) {
			//It is time for this til update
			int oldTIL = currSQI->state()->getEditableInnerState().nextTIL;
			Planner::ActionSegment tempSeg(0, VAL::E_AT, oldTIL, Planner::RPGHeuristic::emptyIntList);
			auto_ptr<Planner::SearchQueueItem> succ = auto_ptr<Planner::SearchQueueItem>(new 
				Planner::SearchQueueItem(Planner::FF::applyActionToState(tempSeg, *(currSQI->state())), true));

			const auto_ptr<Planner::ParentData> incrementalData(
					Planner::FF::allowCompressionSafeScheduler ? 0 : Planner::LPScheduler::prime(currSQI->plan, 
					currSQI->state()->getInnerState().temporalConstraints, currSQI->state()->startEventQueue));

			evaluateStateAndUpdatePlan(succ, tempSeg,  *(succ->state()), currSQI->state(), 
					incrementalData.get(), currSQI->plan);
			currSQI = succ.release();

		} else if (til->duration > timestamp) {
			//It is not yet time for this TIL
			return currSQI;
		}
	}
	return currSQI;
}

std::pair<Planner::MinimalState, list<Planner::FFEvent> > TRH::reprocessPlan(list<Planner::FFEvent> & oldSoln)
{
	Planner::FF::WAStar = false;
	set<int> goals;
	set<int> numericGoals;
	Planner::ExtendedMinimalState initialState;

	{
		Planner::LiteralSet tinitialState;
		vector<double> tinitialFluents;

		Planner::RPGBuilder::getNonStaticInitialState(tinitialState, tinitialFluents);

		initialState.getEditableInnerState().setFacts(tinitialState);
		initialState.getEditableInnerState().setFacts(tinitialFluents);
	}

	PDDL::PDDLStateFactory pddlFactory(initialState.getInnerState(), PDDL::PDDLDomainFactory::getInstance()->getConstants());

	{
		list<Literal*>::iterator gsItr = Planner::RPGBuilder::getLiteralGoals().begin();
		const list<Literal*>::iterator gsEnd = Planner::RPGBuilder::getLiteralGoals().end();

		for (; gsItr != gsEnd; ++gsItr) {
			pair<bool, bool> & currStatic = Planner::RPGBuilder::isStatic(*gsItr);
			if (currStatic.first) {
				assert(currStatic.second);
			} else {
				goals.insert((*gsItr)->getStateID());
			}

		}

	}
	{
		list<pair<int, int> >::iterator gsItr = Planner::RPGBuilder::getNumericRPGGoals().begin();
		const list<pair<int, int> >::iterator gsEnd = Planner::RPGBuilder::getNumericRPGGoals().end();

		for (; gsItr != gsEnd; ++gsItr) {
			if (gsItr->first != -1) {
				numericGoals.insert(gsItr->first);
			}
			if (gsItr->second != -1) {
				numericGoals.insert(gsItr->second);
			}
		}
	}

	list<Planner::FFEvent*> sortedSoln;

	list<Planner::FFEvent>::iterator osItr = oldSoln.begin();
	const list<Planner::FFEvent>::iterator osEnd = oldSoln.end();

	for (; osItr != osEnd; ++osItr) {
		if ((osItr->time_spec == VAL::E_AT_END) && Planner::TemporalAnalysis::canSkipToEnd(osItr->action->getID())) {
			continue;
		}

		list<Planner::FFEvent*>::iterator sortedItr = sortedSoln.begin();
		const list<Planner::FFEvent*>::iterator sortedEnd = sortedSoln.end();

		for (; sortedItr != sortedEnd; ++sortedItr) {
			if (osItr->lpTimestamp < (*sortedItr)->lpTimestamp) {
				sortedSoln.insert(sortedItr, &(*osItr));
				break;
			}
		}
		if (sortedItr == sortedEnd) {
			sortedSoln.push_back(&(*osItr));
		}
	}

	auto_ptr<Planner::StateHash> visitedStates(Planner::FF::getStateHash());

	const list<Planner::FFEvent*>::const_iterator oldSolnEnd = sortedSoln.end();
	// cout << "Beginning the replay" << endl;

	Planner::SearchQueueItem * currSQI = new Planner::SearchQueueItem(&initialState, false);
	{
		list<Planner::FFEvent> tEvent;
		//Update Time
		Planner::LPScheduler tryToSchedule(currSQI->state()->getInnerState(), currSQI->plan, tEvent, -1, 
			currSQI->state()->startEventQueue, 0, currSQI->state()->entriesForAction, 
			0, 0, &(currSQI->state()->tilComesBefore), Planner::FF::scheduleToMetric);
		//Check if it is valid
		if (!tryToSchedule.isSolved()) return std::pair<Planner::MinimalState, list<Planner::FFEvent> >();
	}

	list<Planner::FFEvent*>::const_iterator oldSolnItr = sortedSoln.begin();

	const int lastStep = sortedSoln.size() - 1;

	for (int stepID = 0; oldSolnItr != oldSolnEnd; ++oldSolnItr, ++stepID) {
		Planner::FFEvent * eventToApply = *oldSolnItr;

		currSQI = applyTILsIfRequired(currSQI, eventToApply->lpTimestamp);
		//Update divisionID / nextTIL
		eventToApply->divisionID = currSQI->state()->getInnerState().nextTIL;

		Planner::ActionSegment nextSeg(eventToApply->action, eventToApply->time_spec, 
			eventToApply->divisionID, Planner::RPGHeuristic::emptyIntList);

		if (stepID == lastStep) {
			Planner::FF::scheduleToMetric = true;
		}

		const auto_ptr<Planner::ParentData> incrementalData(Planner::FF::allowCompressionSafeScheduler ? 
				0 : Planner::LPScheduler::prime(currSQI->plan, currSQI->state()->getInnerState().temporalConstraints,
				currSQI->state()->startEventQueue));

		auto_ptr<Planner::SearchQueueItem> succ = auto_ptr<Planner::SearchQueueItem>(new Planner::SearchQueueItem(
				Planner::FF::applyActionToState(nextSeg, *(currSQI->state())), true));
		succ->heuristicValue.makespan = currSQI->heuristicValue.makespan;

		evaluateStateAndUpdatePlan(succ, nextSeg, *(succ->state()), currSQI->state(), incrementalData.get(), currSQI->plan);

		delete currSQI;
		currSQI = succ.release();

	}
	std::pair<Planner::MinimalState, list<Planner::FFEvent> > toReturn(
		currSQI->state()->getInnerState(), currSQI->plan);

	delete currSQI;
	return toReturn;
}

bool TRH::evaluateStateAndUpdatePlan(auto_ptr<Planner::SearchQueueItem> & succ, 
	const Planner::ActionSegment & actionToBeApplied,
	Planner::ExtendedMinimalState & state, 
	Planner::ExtendedMinimalState * prevState,
	Planner::ParentData * const incrementalData,
	list<Planner::FFEvent> & header)
{
	Planner::FFEvent extraEvent;
	Planner::FFEvent extraEventTwo;

	Planner::FFEvent * reusedEndEvent = 0;

	bool eventOneDefined = false;
	bool eventTwoDefined = false;

	map<double, list<pair<int, int> > > * justApplied = 0;
	map<double, list<pair<int, int> > > actualJustApplied;
	double tilFrom = 0.001;

	succ->plan = header;

	int stepID = -1;

	if (actionToBeApplied.second == VAL::E_AT_START) {
		extraEvent = Planner::FFEvent(actionToBeApplied.first, state.startEventQueue.back().minDuration, state.startEventQueue.back().maxDuration);
		eventOneDefined = true;

		assert(extraEvent.time_spec == VAL::E_AT_START);
		Planner::FF::makeJustApplied(actualJustApplied, tilFrom, state, true);
		if (!actualJustApplied.empty()) justApplied = &actualJustApplied;

		if (!Planner::RPGBuilder::getRPGDEs(actionToBeApplied.first->getID()).empty()) { // if it's not a non-temporal action

			const int endStepID = state.getInnerState().planLength - 1;
			const int startStepID = endStepID - 1;
			extraEventTwo = Planner::FFEvent(actionToBeApplied.first, startStepID, state.startEventQueue.back().minDuration, state.startEventQueue.back().maxDuration);
			extraEvent.pairWithStep = endStepID;
			eventTwoDefined = true;

			if (!Planner::TemporalAnalysis::canSkipToEnd(actionToBeApplied.first->getID())) {
				extraEventTwo.getEffects = false;
			}

			stepID = startStepID;
		} else {
			stepID = state.getInnerState().planLength - 1;
		}
	} else if (actionToBeApplied.second == VAL::E_AT_END) {
		map<int, list<list<Planner::StartEvent>::iterator > >::iterator tsiOld = state.entriesForAction.find(actionToBeApplied.first->getID());
		assert(tsiOld != state.entriesForAction.end());

		list<Planner::StartEvent>::iterator pairWith = tsiOld->second.front();
		tsiOld->second.pop_front();
		if (tsiOld->second.empty()) state.entriesForAction.erase(tsiOld);

		stepID = pairWith->stepID + 1;

		{
			list<Planner::FFEvent>::iterator pwItr = succ->plan.begin();
			for (int sID = 0; sID <= pairWith->stepID; ++sID, ++pwItr) ;
			pwItr->getEffects = true;
			reusedEndEvent = &(*pwItr);
		}

		state.startEventQueue.erase(pairWith);

		Planner::FF::makeJustApplied(actualJustApplied, tilFrom, state, false);
		if (!actualJustApplied.empty()) justApplied = &actualJustApplied;
	} else {
		extraEvent = Planner::FFEvent(actionToBeApplied.divisionID);
		eventOneDefined = true;
		stepID = state.getInnerState().planLength - 1;
	}


	list<Planner::FFEvent> nowList;

	if (eventOneDefined) nowList.push_back(extraEvent);
	if (eventTwoDefined) nowList.push_back(extraEventTwo);

	assert(stepID != -1);

	//Update Time
	Planner::LPScheduler * tryToSchedule = new Planner::LPScheduler(state.getInnerState(), succ->plan, nowList, stepID, 
		state.startEventQueue, incrementalData, state.entriesForAction, 
		(prevState ? &prevState->getInnerState().secondMin : 0), 
		(prevState ? &prevState->getInnerState().secondMax : 0), 
		&(state.tilComesBefore), Planner::FF::scheduleToMetric);

	//Check if it is valid
	if (!tryToSchedule->isSolved()) return false;

	delete tryToSchedule;

	if (eventTwoDefined) {
		extraEventTwo = nowList.back();
		nowList.pop_back();
	}

	if (eventOneDefined) {
		extraEvent = nowList.back();
	}

	if (eventOneDefined) {
		succ->plan.push_back(extraEvent);
	}

	if (eventTwoDefined) {
		succ->plan.push_back(extraEventTwo);
	}

	if (actionToBeApplied.second == VAL::E_AT_START // If it's the start of an action...
			&& !Planner::RPGBuilder::getRPGDEs(actionToBeApplied.first->getID()).empty() // that is temporal..
			&& Planner::TemporalAnalysis::canSkipToEnd(actionToBeApplied.first->getID())) { // and compression-safe

	   // we can pull it off the start event queue as we don't need to worry about applying it

		state.startEventQueue.pop_back();
	}
	return true;
}

string TRH::writeTempState(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory) {
	// static int oneShot = 0;
	// if (oneShot++ > 1) {
	// 	cerr << "Exiting..." << endl;
	// 	exit(0);
	// }

	ostringstream stateFileName;
	stateFileName << TEMP_FILE_PREFIX << TRH_INSTANCE_ID;
	writeStateToFile(state, plan, timestamp, heuristic, 
		pddlFactory, stateFileName.str());
	return stateFileName.str();
}

void TRH::writeBadState(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory, int stateNum) {
	
	ostringstream stateFileName;
	stateFileName << "BadState" << TRH_INSTANCE_ID << "-" << stateNum;
	writeStateToFile(state, plan, timestamp, heuristic, 
		pddlFactory, stateFileName.str());
}


void TRH::writeStateToFile(const Planner::MinimalState & state,
	std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
	PDDL::PDDLStateFactory pddlFactory, string fileName) {

	clock_t begin_time = clock();

	/*Generate Domain*/

	//Domain
	PDDL::PDDLDomain domain = PDDL::PDDLDomainFactory::getInstance()->getDeTILedDomain(
			VAL::current_analysis->the_domain, state, timestamp);

	//Shared data
	std::list<PDDL::Proposition> tilRequiredObjects = domain.getTILRequiredObjects();
	std::list<PDDL::Proposition> tilPredicates = domain.getTILPredicates();
	std::list<PDDL::Proposition> tilGoalPredicates = domain.getTILGoalPredicates();
	std::list<PDDL::Proposition> pendingActionRequiredObjects = domain.getPendingActionRequiredObjects();
	std::set<PDDL::PDDLObject> domainObjectSymbolTable = domain.getDomainObjectSymbolTable();

	//State
	PDDL::PDDLState pddlState = pddlFactory.getDeTILedPDDLState(state, plan, timestamp, 
			heuristic, tilPredicates, tilGoalPredicates, tilRequiredObjects, 
			pendingActionRequiredObjects, domainObjectSymbolTable);
				
	TRH::TRH::TIME_SPENT_CONVERTING_PDDL_STATE += double( clock () - begin_time ) /  CLOCKS_PER_SEC;
		
	//Write State/Domain to disk for heuristic computation
	begin_time = clock();
	
	string domainFileName = fileName + TEMP_DOMAIN_SUFFIX;
	pddlState.writeDeTILedStateToFile(TEMP_FILE_PATH, fileName);
	domain.writeToFile(TEMP_FILE_PATH, domainFileName);
	TRH::TRH::TIME_SPENT_IN_PRINTING_TO_FILE += double( clock () - begin_time ) /  CLOCKS_PER_SEC;
}

void TRH::removeTempState(string fileName) {
	ostringstream stateFileName;
	ostringstream domainFileName;

	domainFileName << TEMP_FILE_PATH << fileName
		<< TEMP_DOMAIN_SUFFIX << TEMP_FILE_EXT;
	stateFileName << TEMP_FILE_PATH << fileName << TEMP_FILE_EXT;

	//Remove Domain File
	remove(domainFileName.str().c_str());
	//Remove State File
	remove(stateFileName.str().c_str());
}

}