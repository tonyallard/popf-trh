#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <memory>
#include <limits>
#include <random>

#include "TRH.h"
#include "../util/Util.h"
#include "PlannerExecutionReader.h"
#include "../hRelax/HRelax.cpp"

#include "PDDLDomainFactory.h"
#include "Proposition.h"
#include "PDDLObject.h"
#include "PDDLUtils.h"

#include "../FFSolver.h"
#include "instantiation.h"
#include "../temporalanalysis.h"

namespace TRH {

TRH * TRH::INSTANCE = NULL;

const char * TRH::H_CMD = "./lib/popf3-clp";
const string TRH::TEMP_FILE_PATH = "";//"/mnt/ramdisk/";//"/tmp/";
const string TRH::TEMP_FILE_PREFIX = "temp";
const string TRH::TEMP_DOMAIN_SUFFIX = "-domain";
const string TRH::TEMP_FILE_EXT = ".pddl";

double TRH::TIME_SPENT_IN_HEURISTIC = 0.0;
double TRH::TIME_SPENT_IN_PRINTING_TO_FILE = 0.0;
double TRH::TIME_SPENT_CONVERTING_PDDL_STATE = 0.0;

TRH * TRH::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new TRH();
	}
	return INSTANCE;
}

TRH::TRH() : 
	trhInstanceID(generateNewInstanceID()),
	hCommand(buildCommand()) {
	ostringstream fileName;
	fileName << TEMP_FILE_PREFIX << trhInstanceID;
	stateFileName = fileName.str();
}

int TRH::generateNewInstanceID() {
	static std::random_device rd;
	static std::default_random_engine generator(rd());
	static std::uniform_int_distribution<int> distribution(0,
		std::numeric_limits<int>::max());
	// return distribution(generator);
	return 11;
}

pair<double, int> TRH::getHeuristic(Planner::ExtendedMinimalState & theState,
		std::list<Planner::FFEvent>& header, std::list<Planner::FFEvent> & now, 
		double timestamp, double heuristic, list<Planner::ActionSegment> & helpfulActions, 
		PDDL::PDDLStateFactory pddlFactory) {

	const Planner::MinimalState & state = theState.getInnerState();

	pair<PDDL::PDDLDomain, PDDL::PDDLState> tempProb = 
		writeStateToFile(state, header, timestamp, heuristic, pddlFactory, stateFileName);

	Planner::FF::STATES_EVALUATED++;
	string result = runPlanner();
	
	//Read in the results of the relaxed plan
	PlannerExecutionReader reader(result, tempProb.first.getTILMap());
	Planner::FF::STATES_EVALUATED_IN_HEURISTIC += reader.getHeuristicStatesEvaluated();
	if (!reader.isSolutionFound()) {
		return std::make_pair (-1.0,  -1);
	}

	//Create the total plan
	list<Planner::FFEvent> proposedPlan(header);
	proposedPlan.insert(proposedPlan.end(), now.begin(), now.end());
	addRelaxedPlan(proposedPlan, reader.getRelaxedPlan());
	//Run relaxation heuristic on relaxed plan
	hRelax::HRelax * relaxationHeuristic = hRelax::HRelax::getInstance();
	pair<double, list<Planner::FFEvent> > hVal = relaxationHeuristic->getHeuristic(proposedPlan);
	// cout << hVal.first << endl;
	// exit(0);
	if (hVal.first == 0.0) {
		// cout << "Executed Plan" << endl;
		// Planner::FFEvent::printPlan(proposedPlan);
		std::pair<Planner::MinimalState, list<Planner::FFEvent> > solution = reprocessPlan(hVal.second);
		Planner::FF::workingBestSolution.update(solution.second, solution.first.temporalConstraints, 
			Planner::FF::evaluateMetric(solution.first, list<Planner::FFEvent>(), false));
	} else {
		helpfulActions.insert(helpfulActions.end(), reader.getHelpfulActions().begin(), 
			reader.getHelpfulActions().end());
	}
	return std::make_pair (hVal.first, reader.getRelaxedPLanLength());
}

void TRH::addRelaxedPlan(list<Planner::FFEvent> & proposedPlan, list<Planner::FFEvent> & relaxedPlan) {
	list<Planner::FFEvent>::iterator rPlanItr = relaxedPlan.begin();
	for (; rPlanItr != relaxedPlan.end(); rPlanItr++) {
		Planner::FFEvent event = *rPlanItr;
		if (event.pairWithStep >= 0) {
			if (event.time_spec == VAL::time_spec::E_AT_START){
				event.pairWithStep = proposedPlan.size() + 1;
			} else if (event.time_spec == VAL::time_spec::E_AT_END) {
				event.pairWithStep = proposedPlan.size() - 1;
			} else {
				std::cerr << "This case not catered for.";
				assert(false);
			}
		}
		proposedPlan.push_back(event);
	}
}

string TRH::runPlanner() {

	clock_t begin_time = clock();

	std::shared_ptr<FILE> pipe(popen(hCommand.c_str(), "r"), pclose);
	if (!pipe)
		return "";
	char buffer[128];
	std::string result = "";
	while (!feof(pipe.get())) {
		if (fgets(buffer, 128, pipe.get()) != NULL)
			// cout << buffer;
			result += buffer;
	}

	TRH::TRH::TIME_SPENT_IN_HEURISTIC += double( clock () - begin_time ) /  CLOCKS_PER_SEC;
	// removeTempState(stateFileName);
	return result;
}

string TRH::buildCommand() {
	ostringstream cmd;
	cmd << H_CMD << " " 
		<< TEMP_FILE_PATH << TEMP_FILE_PREFIX << trhInstanceID 
			<< TEMP_DOMAIN_SUFFIX << TEMP_FILE_EXT
		<< " " << TEMP_FILE_PATH << TEMP_FILE_PREFIX << trhInstanceID 
			<< TEMP_FILE_EXT;
	return cmd.str();
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
			if ((osItr->lpTimestamp - (*sortedItr)->lpTimestamp) < -(EPSILON / 10.0)) {
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
		// cout << "Applying: " << PDDL::getActionName(eventToApply) << "-" 
		// 	<< eventToApply->time_spec << endl;
		Planner::ActionSegment nextSeg;
		if (eventToApply->time_spec == VAL::time_spec::E_AT) {
			int oldTIL = currSQI->state()->getEditableInnerState().nextTIL;
			nextSeg = Planner::ActionSegment(0, VAL::E_AT, oldTIL, Planner::RPGHeuristic::emptyIntList);
		} else {
			//Update divisionID / nextTIL
			eventToApply->divisionID = currSQI->state()->getInnerState().nextTIL;
			nextSeg = Planner::ActionSegment(eventToApply->action, eventToApply->time_spec, 
				eventToApply->divisionID, Planner::RPGHeuristic::emptyIntList);
		}
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
		// Planner::FFEvent::printPlan(succ->plan);
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

void TRH::writeBadState(const Planner::MinimalState & state,
		std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
		PDDL::PDDLStateFactory pddlFactory, int stateNum) {
	
	ostringstream stateFileName;
	stateFileName << "BadState" << trhInstanceID << "-" << stateNum;
	writeStateToFile(state, plan, timestamp, heuristic, 
		pddlFactory, stateFileName.str());
}


pair<PDDL::PDDLDomain, PDDL::PDDLState> TRH::writeStateToFile(const Planner::MinimalState & state,
	std::list<Planner::FFEvent>& plan, double timestamp, double heuristic, 
	PDDL::PDDLStateFactory pddlFactory, const string & filename) {

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
	
	string domainFileName = filename + TEMP_DOMAIN_SUFFIX;
	pddlState.writeDeTILedStateToFile(TEMP_FILE_PATH, filename);
	domain.writeToFile(TEMP_FILE_PATH, domainFileName);
	TRH::TRH::TIME_SPENT_IN_PRINTING_TO_FILE += double( clock () - begin_time ) /  CLOCKS_PER_SEC;
	return pair<PDDL::PDDLDomain, PDDL::PDDLState>(domain, pddlState);
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