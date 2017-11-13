/*
 * HRelax.cpp
 *
 *  Created on: 5 Oct 2015
 *      Author: tony
 */

#include <sstream>

#include "HRelax.h"
#include "../kk/KK.h"
#include "../stn/ColinSTNImpl.h"
#include "../pddl/PDDLUtils.h"
#include "../util/Util.h"
#include "../itc/ITC.h"
#include "../executor/PlanExecutor.h"
#include "TemporalConflictRelaxation.h"

using namespace hRelax;
using namespace std;

const double HRelax::PROBLEM_UNSOLVABLE_H_VALUE = -1.0;
const char HRelax::TIL_STRING_DELIM = '-';

const string HRelax::H_PLAN_DELIM_START = "=====Plan Start====="; 
const string HRelax::H_PLAN_DELIM_STOP = "=====Plan Stop=====";

HRelax * HRelax::INSTANCE = NULL;

HRelax * HRelax::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new HRelax();
	}
	return INSTANCE;
}

pair<double, list<Planner::FFEvent> > HRelax::getHeuristic(std::list<Planner::FFEvent> & plan) {
	//Create Dummy Initial Event @ t_0 for STN
	Planner::FFEvent * initialEvent = createInitialEvent();

	//Determine Plan De-Ordering
    ::KK::KK * deordAlg = ::KK::KK::getInstance();
	std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > orderingConstraints =
			deordAlg->getOrderingConstratints(plan, initialEvent);
	// Generate STN
	stn::ColinSTNImpl stn = stn::ColinSTNImpl::makeColinSTN(plan, 
		orderingConstraints);
	
	// Check consistency --> should be 'yes' or we have a problem
	bool consistent = stn.isConsistent();
	
	// cout << "Is STN consistent? " << (consistent ? "yes" : "no") << std::endl;
	if (!consistent) {
		cerr << "Error: Initial STN not consistent. Something in the relaxed plan is wrong!"
			<< endl;
		assert(false);
	}

	//Re-tighten TIL constraint
	// cout << "There are " << tilEvents.size() << " TIL actions." << std::endl;
	reAddTemporalConstraintsToTIL(stn, plan, initialEvent);
	// cout << stn << endl;

	// Re-check consistency --> if yes then heuristic is 0
	consistent = stn.isConsistent();
	// cout << "Is STN still consistent? " << (consistent > 0 ? "yes" : "no")
	// 		<< std::endl;
	if (consistent) {
		// cout << "This is a plan to the original problem. Dispatching an executable plan."
			// << endl;
		//The relaxed plan is a solution to the original problem
		executePlan(plan, stn, initialEvent);
		return pair<double, list<Planner::FFEvent> >(0.0, plan);
	}
	
	//Determine the minimum relaxations required to make the STN consistent
	TemporalConflictRelaxation tcr;
	//Keep track of the original constraint weights 
	//so we can determine the deltas
	map<const Util::triple<const Planner::FFEvent *, double> *, 
		double> relaxHist;

	int itrs = 0;
	while (!consistent) {
		// cout << "Relaxation Iteration " << ++itrs << endl;
		//Determine constraints involved in 
		//negative cycle
		ITC::ITC * itc = ITC::ITC::getInstance();
		std::set<const Util::triple<const Planner::FFEvent *, double> *> conflictedConstraints =
			// itc->checkTemporalConsistencyBF(&stn, initialEvent);
			itc->checkTemporalConsistencyFW(&stn);

		// cout << "Found " << conflictedConstraints.size()
		// 	<< " conflicted constraints" << std::endl;

		if (!conflictedConstraints.size()) {
			// cerr << "Error: No conflicted constraints found" << endl;
			assert(false);
		}

		//Determine minimum required relaxations
		//To make remove negative cycle
		// cout << "Determining Relaxations..." << endl;
		std::set<std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> > relaxations = tcr.determineTemporalRelaxations(
			conflictedConstraints, initialEvent);
		// cout << getRelaxationsString(relaxations) << endl;
		// cout << "Updating edge weights..." << endl;
		//Make changes to STN
		std::set<std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> >::const_iterator relaxItr = relaxations.begin();
		for (; relaxItr != relaxations.end(); relaxItr++) {
			std::pair<const Util::triple<const Planner::FFEvent *, double> *, 
					double> relaxation = *relaxItr;
			//If relaxation is new store its original value
			//for comparison later
			if (relaxHist.find(relaxation.first) == relaxHist.end()) {
				relaxHist[relaxation.first] = relaxation.first->second;
			}
			//Get the delta reqiured to make STN consistent
			stn.updateEdgeWeight(relaxation.first->first, relaxation.first->third,
					relaxation.second);
		}
		consistent = stn.isConsistent();
		// cout << "Is STN consistent yet? " << (consistent ? "yes" : "no")
		// 	<< std::endl;
	}
	//If we get here the STN should be consistent
	consistent = stn.isConsistent();
	if (!consistent) {
		cerr << "Error: The STN is still not consistent!" << endl;
		assert(false);
	}


	//Sum relaxations to calculate h-val
	double heuristic = getHeuristicValue(relaxHist);
	delete initialEvent;
	return pair<double, list<Planner::FFEvent> >(heuristic, plan);
}

void HRelax::executePlan(std::list<Planner::FFEvent> & plan, 
		stn::ColinSTNImpl & stn, Planner::FFEvent * initialEvent) {
	std::set<Planner::FFEvent *> actionList;
	actionList.insert(initialEvent);
	std::list<Planner::FFEvent>::iterator planItr = plan.begin();
	for (; planItr != plan.end(); planItr++) {
		actionList.insert(&*planItr);
	}
	// Update plan with relaxations for output
	executor::PlanExecutor * executor = executor::PlanExecutor::getInstance();
	executor->updateEventTimings(actionList, stn);
}

/**
 *
 * Sum relaxations to calculate h-val
 */
double HRelax::getHeuristicValue(map<const Util::triple<const Planner::FFEvent *, double> *,
		double> & relaxedTemporalConstraints) {
	double heuristic = 0.0;
	map<const Util::triple<const Planner::FFEvent *, double> *,
		double>::const_iterator finalRelaxItr = relaxedTemporalConstraints.begin();
	for (; finalRelaxItr != relaxedTemporalConstraints.end(); finalRelaxItr++) {
		//Sum relaxations to find h-val
		heuristic += abs(finalRelaxItr->second - finalRelaxItr->first->second);	
	}
	return heuristic;
}

/**
 * Cycles through til events and re-adds the temporal constraints
 */
void HRelax::reAddTemporalConstraintsToTIL(stn::ColinSTNImpl & stn,
		const std::list<Planner::FFEvent> & plan,
		const Planner::FFEvent * initialEvent) {
	std::list<Planner::FFEvent>::const_iterator planItr =
			plan.begin();
	for (; planItr != plan.end(); planItr++) {
		if (planItr->time_spec == VAL::time_spec::E_AT) {
			int tilID = planItr->divisionID;
			double tilTemporalConstraint = Planner::RPGBuilder::getAllTimedInitialLiterals()[tilID]->duration;
			const Planner::FFEvent * event = &(*planItr);
			// Find and update edge representing max time constraint
			if (stn.edgeExists(initialEvent, event)) {
				stn.updateEdgeWeight(initialEvent, event,
						tilTemporalConstraint);
			} else {
				std::cerr << "Error: Maximum TIL causal link does not exist!" << std::endl;
				assert(false);
			}
			// Find and update edge representing min time constraint
			if (stn.edgeExists(event, initialEvent)) {
				stn.updateEdgeWeight(event, initialEvent,
						-tilTemporalConstraint);
			} else {
				std::cerr << "Error: Minimum TIL causal link does not exist!" << std::endl;
				assert(false);
			}
		}
	}
}

std::string HRelax::getConstraintsString(
	std::set<const Util::triple<const Planner::FFEvent *, double> *> constraints) {
	ostringstream output;
	output << "There are " << constraints.size() << " constraints." << endl;
	std::set<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator constItr
		= constraints.begin();
	for (; constItr != constraints.end(); constItr++) {
		output << getConstraintString(*constItr) << endl;
	}
	return output.str();
}

std::string HRelax::getRelaxationsString(std::set<
			std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> > relaxations) {
	ostringstream output;
	output << "There are " << relaxations.size() << " relaxations" << endl;
	std::set<
			std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> >::const_iterator relaxItr = relaxations.begin();
	for (; relaxItr != relaxations.end(); relaxItr++) {
		std::pair<const Util::triple<const Planner::FFEvent *, double> *, double> relaxation =
				*relaxItr;
		output << getConstraintString(relaxation.first) << " Relaxed to: "
			<< relaxation.second << endl;
	}
	return output.str();
}

std::string HRelax::getConstraintString(
	const Util::triple<const Planner::FFEvent *, double> * constraint) {
	ostringstream output;
	output << PDDL::getActionName(constraint->first) << "-"
			<< constraint->first->time_spec << "--["
			<< constraint->second << "]--"
			<< PDDL::getActionName(constraint->third)
			<< "-" << constraint->third->time_spec;
	return output.str();
}

Planner::FFEvent * HRelax::createInitialEvent() {
	VAL::operator_symbol * opSym = new VAL::operator_symbol(
			"Dummy-Initial-Action");
	VAL::var_symbol_list * vsl = new VAL::var_symbol_list();
	VAL::operator_ * op = new VAL::operator_(opSym, vsl, 0, 0, 0);
	Inst::instantiatedOp * iop = new Inst::instantiatedOp(op, 0);
	Planner::RPGBuilder::addInstantiatedOp(iop);
	iop->setID(Inst::instantiatedOp::howMany());
	Planner::FFEvent * initialEvent = new Planner::FFEvent(iop, 0.0, 0.0);
	return initialEvent;
}