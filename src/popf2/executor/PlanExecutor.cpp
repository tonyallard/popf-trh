/*
 * PlanExecutor.cpp
 *
 *  Created on: 08 Sept 2017
 *      Author: tony
 */

#include <limits>

#include "PlanExecutor.h"

using namespace executor;
using namespace std;

PlanExecutor * PlanExecutor::INSTANCE = NULL;

PlanExecutor * PlanExecutor::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new PlanExecutor();
	}
	return INSTANCE;
}


void PlanExecutor::updateEventTimings(
	const std::set<Planner::FFEvent *> & plan,
	stn::ColinSTNImpl & stn,
	const Planner::FFEvent * initalEvent) {
	
	//Floyd-Warshall to determine consistent dispatch times
	std::vector<std::vector<double> > d = stn.toMatrix();
	for (int k = 0; k < stn.size(); k++) {
		for (int i = 0; i < stn.size(); i++) {
			for (int j = 0; j < stn.size(); j++) {
				if (d[i][k] + d[k][j] < d[i][j]) {
					d[i][j] = d[i][k] + d[k][j];
        			//p[i][j] = p[k][j];
	  			}
			}
		}
	}

	//Find index of initial event
	int initialEvtIdx = 0;
	for (int j = 0; j < stn.size(); j++) {
		if (stn.getVertex(j) == initalEvent) {
			initialEvtIdx = j;
			break;
		}
	}

	for (int i = 0; i < stn.size(); i++) {
		
		const Planner::FFEvent * evt = stn.getVertex(i);
		Planner::FFEvent * event = NULL;
		for (auto aEvent : plan) {
			if (evt == &(*aEvent)) {
				event = &*aEvent;
				break;
			}
		}
		if (d[i][i] < -stn::ColinSTNImpl::ACCURACY) {
			cerr << "Inconsistent STN found while dispatching (" << d[i][i] << ")." << endl;
		}
		event->lpTimestamp = d[initialEvtIdx][i];
	}

}
  
/**
 * Create a minimal STN
 * 
 * This is based on:
 * Muscettola, Nicola, Paul Morris, and Ioannis Tsamardinos. 
 * "Reformulating temporal plans for efficient execution." KR. 1998.
 *
 */
void PlanExecutor::createMinimalSTN(stn::ColinSTNImpl & stn,
	const std::set<Planner::FFEvent *> & plan) {
	// cycle through each triple of nodes and determine 
	// if there are any domincance constraints
	// Minimal Dispatch Filtering Algorithm
	// cout << "Remove dominated edges" << endl;
	set<const Util::triple<const Planner::FFEvent *, double> *> markedForRemoval;
	std::set<Planner::FFEvent *>::const_iterator aItr = plan.begin();
	for (; aItr != plan.end(); aItr++) {
		const Planner::FFEvent * a = *aItr;
		std::set<Planner::FFEvent *>::const_iterator bItr = plan.begin();
		for (; bItr != plan.end(); bItr++) {
			const Planner::FFEvent * b = *bItr;
			const Util::triple<const Planner::FFEvent *, double> * aTOb = 
					stn.STN::getEdge(a, b);
			//Check an edge exists
			if (aTOb) {
				std::set<Planner::FFEvent *>::const_iterator cItr = plan.begin();
				for (; cItr != plan.end(); cItr++) {
					const Planner::FFEvent * c = *cItr;
					const Util::triple<const Planner::FFEvent *, double> * aTOc = 
						stn.STN::getEdge(a,c);
					//Check an edge exists and is the same sign as the previous
					if ((aTOc) && ((aTOb->second < 0) == (aTOc->second < 0))){
						const Util::triple<const Planner::FFEvent *, double> * bTOc = 
							stn.STN::getEdge(b, c);
						//Check an edge exists and is the same sign as the previous
						if ((bTOc) && ((aTOb->second < 0) == (bTOc->second < 0))) {
							//Triangle equality is satisfied
							if (aTOb->second + bTOc->second == aTOc->second) {
								if (((aTOc < 0) && (aTOb < 0)) ||
									((aTOc >= 0) && (bTOc >= 0))) {
									//A to C is dominated
									markedForRemoval.insert(aTOc);
								}
							}
						}
					}
				}
			}
		}	
	}
	stn.removeAllEdges(markedForRemoval);
}

bool PlanExecutor::containsScheduledActions(const Planner::FFEvent * event, 
		const set<const Util::triple<const Planner::FFEvent *, double> *> & constraints,
		const set<const Planner::FFEvent *> & alreadyScheduled) {

	set<const Util::triple<const Planner::FFEvent *, 
		double> *>::const_iterator constItr = constraints.begin();

	for (; constItr != constraints.end(); constItr++) {
		const Util::triple<const Planner::FFEvent *, 
			double> * constraint = *constItr;
		if ((constraint->first == event) && 
			(alreadyScheduled.find(constraint->third) == alreadyScheduled.end())) {
			return false;
		}
		else if ((constraint->third == event) && 
			(alreadyScheduled.find(constraint->first) == alreadyScheduled.end())) {
			return false;
		}
	}
	return true;
}

pair<double, double> PlanExecutor::getExecutionTime(const Planner::FFEvent * event, 
		const set<const Util::triple<const Planner::FFEvent *, 
			double> *> & constraints) {
	if (constraints.empty()) {
		return pair<double, double>(0.0, 0.0);
	}

	pair<double, double> executionTime = pair<double, double>(
		-std::numeric_limits<double>::max(), std::numeric_limits<int>::max());

	set<const Util::triple<const Planner::FFEvent *, 
		double> *>::const_iterator constItr = constraints.begin();
	for (; constItr != constraints.end(); constItr++) {
		const Util::triple<const Planner::FFEvent *, 
			double> * constraint = *constItr;
		if (constraint->first == event) {
			double minStart = constraint->third->lpTimestamp - constraint->second;
			if (executionTime.first < minStart) {
				executionTime.first = minStart;
			}
		} else if (constraint->third == event) {
			double maxStart = constraint->first->lpTimestamp + constraint->second;
			if (executionTime.second > maxStart) {
				executionTime.second = maxStart;
			}
		}
	}
	return executionTime;
}

set<const Util::triple<const Planner::FFEvent *, 
	double> *> PlanExecutor::getPrecedenceConstraints(
		const Planner::FFEvent * event, const stn::ColinSTNImpl & stn) {
	set<const Util::triple<const Planner::FFEvent *, 
		double> *> precedenceConstraints;
	
	std::vector<const Util::triple<const Planner::FFEvent *, 
		double> *> inEdges = stn.getInEdges(event);
	std::vector<const Util::triple<const Planner::FFEvent *, 
		double> *>::const_iterator edgeItr = inEdges.begin();

	for (; edgeItr != inEdges.end(); edgeItr++) {
		if ((*edgeItr)->second > 0) {
			precedenceConstraints.insert(*edgeItr);
		}
	}

	std::vector<const Util::triple<const Planner::FFEvent *, 
		double> *> outEdges = stn.getOutEdges(event);
	edgeItr = outEdges.begin();

	for (; edgeItr != outEdges.end(); edgeItr++) {
		if ((*edgeItr)->second <= 0) {
			precedenceConstraints.insert(*edgeItr);
		}
	}
	return precedenceConstraints;
}