/*
 * HRelax.h
 *
 *  Created on: 5 Oct 2015
 *      Author: tony
 */

#ifndef __HRELAX_HRELAX_H_
#define __HRELAX_HRELAX_H_

#include <list>
#include <set>

#include "../stn/ColinSTNImpl.h"

#include "../FFEvent.h"

namespace hRelax {

class HRelax {

private:
	static const char TIL_STRING_DELIM;

	bool earlyTermination;
	int heuristicMode;

	void executePlan(std::list<Planner::FFEvent> & plan, 
		stn::ColinSTNImpl & stn, Planner::FFEvent * initialEvent);
	double getHeuristicValue(map<const Util::triple<const Planner::FFEvent *, double> *,
		double> & relaxedTemporalConstraints);
	void reAddTemporalConstraintsToTIL(stn::ColinSTNImpl & stn,
		const std::list<Planner::FFEvent> & plan,
		const Planner::FFEvent * initialEvent);
	
	std::string getConstraintsString(
	std::set<const Util::triple<const Planner::FFEvent *, double> *> constraints);
	std::string getRelaxationsString(std::set<
			std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> > relaxations);
	std::string getConstraintString(
	const Util::triple<const Planner::FFEvent *, double> * constrint);
	
	Planner::FFEvent * createInitialEvent();
public:
	const static double PROBLEM_UNSOLVABLE_H_VALUE;
	const static string H_PLAN_DELIM_START;
	const static string H_PLAN_DELIM_STOP;
	
	HRelax() : earlyTermination(true), heuristicMode(0) {
	};

	HRelax(bool earlyTermination, int heuristicMode) 
		: earlyTermination(earlyTermination), heuristicMode(heuristicMode) {
	};

	pair<double, list<Planner::FFEvent> > getHeuristic(
		std::list<Planner::FFEvent> & plan,
		int relaxedplanLength);

};

}

#endif /* __HRELAX_HRELAX_H_ */