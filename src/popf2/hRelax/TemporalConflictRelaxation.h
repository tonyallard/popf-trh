/*
 * hRelax.h
 *
 *  Created on: 22 Sep 2015
 *      Author: tony
 */

#ifndef __HRELAX_TEMPORALCONFLICTRELAXATION_H_
#define __HRELAX_TEMPORALCONFLICTRELAXATION_H_

#include <set>
#include <map>
#include <vector>

#include "../util/Util.h"
#include "../FFEvent.h"
#include "../solver.h"

using namespace std;

namespace hRelax {

class TemporalConflictRelaxation {

private:
	vector<Util::triple<pair<int,double>, double, double> > decisionVars;
	vector<vector<pair<int,double> > > constraintRows;
	set<const Util::triple<const Planner::FFEvent *, double> *> relaxableConstraints;
	//Map conflicts to positions in solver matrix
	map<const Util::triple<const Planner::FFEvent *, double> *, int> conflictToMatrixIdxMap;
	//Static vars
	static const double INF_UPPER_BOUND;
	static const double UNITY_COEFFICIENT;

	//Deny copying
	TemporalConflictRelaxation(TemporalConflictRelaxation const & other) {
	}
	;
	TemporalConflictRelaxation& operator=(TemporalConflictRelaxation const&) {
	}
	;

	//Private functions
	map<int, double> solve();
	void addDecisionVarsToObjectiveFunction(
		std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts);
	void addSTNConsistencyConstraint(
			std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts);
	bool constraintExistsInSolver(const Util::triple<const Planner::FFEvent *, double> * constraint);
	set<const Util::triple<const Planner::FFEvent *, double> *> getTILConstraints(
			const std::set<
					const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
			const Planner::FFEvent * initialEvent);
	void addRelaxableConstraints(
			const std::set<
					const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
			const std::set<
					const Util::triple<const Planner::FFEvent *, double> *> & tilConstraints);
public:
	TemporalConflictRelaxation(){};
	//Destructor
	~TemporalConflictRelaxation() {
	}
	std::set<
			std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> > determineTemporalRelaxations(
			std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
			const Planner::FFEvent * initialEvent);

};

}

#endif /* __HRELAX_TEMPORALCONFLICTRELAXATION_H_ */
