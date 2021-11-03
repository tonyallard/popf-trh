/*
 * PlanExecutor.h
 *
 *  Created on: 08 Sept 2017
 *      Author: tony
 */

#ifndef __EXECUTOR_PLANEXECUTOR_H_
#define __EXECUTOR_PLANEXECUTOR_H_

#include <set>
#include <list>

#include "../util/Util.h"
#include "../FFEvent.h"
#include "../stn/ColinSTNImpl.h"

namespace executor {

class PlanExecutor {

private:

	static PlanExecutor * INSTANCE;
	//Singleton
	PlanExecutor() {
	}
	;
	PlanExecutor(PlanExecutor const & other) {
	}
	;
	PlanExecutor& operator=(PlanExecutor const&) {
	}
	;
	void createMinimalSTN(stn::ColinSTNImpl & stn,
	const std::set<Planner::FFEvent *> & plan);
	bool containsScheduledActions(const Planner::FFEvent * event, 
		const set<const Util::triple<const Planner::FFEvent *, double> *> & constraints,
		const set<const Planner::FFEvent *> & alreadyScheduled);

	pair<double, double> getExecutionTime(const Planner::FFEvent * event, 
		const set<const Util::triple<const Planner::FFEvent *, double> *> & constraints);

	set<const Util::triple<const Planner::FFEvent *, double> *> getPrecedenceConstraints(
		const Planner::FFEvent * event, const stn::ColinSTNImpl & stn);

public:
	static PlanExecutor * getInstance();
	void updateEventTimings(const std::set<Planner::FFEvent *> & plan,
		stn::ColinSTNImpl & stn,
		const Planner::FFEvent * initalEvent);
};

}

#endif /* __EXECUTOR_PLANEXECUTOR_H_ */