/*
 * kk.h
 *
 *  Created on: 25 Aug 2015
 *      Author: tony
 */

#ifndef __KK_KK_H_
#define __KK_KK_H_

#include <list>
#include <set>

#include "../FFEvent.h"
#include "../util/Util.h"
#include "Literal.h"
#include "PDDLUtils.h"


namespace KK {

class KK {

private:
	static KK * INSTANCE;
	KK() {
		}
		;
	KK(KK const & other) {
		}
		;
	KK& operator=(KK const&) {
		}
		;
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > getActionValidationStructure(
			const std::list<Planner::FFEvent> & plan, const Planner::FFEvent * initialAction);
	std::set<const Planner::FFEvent *> findAllThreateningActions(
			Util::triple<const Planner::FFEvent *, PDDL::Literal> * causalLink, 
			const std::list<Planner::FFEvent> & plan);
	bool doesEventThreatenCausalLink(const Planner::FFEvent * event, PDDL::Literal * condition);
	const Planner::FFEvent * findMinimumSupportingAction(
			const PDDL::Literal * condition, int indexOfEvent,
			const std::list<Planner::FFEvent> & plan);
	void printValidationStructure(
			std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & ordering);
	void printCausalLink(
			Util::triple<const Planner::FFEvent *, PDDL::Literal> & causalLink);
	void printActionOrderings(
			std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > & ordering);
	void printActionOrdering(
			std::pair<const Planner::FFEvent *, const Planner::FFEvent *> & ordering);

public:
	static KK * getInstance();
	std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > getOrderingConstratints(
		const std::list<Planner::FFEvent> & plan, const Planner::FFEvent * initialAction);

};

}

#endif /* __KK_KK_H_ */
