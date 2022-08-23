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
	bool getActionValidationStructure(
		std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & validationStructure,
		const std::list<Planner::FFEvent> & plan, 
		const Planner::FFEvent * initialAction);

	bool supportExists(
		const Planner::FFEvent * producer, 
		const Planner::FFEvent * consumer,
		const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & validationStructure);

	bool threatExists(
		const Planner::FFEvent * threat, 
		const Planner::FFEvent * event,
		const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & validationStructure,
		bool eventIsProducer);
		
	bool doesEventThreatenCausalLink(
		const Planner::FFEvent * event, 
		PDDL::Literal * condition);

	const Planner::FFEvent * findMinimumSupportingAction(
		const PDDL::Literal * condition, 
		int indexOfEvent,
		const std::list<Planner::FFEvent> & plan,
		const Planner::FFEvent * initialAction);

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
	
	bool getOrderingConstratints(
		std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > & actionOrderings,
		const std::list<Planner::FFEvent> & plan, 
		const Planner::FFEvent * initialAction);

};

}

#endif /* __KK_KK_H_ */
