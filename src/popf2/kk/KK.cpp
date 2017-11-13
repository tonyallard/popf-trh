/*
 * kk.cpp
 *
 *  Created on: 25 Aug 2015
 *      Author: tony
 */

#include <iostream>

#include "KK.h"
#include "Proposition.h"
#include "LiteralFactory.h"

namespace KK {

KK * KK::INSTANCE = NULL;

KK * KK::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new KK();
	}
	return INSTANCE;
}

std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > KK::getOrderingConstratints(
		const std::list<Planner::FFEvent> & plan, const Planner::FFEvent * initialAction) {
	//Build a validation structure for the plan
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > validationStructure =
			getActionValidationStructure(plan, initialAction);

	std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > actionOrderings;
	//Construct generalised ordering
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator vItr =
			validationStructure.begin();
	const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator vItrEnd =
			validationStructure.end();
	for (int i = 0; vItr != vItrEnd; vItr++, i++) {
		Util::triple<const Planner::FFEvent *, PDDL::Literal> causalLink = *vItr;
		// Add causal link ordering
		pair<const Planner::FFEvent *, const Planner::FFEvent *> actionOrdering(
				causalLink.first, causalLink.third);
		actionOrderings.insert(actionOrdering);
		// Add ordering for all threatening actions
		std::set<const Planner::FFEvent *> threatingActions =
				findAllThreateningActions(&causalLink, plan);
		std::set<const Planner::FFEvent *>::const_iterator taItr =
				threatingActions.begin();
		const std::set<const Planner::FFEvent *>::const_iterator taItrEnd =
				threatingActions.end();
		for (; taItr != taItrEnd; taItr++) {
			const Planner::FFEvent * threateningAction = *taItr;
			//Add unique ordering for before first action and after second
			pair<const Planner::FFEvent *, const Planner::FFEvent *> threatOrderingBefore(
					threateningAction, causalLink.first);
			pair<const Planner::FFEvent *, const Planner::FFEvent *> threatOrderingAfter(
					causalLink.third, threateningAction);
			//If threatening action is before the first action in the casual chain
			//add an ordering to explicitly state that requirement
			if ((PDDL::isBefore(threateningAction, causalLink.first, plan)) &&
				(causalLink.first != initialAction)) {
				actionOrderings.insert(threatOrderingBefore);
			}
			//If the threatening action is after the second action in the causal chain
			//add an ordering to explicitly state that requirement
			if (PDDL::isAfter(threateningAction, causalLink.third, plan)) {
				actionOrderings.insert(threatOrderingAfter);
			}
		}
	}
	// printActionOrderings(actionOrderings);
	return actionOrderings;
}

/**
 * Finds all actions that could remove support for the condition in the causal link
 */
std::set<const Planner::FFEvent *> KK::findAllThreateningActions(
		Util::triple<const Planner::FFEvent *, PDDL::Literal> * causalLink,
		const std::list<Planner::FFEvent> & plan) {
	std::set<const Planner::FFEvent *> threateningActions;
	std::list<Planner::FFEvent>::const_iterator eventItr = plan.begin();
	const std::list<Planner::FFEvent>::const_iterator eventItrEnd = plan.end();
	for (; eventItr != eventItrEnd; eventItr++) {
		const Planner::FFEvent * event = &(*eventItr);
		if ((event != causalLink->first) && (event != causalLink->third)) {
			if ((event->time_spec != VAL::time_spec::E_AT)
					&& ((event->action == causalLink->first->action)
					|| (event->action == causalLink->third->action))) {
				continue;
			}
			if (KK::doesEventThreatenCausalLink(event, &causalLink->second)) {
				threateningActions.insert(event);
			}
		}
	}
	return threateningActions;
}

/**
 * Checks if event could remove the support of the condition
 */
bool KK::doesEventThreatenCausalLink(const Planner::FFEvent * event,
		PDDL::Literal * condition) {
	//Get the effects of the action opposite to the sign of the condition
	std::list<PDDL::Proposition> effects = PDDL::getActionEffects(
			event, !condition->isPositive());
	//Check if the effect removes the support of the condition
	if (PDDL::supported(condition->getProposition(), &effects)) {
		return true;
	}
	return false;
}

/**
 * For each action finds the minimum action which supports each of its preconditions
 * and generates a validation structure that identifies these dependencies
 */
std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > KK::getActionValidationStructure(
		const std::list<Planner::FFEvent> & plan, const Planner::FFEvent * initialAction) {

	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > validationStructure;
	
	//Cycle through events in the plan
	list<Planner::FFEvent>::const_iterator eventItr = plan.begin();
	const list<Planner::FFEvent>::const_iterator eventItrEnd = plan.end();
	for (int i = 0; eventItr != eventItrEnd; eventItr++, i++) {
		//for each precondition find the minimum action that achieves
		//the precondition without something in the middle that removes it.
		const Planner::FFEvent * event = &(*eventItr);
		std::list<PDDL::Literal> conditions = PDDL::getActionConditions(event);
		//Handle special case where conditions are empty
		if (conditions.empty()) {
			Util::triple<const Planner::FFEvent *, PDDL::Literal> ordering;
			ordering.make_triple(initialAction, 
				PDDL::LiteralFactory::getInstance()->getEmptyLiteral(), event);
			validationStructure.insert(ordering);
			continue;
		}

		std::list<PDDL::Literal>::const_iterator condItr = conditions.begin();
		const std::list<PDDL::Literal>::const_iterator condItrEnd =
				conditions.end();
		for (; condItr != condItrEnd; condItr++) {
			const Planner::FFEvent * support = findMinimumSupportingAction(
					&(*condItr), i, plan);
			if (!support) {
				//if null it must be in the initial state
				//Make minimum supporting action our initial action
				support = initialAction;
			}
			//Add the ordering constraint
			Util::triple<const Planner::FFEvent *, PDDL::Literal> ordering;
			ordering.make_triple(support, *condItr, event);
			validationStructure.insert(ordering);
		}
	}
	// printValidationStructure(validationStructure);
	return validationStructure;
}

/**
 * Cycles backward through a plan to find the minimum action that
 * adds support for a condition. Minimum is defined as the action
 * which is farthest away without an action in between that removes
 * support for the precondition
 */
const Planner::FFEvent * KK::findMinimumSupportingAction(
		const PDDL::Literal * condition, int indexOfEvent,
		const std::list<Planner::FFEvent> & plan) {

	list<Planner::FFEvent>::const_reverse_iterator eventItr = plan.rbegin();

	//Advance to where the event is
	std::advance(eventItr, plan.size() - indexOfEvent - 1);
	eventItr++; //begin with immediately preceeding event
	const Planner::FFEvent * currentSupportingEvent = 0;
	for (; eventItr != plan.rend(); eventItr++) {
		const Planner::FFEvent * event = &(*eventItr);
		//Check if the event removes support for the condition
		std::list<PDDL::Proposition> effects = PDDL::getActionEffects(
				event, !condition->isPositive());
		if (PDDL::supported(condition->getProposition(), &effects)) {
			//If it is return previous suporter
			return currentSupportingEvent;
		}
		//get effects
		effects = PDDL::getActionEffects(event, condition->isPositive());
		//check if effects support this
		if (PDDL::supported(condition->getProposition(), &effects)) {
			currentSupportingEvent = event;
		}
	}
	return currentSupportingEvent;
}

void KK::printValidationStructure(
		std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & ordering) {
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator ordItr =
			ordering.begin();
	const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator ordItrEnd =
			ordering.end();
	cout << "There are " << ordering.size() << " causal links" << std::endl;
	for (; ordItr != ordItrEnd; ordItr++) {
		Util::triple<const Planner::FFEvent *, PDDL::Literal> order = *ordItr;
		KK::printCausalLink(order);
	}
}

void KK::printCausalLink(
		Util::triple<const Planner::FFEvent *, PDDL::Literal> & causalLink) {
	cout << PDDL::getActionName(causalLink.first)
			<< " supports " << causalLink.second << " for "
			<< PDDL::getActionName(causalLink.third)
			<< std::endl;
}

void KK::printActionOrderings(
		std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > & orderings) {
	std::set<pair<const Planner::FFEvent *, const Planner::FFEvent *> >::const_iterator ordItr =
			orderings.begin();
	const std::set<pair<const Planner::FFEvent *, const Planner::FFEvent *> >::const_iterator ordItrEnd =
			orderings.end();
	cout << "There are " << orderings.size() << " action orderings"
			<< std::endl;
	for (; ordItr != ordItrEnd; ordItr++) {
		std::pair<const Planner::FFEvent *, const Planner::FFEvent *> ordering =
				*ordItr;
		KK::printActionOrdering(ordering);
	}
}

void KK::printActionOrdering(
		std::pair<const Planner::FFEvent *, const Planner::FFEvent *> & ordering) {
	cout << PDDL::getActionName(ordering.first)
			<< "-" << ordering.first->time_spec
			<< " ordered before "
			<< PDDL::getActionName(ordering.second)
			<< "-" << ordering.second->time_spec
			<< std::endl;
}

}
