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

bool KK::getOrderingConstratints(
	std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > & actionOrderings,
	const std::list<Planner::FFEvent> & plan, 
	const Planner::FFEvent * initialAction) {

	//Build a validation structure for the plan
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > validationStructure;
	bool success = getActionValidationStructure(validationStructure, plan, initialAction);
	if (!success) {
		return false;
	}
	// printValidationStructure(validationStructure);

	//Construct generalised ordering
	list<Planner::FFEvent>::const_iterator eventItr = plan.begin();
	const list<Planner::FFEvent>::const_iterator eventItrEnd = plan.end();
	const list<Planner::FFEvent>::const_iterator lastEventItr = std::prev(plan.end());
	for (; eventItr != lastEventItr; eventItr++) {
		//list<Planner::FFEvent>::const_iterator nextEventItr = plan.begin();
		list<Planner::FFEvent>::const_iterator nextEventItr = std::next(eventItr, 1); //advance to the next action
		const Planner::FFEvent * event = &(*eventItr); // a
		pair<const Planner::FFEvent *, const Planner::FFEvent *> initialOrdering(
			initialAction, event);
		actionOrderings.insert(initialOrdering);
		for (; nextEventItr != eventItrEnd; nextEventItr++) {
			const Planner::FFEvent * nextEvent = &(*nextEventItr); // b
			pair<const Planner::FFEvent *, const Planner::FFEvent *> actionOrdering(
				event, nextEvent);
			if (supportExists(event, nextEvent, validationStructure)) { // a supports b
				actionOrderings.insert(actionOrdering);
			} else if (threatExists(nextEvent, event, validationStructure, false)) { //b threatens c support a
				actionOrderings.insert(actionOrdering);
			} else if (threatExists(event, nextEvent, validationStructure, true)) { //a threatens b support c
				actionOrderings.insert(actionOrdering);
			}
		}
	}
	//Add initial ordering for last event if there is a plan 
	if (plan.size()) {
		pair<const Planner::FFEvent *, const Planner::FFEvent *> initialOrdering(
			initialAction, &*plan.rbegin());
		actionOrderings.insert(initialOrdering);
	}
	//printActionOrderings(actionOrderings);
	return true;
}

bool KK::supportExists(
	const Planner::FFEvent * producer, 
	const Planner::FFEvent * consumer,
	const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & validationStructure) {

	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator vItr =
		validationStructure.begin();
	const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator vItrEnd =
		validationStructure.end();

	for (;vItr != vItrEnd; vItr++) {
		Util::triple<const Planner::FFEvent *, PDDL::Literal> causalLink = *vItr;
		if ((causalLink.first == producer) && (causalLink.third == consumer)) {
			return true;
		}
	}
	return false;
 }

bool KK::threatExists(
	const Planner::FFEvent * threat, 
	const Planner::FFEvent * event,
	const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & validationStructure,
	bool eventIsProducer) {
	
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator vItr =
		validationStructure.begin();
	const std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> >::const_iterator vItrEnd =
		validationStructure.end();

	for (; vItr != vItrEnd; vItr++) {
		Util::triple<const Planner::FFEvent *, PDDL::Literal> causalLink = *vItr;

		if ((eventIsProducer) && (causalLink.first == event)) {
			if (KK::doesEventThreatenCausalLink(threat, &causalLink.second)) {
				return true;
			}
		} else if ((!eventIsProducer) && (causalLink.third == event)) {
			if (KK::doesEventThreatenCausalLink(threat, &causalLink.second)) {
				return true;
			}
		}
	}
	return false;
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
bool KK::getActionValidationStructure(
	std::set<Util::triple<const Planner::FFEvent *, PDDL::Literal> > & validationStructure,
	const std::list<Planner::FFEvent> & plan, 
	const Planner::FFEvent * initialAction) {

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
					&(*condItr), i, plan, initialAction);
			if (!support) {
				//If there is no support then this is a fail
				// cerr << "There was no support for " 
				//      << *condItr << "condition in action " 
				//      << PDDL::getActionName(event) << endl;
				return false;
			}
			//Add the ordering constraint
			Util::triple<const Planner::FFEvent *, PDDL::Literal> ordering;
			ordering.make_triple(support, *condItr, event);
			validationStructure.insert(ordering);
		}
	}
	// printValidationStructure(validationStructure);
	return true;
}

/**
 * Cycles backward through a plan to find the minimum action that
 * adds support for a condition. Minimum is defined as the action
 * which is farthest away without an action in between that removes
 * support for the precondition
 */
const Planner::FFEvent * KK::findMinimumSupportingAction(
	const PDDL::Literal * condition, 
	int indexOfEvent,
	const std::list<Planner::FFEvent> & plan,
	const Planner::FFEvent * initialAction) {


	list<Planner::FFEvent>::const_reverse_iterator eventItr = plan.rbegin();

	const Planner::FFEvent * support = NULL;
	//Advance to where the event is
	std::advance(eventItr, plan.size() - indexOfEvent - 1);
	eventItr++; //begin with immediately preceeding event
	for (; eventItr != plan.rend(); eventItr++) {
		const Planner::FFEvent * event = &(*eventItr);
		
		//Check if the event adds support for the condition
		std::list<PDDL::Proposition> effects = 
			PDDL::getActionEffects(event, condition->isPositive());
		if (PDDL::supported(condition->getProposition(), &effects)) {
			support = event;
			//break;
		}

		//Check if the event removes support for the condition
		effects = PDDL::getActionEffects(
			event, !condition->isPositive());
		if (PDDL::supported(condition->getProposition(), &effects)) {
			//If it does, then we have a problem
			return support;
		}
	}
	//Check if support is in the initial state
	if (support == NULL) {
		std::list<PDDL::Proposition> effects = 
			PDDL::getActionEffects(initialAction, condition->isPositive());
		if (PDDL::supported(condition->getProposition(), &effects)) {
			return initialAction;
		}
	}
	return support;
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
			<< " ordered before "
			<< PDDL::getActionName(ordering.second)
			<< std::endl;
}

}
