/*
 * TILFactory.cpp
 *
 *  Created on: 20 Oct 2017
 *      Author: tony
 */

#include "PropositionFactory.h"
#include "TILFactory.h"
#include "PDDLUtils.h"

using namespace std;

namespace PDDL {

TILFactory * TILFactory::INSTANCE = NULL;

TILFactory * TILFactory::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new TILFactory();
	}
	return INSTANCE;
}

PDDL::TIL TILFactory::getTIL(Planner::RPGBuilder::FakeTILAction aTIL, int tilIndex,
		std::list<std::pair<std::string, std::string> > constants) {
	std::list<PDDL::Proposition> addEffects;
	std::list<PDDL::Proposition> delEffects;

	std::set<PDDLObject> parameters;
	//Get Add effects
	std::list<Inst::Literal*>::const_iterator tilAddLitInt =
			aTIL.addEffects.begin();
	std::list<Inst::Literal*>::const_iterator tilAddLitIntEnd =
			aTIL.addEffects.end();
	for (; tilAddLitInt != tilAddLitIntEnd; tilAddLitInt++) {
		Inst::Literal * literal = (*tilAddLitInt);
		parameters = PDDL::extractParameters(literal, parameters, constants);
		PDDL::Proposition lit = PropositionFactory::getInstance()->
			getProposition(literal);
		addEffects.push_back(lit);
	}

	//Get Del Effects
	std::list<Inst::Literal*>::const_iterator tilDelLitInt =
			aTIL.delEffects.begin();
	std::list<Inst::Literal*>::const_iterator tilDelLitIntEnd =
			aTIL.delEffects.end();
	for (; tilDelLitInt != tilDelLitIntEnd; tilDelLitInt++) {
		Inst::Literal * literal = (*tilDelLitInt);
		parameters = PDDL::extractParameters(literal, parameters, constants);
		PDDL::Proposition lit = PropositionFactory::getInstance()->
			getProposition(literal);
		delEffects.push_back(lit);
	}
	return PDDL::TIL(tilIndex, addEffects, delEffects, parameters);
}

}