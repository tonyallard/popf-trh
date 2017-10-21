/*
 * LiteralFactory.cpp
 *
 *  Created on: 29 Apr 2016
 *      Author: tony
 */

#include "LiteralFactory.h"
#include "Literal.h"
#include "PropositionFactory.h"
#include "Proposition.h"

using namespace std;

namespace PDDL {

LiteralFactory * LiteralFactory::INSTANCE = NULL;

LiteralFactory * LiteralFactory::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new LiteralFactory();
	}
	return INSTANCE;
}

PDDL::Literal LiteralFactory::getLiteral(const VAL::simple_goal * goal){
	PDDL::Proposition prop = PropositionFactory::getInstance()->
		getProposition(goal->getProp(), true, false);
	bool positive = goal->getPolarity() != VAL::polarity::E_NEG;
	return Literal(prop, positive);
}

list<PDDL::Literal> LiteralFactory::getLiterals(list<Inst::Literal*> * propositions,
		bool positive) {
	list<PDDL::Literal> literals;
	list<Inst::Literal*>::const_iterator propItr = propositions->begin();
	const list<Inst::Literal*>::const_iterator propItrEnd =
			propositions->end();

	for (; propItr != propItrEnd; propItr++) {
		PDDL::Literal literal = getLiteral(*propItr, positive);
		literals.push_back(literal);
	}
	return literals;
}

PDDL::Literal LiteralFactory::getLiteral(const Inst::Literal * aProposition, bool positive) {
	PDDL::Proposition proposition = PropositionFactory::getInstance()->
		getProposition(aProposition);
	return PDDL::Literal(proposition, positive);
}

}
