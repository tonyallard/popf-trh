/*
 * PDDLStateFactory.h
 *
 *  Created on: 2 Jul 2015
 *      Author: tony
 */

#ifndef __LITERALFACTORY
#define __LITERALFACTORY

#include <string>
#include <list>

#include "Literal.h"

#include "ptree.h"
#include "instantiation.h"

namespace PDDL {

class LiteralFactory {
public:

	static LiteralFactory * getInstance();

	PDDL::Literal getLiteral(const VAL::simple_goal * goal);
	PDDL::Literal getLiteral(const Inst::Literal * aLiteral, bool positive);
	std::list<PDDL::Literal> getLiterals(std::list<Inst::Literal*> * literals,
		bool positive);

private:
	//Singleton Instance
	static LiteralFactory * INSTANCE;
	
	//Private constructor
	LiteralFactory(){};
	//Singleton
	LiteralFactory(LiteralFactory const & other) {
	}
	;
	//Singleton
	LiteralFactory& operator=(LiteralFactory const&) {
	}
	;
};

}

#endif /* __LITERALFACTORY */
