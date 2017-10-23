/*
 * MathematicalEffect.h
 *
 *  Created on: 3 May 2016
 *      Author: tony
 */

#ifndef __PDDL_PNEEFFECT
#define __PDDL_PNEEFFECT

#include <string>
#include <list>
#include <map>

#include "PDDLObject.h"
#include "PNE.h"
#include "../RPGBuilder.h"

using namespace std;

namespace PDDL {

class PNEEffect {

private:
	VAL::assign_op op;
	PDDL::PNE effectVar;
	list<Planner::RPGBuilder::Operand> equation;

public:
	PNEEffect(PDDL::PNE effectVar, VAL::assign_op op, list<Planner::RPGBuilder::Operand> equation) :
		effectVar(effectVar), op(op), equation(equation) {
	}
	;
	std::string toParameterisedString(map<PDDLObject, string> parameterTable) const;
	friend std::ostream & operator<<(std::ostream & output, const PDDL::PNEEffect & pneEffect);

};
}
#endif /* __PDDL_PNEEFFECT */
