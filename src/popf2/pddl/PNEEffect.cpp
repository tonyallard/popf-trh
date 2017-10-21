/*
 * PNEEffect.cpp
 *
 *  Created on: 3 May 2016
 *      Author: tony
 */

#include <sstream>

#include "PNEEffect.h"
#include "PDDLUtils.h"
#include "ExpressionTree.h"

namespace PDDL {

std::ostream & operator<<(std::ostream & output,
		const PDDL::PNEEffect & pneEffect) {
	//Effect on fluent
	output << "(" << getAssignmentString(pneEffect.op);
	//Fluent Name
	output << pneEffect.effectVar;
	//Closing parenthesis
	output << ")";
	return output;
}

std::string PNEEffect::toParameterisedString(
		map<PDDLObject, string> parameterTable) const {
	ostringstream output;
	//Effect on fluent
	output << "(" << getAssignmentString(op) << " ";
	//Fluent Name
	output << effectVar.toActionEffectString(parameterTable) << " ";
	//Convert equation for RPN (postfix) to PN (prefix)
	ExpressionTree * expTree = ExpressionTree::postfix2expTree(equation,
			parameterTable);
	output << ExpressionTree::preOrder(expTree->getExpRoot());
	//Closing parenthesis
	output << ")";
	return output.str();
}

}

